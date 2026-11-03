//! Thrusters over bidirectional DShot300, from `dshot.c` and `bidir_dshot.pio`.
//!
//! The backplane drives eight BLHeli ESCs (patched for 3D mode, see `puddles_esc_firmware/` in the C tree) on one
//! PIO state machine each. A frame goes out to every ESC every [`TX_PERIOD_US`]; after each frame the ESC answers
//! with its eRPM period. Commands from `command/dshot` restart the frames immediately.
//!
//! [`Thrusters`] is the command state of `dshot.c` with the hardware taken out: which values to send, the 200 ms
//! command timeout, the "thrusters allowed" holdoff after a timeout or after the physical kill switch is
//! inserted, and the spacing between frames. [`Telemetry`] tracks each ESC's eRPM replies, and [`rpm_feedback`]
//! turns them into `state/thrusters/rpm` (`ros_send_rpm`).

use titan_link::puddles_backplane::RpmFeedback;

/// Thrusters on the backplane.
pub const NUM_THRUSTERS: usize = titan_link::puddles_backplane::NUM_THRUSTERS;
/// Time between frames (`DSHOT_TX_RATE_US`).
pub const TX_PERIOD_US: u64 = 1500;
/// Commands must arrive at least this often or the thrusters stop (`DSHOT_MIN_UPDATE_RATE_MS`).
pub const MIN_UPDATE_RATE_US: u64 = 200_000;
/// After a command timeout with the thrusters running, commands are ignored this long
/// (`DSHOT_UPDATE_DISABLE_TIME_MS`).
pub const UPDATE_DISABLE_TIME_US: u64 = 1_000_000;
/// After the physical kill switch is inserted, commands are ignored this long while the ESCs power up and chime
/// (`DSHOT_WAKEUP_DELAY_MS`).
pub const WAKEUP_DELAY_US: u64 = 5_000_000;
/// DShot bit rate in kbit/s (`DSHOT_RATE`).
pub const RATE_KHZ: u32 = 300;
/// Frames without a valid reply before a thruster's telemetry counts as invalid (`TELEM_MAX_MISSED_PACKETS`).
pub const TELEM_MAX_MISSED_PACKETS: u8 = 3;
/// T200 thrusters have 7 pole pairs (`num_pole_pairs`).
pub const NUM_POLE_PAIRS: u32 = 7;
/// Throttle range (`DshotCommand.DSHOT_MIN` and `DSHOT_MAX`).
pub const THROTTLE_MIN: i16 = -999;
pub const THROTTLE_MAX: i16 = 999;
/// The eRPM period the ESC reports when the motor is stopped.
pub const PERIOD_STOPPED_US: u16 = 0x1FF << 7;

/// Shortest time between the start of two frames: the 16-bit frame, the 21-bit reply at 5/4 the rate (counted at
/// the full rate, as C does) and 30 us of line turnaround each way (`bidir_dshot_min_frame_period_us`).
pub const fn min_frame_period_us(rate_khz: u32) -> u64 {
    (1000 * (16 + 21)) / rate_khz as u64 + 30 * 2
}

const _: () = assert!(min_frame_period_us(RATE_KHZ) <= TX_PERIOD_US, "frames must fit in the transmit period");

/// PIO clock cycles per DShot bit in `bidir_dshot`.
pub const PIO_CYCLES_PER_BIT: u32 = 40;

/// The value to push into the `delayed_start` entry of `bidir_dshot` to hold off the first frame for `delay_us`:
/// each loop of the delay takes one bit time (`bidir_dshot_reset`).
pub fn delayed_start_count(delay_us: u64) -> u32 {
    (delay_us * RATE_KHZ as u64 / 1000) as u32
}

/// Builds a 16-bit bidirectional DShot frame: 11-bit throttle, telemetry request bit and the inverted checksum.
pub fn encode_frame(throttle: u16, request_telemetry: bool) -> u16 {
    let cmd = ((throttle & 0x7FF) << 1) | request_telemetry as u16;
    let crc = !(cmd ^ (cmd >> 4) ^ (cmd >> 8)) & 0x0F;
    (cmd << 4) | crc
}

/// The word `bidir_dshot` takes: the frame inverted (the line idles high and a bit is asserted by pulling it low)
/// and left-justified, as the program shifts 16 bits out MSB first.
pub fn pio_word(frame: u16) -> u32 {
    ((!frame) as u32) << 16
}

/// GCR symbol to nibble; `-1` marks invalid symbols.
const GCR_LOOKUP: [i8; 32] = [
    -1, -1, -1, -1, // 0x00-0x03
    -1, -1, -1, -1, // 0x04-0x07
    -1, 0x9, 0xA, 0xB, // 0x08-0x0B
    -1, 0xD, 0xE, 0xF, // 0x0C-0x0F
    -1, -1, 0x2, 0x3, // 0x10-0x13
    -1, 0x5, 0x6, 0x7, // 0x14-0x17
    -1, 0x0, 0x8, 0x1, // 0x18-0x1B
    -1, 0x4, 0xC, -1, // 0x1C-0x1F
];

/// Decodes the 21-bit eRPM reply `bidir_dshot` pushes into the eRPM period in microseconds
/// (`decode_erpm_period`). Returns `None` on an empty reply, an invalid GCR symbol or a bad checksum.
pub fn decode_erpm_period(packet: u32) -> Option<u16> {
    if packet == 0 {
        return None;
    }
    // A 1 in the GCR stream is a level change on the line
    let gcr = packet ^ (packet >> 1);
    let nibble = |shift: u32| {
        let value = GCR_LOOKUP[((gcr >> shift) & 0x1F) as usize];
        (value >= 0).then_some(value as u32)
    };
    let crc = nibble(0)?;
    let low = nibble(5)?;
    let mid = nibble(10)?;
    let high = nibble(15)?;
    if !(low ^ mid ^ high) & 0xF != crc {
        return None;
    }
    let data = (high << 8) | (mid << 4) | low;
    // 9-bit mantissa shifted by a 3-bit exponent: at most 0x1FF << 7, which fits a u16
    Some(((data & 0x1FF) << (data >> 9)) as u16)
}

/// The DShot value for one throttle in -999..=999, and the direction it spins in (`None` when stopped, leaving
/// the recorded direction as it was). 3D mode: 48-1047 forward, 1049-2047 reverse, 0 stop.
pub fn throttle_value(throttle: i16) -> Result<(u16, Option<bool>), i16> {
    match throttle {
        0 => Ok((0, None)),
        1..=THROTTLE_MAX => Ok((throttle as u16 + 48, Some(false))),
        THROTTLE_MIN..=-1 => Ok(((-throttle) as u16 + 1048, Some(true))),
        _ => Err(throttle),
    }
}

/// Why a command was ignored.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Ignored {
    /// The vehicle is killed.
    Killed,
    /// Within the holdoff after a timeout or after the kill switch was inserted.
    NotAllowedYet,
    /// The previous frame may still be on the wire.
    FrameInProgress,
}

/// What [`Thrusters::update`] did with a command.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Update {
    /// Nothing changed; keep the current frame schedule.
    Ignored(Ignored),
    /// New values are set. Restart the state machines and send a frame now.
    Applied,
    /// A throttle was out of range. The thrusters are stopped and the board raises `FAULT_ROS_BAD_COMMAND` with the
    /// value; like `Applied`, the (now neutral) frames go out immediately.
    Invalid(i16),
}

/// What to send in one round of frames.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Frames {
    /// DShot throttle values, one per thruster.
    pub values: [u16; NUM_THRUSTERS],
    /// The last command was more than [`MIN_UPDATE_RATE_US`] ago while the thrusters ran: the board raises
    /// `FAULT_THRUSTER_TIMEOUT`.
    pub timed_out: bool,
}

/// The thruster command state of `dshot.c`. All times are microseconds since boot.
#[derive(Clone, Debug)]
pub struct Thrusters {
    values: [u16; NUM_THRUSTERS],
    reversed: [bool; NUM_THRUSTERS],
    /// Some thruster is commanded to move (`dshot_thrusters_on`).
    on: bool,
    /// When the last command expires; `None` is the C nil time, which counts as expired.
    command_timeout: Option<u64>,
    /// Commands are ignored before this (`dshot_time_thrusters_allowed`).
    allowed_at: u64,
    /// Earliest start of the next frame (`dshot_next_allowed_frame_tx`).
    next_frame_at: u64,
    /// Previous physical kill switch state passed to [`Thrusters::notify_physical_kill_switch`].
    physical_killed: bool,
}

impl Default for Thrusters {
    fn default() -> Self {
        Self::new()
    }
}

impl Thrusters {
    pub const fn new() -> Self {
        Thrusters {
            values: [0; NUM_THRUSTERS],
            reversed: [false; NUM_THRUSTERS],
            on: false,
            command_timeout: None,
            allowed_at: 0,
            next_frame_at: 0,
            physical_killed: true,
        }
    }

    /// Stops all thrusters at the next frame (`dshot_stop_thrusters`). Safe to call from the kill hook.
    pub fn stop(&mut self) {
        self.values = [0; NUM_THRUSTERS];
        self.on = false;
        self.command_timeout = None;
    }

    /// Holds off commands until at least `at` (`dshot_set_time_thrusters_allowed`); never shortens the holdoff.
    fn allow_from(&mut self, at: u64) {
        self.allowed_at = self.allowed_at.max(at);
    }

    /// Reports the physical kill switch, which C does on every safety tick and every edge
    /// (`dshot_notify_physical_kill_switch_change`). While the switch is out, and once more on the call that
    /// sees it inserted, the holdoff is pushed to [`WAKEUP_DELAY_US`] from now, so the thrusters stay off for
    /// 5 s after the ESCs are powered.
    pub fn notify_physical_kill_switch(&mut self, now: u64, asserting_kill: bool) {
        if self.physical_killed {
            self.allow_from(now + WAKEUP_DELAY_US);
        }
        self.physical_killed = asserting_kill;
    }

    /// Applies a command from `command/dshot` (`dshot_update_thrusters`).
    pub fn update(&mut self, now: u64, killed: bool, throttles: &[i16; NUM_THRUSTERS]) -> Update {
        if killed {
            return Update::Ignored(Ignored::Killed);
        }
        if now < self.allowed_at {
            return Update::Ignored(Ignored::NotAllowedYet);
        }
        if now < self.next_frame_at {
            return Update::Ignored(Ignored::FrameInProgress);
        }

        let mut result = Update::Applied;
        self.on = false;
        // In order, like C: entries before an invalid one have already updated their direction
        for (i, &throttle) in throttles.iter().enumerate() {
            match throttle_value(throttle) {
                Ok((value, direction)) => {
                    self.values[i] = value;
                    if let Some(reversed) = direction {
                        self.reversed[i] = reversed;
                        self.on = true;
                    }
                }
                Err(invalid) => {
                    self.stop();
                    result = Update::Invalid(invalid);
                    break;
                }
            }
        }
        self.command_timeout = Some(now + MIN_UPDATE_RATE_US);
        result
    }

    /// Prepares a round of frames (`dshot_transmit_timer_cb`): applies the command timeout, the kill state and the
    /// holdoff, and books the time the frames occupy the line.
    pub fn transmit(&mut self, now: u64, killed: bool) -> Frames {
        let expired = self.command_timeout.is_none_or(|t| now >= t);
        let timed_out = expired && self.on;
        if timed_out {
            self.allow_from(now + UPDATE_DISABLE_TIME_US);
        }
        if expired || killed || now < self.allowed_at {
            self.stop();
        }
        self.next_frame_at = now + min_frame_period_us(RATE_KHZ);
        Frames { values: self.values, timed_out }
    }

    /// Earliest start of the next frame, for the delayed start after a restart.
    pub fn next_frame_at(&self) -> u64 {
        self.next_frame_at
    }

    /// Direction each thruster was last commanded in (`dshot_rpm_reversed`).
    pub fn reversed(&self) -> [bool; NUM_THRUSTERS] {
        self.reversed
    }
}

/// eRPM telemetry of one ESC (`struct dshot_rpm_telemetry`).
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
pub struct Telemetry {
    missed: u8,
    period_us: u16,
    valid: bool,
}

impl Telemetry {
    pub const fn new() -> Self {
        Telemetry { missed: 0, period_us: 0, valid: false }
    }

    /// Counts a frame sent to the ESC. After [`TELEM_MAX_MISSED_PACKETS`] frames without a valid reply the
    /// telemetry turns invalid.
    pub fn on_transmit(&mut self) {
        if self.missed < TELEM_MAX_MISSED_PACKETS {
            self.missed += 1;
        } else {
            self.valid = false;
        }
    }

    /// Handles a reply from the ESC. Undecodable replies are ignored.
    pub fn on_reply(&mut self, packet: u32) {
        if let Some(period_us) = decode_erpm_period(packet) {
            self.period_us = period_us;
            self.valid = true;
            self.missed = 0;
        }
    }

    /// The signed mechanical RPM, or `None` if the telemetry isn't valid (`ros_send_rpm`). Speeds that don't fit an
    /// `i16`, and a zero period, report `i16::MAX`.
    pub fn rpm(&self, reversed: bool) -> Option<i16> {
        if !self.valid {
            return None;
        }
        let rpm = match self.period_us {
            PERIOD_STOPPED_US => 0,
            0 => i16::MAX,
            period => {
                let erpm = 60_000_000 / period as u32;
                let rpm = erpm / NUM_POLE_PAIRS;
                if rpm < i16::MAX as u32 { rpm as i16 } else { i16::MAX }
            }
        };
        Some(if reversed { -rpm } else { rpm })
    }
}

/// `state/thrusters/rpm` from the telemetry of every ESC and the direction each was last commanded in.
pub fn rpm_feedback(telemetry: &[Telemetry; NUM_THRUSTERS], reversed: &[bool; NUM_THRUSTERS]) -> RpmFeedback {
    let mut feedback = RpmFeedback::default();
    for (i, (t, &rev)) in telemetry.iter().zip(reversed).enumerate() {
        if let Some(rpm) = t.rpm(rev) {
            feedback.valid_mask |= 1 << i;
            feedback.rpm[i] = rpm;
        }
    }
    feedback
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: u64 = 1000;

    #[test]
    fn throttle_mapping() {
        assert_eq!(throttle_value(0), Ok((0, None)));
        assert_eq!(throttle_value(1), Ok((49, Some(false))));
        assert_eq!(throttle_value(999), Ok((1047, Some(false))));
        assert_eq!(throttle_value(-1), Ok((1049, Some(true))));
        assert_eq!(throttle_value(-999), Ok((2047, Some(true))));
        for bad in [1000, -1000, i16::MAX, i16::MIN] {
            assert_eq!(throttle_value(bad), Err(bad));
        }
    }

    /// The checksum written from the protocol description: XOR of the three nibbles of the 12-bit packet, inverted
    /// for bidirectional DShot.
    fn reference_frame(throttle: u16, telemetry: bool) -> u16 {
        let packet = (throttle << 1) | telemetry as u16;
        let csum = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0xF;
        (packet << 4) | (!csum & 0xF)
    }

    #[test]
    fn frame_encoding() {
        for throttle in 0..2048 {
            for telemetry in [false, true] {
                assert_eq!(encode_frame(throttle, telemetry), reference_frame(throttle, telemetry));
            }
        }
        assert_eq!(encode_frame(0, false), 0x000F);
        assert_eq!(encode_frame(1046, false), 0x82C9);
        assert_eq!(pio_word(0x000F), 0xFFF0_0000);
        assert_eq!(pio_word(0x82C9), 0x7D36_0000);
    }

    /// GCR encoding of a 12-bit telemetry value as the ESC puts it on the line, per the bidirectional DShot spec.
    fn encode_reply(value: u32) -> u32 {
        const GCR: [u32; 16] =
            [0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17, 0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F];
        let crc = !(value ^ (value >> 4) ^ (value >> 8)) & 0xF;
        let word = (value << 4) | crc;
        let gcr = (0..4).rev().fold(0, |acc, i| (acc << 5) | GCR[((word >> (i * 4)) & 0xF) as usize]);
        // Each 1 toggles the line level, starting from the low start bit
        let mut level = 0;
        (0..20).rev().fold(0, |acc, bit| {
            level ^= (gcr >> bit) & 1;
            acc | (level << bit)
        })
    }

    #[test]
    fn erpm_decoding() {
        for value in 0..4096u32 {
            let expected = ((value & 0x1FF) << (value >> 9)) as u16;
            assert_eq!(decode_erpm_period(encode_reply(value)), Some(expected), "{value:#x}");
        }
        assert_eq!(decode_erpm_period(0), None);
        let good = encode_reply(0x2A5);
        let rejected = (0..20).filter(|bit| decode_erpm_period(good ^ (1 << bit)).is_none()).count();
        // A flipped bit changes two GCR transitions; the symbols or the checksum catch nearly all of them
        assert!(rejected >= 18, "only {rejected} of 20 single bit errors caught");
    }

    #[test]
    fn rpm_conversion() {
        let reply = |period: u32| {
            let mut t = Telemetry::new();
            // Exponent 0 holds periods up to 511 us; larger ones need a shift
            let mut exp = 0;
            while (period >> exp) > 0x1FF {
                exp += 1;
            }
            t.on_reply(encode_reply((exp << 9) | (period >> exp)));
            t
        };
        // 1000 us eRPM period: 60000 eRPM, 8571 RPM with 7 pole pairs
        assert_eq!(reply(1000).rpm(false), Some(8571));
        assert_eq!(reply(1000).rpm(true), Some(-8571));
        assert_eq!(reply(PERIOD_STOPPED_US as u32).rpm(true), Some(0));
        // Faster than i16 RPM can hold (or a zero period) saturates, sign still applied
        assert_eq!(reply(200).rpm(false), Some(i16::MAX));
        assert_eq!(reply(0).rpm(true), Some(-i16::MAX));
        assert_eq!(Telemetry::new().rpm(false), None);
    }

    #[test]
    fn telemetry_goes_invalid_after_missed_replies() {
        let mut t = Telemetry::new();
        t.on_reply(encode_reply(100));
        for _ in 0..TELEM_MAX_MISSED_PACKETS {
            t.on_transmit();
            assert!(t.rpm(false).is_some());
        }
        t.on_transmit();
        assert_eq!(t.rpm(false), None);
        // Garbage doesn't revive it, a good reply does
        t.on_reply(0x1F_FFFF);
        assert_eq!(t.rpm(false), None);
        t.on_reply(encode_reply(100));
        assert!(t.rpm(false).is_some());
    }

    #[test]
    fn feedback_mask_and_values() {
        let mut telemetry = [Telemetry::new(); NUM_THRUSTERS];
        telemetry[0].on_reply(encode_reply(PERIOD_STOPPED_US as u32 >> 7 | 7 << 9));
        telemetry[5].on_reply(encode_reply(500));
        let mut reversed = [false; NUM_THRUSTERS];
        reversed[5] = true;
        let feedback = rpm_feedback(&telemetry, &reversed);
        assert_eq!(feedback.valid_mask, 0b0010_0001);
        assert_eq!(feedback.rpm, [0, 0, 0, 0, 0, -17142, 0, 0]);
    }

    /// A board that powered up with the kill switch inserted: no wake-up holdoff left.
    fn enabled_thrusters(now: u64) -> Thrusters {
        let mut t = Thrusters::new();
        t.notify_physical_kill_switch(0, false);
        assert!(now >= WAKEUP_DELAY_US);
        t
    }

    #[test]
    fn command_runs_until_timeout() {
        let start = 10_000 * MS;
        let mut t = enabled_thrusters(start);
        let cmd = [100, -100, 0, 0, 0, 0, 0, 999];
        assert_eq!(t.update(start, false, &cmd), Update::Applied);
        let frames = t.transmit(start, false);
        assert_eq!(frames.values, [148, 1148, 0, 0, 0, 0, 0, 1047]);
        assert!(!frames.timed_out);
        assert_eq!(t.reversed()[..2], [false, true]);

        // Keeps sending until 200 ms after the command
        let frames = t.transmit(start + 199 * MS, false);
        assert_eq!(frames.values[0], 148);
        let frames = t.transmit(start + 200 * MS, false);
        assert!(frames.timed_out);
        assert_eq!(frames.values, [0; NUM_THRUSTERS]);
        // Reported once, then commands are ignored for a second
        assert!(!t.transmit(start + 202 * MS, false).timed_out);
        assert_eq!(t.update(start + 1199 * MS, false, &cmd), Update::Ignored(Ignored::NotAllowedYet));
        assert_eq!(t.update(start + 1200 * MS, false, &cmd), Update::Applied);
    }

    #[test]
    fn neutral_commands_do_not_time_out() {
        let start = 10_000 * MS;
        let mut t = enabled_thrusters(start);
        assert_eq!(t.update(start, false, &[0; NUM_THRUSTERS]), Update::Applied);
        assert!(!t.transmit(start + 500 * MS, false).timed_out);
        assert_eq!(t.update(start + 501 * MS, false, &[1; NUM_THRUSTERS]), Update::Applied);
    }

    #[test]
    fn kill_and_stop() {
        let start = 10_000 * MS;
        let mut t = enabled_thrusters(start);
        assert_eq!(t.update(start, true, &[5; NUM_THRUSTERS]), Update::Ignored(Ignored::Killed));
        assert_eq!(t.update(start, false, &[5; NUM_THRUSTERS]), Update::Applied);
        // Killed while running: zeroed at the next frame, and no timeout fault for it
        assert_eq!(t.transmit(start + MS, true).values, [0; NUM_THRUSTERS]);
        assert!(!t.transmit(start + 300 * MS, false).timed_out);

        assert_eq!(t.update(start + 400 * MS, false, &[5; NUM_THRUSTERS]), Update::Applied);
        t.stop();
        let frames = t.transmit(start + 401 * MS, false);
        assert_eq!(frames.values, [0; NUM_THRUSTERS]);
        assert!(!t.transmit(start + 700 * MS, false).timed_out);
    }

    #[test]
    fn frames_are_spaced() {
        let start = 10_000 * MS;
        let mut t = enabled_thrusters(start);
        t.transmit(start, false);
        let gap = min_frame_period_us(RATE_KHZ);
        assert_eq!(gap, 183);
        assert_eq!(t.next_frame_at(), start + gap);
        assert_eq!(t.update(start + gap - 1, false, &[1; 8]), Update::Ignored(Ignored::FrameInProgress));
        assert_eq!(t.update(start + gap, false, &[1; 8]), Update::Applied);
    }

    #[test]
    fn invalid_throttle_stops_everything() {
        let start = 10_000 * MS;
        let mut t = enabled_thrusters(start);
        assert_eq!(t.update(start, false, &[-5, 1000, 7, 0, 0, 0, 0, 0]), Update::Invalid(1000));
        let frames = t.transmit(start, false);
        assert_eq!(frames.values, [0; NUM_THRUSTERS]);
        // Thruster 1 had already been marked reversed, thruster 3 never got its value
        assert!(t.reversed()[0]);
        // The (neutral) command still counts, so no timeout fault follows
        assert!(!t.transmit(start + 300 * MS, false).timed_out);
    }

    #[test]
    fn physical_kill_switch_wakeup_delay() {
        let mut t = Thrusters::new();
        // Switch out from boot until 3 s, then inserted
        for ms in (0..3000).step_by(10) {
            t.notify_physical_kill_switch(ms * MS, true);
        }
        t.notify_physical_kill_switch(3000 * MS, false);
        t.notify_physical_kill_switch(3010 * MS, false);
        let cmd = [1; NUM_THRUSTERS];
        assert_eq!(t.update(7999 * MS, false, &cmd), Update::Ignored(Ignored::NotAllowedYet));
        assert_eq!(t.update(8000 * MS, false, &cmd), Update::Applied);
        // A timeout holdoff never shortens the wake-up holdoff
        t.allow_from(100);
        assert_eq!(t.allowed_at, 8000 * MS);
    }

    #[test]
    fn delayed_start() {
        // One loop of the delay is one bit: 3.33 us at DShot300
        assert_eq!(delayed_start_count(0), 0);
        assert_eq!(delayed_start_count(100), 30);
        assert_eq!(delayed_start_count(183), 54);
    }
}
