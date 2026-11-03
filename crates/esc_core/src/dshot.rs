//! DShot throttle mapping and frame encoding, from `dshot.c`.
//!
//! A DShot frame is 16 bits, MSB first: an 11-bit throttle value, the telemetry request bit, and a 4-bit checksum.
//! Bidirectional DShot inverts the checksum. With the ESCs patched for 3D mode, throttle 48-1047 is forward and
//! 1049-2047 is reverse; 0 is disarmed/neutral.

use crate::{NUM_THRUSTERS, TELEM_PACKET_DELAY};

/// Largest throttle magnitude accepted by [`throttle_command`].
pub const THROTTLE_MAX: i16 = 999;

/// A throttle value mapped to DShot.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ThrottleCommand {
    /// The 11-bit DShot throttle value.
    pub value: u16,
    /// `Some(true)` when spinning in reverse, `Some(false)` forward, `None` when not spinning (direction unchanged).
    pub reversed: Option<bool>,
}

/// Maps a throttle in -999..=999 to a DShot throttle value. Anything else (including 0) maps to 0.
pub fn throttle_command(throttle: i16) -> ThrottleCommand {
    if (1..=THROTTLE_MAX).contains(&throttle) {
        ThrottleCommand { value: throttle as u16 + 48, reversed: Some(false) }
    } else if (-THROTTLE_MAX..0).contains(&throttle) {
        ThrottleCommand { value: (-throttle) as u16 + 1048, reversed: Some(true) }
    } else {
        ThrottleCommand { value: 0, reversed: None }
    }
}

/// Builds the 16-bit bidirectional DShot frame (inverted checksum).
pub fn encode_frame(throttle: u16, request_telemetry: bool) -> u16 {
    let cmd = ((throttle & 0x7FF) << 1) | request_telemetry as u16;
    let crc = !(cmd ^ (cmd >> 4) ^ (cmd >> 8)) & 0x0F;
    (cmd << 4) | crc
}

/// The word pushed into the `bidir_dshot` PIO program: the frame inverted (the line idles high and a bit is
/// "asserted" by driving it low) and left-justified, as the program shifts out 16 bits MSB first.
pub fn pio_word(frame: u16) -> u32 {
    ((!frame) as u32) << 16
}

/// Decides which ESC gets the telemetry request bit, rotating through the thrusters every
/// [`TELEM_PACKET_DELAY`] frames (the static counters in `dshot_update_thrusters`).
#[derive(Clone, Default, Debug)]
pub struct TelemetrySchedule {
    thruster: usize,
    delay: u32,
}

impl TelemetrySchedule {
    pub const fn new() -> Self {
        TelemetrySchedule { thruster: 0, delay: 0 }
    }

    /// Advances by one frame. Returns the thruster whose frame should request telemetry, if any.
    pub fn next_frame(&mut self) -> Option<usize> {
        self.delay += 1;
        if self.delay == TELEM_PACKET_DELAY {
            self.delay = 0;
            self.thruster = (self.thruster + 1) % NUM_THRUSTERS;
            Some(self.thruster)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn throttle_mapping() {
        assert_eq!(throttle_command(0), ThrottleCommand { value: 0, reversed: None });
        assert_eq!(throttle_command(1), ThrottleCommand { value: 49, reversed: Some(false) });
        assert_eq!(throttle_command(999), ThrottleCommand { value: 1047, reversed: Some(false) });
        assert_eq!(throttle_command(-1), ThrottleCommand { value: 1049, reversed: Some(true) });
        assert_eq!(throttle_command(-999), ThrottleCommand { value: 2047, reversed: Some(true) });
        // Out of range values are treated as neutral
        assert_eq!(throttle_command(1000), ThrottleCommand { value: 0, reversed: None });
        assert_eq!(throttle_command(-1000), ThrottleCommand { value: 0, reversed: None });
        assert_eq!(throttle_command(i16::MIN), ThrottleCommand { value: 0, reversed: None });
    }

    /// Reference implementation of the bidirectional DShot checksum, written from the protocol description.
    fn reference_frame(throttle: u16, telemetry: bool) -> u16 {
        let packet = (throttle << 1) | telemetry as u16;
        let mut csum = 0;
        let mut data = packet;
        for _ in 0..3 {
            csum ^= data;
            data >>= 4;
        }
        let csum = !csum & 0xF;
        (packet << 4) | csum
    }

    #[test]
    fn frame_encoding() {
        for throttle in 0..2048 {
            for telemetry in [false, true] {
                assert_eq!(encode_frame(throttle, telemetry), reference_frame(throttle, telemetry));
            }
        }
        // Known values: throttle 0 without telemetry has checksum 0xF in bidirectional mode
        assert_eq!(encode_frame(0, false), 0x000F);
        // 1046 << 1 = 0x82C; 0x82C ^ 0x82 ^ 0x8 = 0x8A6, inverted low nibble 0x9
        assert_eq!(encode_frame(1046, false), 0x82C9);
        // Throttle values wider than 11 bits are masked
        assert_eq!(encode_frame(0x800 | 48, true), encode_frame(48, true));
    }

    #[test]
    fn pio_word_is_inverted_and_left_justified() {
        assert_eq!(pio_word(0x000F), 0xFFF0_0000);
        assert_eq!(pio_word(0xFFFF), 0);
        assert_eq!(pio_word(0x1234) >> 16, !0x1234u16 as u32);
        assert_eq!(pio_word(0x1234) & 0xFFFF, 0);
    }

    #[test]
    fn telemetry_rotates_every_hundred_frames() {
        let mut schedule = TelemetrySchedule::new();
        let mut requests = [0usize; NUM_THRUSTERS];
        let mut order = [usize::MAX; 8];
        let mut n = 0;
        for frame in 1..=800u32 {
            if let Some(t) = schedule.next_frame() {
                assert_eq!(frame % TELEM_PACKET_DELAY, 0);
                requests[t] += 1;
                order[n] = t;
                n += 1;
            }
        }
        assert_eq!(requests, [2; NUM_THRUSTERS]);
        // Like the C code, the first request goes to thruster 1
        assert_eq!(order, [1, 2, 3, 0, 1, 2, 3, 0]);
    }
}
