//! ESC UART telemetry, from `dshot.c` and `uart_telemetry.h`.
//!
//! When a DShot frame has its telemetry bit set, the ESC answers on its telemetry wire with a 10-byte KISS/BLHeli
//! frame at 115200 baud. Only one request is outstanding at a time; [`UartTelemetry`] tracks it and keeps the latest
//! data and statistics for each ESC, as `dshot_uart_prep_for_rx`, `dshot_uart_telem_cb` and `dshot_get_telemetry`
//! did with their shared state.

use crate::{ESC_NUM_POLE_PAIRS, NUM_THRUSTERS, TELEM_MAX_MISSED_UART_PACKETS};

/// Length of a telemetry frame including the CRC byte.
pub const FRAME_LEN: usize = 10;

/// One step of the KISS telemetry CRC-8 (polynomial 0x07).
pub fn crc8_update(crc: u8, byte: u8) -> u8 {
    let mut crc = crc ^ byte;
    for _ in 0..8 {
        crc = if crc & 0x80 != 0 { 0x07 ^ (crc << 1) } else { crc << 1 };
    }
    crc
}

/// CRC-8 over `bytes`.
pub fn crc8(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0, |crc, &b| crc8_update(crc, b))
}

/// Latest telemetry and statistics of one ESC (`struct dshot_uart_telemetry`).
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EscTelemetry {
    /// Requests missed in a row (reset on every good frame).
    pub missed_count: u16,
    /// Requests missed without invalidating the telemetry, since the last [`UartTelemetry::take`].
    pub total_missed: u16,
    /// Requests missed while the telemetry was already invalid, since the last take.
    pub total_invalid: u16,
    /// Good frames received since the last take.
    pub total_success: u16,
    /// The data fields below are current.
    pub valid: bool,
    /// The thruster was commanded in reverse when this frame was requested (the ESC reports absolute RPM).
    pub rpm_reversed: bool,
    /// Temperature in °C.
    pub temperature: u8,
    /// Voltage in hundredths of a volt.
    pub voltage: u16,
    /// Current in hundredths of an amp.
    pub current: u16,
    /// Consumption in mAh.
    pub consumption: u16,
    /// Electrical RPM in hundreds.
    pub rpm: u16,
}

impl EscTelemetry {
    /// Mechanical RPM with the guessed sign, saturated to `i16` (the C code let it wrap).
    pub fn mechanical_rpm(&self) -> i16 {
        let rpm = self.rpm as i32 * 100 / ESC_NUM_POLE_PAIRS;
        let rpm = if self.rpm_reversed { -rpm } else { rpm };
        rpm.clamp(i16::MIN as i32, i16::MAX as i32) as i16
    }
}

/// Bookkeeping for the ESC telemetry of one board.
#[derive(Clone, Debug)]
pub struct UartTelemetry {
    escs: [EscTelemetry; NUM_THRUSTERS],
    buffer: [u8; FRAME_LEN],
    recv_index: usize,
    crc: u8,
    /// The ESC the outstanding request went to.
    pending: Option<usize>,
    rpm_reversed: bool,
}

impl Default for UartTelemetry {
    fn default() -> Self {
        Self::new()
    }
}

impl UartTelemetry {
    pub const fn new() -> Self {
        const ZERO: EscTelemetry = EscTelemetry {
            missed_count: 0,
            total_missed: 0,
            total_invalid: 0,
            total_success: 0,
            valid: false,
            rpm_reversed: false,
            temperature: 0,
            voltage: 0,
            current: 0,
            consumption: 0,
            rpm: 0,
        };
        UartTelemetry {
            escs: [ZERO; NUM_THRUSTERS],
            buffer: [0; FRAME_LEN],
            recv_index: 0,
            crc: 0,
            pending: None,
            rpm_reversed: false,
        }
    }

    /// Call right before sending a frame that requests telemetry from `thruster`. An unanswered previous request
    /// counts as missed; after [`TELEM_MAX_MISSED_UART_PACKETS`] misses in a row the ESC's telemetry is invalid.
    pub fn prepare_for_rx(&mut self, thruster: usize, rpm_reversed: bool) {
        if let Some(last) = self.pending {
            let esc = &mut self.escs[last];
            if esc.missed_count < TELEM_MAX_MISSED_UART_PACKETS {
                esc.missed_count += 1;
                esc.total_missed = esc.total_missed.saturating_add(1);
            } else {
                esc.valid = false;
                esc.total_invalid = esc.total_invalid.saturating_add(1);
            }
        }
        self.crc = 0;
        self.recv_index = 0;
        self.pending = Some(thruster);
        self.rpm_reversed = rpm_reversed;
    }

    /// Handles a byte received on the telemetry wire of `thruster`. Bytes from ESCs that weren't asked are dropped.
    pub fn receive(&mut self, thruster: usize, byte: u8) {
        if self.pending != Some(thruster) || self.recv_index >= FRAME_LEN {
            return;
        }
        self.buffer[self.recv_index] = byte;
        self.recv_index += 1;
        let last_crc = self.crc;
        self.crc = crc8_update(last_crc, byte);
        if self.recv_index < FRAME_LEN {
            return;
        }
        // The CRC over the first nine bytes must equal the tenth. On a mismatch the request stays pending, and
        // counts as missed when the next one goes out.
        let b = &self.buffer;
        if last_crc == b[9] {
            let esc = &mut self.escs[thruster];
            esc.temperature = b[0];
            esc.voltage = u16::from_be_bytes([b[1], b[2]]);
            esc.current = u16::from_be_bytes([b[3], b[4]]);
            esc.consumption = u16::from_be_bytes([b[5], b[6]]);
            esc.rpm = u16::from_be_bytes([b[7], b[8]]);
            esc.rpm_reversed = self.rpm_reversed;
            esc.missed_count = 0;
            esc.valid = true;
            esc.total_success = esc.total_success.saturating_add(1);
            self.pending = None;
        }
    }

    /// Returns the telemetry of `thruster` and clears its statistics, like `dshot_get_telemetry`. The data fields
    /// are only filled in while the telemetry is valid; otherwise only the statistics are.
    pub fn take(&mut self, thruster: usize) -> EscTelemetry {
        let esc = &mut self.escs[thruster];
        let out = if esc.valid {
            *esc
        } else {
            EscTelemetry {
                total_missed: esc.total_missed,
                total_invalid: esc.total_invalid,
                total_success: esc.total_success,
                ..EscTelemetry::default()
            }
        };
        esc.total_missed = 0;
        esc.total_invalid = 0;
        esc.total_success = 0;
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(temperature: u8, voltage: u16, current: u16, consumption: u16, rpm: u16) -> [u8; FRAME_LEN] {
        let mut f = [0u8; FRAME_LEN];
        f[0] = temperature;
        f[1..3].copy_from_slice(&voltage.to_be_bytes());
        f[3..5].copy_from_slice(&current.to_be_bytes());
        f[5..7].copy_from_slice(&consumption.to_be_bytes());
        f[7..9].copy_from_slice(&rpm.to_be_bytes());
        f[9] = crc8(&f[..9]);
        f
    }

    fn feed(t: &mut UartTelemetry, thruster: usize, bytes: &[u8]) {
        for &b in bytes {
            t.receive(thruster, b);
        }
    }

    #[test]
    fn crc_matches_reference() {
        // CRC-8/SMBUS (poly 0x07, init 0) check value
        assert_eq!(crc8(b"123456789"), 0xF4);
        assert_eq!(crc8(&[]), 0);
    }

    #[test]
    fn good_frame_is_decoded() {
        let mut t = UartTelemetry::new();
        t.prepare_for_rx(2, true);
        feed(&mut t, 2, &frame(41, 1612, 250, 1234, 35));
        let esc = t.take(2);
        assert!(esc.valid);
        assert_eq!((esc.temperature, esc.voltage, esc.current, esc.consumption, esc.rpm), (41, 1612, 250, 1234, 35));
        assert_eq!(esc.total_success, 1);
        // 3500 eRPM / 7 pole pairs, reversed
        assert_eq!(esc.mechanical_rpm(), -500);
        // Statistics are cleared by take, data stays
        let again = t.take(2);
        assert_eq!(again.total_success, 0);
        assert_eq!(again.voltage, 1612);
    }

    #[test]
    fn bytes_from_other_escs_and_bad_crc_are_ignored() {
        let mut t = UartTelemetry::new();
        t.prepare_for_rx(0, false);
        feed(&mut t, 1, &frame(1, 2, 3, 4, 5));
        let mut bad = frame(1, 2, 3, 4, 5);
        bad[9] ^= 0xFF;
        feed(&mut t, 0, &bad);
        assert!(!t.take(0).valid);
        assert!(!t.take(1).valid);
        // Extra bytes after a full (bad) frame are dropped until the next request
        feed(&mut t, 0, &frame(1, 2, 3, 4, 5));
        assert!(!t.take(0).valid);
    }

    #[test]
    fn misses_invalidate_after_three() {
        let mut t = UartTelemetry::new();
        t.prepare_for_rx(1, false);
        feed(&mut t, 1, &frame(30, 1600, 0, 0, 0));
        assert!(t.take(1).valid);

        // Three unanswered requests are tolerated
        for _ in 0..3 {
            t.prepare_for_rx(1, false);
        }
        t.prepare_for_rx(3, false); // closes the third miss on ESC 1
        let esc = t.take(1);
        assert!(esc.valid);
        assert_eq!(esc.total_missed, 3);

        // The fourth makes it invalid; from then on only statistics are reported
        t.prepare_for_rx(1, false);
        t.prepare_for_rx(3, false);
        let esc = t.take(1);
        assert!(!esc.valid);
        assert_eq!(esc.total_invalid, 1);
        assert_eq!(esc.voltage, 0);

        // A good frame recovers it and resets the miss counter
        t.prepare_for_rx(1, false);
        feed(&mut t, 1, &frame(30, 1500, 0, 0, 0));
        let esc = t.take(1);
        assert!(esc.valid);
        assert_eq!(esc.missed_count, 0);
        assert_eq!(esc.voltage, 1500);
    }

    #[test]
    fn mechanical_rpm_saturates() {
        let esc = EscTelemetry { rpm: u16::MAX, ..Default::default() };
        assert_eq!(esc.mechanical_rpm(), i16::MAX);
        let esc = EscTelemetry { rpm: u16::MAX, rpm_reversed: true, ..Default::default() };
        assert_eq!(esc.mechanical_rpm(), i16::MIN);
    }
}
