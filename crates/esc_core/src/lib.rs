//! Hardware independent core of the Talos ESC board (`talos/ESCBoard`).
//!
//! Each ESC board drives four APD 80F3 ESCs (patched for 3D mode, see `esc_firmware_patching/` in the C tree) with
//! bidirectional DShot600 and listens to their UART telemetry. A closed loop RPM controller turns target RPMs into
//! DShot throttle commands using the eRPM the ESCs send back after every frame.
//!
//! * [`dshot`]: throttle mapping, DShot frame encoding and the telemetry request schedule (`dshot.c`)
//! * [`erpm`]: GCR decoding of the bidirectional DShot eRPM reply (`dshot.c`)
//! * [`telemetry`]: KISS/BLHeli UART telemetry frames and the per-ESC bookkeeping (`dshot.c`, `uart_telemetry.h`)
//! * [`controller`]: the feed-forward + PI thruster controller (`Thruster_Controller/ThrusterController.c`)
//! * [`control_loop`]: the core 1 control loop logic (`core1.c`) with the I/O taken out
//!
//! Everything here is a direct port of the C firmware so the Rust board behaves like the C one; the few deliberate
//! differences (integer overflow, division by zero on zero gains) are called out where they occur.

#![no_std]

pub mod control_loop;
pub mod controller;
pub mod dshot;
pub mod erpm;
pub mod telemetry;

/// Thrusters driven by one ESC board.
pub const NUM_THRUSTERS: usize = 4;

/// Electrical to mechanical RPM conversion (T200 thrusters have 7 pole pairs).
pub const ESC_NUM_POLE_PAIRS: i32 = 7;

/// New commands must arrive at least this often, or the thrusters are disabled.
pub const DSHOT_MIN_UPDATE_RATE_MS: u64 = 500;

/// After a command times out while the thrusters were moving, commands are ignored for this long (prevents jitter
/// from commands arriving slightly slower than [`DSHOT_MIN_UPDATE_RATE_MS`]).
pub const DSHOT_UPDATE_DISABLE_TIME_MS: u64 = 1000;

/// Thrusters stay disabled this long after the ESCs power up, while they chime.
pub const ESC_POWERUP_DELAY_MS: u64 = 5000;

/// DShot bit rate in kbit/s (DShot600).
pub const DSHOT_RATE_KHZ: u32 = 600;

/// Minimum time per control loop tick. Sending frames faster makes the ESCs drop them, and the ESC may still be
/// answering the previous frame.
pub const DSHOT_MIN_FRAME_TIME_US: u64 = 400;

/// The ESC rail counts as powered above this voltage.
pub const ESC_POWER_THRESHOLD_MV: u32 = 9000;

/// Number of DShot frames between UART telemetry requests (telemetry for each ESC refreshes every
/// `TELEM_PACKET_DELAY * NUM_THRUSTERS` frames).
pub const TELEM_PACKET_DELAY: u32 = 100;

/// UART telemetry requests that may go unanswered before an ESC's telemetry counts as invalid.
pub const TELEM_MAX_MISSED_UART_PACKETS: u16 = 3;

/// eRPM replies that may be missed before a thruster is taken offline and its command cleared.
pub const TELEM_MAX_MISSED_DSHOT_PACKETS: u8 = 20;

/// UART telemetry baud rate.
pub const TELEM_BAUD: u32 = 115_200;

/// Converts a raw 12-bit ADC reading of the divided VCC rail (3.3 V reference, 10k/1k divider) to millivolts.
pub fn vcc_mv_from_adc(raw: u16) -> u32 {
    (raw as u32 & 0xFFF) * 36_300 / 4096
}

/// True if the ESC rail at `vcc_mv` counts as powered.
pub fn esc_board_on(vcc_mv: u32) -> bool {
    vcc_mv > ESC_POWER_THRESHOLD_MV
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vcc_conversion() {
        assert_eq!(vcc_mv_from_adc(0), 0);
        // Full scale is 3.3 V * 11 = 36.3 V
        assert_eq!(vcc_mv_from_adc(4095), 36_291);
        // Upper bits (ADC error flag) are masked like `adc_fifo_get() & 0xFFF`
        assert_eq!(vcc_mv_from_adc(0x8000 | 2048), 18_150);
        assert!(!esc_board_on(vcc_mv_from_adc(1015)));
        assert!(esc_board_on(vcc_mv_from_adc(1100)));
    }
}
