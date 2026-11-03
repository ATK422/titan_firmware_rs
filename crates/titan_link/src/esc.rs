//! ESC board (`talos/ESCBoard`) messages.
//!
//! Each ESC board drives four of Talos' eight thrusters: board 0 has thrusters 0-3, board 1 has 4-7. Messages from
//! the board say which half they describe in `start_thruster`, so the bridge doesn't need to know which board it
//! serves. Thruster commands from the robot computer's controller don't travel here: they arrive as CANmore
//! utility frames on channel 0 (four big-endian `i16` target RPMs), straight from the controller software.

use serde::{Deserialize, Serialize};

/// Thrusters driven by one ESC board.
pub const THRUSTERS_PER_BOARD: usize = 4;
/// Thrusters on the vehicle (`riptide_msgs2/DshotCommand::values`).
pub const TOTAL_THRUSTERS: usize = 8;

#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HostToDevice {
    /// From `command/thruster_rpm` (`riptide_msgs2/DshotCommand`): target RPM of all eight thrusters. Each board
    /// takes its own four and answers with [`DeviceToHost::Rpm`].
    ThrusterRpm([i16; TOTAL_THRUSTERS]),
    /// From `dshot_tune/set_raw`: treat targets as raw DShot throttle (-999..=999), bypassing the controller.
    SetRawMode(bool),
    /// From `dshot_tune/p_gain`.
    SetPGain(i32),
    /// From `dshot_tune/i_gain`.
    SetIGain(i32),
    /// From `dshot_tune/i_bound`.
    SetIBound(i32),
    /// From `dshot_tune/hard_limit`.
    SetHardLimit(i32),
    /// From `dshot_tune/min_command`.
    SetMinCommand(i32),
}

#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeviceToHost {
    /// For `state/thrusters/rpm` and `state/thrusters/cmd` (`riptide_msgs2/DshotRPMFeedback`), sent after each
    /// [`HostToDevice::ThrusterRpm`].
    Rpm(RpmFeedback),
    /// For `state/thrusters/telemetry` (`riptide_msgs2/DshotPartialTelemetry`), every 500 ms.
    Telemetry(Telemetry),
}

/// RPM feedback and current commands of one board's thrusters.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RpmFeedback {
    /// Index of the first thruster of this board (0 or 4).
    pub start_thruster: u8,
    /// Bit `i` set: `rpm[i]` is valid. The bridge shifts it by `start_thruster`.
    pub valid_mask: u8,
    /// Mechanical RPM (signed by the commanded direction).
    pub rpm: [i16; THRUSTERS_PER_BOARD],
    /// DShot throttle currently sent (-999..=999).
    pub cmd: [i16; THRUSTERS_PER_BOARD],
}

/// Board level telemetry (`riptide_msgs2/DshotPartialTelemetry`).
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Telemetry {
    /// Index of the first thruster of this board (0 or 4).
    pub start_thruster: u8,
    /// The ESC rail is above the power threshold.
    pub escs_powered: bool,
    /// Any thruster is being sent a non-neutral command.
    pub thrusters_moving: bool,
    /// Why the control loop is disabled (bits `CORE1_DISABLED_*`), 0 when running.
    pub disabled_flags: u8,
    /// ESC rail voltage in millivolts.
    pub vcc_mv: u32,
    /// Time over which `tick_cnt` was counted.
    pub packet_delta_us: u32,
    /// Control loop ticks since the previous telemetry message.
    pub tick_cnt: u16,
    pub escs: [EscTelemetry; THRUSTERS_PER_BOARD],
}

/// Telemetry of one ESC (`riptide_msgs2/DshotSingleTelemetry`) in the units the ESC reports.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EscTelemetry {
    /// UART telemetry and eRPM feedback are both coming in. The data fields below are zero otherwise.
    pub esc_online: bool,
    pub thruster_ready: bool,
    pub temperature_c: u8,
    /// Mechanical RPM from the UART telemetry.
    pub rpm: i16,
    /// Hundredths of a volt.
    pub voltage_cv: u16,
    /// Hundredths of an amp.
    pub current_ca: u16,
    pub consumption_mah: u16,
    pub dshot_missed_cnt: u16,
    pub dshot_offline_cnt: u16,
    pub uart_success_cnt: u16,
    pub uart_missed_cnt: u16,
    pub uart_disabled_cnt: u16,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{MAX_ENCODED_LEN, MAX_FD_FRAME_LEN, decode, encode};

    fn worst_esc() -> EscTelemetry {
        EscTelemetry {
            esc_online: true,
            thruster_ready: true,
            temperature_c: u8::MAX,
            rpm: i16::MIN,
            voltage_cv: u16::MAX,
            current_ca: u16::MAX,
            consumption_mah: u16::MAX,
            dshot_missed_cnt: u16::MAX,
            dshot_offline_cnt: u16::MAX,
            uart_success_cnt: u16::MAX,
            uart_missed_cnt: u16::MAX,
            uart_disabled_cnt: u16::MAX,
        }
    }

    fn round_trip<T>(msg: T, max_len: usize) -> usize
    where
        T: Serialize + for<'de> Deserialize<'de> + PartialEq + core::fmt::Debug,
    {
        let mut buf = [0u8; MAX_ENCODED_LEN];
        let encoded = encode(&msg, &mut buf).unwrap();
        let len = encoded.len();
        assert!(len <= max_len, "{len} bytes > {max_len}");
        assert_eq!(decode::<T>(encoded).unwrap(), msg);
        len
    }

    #[test]
    fn commands_fit_one_fd_frame() {
        let msgs = [
            crate::HostToDevice::Esc(HostToDevice::ThrusterRpm([i16::MIN; TOTAL_THRUSTERS])),
            crate::HostToDevice::Esc(HostToDevice::SetRawMode(true)),
            crate::HostToDevice::Esc(HostToDevice::SetPGain(i32::MIN)),
            crate::HostToDevice::Esc(HostToDevice::SetIGain(1000)),
            crate::HostToDevice::Esc(HostToDevice::SetIBound(300)),
            crate::HostToDevice::Esc(HostToDevice::SetHardLimit(725)),
            crate::HostToDevice::Esc(HostToDevice::SetMinCommand(0)),
        ];
        for msg in msgs {
            round_trip(msg, MAX_FD_FRAME_LEN);
        }
    }

    #[test]
    fn rpm_feedback_fits_one_fd_frame() {
        let msg = crate::DeviceToHost::Esc(DeviceToHost::Rpm(RpmFeedback {
            start_thruster: 4,
            valid_mask: 0xF,
            rpm: [i16::MIN; THRUSTERS_PER_BOARD],
            cmd: [-999; THRUSTERS_PER_BOARD],
        }));
        round_trip(msg, MAX_FD_FRAME_LEN);
    }

    #[test]
    fn worst_case_telemetry_fits() {
        // Sent every 500 ms, so a few fragments are fine
        let msg = crate::DeviceToHost::Esc(DeviceToHost::Telemetry(Telemetry {
            start_thruster: 4,
            escs_powered: true,
            thrusters_moving: true,
            disabled_flags: 0x3F,
            vcc_mv: u32::MAX,
            packet_delta_us: u32::MAX,
            tick_cnt: u16::MAX,
            escs: [worst_esc(), worst_esc(), worst_esc(), worst_esc()],
        }));
        let len = round_trip(msg, MAX_ENCODED_LEN);
        assert!(len <= 3 * MAX_FD_FRAME_LEN);
        // A typical message (small counters) is much shorter
        let typical = crate::DeviceToHost::Esc(DeviceToHost::Telemetry(Telemetry {
            start_thruster: 0,
            escs_powered: true,
            vcc_mv: 16_200,
            packet_delta_us: 500_000,
            tick_cnt: 1250,
            ..Default::default()
        }));
        round_trip(typical, MAX_FD_FRAME_LEN);
    }
}
