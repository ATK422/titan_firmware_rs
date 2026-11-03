//! Puddles backplane (`puddles/Backplane`) messages.
//!
//! The backplane is the only board on Puddles. It drives the eight thrusters over bidirectional DShot, reads the
//! depth sensor and the rail voltages, senses the physical kill switch and switches power to the robot computer and
//! the acoustics. Unlike the Talos boards, its C node publishes `state/kill` (from the physical switch) and takes
//! software kills on `command/software_kill`, and its heartbeat topic is `state/fw_heartbeat`.

use serde::{Deserialize, Serialize};

/// Number of thrusters (`NUM_THRUSTERS`).
pub const NUM_THRUSTERS: usize = 8;

/// Longest software kill sender ID the firmware accepts is one less than this
/// (`SAFETY_SOFTWARE_KILL_FRAME_STR_SIZE`). The bridge sends at most this many bytes, so an ID that is too long
/// still arrives too long and gets rejected like in C.
pub const SENDER_ID_LEN: usize = 32;

/// Messages from the bridge to the board.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HostToDevice {
    /// From `command/dshot` (`riptide_msgs2/DshotCommand`): one throttle per thruster in -999..=999.
    Dshot([i16; NUM_THRUSTERS]),
    /// From `command/software_kill` (`riptide_msgs2/KillSwitchReport`).
    SoftwareKill(SoftwareKill),
    /// From `command/electrical` (`riptide_msgs2/ElectricalCommand`), for the commands the backplane implements.
    Electrical(ElectricalCommand),
}

/// A software kill switch update, field for field `riptide_msgs2/KillSwitchReport`.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SoftwareKill {
    /// Identifies the node, so only the node that killed can unkill. At most [`SENDER_ID_LEN`] bytes: longer IDs
    /// are cut to that length, which the firmware rejects.
    pub sender_id: heapless::Vec<u8, SENDER_ID_LEN>,
    pub kill_switch_id: u8,
    pub asserting_kill: bool,
    pub needs_update: bool,
}

/// `riptide_msgs2/ElectricalCommand` values the backplane acts on. The C firmware ignored every other command.
#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ElectricalCommand {
    /// `CYCLE_COMPUTER`: switch the robot computer off for a second.
    CycleComputer,
    /// `CYCLE_ACCOUSTICS`: switch the acoustics off for a second.
    CycleAcoustics,
    /// `CLEAR_DEPTH`: zero the depth sensor again.
    ClearDepth,
}

impl ElectricalCommand {
    /// Maps a `riptide_msgs2/ElectricalCommand.command` value, or `None` for commands the backplane ignores.
    pub fn from_ros(command: u8) -> Option<Self> {
        match command {
            0 => Some(ElectricalCommand::CycleComputer),
            1 => Some(ElectricalCommand::CycleAcoustics),
            7 => Some(ElectricalCommand::ClearDepth),
            _ => None,
        }
    }
}

/// Messages from the board to the bridge.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeviceToHost {
    /// For `state/kill` (`std_msgs/Bool`, best effort): whether the vehicle is killed.
    Kill(bool),
    /// For `state/thrusters/rpm` (`riptide_msgs2/DshotRPMFeedback`, best effort), after every thruster command.
    Rpm(RpmFeedback),
    /// For `state/depth/raw` (`riptide_msgs2/Depth`, sensor data QoS): the `depth` field in metres, negative below
    /// the surface. The bridge adds the header and variance.
    Depth(f32),
    /// For `state/depth/temp` (`std_msgs/Float32`, sensor data QoS): water temperature in degrees Celsius.
    WaterTemp(f32),
    /// For `state/electrical` (`riptide_msgs2/ElectricalReadings`).
    Electrical(ElectricalReadings),
}

/// Field for field `riptide_msgs2/DshotRPMFeedback`.
#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RpmFeedback {
    /// Bit `i` is set when `rpm[i]` is valid.
    pub valid_mask: u8,
    /// Signed mechanical RPM of each thruster.
    pub rpm: [i16; NUM_THRUSTERS],
}

/// The `riptide_msgs2/ElectricalReadings` fields the backplane measures, in volts. NaN until the ADC has read the
/// rail, as in C. The other fields are published as zero / false.
#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ElectricalReadings {
    pub three_volt_voltage: f32,
    pub five_volt_voltage: f32,
    pub twelve_volt_voltage: f32,
    pub balanced_voltage: f32,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{MAX_ENCODED_LEN, MAX_FD_FRAME_LEN, decode, encode};

    #[test]
    fn round_trip_and_size() {
        let host = [
            crate::HostToDevice::PuddlesBackplane(HostToDevice::Dshot([i16::MIN, -999, -1, 0, 1, 999, i16::MAX, 5])),
            crate::HostToDevice::PuddlesBackplane(HostToDevice::SoftwareKill(SoftwareKill {
                sender_id: heapless::Vec::from_slice(&[b'x'; SENDER_ID_LEN]).unwrap(),
                kill_switch_id: 3,
                asserting_kill: true,
                needs_update: true,
            })),
            crate::HostToDevice::PuddlesBackplane(HostToDevice::Electrical(ElectricalCommand::ClearDepth)),
        ];
        for msg in host {
            let mut buf = [0u8; MAX_FD_FRAME_LEN];
            let encoded = encode(&msg, &mut buf).expect("fits one CAN FD frame");
            assert_eq!(decode::<crate::HostToDevice>(encoded).unwrap(), msg);
        }

        let device = [
            DeviceToHost::Kill(true),
            DeviceToHost::Rpm(RpmFeedback { valid_mask: 0xFF, rpm: [i16::MIN; NUM_THRUSTERS] }),
            DeviceToHost::Depth(-1.25),
            DeviceToHost::WaterTemp(18.5),
            DeviceToHost::Electrical(ElectricalReadings {
                three_volt_voltage: f32::NAN,
                five_volt_voltage: 5.0,
                twelve_volt_voltage: 12.0,
                balanced_voltage: 16.8,
            }),
        ];
        for msg in device {
            let msg = crate::DeviceToHost::PuddlesBackplane(msg);
            let mut buf = [0u8; MAX_ENCODED_LEN];
            let len = encode(&msg, &mut buf).unwrap().len();
            assert!(len <= MAX_FD_FRAME_LEN);
            let decoded = decode::<crate::DeviceToHost>(&buf[..len]).unwrap();
            // NaN != NaN, so compare the encodings
            let mut again = [0u8; MAX_ENCODED_LEN];
            assert_eq!(encode(&decoded, &mut again).unwrap(), &buf[..len]);
        }
    }

    #[test]
    fn electrical_command_mapping() {
        assert_eq!(ElectricalCommand::from_ros(0), Some(ElectricalCommand::CycleComputer));
        assert_eq!(ElectricalCommand::from_ros(1), Some(ElectricalCommand::CycleAcoustics));
        assert_eq!(ElectricalCommand::from_ros(7), Some(ElectricalCommand::ClearDepth));
        for ignored in [2, 3, 4, 5, 6, 8, 9, 10, 255] {
            assert_eq!(ElectricalCommand::from_ros(ignored), None);
        }
    }
}
