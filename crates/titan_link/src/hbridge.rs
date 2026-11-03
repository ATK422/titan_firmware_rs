//! Lilboat H-bridge board (`lilboat/HBridgeBoard`) messages.

use serde::{Deserialize, Serialize};

/// Number of bridge command topics.
pub const NUM_BRIDGE_TOPICS: usize = 5;

/// Messages from the bridge to the board.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HostToDevice {
    /// From one of the five bridge command topics (`std_msgs/Float32`, `[-1, 1]`). `bridge` is the C topic
    /// index: 0 `command/thruster/port`, 1 `command/thruster/stbd`, 2 `command/motor/stbd`, 3 `command/motor/port`,
    /// 4 `command/bridge2/unused`.
    SetTarget { bridge: u8, percent: f32 },
    /// From `command/actuator/arm` (`std_msgs/Bool`): arm (`true`) or disarm the servos.
    ActuatorArm(bool),
    /// From `command/actuator/move_speed_for_time` (`std_msgs/Int32`): run the claw grip at this speed for 1 s.
    MoveSpeedForTime(i32),
}

/// Messages from the board to the bridge.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeviceToHost {
    /// For `state/actuator/status` (`riptide_msgs2/ActuatorStatus`): the claw state. The C board set no other
    /// field; the bridge publishes them as zero.
    ActuatorStatus { claw_state: u8 },
    /// For `state/actuator/degrees` (`std_msgs/Int32`): last claw grip position read.
    ActuatorDegrees(i32),
    /// For `state/actuator/cmd_status` (`std_msgs/Bool`) and `state/actuator/cmd_feedback` (`std_msgs/String`):
    /// the outcome of the last actuator command.
    CmdFeedback { success: bool, message: Feedback },
}

/// The `state/actuator/cmd_feedback` text.
#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Feedback {
    /// Empty string.
    None,
    KillSwitchRemoved,
    AlreadyArmed,
    SpeedOutOfRange,
}

impl Feedback {
    /// The string the C firmware published.
    pub fn as_str(self) -> &'static str {
        match self {
            Feedback::None => "",
            Feedback::KillSwitchRemoved => "Kill Switch Removed",
            Feedback::AlreadyArmed => "At least one actuator already armed",
            Feedback::SpeedOutOfRange => "Requested speed out of range",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{MAX_FD_FRAME_LEN, decode, encode};

    #[test]
    fn messages_round_trip_in_one_frame() {
        let host = [
            HostToDevice::SetTarget { bridge: 4, percent: -0.75 },
            HostToDevice::ActuatorArm(true),
            HostToDevice::MoveSpeedForTime(-1000),
        ];
        for msg in host {
            let msg = crate::HostToDevice::HBridge(msg);
            let mut buf = [0u8; MAX_FD_FRAME_LEN];
            let encoded = encode(&msg, &mut buf).unwrap();
            assert_eq!(decode::<crate::HostToDevice>(encoded).unwrap(), msg);
        }
        let device = [
            DeviceToHost::ActuatorStatus { claw_state: 5 },
            DeviceToHost::ActuatorDegrees(240),
            DeviceToHost::CmdFeedback { success: false, message: Feedback::AlreadyArmed },
        ];
        for msg in device {
            let msg = crate::DeviceToHost::HBridge(msg);
            let mut buf = [0u8; MAX_FD_FRAME_LEN];
            let encoded = encode(&msg, &mut buf).unwrap();
            assert_eq!(decode::<crate::DeviceToHost>(encoded).unwrap(), msg);
        }
    }

    #[test]
    fn feedback_strings_match_c() {
        assert_eq!(Feedback::None.as_str(), "");
        assert_eq!(Feedback::KillSwitchRemoved.as_str(), "Kill Switch Removed");
        assert_eq!(Feedback::AlreadyArmed.as_str(), "At least one actuator already armed");
        assert_eq!(Feedback::SpeedOutOfRange.as_str(), "Requested speed out of range");
    }
}
