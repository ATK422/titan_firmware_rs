//! Poac board (`talos/PoacBoard`) messages: the power board's messages plus the torpedo/dropper servo.
//!
//! Despite the name nothing on this board does acoustics: the C firmware is the power board firmware with a
//! Hiwonder servo that fires the torpedoes and drops the markers.

use serde::{Deserialize, Serialize};

use crate::power;

#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HostToDevice {
    /// The power stage: `command/software_kill` and `command/electrical`.
    Power(power::HostToDevice),
    /// From `command/actuator/torpedo` (`std_msgs/Empty`).
    FireTorpedo,
    /// From `command/actuator/dropper` (`std_msgs/Empty`).
    DropMarker,
    /// From `command/actuator/notify_reload` (`std_msgs/Empty`).
    NotifyReload,
    /// From `command/actuator/arm` (`std_msgs/Bool`).
    Arm(bool),
    /// From `command/actuator/torpedo_marker/go_home` (`std_msgs/Empty`).
    GoHome,
    /// From `command/actuator/torpedo_marker/set_home` (`std_msgs/Empty`).
    SetHome,
}

/// For `state/actuator/status`: the fields of `riptide_msgs2/ActuatorStatus` the board sets (`rack_state` stays 0).
#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ActuatorStatus {
    pub actuators_armed: bool,
    pub claw_state: u8,
    pub torpedo_state: u8,
    pub torpedo_available_count: u8,
    pub dropper_state: u8,
    pub dropper_available_count: u8,
}

/// Why an actuator command was refused.
#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CommandError {
    KillSwitchRemoved,
    NotArmed,
    AllTorpedosFired,
    AlreadyFiring,
    Busy,
    AllDropped,
    AlreadyArmed,
    NotConnected,
    MustBeDisarmed,
}

impl CommandError {
    /// The text the C firmware published on `state/actuator/cmd_feedback`.
    pub fn message(self) -> &'static str {
        match self {
            CommandError::KillSwitchRemoved => "Kill Switch Removed",
            CommandError::NotArmed => "Not Armed",
            CommandError::AllTorpedosFired => "All Torpedos Fired",
            CommandError::AlreadyFiring => "Already Firing",
            CommandError::Busy => "Busy",
            CommandError::AllDropped => "All Dropped",
            CommandError::AlreadyArmed => "Already Armed",
            CommandError::NotConnected => "Not Connected",
            CommandError::MustBeDisarmed => "Must be disarmed",
        }
    }
}

#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeviceToHost {
    /// The power stage topics.
    Power(power::DeviceToHost),
    /// For `state/actuator/busy` and `state/actuator/status`, every 500 ms.
    Status { busy: bool, status: ActuatorStatus },
    /// For `state/actuator/cmd_feedback` (the error text, empty on success) and `state/actuator/cmd_status`
    /// (success): the outcome of the last command.
    CommandFeedback(Option<CommandError>),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{MAX_FD_FRAME_LEN, decode, encode};

    #[test]
    fn round_trip() {
        let host = [
            HostToDevice::Power(power::HostToDevice::ElectricalCommand(5)),
            HostToDevice::FireTorpedo,
            HostToDevice::DropMarker,
            HostToDevice::NotifyReload,
            HostToDevice::Arm(true),
            HostToDevice::GoHome,
            HostToDevice::SetHome,
        ];
        for msg in host {
            let msg = crate::HostToDevice::Poac(msg);
            let mut buf = [0u8; MAX_FD_FRAME_LEN];
            let encoded = encode(&msg, &mut buf).unwrap();
            assert_eq!(decode::<crate::HostToDevice>(encoded).unwrap(), msg);
        }

        let status = ActuatorStatus {
            actuators_armed: true,
            claw_state: 0,
            torpedo_state: 3,
            torpedo_available_count: 2,
            dropper_state: 2,
            dropper_available_count: 1,
        };
        let device = [
            DeviceToHost::Power(power::DeviceToHost::KillState(false)),
            DeviceToHost::Status { busy: true, status },
            DeviceToHost::CommandFeedback(None),
            DeviceToHost::CommandFeedback(Some(CommandError::MustBeDisarmed)),
        ];
        for msg in device {
            let msg = crate::DeviceToHost::Poac(msg);
            let mut buf = [0u8; MAX_FD_FRAME_LEN];
            let encoded = encode(&msg, &mut buf).unwrap();
            assert_eq!(decode::<crate::DeviceToHost>(encoded).unwrap(), msg);
        }
    }

    #[test]
    fn feedback_texts() {
        assert_eq!(CommandError::AllTorpedosFired.message(), "All Torpedos Fired");
        assert_eq!(CommandError::MustBeDisarmed.message(), "Must be disarmed");
    }
}
