//! Actuator board (`talos/Actuator`) messages.
//!
//! The C board served ROS services from micro-ROS. Here each service call becomes a [`HostToDevice::Command`]
//! tagged with a sequence number, and the board answers with a [`DeviceToHost::CommandResult`] carrying the same
//! number; the bridge hosts the ROS service and matches the two up. Failure reasons travel as an
//! [`ActuatorError`] code and the bridge turns them back into the C firmware's message strings.
//!
//! The board was built either for the original actuators (`ACTUATOR_V1_SUPPORT`: solenoid torpedoes, droppers and
//! a DC motor claw with timings set through ROS parameters) or for the Dynamixel actuators (`ACTUATOR_V2_SUPPORT`,
//! the default). The status says which ([`Variant`]), so the bridge can offer the matching parameters, topics and
//! services.

use serde::{Deserialize, Serialize};

/// Messages from the bridge to the board.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HostToDevice {
    /// A service call (or, for V1 timings, a parameter change). Answered by [`DeviceToHost::CommandResult`].
    Command { seq: u8, command: Command },
}

/// What the bridge asks the board to do.
#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Command {
    /// `command/actuator/arm` with `true`.
    Arm,
    /// `command/actuator/arm` with `false`. Always succeeds.
    Disarm,
    /// `command/actuator/torpedo`.
    FireTorpedo,
    /// `command/actuator/dropper`.
    DropMarker,
    /// `command/actuator/notify_reload`: reset the torpedo count, then the dropper count.
    NotifyReload,
    /// `command/actuator/claw`: `true` opens, `false` closes.
    Claw { open: bool },
    /// `command/actuator/torpedo_marker/go_home` (V2).
    TorpedoMarkerMoveHome,
    /// `command/actuator/torpedo_marker/set_home` (V2).
    TorpedoMarkerSetHome,
    /// `command/actuator/claw/set_closed_pos` (V2).
    ClawSetClosedPosition,
    /// The `claw_timing_ms` parameter (V1): claw open and close time.
    SetClawTiming { ms: u16 },
    /// The `dropper_active_timing_ms` parameter (V1).
    SetDropperTiming { ms: u16 },
    /// The `torpedo_<coil>_timing_us` parameters (V1), applied to both torpedoes.
    SetTorpedoTiming { timing: TorpedoTiming, us: u16 },
}

/// One step of the V1 torpedo coil sequence (`enum torpedo_timing_type`).
#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TorpedoTiming {
    Coil1On,
    Coil1To2Delay,
    Coil2On,
    Coil2To3Delay,
    Coil3On,
}

impl TorpedoTiming {
    pub const ALL: [TorpedoTiming; 5] = [
        TorpedoTiming::Coil1On,
        TorpedoTiming::Coil1To2Delay,
        TorpedoTiming::Coil2On,
        TorpedoTiming::Coil2To3Delay,
        TorpedoTiming::Coil3On,
    ];

    /// The ROS parameter that sets this timing.
    pub fn parameter_name(self) -> &'static str {
        match self {
            TorpedoTiming::Coil1On => "torpedo_coil1_on_timing_us",
            TorpedoTiming::Coil1To2Delay => "torpedo_coil1_2_delay_timing_us",
            TorpedoTiming::Coil2On => "torpedo_coil2_on_timing_us",
            TorpedoTiming::Coil2To3Delay => "torpedo_coil2_3_delay_timing_us",
            TorpedoTiming::Coil3On => "torpedo_coil3_on_timing_us",
        }
    }

    /// The value the C firmware set on connect.
    pub fn default_us(self) -> u16 {
        match self {
            TorpedoTiming::Coil1On => 23000,
            TorpedoTiming::Coil1To2Delay => 250,
            TorpedoTiming::Coil2On => 15000,
            TorpedoTiming::Coil2To3Delay => 250,
            TorpedoTiming::Coil3On => 13000,
        }
    }
}

/// Default of the `claw_timing_ms` parameter.
pub const DEFAULT_CLAW_TIMING_MS: u16 = 4500;
/// Default of the `dropper_active_timing_ms` parameter.
pub const DEFAULT_DROPPER_TIMING_MS: u16 = 250;

/// Messages from the board to the bridge.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeviceToHost {
    /// The answer to [`HostToDevice::Command`] with the same `seq`.
    CommandResult { seq: u8, result: Result<(), ActuatorError> },
    /// For `state/actuator/status` and `state/actuator/busy`, every second while connected.
    Status(Status),
    /// For `state/actuator/dynamixel` (V2): one per connected servo after each [`DeviceToHost::Status`].
    Dynamixel(DynamixelStatus),
    /// Sent by V1 boards when ROS connects: the bridge answers with every timing parameter, as the C firmware's
    /// parameter server set them on connect.
    RequestTimings,
}

/// Actuator hardware the firmware was built for.
#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Variant {
    /// Solenoid torpedoes, GPIO droppers and a DC motor claw (`ACTUATOR_V1_SUPPORT`).
    V1,
    /// Dynamixel torpedo/marker and claw (`ACTUATOR_V2_SUPPORT`).
    V2,
}

/// `riptide_msgs2/ActuatorStatus` plus the `state/actuator/busy` flag.
#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Status {
    pub variant: Variant,
    pub armed: bool,
    /// An actuator is carrying out a command.
    pub busy: bool,
    pub claw: ClawState,
    pub torpedo: TorpedoState,
    pub torpedo_available: u8,
    pub dropper: DropperState,
    pub dropper_available: u8,
}

#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClawState {
    Error,
    Disarmed,
    Unknown,
    Opened,
    Closed,
    Opening,
    Closing,
}

#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TorpedoState {
    Error,
    Disarmed,
    Charging,
    Charged,
    Firing,
    Fired,
}

#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DropperState {
    Error,
    Disarmed,
    Ready,
    Dropping,
    Dropped,
}

/// Field for field copy of `riptide_msgs2/DynamixelStatus`.
#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DynamixelStatus {
    pub model: u16,
    pub fw_version: u8,
    pub id: u8,
    pub hw_err_status: u8,
    pub torque_enable: bool,
    pub moving: bool,
    /// 0.1 V units.
    pub voltage: u16,
    /// °C.
    pub temperature: u8,
    pub position: i32,
    pub load: i16,
}

/// Why a command failed. [`ActuatorError::message`] gives the text the C firmware put in the service response.
#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ActuatorError {
    AlreadyArmed,
    KillSwitchRemoved,
    /// V2 arming with a servo missing.
    ActuatorNotConnected,
    NotConnected,
    NotArmed,
    Busy,
    HardwareError,
    NotHomed,
    MustBeDisarmed,
    StateReadError,
    HomingOffsetTooLarge,
    AllDropped,
    AllFired,
    /// V1: the kill switch fired while arming or firing.
    Killed,
    AllTorpedosFired,
    AlreadyFiring,
    NotCharged,
    MissingTimings,
    PioError,
    FiringInProgress,
    AllDroppersFired,
    KillSwitchPulled,
    DropperActive,
    KillPulled,
    InvalidState,
    /// A V1 timing was out of range or changed while armed (the C parameter callback just returned false).
    TimingRejected,
    /// The command belongs to the other actuator variant (the C board didn't have that service at all).
    Unsupported,
}

impl ActuatorError {
    /// The C firmware's message for this error.
    pub fn message(self) -> &'static str {
        match self {
            ActuatorError::AlreadyArmed => "Already Armed",
            ActuatorError::KillSwitchRemoved => "Kill Switch Removed",
            ActuatorError::ActuatorNotConnected => "Actuator Not Connected",
            ActuatorError::NotConnected => "Not Connected",
            ActuatorError::NotArmed => "Not Armed",
            ActuatorError::Busy => "Busy",
            ActuatorError::HardwareError => "Hardware Error",
            ActuatorError::NotHomed => "Not Homed",
            ActuatorError::MustBeDisarmed => "Must be disarmed",
            ActuatorError::StateReadError => "State read error",
            ActuatorError::HomingOffsetTooLarge => "Homing offset too large",
            ActuatorError::AllDropped => "All Dropped",
            ActuatorError::AllFired => "All Fired",
            ActuatorError::Killed => "Killed",
            ActuatorError::AllTorpedosFired => "All Torpedos Fired",
            ActuatorError::AlreadyFiring => "Already Firing",
            ActuatorError::NotCharged => "Not Charged",
            ActuatorError::MissingTimings => "Missing Timings",
            ActuatorError::PioError => "PIO Error",
            ActuatorError::FiringInProgress => "Firing in progress",
            ActuatorError::AllDroppersFired => "All Droppers Fired",
            ActuatorError::KillSwitchPulled => "Kill Switch Pulled",
            ActuatorError::DropperActive => "Dropper Active",
            ActuatorError::KillPulled => "Kill Pulled",
            ActuatorError::InvalidState => "Invalid State",
            ActuatorError::TimingRejected => "Timing Rejected",
            ActuatorError::Unsupported => "Not supported by this actuator version",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{DeviceToHost as D2H, HostToDevice as H2D, MAX_FD_FRAME_LEN, decode, encode};

    #[test]
    fn round_trip_and_fit_one_frame() {
        let commands = [
            Command::Arm,
            Command::Disarm,
            Command::FireTorpedo,
            Command::DropMarker,
            Command::NotifyReload,
            Command::Claw { open: true },
            Command::TorpedoMarkerMoveHome,
            Command::TorpedoMarkerSetHome,
            Command::ClawSetClosedPosition,
            Command::SetClawTiming { ms: u16::MAX },
            Command::SetDropperTiming { ms: 250 },
            Command::SetTorpedoTiming { timing: TorpedoTiming::Coil2To3Delay, us: u16::MAX },
        ];
        for command in commands {
            let msg = H2D::Actuator(HostToDevice::Command { seq: 255, command });
            let mut buf = [0u8; MAX_FD_FRAME_LEN];
            let encoded = encode(&msg, &mut buf).expect("fits one CAN FD frame");
            assert_eq!(decode::<H2D>(encoded).unwrap(), msg);
        }

        let device = [
            DeviceToHost::CommandResult { seq: 7, result: Ok(()) },
            DeviceToHost::CommandResult { seq: 8, result: Err(ActuatorError::Unsupported) },
            DeviceToHost::Status(Status {
                variant: Variant::V2,
                armed: true,
                busy: false,
                claw: ClawState::Closing,
                torpedo: TorpedoState::Fired,
                torpedo_available: 2,
                dropper: DropperState::Dropping,
                dropper_available: 1,
            }),
            DeviceToHost::Dynamixel(DynamixelStatus {
                model: 1060,
                fw_version: 46,
                id: 3,
                hw_err_status: 0x24,
                torque_enable: true,
                moving: true,
                voltage: 121,
                temperature: 30,
                position: i32::MIN,
                load: i16::MIN,
            }),
            DeviceToHost::RequestTimings,
        ];
        for msg in device {
            let msg = D2H::Actuator(msg);
            let mut buf = [0u8; MAX_FD_FRAME_LEN];
            let encoded = encode(&msg, &mut buf).expect("fits one CAN FD frame");
            assert_eq!(decode::<D2H>(encoded).unwrap(), msg);
        }
    }

    #[test]
    fn timing_names_match_c_parameters() {
        let names: [&str; 5] = TorpedoTiming::ALL.map(TorpedoTiming::parameter_name);
        assert_eq!(
            names,
            [
                "torpedo_coil1_on_timing_us",
                "torpedo_coil1_2_delay_timing_us",
                "torpedo_coil2_on_timing_us",
                "torpedo_coil2_3_delay_timing_us",
                "torpedo_coil3_on_timing_us",
            ]
        );
    }
}
