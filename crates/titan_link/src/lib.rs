//! `titan_link`: the protocol spoken between Rust firmware and the `titan_bridge` ROS node.
//!
//! The C firmware runs micro-ROS on the microcontroller and talks DDS-XRCE to a micro-ROS agent. The Rust firmware
//! instead exchanges these small, typed messages with `titan_bridge`, which owns the ROS node, topics and QoS on the
//! host side. Each message is serialized with `postcard` and carried in one CANmore message with subtype
//! `canmore::MSG_SUBTYPE_TITAN_LINK`; every message here fits in a single CAN FD frame.
//!
//! Connection management mirrors the micro-ROS behaviour: the bridge sends [`HostToDevice::Ping`] every
//! [`HOST_PING_INTERVAL_MS`], and the firmware treats ROS as connected while pings keep arriving within
//! [`HOST_TIMEOUT_MS`]. While connected, the firmware sends a heartbeat every [`HEARTBEAT_INTERVAL_MS`] and its
//! firmware status every [`FIRMWARE_STATUS_INTERVAL_MS`].

#![no_std]

use serde::{Deserialize, Serialize};

pub mod actuator;
pub mod camera_cage;
pub mod esc;
pub mod hbridge;
pub mod ivc;
pub mod ivc2;
pub mod ivc_dac;
pub mod led;
pub mod led2;
pub mod po_half;
pub mod poac;
pub mod power;
pub mod puddles_backplane;
pub mod sbc;
pub mod smart_battery;

/// Bumped on any incompatible change to the messages below.
pub const PROTOCOL_VERSION: u8 = 2;

/// How often the bridge pings the firmware.
pub const HOST_PING_INTERVAL_MS: u64 = 100;
/// How long without a ping before the firmware considers ROS disconnected.
/// Matches `MAX_MISSSED_HEARTBEATS` (7) heartbeats at 100 ms in the C firmware.
pub const HOST_TIMEOUT_MS: u64 = 700;
/// How often the firmware publishes its heartbeat while connected.
pub const HEARTBEAT_INTERVAL_MS: u64 = 100;
/// How often the firmware publishes its status while connected.
pub const FIRMWARE_STATUS_INTERVAL_MS: u64 = 1000;

/// Largest encoded message. Messages longer than one CAN frame are fragmented by CANmore, so this only bounds
/// buffer sizes; keep frequent messages within [`MAX_FD_FRAME_LEN`] where possible.
pub const MAX_ENCODED_LEN: usize = 256;

/// Payload of one CAN FD frame.
pub const MAX_FD_FRAME_LEN: usize = 64;

/// Maximum length of [`FirmwareStatus::board_name`], chosen so a worst case status still fits one CAN FD frame.
pub const BOARD_NAME_LEN: usize = 24;

/// Messages from the bridge to the firmware.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HostToDevice {
    /// Keeps the connection alive.
    Ping { protocol_version: u8 },
    /// Forwarded from the `state/kill` topic. `true` asserts kill (disables the vehicle).
    KillSwitch { asserting_kill: bool },
    // Board specific traffic. Append new boards at the end: the variant index is part of the wire format.
    /// IVC (inter-vehicle communication) board, `talos/IVCBoard`.
    Ivc(ivc::HostToDevice),
    /// IVC board, second revision firmware (`talos/IVCBoard2`).
    Ivc2(ivc2::HostToDevice),
    /// IVC board with DAC transmitter (`talos/IVCDAC`).
    IvcDac(ivc_dac::HostToDevice),
    /// Actuator board (`talos/Actuator`).
    Actuator(actuator::HostToDevice),
    /// Camera cage breakout board (`talos/CameraCageBB`).
    CameraCage(camera_cage::HostToDevice),
    /// ESC board (`talos/ESCBoard`).
    Esc(esc::HostToDevice),
    /// LED board (`talos/LEDBoard`).
    Led(led::HostToDevice),
    /// LED board, second revision firmware (`talos/LEDBoard2`).
    Led2(led2::HostToDevice),
    /// Power board with the torpedo/dropper servo (`talos/PoacBoard`).
    Poac(poac::HostToDevice),
    /// Power board (`talos/PowerBoard`).
    Power(power::HostToDevice),
    /// Smart battery housing MCU (`talos/SmartBattery`).
    SmartBattery(smart_battery::HostToDevice),
    /// SBC board (`SBCBoard`).
    Sbc(sbc::HostToDevice),
    /// Puddles backplane (`puddles/Backplane`).
    PuddlesBackplane(puddles_backplane::HostToDevice),
    /// Lilboat H-bridge board (`lilboat/HBridgeBoard`).
    HBridge(hbridge::HostToDevice),
    /// Lilboat power half board (`lilboat/PoHalfBoard`).
    PoHalf(po_half::HostToDevice),
}

/// Messages from the firmware to the bridge.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeviceToHost {
    /// Published on the `heartbeat` topic as the board's client ID.
    Heartbeat,
    /// Published on the `state/firmware` topic.
    FirmwareStatus(FirmwareStatus),
    // Board specific traffic. Append new boards at the end: the variant index is part of the wire format.
    /// IVC (inter-vehicle communication) board, `talos/IVCBoard`.
    Ivc(ivc::DeviceToHost),
    /// IVC board, second revision firmware (`talos/IVCBoard2`).
    Ivc2(ivc2::DeviceToHost),
    /// IVC board with DAC transmitter (`talos/IVCDAC`).
    IvcDac(ivc_dac::DeviceToHost),
    /// Actuator board (`talos/Actuator`).
    Actuator(actuator::DeviceToHost),
    /// Camera cage breakout board (`talos/CameraCageBB`).
    CameraCage(camera_cage::DeviceToHost),
    /// ESC board (`talos/ESCBoard`).
    Esc(esc::DeviceToHost),
    /// LED board (`talos/LEDBoard`).
    Led(led::DeviceToHost),
    /// LED board, second revision firmware (`talos/LEDBoard2`).
    Led2(led2::DeviceToHost),
    /// Power board with the torpedo/dropper servo (`talos/PoacBoard`).
    Poac(poac::DeviceToHost),
    /// Power board (`talos/PowerBoard`).
    Power(power::DeviceToHost),
    /// Smart battery housing MCU (`talos/SmartBattery`).
    SmartBattery(smart_battery::DeviceToHost),
    /// SBC board (`SBCBoard`).
    Sbc(sbc::DeviceToHost),
    /// Puddles backplane (`puddles/Backplane`).
    PuddlesBackplane(puddles_backplane::DeviceToHost),
    /// Lilboat H-bridge board (`lilboat/HBridgeBoard`).
    HBridge(hbridge::DeviceToHost),
    /// Lilboat power half board (`lilboat/PoHalfBoard`).
    PoHalf(po_half::DeviceToHost),
}

/// Field for field copy of `riptide_msgs2/msg/FirmwareStatus`.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FirmwareStatus {
    pub board_name: heapless::String<BOARD_NAME_LEN>,
    pub bus_id: u8,
    pub client_id: u8,
    pub version_major: u8,
    pub version_minor: u8,
    pub version_release_type: u8,
    pub uptime_ms: u32,
    pub faults: u32,
    pub kill_switches_enabled: u32,
    pub kill_switches_asserting_kill: u32,
    pub kill_switches_needs_update: u32,
    pub kill_switches_timed_out: u32,
}

/// Release type reported in [`FirmwareStatus::version_release_type`], matching `titan/version.h`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum ReleaseType {
    Debug = 0,
    Dev = 1,
    Clean = 2,
    Tagged = 3,
}

/// Encoding or decoding failed.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Error;

impl From<postcard::Error> for Error {
    fn from(_: postcard::Error) -> Self {
        Error
    }
}

/// Serializes `msg` into `buf`, returning the used part.
pub fn encode<'a, T: Serialize>(msg: &T, buf: &'a mut [u8]) -> Result<&'a mut [u8], Error> {
    Ok(postcard::to_slice(msg, buf)?)
}

/// Deserializes a message. Trailing bytes (such as CAN FD padding) are rejected.
pub fn decode<'a, T: Deserialize<'a>>(bytes: &'a [u8]) -> Result<T, Error> {
    let (msg, rest) = postcard::take_from_bytes(bytes)?;
    if rest.is_empty() { Ok(msg) } else { Err(Error) }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn full_status() -> FirmwareStatus {
        FirmwareStatus {
            board_name: heapless::String::try_from("talos/ivc_board_longname").unwrap(),
            bus_id: 1,
            client_id: 8,
            version_major: 255,
            version_minor: 255,
            version_release_type: ReleaseType::Tagged as u8,
            uptime_ms: u32::MAX,
            faults: u32::MAX,
            kill_switches_enabled: u32::MAX,
            kill_switches_asserting_kill: u32::MAX,
            kill_switches_needs_update: u32::MAX,
            kill_switches_timed_out: u32::MAX,
        }
    }

    #[test]
    fn worst_case_status_fits_one_fd_frame() {
        let mut buf = [0u8; MAX_FD_FRAME_LEN];
        let msg = DeviceToHost::FirmwareStatus(full_status());
        let encoded = encode(&msg, &mut buf).expect("must fit in a CAN FD frame");
        assert_eq!(decode::<DeviceToHost>(encoded).unwrap(), msg);
    }

    #[test]
    fn round_trip_all_messages() {
        let host = [
            HostToDevice::Ping { protocol_version: PROTOCOL_VERSION },
            HostToDevice::KillSwitch { asserting_kill: true },
            HostToDevice::Ivc(ivc::HostToDevice::Tx(0xA5)),
            HostToDevice::Ivc(ivc::HostToDevice::DebugTxFrequency(18000.0)),
        ];
        for msg in host {
            let mut buf = [0u8; MAX_ENCODED_LEN];
            let encoded = encode(&msg, &mut buf).unwrap();
            assert_eq!(decode::<HostToDevice>(encoded).unwrap(), msg);
        }

        let device = [
            DeviceToHost::Heartbeat,
            DeviceToHost::Ivc(ivc::DeviceToHost::Rx(0x3F)),
            DeviceToHost::Ivc(ivc::DeviceToHost::RxDebugMagnitude(312.5)),
        ];
        for msg in device {
            let mut buf = [0u8; MAX_ENCODED_LEN];
            let encoded = encode(&msg, &mut buf).unwrap();
            assert_eq!(decode::<DeviceToHost>(encoded).unwrap(), msg);
        }
    }

    #[test]
    fn padding_is_rejected() {
        let mut buf = [0u8; MAX_ENCODED_LEN];
        let len = encode(&DeviceToHost::Heartbeat, &mut buf).unwrap().len();
        assert!(decode::<DeviceToHost>(&buf[..len + 3]).is_err());
    }
}
