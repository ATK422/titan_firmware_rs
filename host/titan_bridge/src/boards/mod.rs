//! Per-board ROS interfaces. Each board module exports a [`BoardProfile`] named `PROFILE`, listed in [`BOARDS`].

use std::sync::Arc;

use anyhow::Result;
use rclrs::Node;
use titan_link::DeviceToHost;

use crate::link::Link;

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

/// Every board the bridge knows about.
pub const BOARDS: &[&BoardProfile] = &[
    &ivc::PROFILE,
    &ivc2::PROFILE,
    &ivc_dac::PROFILE,
    &actuator::PROFILE,
    &camera_cage::PROFILE,
    &esc::PROFILE,
    &esc::PROFILE_BOARD1,
    &led::PROFILE,
    &led2::PROFILE,
    &poac::PROFILE,
    &power::PROFILE,
    &smart_battery::PROFILE,
    &sbc::PROFILE,
    &puddles_backplane::PROFILE,
    &hbridge::PROFILE,
    &po_half::PROFILE,
];

/// Boards whose C firmware published the heartbeat on `state/fw_heartbeat` instead of `heartbeat`.
const FW_HEARTBEAT_BOARDS: &[&str] =
    &["actuator", "camera_cage", "esc0", "esc1", "power", "puddles_backplane", "smart_battery"];

/// Boards that publish `state/kill` (they own the physical kill switch) instead of subscribing to it.
const KILL_SOURCE_BOARDS: &[&str] = &["poac", "po_half", "power", "puddles_backplane"];

/// Boards whose C node name carried the client ID (`smart_battery_<serial>`).
const NODE_NAME_WITH_CLIENT_ID: &[&str] = &["smart_battery"];

impl BoardProfile {
    /// True if the board takes the robot's kill state from `state/kill`.
    pub fn subscribes_kill(&self) -> bool {
        !KILL_SOURCE_BOARDS.contains(&self.name)
    }

    /// ROS node name for the board with the given client ID.
    pub fn node_name_for(&self, client_id: u8) -> String {
        if NODE_NAME_WITH_CLIENT_ID.contains(&self.name) {
            format!("{}_{client_id}", self.node_name)
        } else {
            self.node_name.to_string()
        }
    }

    /// Topic the board's heartbeat is published on, matching its C firmware.
    pub fn heartbeat_topic(&self) -> &'static str {
        if FW_HEARTBEAT_BOARDS.contains(&self.name) { "state/fw_heartbeat" } else { "heartbeat" }
    }
}

/// Board specific topics.
pub trait BoardTopics: Send + Sync {
    /// Publishes a board specific message. Returns `Ok(false)` if the message is not for this board.
    fn handle(&self, msg: &DeviceToHost) -> Result<bool>;

    /// Handles a CANmore utility frame from the board. Returns `Ok(false)` if the channel is not used by this board.
    fn handle_utility(&self, channel: u8, data: &[u8]) -> Result<bool> {
        let _ = (channel, data);
        Ok(false)
    }
}

/// Topics of a board that only has the common ones.
pub struct NoTopics;

impl BoardTopics for NoTopics {
    fn handle(&self, _msg: &DeviceToHost) -> Result<bool> {
        Ok(false)
    }
}

/// How the bridge reaches the board.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Transport {
    /// CANmore over SocketCAN. `fd` for the internal CAN FD bus, classic CAN otherwise.
    Can { fd: bool },
    /// UDP over Ethernet (Puddles).
    Udp,
}

/// Creates a board's topics on `node`, sending commands through the link and logging under the given name.
pub type CreateTopics = fn(&Node, Arc<dyn Link>, String) -> Result<Box<dyn BoardTopics>>;

pub struct BoardProfile {
    /// Name used on the command line.
    pub name: &'static str,
    /// ROS node name, matching `PICO_TARGET_NAME` of the C firmware.
    pub node_name: &'static str,
    /// Default ROS namespace (the robot).
    pub namespace: &'static str,
    /// Default CANmore client ID. Boards that choose their ID at runtime are bridged with `--client-id`.
    pub client_id: u8,
    pub transport: Transport,
    pub create: CreateTopics,
}
