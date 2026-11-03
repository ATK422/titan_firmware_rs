//! Transport-neutral view of the connection to one board.

use anyhow::{Result, bail};
use canmore::Heartbeat;
use titan_link::{DeviceToHost, HostToDevice};

/// Something received from the board.
pub enum Event {
    Message(DeviceToHost),
    /// A CANmore heartbeat (CAN transports only).
    Heartbeat(Heartbeat),
    /// A CANmore utility frame other than the heartbeat (CAN transports only).
    Utility {
        channel: u8,
        data: Vec<u8>,
    },
    /// Nothing arrived for a while; lets the caller check timeouts.
    Idle,
}

/// A connection to one board.
pub trait Link: Send + Sync {
    /// Sends a message to the board.
    fn send(&self, msg: &HostToDevice) -> Result<()>;

    /// Blocks until something arrives from the board or a short timeout passes (then returns [`Event::Idle`]).
    fn receive(&self) -> Result<Event>;

    /// Sends a CANmore utility frame to the board, for latency sensitive traffic such as thruster commands.
    #[allow(dead_code)] // Used by boards that take thruster commands
    fn send_utility(&self, channel: u8, data: &[u8]) -> Result<()> {
        let _ = (channel, data);
        bail!("this transport has no utility frames")
    }
}
