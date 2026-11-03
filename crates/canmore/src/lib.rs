//! CANmore: the UWRT CAN transport layer, ported from `titan_canmore`.
//!
//! CANmore packs all framing information into the CAN identifier so the full data field is available to higher
//! level protocols. Two frame types exist:
//!
//! * **Message frames** carry a message of up to 1024 bytes, fragmented over several CAN frames and protected by a
//!   CRC-18. The 6-bit message subtype says what the message contains ([`MSG_SUBTYPE_XRCE_DDS`] for micro-ROS,
//!   [`MSG_SUBTYPE_TITAN_LINK`] for the Rust firmware link protocol).
//! * **Utility frames** carry out-of-band data on one of 16 channels, such as the heartbeat on
//!   [`CHAN_HEARTBEAT`].
//!
//! The bit layouts, CRC and fragmentation rules match `canmore/protocol.h` and `canmore/msg_encoding.c` exactly, so
//! frames produced here are understood by the existing C firmware and tooling.

#![no_std]

mod crc18;
pub mod id;
pub mod msg;
pub mod reg_mapped;

pub use id::{Direction, FrameType};
pub use msg::{DecodeError, Message, MsgDecoder, MsgEncoder};

/// Maximum data length of a classic CAN frame.
pub const MAX_FRAME_SIZE: usize = 8;
/// Maximum data length of a CAN FD frame.
pub const MAX_FD_FRAME_SIZE: usize = 64;
/// Maximum CANmore message length in bytes.
pub const MAX_MSG_LENGTH: usize = 1024;

/// Message subtype used for DDS-XRCE (micro-ROS) traffic.
pub const MSG_SUBTYPE_XRCE_DDS: u8 = 0;
/// Message subtype used for the `titan_link` protocol spoken between Rust firmware and `titan_bridge`.
///
/// This is an addition to the C specification, which only allocates subtype 0. Using a separate subtype keeps the
/// two protocols distinguishable if a micro-ROS agent and the bridge ever share a bus.
pub const MSG_SUBTYPE_TITAN_LINK: u8 = 1;

/// Utility channel carrying DShot commands to ESC boards.
pub const CHAN_THRUSTER_CMDS: u8 = 0;
/// Utility channel carrying the camera feed.
pub const CHAN_CAMERA_FEED: u8 = 12;
/// Utility channel carrying the remote TTY on Linux devices.
pub const CHAN_REMOTE_TTY: u8 = 13;
/// Utility channel carrying the register mapped control interface.
pub const CHAN_CONTROL_INTERFACE: u8 = 14;
/// Utility channel carrying client heartbeats.
pub const CHAN_HEARTBEAT: u8 = 15;

/// Interval between client heartbeats.
pub const HEARTBEAT_INTERVAL_MS: u64 = 500;

/// Client and bus ID assignments, mirroring `canmore/client_ids.h`. Client ID 0 is reserved.
pub mod client_ids {
    pub const BUS_ID_INTERNAL: u8 = 1;
    pub const CLIENT_ID_POWER_BOARD: u8 = 1;
    pub const CLIENT_ID_ESC_BOARD0: u8 = 2;
    pub const CLIENT_ID_ESC_BOARD1: u8 = 3;
    pub const CLIENT_ID_CAMERA_CAGE_BB: u8 = 4;
    pub const CLIENT_ID_ACTUATOR_BOARD: u8 = 5;
    pub const CLIENT_ID_LED_BOARD: u8 = 6;
    pub const CLIENT_ID_SBC_BOARD: u8 = 7;
    pub const CLIENT_ID_IVC_BOARD: u8 = 8;

    pub const BUS_ID_EXTERNAL: u8 = 2;
    pub const CLIENT_ID_SMART_BATTERY_PORT: u8 = 1;
    pub const CLIENT_ID_SMART_BATTERY_STBD: u8 = 2;
    pub const CLIENT_ID_DOWNWARDS_CAMERA: u8 = 6;
}

/// A raw CAN or CAN FD frame.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Frame {
    /// The 11-bit or 29-bit identifier.
    pub id: u32,
    /// True for a 29-bit identifier.
    pub extended: bool,
    len: u8,
    data: [u8; MAX_FD_FRAME_SIZE],
}

impl Frame {
    /// Creates a frame. Returns `None` if `data` is longer than a CAN FD frame.
    pub fn new(id: u32, extended: bool, data: &[u8]) -> Option<Self> {
        if data.len() > MAX_FD_FRAME_SIZE {
            return None;
        }
        let mut frame = Frame { id, extended, len: data.len() as u8, data: [0; MAX_FD_FRAME_SIZE] };
        frame.data[..data.len()].copy_from_slice(data);
        Some(frame)
    }

    /// The frame payload.
    pub fn data(&self) -> &[u8] {
        &self.data[..self.len as usize]
    }

    /// True if the payload cannot be sent in a classic CAN frame.
    pub fn needs_fd(&self) -> bool {
        self.len as usize > MAX_FRAME_SIZE
    }
}

/// Mode reported in the heartbeat for the client's control interface.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum ControlInterfaceMode {
    None = 0b000,
    Normal = 0b001,
    Bootloader = 0b010,
    Linux = 0b011,
    BootDelay = 0b101,
}

/// The one byte heartbeat sent by every client on [`CHAN_HEARTBEAT`].
///
/// Layout (MSB first): `term_enabled | term_valid | mode[2:0] | error | cnt[1:0]`.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct Heartbeat {
    /// Mod-4 counter incremented on every heartbeat, used to detect dropped frames.
    pub cnt: u8,
    /// True if the client is in an error state.
    pub error: bool,
    /// Raw 3-bit control interface mode (see [`ControlInterfaceMode`]).
    pub mode: u8,
    /// True if `term_enabled` holds a valid value.
    pub term_valid: bool,
    /// True if the client's termination resistor is enabled.
    pub term_enabled: bool,
}

impl Heartbeat {
    pub fn to_byte(self) -> u8 {
        (self.cnt & 0b11)
            | (self.error as u8) << 2
            | (self.mode & 0b111) << 3
            | (self.term_valid as u8) << 6
            | (self.term_enabled as u8) << 7
    }

    pub fn from_byte(byte: u8) -> Self {
        Heartbeat {
            cnt: byte & 0b11,
            error: byte & (1 << 2) != 0,
            mode: (byte >> 3) & 0b111,
            term_valid: byte & (1 << 6) != 0,
            term_enabled: byte & (1 << 7) != 0,
        }
    }

    /// The utility frame carrying this heartbeat from `client_id`.
    pub fn frame(self, client_id: u8) -> Frame {
        let id = id::util_id(client_id, Direction::ClientToAgent, CHAN_HEARTBEAT);
        Frame::new(id, false, &[self.to_byte()]).unwrap()
    }
}

/// Rounds a payload length up to the next length a CAN FD frame can carry.
pub fn fd_frame_len(len: usize) -> usize {
    fd_dlc_to_len(fd_len_to_dlc(len))
}

/// Converts a payload length to a CAN FD data length code.
pub fn fd_len_to_dlc(len: usize) -> u8 {
    match len {
        0..=8 => len as u8,
        9..=12 => 9,
        13..=16 => 10,
        17..=20 => 11,
        21..=24 => 12,
        25..=32 => 13,
        33..=48 => 14,
        _ => 15,
    }
}

/// Converts a CAN FD data length code to a payload length.
pub fn fd_dlc_to_len(dlc: u8) -> usize {
    const DLC_TO_LEN: [u8; 16] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64];
    DLC_TO_LEN[(dlc & 0xF) as usize] as usize
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn heartbeat_round_trip() {
        let hb = Heartbeat {
            cnt: 3,
            error: true,
            mode: ControlInterfaceMode::Normal as u8,
            term_valid: true,
            term_enabled: false,
        };
        assert_eq!(hb.to_byte(), 0b0100_1111);
        assert_eq!(Heartbeat::from_byte(hb.to_byte()), hb);
    }

    #[test]
    fn heartbeat_frame_id() {
        // CANMORE_CALC_UTIL_ID_C2A(8, 15) = 8 << 6 | 1 << 5 | 0 << 4 | 15
        let frame = Heartbeat::default().frame(client_ids::CLIENT_ID_IVC_BOARD);
        assert_eq!(frame.id, 0x22F);
        assert!(!frame.extended);
    }

    #[test]
    fn fd_lengths() {
        assert_eq!(fd_frame_len(8), 8);
        assert_eq!(fd_frame_len(9), 12);
        assert_eq!(fd_frame_len(33), 48);
        assert_eq!(fd_frame_len(64), 64);
        for dlc in 0..16 {
            assert_eq!(fd_len_to_dlc(fd_dlc_to_len(dlc)), dlc);
        }
    }
}
