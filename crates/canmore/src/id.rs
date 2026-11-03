//! CAN identifier layouts.
//!
//! ```text
//! Standard:  | CLIENT ID (10:6) | T (5) | D (4) | NOC (3:0) |
//! Extended:  | CLIENT ID (28:24) | T (23) | D (22) | NOC (21:18) | EXTRA (17:0) |
//! First msg: EXTRA = | SINGLE (17) | SUBTYPE (16:11) | LENGTH (10:0) |
//! Last msg:  EXTRA = CRC-18
//! ```

const CLIENT_ID_MASK: u32 = 0x1F;
const NOC_MASK: u32 = 0xF;
const EXTRA_MASK: u32 = 0x3FFFF;

const STD_NOC_OFFSET: u32 = 0;
const STD_DIRECTION_OFFSET: u32 = 4;
const STD_TYPE_OFFSET: u32 = 5;
const STD_CLIENT_ID_OFFSET: u32 = 6;

const EXT_NOC_OFFSET: u32 = 18;
const EXT_DIRECTION_OFFSET: u32 = 22;
const EXT_TYPE_OFFSET: u32 = 23;
const EXT_CLIENT_ID_OFFSET: u32 = 24;

const MSG_LEN_MASK: u32 = 0x7FF;
const MSG_SUBTYPE_OFFSET: u32 = 11;
const MSG_SUBTYPE_MASK: u32 = 0x3F;
const MSG_SINGLE_OFFSET: u32 = 17;

/// Which way a frame travels.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum Direction {
    ClientToAgent = 0,
    AgentToClient = 1,
}

impl Direction {
    pub fn opposite(self) -> Self {
        match self {
            Direction::ClientToAgent => Direction::AgentToClient,
            Direction::AgentToClient => Direction::ClientToAgent,
        }
    }
}

/// CANmore frame type.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum FrameType {
    Msg = 0,
    Util = 1,
}

/// Builds a standard (11-bit) identifier.
pub const fn std_id(client_id: u8, frame_type: FrameType, direction: Direction, noc: u8) -> u32 {
    ((client_id as u32 & CLIENT_ID_MASK) << STD_CLIENT_ID_OFFSET)
        | ((frame_type as u32) << STD_TYPE_OFFSET)
        | ((direction as u32) << STD_DIRECTION_OFFSET)
        | ((noc as u32 & NOC_MASK) << STD_NOC_OFFSET)
}

/// Builds an extended (29-bit) identifier.
pub const fn ext_id(client_id: u8, frame_type: FrameType, direction: Direction, noc: u8, extra: u32) -> u32 {
    ((client_id as u32 & CLIENT_ID_MASK) << EXT_CLIENT_ID_OFFSET)
        | ((frame_type as u32) << EXT_TYPE_OFFSET)
        | ((direction as u32) << EXT_DIRECTION_OFFSET)
        | ((noc as u32 & NOC_MASK) << EXT_NOC_OFFSET)
        | (extra & EXTRA_MASK)
}

/// The extra field of the first frame in a message.
pub const fn msg_first_extra(single: bool, len: usize, subtype: u8) -> u32 {
    (len as u32 & MSG_LEN_MASK)
        | ((subtype as u32 & MSG_SUBTYPE_MASK) << MSG_SUBTYPE_OFFSET)
        | ((single as u32) << MSG_SINGLE_OFFSET)
}

/// Standard identifier of a utility frame.
pub const fn util_id(client_id: u8, direction: Direction, channel: u8) -> u32 {
    std_id(client_id, FrameType::Util, direction, channel)
}

/// Identifier mask matching client ID, type and direction of standard frames (ignores NOC).
pub const STD_CLIENT_TYPE_DIRECTION_MASK: u32 =
    (CLIENT_ID_MASK << STD_CLIENT_ID_OFFSET) | (1 << STD_TYPE_OFFSET) | (1 << STD_DIRECTION_OFFSET);

/// Identifier mask matching client ID, type and direction of extended frames (ignores NOC and extra).
pub const EXT_CLIENT_TYPE_DIRECTION_MASK: u32 =
    (CLIENT_ID_MASK << EXT_CLIENT_ID_OFFSET) | (1 << EXT_TYPE_OFFSET) | (1 << EXT_DIRECTION_OFFSET);

/// Decoded identifier fields common to standard and extended frames.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Fields {
    pub client_id: u8,
    pub frame_type: FrameType,
    pub direction: Direction,
    pub noc: u8,
    /// The 18-bit extra field. Always 0 for standard frames.
    pub extra: u32,
}

impl Fields {
    pub fn decode(id: u32, extended: bool) -> Self {
        let (client_off, type_off, dir_off, noc_off) = if extended {
            (EXT_CLIENT_ID_OFFSET, EXT_TYPE_OFFSET, EXT_DIRECTION_OFFSET, EXT_NOC_OFFSET)
        } else {
            (STD_CLIENT_ID_OFFSET, STD_TYPE_OFFSET, STD_DIRECTION_OFFSET, STD_NOC_OFFSET)
        };
        Fields {
            client_id: ((id >> client_off) & CLIENT_ID_MASK) as u8,
            frame_type: if (id >> type_off) & 1 == 0 { FrameType::Msg } else { FrameType::Util },
            direction: if (id >> dir_off) & 1 == 0 { Direction::ClientToAgent } else { Direction::AgentToClient },
            noc: ((id >> noc_off) & NOC_MASK) as u8,
            extra: if extended { id & EXTRA_MASK } else { 0 },
        }
    }

    /// Length field of a first message frame.
    pub fn msg_len(&self) -> usize {
        (self.extra & MSG_LEN_MASK) as usize
    }

    /// Subtype field of a first message frame.
    pub fn msg_subtype(&self) -> u8 {
        ((self.extra >> MSG_SUBTYPE_OFFSET) & MSG_SUBTYPE_MASK) as u8
    }

    /// Single flag of a first message frame.
    pub fn msg_single(&self) -> bool {
        (self.extra >> MSG_SINGLE_OFFSET) & 1 != 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn std_id_layout() {
        let id = std_id(8, FrameType::Util, Direction::AgentToClient, 14);
        assert_eq!(id, (8 << 6) | (1 << 5) | (1 << 4) | 14);
        let f = Fields::decode(id, false);
        assert_eq!(f.client_id, 8);
        assert_eq!(f.frame_type, FrameType::Util);
        assert_eq!(f.direction, Direction::AgentToClient);
        assert_eq!(f.noc, 14);
    }

    #[test]
    fn ext_first_id_layout() {
        let id = ext_id(31, FrameType::Msg, Direction::ClientToAgent, 0, msg_first_extra(true, 1024, 63));
        assert!(id < (1 << 29));
        let f = Fields::decode(id, true);
        assert_eq!(f.client_id, 31);
        assert_eq!(f.frame_type, FrameType::Msg);
        assert_eq!(f.noc, 0);
        assert!(f.msg_single());
        assert_eq!(f.msg_subtype(), 63);
        assert_eq!(f.msg_len(), 1024);
    }
}
