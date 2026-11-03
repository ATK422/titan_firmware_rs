//! SBC board (`SBCBoard`).
//!
//! The C board never started micro-ROS, so it had no node at all. Bridging it gives the common topics only
//! (`heartbeat`, `state/firmware`, `state/kill`).

use canmore::client_ids;

use super::{BoardProfile, NoTopics, Transport};

pub const PROFILE: BoardProfile = BoardProfile {
    name: "sbc",
    node_name: "sbc_board",
    namespace: "talos",
    client_id: client_ids::CLIENT_ID_SBC_BOARD,
    transport: Transport::Can { fd: true },
    create: |_, _, _| Ok(Box::new(NoTopics)),
};
