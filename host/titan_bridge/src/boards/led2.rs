//! LED board, second revision firmware (`talos/LEDBoard2`).
//!
//! The C `ros.c` has only the common topics (`heartbeat`, `state/firmware`, `state/kill`), so the board has no
//! topics of its own. It shares `talos/LEDBoard`'s client ID: bridge one or the other.

use canmore::client_ids;

use super::{BoardProfile, NoTopics, Transport};

pub const PROFILE: BoardProfile = BoardProfile {
    name: "led2",
    node_name: "led_board2",
    namespace: "talos",
    client_id: client_ids::CLIENT_ID_LED_BOARD,
    transport: Transport::Can { fd: true },
    create: |_, _, _| Ok(Box::new(NoTopics)),
};
