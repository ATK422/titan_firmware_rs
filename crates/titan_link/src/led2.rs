//! LED board, second revision firmware (`talos/LEDBoard2`) messages.
//!
//! The C firmware's `ros.c` has only the common topics (`heartbeat`, `state/firmware`, `state/kill`), so the board
//! has no messages of its own.

use serde::{Deserialize, Serialize};

/// Messages from the bridge to the board.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HostToDevice {}

/// Messages from the board to the bridge.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeviceToHost {}
