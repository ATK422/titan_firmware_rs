//! SBC board (`SBCBoard`) messages.
//!
//! The C board is a skeleton that never brings up a ROS transport, so it has no topics of its own. Messages for
//! the BQ25730 charger, USB-PD controller and screen belong here once those are written.

use serde::{Deserialize, Serialize};

/// Messages from the bridge to the board.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HostToDevice {}

/// Messages from the board to the bridge.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeviceToHost {}
