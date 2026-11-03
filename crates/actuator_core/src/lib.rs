//! Hardware independent logic of the Talos actuator board (`talos/Actuator`), tested on the host.
//!
//! * [`v2`]: the Dynamixel actuators (`actuators_v2`, the default build): torpedo/marker cam and claw servos
//! * [`v1`]: the original timed actuators (`actuators_v1`): timings, capacitor charge, claw state machine
//!
//! The states and errors are the `titan_link` types the board reports to the bridge.

#![cfg_attr(not(test), no_std)]

pub mod v1;
pub mod v2;

pub use titan_link::actuator::{ActuatorError, ClawState, DropperState, TorpedoState, TorpedoTiming};
