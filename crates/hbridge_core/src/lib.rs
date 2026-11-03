//! Hardware independent logic of the Lilboat H-bridge board (`lilboat/HBridgeBoard`), host-tested.
//!
//! * [`bridge`]: H-bridge PWM parameters, target clamping, slew limiting and the nFAULT wake check (`hbridge.c`)
//! * [`hiwonder`]: the Hiwonder bus servo packet format (`hiwonder_driver.c`)
//! * [`actuators`]: the claw servos' state and the ROS actuator commands (`actuator.c`, `ros_actuators.c`)
//! * [`timing`]: a periodic timer that notices missed runs (`timer_ready` in the C `main.c`)
//!
//! The C board has no motor current sensing: the CD74HC4051 multiplexer only carries the bridges' nFAULT lines,
//! which [`bridge::count_awake`] evaluates during the wake-up sequence.

#![no_std]

pub mod actuators;
pub mod bridge;
pub mod hiwonder;
pub mod timing;
