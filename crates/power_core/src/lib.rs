//! Hardware independent logic of the power boards, shared by the Rust ports of:
//!
//! * `talos/PowerBoard` (`firmware/power_board`)
//! * `talos/PoacBoard` (`firmware/poac_board`), the power board with the torpedo/dropper servo added
//! * `lilboat/PoHalfBoard` (`firmware/po_half_board`)
//!
//! All three are the vehicle's kill switch source: they read the physical kill switch, take software kill commands
//! and publish the resulting kill state. They also report the battery and rail voltages and drive a fan or
//! solenoids. Everything here is plain logic with no hardware access, so it runs (and is tested) on the host:
//!
//! * [`electrical`]: ADC conversions, voltage dividers, rail mapping and the `ElectricalCommand` fan switch
//! * [`kill`]: software kill switch locking, physical kill notifications and kill edge detection
//! * [`solenoid`]: the PoHalf solenoid sequencer (`solenoid.c`)
//! * [`hiwonder`]: the Hiwonder bus servo protocol used by the Poac board
//! * [`actuator`]: the Poac servo state machine and torpedo/dropper logic (`actuator.c`, `ros_torp.c`)

#![no_std]

pub mod actuator;
pub mod electrical;
pub mod hiwonder;
pub mod kill;
pub mod solenoid;
