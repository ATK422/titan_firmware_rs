//! Register layouts of the control interfaces built on the register mapped protocol, ported from
//! `canmore/reg_mapped/interface/*.h`.
//!
//! Each module serves one [`ControlInterfaceMode`](crate::ControlInterfaceMode):
//!
//! * [`debug`]: [`Normal`](crate::ControlInterfaceMode::Normal), an application running on a microcontroller
//! * [`bootloader`]: [`Bootloader`](crate::ControlInterfaceMode::Bootloader), the CAN/Ethernet bootloader
//! * [`linux`]: [`Linux`](crate::ControlInterfaceMode::Linux), the debug daemon on a Linux machine

pub mod bootloader;
pub mod debug;
pub mod linux;
