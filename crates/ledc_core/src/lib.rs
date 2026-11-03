//! Hardware independent logic of the Talos LED boards (`talos/LEDBoard` and `talos/LEDBoard2`).
//!
//! Both boards drive the aluminium LED module through two ST L99LD20 dual buck LED controllers on one SPI bus and
//! measure the module temperature with a thermistor. This crate holds everything that does not touch the RP2040:
//!
//! * [`l99ld20`]: the controller's SPI frame format (with its parity bit) and control/status register fields
//! * [`driver`]: register access over any `embedded-hal` SPI bus, shared by both firmwares
//! * [`pattern`]: the colour patterns of `LEDBoard` (solid, flashing, breathing, status flashes)
//! * [`board`]: the rest of `LEDBoard`'s `ledc_driver.c`: `command/led` handling, singleton flashes, depth and
//!   temperature based brightness, and the colour to buck duty mapping
//! * [`thermal`]: thermistor conversion and temperature derating

#![no_std]

pub mod board;
pub mod driver;
pub mod l99ld20;
pub mod pattern;
pub mod thermal;

pub use pattern::{Mode, Rgb};
