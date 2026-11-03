//! Hardware independent math for the sensor drivers in `titan_rp2040::drivers`.
//!
//! The firmware drivers only move bytes over I2C and handle timing. Everything that turns those bytes into
//! physical units lives here, so it can be checked with `cargo test` on the host:
//!
//! * [`sht41`]: CRC-8 and the temperature/humidity conversions (`driver_sht41`)
//! * [`ms5837`]: PROM CRC-4, first and second order pressure/temperature compensation, surface zeroing and the
//!   pressure to depth conversion (`driver_depth`)
//! * [`mcp3426`]: configuration byte and code to voltage conversion (`driver_mcp3426`)

#![no_std]

pub mod mcp3426;
pub mod ms5837;
pub mod sht41;
