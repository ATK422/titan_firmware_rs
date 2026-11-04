//! Portable logic of the CANmore control interfaces served by the RP2040 firmware.
//!
//! * [`debug`]: the register map an application serves in [`Normal`](canmore::ControlInterfaceMode::Normal) mode
//!   (`titan_debug`): version, uptime, reboot, enter bootloader, safety status and faults, crash log, memory access
//!   for gdb and the remote commands. The hardware access goes through [`debug::DebugBackend`].
//! * [`remote_cmd`]: the remote command registry, argument parsing, response buffer and the built-in `help` command
//!   (`remote_cmd_handler.c`).
//! * [`bootloader`]: the register map of the CAN/Ethernet bootloader (`bl_server.c`), the flash layout shared by the
//!   bootloader and applications linked for it, and the application image check (`boot_app.c`). The flash access
//!   goes through [`bootloader::FlashBackend`].
//! * [`crash_log`]: the crash log kept across resets and the watchdog scratch register protocol behind it
//!   (`crash_reporting.c`).
//! * [`memory`]: the address ranges the gdb stub may touch.
//! * [`crc32`]: the CANmore CRC-32 (CRC-32/MPEG-2), also used by the RP2040 boot2 checksum.
//!
//! Everything here is `no_std`, allocation free and host-tested against [`canmore::reg_mapped::Client`], the same
//! client the host tools use.

#![no_std]

pub mod bootloader;
pub mod crash_log;
pub mod crc32;
pub mod debug;
pub mod memory;
pub mod remote_cmd;

#[cfg(test)]
mod tests;
