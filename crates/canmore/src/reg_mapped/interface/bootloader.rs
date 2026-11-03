//! Bootloader interface (`reg_mapped/interface/bootloader.h`), used to reflash a device in bootloader mode.
//!
//! ```text
//! 0x00: MCU control     (registers)
//! 0x01: Version string  RO memory
//! 0x02: GDB stub        (registers)
//! 0x08: Flash control   (registers)
//! 0x09: Flash buffer    RW memory, 256 bytes
//! ```
//!
//! To write flash: fill the flash buffer, write the target address, then write [`flash_control::COMMAND_WRITE`] to
//! the command register. The command register write is only acknowledged once the operation finished. To verify,
//! issue [`flash_control::COMMAND_READ`] then [`flash_control::COMMAND_CRC`] and compare the CRC-32 register.

pub const MCU_CONTROL_PAGE: u8 = 0x00;
pub const VERSION_STRING_PAGE: u8 = 0x01;
pub const GDB_STUB_PAGE: u8 = 0x02;
pub const FLASH_CONTROL_PAGE: u8 = 0x08;
pub const FLASH_BUFFER_PAGE: u8 = 0x09;

/// MCU control page: identification and reboot.
pub mod mcu_control {
    /// RO: always [`MAGIC_VALUE`].
    pub const MAGIC: u8 = 0x00;
    pub const MAGIC_VALUE: u32 = 0x10ad2040;
    /// RO: bootloader major version.
    pub const MAJOR_VERSION: u8 = 0x01;
    /// RO: bootloader minor version.
    pub const MINOR_VERSION: u8 = 0x02;
    /// RO: bootloader release type (see [`crate::reg_mapped::interface::debug::release_type`]).
    pub const RELEASE_TYPE: u8 = 0x03;
    /// RO: lower 32 bits of the flash unique ID.
    pub const LOWER_FLASH_ID: u8 = 0x04;
    /// RO: upper 32 bits of the flash unique ID.
    pub const UPPER_FLASH_ID: u8 = 0x05;
    /// WO: any write reboots the MCU.
    pub const REBOOT_MCU: u8 = 0x06;
}

/// GDB stub page: word access to memory for a remote debugger.
pub mod gdb_stub {
    /// WO: reads the word at the written address into [`MEMORY_DATA`]. Must be in flash, RAM or ROM.
    pub const READ_WORD_ADDR: u8 = 0x00;
    /// WO: writes [`MEMORY_DATA`] to the written address. Must be in RAM.
    pub const WRITE_WORD_ADDR: u8 = 0x01;
    /// RW: data for the read/write word registers.
    pub const MEMORY_DATA: u8 = 0x02;
    /// RO: PC of the debug stub.
    pub const PC_REGISTER: u8 = 0x03;
    /// RO: SP of the debug stub.
    pub const SP_REGISTER: u8 = 0x04;
    /// RO: LR of the debug stub.
    pub const LR_REGISTER: u8 = 0x05;
}

/// Flash control page: flash commands and geometry.
pub mod flash_control {
    /// WO: runs a command on the target address. Answers after the operation finished, with
    /// [`InvalidData`](crate::reg_mapped::ServerError::InvalidData) on a bad command or address.
    pub const COMMAND: u8 = 0x00;
    /// Reads 256 bytes at the (256 byte aligned) target address into the flash buffer.
    pub const COMMAND_READ: u32 = 0x00;
    /// Writes the flash buffer to the (256 byte aligned) target address.
    pub const COMMAND_WRITE: u32 = 0x01;
    /// Erases the 4 KiB sector at the (4 KiB aligned) target address.
    pub const COMMAND_ERASE: u32 = 0x02;
    /// Computes the CRC-32 of the flash buffer into [`CRC`].
    pub const COMMAND_CRC: u32 = 0x03;
    /// RW: target address for the command.
    pub const TARGET_ADDR: u8 = 0x01;
    /// RW: holds [`BL_WRITE_KEY_VALUE`] to allow erasing/writing the bootloader region.
    pub const BL_WRITE_KEY: u8 = 0x02;
    pub const BL_WRITE_KEY_VALUE: u32 = 0x55AA55AA;
    /// RO: result of [`COMMAND_CRC`].
    pub const CRC: u8 = 0x03;
    /// RO: flash size in bytes.
    pub const FLASH_SIZE: u8 = 0x04;
    /// RO: flash address where the bootloader expects the application.
    pub const APP_BASE: u8 = 0x05;
}

/// Size of the flash buffer page in bytes.
pub const FLASH_BUFFER_SIZE: usize = 256;
/// Flash erase sector size.
pub const FLASH_ERASE_SIZE: usize = 0x1000;
pub const FLASH_READ_ADDR_ALIGN_MASK: u32 = 0xFF;
pub const FLASH_WRITE_ADDR_ALIGN_MASK: u32 = 0xFF;
pub const FLASH_ERASE_ADDR_ALIGN_MASK: u32 = 0xFFF;

/// Bootloader version this interface description matches.
pub const EXPECTED_VERSION_MAJOR: u32 = 1;
pub const EXPECTED_VERSION_MINOR: u32 = 0;
