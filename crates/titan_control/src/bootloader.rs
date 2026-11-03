//! The bootloader register map (`bl_server.c`), the flash layout shared by the bootloader and the applications
//! linked for it, and the application check of `boot_app.c`.
//!
//! # Flash layout
//!
//! ```text
//! 0x10000000  bootloader boot2 (256 bytes, W25Q080)
//! 0x10000100  bootloader                                  BOOTLOADER_SIZE = 16 KiB in total
//! 0x10004000  application boot2 (256 bytes, CRC-32 checked by the bootloader)      = APP_BASE
//! 0x10004100  application vector table, then the rest of the application
//! ```
//!
//! This is the layout of `memmap_bootloader.ld` / `memmap_app.ld`, so C and Rust applications and bootloaders mix:
//! an application image (`*_ota.uf2`, or a Rust ELF linked with `TITAN_BOOTLOADER=1`) starts with its own boot2
//! at [`APP_BASE`]. The bootloader validates the application by the boot2 checksum ([`app_boot2_valid`]), calls
//! that boot2 to set up XIP and jumps to the vector table at `APP_BASE + 0x100`.
//!
//! # Entering the bootloader
//!
//! An application reboots into the bootloader by writing [`BOOTLOADER_MAGIC`] to watchdog scratch register 4 and
//! resetting through the watchdog (`safety_enter_bootloader`). Otherwise the bootloader reports
//! [`BootDelay`](canmore::ControlInterfaceMode::BootDelay) in its heartbeat for [`BOOT_DELAY_MS`] and stays only if
//! [`canmore::reg_mapped::BOOTLOADER_REQUEST`] arrives on the control interface, or if the application is invalid.

use canmore::reg_mapped::interface::bootloader::{
    self as itf, FLASH_BUFFER_SIZE, FLASH_ERASE_ADDR_ALIGN_MASK, FLASH_ERASE_SIZE, FLASH_READ_ADDR_ALIGN_MASK,
    FLASH_WRITE_ADDR_ALIGN_MASK, flash_control, gdb_stub, mcu_control,
};
use canmore::reg_mapped::{Access, MemoryPage, RegisterMap, ServerError};

use crate::{crc32, memory};

/// Start of the XIP flash window.
pub const FLASH_BASE: u32 = memory::XIP_BASE;
/// Flash size of every UWRT RP2040 board (W25Q16JV / W25Q080 class, 2 MiB).
pub const FLASH_SIZE: u32 = 2 * 1024 * 1024;
/// Flash reserved for the bootloader, including its boot2.
pub const BOOTLOADER_SIZE: u32 = 16 * 1024;
/// Where the application image (its boot2) starts.
pub const APP_BASE: u32 = FLASH_BASE + BOOTLOADER_SIZE;
/// Offset of the application vector table from [`APP_BASE`] (after the application boot2).
pub const APP_VECTOR_OFFSET: u32 = 0x100;
/// Size of a boot2 image; its last word is the CRC-32 of the rest.
pub const BOOT2_SIZE: usize = 256;

/// Watchdog scratch 4 value asking the bootloader to stay in bootloader mode (`safety_enter_bootloader`).
pub const BOOTLOADER_MAGIC: u32 = 0xB007_10AD;
/// Watchdog scratch 4 value the bootloader leaves when it boots the application, so a reset by the still running
/// bootloader watchdog can be told apart (`WATCHDOG_BOOTLOADER_NON_REBOOT_MAGIC`).
pub const NON_REBOOT_MAGIC: u32 = 0x3E2F_EA55;
/// How long the bootloader waits for [`canmore::reg_mapped::BOOTLOADER_REQUEST`] after boot.
pub const BOOT_DELAY_MS: u32 = 500;
/// How long the bootloader waits for the link before the boot delay (Ethernet only; CAN is always online).
pub const LINK_DELAY_MS: u32 = 5000;
/// The bootloader reboots if no request arrived for this long, so a board doesn't stay offline.
pub const BOOTLOADER_TIMEOUT_MS: u32 = 30_000;
/// Watchdog timeout while in the bootloader.
pub const WATCHDOG_TIMEOUT_MS: u32 = 3000;

/// Bootloader protocol version (`titan_generate_version_tag(titan_bootloader 1 0)`). The tools warn on others.
pub const VERSION_MAJOR: u32 = itf::EXPECTED_VERSION_MAJOR;
pub const VERSION_MINOR: u32 = itf::EXPECTED_VERSION_MINOR;

const _: () = assert!(FLASH_BUFFER_SIZE == 256 && FLASH_ERASE_SIZE == 4096);
const _: () = assert!((APP_BASE as usize).is_multiple_of(FLASH_ERASE_SIZE));

/// True if `page` (the first page of an image) holds a boot2 with a valid checksum.
pub fn app_boot2_valid(page: &[u8; BOOT2_SIZE]) -> bool {
    let expected = u32::from_le_bytes(page[BOOT2_SIZE - 4..].try_into().unwrap());
    crc32::compute(&page[..BOOT2_SIZE - 4]) == expected
}

/// True if `pkt` is the bootloader request sent to a device in boot delay.
pub fn is_bootloader_request(pkt: &[u8]) -> bool {
    pkt == canmore::reg_mapped::BOOTLOADER_REQUEST
}

/// Flash access for the bootloader map. Offsets are from [`FLASH_BASE`] and already validated.
pub trait FlashBackend {
    /// Reads 256 bytes at `offset`.
    fn read(&mut self, offset: u32, out: &mut [u8; FLASH_BUFFER_SIZE]);
    /// Programs 256 (erased) bytes at `offset`.
    fn program(&mut self, offset: u32, data: &[u8; FLASH_BUFFER_SIZE]);
    /// Erases the 4 KiB sector at `offset`.
    fn erase(&mut self, offset: u32);
    /// Reads the word at `addr` for the gdb stub (already range checked; may be a flash address).
    fn read_word(&mut self, addr: u32) -> u32;
    /// Writes the word at `addr` for the gdb stub (already range checked).
    fn write_word(&mut self, addr: u32, value: u32);
    /// `(pc, sp, lr)` of the stub.
    fn stub_registers(&mut self) -> (u32, u32, u32);
}

/// Fixed identification of the bootloader.
#[derive(Clone, Copy, Debug)]
pub struct BootloaderInfo {
    /// Version string page contents, NUL terminated.
    pub version_string: &'static [u8],
    pub major: u32,
    pub minor: u32,
    pub release_type: u32,
    pub flash_id: u64,
    pub flash_size: u32,
    pub app_base: u32,
}

/// State kept between requests.
pub struct BootloaderState {
    buffer: [u8; FLASH_BUFFER_SIZE],
    target_addr: u32,
    bl_write_key: u32,
    crc: u32,
    gdb_memory_data: u32,
    /// The agent asked for a reboot.
    pub reboot: bool,
    /// A flash command ran since the flag was last cleared (drives the status LED).
    pub flash_activity: bool,
}

impl Default for BootloaderState {
    fn default() -> Self {
        Self::new()
    }
}

impl BootloaderState {
    pub const fn new() -> Self {
        BootloaderState {
            buffer: [0; FLASH_BUFFER_SIZE],
            target_addr: 0,
            bl_write_key: 0,
            crc: 0,
            gdb_memory_data: 0,
            reboot: false,
            flash_activity: false,
        }
    }
}

/// The bootloader register map. Build one per request.
pub struct BootloaderMap<'a, F: FlashBackend + ?Sized> {
    pub info: &'a BootloaderInfo,
    pub flash: &'a mut F,
    pub state: &'a mut BootloaderState,
}

const ADDRESS: ServerError = ServerError::InvalidRegisterAddress;
const MODE: ServerError = ServerError::InvalidRegisterMode;
const DATA: ServerError = ServerError::InvalidData;

impl<F: FlashBackend + ?Sized> BootloaderMap<'_, F> {
    pub fn new<'a>(
        info: &'a BootloaderInfo,
        flash: &'a mut F,
        state: &'a mut BootloaderState,
    ) -> BootloaderMap<'a, F> {
        BootloaderMap { info, flash, state }
    }

    fn memory_page(&mut self, page: u8) -> Result<MemoryPage<'_>, ServerError> {
        match page {
            itf::VERSION_STRING_PAGE => Ok(MemoryPage::Bytes(Access::ReadOnly(self.info.version_string))),
            itf::FLASH_BUFFER_PAGE => Ok(MemoryPage::Bytes(Access::ReadWrite(&mut self.state.buffer))),
            _ => Err(ADDRESS),
        }
    }

    /// `flash_control_command_cb`.
    fn command(&mut self, command: u32) -> Result<(), ServerError> {
        let target = self.state.target_addr;
        let allow_bl_write = self.state.bl_write_key == flash_control::BL_WRITE_KEY_VALUE;
        let min_write = if allow_bl_write { FLASH_BASE } else { self.info.app_base };
        let flash_end = FLASH_BASE + self.info.flash_size;
        let max_page = flash_end - FLASH_BUFFER_SIZE as u32;
        let max_sector = flash_end - FLASH_ERASE_SIZE as u32;
        let check = |min: u32, max: u32, mask: u32| {
            if target < min || target > max || target & mask != 0 { Err(DATA) } else { Ok(target - FLASH_BASE) }
        };
        match command {
            flash_control::COMMAND_READ => {
                let offset = check(FLASH_BASE, max_page, FLASH_READ_ADDR_ALIGN_MASK)?;
                self.flash.read(offset, &mut self.state.buffer);
            }
            flash_control::COMMAND_WRITE => {
                let offset = check(min_write, max_page, FLASH_WRITE_ADDR_ALIGN_MASK)?;
                self.flash.program(offset, &self.state.buffer);
                self.state.flash_activity = true;
            }
            flash_control::COMMAND_ERASE => {
                let offset = check(min_write, max_sector, FLASH_ERASE_ADDR_ALIGN_MASK)?;
                self.flash.erase(offset);
                self.state.flash_activity = true;
            }
            flash_control::COMMAND_CRC => self.state.crc = crc32::compute(&self.state.buffer),
            _ => return Err(DATA),
        }
        Ok(())
    }
}

impl<F: FlashBackend + ?Sized> RegisterMap for BootloaderMap<'_, F> {
    fn read(&mut self, page: u8, offset: u8) -> Result<u32, ServerError> {
        match page {
            itf::MCU_CONTROL_PAGE => match offset {
                mcu_control::MAGIC => Ok(mcu_control::MAGIC_VALUE),
                mcu_control::MAJOR_VERSION => Ok(self.info.major),
                mcu_control::MINOR_VERSION => Ok(self.info.minor),
                mcu_control::RELEASE_TYPE => Ok(self.info.release_type),
                mcu_control::LOWER_FLASH_ID => Ok(self.info.flash_id as u32),
                mcu_control::UPPER_FLASH_ID => Ok((self.info.flash_id >> 32) as u32),
                mcu_control::REBOOT_MCU => Err(MODE),
                _ => Err(ADDRESS),
            },
            itf::GDB_STUB_PAGE => match offset {
                gdb_stub::MEMORY_DATA => Ok(self.state.gdb_memory_data),
                gdb_stub::PC_REGISTER => Ok(self.flash.stub_registers().0),
                gdb_stub::SP_REGISTER => Ok(self.flash.stub_registers().1),
                gdb_stub::LR_REGISTER => Ok(self.flash.stub_registers().2),
                gdb_stub::READ_WORD_ADDR | gdb_stub::WRITE_WORD_ADDR => Err(MODE),
                _ => Err(ADDRESS),
            },
            itf::FLASH_CONTROL_PAGE => match offset {
                flash_control::TARGET_ADDR => Ok(self.state.target_addr),
                flash_control::BL_WRITE_KEY => Ok(self.state.bl_write_key),
                flash_control::CRC => Ok(self.state.crc),
                flash_control::FLASH_SIZE => Ok(self.info.flash_size),
                flash_control::APP_BASE => Ok(self.info.app_base),
                flash_control::COMMAND => Err(MODE),
                _ => Err(ADDRESS),
            },
            _ => self.memory_page(page)?.read(offset),
        }
    }

    fn write(&mut self, page: u8, offset: u8, data: u32) -> Result<(), ServerError> {
        match page {
            itf::MCU_CONTROL_PAGE => match offset {
                mcu_control::REBOOT_MCU => {
                    self.state.reboot = true;
                    Ok(())
                }
                mcu_control::MAGIC..=mcu_control::UPPER_FLASH_ID => Err(MODE),
                _ => Err(ADDRESS),
            },
            itf::GDB_STUB_PAGE => match offset {
                gdb_stub::READ_WORD_ADDR => {
                    if !memory::readable(data, self.info.flash_size) {
                        return Err(DATA);
                    }
                    self.state.gdb_memory_data = self.flash.read_word(data);
                    Ok(())
                }
                gdb_stub::WRITE_WORD_ADDR => {
                    if !memory::writable(data) {
                        return Err(DATA);
                    }
                    self.flash.write_word(data, self.state.gdb_memory_data);
                    Ok(())
                }
                gdb_stub::MEMORY_DATA => {
                    self.state.gdb_memory_data = data;
                    Ok(())
                }
                gdb_stub::PC_REGISTER | gdb_stub::SP_REGISTER | gdb_stub::LR_REGISTER => Err(MODE),
                _ => Err(ADDRESS),
            },
            itf::FLASH_CONTROL_PAGE => match offset {
                flash_control::COMMAND => self.command(data),
                flash_control::TARGET_ADDR => {
                    self.state.target_addr = data;
                    Ok(())
                }
                flash_control::BL_WRITE_KEY => {
                    self.state.bl_write_key = data;
                    Ok(())
                }
                flash_control::CRC | flash_control::FLASH_SIZE | flash_control::APP_BASE => Err(MODE),
                _ => Err(ADDRESS),
            },
            _ => self.memory_page(page)?.write(offset, data),
        }
    }

    fn read_multiword(&mut self, page: u8, offset: u8, out: &mut [u8]) -> Result<(), ServerError> {
        self.memory_page(page)?.read_multiword(offset, out)
    }

    fn write_multiword(&mut self, page: u8, offset: u8, data: &[u8]) -> Result<(), ServerError> {
        self.memory_page(page)?.write_multiword(offset, data)
    }
}
