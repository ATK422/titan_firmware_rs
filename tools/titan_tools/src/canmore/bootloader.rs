//! Client for the CANmore bootloader, ported from `canmore_dbg_cpp/BootloaderClient.cpp`.

use anyhow::{Result, bail, ensure};
use canmore::ControlInterfaceMode;
use canmore::reg_mapped::interface::bootloader::{
    self as itf, EXPECTED_VERSION_MAJOR, EXPECTED_VERSION_MINOR, FLASH_BUFFER_SIZE, flash_control, gdb_stub,
    mcu_control,
};
use canmore::reg_mapped::interface::debug::release_type;

use super::gdb::GdbTarget;
use super::{RegClient, flash_id_from_words};
use crate::crc32;
use crate::flash::{FLASH_BASE, FlashInterface, PAGE_SIZE, Page};

const _: () = assert!(FLASH_BUFFER_SIZE == PAGE_SIZE);

/// Transient communication errors (bad cables) are retried this many times where retrying is harmless.
const CMD_RETRY_ATTEMPTS: usize = 3;

/// Client for the bootloader interface ([`ControlInterfaceMode::Bootloader`]). Implements [`FlashInterface`].
pub struct BootloaderClient {
    client: RegClient,
    flash_id: u64,
    flash_size: u32,
    bootloader_size: u32,
    /// Warnings about the bootloader version found while connecting.
    pub warnings: Vec<String>,
}

impl BootloaderClient {
    /// Connects: checks the magic and version, and reads the flash geometry.
    pub fn new(mut client: RegClient) -> Result<Self> {
        let page = itf::MCU_CONTROL_PAGE;
        let magic = client.read(page, mcu_control::MAGIC)?;
        if magic != mcu_control::MAGIC_VALUE {
            bail!("Unexpected bootloader magic 0x{magic:08x}");
        }

        let mut warnings = Vec::new();
        let major = client.read(page, mcu_control::MAJOR_VERSION)?;
        let minor = client.read(page, mcu_control::MINOR_VERSION)?;
        let release = client.read(page, mcu_control::RELEASE_TYPE)?;
        if major != EXPECTED_VERSION_MAJOR || minor != EXPECTED_VERSION_MINOR {
            warnings.push(format!(
                "Warning! Unexpected version {major}.{minor} - This program expects version \
                 {EXPECTED_VERSION_MAJOR}.{EXPECTED_VERSION_MINOR}"
            ));
        }
        match release {
            release_type::DEBUG => {
                warnings.push("Warning! Bootloader running debug firmware! This may not operate as expected".into())
            }
            release_type::DEV => warnings.push("Note: Target bootloader is a development version".into()),
            release_type::CLEAN | release_type::TAGGED => {}
            other => bail!("Invalid bootloader version type {other}"),
        }

        let lower = client.read(page, mcu_control::LOWER_FLASH_ID)?;
        let upper = client.read(page, mcu_control::UPPER_FLASH_ID)?;
        let flash_size = client.read(itf::FLASH_CONTROL_PAGE, flash_control::FLASH_SIZE)?;
        let app_base = client.read(itf::FLASH_CONTROL_PAGE, flash_control::APP_BASE)?;
        ensure!(app_base >= FLASH_BASE, "bootloader reported invalid application base 0x{app_base:08x}");

        // Lock the bootloader region again in case an earlier session unlocked it
        client.write(itf::FLASH_CONTROL_PAGE, flash_control::BL_WRITE_KEY, 0)?;

        Ok(BootloaderClient {
            client,
            flash_id: flash_id_from_words(lower, upper),
            flash_size,
            bootloader_size: app_base - FLASH_BASE,
            warnings,
        })
    }

    pub fn open(link: &super::Link, client_id: u8) -> Result<Self> {
        Self::new(RegClient::open(link, client_id, ControlInterfaceMode::Bootloader)?)
    }

    pub fn version(&mut self) -> Result<String> {
        self.client.read_string_page(itf::VERSION_STRING_PAGE)
    }

    fn check_addr(&self, addr: u32, align_mask: u32) -> Result<()> {
        ensure!(
            addr >= FLASH_BASE && addr - FLASH_BASE < self.flash_size && addr & align_mask == 0,
            "invalid flash address 0x{addr:08x}"
        );
        Ok(())
    }

    fn retry<T>(&mut self, mut f: impl FnMut(&mut RegClient) -> Result<T>) -> Result<T> {
        let mut attempt = 1;
        loop {
            match f(&mut self.client) {
                Err(_) if attempt < CMD_RETRY_ATTEMPTS => attempt += 1,
                result => return result,
            }
        }
    }
}

impl FlashInterface for BootloaderClient {
    fn erase_sector(&mut self, addr: u32) -> Result<()> {
        self.check_addr(addr, itf::FLASH_ERASE_ADDR_ALIGN_MASK)?;
        self.client.write(itf::FLASH_CONTROL_PAGE, flash_control::TARGET_ADDR, addr)?;
        self.client.write(itf::FLASH_CONTROL_PAGE, flash_control::COMMAND, flash_control::COMMAND_ERASE)
    }

    fn read_page(&mut self, addr: u32) -> Result<Page> {
        self.check_addr(addr, itf::FLASH_READ_ADDR_ALIGN_MASK)?;
        self.client.write(itf::FLASH_CONTROL_PAGE, flash_control::TARGET_ADDR, addr)?;
        self.client.write(itf::FLASH_CONTROL_PAGE, flash_control::COMMAND, flash_control::COMMAND_READ)?;
        let mut page = [0u8; PAGE_SIZE];
        self.client.read_bytes(itf::FLASH_BUFFER_PAGE, 0, &mut page)?;
        Ok(page)
    }

    fn write_page(&mut self, addr: u32, data: &Page) -> Result<()> {
        self.check_addr(addr, itf::FLASH_WRITE_ADDR_ALIGN_MASK)?;
        // Filling the buffer and setting the address can be retried...
        self.retry(|client| {
            client.write_bytes(itf::FLASH_BUFFER_PAGE, 0, data)?;
            client.write(itf::FLASH_CONTROL_PAGE, flash_control::TARGET_ADDR, addr)
        })?;
        // ...programming can't: if it fails once, give up
        self.client.write(itf::FLASH_CONTROL_PAGE, flash_control::COMMAND, flash_control::COMMAND_WRITE)
    }

    fn verify_page(&mut self, addr: u32, data: &Page) -> Result<bool> {
        self.check_addr(addr, itf::FLASH_READ_ADDR_ALIGN_MASK)?;
        let expected = crc32::compute(data);
        // Let the device read the page and CRC it, instead of transferring it back
        let actual = self.retry(|client| {
            client.write(itf::FLASH_CONTROL_PAGE, flash_control::TARGET_ADDR, addr)?;
            client.write(itf::FLASH_CONTROL_PAGE, flash_control::COMMAND, flash_control::COMMAND_READ)?;
            client.write(itf::FLASH_CONTROL_PAGE, flash_control::COMMAND, flash_control::COMMAND_CRC)?;
            client.read(itf::FLASH_CONTROL_PAGE, flash_control::CRC)
        })?;
        Ok(expected == actual)
    }

    fn bootloader_size(&self) -> u32 {
        self.bootloader_size
    }

    fn flash_id(&self) -> u64 {
        self.flash_id
    }

    fn flash_size(&self) -> u32 {
        self.flash_size
    }

    fn reboot(&mut self) -> Result<()> {
        self.client.write(itf::MCU_CONTROL_PAGE, mcu_control::REBOOT_MCU, 1)
    }

    fn warn_on_bootloader_overwrite(&self) -> bool {
        true
    }

    fn enable_bootloader_overwrite(&mut self) -> Result<()> {
        self.client.write(itf::FLASH_CONTROL_PAGE, flash_control::BL_WRITE_KEY, flash_control::BL_WRITE_KEY_VALUE)
    }
}

impl GdbTarget for BootloaderClient {
    fn read_memory(&mut self, addr: u32) -> Result<u32> {
        self.client.write(itf::GDB_STUB_PAGE, gdb_stub::READ_WORD_ADDR, addr)?;
        self.client.read(itf::GDB_STUB_PAGE, gdb_stub::MEMORY_DATA)
    }

    fn write_memory(&mut self, addr: u32, data: u32) -> Result<()> {
        self.client.write(itf::GDB_STUB_PAGE, gdb_stub::MEMORY_DATA, data)?;
        self.client.write(itf::GDB_STUB_PAGE, gdb_stub::WRITE_WORD_ADDR, addr)
    }

    fn stub_pc(&mut self) -> Result<u32> {
        self.client.read(itf::GDB_STUB_PAGE, gdb_stub::PC_REGISTER)
    }

    fn stub_sp(&mut self) -> Result<u32> {
        self.client.read(itf::GDB_STUB_PAGE, gdb_stub::SP_REGISTER)
    }

    fn stub_lr(&mut self) -> Result<u32> {
        self.client.read(itf::GDB_STUB_PAGE, gdb_stub::LR_REGISTER)
    }

    fn ping(&mut self) -> Result<()> {
        let magic = self.client.read(itf::MCU_CONTROL_PAGE, mcu_control::MAGIC)?;
        if magic != mcu_control::MAGIC_VALUE {
            bail!("Unexpected bootloader magic during ping");
        }
        Ok(())
    }
}
