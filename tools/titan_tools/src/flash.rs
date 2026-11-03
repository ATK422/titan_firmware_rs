//! The flash interface every upload path implements, ported from `rp2040_flash_itf/RP2040FlashInterface.hpp`.

use anyhow::Result;

/// Start of the RP2040's XIP flash window.
pub const FLASH_BASE: u32 = 0x1000_0000;
/// Largest flash the RP2040 can address.
pub const MAX_FLASH_SIZE: u32 = 16 * 1024 * 1024;
/// Erase sector size of every flash chip used with the RP2040.
pub const FLASH_ERASE_SIZE: u32 = 4096;
/// Size of a flash program page, and of the data in a UF2 block.
pub const PAGE_SIZE: usize = 256;

/// One flash page.
pub type Page = [u8; PAGE_SIZE];

/// Low level access to the flash of an RP2040, over whichever link reached it (USB bootrom, CANmore bootloader).
///
/// All addresses are absolute (starting at [`FLASH_BASE`]); writes and reads are one aligned page.
pub trait FlashInterface {
    /// Erases the 4 KiB sector at `addr`.
    fn erase_sector(&mut self, addr: u32) -> Result<()>;
    /// Reads the page at `addr`.
    fn read_page(&mut self, addr: u32) -> Result<Page>;
    /// Programs the (erased) page at `addr`.
    fn write_page(&mut self, addr: u32, data: &Page) -> Result<()>;
    /// Checks the page at `addr` holds `data`.
    fn verify_page(&mut self, addr: u32, data: &Page) -> Result<bool> {
        Ok(&self.read_page(addr)? == data)
    }
    /// Size of the bootloader at the start of flash, or 0 if there is none.
    fn bootloader_size(&self) -> u32;
    /// The flash chip's unique ID, used as the board serial number. 0 if unknown.
    fn flash_id(&self) -> u64;
    /// Flash size in bytes.
    fn flash_size(&self) -> u32;
    /// Reboots into the freshly written image.
    fn reboot(&mut self) -> Result<()>;
    /// True if writing the bootloader region through this interface can brick the link (it is the bootloader).
    fn warn_on_bootloader_overwrite(&self) -> bool;
    /// Unlocks writes to the bootloader region. Only needed if
    /// [`warn_on_bootloader_overwrite`](Self::warn_on_bootloader_overwrite) is true.
    fn enable_bootloader_overwrite(&mut self) -> Result<()> {
        Ok(())
    }
}

/// Erases each sector the first time a page in it is written, so callers can write pages in any order.
pub struct EraseWrapper<'a> {
    interface: &'a mut dyn FlashInterface,
    erased: Vec<bool>,
}

impl<'a> EraseWrapper<'a> {
    pub fn new(interface: &'a mut dyn FlashInterface) -> Self {
        EraseWrapper { interface, erased: vec![false; (MAX_FLASH_SIZE / FLASH_ERASE_SIZE) as usize] }
    }

    pub fn write_page(&mut self, addr: u32, data: &Page) -> Result<()> {
        let sector = ((addr - FLASH_BASE) / FLASH_ERASE_SIZE) as usize;
        if !self.erased[sector] {
            self.erased[sector] = true;
            self.interface.erase_sector(FLASH_BASE + sector as u32 * FLASH_ERASE_SIZE)?;
        }
        self.interface.write_page(addr, data)
    }

    pub fn interface(&mut self) -> &mut dyn FlashInterface {
        self.interface
    }
}

/// Something that can report the RP2040s it finds (a CAN bus, the UDP network, USB).
pub trait Discovery {
    fn discover_devices(&self) -> Vec<Box<dyn Device>>;
}

/// An RP2040 found by a [`Discovery`], ported from `RP2040Device`.
pub trait Device {
    /// Where the device was found, for display ("can0 - Client 8", "USB").
    fn interface(&self) -> String;
    /// The device's flash unique ID, or 0 if it can't be read.
    fn flash_id(&mut self) -> u64;
    /// True if [`flash_interface`](Self::flash_interface) can be used in the current mode.
    fn supports_flash_interface(&self) -> bool;
    /// Extra `(label, value)` pairs to show when listing devices.
    fn additional_info(&mut self) -> Vec<(String, String)>;
    /// Puts the device into a mode where it can be flashed (rebooting it if needed) and returns the interface.
    fn flash_interface(&mut self) -> Result<Box<dyn FlashInterface>>;
}
