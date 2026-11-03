//! RP2040 USB access: the bootrom's PICOBOOT interface and the pico-sdk reset interface, ported from `pico_usb`.
//!
//! Three kinds of devices are recognised (Raspberry Pi vendor ID 0x2e8a):
//!
//! * the bootrom (BOOTSEL mode, PID 0x0003), flashed directly over PICOBOOT
//! * pico-sdk applications with USB stdio (PID 0x000a), rebooted into the bootrom through the reset interface
//! * picoprobes (PID 0x000c), listed but not usable: the OpenOCD upload path is not ported
//!
//! Requires access to the USB devices (see `titan_firmware/tools/rp2040.rules`). libusb is built from source by
//! the `rusb` crate (`vendored` feature), so no system libusb is needed, only a C compiler.

pub mod picoboot;

use std::time::{Duration, Instant};

use anyhow::{Context as _, Result, anyhow, bail};
use rusb::{Context, Device as UsbDevice, DeviceHandle, Direction, Recipient, RequestType, UsbContext};

use crate::binary_info::{self, AppInfo};
use crate::device_map::DeviceMap;
use crate::flash::{self, FlashInterface, PAGE_SIZE, Page};
use picoboot::{FLASH_END, FLASH_START, Picoboot};

pub const PICO_VID: u16 = 0x2e8a;
pub const PICO_BOOTROM_PID: u16 = 0x0003;
pub const PICO_SERIAL_PID: u16 = 0x000a;
pub const PICO_PROBE_PID: u16 = 0x000c;

const RESET_INTERFACE_SUBCLASS: u8 = 0x00;
const RESET_INTERFACE_PROTOCOL: u8 = 0x01;
const RESET_REQUEST_BOOTSEL: u8 = 0x01;

/// Flash access through the bootrom, ported from `RP2040BootromInterface`.
pub struct BootromInterface {
    conn: Picoboot<Context>,
    flash_id: u64,
    flash_size: u32,
    /// Binary info of the image at the start of flash, and of the application after it if that is a bootloader.
    pub first_app: AppInfo,
    pub nested_app: AppInfo,
    /// Warnings found while connecting (unknown flash chip).
    pub warnings: Vec<String>,
}

impl BootromInterface {
    pub fn open(handle: DeviceHandle<Context>) -> Result<Self> {
        let conn = Picoboot::open(handle)?;
        let mut itf = BootromInterface {
            conn,
            flash_id: 0,
            flash_size: FLASH_END - FLASH_START,
            first_app: AppInfo::default(),
            nested_app: AppInfo::default(),
            warnings: Vec::new(),
        };
        if let Err(e) = itf.read_flash_ids() {
            itf.warnings.push(format!("Could not read flash ID: {e:#}"));
        }
        let (first, nested) = binary_info::extract_device_info(&mut itf);
        itf.first_app = first;
        itf.nested_app = nested;
        Ok(itf)
    }

    /// Runs the ID stub from the end of SRAM to read the flash unique ID and JEDEC ID.
    fn read_flash_ids(&mut self) -> Result<()> {
        let loc = picoboot::STUB_LOCATION;
        self.conn.write(loc, &picoboot::FLASH_GETID_BIN)?;
        self.conn.exec(loc)?;
        self.conn.enter_cmd_xip()?;
        let mut data = [0u8; 12];
        self.conn.read(loc + picoboot::FLASH_GETID_BIN.len() as u32, &mut data)?;
        self.flash_id = u64::from_le_bytes(data[..8].try_into().unwrap());
        let jedec_id = u32::from_le_bytes([data[8], data[9], data[10], 0]);
        let chip = DeviceMap::get().lookup_flash_chip(jedec_id);
        if chip.is_unknown {
            self.warnings.push(format!(
                "Unknown flash chip ID '{}' - please define in DeviceList.jsonc - Falling back to max capacity",
                chip.hex_id()
            ));
        }
        self.flash_size = chip.capacity;
        Ok(())
    }
}

impl FlashInterface for BootromInterface {
    fn erase_sector(&mut self, addr: u32) -> Result<()> {
        if !(FLASH_START..FLASH_END).contains(&addr) || !addr.is_multiple_of(flash::FLASH_ERASE_SIZE) {
            bail!("Invalid erase address 0x{addr:08x}");
        }
        self.conn.flash_erase(addr, flash::FLASH_ERASE_SIZE)
    }

    fn read_page(&mut self, addr: u32) -> Result<Page> {
        if addr < FLASH_START || addr + PAGE_SIZE as u32 > FLASH_END {
            bail!("Invalid read address 0x{addr:08x}");
        }
        let mut page = [0u8; PAGE_SIZE];
        self.conn.read(addr, &mut page)?;
        Ok(page)
    }

    fn write_page(&mut self, addr: u32, data: &Page) -> Result<()> {
        if !(FLASH_START..FLASH_END).contains(&addr) || !addr.is_multiple_of(PAGE_SIZE as u32) {
            bail!("Invalid write address 0x{addr:08x}");
        }
        self.conn.write(addr, data)
    }

    fn bootloader_size(&self) -> u32 {
        if self.first_app.is_bootloader { self.first_app.bl_app_base - flash::FLASH_BASE } else { 0 }
    }

    fn flash_id(&self) -> u64 {
        self.flash_id
    }

    fn flash_size(&self) -> u32 {
        self.flash_size
    }

    fn reboot(&mut self) -> Result<()> {
        // Mark the reset as a clean boot from the bootrom, then reboot into flash
        self.conn.write(picoboot::STUB_LOCATION, &picoboot::CFG_WATCHDOG_BIN)?;
        self.conn.exec(picoboot::STUB_LOCATION)?;
        self.conn.reboot(0, 0, 10)
    }

    fn warn_on_bootloader_overwrite(&self) -> bool {
        false
    }
}

/// An RP2040 in BOOTSEL mode.
pub struct BootromDevice {
    interface: Option<BootromInterface>,
    bus: u8,
    address: u8,
}

impl BootromDevice {
    pub fn open(device: &UsbDevice<Context>) -> Result<Self> {
        let interface = BootromInterface::open(device.open()?)?;
        Ok(BootromDevice { interface: Some(interface), bus: device.bus_number(), address: device.address() })
    }

    pub fn location(&self) -> (u8, u8) {
        (self.bus, self.address)
    }

    pub fn interface(&self) -> Option<&BootromInterface> {
        self.interface.as_ref()
    }
}

impl flash::Device for BootromDevice {
    fn interface(&self) -> String {
        "USB".into()
    }

    fn flash_id(&mut self) -> u64 {
        self.interface.as_ref().map_or(0, |i| i.flash_id)
    }

    fn supports_flash_interface(&self) -> bool {
        self.interface.is_some()
    }

    fn additional_info(&mut self) -> Vec<(String, String)> {
        let mut info = vec![("Mode".to_owned(), "Bootrom".to_owned())];
        if let Some(itf) = &self.interface {
            info.extend(binary_info::report_version_info(&itf.first_app, &itf.nested_app));
        }
        info
    }

    fn flash_interface(&mut self) -> Result<Box<dyn FlashInterface>> {
        let itf = self.interface.take().context("bootrom interface already in use")?;
        for warning in &itf.warnings {
            eprintln!("{warning}");
        }
        Ok(Box::new(itf))
    }
}

/// A pico-sdk application with USB stdio, which can be rebooted into the bootrom.
pub struct NormalDevice {
    handle: DeviceHandle<Context>,
    context: Context,
    flash_id: u64,
    reset_interface: Option<u8>,
    bus: u8,
    address: u8,
}

/// Decodes the pico-sdk USB serial string (the flash unique ID in hex, first byte first).
pub fn decode_serial(serial: &str) -> u64 {
    if serial.len() != 16 {
        return 0;
    }
    let bytes: Option<Vec<u8>> =
        (0..8).map(|i| serial.get(i * 2..i * 2 + 2).and_then(|b| u8::from_str_radix(b, 16).ok())).collect();
    bytes.map_or(0, |b| u64::from_le_bytes(b.try_into().unwrap()))
}

impl NormalDevice {
    pub fn open(context: &Context, device: &UsbDevice<Context>) -> Result<Self> {
        let handle = device.open()?;
        let desc = device.device_descriptor()?;
        let serial = desc
            .serial_number_string_index()
            .and_then(|i| handle.read_string_descriptor_ascii(i).ok())
            .unwrap_or_default();
        let config = device.active_config_descriptor()?;
        let reset_interface = config
            .interfaces()
            .filter_map(|i| i.descriptors().next())
            .find(|alt| {
                alt.class_code() == 0xFF
                    && alt.sub_class_code() == RESET_INTERFACE_SUBCLASS
                    && alt.protocol_code() == RESET_INTERFACE_PROTOCOL
            })
            .map(|alt| alt.interface_number());
        Ok(NormalDevice {
            handle,
            context: context.clone(),
            flash_id: decode_serial(&serial),
            reset_interface,
            bus: device.bus_number(),
            address: device.address(),
        })
    }

    /// Asks the application to reboot into the bootrom (PICOBOOT only, no mass storage) and reconnects to it.
    pub fn switch_to_bootrom(&mut self) -> Result<BootromInterface> {
        let itf = self.reset_interface.context("Device does not support switching to bootrom mode")?;
        self.handle.claim_interface(itf)?;
        let request_type = rusb::request_type(Direction::Out, RequestType::Class, Recipient::Interface);
        let disable_mass_storage = 1;
        // The device resets before answering, so the result is meaningless
        let _ = self.handle.write_control(
            request_type,
            RESET_REQUEST_BOOTSEL,
            disable_mass_storage,
            itf as u16,
            &[],
            Duration::from_millis(500),
        );

        let start = Instant::now();
        while start.elapsed() < Duration::from_secs(5) {
            std::thread::sleep(Duration::from_millis(100));
            if let Some(itf) = self.rediscover_bootrom() {
                return Ok(itf);
            }
        }
        Err(anyhow!("Failed to rediscover RP2040 in bootrom - This could be due to the watchdog timer"))
    }

    /// Finds the bootrom device with our flash ID that appeared on the same bus.
    fn rediscover_bootrom(&self) -> Option<BootromInterface> {
        let devices = self.context.devices().ok()?;
        for device in devices.iter() {
            if device.bus_number() != self.bus || device.address() == self.address {
                continue;
            }
            let Ok(desc) = device.device_descriptor() else { continue };
            if desc.vendor_id() != PICO_VID || desc.product_id() != PICO_BOOTROM_PID {
                continue;
            }
            if let Ok(itf) = device.open().map_err(anyhow::Error::from).and_then(BootromInterface::open)
                && (itf.flash_id == self.flash_id || self.flash_id == 0)
            {
                return Some(itf);
            }
        }
        None
    }
}

impl flash::Device for NormalDevice {
    fn interface(&self) -> String {
        "USB".into()
    }

    fn flash_id(&mut self) -> u64 {
        self.flash_id
    }

    fn supports_flash_interface(&self) -> bool {
        self.reset_interface.is_some()
    }

    fn additional_info(&mut self) -> Vec<(String, String)> {
        vec![("Mode".to_owned(), "Application".to_owned())]
    }

    fn flash_interface(&mut self) -> Result<Box<dyn FlashInterface>> {
        Ok(Box::new(self.switch_to_bootrom()?))
    }
}

/// A picoprobe. Listed so the user can see it, but flashing through it (OpenOCD) is not supported.
pub struct PicoprobeDevice {
    serial: String,
}

impl flash::Device for PicoprobeDevice {
    fn interface(&self) -> String {
        format!("Picoprobe {}", self.serial)
    }

    fn flash_id(&mut self) -> u64 {
        0
    }

    fn supports_flash_interface(&self) -> bool {
        false
    }

    fn additional_info(&mut self) -> Vec<(String, String)> {
        vec![("Mode".to_owned(), "Picoprobe (OpenOCD flashing not supported, use probe-rs)".to_owned())]
    }

    fn flash_interface(&mut self) -> Result<Box<dyn FlashInterface>> {
        bail!("Flashing through a picoprobe is not supported by this tool. Use `probe-rs` or `cargo run` instead.")
    }
}

/// Finds RP2040s on USB.
pub struct UsbDiscovery {
    context: Context,
}

impl UsbDiscovery {
    pub fn new() -> Result<Self> {
        Ok(UsbDiscovery { context: Context::new().context("initializing libusb")? })
    }

    /// Opens every RP2040 found. Errors for individual devices are printed and the device is skipped.
    pub fn discover(&self) -> Vec<Box<dyn flash::Device>> {
        let mut found: Vec<Box<dyn flash::Device>> = Vec::new();
        let Ok(devices) = self.context.devices() else { return found };
        for device in devices.iter() {
            let Ok(desc) = device.device_descriptor() else { continue };
            if desc.vendor_id() != PICO_VID {
                continue;
            }
            let result: Result<Option<Box<dyn flash::Device>>> = match desc.product_id() {
                PICO_BOOTROM_PID => BootromDevice::open(&device).map(|d| Some(Box::new(d) as _)),
                PICO_SERIAL_PID => NormalDevice::open(&self.context, &device).map(|d| Some(Box::new(d) as _)),
                PICO_PROBE_PID => (|| {
                    let handle = device.open()?;
                    let index = desc.serial_number_string_index().context("picoprobe has no serial number")?;
                    let serial = handle.read_string_descriptor_ascii(index)?;
                    Ok(Some(Box::new(PicoprobeDevice { serial }) as _))
                })(),
                _ => Ok(None),
            };
            match result {
                Ok(Some(dev)) => found.push(dev),
                Ok(None) => {}
                Err(e) => {
                    eprintln!(
                        "Error discovering usb device on Bus {} Addr {}\n  what():  {e:#}",
                        device.bus_number(),
                        device.address()
                    );
                    if e.downcast_ref::<rusb::Error>() == Some(&rusb::Error::Access) {
                        eprintln!("Ensure you have permissions to access the USB device (see tools/rp2040.rules)");
                    }
                    eprintln!();
                }
            }
        }
        found
    }
}

impl flash::Discovery for UsbDiscovery {
    fn discover_devices(&self) -> Vec<Box<dyn flash::Device>> {
        self.discover()
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn serial_decoding() {
        let serial = super::decode_serial("E46214B0630D2A32");
        assert_eq!(crate::device_map::hex_serial(serial), "E46214B0630D2A32");
        assert_eq!(super::decode_serial("E46214B0630D2A3"), 0);
        assert_eq!(super::decode_serial("E46214B0630D2AXX"), 0);
    }
}
