//! Extraction of pico-sdk `binary_info` from an image or a device, ported from `binary_info_cpp`.
//!
//! The pico-sdk embeds a small table near the start of every image: a marker, the address range of an array of
//! pointers to `binary_info` entries, and a copy table describing data that is copied from flash to RAM at boot
//! (entries may point into RAM, so reads of those addresses are redirected to the flash copy). The UWRT bootloader
//! adds its own `'U','W'` tagged entries, including the address of the application image that follows it.
//!
//! Rust firmware built with embassy doesn't carry this table, so nothing is found for it. That's fine: every caller
//! treats missing info as "unknown".

use std::collections::BTreeMap;

use crate::flash::{FLASH_BASE, FlashInterface, PAGE_SIZE, Page};

pub const MARKER_START: u32 = 0x7188_ebf2;
pub const MARKER_END: u32 = 0xe71a_a390;

const TYPE_LIST_ZERO_TERMINATED: u16 = 3;
const TYPE_ID_AND_INT: u16 = 5;
const TYPE_ID_AND_STRING: u16 = 6;
const TYPE_PINS_WITH_FUNC: u16 = 8;
const TYPE_PINS_WITH_NAME: u16 = 9;
const TYPE_NAMED_GROUP: u16 = 10;

const fn make_tag(c1: u8, c2: u8) -> u16 {
    (c2 as u16) << 8 | c1 as u16
}

pub const TAG_RASPBERRY_PI: u16 = make_tag(b'R', b'P');
pub const TAG_UWRT: u16 = make_tag(b'U', b'W');

pub mod id {
    pub const RP_PROGRAM_NAME: u32 = 0x02031c86;
    pub const RP_PROGRAM_VERSION_STRING: u32 = 0x11a9bc3a;
    pub const RP_PROGRAM_BUILD_DATE_STRING: u32 = 0x9da22254;
    pub const RP_BINARY_END: u32 = 0x68f465de;
    pub const RP_PROGRAM_URL: u32 = 0x1856239a;
    pub const RP_PROGRAM_DESCRIPTION: u32 = 0xb6a07c19;
    pub const RP_PROGRAM_FEATURE: u32 = 0xa1f4b453;
    pub const RP_PROGRAM_BUILD_ATTRIBUTE: u32 = 0x4275f0d3;
    pub const RP_SDK_VERSION: u32 = 0x5360b3ab;
    pub const RP_PICO_BOARD: u32 = 0xb63cffbb;
    pub const RP_BOOT2_NAME: u32 = 0x7f8882e1;

    pub const UW_BOOTLOADER_ENABLED: u32 = 0xa4ae7ab0;
    pub const UW_APPLICATION_BASE: u32 = 0x80244d0f;
    pub const UW_CLIENT_ID: u32 = 0x8fc386f3;
    pub const UW_DEVICE_IP_ADDRESS: u32 = 0x95b9a93f;
    pub const UW_AGENT_IP_ADDRESS: u32 = 0x851b754e;
    pub const UW_AGENT_PORT: u32 = 0xb7e0a019;
    pub const UW_VERSION_DATA: u32 = 0x41370cbc;
    pub const UW_MESSAGE: u32 = 0xcecaa2aa;
    pub const UW_FLAG: u32 = 0xc2caac84;
}

/// Pin function names by GPIO function select, as shown by picotool.
const PIN_FUNCTIONS: [[&str; 30]; 10] = {
    const fn alternating(a: &'static str, b: &'static str) -> [&'static str; 30] {
        let mut out = [""; 30];
        let mut i = 0;
        while i < 30 {
            out[i] = if i % 2 == 0 { a } else { b };
            i += 1;
        }
        out
    }
    const SPI: [&str; 30] = {
        const NAMES: [&str; 8] =
            ["SPI0 RX", "SPI0 CSn", "SPI0 SCK", "SPI0 TX", "SPI1 RX", "SPI1 CSn", "SPI1 SCK", "SPI1 TX"];
        let mut out = [""; 30];
        let mut i = 0;
        while i < 30 {
            // Pins 0-7 are SPI0, 8-15 SPI1, repeating every 16
            out[i] = NAMES[(i % 4) + if (i / 8) % 2 == 0 { 0 } else { 4 }];
            i += 1;
        }
        out
    };
    const UART: [&str; 30] = {
        const NAMES: [&str; 8] =
            ["UART0 TX", "UART0 RX", "UART0 CTS", "UART0 RTS", "UART1 TX", "UART1 RX", "UART1 CTS", "UART1 RTS"];
        // UART0 on 0-3, 12-15, 16-19, 28-29; UART1 on 4-11, 20-27
        const UNIT: [usize; 8] = [0, 1, 1, 0, 0, 1, 1, 0];
        let mut out = [""; 30];
        let mut i = 0;
        while i < 30 {
            out[i] = NAMES[(i % 4) + UNIT[i / 4] * 4];
            i += 1;
        }
        out
    };
    const I2C: [&str; 30] = {
        const NAMES: [&str; 4] = ["I2C0 SDA", "I2C0 SCL", "I2C1 SDA", "I2C1 SCL"];
        let mut out = [""; 30];
        let mut i = 0;
        while i < 30 {
            out[i] = NAMES[i % 4];
            i += 1;
        }
        out
    };
    const PWM: [&str; 30] = {
        const NAMES: [&str; 16] = [
            "PWM0 A", "PWM0 B", "PWM1 A", "PWM1 B", "PWM2 A", "PWM2 B", "PWM3 A", "PWM3 B", "PWM4 A", "PWM4 B",
            "PWM5 A", "PWM5 B", "PWM6 A", "PWM6 B", "PWM7 A", "PWM7 B",
        ];
        let mut out = [""; 30];
        let mut i = 0;
        while i < 30 {
            out[i] = NAMES[i % 16];
            i += 1;
        }
        out
    };
    const CLOCK: [&str; 30] = {
        let mut out = [""; 30];
        out[20] = "CLOCK GPIN0";
        out[21] = "CLOCK GPOUT0";
        out[22] = "CLOCK GPIN1";
        out[23] = "CLOCK GPOUT1";
        out[24] = "CLOCK GPOUT2";
        out[25] = "CLOCK GPOUT3";
        out
    };
    const USB: [&str; 30] = {
        const NAMES: [&str; 3] = ["USB OVCUR DET", "USB VBUS DET", "USB VBUS EN"];
        let mut out = [""; 30];
        let mut i = 0;
        while i < 30 {
            out[i] = NAMES[i % 3];
            i += 1;
        }
        out
    };
    [
        [""; 30],
        SPI,
        UART,
        I2C,
        PWM,
        alternating("SIO", "SIO"),
        alternating("PIO0", "PIO0"),
        alternating("PIO1", "PIO1"),
        CLOCK,
        USB,
    ]
};

/// Everything the tools care about from one image's binary info.
#[derive(Clone, Default, Debug, PartialEq, Eq)]
pub struct AppInfo {
    pub is_bootloader: bool,
    /// Address of the application image after the bootloader. Only valid if `is_bootloader`.
    pub bl_app_base: u32,

    pub pins: BTreeMap<u32, Vec<String>>,
    pub client_ids: Vec<u32>,
    /// Named feature groups: `(tag, id)` → `(label, flags)`.
    pub named_feature_groups: BTreeMap<(u16, u32), (String, u16)>,
    pub named_feature_group_values: BTreeMap<String, Vec<String>>,
    pub program_name: String,
    pub program_build_date: String,
    pub program_version: String,
    pub program_url: String,
    pub program_description: String,
    pub board_type: String,
    pub sdk_version: String,
    pub boot2_name: String,
    pub device_ip_address: String,
    pub agent_ip_address: String,
    pub message: String,
    pub program_features: Vec<String>,
    pub build_attributes: Vec<String>,
    pub binary_start: u32,
    pub binary_end: u32,
    pub agent_port: u32,
}

/// An address that isn't backed by the image or device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NotMapped;

/// Byte level read access to an image or device memory.
pub trait MemoryAccess {
    /// Fills `buf` from `addr`. With `zero_fill`, unmapped bytes read as 0 instead of failing.
    fn read(&mut self, addr: u32, buf: &mut [u8], zero_fill: bool) -> Result<(), NotMapped>;
    /// Address the image starts at.
    fn binary_start(&self) -> u32;
}

/// Memory access over a contiguous run of pages starting at `base` (an image, or flash read page by page).
pub trait PagedMemory {
    fn base(&self) -> u32;
    fn size(&self) -> u32;
    fn read_page(&mut self, addr: u32) -> Option<Page>;
}

impl<T: PagedMemory> MemoryAccess for T {
    fn read(&mut self, mut addr: u32, buf: &mut [u8], zero_fill: bool) -> Result<(), NotMapped> {
        let mut pos = 0;
        while pos < buf.len() {
            let page_off = (addr as usize) % PAGE_SIZE;
            let page_addr = addr - page_off as u32;
            let this_size = (PAGE_SIZE - page_off).min(buf.len() - pos);
            let in_range = page_addr >= self.base() && (page_addr as u64) < self.base() as u64 + self.size() as u64;
            match in_range.then(|| self.read_page(page_addr)).flatten() {
                Some(page) => buf[pos..pos + this_size].copy_from_slice(&page[page_off..page_off + this_size]),
                None if zero_fill => buf[pos..pos + this_size].fill(0),
                None => return Err(NotMapped),
            }
            pos += this_size;
            addr = addr.wrapping_add(this_size as u32);
        }
        Ok(())
    }

    fn binary_start(&self) -> u32 {
        self.base()
    }
}

/// Reads a device's flash through a [`FlashInterface`], caching pages.
pub struct DeviceMemory<'a> {
    interface: &'a mut dyn FlashInterface,
    base: u32,
    cache: BTreeMap<u32, Page>,
}

impl<'a> DeviceMemory<'a> {
    pub fn new(interface: &'a mut dyn FlashInterface) -> Self {
        DeviceMemory { interface, base: FLASH_BASE, cache: BTreeMap::new() }
    }
}

impl PagedMemory for DeviceMemory<'_> {
    fn base(&self) -> u32 {
        self.base
    }
    fn size(&self) -> u32 {
        self.interface.flash_size()
    }
    fn read_page(&mut self, addr: u32) -> Option<Page> {
        if let Some(page) = self.cache.get(&addr) {
            return Some(*page);
        }
        let page = self.interface.read_page(addr).ok()?;
        self.cache.insert(addr, page);
        Some(page)
    }
}

/// Redirects reads of RAM addresses to the flash data they were copied from at boot.
struct Remapped<'a> {
    inner: &'a mut dyn MemoryAccess,
    /// `(dest_start, dest_end, source)`, sorted by `dest_start`.
    map: Vec<(u32, u32, u32)>,
}

impl Remapped<'_> {
    fn read(&mut self, mut addr: u32, buf: &mut [u8], zero_fill: bool) -> Result<(), NotMapped> {
        let mut pos = 0;
        while pos < buf.len() {
            let remaining = buf.len() - pos;
            let (src, len) = match self.map.iter().find(|(from, to, _)| addr >= *from && addr < *to) {
                Some(&(from, to, src)) => (src + (addr - from), ((to - addr) as usize).min(remaining)),
                None => {
                    let next = self.map.iter().map(|m| m.0).filter(|&from| from > addr).min().unwrap_or(u32::MAX);
                    (addr, ((next - addr) as usize).min(remaining))
                }
            };
            self.inner.read(src, &mut buf[pos..pos + len], zero_fill)?;
            pos += len;
            addr = addr.wrapping_add(len as u32);
        }
        Ok(())
    }

    fn u32(&mut self, addr: u32) -> Result<u32, NotMapped> {
        let mut b = [0u8; 4];
        self.read(addr, &mut b, false)?;
        Ok(u32::from_le_bytes(b))
    }

    fn u16(&mut self, addr: u32) -> Result<u16, NotMapped> {
        let mut b = [0u8; 2];
        self.read(addr, &mut b, false)?;
        Ok(u16::from_le_bytes(b))
    }

    fn string(&mut self, addr: u32) -> Result<String, NotMapped> {
        let mut buf = [0u8; 512];
        self.read(addr, &mut buf, true)?;
        let len = buf.iter().position(|&b| b == 0).unwrap_or(buf.len());
        Ok(String::from_utf8_lossy(&buf[..len]).into_owned())
    }
}

/// Binary info header found in an image.
struct Header {
    entries: Vec<u32>,
    copy_map: Vec<(u32, u32, u32)>,
}

fn read_u32(access: &mut dyn MemoryAccess, addr: u32) -> Result<u32, NotMapped> {
    let mut b = [0u8; 4];
    access.read(addr, &mut b, false)?;
    Ok(u32::from_le_bytes(b))
}

/// Searches the first two pages after `base` (one when there's no boot2) for the binary info marker.
fn find_header(access: &mut dyn MemoryAccess, base: u32) -> Option<Header> {
    const SEARCH_WORDS: usize = 128;
    let mut raw = [0u8; SEARCH_WORDS * 4];
    access.read(base, &mut raw, false).ok()?;
    let words: Vec<u32> = raw.as_chunks::<4>().0.iter().map(|w| u32::from_le_bytes(*w)).collect();

    for i in 0..SEARCH_WORDS.saturating_sub(4) {
        if words[i] != MARKER_START || words[i + 4] != MARKER_END {
            continue;
        }
        let (from, to, copy_table) = (words[i + 1], words[i + 2], words[i + 3]);
        if to <= from || from % 4 != 0 || to % 4 != 0 {
            continue;
        }
        let parse = |access: &mut dyn MemoryAccess| -> Result<Header, NotMapped> {
            let mut raw = vec![0u8; (to - from) as usize];
            access.read(from, &mut raw, false)?;
            let entries = raw.as_chunks::<4>().0.iter().map(|w| u32::from_le_bytes(*w)).collect();
            let mut copy_map = Vec::new();
            let mut entry = copy_table;
            // Arbitrary limit, like picotool
            while copy_map.len() < 10 {
                let source = read_u32(access, entry)?;
                if source == 0 {
                    break;
                }
                let (dest_start, dest_end) = (read_u32(access, entry + 4)?, read_u32(access, entry + 8)?);
                if dest_end > dest_start {
                    copy_map.push((dest_start, dest_end, source));
                }
                entry += 12;
            }
            copy_map.sort_unstable();
            Ok(Header { entries, copy_map })
        };
        // An unmapped address means this wasn't really binary info, keep searching
        if let Ok(header) = parse(access) {
            return Some(header);
        }
    }
    None
}

/// What an entry visit reported.
enum Entry {
    Int { tag: u16, id: u32, value: u32 },
    Str { tag: u16, id: u32, value: String },
    Pin { pin: u32, name: String },
    NamedGroup { parent_tag: u16, parent_id: u32, group_tag: u16, group_id: u32, label: String, flags: u16 },
}

fn visit(mem: &mut Remapped, addr: u32, out: &mut Vec<Entry>, depth: u32) -> Result<(), NotMapped> {
    let ty = mem.u16(addr)?;
    let tag = mem.u16(addr + 2)?;
    match ty {
        TYPE_LIST_ZERO_TERMINATED if depth < 4 => {
            let mut list = mem.u32(addr + 4)?;
            loop {
                let entry = mem.u32(list)?;
                if entry == 0 {
                    break;
                }
                visit(mem, entry, out, depth + 1)?;
                list += 4;
            }
        }
        TYPE_ID_AND_INT => out.push(Entry::Int { tag, id: mem.u32(addr + 4)?, value: mem.u32(addr + 8)? }),
        TYPE_ID_AND_STRING => {
            let id = mem.u32(addr + 4)?;
            let value = mem.u32(addr + 8)?;
            out.push(Entry::Str { tag, id, value: mem.string(value)? });
        }
        TYPE_PINS_WITH_FUNC => {
            let encoding = mem.u32(addr + 4)?;
            let func = ((encoding >> 3) & 0xF) as usize;
            // Only the multi-pin encoding is decoded, like picotool
            if encoding & 7 == 2 && func < PIN_FUNCTIONS.len() {
                let mut mask = 0u32;
                let mut last = None;
                let mut work = encoding >> 7;
                for _ in 0..5 {
                    let cur = work & 0x1F;
                    mask |= 1 << cur;
                    if last == Some(cur) {
                        break;
                    }
                    last = Some(cur);
                    work >>= 5;
                }
                for pin in (0..30).filter(|p| mask & (1 << p) != 0) {
                    out.push(Entry::Pin { pin, name: PIN_FUNCTIONS[func][pin as usize].to_owned() });
                }
            }
        }
        TYPE_PINS_WITH_NAME => {
            let mask = mem.u32(addr + 4)?;
            let label_addr = mem.u32(addr + 8)?;
            let label = mem.string(label_addr)?;
            let mut names = label.split('|');
            for pin in (0..30).filter(|p| mask & (1 << p) != 0) {
                // Names are '|' separated, one per set pin in order; an empty name leaves the pin out
                match names.next() {
                    Some(name) if !name.is_empty() => out.push(Entry::Pin { pin, name: name.to_owned() }),
                    _ => {}
                }
            }
        }
        TYPE_NAMED_GROUP => {
            let label_addr = mem.u32(addr + 16)?;
            out.push(Entry::NamedGroup {
                parent_tag: tag,
                parent_id: mem.u32(addr + 4)?,
                flags: mem.u16(addr + 8)?,
                group_tag: mem.u16(addr + 10)?,
                group_id: mem.u32(addr + 12)?,
                label: mem.string(label_addr)?,
            })
        }
        _ => {}
    }
    Ok(())
}

fn ip_to_string(word: u32) -> String {
    let b = word.to_le_bytes();
    format!("{}.{}.{}.{}", b[0], b[1], b[2], b[3])
}

/// Extracts the binary info of the image at `base` (or at the memory's binary start if `base` is 0).
///
/// Whatever was decoded before an unreadable address is kept, like the C++ tools.
pub fn extract_app_info(access: &mut dyn MemoryAccess, base: u32) -> AppInfo {
    let mut app = AppInfo::default();
    let base = if base == 0 { access.binary_start() } else { base };
    app.binary_start = base;
    if base == 0 {
        return app;
    }
    let Some(header) = find_header(access, base) else {
        return app;
    };
    let mut mem = Remapped { inner: access, map: header.copy_map };
    let mut entries = Vec::new();
    for &addr in &header.entries {
        if visit(&mut mem, addr, &mut entries, 0).is_err() {
            break;
        }
    }

    // Named feature groups first, since their values are reported as plain strings
    for entry in &entries {
        if let Entry::NamedGroup { parent_tag, parent_id, group_tag, group_id, label, flags } = entry
            && *parent_tag == TAG_RASPBERRY_PI
            && *parent_id == id::RP_PROGRAM_FEATURE
        {
            app.named_feature_groups.insert((*group_tag, *group_id), (label.clone(), *flags));
        }
    }

    for entry in entries {
        match entry {
            Entry::Int { tag: TAG_UWRT, id, value } => match id {
                id::UW_BOOTLOADER_ENABLED => app.is_bootloader = value != 0,
                id::UW_APPLICATION_BASE => app.bl_app_base = value,
                id::UW_DEVICE_IP_ADDRESS => app.device_ip_address = ip_to_string(value),
                id::UW_AGENT_IP_ADDRESS => app.agent_ip_address = ip_to_string(value),
                id::UW_AGENT_PORT => app.agent_port = value,
                id::UW_CLIENT_ID => app.client_ids.push(value),
                _ => {}
            },
            Entry::Int { tag: TAG_RASPBERRY_PI, id: id::RP_BINARY_END, value } => app.binary_end = value,
            Entry::Str { tag, id, value } => {
                if let Some((label, _)) = app.named_feature_groups.get(&(tag, id)) {
                    app.named_feature_group_values.entry(label.clone()).or_default().push(value);
                    continue;
                }
                match (tag, id) {
                    (TAG_UWRT, id::UW_MESSAGE) => app.message = value,
                    (TAG_RASPBERRY_PI, id::RP_PROGRAM_NAME) => app.program_name = value,
                    (TAG_RASPBERRY_PI, id::RP_PROGRAM_VERSION_STRING) => app.program_version = value,
                    (TAG_RASPBERRY_PI, id::RP_PROGRAM_BUILD_DATE_STRING) => app.program_build_date = value,
                    (TAG_RASPBERRY_PI, id::RP_PROGRAM_URL) => app.program_url = value,
                    (TAG_RASPBERRY_PI, id::RP_PROGRAM_DESCRIPTION) => app.program_description = value,
                    (TAG_RASPBERRY_PI, id::RP_PROGRAM_FEATURE) => app.program_features.push(value),
                    (TAG_RASPBERRY_PI, id::RP_PROGRAM_BUILD_ATTRIBUTE) => app.build_attributes.push(value),
                    (TAG_RASPBERRY_PI, id::RP_PICO_BOARD) => app.board_type = value,
                    (TAG_RASPBERRY_PI, id::RP_SDK_VERSION) => app.sdk_version = value,
                    (TAG_RASPBERRY_PI, id::RP_BOOT2_NAME) => app.boot2_name = value,
                    _ => {}
                }
            }
            Entry::Pin { pin, name } => app.pins.entry(pin).or_default().push(name),
            _ => {}
        }
    }
    app
}

/// Extracts the info of the image at the start of a device's flash, and of the application after it if that image
/// is a bootloader.
pub fn extract_device_info(interface: &mut dyn FlashInterface) -> (AppInfo, AppInfo) {
    let mut mem = DeviceMemory::new(interface);
    let first = extract_app_info(&mut mem, FLASH_BASE);
    let nested = if first.is_bootloader { extract_app_info(&mut mem, first.bl_app_base) } else { AppInfo::default() };
    (first, nested)
}

fn name_and_version(app: &AppInfo) -> Option<(&'static str, String)> {
    if app.program_name.is_empty() {
        None
    } else if app.program_version.is_empty() {
        Some(("Name", app.program_name.clone()))
    } else {
        Some(("Version", format!("{} {}", app.program_name, app.program_version)))
    }
}

/// Version lines to show for a device, ported from `BinaryInfo::reportVersionInfo`.
pub fn report_version_info(first: &AppInfo, nested: &AppInfo) -> Vec<(String, String)> {
    let mut out = Vec::new();
    if first.is_bootloader {
        if let Some((kind, value)) = name_and_version(first) {
            out.push((format!("BL {kind}"), value));
        }
        if let Some((kind, value)) = name_and_version(nested) {
            out.push((format!("App {kind}"), value));
        }
    } else if let Some((kind, value)) = name_and_version(first) {
        out.push((format!("App {kind}"), value));
    }
    out
}

/// Builds binary info structures into a fake image. Used by tests here and in the tools.
#[doc(hidden)]
pub mod builder {
    use super::*;

    /// Lays out a minimal binary info table at the start of `image` (placed at `base`), pointing at the given
    /// entries. Returns the image.
    pub struct Builder {
        pub base: u32,
        pub data: Vec<u8>,
        entries: Vec<u32>,
    }

    impl Builder {
        /// Starts an image whose first 0x100 bytes are left for boot2 and the vector table, like an SDK image.
        pub fn new(base: u32) -> Self {
            Builder { base, data: vec![0u8; 0x200], entries: Vec::new() }
        }

        fn push(&mut self, bytes: &[u8]) -> u32 {
            while !self.data.len().is_multiple_of(4) {
                self.data.push(0);
            }
            let addr = self.base + self.data.len() as u32;
            self.data.extend_from_slice(bytes);
            addr
        }

        pub fn string(&mut self, s: &str) -> u32 {
            let mut bytes = s.as_bytes().to_vec();
            bytes.push(0);
            self.push(&bytes)
        }

        pub fn int(&mut self, tag: u16, id: u32, value: u32) {
            let mut b = Vec::new();
            b.extend_from_slice(&TYPE_ID_AND_INT.to_le_bytes());
            b.extend_from_slice(&tag.to_le_bytes());
            b.extend_from_slice(&id.to_le_bytes());
            b.extend_from_slice(&value.to_le_bytes());
            let addr = self.push(&b);
            self.entries.push(addr);
        }

        pub fn str(&mut self, tag: u16, id: u32, value: &str) {
            let s = self.string(value);
            let mut b = Vec::new();
            b.extend_from_slice(&TYPE_ID_AND_STRING.to_le_bytes());
            b.extend_from_slice(&tag.to_le_bytes());
            b.extend_from_slice(&id.to_le_bytes());
            b.extend_from_slice(&s.to_le_bytes());
            let addr = self.push(&b);
            self.entries.push(addr);
        }

        pub fn pins_with_name(&mut self, mask: u32, label: &str) {
            let s = self.string(label);
            let mut b = Vec::new();
            b.extend_from_slice(&TYPE_PINS_WITH_NAME.to_le_bytes());
            b.extend_from_slice(&TAG_RASPBERRY_PI.to_le_bytes());
            b.extend_from_slice(&mask.to_le_bytes());
            b.extend_from_slice(&s.to_le_bytes());
            let addr = self.push(&b);
            self.entries.push(addr);
        }

        /// Writes the header at offset 0x100 (where SDK images put it, after the vector table start) and returns
        /// the image bytes, padded to a whole page.
        pub fn finish(mut self) -> Vec<u8> {
            let entries: Vec<u8> = self.entries.iter().flat_map(|e| e.to_le_bytes()).collect();
            let from = self.push(&entries);
            let to = from + entries.len() as u32;
            let copy_table = self.push(&[0u8; 4]);
            let header = [MARKER_START, from, to, copy_table, MARKER_END];
            for (i, word) in header.iter().enumerate() {
                self.data[0x100 + i * 4..0x104 + i * 4].copy_from_slice(&word.to_le_bytes());
            }
            while !self.data.len().is_multiple_of(PAGE_SIZE) {
                self.data.push(0);
            }
            self.data
        }
    }
}

#[cfg(test)]
mod tests {
    use super::builder::Builder;
    use super::*;

    struct Image {
        base: u32,
        data: Vec<u8>,
    }

    impl PagedMemory for Image {
        fn base(&self) -> u32 {
            self.base
        }
        fn size(&self) -> u32 {
            self.data.len() as u32
        }
        fn read_page(&mut self, addr: u32) -> Option<Page> {
            let off = (addr - self.base) as usize;
            self.data.get(off..off + PAGE_SIZE).map(|s| s.try_into().unwrap())
        }
    }

    #[test]
    fn extracts_sdk_style_info() {
        let mut b = Builder::new(FLASH_BASE);
        b.str(TAG_RASPBERRY_PI, id::RP_PROGRAM_NAME, "power_board");
        b.str(TAG_RASPBERRY_PI, id::RP_PROGRAM_VERSION_STRING, "1.2.3");
        b.str(TAG_RASPBERRY_PI, id::RP_PICO_BOARD, "talos/power_board");
        b.str(TAG_RASPBERRY_PI, id::RP_PROGRAM_FEATURE, "CAN bus");
        b.str(TAG_RASPBERRY_PI, id::RP_PROGRAM_FEATURE, "Safety");
        b.int(TAG_UWRT, id::UW_CLIENT_ID, 1);
        b.int(TAG_UWRT, id::UW_AGENT_IP_ADDRESS, u32::from_le_bytes([192, 168, 1, 23]));
        b.int(TAG_RASPBERRY_PI, id::RP_BINARY_END, 0x1000_4000);
        b.pins_with_name(0b1011, "LED|BTN||");
        let mut image = Image { base: FLASH_BASE, data: b.finish() };

        let app = extract_app_info(&mut image, 0);
        assert_eq!(app.program_name, "power_board");
        assert_eq!(app.program_version, "1.2.3");
        assert_eq!(app.board_type, "talos/power_board");
        assert_eq!(app.program_features, ["CAN bus", "Safety"]);
        assert_eq!(app.client_ids, [1]);
        assert_eq!(app.agent_ip_address, "192.168.1.23");
        assert_eq!(app.binary_start, FLASH_BASE);
        assert_eq!(app.binary_end, 0x1000_4000);
        assert_eq!(app.pins.get(&0).unwrap(), &["LED"]);
        assert_eq!(app.pins.get(&1).unwrap(), &["BTN"]);
        assert!(!app.pins.contains_key(&3));
        assert!(!app.is_bootloader);

        let report = report_version_info(&app, &AppInfo::default());
        assert_eq!(report, [("App Version".to_string(), "power_board 1.2.3".to_string())]);
    }

    #[test]
    fn no_info_in_plain_image() {
        let mut image = Image { base: FLASH_BASE, data: vec![0xFF; 4096] };
        let app = extract_app_info(&mut image, 0);
        assert_eq!(app, AppInfo { binary_start: FLASH_BASE, ..Default::default() });
    }

    #[test]
    fn pin_function_table_matches_c() {
        assert_eq!(PIN_FUNCTIONS[1][9], "SPI1 CSn");
        assert_eq!(PIN_FUNCTIONS[1][29], "SPI1 CSn");
        assert_eq!(PIN_FUNCTIONS[2][4], "UART1 TX");
        assert_eq!(PIN_FUNCTIONS[2][12], "UART0 TX");
        assert_eq!(PIN_FUNCTIONS[2][29], "UART0 RX");
        assert_eq!(PIN_FUNCTIONS[3][29], "I2C0 SCL");
        assert_eq!(PIN_FUNCTIONS[4][17], "PWM0 B");
        assert_eq!(PIN_FUNCTIONS[9][5], "USB VBUS EN");
    }
}
