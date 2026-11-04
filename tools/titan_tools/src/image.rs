//! Firmware images to flash: UF2 files (ported from `upload_tool/UF2.cpp`) and ELF files.
//!
//! An image is a contiguous run of 256 byte pages starting at a page aligned flash address, plus the binary info of
//! each program in it (a bootloader image contains the bootloader followed by the application it boots).

use std::path::Path;

use anyhow::{Context, Result, bail, ensure};
use object::{Object, ObjectSegment};

use crate::binary_info::{self, AppInfo, PagedMemory};
use crate::flash::{FLASH_BASE, MAX_FLASH_SIZE, PAGE_SIZE, Page};

pub const UF2_MAGIC_START0: u32 = 0x0A32_4655;
pub const UF2_MAGIC_START1: u32 = 0x9E5D_5157;
pub const UF2_MAGIC_END: u32 = 0x0AB1_6F30;
pub const UF2_FLAG_FAMILY_ID_PRESENT: u32 = 0x0000_2000;
pub const RP2040_FAMILY_ID: u32 = 0xE48B_FF56;
pub const UF2_BLOCK_SIZE: usize = 512;

/// A firmware image ready to be flashed.
#[derive(Clone, Debug)]
pub struct FlashImage {
    base_address: u32,
    pages: Vec<Page>,
    /// Binary info of each program in the image, starting with the one at the base address.
    pub apps: Vec<AppInfo>,
    /// The board type the image was built for, or empty if it doesn't say.
    pub board_type: String,
}

impl FlashImage {
    /// Loads a UF2 or ELF file, detected from its contents.
    pub fn load(path: &Path) -> Result<Self> {
        let data = std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
        if data.starts_with(b"\x7fELF") {
            Self::from_elf(&data).with_context(|| format!("loading ELF {}", path.display()))
        } else {
            Self::from_uf2(&data).with_context(|| format!("loading UF2 {}", path.display()))
        }
    }

    /// Parses an RP2040 UF2 file. The blocks must be in order and cover a contiguous address range.
    pub fn from_uf2(data: &[u8]) -> Result<Self> {
        ensure!(!data.is_empty(), "empty UF2 file");
        ensure!(data.len().is_multiple_of(UF2_BLOCK_SIZE), "unexpected end of UF2 file");

        let mut pages = Vec::with_capacity(data.len() / UF2_BLOCK_SIZE);
        let mut base_address = 0;
        let mut num_blocks = 0;
        for (block_no, block) in data.as_chunks::<UF2_BLOCK_SIZE>().0.iter().enumerate() {
            let word = |i: usize| u32::from_le_bytes(block[i * 4..i * 4 + 4].try_into().unwrap());
            let (magic0, magic1, flags, target_addr, payload_size, this_block, total_blocks, family) =
                (word(0), word(1), word(2), word(3), word(4), word(5), word(6), word(7));
            let magic_end = word(127);

            ensure!(magic0 == UF2_MAGIC_START0, "invalid UF2 magic start0: {magic0:#010x}");
            ensure!(magic1 == UF2_MAGIC_START1, "invalid UF2 magic start1: {magic1:#010x}");
            ensure!(magic_end == UF2_MAGIC_END, "invalid UF2 magic end: {magic_end:#010x}");
            // The only flag allowed is the one marking file_size as the family ID
            ensure!(flags == UF2_FLAG_FAMILY_ID_PRESENT, "unexpected UF2 flags: {flags:#010x}");
            ensure!(family == RP2040_FAMILY_ID, "invalid family id: {family:#010x}");
            ensure!(payload_size == PAGE_SIZE as u32, "invalid payload size: {payload_size:#010x}");
            ensure!(target_addr % PAGE_SIZE as u32 == 0, "invalid address: {target_addr:#010x} unaligned");
            ensure!(
                (FLASH_BASE..=FLASH_BASE + MAX_FLASH_SIZE - PAGE_SIZE as u32).contains(&target_addr),
                "invalid address: {target_addr:#010x} out of range of flash"
            );
            ensure!(this_block < total_blocks, "invalid block no: {this_block:#x} >= num blocks {total_blocks:#x}");

            if block_no == 0 {
                base_address = target_addr;
                num_blocks = total_blocks;
            } else {
                ensure!(
                    total_blocks == num_blocks,
                    "invalid num blocks: {total_blocks:#x} != expected {num_blocks:#x}"
                );
            }
            ensure!(this_block as usize == block_no, "out of order UF2 block");
            ensure!(
                target_addr == base_address + (block_no * PAGE_SIZE) as u32,
                "non-contiguous UF2 address {target_addr:#010x}"
            );
            pages.push(block[32..32 + PAGE_SIZE].try_into().unwrap());
        }
        ensure!(pages.len() == num_blocks as usize, "expected additional UF2 blocks");

        Self::new(base_address, pages)
    }

    /// Loads the flash contents of an ELF file: every loadable segment whose physical (load) address is in flash.
    ///
    /// This is what `elf2uf2` does, so an ELF and the UF2 made from it give the same image. Gaps between segments
    /// are zero filled.
    pub fn from_elf(data: &[u8]) -> Result<Self> {
        use object::read::elf::{ElfFile32, ProgramHeader};

        // The RP2040 is a 32-bit little endian target
        let file = ElfFile32::<object::Endianness>::parse(data).context("parsing ELF (expected a 32-bit ARM ELF)")?;
        let endian = file.endian();
        let flash_end = FLASH_BASE as u64 + MAX_FLASH_SIZE as u64;
        let mut segments = Vec::new();
        for segment in file.segments() {
            let header = segment.elf_program_header();
            if header.p_type(endian) != object::elf::PT_LOAD {
                continue;
            }
            let data = segment.data().context("reading ELF segment")?;
            if data.is_empty() {
                continue;
            }
            // The physical (load) address is where the data sits in flash, even for .data copied to RAM at boot
            let addr = header.p_paddr(endian) as u64;
            if addr < FLASH_BASE as u64 || addr >= flash_end {
                continue;
            }
            ensure!(addr + data.len() as u64 <= flash_end, "ELF segment at {addr:#x} runs past the end of flash");
            segments.push((addr as u32, data));
        }
        ensure!(!segments.is_empty(), "ELF file has no data to load into flash");

        let start = segments.iter().map(|s| s.0).min().unwrap() & !(PAGE_SIZE as u32 - 1);
        let end = segments.iter().map(|s| s.0 + s.1.len() as u32).max().unwrap();
        let mut flat = vec![0u8; (end - start).next_multiple_of(PAGE_SIZE as u32) as usize];
        let mut written = vec![false; flat.len()];
        for (addr, data) in segments {
            let off = (addr - start) as usize;
            ensure!(!written[off..off + data.len()].iter().any(|&w| w), "overlapping ELF segments at {addr:#x}");
            flat[off..off + data.len()].copy_from_slice(data);
            written[off..off + data.len()].fill(true);
        }
        Self::new(start, flat.as_chunks::<PAGE_SIZE>().0.to_vec())
    }

    /// Builds an image from pages and extracts the binary info of every program in it.
    pub fn new(base_address: u32, pages: Vec<Page>) -> Result<Self> {
        ensure!(base_address.is_multiple_of(PAGE_SIZE as u32), "unaligned image base {base_address:#x}");
        ensure!(!pages.is_empty(), "empty image");
        let mut image = FlashImage { base_address, pages, apps: Vec::new(), board_type: String::new() };

        // Follow the chain of images: a bootloader reports the base of the application after it
        let mut search_base = 0;
        loop {
            let app = binary_info::extract_app_info(&mut image, search_base);
            let next = if app.is_bootloader { app.bl_app_base } else { 0 };
            if !app.board_type.is_empty() {
                if image.board_type.is_empty() {
                    image.board_type = app.board_type.clone();
                } else if image.board_type != app.board_type {
                    bail!("mismatched board types in image: {} and {}", image.board_type, app.board_type);
                }
            }
            image.apps.push(app);
            if next == 0 || next <= search_base.max(image.base_address) || image.apps.len() > 8 {
                break;
            }
            search_base = next;
        }
        Ok(image)
    }

    pub fn base_address(&self) -> u32 {
        self.base_address
    }

    /// Offset of the image from the start of flash.
    pub fn base_flash_offset(&self) -> u32 {
        self.base_address - FLASH_BASE
    }

    pub fn num_pages(&self) -> usize {
        self.pages.len()
    }

    pub fn size(&self) -> u32 {
        (self.pages.len() * PAGE_SIZE) as u32
    }

    pub fn page(&self, index: usize) -> &Page {
        &self.pages[index]
    }

    pub fn page_address(&self, index: usize) -> u32 {
        self.base_address + (index * PAGE_SIZE) as u32
    }

    /// The page at absolute flash address `addr`, if it is in the image.
    pub fn page_at(&self, addr: u32) -> Option<&Page> {
        if !addr.is_multiple_of(PAGE_SIZE as u32) || addr < self.base_address {
            return None;
        }
        self.pages.get(((addr - self.base_address) as usize) / PAGE_SIZE)
    }

    /// Encodes the image as an RP2040 UF2 file.
    pub fn to_uf2(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(self.pages.len() * UF2_BLOCK_SIZE);
        for (i, page) in self.pages.iter().enumerate() {
            let header = [
                UF2_MAGIC_START0,
                UF2_MAGIC_START1,
                UF2_FLAG_FAMILY_ID_PRESENT,
                self.page_address(i),
                PAGE_SIZE as u32,
                i as u32,
                self.pages.len() as u32,
                RP2040_FAMILY_ID,
            ];
            out.extend(header.iter().flat_map(|w| w.to_le_bytes()));
            out.extend_from_slice(page);
            out.resize(out.len() + (UF2_BLOCK_SIZE - 32 - PAGE_SIZE - 4), 0);
            out.extend_from_slice(&UF2_MAGIC_END.to_le_bytes());
        }
        out
    }
}

impl PagedMemory for FlashImage {
    fn base(&self) -> u32 {
        self.base_address
    }
    fn size(&self) -> u32 {
        FlashImage::size(self)
    }
    fn read_page(&mut self, addr: u32) -> Option<Page> {
        self.page_at(addr).copied()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::binary_info::builder::Builder;
    use crate::binary_info::{TAG_RASPBERRY_PI, TAG_UWRT, id};

    fn sample_image() -> FlashImage {
        let mut b = Builder::new(FLASH_BASE + 0x4000);
        b.str(TAG_RASPBERRY_PI, id::RP_PROGRAM_NAME, "ivc_board");
        b.str(TAG_RASPBERRY_PI, id::RP_PICO_BOARD, "talos/ivc_board");
        b.int(TAG_UWRT, id::UW_CLIENT_ID, 8);
        let mut data = b.finish();
        data.extend((0..1000u32).map(|i| i as u8));
        data.resize(data.len().next_multiple_of(PAGE_SIZE), 0);
        FlashImage::new(FLASH_BASE + 0x4000, data.as_chunks::<PAGE_SIZE>().0.to_vec()).unwrap()
    }

    #[test]
    fn uf2_round_trip() {
        let image = sample_image();
        let uf2 = image.to_uf2();
        assert_eq!(uf2.len(), image.num_pages() * UF2_BLOCK_SIZE);
        let parsed = FlashImage::from_uf2(&uf2).unwrap();
        assert_eq!(parsed.base_address(), FLASH_BASE + 0x4000);
        assert_eq!(parsed.base_flash_offset(), 0x4000);
        assert_eq!(parsed.pages, image.pages);
        assert_eq!(parsed.board_type, "talos/ivc_board");
        assert_eq!(parsed.apps.len(), 1);
        assert_eq!(parsed.apps[0].program_name, "ivc_board");
        assert_eq!(parsed.apps[0].client_ids, [8]);
    }

    #[test]
    fn uf2_validation() {
        let uf2 = sample_image().to_uf2();
        let corrupt = |offset: usize, value: u32| {
            let mut bad = uf2.clone();
            bad[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
            FlashImage::from_uf2(&bad).unwrap_err().to_string()
        };
        assert!(corrupt(0, 0).contains("magic start0"));
        assert!(corrupt(508, 0).contains("magic end"));
        assert!(corrupt(28, 0x1234).contains("family id"));
        assert!(corrupt(12, FLASH_BASE + 1).contains("unaligned"));
        assert!(corrupt(12, 0x2000_0000).contains("out of range"));
        // Second block pointing somewhere else
        assert!(corrupt(512 + 12, FLASH_BASE).contains("non-contiguous"));
        assert!(corrupt(512 + 20, 0).contains("out of order"));
        assert!(FlashImage::from_uf2(&uf2[..uf2.len() - 512]).unwrap_err().to_string().contains("additional"));
        assert!(FlashImage::from_uf2(&uf2[..100]).unwrap_err().to_string().contains("end of UF2"));
    }

    #[test]
    fn page_lookup() {
        let image = sample_image();
        assert_eq!(image.page_at(FLASH_BASE + 0x4000), Some(image.page(0)));
        assert_eq!(image.page_at(FLASH_BASE + 0x4100), Some(image.page(1)));
        assert_eq!(image.page_at(FLASH_BASE + 0x4001), None);
        assert_eq!(image.page_at(FLASH_BASE), None);
        assert_eq!(image.page_at(image.page_address(image.num_pages())), None);
    }

    /// The real Rust firmware ELF, if it has been built.
    #[test]
    fn rust_firmware_elf() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../firmware/target/thumbv6m-none-eabi/release/ivc_board");
        let Ok(data) = std::fs::read(path) else {
            eprintln!("skipping: {path} not built");
            return;
        };
        let image = FlashImage::from_elf(&data).unwrap();
        // Images are linked behind the bootloader: their boot2 at +16 KiB, followed by the vector table
        assert_eq!(image.base_address(), FLASH_BASE + 0x4000);
        let vectors = image.page(1);
        let initial_sp = u32::from_le_bytes(vectors[0..4].try_into().unwrap());
        let reset = u32::from_le_bytes(vectors[4..8].try_into().unwrap());
        assert!((0x2000_0000..=0x2004_2000).contains(&initial_sp), "initial SP {initial_sp:#x}");
        let base = image.base_address();
        assert!((base..base + image.size()).contains(&reset), "reset vector {reset:#x}");
        assert_eq!(reset & 1, 1, "reset vector must be a thumb address");
        // No pico-sdk binary info in Rust firmware
        assert_eq!(image.apps.len(), 1);
        assert!(image.board_type.is_empty());

        // Converting to UF2 and back gives the same image
        let reparsed = FlashImage::from_uf2(&image.to_uf2()).unwrap();
        assert_eq!(reparsed.pages, image.pages);
    }
}
