//! RP2040 address ranges the gdb stub pages may access, from `bindings/gdb_stub.h` and `bl_server.c`.
//!
//! Reads may touch flash (any XIP alias), ROM, RAM and the peripheral blocks; writes only RAM and peripherals. Both
//! must be word aligned. Peripheral reads can have side effects (FIFO pops), exactly as with the C firmware.

pub const ROM_BASE: u32 = 0x0000_0000;
pub const ROM_SIZE: u32 = 16 * 1024;
pub const XIP_BASE: u32 = 0x1000_0000;
pub const XIP_NOALLOC_BASE: u32 = 0x1100_0000;
pub const XIP_NOCACHE_BASE: u32 = 0x1200_0000;
pub const XIP_NOCACHE_NOALLOC_BASE: u32 = 0x1300_0000;
/// Striped SRAM, including SCRATCH_X and SCRATCH_Y.
pub const SRAM_BASE: u32 = 0x2000_0000;
pub const SRAM_END: u32 = 0x2004_2000;
/// Non-striped aliases of SRAM banks 0-3.
pub const SRAM0_BASE: u32 = 0x2100_0000;
pub const SRAM_NONSTRIPED_END: u32 = 0x2104_0000;
pub const APB_BASE: u32 = 0x4000_0000;
pub const APB_END: u32 = 0x4007_0000;
pub const AHB_BASE: u32 = 0x5000_0000;
pub const AHB_END: u32 = 0x5040_0000;
pub const SIO_BASE: u32 = 0xD000_0000;
pub const SIO_END: u32 = SIO_BASE + 0x180;
/// Cortex-M0+ system control space.
pub const SCS_BASE: u32 = 0xE000_E000;
pub const SCS_END: u32 = 0xE000_F000;

fn in_range(addr: u32, start: u32, end: u32) -> bool {
    (start..end).contains(&addr)
}

fn writable_range(addr: u32) -> bool {
    in_range(addr, SRAM_BASE, SRAM_END)
        || in_range(addr, SRAM0_BASE, SRAM_NONSTRIPED_END)
        || in_range(addr, APB_BASE, APB_END)
        || in_range(addr, AHB_BASE, AHB_END)
        || in_range(addr, SIO_BASE, SIO_END)
        || in_range(addr, SCS_BASE, SCS_END)
}

/// True if `addr` is in flash (any XIP alias) for a chip with `flash_size` bytes of flash.
pub fn is_flash(addr: u32, flash_size: u32) -> bool {
    [XIP_BASE, XIP_NOALLOC_BASE, XIP_NOCACHE_BASE, XIP_NOCACHE_NOALLOC_BASE]
        .iter()
        .any(|&base| addr >= base && addr - base < flash_size)
}

/// True if the gdb stub may read the word at `addr`.
pub fn readable(addr: u32, flash_size: u32) -> bool {
    addr.is_multiple_of(4) && (writable_range(addr) || is_flash(addr, flash_size) || addr < ROM_BASE + ROM_SIZE)
}

/// True if the gdb stub may write the word at `addr`.
pub fn writable(addr: u32) -> bool {
    addr.is_multiple_of(4) && writable_range(addr)
}

/// Offset into flash of a flash address in any XIP alias (only meaningful if [`is_flash`]).
pub fn flash_offset(addr: u32) -> u32 {
    addr & 0x00FF_FFFF
}

#[cfg(test)]
mod tests {
    use super::*;

    const FLASH: u32 = 2 * 1024 * 1024;

    #[test]
    fn ranges() {
        assert!(readable(0x2000_0000, FLASH));
        assert!(readable(0x2004_1FFC, FLASH));
        assert!(!readable(0x2004_2000, FLASH));
        assert!(!readable(0x2000_0001, FLASH));
        assert!(readable(0x1000_0000, FLASH));
        assert!(readable(0x131F_FFFC, FLASH));
        assert!(!readable(0x1020_0000, FLASH));
        assert!(readable(0x0000_3FFC, FLASH));
        assert!(!readable(0x0000_4000, FLASH));
        assert!(readable(0xD000_017C, FLASH));
        assert!(!readable(0xD000_0180, FLASH));
        assert!(readable(0xE000_ED08, FLASH));
        assert!(!readable(0xE000_0000, FLASH));

        assert!(writable(0x2000_1000));
        assert!(writable(0x4005_8000));
        assert!(!writable(0x1000_0000));
        assert!(!writable(0x0000_0100));
        assert!(!writable(0x2000_1002));
    }
}
