//! The CRC-32 of `canmore/crc32.c`: polynomial 0x04C11DB7, MSB first, initial value 0xFFFFFFFF, no final XOR
//! (CRC-32/MPEG-2). The RP2040 boot ROM checks boot2 with the same CRC, which is why the bootloader can use it to
//! validate an application's boot2.
//!
//! Bitwise to stay small: the bootloader has to fit in 16 KiB.

pub const INITIAL: u32 = 0xFFFF_FFFF;
const POLYNOMIAL: u32 = 0x04C1_1DB7;

/// Continues a CRC over `data`.
pub fn update(mut crc: u32, data: &[u8]) -> u32 {
    for &byte in data {
        crc ^= (byte as u32) << 24;
        for _ in 0..8 {
            crc = if crc & (1 << 31) != 0 { (crc << 1) ^ POLYNOMIAL } else { crc << 1 };
        }
    }
    crc
}

/// CRC of `data`.
pub fn compute(data: &[u8]) -> u32 {
    update(INITIAL, data)
}

#[cfg(test)]
mod tests {
    #[test]
    fn mpeg2_check_value() {
        assert_eq!(super::compute(b"123456789"), 0x0376_E6E7);
        assert_eq!(super::update(super::update(super::INITIAL, b"1234"), b"56789"), 0x0376_E6E7);
    }
}
