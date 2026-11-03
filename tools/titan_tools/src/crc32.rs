//! The CRC-32 used by the CANmore bootloader and Linux file transfer (`canmore/crc32.c`).
//!
//! Polynomial 0x04C11DB7, MSB first, initial value 0xFFFFFFFF, no final XOR (CRC-32/MPEG-2).

pub const INITIAL: u32 = 0xFFFF_FFFF;
const POLYNOMIAL: u32 = 0x04C1_1DB7;

pub fn update(mut crc: u32, data: &[u8]) -> u32 {
    for &byte in data {
        crc ^= (byte as u32) << 24;
        for _ in 0..8 {
            crc = if crc & (1 << 31) != 0 { (crc << 1) ^ POLYNOMIAL } else { crc << 1 };
        }
    }
    crc
}

pub fn compute(data: &[u8]) -> u32 {
    update(INITIAL, data)
}

#[cfg(test)]
mod tests {
    #[test]
    fn mpeg2_check_value() {
        assert_eq!(super::compute(b"123456789"), 0x0376_E6E7);
        let split = super::update(super::update(super::INITIAL, b"1234"), b"56789");
        assert_eq!(split, 0x0376_E6E7);
    }
}
