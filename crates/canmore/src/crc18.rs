//! CRC-18 used to validate re-assembled messages: polynomial 0x23979, initial value 0x3FFFF, no final XOR.

pub const INITIAL: u32 = 0x3FFFF;
pub const MASK: u32 = 0x3FFFF;
const POLY: u32 = 0x23979;

static TABLE: [u32; 256] = build_table();

const fn build_table() -> [u32; 256] {
    let mut table = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        let mut crc = (i as u32) << 10;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 0x20000 != 0 { (crc << 1) ^ POLY } else { crc << 1 };
            bit += 1;
        }
        table[i] = crc & MASK;
        i += 1;
    }
    table
}

/// Feeds `data` into a running CRC. Mask the result with [`MASK`] once all data is fed.
pub fn update(crc: u32, data: &[u8]) -> u32 {
    data.iter().fold(crc, |crc, &byte| (crc << 8) ^ TABLE[(((crc >> 10) ^ byte as u32) & 0xFF) as usize])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn table_matches_c_lookup() {
        // Spot checks against `crc18_lookup` in canmore/msg_encoding.c
        assert_eq!(TABLE[1], 0x23979);
        assert_eq!(TABLE[2], 0x24b8b);
        assert_eq!(TABLE[3], 0x072f2);
        assert_eq!(TABLE[16], 0x0f237);
        assert_eq!(TABLE[128], 0x1a8c1);
        assert_eq!(TABLE[200], 0x105ba);
        assert_eq!(TABLE[255], 0x2cf7e);
    }
}
