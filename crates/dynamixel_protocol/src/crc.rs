//! The protocol 2.0 CRC: CRC-16 with polynomial 0x8005, initial value 0, no reflection and no final XOR
//! (CRC-16/BUYPASS, also called CRC-16/UMTS). The table matches `crc_table` in the C `dxl_packet.c`.

const POLY: u16 = 0x8005;

const fn make_table() -> [u16; 256] {
    let mut table = [0u16; 256];
    let mut i = 0;
    while i < 256 {
        let mut crc = (i as u16) << 8;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ POLY } else { crc << 1 };
            bit += 1;
        }
        table[i] = crc;
        i += 1;
    }
    table
}

static TABLE: [u16; 256] = make_table();

/// Running CRC over a packet, fed one byte at a time.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Crc16(u16);

impl Crc16 {
    pub const fn new() -> Self {
        Crc16(0)
    }

    /// Adds one byte (`update_dxl_crc`).
    pub fn update(&mut self, byte: u8) {
        let i = ((self.0 >> 8) as u8 ^ byte) as usize;
        self.0 = (self.0 << 8) ^ TABLE[i];
    }

    pub fn update_slice(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.update(b);
        }
    }

    pub fn value(self) -> u16 {
        self.0
    }
}

/// CRC of a whole buffer.
pub fn crc16(bytes: &[u8]) -> u16 {
    let mut crc = Crc16::new();
    crc.update_slice(bytes);
    crc.value()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn table_matches_c() {
        // First, second and last rows of crc_table in dxl_packet.c
        assert_eq!(&TABLE[..4], &[0x0000, 0x8005, 0x800F, 0x000A]);
        assert_eq!(&TABLE[252..], &[0x0208, 0x820D, 0x8207, 0x0202]);
    }

    #[test]
    fn check_value() {
        // Standard check value of CRC-16/BUYPASS
        assert_eq!(crc16(b"123456789"), 0xFEE8);
    }

    #[test]
    fn ping_packet_crc() {
        // Robotis e-manual, protocol 2.0 ping example: CRC bytes 0x19 0x4E (little endian)
        assert_eq!(crc16(&[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01]), 0x4E19);
    }
}
