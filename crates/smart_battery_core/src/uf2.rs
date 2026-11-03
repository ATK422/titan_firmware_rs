//! The `id_gen/id.uf2` image of the C board, which programs the board serial number (and thereby the CANmore
//! client ID) into the last flash sector.
//!
//! The C repository ships one hand-edited UF2 file ("hex edit the byte at offset 0x20"). [`id_block`] builds the
//! same 512 byte UF2 block for any ID; the `smart_battery_id` tool writes it to a file for `picotool load` or drag
//! and drop onto the BOOTSEL drive.

/// XIP address of the ID (`0x101FF000`, the start of the last 4 KiB sector of the 2 MiB flash).
pub const ID_ADDRESS: u32 = 0x101F_F000;
/// RP2040 UF2 family ID.
pub const RP2040_FAMILY_ID: u32 = 0xE48B_FF56;

const MAGIC_START0: u32 = 0x0A32_4655;
const MAGIC_START1: u32 = 0x9E5D_5157;
const MAGIC_END: u32 = 0x0AB1_6F30;
/// The family ID field is present.
const FLAG_FAMILY_ID: u32 = 0x0000_2000;
const PAYLOAD_LEN: u32 = 256;

/// True for IDs the firmware accepts: an erased (0xFF) or zeroed byte means "unprogrammed".
pub fn valid_id(id: u8) -> bool {
    id != 0 && id != 0xFF
}

/// A single block UF2 image writing `id` followed by 255 zero bytes to [`ID_ADDRESS`].
pub fn id_block(id: u8) -> [u8; 512] {
    let mut block = [0u8; 512];
    let words = [MAGIC_START0, MAGIC_START1, FLAG_FAMILY_ID, ID_ADDRESS, PAYLOAD_LEN, 0, 1, RP2040_FAMILY_ID];
    for (i, w) in words.iter().enumerate() {
        block[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
    }
    block[32] = id;
    block[508..].copy_from_slice(&MAGIC_END.to_le_bytes());
    block
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_the_c_id_uf2() {
        // Header and trailer of titan_firmware/talos/SmartBattery/id_gen/id.uf2 (which holds ID 5)
        let header: [u8; 33] = [
            0x55, 0x46, 0x32, 0x0a, 0x57, 0x51, 0x5d, 0x9e, 0x00, 0x20, 0x00, 0x00, 0x00, 0xf0, 0x1f, 0x10, 0x00, 0x01,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x56, 0xff, 0x8b, 0xe4, 0x05,
        ];
        let block = id_block(5);
        assert_eq!(block[..33], header);
        assert!(block[33..508].iter().all(|&b| b == 0));
        assert_eq!(block[508..], [0x30, 0x6f, 0xb1, 0x0a]);
    }

    #[test]
    fn id_validity() {
        assert!(!valid_id(0));
        assert!(!valid_id(0xFF));
        assert!(valid_id(1));
        assert!(valid_id(2));
    }
}
