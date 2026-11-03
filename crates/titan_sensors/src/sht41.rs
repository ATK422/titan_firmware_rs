//! Sensirion SHT41 temperature/humidity sensor math, from `driver_sht41/sht41.c`.
//!
//! A measurement is 6 bytes: temperature MSB, LSB, CRC, humidity MSB, LSB, CRC. Each CRC covers the two bytes
//! before it.

/// CRC-8 polynomial x^8 + x^5 + x^4 + 1.
const CRC_POLY: u8 = 0x31;
/// CRC register initial value.
const CRC_INIT: u8 = 0xFF;

/// Raw 16-bit sensor words of one measurement.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RawMeasurement {
    pub temperature: u16,
    pub humidity: u16,
}

/// One of the CRCs in a measurement didn't match its data.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CrcMismatch;

/// Sensirion CRC-8 (polynomial 0x31, init 0xFF, no reflection, no final XOR).
pub fn crc8(data: &[u8]) -> u8 {
    let mut crc = CRC_INIT;
    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 { (crc << 1) ^ CRC_POLY } else { crc << 1 };
        }
    }
    crc
}

/// Checks both CRCs of a 6 byte measurement and extracts the raw words.
pub fn parse_measurement(buf: &[u8; 6]) -> Result<RawMeasurement, CrcMismatch> {
    if crc8(&buf[0..2]) != buf[2] || crc8(&buf[3..5]) != buf[5] {
        return Err(CrcMismatch);
    }
    Ok(RawMeasurement {
        temperature: u16::from_be_bytes([buf[0], buf[1]]),
        humidity: u16::from_be_bytes([buf[3], buf[4]]),
    })
}

/// Converts a raw temperature word to degrees Celsius.
pub fn temperature_c(raw: u16) -> f32 {
    -45.0 + 175.0 * raw as f32 / 65535.0
}

/// Converts a raw humidity word to %RH. Like the C driver this isn't clamped, so it can fall slightly outside
/// 0-100 %RH, as the datasheet allows.
pub fn humidity_rh(raw: u16) -> f32 {
    -6.0 + 125.0 * raw as f32 / 65535.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_matches_datasheet_example() {
        // SHT4x datasheet, section 4.4: CRC(0xBEEF) = 0x92
        assert_eq!(crc8(&[0xBE, 0xEF]), 0x92);
        assert_eq!(crc8(&[0x00, 0x00]), 0x81);
    }

    #[test]
    fn parses_valid_measurement() {
        let buf = [0x66, 0x66, crc8(&[0x66, 0x66]), 0x80, 0x00, crc8(&[0x80, 0x00])];
        assert_eq!(parse_measurement(&buf), Ok(RawMeasurement { temperature: 0x6666, humidity: 0x8000 }));
    }

    #[test]
    fn rejects_either_bad_crc() {
        let good = [0xBE, 0xEF, 0x92, 0xBE, 0xEF, 0x92];
        assert!(parse_measurement(&good).is_ok());
        for corrupt in [0, 1, 2, 3, 4, 5] {
            let mut buf = good;
            buf[corrupt] ^= 0x01;
            assert_eq!(parse_measurement(&buf), Err(CrcMismatch), "byte {corrupt}");
        }
    }

    #[test]
    fn conversions_cover_full_scale() {
        assert_eq!(temperature_c(0), -45.0);
        assert_eq!(temperature_c(u16::MAX), 130.0);
        assert!((temperature_c(0x6666) - 25.0).abs() < 0.01);
        assert_eq!(humidity_rh(0), -6.0);
        assert_eq!(humidity_rh(u16::MAX), 119.0);
        assert!((humidity_rh(0x8000) - 56.5).abs() < 0.01);
    }
}
