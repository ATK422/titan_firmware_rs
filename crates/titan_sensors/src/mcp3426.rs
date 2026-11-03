//! Microchip MCP3426 delta-sigma ADC math, from `driver_mcp3426/mcp3426.c`.
//!
//! The C driver runs the ADC in continuous mode at 16 bits (15 samples/s) and gain 1, stepping through the
//! channels. A read returns the output code (big endian) followed by the configuration register, whose `RDY` bit
//! is clear once the code holds a new conversion.

/// Internal reference voltage.
pub const VREF: f32 = 2.048;
/// Number of channel selections in the configuration register. The MCP3426 only bonds out channels 1 and 2;
/// the C driver cycles through all four anyway, so this does too.
pub const CHANNEL_COUNT: usize = 4;

/// `RDY` bit: written as 1 to start a conversion (ignored in continuous mode), read as 0 when the result is new.
const RDY: u8 = 1 << 7;

/// Conversion mode (`O/C` bit).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Mode {
    OneShot = 0,
    Continuous = 1,
}

/// Resolution / sample rate (`S1:S0`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SampleRate {
    /// 12 bits, 240 samples/s.
    Bits12 = 0,
    /// 14 bits, 60 samples/s.
    Bits14 = 1,
    /// 16 bits, 15 samples/s.
    Bits16 = 2,
}

/// PGA gain (`G1:G0`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Gain {
    X1 = 0,
    X2 = 1,
    X4 = 2,
    X8 = 3,
}

/// Builds a configuration byte (`mcp3426_config_reg` in C) with the `RDY` bit set.
///
/// `channel` is 0-based and must be below [`CHANNEL_COUNT`].
pub const fn config_byte(channel: u8, mode: Mode, rate: SampleRate, gain: Gain) -> u8 {
    assert!((channel as usize) < CHANNEL_COUNT);
    RDY | (channel << 5) | ((mode as u8) << 4) | ((rate as u8) << 2) | gain as u8
}

/// A decoded read: the channel the conversion belongs to and its output code.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Sample {
    /// 0-based channel from the returned configuration register.
    pub channel: u8,
    /// Signed 16-bit output code.
    pub code: i16,
}

/// Decodes a 3 byte read. Returns `None` while the conversion isn't ready (`RDY` set).
pub fn parse_read(buf: &[u8; 3]) -> Option<Sample> {
    let config = buf[2];
    if config & RDY != 0 {
        return None;
    }
    Some(Sample { channel: (config >> 5) & 0x03, code: i16::from_be_bytes([buf[0], buf[1]]) })
}

/// Converts a 16-bit, gain 1 output code to the voltage on the input (LSB = 2 * VREF / 2^16 = 62.5 uV).
///
/// The code is two's complement. The C driver read it as unsigned, so a slightly negative input (noise around
/// 0 V) came out as about 4.096 V; this returns the small negative voltage instead.
pub fn code_to_voltage(code: i16) -> f32 {
    code as f32 * (VREF * 2.0) / 65536.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn config_bytes_match_c_driver() {
        // C: (1 << 7) | (chan << 5) | (CONTINUOUS << 4) | (16_BIT << 2) | GAIN_X1
        let expected = [0x98, 0xB8, 0xD8, 0xF8];
        for (channel, &byte) in expected.iter().enumerate() {
            assert_eq!(config_byte(channel as u8, Mode::Continuous, SampleRate::Bits16, Gain::X1), byte);
        }
        assert_eq!(config_byte(1, Mode::OneShot, SampleRate::Bits12, Gain::X8), 0xA3);
    }

    #[test]
    fn parse_waits_for_ready() {
        assert_eq!(parse_read(&[0x12, 0x34, 0x98]), None);
        assert_eq!(parse_read(&[0x12, 0x34, 0x58]), Some(Sample { channel: 2, code: 0x1234 }));
        assert_eq!(parse_read(&[0xFF, 0xFF, 0x78]), Some(Sample { channel: 3, code: -1 }));
    }

    #[test]
    fn voltage_scaling() {
        assert_eq!(code_to_voltage(0), 0.0);
        assert_eq!(code_to_voltage(16384), 1.024);
        assert_eq!(code_to_voltage(i16::MAX), 2.048 - 0.0000625);
        assert_eq!(code_to_voltage(-1), -0.0000625);
        assert_eq!(code_to_voltage(i16::MIN), -2.048);
    }
}
