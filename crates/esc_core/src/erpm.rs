//! Decoding of the eRPM reply of bidirectional DShot, from `dshot.c`.
//!
//! After each frame the ESC answers with 21 bits at 5/4 the DShot rate. The bits are run-length encoded (a `1`
//! marks a level change), and once that is undone the 20 remaining bits are four 5-bit GCR symbols carrying a
//! 16-bit value: a 12-bit eRPM period (3-bit exponent, 9-bit mantissa, in microseconds) and a 4-bit checksum.

use crate::ESC_NUM_POLE_PAIRS;

/// GCR symbol to nibble; `-1` marks invalid symbols.
const GCR_LOOKUP: [i8; 32] = [
    -1, -1, -1, -1, // 0x00-0x03
    -1, -1, -1, -1, // 0x04-0x07
    -1, 0x9, 0xA, 0xB, // 0x08-0x0B
    -1, 0xD, 0xE, 0xF, // 0x0C-0x0F
    -1, -1, 0x2, 0x3, // 0x10-0x13
    -1, 0x5, 0x6, 0x7, // 0x14-0x17
    -1, 0x0, 0x8, 0x1, // 0x18-0x1B
    -1, 0x4, 0xC, -1, // 0x1C-0x1F
];

/// The eRPM period the ESC reports when the motor is stopped.
pub const PERIOD_STOPPED_US: u32 = 0x1FF << 7;

/// Decodes a 21-bit reply as received by the `bidir_dshot` PIO program into the eRPM period in microseconds.
/// Returns `None` on an invalid symbol or checksum.
pub fn decode_period(packet: u32) -> Option<u32> {
    if packet == 0 {
        return None;
    }
    let gcr = packet ^ (packet >> 1);
    let nibble = |shift: u32| {
        let value = GCR_LOOKUP[((gcr >> shift) & 0x1F) as usize];
        (value >= 0).then_some(value as u32)
    };
    let crc = nibble(0)?;
    let low = nibble(5)?;
    let mid = nibble(10)?;
    let high = nibble(15)?;
    if !(low ^ mid ^ high) & 0xF != crc {
        return None;
    }
    let data = (high << 8) | (mid << 4) | low;
    Some((data & 0x1FF) << (data >> 9))
}

/// Converts an eRPM period into mechanical RPM, negated if the thruster was last commanded in reverse (the ESC only
/// reports the magnitude). Returns `None` for a zero period.
pub fn rpm_from_period(period_us: u32, reversed: bool) -> Option<i32> {
    match period_us {
        0 => None,
        PERIOD_STOPPED_US => Some(0),
        _ => {
            let erpm = 60_000_000 / period_us;
            let rpm = (erpm / ESC_NUM_POLE_PAIRS as u32) as i32;
            Some(if reversed { -rpm } else { rpm })
        }
    }
}

/// [`decode_period`] followed by [`rpm_from_period`], like `dshot_get_rpm`.
pub fn decode_rpm(packet: u32, reversed: bool) -> Option<i32> {
    rpm_from_period(decode_period(packet)?, reversed)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The standard GCR table (nibble to 5-bit symbol) from the bidirectional DShot description.
    const GCR_ENCODE: [u32; 16] =
        [0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17, 0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F];

    /// Builds the 21-bit line pattern an ESC sends for a 12-bit period value (exponent << 9 | mantissa).
    fn encode_reply(value: u32) -> u32 {
        let crc = !(value ^ (value >> 4) ^ (value >> 8)) & 0xF;
        let word = (value << 4) | crc;
        let mut gcr = 0;
        for i in (0..4).rev() {
            gcr = (gcr << 5) | GCR_ENCODE[((word >> (i * 4)) & 0xF) as usize];
        }
        // Undo `p ^ (p >> 1)`: a 1 in the GCR stream is a level change on the line. The line starts low (start bit).
        let mut packet = 0u32;
        let mut level = 0;
        for bit in (0..20).rev() {
            level ^= (gcr >> bit) & 1;
            packet |= level << bit;
        }
        packet
    }

    #[test]
    fn decodes_every_valid_reply() {
        for value in 0..4096u32 {
            let expected = (value & 0x1FF) << (value >> 9);
            assert_eq!(decode_period(encode_reply(value)), Some(expected), "value {value:#x}");
        }
    }

    #[test]
    fn rejects_corruption() {
        assert_eq!(decode_period(0), None);
        let good = encode_reply(0x2A5);
        let mut rejected = 0;
        for bit in 0..20 {
            if decode_period(good ^ (1 << bit)).is_none() {
                rejected += 1;
            }
        }
        // A single flipped line bit changes two GCR bits; GCR plus the checksum catch all of them
        assert_eq!(rejected, 20);
    }

    #[test]
    fn rpm_conversion() {
        assert_eq!(rpm_from_period(0, false), None);
        assert_eq!(rpm_from_period(PERIOD_STOPPED_US, true), Some(0));
        // 1000 us eRPM period = 60000 eRPM = 8571 mechanical RPM
        assert_eq!(rpm_from_period(1000, false), Some(8571));
        assert_eq!(rpm_from_period(1000, true), Some(-8571));
        assert_eq!(rpm_from_period(1, false), Some(60_000_000 / 7));
        // Stopped reply from a real ESC: exponent 7, mantissa 0x1FF
        assert_eq!(decode_rpm(encode_reply(0xFFF), false), Some(0));
        assert_eq!(decode_rpm(encode_reply(0), false), None);
    }
}
