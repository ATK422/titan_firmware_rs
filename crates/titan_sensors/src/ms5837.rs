//! TE MS5837-02BA / MS5837-30BA pressure sensor math, from `driver_depth/src/ms5837.c` and `depth.c`.
//!
//! The sensor stores six calibration coefficients (C1-C6) in a PROM protected by a 4-bit CRC. A measurement is
//! two 24-bit ADC conversions: D1 (pressure) and D2 (temperature). [`compensate`] combines them with the
//! coefficients into a temperature compensated pressure, using the datasheet's first and second order formulas in
//! the integer form of the Blue Robotics Arduino library, which the C driver copied.

/// Sensor model. The two use different scaling and second order compensation.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Model {
    /// 0-2 bar, 0.01 mbar (1 Pa) resolution.
    Ms5837_02ba,
    /// 0-30 bar, 0.1 mbar (10 Pa) resolution.
    Ms5837_30ba,
}

/// Number of PROM words read from the sensor (word 0 holds the CRC and factory data, 1-6 hold C1-C6).
pub const PROM_WORDS: usize = 7;

/// Fluid density used to turn pressure into depth, in kg/m^3. Fresh water, as in the C driver.
pub const FLUID_DENSITY: f64 = 997.0;
/// Standard gravity in m/s^2.
pub const GRAVITY: f64 = 9.80665;

/// A compensated measurement.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Measurement {
    /// Absolute pressure in pascals.
    pub pressure_pa: i32,
    /// Temperature in hundredths of a degree Celsius.
    pub temperature_centi_c: i32,
}

/// The CRC stored in the top nibble of PROM word 0.
pub fn stored_crc(prom: &[u16; PROM_WORDS]) -> u8 {
    (prom[0] >> 12) as u8
}

/// Computes the PROM CRC-4 with the datasheet's algorithm.
///
/// The algorithm runs over 8 words with the CRC nibble and the (nonexistent) eighth word zeroed. The C version
/// zeroed them in the caller's array; this works on a copy.
pub fn crc4(prom: &[u16; PROM_WORDS]) -> u8 {
    let mut words = [0u16; 8];
    words[..PROM_WORDS].copy_from_slice(prom);
    words[0] &= 0x0FFF;

    let mut rem: u16 = 0;
    for cnt in 0..16 {
        let word = words[cnt >> 1];
        rem ^= if cnt % 2 == 1 { word & 0x00FF } else { word >> 8 };
        for _ in 0..8 {
            rem = if rem & 0x8000 != 0 { (rem << 1) ^ 0x3000 } else { rem << 1 };
        }
    }
    ((rem >> 12) & 0x000F) as u8
}

/// Checks the PROM CRC. On a mismatch returns `(stored, computed)`.
pub fn check_prom(prom: &[u16; PROM_WORDS]) -> Result<(), (u8, u8)> {
    let (stored, computed) = (stored_crc(prom), crc4(prom));
    if stored == computed { Ok(()) } else { Err((stored, computed)) }
}

/// Computes temperature and pressure from the PROM and the raw D1 (pressure) and D2 (temperature) conversions.
///
/// This follows the C integer arithmetic (truncating division, 64-bit intermediates, truncation to 32 bits)
/// except in one place: the 30BA high temperature term `2 * dT^2 / 2^37` squared `dT` in 32 bits, which
/// overflows above about 21 °C and made the term (nearly) zero. Here it's computed in 64 bits as the datasheet
/// intends. The term only shifts the reported temperature, by under 0.1 °C at 45 °C, and doesn't affect pressure.
pub fn compensate(model: Model, prom: &[u16; PROM_WORDS], d1: u32, d2: u32) -> Measurement {
    let c = |i: usize| prom[i] as i64;
    let d1 = d1 as i64;

    // First order
    let dt = d2 as i64 - c(5) * 256;
    let (sens, off) = match model {
        Model::Ms5837_02ba => (c(1) * 65536 + (c(3) * dt) / 128, c(2) * 131072 + (c(4) * dt) / 64),
        Model::Ms5837_30ba => (c(1) * 32768 + (c(3) * dt) / 256, c(2) * 65536 + (c(4) * dt) / 128),
    };
    let temp = (2000 + dt * c(6) / 8_388_608) as i32;

    // Second order. The C code truncated each term to 32 bits; within the sensor's range none of them overflow.
    let t = temp as i64;
    let (ti, offi, sensi) = match model {
        Model::Ms5837_02ba if temp / 100 < 20 => {
            (11 * dt * dt / 34_359_738_368, 31 * (t - 2000) * (t - 2000) / 8, 63 * (t - 2000) * (t - 2000) / 32)
        }
        Model::Ms5837_02ba => (0, 0, 0),
        Model::Ms5837_30ba if temp / 100 < 20 => {
            let mut offi = 3 * (t - 2000) * (t - 2000) / 2;
            let mut sensi = 5 * (t - 2000) * (t - 2000) / 8;
            if temp / 100 < -15 {
                offi += 7 * (t + 1500) * (t + 1500);
                sensi += 4 * (t + 1500) * (t + 1500);
            }
            (3 * dt * dt / 8_589_934_592, offi, sensi)
        }
        Model::Ms5837_30ba => (2 * dt * dt / 137_438_953_472, (t - 2000) * (t - 2000) / 16, 0),
    };
    let (ti, offi, sensi) = (ti as i32, offi as i32, sensi as i32);

    let off2 = off - offi as i64;
    let sens2 = sens - sensi as i64;
    let temperature_centi_c = temp - ti;
    let pressure_pa = match model {
        // 02BA counts in 0.01 mbar, which is already pascals
        Model::Ms5837_02ba => ((d1 * sens2 / 2_097_152 - off2) / 32768) as i32,
        // 30BA counts in 0.1 mbar
        Model::Ms5837_30ba => (((d1 * sens2 / 2_097_152 - off2) / 8192) as i32).wrapping_mul(10),
    };

    Measurement { pressure_pa, temperature_centi_c }
}

/// Depth in metres of water below the surface, given the absolute pressure and the pressure at the surface.
pub fn depth_m(pressure_pa: i32, surface_pressure_pa: i32) -> f64 {
    pressure_pa.wrapping_sub(surface_pressure_pa) as f64 / (FLUID_DENSITY * GRAVITY)
}

/// Averages the surface pressure while zeroing, like `depth_zero_read_cb` in C.
///
/// Feed it consecutive pressure readings. The first [`Zeroing::DISCARD`] readings let the sensor settle and are
/// ignored. The next one seeds the estimate and the following ones go through an exponential filter
/// (`surface = 0.7 * surface + 0.3 * pressure`). One reading after the last filtered one, [`Zeroing::feed`]
/// returns the surface pressure. That final reading is ignored, as it was in C.
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Zeroing {
    count: u32,
    surface_pa: i32,
}

impl Zeroing {
    /// Readings ignored while the sensor settles.
    pub const DISCARD: u32 = 20;
    /// Readings that go into the estimate.
    pub const AVERAGE: u32 = 20;
    /// Total readings [`Zeroing::feed`] consumes before it returns the surface pressure.
    pub const READINGS: u32 = Self::DISCARD + Self::AVERAGE + 1;

    pub const fn new() -> Self {
        Zeroing { count: 0, surface_pa: 0 }
    }

    /// Adds a reading. Returns the surface pressure once zeroing is complete.
    pub fn feed(&mut self, pressure_pa: i32) -> Option<i32> {
        self.count += 1;
        if self.count > Self::DISCARD + Self::AVERAGE {
            return Some(self.surface_pa);
        }
        if self.count == Self::DISCARD + 1 {
            self.surface_pa = pressure_pa;
        } else if self.count > Self::DISCARD + 1 {
            self.surface_pa = (self.surface_pa as f64 * 0.7 + pressure_pa as f64 * 0.3) as i32;
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// MS5837-30BA datasheet example coefficients, with factory ID 0x0340 and a valid CRC in word 0.
    const PROM_30BA: [u16; 7] = [0x9340, 34982, 36352, 20328, 22354, 26646, 26146];
    /// MS5837-02BA datasheet example coefficients, with factory ID 0x0340 and a valid CRC in word 0.
    const PROM_02BA: [u16; 7] = [0x3340, 46372, 43981, 29059, 27842, 31553, 28165];

    // Expected values below come from a separate transliteration of the C code with C integer semantics.

    #[test]
    fn crc_accepts_valid_prom() {
        assert_eq!(crc4(&PROM_30BA), 9);
        assert_eq!(crc4(&PROM_02BA), 3);
        assert_eq!(check_prom(&PROM_30BA), Ok(()));
        assert_eq!(check_prom(&PROM_02BA), Ok(()));
    }

    #[test]
    fn crc_ignores_stored_nibble_and_rejects_corruption() {
        let mut prom = PROM_30BA;
        prom[0] = (prom[0] & 0x0FFF) | (0x4 << 12);
        assert_eq!(crc4(&prom), 9);
        assert_eq!(check_prom(&prom), Err((4, 9)));

        let mut prom = PROM_30BA;
        prom[3] ^= 1;
        assert_eq!(check_prom(&prom), Err((9, 8)));

        // A bus that reads back all ones or all zeros must not pass
        assert!(check_prom(&[0xFFFF; 7]).is_err());
        assert_eq!(check_prom(&[0; 7]), Ok(()), "all zero PROM has CRC 0, same as the C code");
    }

    #[test]
    fn datasheet_examples() {
        // Datasheets: 30BA gives 3999.8 mbar, 02BA gives 1100.02 mbar, both at about 20 °C
        let m = compensate(Model::Ms5837_30ba, &PROM_30BA, 4_958_179, 6_815_414);
        assert_eq!(m, Measurement { pressure_pa: 399_980, temperature_centi_c: 1982 });
        let m = compensate(Model::Ms5837_02ba, &PROM_02BA, 6_465_444, 8_077_636);
        assert_eq!(m, Measurement { pressure_pa: 110_002, temperature_centi_c: 2000 });
    }

    #[test]
    fn second_order_branches_30ba() {
        let cases = [
            // very low temperature (below -15 °C)
            (6_000_000, 387_120, -795),
            // low temperature
            (6_500_000, 394_960, 963),
            // high temperature: the C code's 32-bit overflow gave 3180 and 4426 here
            (7_200_000, 406_230, 3178),
            (7_600_000, 412_970, 4418),
        ];
        for (d2, pressure_pa, temperature_centi_c) in cases {
            let m = compensate(Model::Ms5837_30ba, &PROM_30BA, 4_958_179, d2);
            assert_eq!(m, Measurement { pressure_pa, temperature_centi_c }, "d2 = {d2}");
        }
    }

    #[test]
    fn second_order_branches_02ba() {
        let cases = [(7_500_000, 105_081, -45), (8_300_000, 111_800, 2746)];
        for (d2, pressure_pa, temperature_centi_c) in cases {
            let m = compensate(Model::Ms5837_02ba, &PROM_02BA, 6_465_444, d2);
            assert_eq!(m, Measurement { pressure_pa, temperature_centi_c }, "d2 = {d2}");
        }
    }

    #[test]
    fn depth_from_pressure() {
        assert_eq!(depth_m(101_325, 101_325), 0.0);
        let one_metre = (FLUID_DENSITY * GRAVITY) as i32;
        assert!((depth_m(101_325 + one_metre, 101_325) - 1.0).abs() < 1e-3);
        assert!(depth_m(101_000, 101_325) < 0.0);
    }

    #[test]
    fn zeroing_discards_settling_readings() {
        let mut zero = Zeroing::new();
        let mut result = None;
        for n in 1..=Zeroing::READINGS {
            result = zero.feed(100_000 + n as i32 * 10);
            if n < Zeroing::READINGS {
                assert_eq!(result, None, "reading {n}");
            }
        }
        assert_eq!(result, Some(100_376));
    }

    #[test]
    fn zeroing_truncates_like_c() {
        // 0.7 * p + 0.3 * p lands just below p in floating point, and the C cast truncates, so a constant
        // pressure loses a pascal. Kept for parity; 1 Pa is 0.1 mm of water.
        let mut zero = Zeroing::new();
        let result = (0..Zeroing::READINGS).find_map(|_| zero.feed(99_123));
        assert_eq!(result, Some(99_122));
    }
}
