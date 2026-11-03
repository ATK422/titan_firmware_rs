//! The 64 tap integer FIR band-pass filters and the decimation by four (`fft/filter.c`, `process_capture_buffer`).
//!
//! The C firmware filtered every one of the 4096 samples of a capture and then kept every fourth output. The outputs
//! it throws away never feed back into the filter (it is FIR), so [`Fir::filter_decimate`] still pushes every
//! sample into the history but only computes the kept outputs: the same numbers for a quarter of the work.
//!
//! Like the C code, the comms filter and the pinger filters keep separate histories but share one write index, and
//! all six pinger filters share one history.

use crate::DECIMATE_BY;
use crate::pinger::PingerFreq;

/// Filter length (`N_FIR_TAPS`).
pub const TAPS: usize = 64;
/// The coefficients are scaled by 2^15 (`FIR_SCALE_SHIFT`).
pub const SCALE_SHIFT: u32 = 15;

/// One set of filter coefficients, scaled by 2^[`SCALE_SHIFT`].
pub type Coeffs = [i16; TAPS];

/// Comms band, 48.5 to 53.5 kHz (`fir_coeffs_int`).
pub static COMMS: Coeffs = [
    0, 3, 18, 34, 25, -33, -133, -225, -233, -103, 158, 453, 632, 555, 182, -384, -914, -1144, -902, -214, 683, 1412,
    1627, 1170, 172, -986, -1817, -1947, -1281, -66, 1209, 2016, 2016, 1209, -66, -1281, -1947, -1817, -986, 172, 1170,
    1627, 1412, 683, -214, -902, -1144, -914, -384, 182, 555, 632, 453, 158, -103, -233, -225, -133, -33, 25, 34, 18,
    3, 0,
];
/// Pinger, 20 kHz.
pub static PINGER_20K: Coeffs = [
    0, 1, 8, 27, 62, 111, 170, 230, 279, 304, 293, 233, 120, -47, -261, -508, -768, -1016, -1225, -1367, -1421, -1370,
    -1206, -932, -561, -118, 365, 852, 1302, 1678, 1949, 2091, 2091, 1949, 1678, 1302, 852, 365, -118, -561, -932,
    -1206, -1370, -1421, -1367, -1225, -1016, -768, -508, -261, -47, 120, 233, 293, 304, 279, 230, 170, 111, 62, 27, 8,
    1, 0,
];
/// Pinger, 25 kHz.
pub static PINGER_25K: Coeffs = [
    0, -5, -19, -38, -54, -54, -27, 36, 136, 266, 410, 542, 635, 661, 597, 431, 165, -182, -575, -968, -1309, -1546,
    -1635, -1551, -1286, -857, -305, 313, 928, 1469, 1870, 2084, 2084, 1870, 1469, 928, 313, -305, -857, -1286, -1551,
    -1635, -1546, -1309, -968, -575, -182, 165, 431, 597, 661, 635, 542, 410, 266, 136, 36, -27, -54, -54, -38, -19,
    -5, 0,
];
/// Pinger, 30 kHz.
pub static PINGER_30K: Coeffs = [
    0, 2, 2, -11, -45, -101, -168, -227, -253, -221, -114, 69, 309, 571, 802, 945, 953, 793, 465, 0, -540, -1070,
    -1496, -1734, -1727, -1453, -938, -251, 508, 1219, 1769, 2069, 2069, 1769, 1219, 508, -251, -938, -1453, -1727,
    -1734, -1496, -1070, -540, 0, 465, 793, 953, 945, 802, 571, 309, 69, -114, -221, -253, -227, -168, -101, -45, -11,
    2, 2, 0,
];
/// Pinger, 35 kHz.
pub static PINGER_35K: Coeffs = [
    0, 3, 17, 43, 68, 73, 37, -50, -184, -335, -459, -503, -425, -207, 132, 533, 907, 1155, 1190, 967, 497, -147, -842,
    -1439, -1795, -1812, -1462, -795, 64, 942, 1657, 2057, 2057, 1657, 942, 64, -795, -1462, -1812, -1795, -1439, -842,
    -147, 497, 967, 1190, 1155, 907, 533, 132, -207, -425, -503, -459, -335, -184, -50, 37, 73, 68, 43, 17, 3, 0,
];
/// Pinger, 37 kHz (`pinger_coeffs_int`, identical to `pinger_coeffs_37k_int`).
pub static PINGER_37K: Coeffs = [
    0, 0, 8, 33, 75, 116, 130, 89, -21, -191, -386, -545, -602, -504, -235, 172, 635, 1039, 1264, 1218, 870, 264, -481,
    -1198, -1709, -1873, -1624, -996, -115, 824, 1607, 2050, 2050, 1607, 824, -115, -996, -1624, -1873, -1709, -1198,
    -481, 264, 870, 1218, 1264, 1039, 635, 172, -235, -504, -602, -545, -386, -191, -21, 89, 130, 116, 75, 33, 8, 0, 0,
];
/// Pinger, 40 kHz.
pub static PINGER_40K: Coeffs = [
    0, -4, -12, -8, 24, 87, 166, 224, 219, 116, -86, -350, -598, -736, -683, -404, 66, 621, 1108, 1368, 1286, 838, 104,
    -740, -1470, -1872, -1811, -1275, -383, 641, 1529, 2042, 2042, 1529, 641, -383, -1275, -1811, -1872, -1470, -740,
    104, 838, 1286, 1368, 1108, 621, 66, -404, -683, -736, -598, -350, -86, 116, 219, 224, 166, 87, 24, -8, -12, -4, 0,
];

/// Which filter a capture goes through.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FilterSelect {
    /// The comms band ([`COMMS`]).
    Comms,
    /// The band of the selected pinger frequency.
    Pinger(PingerFreq),
}

/// Filter state carried from one capture to the next.
#[derive(Clone, Debug)]
pub struct Fir {
    comms_history: [i16; TAPS],
    pinger_history: [i16; TAPS],
    /// Shared write index (`fir_history_idx`).
    index: usize,
}

impl Default for Fir {
    fn default() -> Self {
        Self::new()
    }
}

impl Fir {
    pub const fn new() -> Self {
        Fir { comms_history: [0; TAPS], pinger_history: [0; TAPS], index: 0 }
    }

    fn push(&mut self, select: FilterSelect, sample: u8) -> (&Coeffs, &[i16; TAPS], usize) {
        let index = self.index;
        self.index = (index + 1) % TAPS;
        let (coeffs, history) = match select {
            FilterSelect::Comms => (&COMMS, &mut self.comms_history),
            FilterSelect::Pinger(freq) => (freq.coeffs(), &mut self.pinger_history),
        };
        history[index] = sample as i16;
        (coeffs, history, index)
    }

    /// `sum(coeffs[i] * history[index - i]) >> 15`, with the history read backwards from the newest sample.
    fn output(coeffs: &Coeffs, history: &[i16; TAPS], index: usize) -> i32 {
        let (newer, older) = history.split_at(index + 1);
        let (head, tail) = coeffs.split_at(index + 1);
        let mut sum = 0i32;
        for (&c, &x) in head.iter().zip(newer.iter().rev()) {
            sum += c as i32 * x as i32;
        }
        for (&c, &x) in tail.iter().zip(older.iter().rev()) {
            sum += c as i32 * x as i32;
        }
        sum >> SCALE_SHIFT
    }

    /// Filters one sample (`fir_filter_int` / `fir_filter_pinger_int`).
    pub fn filter(&mut self, select: FilterSelect, sample: u8) -> i32 {
        let (coeffs, history, index) = self.push(select, sample);
        Self::output(coeffs, history, index)
    }

    /// Filters a whole capture and keeps every [`DECIMATE_BY`]th output, starting with the first
    /// (`process_capture_buffer`). `out` receives `input.len() / DECIMATE_BY` values.
    pub fn filter_decimate(&mut self, select: FilterSelect, input: &[u8], out: &mut [f32]) {
        debug_assert_eq!(input.len(), out.len() * DECIMATE_BY);
        for (block, out) in input.as_chunks::<DECIMATE_BY>().0.iter().zip(out.iter_mut()) {
            *out = self.filter(select, block[0]) as f32;
            for &sample in &block[1..] {
                self.push(select, sample);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::vec::Vec;

    /// Straight port of `fir_filter_int`, with its own state, as the reference.
    struct Reference {
        history: [i16; TAPS],
        index: usize,
    }

    impl Reference {
        #[allow(clippy::needless_range_loop)] // Kept in the shape of the C loop
        fn filter(&mut self, coeffs: &Coeffs, sample: u8) -> i32 {
            self.history[self.index] = sample as i16;
            let mut sum: i32 = 0;
            for i in 0..TAPS {
                sum += coeffs[i] as i32 * self.history[(self.index + TAPS - i) % TAPS] as i32;
            }
            self.index = (self.index + 1) % TAPS;
            sum >> SCALE_SHIFT
        }
    }

    fn noise(len: usize, seed: u32) -> Vec<u8> {
        let mut state = seed;
        (0..len)
            .map(|_| {
                state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                (state >> 24) as u8
            })
            .collect()
    }

    #[test]
    fn matches_c_filter_sample_by_sample() {
        let mut fir = Fir::new();
        let mut reference = Reference { history: [0; TAPS], index: 0 };
        for sample in noise(1000, 7) {
            assert_eq!(fir.filter(FilterSelect::Comms, sample), reference.filter(&COMMS, sample));
        }
    }

    #[test]
    fn decimated_output_matches_filtering_everything() {
        for select in [FilterSelect::Comms, FilterSelect::Pinger(PingerFreq::K35)] {
            let coeffs = match select {
                FilterSelect::Comms => &COMMS,
                FilterSelect::Pinger(f) => f.coeffs(),
            };
            let mut fir = Fir::new();
            let mut reference = Reference { history: [0; TAPS], index: 0 };
            // Two captures, so the history carries over between them
            for seed in [1, 2] {
                let input = noise(crate::NUM_SAMPLES, seed);
                let mut out = [0.0f32; crate::NUM_SAMPLES_DEC];
                fir.filter_decimate(select, &input, &mut out);
                let expected: Vec<f32> = input
                    .iter()
                    .enumerate()
                    .map(|(i, &s)| (i, reference.filter(coeffs, s)))
                    .filter(|(i, _)| i % DECIMATE_BY == 0)
                    .map(|(_, y)| y as f32)
                    .collect();
                assert_eq!(&out[..], &expected[..]);
            }
        }
    }

    #[test]
    fn impulse_response_is_the_coefficients() {
        let mut fir = Fir::new();
        let mut response = Vec::new();
        response.push(fir.filter(FilterSelect::Comms, 128));
        for _ in 1..TAPS {
            response.push(fir.filter(FilterSelect::Comms, 0));
        }
        let expected: Vec<i32> = COMMS.iter().map(|&c| (c as i32 * 128) >> SCALE_SHIFT).collect();
        assert_eq!(response, expected);
    }

    #[test]
    fn worst_case_sum_fits_i32() {
        for coeffs in [&COMMS, &PINGER_20K, &PINGER_25K, &PINGER_30K, &PINGER_35K, &PINGER_37K, &PINGER_40K] {
            let gain: i64 = coeffs.iter().map(|&c| (c as i64).abs()).sum();
            assert!(gain * 255 < i32::MAX as i64);
        }
    }
}
