//! Symbol detection from band magnitudes (`rx_observe_3`, `detect_symbol`).
//!
//! Each tone's magnitude is divided by the noise floor. A tone counts as present when that SNR reaches the minimum
//! SNR (`ivc/set_min_snr`, default [`crate::DEFAULT_MIN_SNR`]); the strongest present tone wins if it dominates the
//! other two by [`MIN_SYMBOL_DOMINANCE`].

use crate::Symbol;

/// Ratio the winning SNR must have over each other SNR (`MIN_SYMBOL_DOMINANCE`).
pub const MIN_SYMBOL_DOMINANCE: f32 = 1.0;
/// Guards the divisions, as in C.
const EPSILON: f32 = 1e-6;

/// The comms bands (`fft_bins` without the unused 37 kHz pinger band, which only fed a debug log).
pub mod comms {
    use crate::spectrum::Band;
    use crate::{FREQ_HIGH_HZ, FREQ_LOW_HZ, FREQ_NOISE_REF_HZ, FREQ_SYNC_HZ};

    pub const REF1: usize = 0;
    pub const REF2: usize = 1;
    pub const LOW: usize = 2;
    pub const HIGH: usize = 3;
    pub const SYNC: usize = 4;
    pub const REF3: usize = 5;
    pub const REF4: usize = 6;

    /// In the same order as `fft_bins`.
    pub const BANDS: [Band; 7] = [
        Band::around(FREQ_NOISE_REF_HZ[0], true),
        Band::around(FREQ_NOISE_REF_HZ[1], true),
        Band::around(FREQ_LOW_HZ, false),
        Band::around(FREQ_HIGH_HZ, false),
        Band::around(FREQ_SYNC_HZ, false),
        Band::around(FREQ_NOISE_REF_HZ[2], true),
        Band::around(FREQ_NOISE_REF_HZ[3], true),
    ];
}

/// Classifies one observation from the tone SNRs (`detect_symbol`).
pub fn detect_symbol(snr_low: f32, snr_high: f32, snr_sync: f32, min_snr: f32) -> Symbol {
    let low_present = snr_low >= min_snr;
    let high_present = snr_high >= min_snr;
    let sync_present = snr_sync >= min_snr;
    if !low_present && !high_present && !sync_present {
        return Symbol::None;
    }

    let dominates = |winner: f32, a: f32, b: f32| {
        winner / (a + EPSILON) >= MIN_SYMBOL_DOMINANCE && winner / (b + EPSILON) >= MIN_SYMBOL_DOMINANCE
    };
    let best = snr_sync.max(snr_low.max(snr_high));
    if best == snr_sync {
        if dominates(snr_sync, snr_low, snr_high) {
            return Symbol::Sync;
        }
    } else if best == snr_low {
        if dominates(snr_low, snr_sync, snr_high) {
            return Symbol::Low;
        }
    } else if dominates(snr_high, snr_sync, snr_low) {
        return Symbol::High;
    }
    Symbol::None
}

/// Classifies the comms band magnitudes of one capture. Nothing is detected until the noise floor has settled
/// (`rx_observe_3`).
pub fn observe(magnitudes: &[f32; 7], noise_floor: Option<f32>, min_snr: f32) -> Symbol {
    let Some(floor) = noise_floor else { return Symbol::None };
    let snr = |band: usize| magnitudes[band] / (floor + EPSILON);
    detect_symbol(snr(comms::LOW), snr(comms::HIGH), snr(comms::SYNC), min_snr)
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::filter::FilterSelect;
    use crate::spectrum::tests::capture;
    use crate::spectrum::{Analyzer, BandSet, NOISE_SETTLE_FRAMES};
    use crate::{DEFAULT_MIN_SNR, NUM_SAMPLES};

    #[test]
    fn classification() {
        let min = DEFAULT_MIN_SNR;
        assert_eq!(detect_symbol(1.0, 1.0, 1.0, min), Symbol::None);
        assert_eq!(detect_symbol(10.0, 1.0, 1.0, min), Symbol::Low);
        assert_eq!(detect_symbol(1.0, 10.0, 1.0, min), Symbol::High);
        assert_eq!(detect_symbol(1.0, 1.0, 10.0, min), Symbol::Sync);
        // With a dominance of 1 the strongest present tone wins; the epsilon makes an exact tie ambiguous
        assert_eq!(detect_symbol(10.0, 9.0, 1.0, min), Symbol::Low);
        assert_eq!(detect_symbol(10.0, 10.0, 10.0, min), Symbol::None);
        assert_eq!(detect_symbol(10.0, 10.0, 1.0, min), Symbol::None);
        // Just at the threshold counts as present
        assert_eq!(detect_symbol(4.0, 0.0, 0.0, min), Symbol::Low);
        assert_eq!(detect_symbol(f32::NAN, 0.0, 0.0, min), Symbol::None);
    }

    #[test]
    fn nothing_before_the_noise_floor_settles() {
        assert_eq!(observe(&[1.0, 1.0, 100.0, 1.0, 1.0, 1.0, 1.0], None, DEFAULT_MIN_SNR), Symbol::None);
        assert_eq!(observe(&[1.0, 1.0, 100.0, 1.0, 1.0, 1.0, 1.0], Some(1.0), DEFAULT_MIN_SNR), Symbol::Low);
    }

    #[test]
    fn detects_each_tone_from_adc_samples() {
        let bands = BandSet::new(comms::BANDS);
        let mut analyzer = Analyzer::new();
        let mut start = 0;
        for _ in 0..NOISE_SETTLE_FRAMES {
            let m = analyzer.process(&capture(&[], 4.0, start as u32, start), FilterSelect::Comms, &bands);
            assert_eq!(observe(&m, analyzer.noise_floor(), DEFAULT_MIN_SNR), Symbol::None);
            start += NUM_SAMPLES;
        }
        for (hz, symbol) in [
            (crate::FREQ_LOW_HZ, Symbol::Low),
            (crate::FREQ_HIGH_HZ, Symbol::High),
            (crate::FREQ_SYNC_HZ, Symbol::Sync),
        ] {
            // Two captures: the first flushes the previous tone out of the FIR history
            let mut got = Symbol::None;
            for _ in 0..2 {
                let samples = capture(&[(hz as f64, 15.0)], 4.0, start as u32, start);
                let m = analyzer.process(&samples, FilterSelect::Comms, &bands);
                got = observe(&m, analyzer.noise_floor(), DEFAULT_MIN_SNR);
                start += NUM_SAMPLES;
            }
            assert_eq!(got, symbol, "{hz} Hz");
        }
        let m = analyzer.process(&capture(&[], 4.0, 1, start), FilterSelect::Comms, &bands);
        assert_eq!(observe(&m, analyzer.noise_floor(), DEFAULT_MIN_SNR), Symbol::None);
    }
}
