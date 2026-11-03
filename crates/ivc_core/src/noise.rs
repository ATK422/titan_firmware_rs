//! Optional noise-tracking thresholds (the `adaptive-threshold` feature of the `ivc_board` firmware).
//!
//! The C firmware, and by default the Rust one, detects a symbol when its band magnitude is above a fixed 300 and
//! the other band is below 300. That works while the noise in a band stays well below 300. Once it doesn't, noise
//! alone crosses the threshold: the "other band below 300" test fails on genuine symbols, and stray detections
//! start packets that aren't there (each costs the receiver 825 ms of decoding garbage). Delivery then drops off a
//! cliff: in the emulator from all to nothing between 76 and 78 dB re 1 µPa²/Hz at 25 m, although the symbols are
//! still ten times stronger than the noise.
//!
//! [`NoiseFloor`] tracks the mean noise magnitude per band and raises both thresholds to [`MIN_SNR`] times it when
//! that is above the fixed ones, so stray detections stay rare (fewer than 1 in 10 000 band magnitudes for
//! Rayleigh distributed noise) however noisy the water gets. It never lowers them: in quiet water nothing changes, so the
//! board is never more sensitive than the calibrated C firmware, which matters in a reverberant pool where echoes
//! of the previous symbol reach a few hundred.
//!
//! The noise sample of a capture is the smaller of its two band magnitudes. A symbol only fills one band, so
//! packets and ACKs don't drag the floor up. The floor is the median of the last [`NOISE_WINDOW`] samples, so the
//! odd capture with energy in both bands (a symbol change in reverberant water, the start of a packet heard
//! halfway through, a pinger's sidelobes) doesn't move it either; a mean of them rose eightfold in the
//! emulator's pool. For Rayleigh distributed noise the smaller of two band magnitudes has a median of
//! `sqrt(ln 2 / (pi / 2))` (0.66) times the mean of either, which [`NoiseFloor::level`] corrects for. The floor is
//! updated only while the modem is idle, and not used until the window is full.
//!
//! The trade-off: in noise loud enough to raise the thresholds, a symbol has to be [`MIN_SNR`] times the noise to
//! be detected, where the fixed threshold took anything above 300 if the other band happened to be quiet. And the
//! floor assumes the noise is about the same in both bands; a steady tone in one band raises it only as far as the
//! other band's noise.

use crate::Thresholds;

/// Noise samples (captures) the floor is the median of: about 0.2 to 0.9 s of idle time, depending on how fast the
/// detector runs. The median of 128 samples is within about 6 % of the true one two times in three.
pub const NOISE_WINDOW: usize = 128;
/// Thresholds are at least this many times the mean noise magnitude. For Rayleigh distributed magnitudes the
/// chance of exceeding `k` times the mean is `exp(-pi k^2 / 4)`: 0.007 % for 3.5, and still 0.07 % if the floor
/// came out 13 % low.
pub const MIN_SNR: f32 = 3.5;
/// Mean band magnitude of Rayleigh noise over the median of the smaller of two: `sqrt(pi / (2 ln 2))`.
const MEDIAN_OF_MIN_TO_MEAN: f32 = 1.505_4;

/// The noise floor of the two detection bands.
#[derive(Clone, Copy, Debug)]
pub struct NoiseFloor {
    /// The last samples, oldest at `next` once the window is full.
    recent: [f32; NOISE_WINDOW],
    /// The same samples in ascending order.
    sorted: [f32; NOISE_WINDOW],
    len: usize,
    next: usize,
}

impl Default for NoiseFloor {
    fn default() -> Self {
        Self::new()
    }
}

impl NoiseFloor {
    pub const fn new() -> Self {
        NoiseFloor { recent: [0.0; NOISE_WINDOW], sorted: [0.0; NOISE_WINDOW], len: 0, next: 0 }
    }

    /// Adds the band magnitudes of a capture taken while the modem was idle.
    pub fn update(&mut self, magnitudes: [f32; 2]) {
        let sample = magnitudes[0].min(magnitudes[1]);
        if self.len == NOISE_WINDOW {
            // Drop the oldest sample from the sorted copy
            let oldest = self.recent[self.next];
            let i = self.sorted[..self.len].iter().position(|&x| x == oldest).unwrap_or(self.len - 1);
            self.sorted.copy_within(i + 1..self.len, i);
            self.len -= 1;
        }
        let i = self.sorted[..self.len].partition_point(|&x| x < sample);
        self.sorted.copy_within(i..self.len, i + 1);
        self.sorted[i] = sample;
        self.len += 1;
        self.recent[self.next] = sample;
        self.next = (self.next + 1) % NOISE_WINDOW;
    }

    /// The mean noise magnitude in a band, once the window is full.
    pub fn level(&self) -> Option<f32> {
        (self.len == NOISE_WINDOW).then(|| {
            let median = (self.sorted[NOISE_WINDOW / 2 - 1] + self.sorted[NOISE_WINDOW / 2]) / 2.0;
            median * MEDIAN_OF_MIN_TO_MEAN
        })
    }

    /// `fixed`, raised to [`MIN_SNR`] times the noise level where that is higher.
    pub fn thresholds(&self, fixed: Thresholds) -> Thresholds {
        match self.level() {
            Some(level) => Thresholds { low: fixed.low.max(MIN_SNR * level), high: fixed.high.max(MIN_SNR * level) },
            None => fixed,
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::{BANDS, BandDetector, NUM_SAMPLES, SAMPLE_RATE_HZ, TALOS_THRESHOLDS, classify};
    use std::vec::Vec;

    /// Deterministic Gaussian noise (Box-Muller over an LCG).
    struct Noise(u64);
    impl Noise {
        fn uniform(&mut self) -> f64 {
            self.0 = self.0.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            ((self.0 >> 11) as f64 + 0.5) / (1u64 << 53) as f64
        }
        fn gaussian(&mut self) -> f64 {
            let (u, v) = (self.uniform(), self.uniform());
            (-2.0 * u.ln()).sqrt() * (2.0 * std::f64::consts::PI * v).cos()
        }
    }

    /// An 8-bit capture: white noise of standard deviation `sigma` plus an optional tone, around mid-scale.
    fn capture(noise: &mut Noise, sigma: f64, tone: Option<(f64, f64)>) -> Vec<u8> {
        let phase = noise.uniform() * 2.0 * std::f64::consts::PI;
        (0..NUM_SAMPLES)
            .map(|i| {
                let t = i as f64 / SAMPLE_RATE_HZ as f64;
                let s = tone.map_or(0.0, |(hz, amp)| amp * (2.0 * std::f64::consts::PI * hz * t + phase).sin());
                (128.0 + s + sigma * noise.gaussian()).round().clamp(0.0, 255.0) as u8
            })
            .collect()
    }

    fn detector() -> BandDetector<2> {
        BandDetector::new(BANDS, SAMPLE_RATE_HZ, NUM_SAMPLES)
    }

    /// The mean band magnitude of white noise of standard deviation `sigma`: Rayleigh with scale `sigma sqrt(N/2)`.
    fn expected_level(sigma: f64) -> f64 {
        sigma * (NUM_SAMPLES as f64 / 2.0).sqrt() * (std::f64::consts::PI / 2.0).sqrt()
    }

    fn settled(sigma: f64, seed: u64) -> (NoiseFloor, Noise) {
        let (d, mut noise, mut floor) = (detector(), Noise(seed), NoiseFloor::new());
        for _ in 0..400 {
            floor.update(d.process(&capture(&mut noise, sigma, None)));
        }
        (floor, noise)
    }

    #[test]
    fn fixed_thresholds_until_settled() {
        let mut floor = NoiseFloor::new();
        for _ in 0..NOISE_WINDOW - 1 {
            floor.update([5000.0, 6000.0]);
            assert_eq!(floor.thresholds(TALOS_THRESHOLDS), TALOS_THRESHOLDS);
        }
        floor.update([5000.0, 6000.0]);
        let level = 5000.0 * MEDIAN_OF_MIN_TO_MEAN;
        assert!((floor.level().unwrap() - level).abs() < 1.0);
        assert_eq!(floor.thresholds(TALOS_THRESHOLDS), Thresholds { low: MIN_SNR * level, high: MIN_SNR * level });
    }

    #[test]
    fn median_of_the_last_window() {
        let mut floor = NoiseFloor::new();
        let mut rng = Noise(11);
        let mut all = Vec::new();
        for i in 0..300 {
            let x = (rng.uniform() * 1000.0) as f32;
            // Repeated values too, which the sorted copy has to drop one at a time
            let x = if i % 7 == 0 { 500.0 } else { x };
            floor.update([x, x + 1.0]);
            all.push(x);
            if all.len() >= NOISE_WINDOW {
                let mut last = all[all.len() - NOISE_WINDOW..].to_vec();
                last.sort_by(f32::total_cmp);
                let median = (last[NOISE_WINDOW / 2 - 1] + last[NOISE_WINDOW / 2]) / 2.0;
                assert_eq!(floor.level(), Some(median * MEDIAN_OF_MIN_TO_MEAN), "after {i}");
            } else {
                assert_eq!(floor.level(), None);
            }
        }
        let expected = (core::f64::consts::PI / (2.0 * core::f64::consts::LN_2)).sqrt();
        assert!((MEDIAN_OF_MIN_TO_MEAN as f64 - expected).abs() < 1e-4);
    }

    #[test]
    fn tracks_white_noise() {
        for sigma in [2.0, 5.0, 10.0, 20.0] {
            let (floor, _) = settled(sigma, sigma as u64);
            let (got, expected) = (floor.level().unwrap() as f64, expected_level(sigma));
            assert!((got / expected - 1.0).abs() < 0.15, "sigma {sigma}: {got} vs {expected}");
        }
    }

    #[test]
    fn quiet_water_keeps_the_fixed_thresholds() {
        // Noise averaging 50 (the emulator's pool shows 10-60): 3.5 x 50 is below 300
        let (floor, _) = settled(50.0 / expected_level(1.0), 1);
        assert_eq!(floor.thresholds(TALOS_THRESHOLDS), TALOS_THRESHOLDS);
    }

    #[test]
    fn symbols_dont_raise_the_floor() {
        let (d, mut noise) = (detector(), Noise(7));
        let sigma = 1.0;
        let (mut quiet, _) = settled(sigma, 7);
        let before = quiet.level().unwrap();
        // A long run of strong symbols in alternate bands (both packets and ACKs fill one band at a time)
        for i in 0..200 {
            let hz = if i % 20 < 10 { 17_000.0 } else { 19_000.0 };
            quiet.update(d.process(&capture(&mut noise, sigma, Some((hz, 60.0)))));
        }
        let after = quiet.level().unwrap();
        assert!(after < 2.0 * before, "{before} -> {after}");
    }

    #[test]
    fn odd_captures_dont_move_the_floor() {
        let (d, mut noise) = (detector(), Noise(13));
        let sigma = 50.0 / expected_level(1.0);
        let (clean, _) = settled(sigma, 13);
        let mut floor = clean;
        // One capture in ten has energy in both bands (a symbol change in a reverberant pool), one in fifty a
        // strong spike in both
        for i in 0..400 {
            let m = d.process(&capture(&mut noise, sigma, None));
            let m = match i % 50 {
                0 => [20_000.0, 20_000.0],
                n if n % 10 == 5 => [m[0] + 2000.0, m[1] + 1000.0],
                _ => m,
            };
            floor.update(m);
        }
        let (before, after) = (clean.level().unwrap(), floor.level().unwrap());
        assert!(after < 1.3 * before, "{before} -> {after}");
        assert_eq!(floor.thresholds(TALOS_THRESHOLDS), TALOS_THRESHOLDS);
    }

    /// Fractions of captures classified as a stray symbol (noise only) and as the right symbol (tone of amplitude
    /// `amp` at 19 kHz plus noise), with fixed thresholds and with the floor's.
    fn rates(sigma: f64, amp: f64, seed: u64) -> [(f64, f64); 2] {
        let d = detector();
        let (floor, mut noise) = settled(sigma, seed);
        let adaptive = floor.thresholds(TALOS_THRESHOLDS);
        let n = 2000;
        [TALOS_THRESHOLDS, adaptive].map(|t| {
            let stray = (0..n).filter(|_| classify(d.process(&capture(&mut noise, sigma, None)), t).is_some()).count();
            let hits = (0..n)
                .filter(|_| classify(d.process(&capture(&mut noise, sigma, Some((19_000.0, amp)))), t) == Some(true))
                .count();
            (stray as f64 / n as f64, hits as f64 / n as f64)
        })
    }

    #[test]
    fn loud_noise_raises_the_thresholds() {
        // Noise averaging 150, as at 77 dB in the emulator, and symbols ten times stronger
        let sigma = 150.0 / expected_level(1.0);
        let [(fixed_stray, fixed_hits), (stray, hits)] = rates(sigma, 1500.0 / 250.0, 3);
        // With the fixed thresholds noise starts a packet several times a second and kills one symbol in twenty...
        assert!(fixed_stray > 0.02 && fixed_hits < 0.97, "fixed: {fixed_stray} stray, {fixed_hits} hits");
        // ...the raised ones hardly ever do
        assert!(stray < 0.003 && hits > 0.995, "adaptive: {stray} stray, {hits} hits");
    }

    #[test]
    fn trade_off_in_loud_noise() {
        // Noise averaging 150 and a symbol of 450: three times the noise. The fixed thresholds detect it most of
        // the time (when the other band stays under 300) but take noise for symbols too; the raised ones need more.
        let sigma = 150.0 / expected_level(1.0);
        let [(fixed_stray, fixed_hits), (stray, hits)] = rates(sigma, 450.0 / 250.0, 5);
        assert!(fixed_hits > hits, "fixed {fixed_hits} vs adaptive {hits}");
        assert!(fixed_stray > 10.0 * stray.max(0.001), "fixed {fixed_stray} vs adaptive {stray}");
    }
}
