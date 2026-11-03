//! Band magnitude detection.
//!
//! The C firmware ran a 500 point real FFT over each 1 ms capture, summed the power of every bin whose centre
//! frequency fell inside a band, and took the square root. With 1 kHz bin spacing only a couple of bins land in each
//! ±500 Hz band, so this computes exactly those bins with the Goertzel algorithm instead. The result is the same
//! magnitude (`|X[k]|` of the unscaled DFT) for a fraction of the work, which matters on a Cortex-M0+ without an FPU.

use libm::{cosf, sqrtf};

/// Most DFT bins a single band may cover.
const MAX_BINS_PER_BAND: usize = 4;

/// An inclusive frequency range.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Band {
    pub min_hz: f32,
    pub max_hz: f32,
}

#[derive(Clone, Copy, Debug)]
struct BandBins {
    /// Goertzel coefficient `2 cos(2 pi k / N)` for each bin in the band.
    coeffs: [f32; MAX_BINS_PER_BAND],
    count: usize,
}

/// Measures the magnitude of `B` frequency bands in fixed length captures of 8-bit samples.
#[derive(Clone, Debug)]
pub struct BandDetector<const B: usize> {
    bands: [BandBins; B],
    num_samples: usize,
}

impl<const B: usize> BandDetector<B> {
    /// Selects every bin `k < num_samples / 2` whose frequency `k * sample_rate / num_samples` lies in each band,
    /// the same selection `compute_bin_amplitudes` made over the FFT output.
    ///
    /// # Panics
    /// If a band covers more than [`MAX_BINS_PER_BAND`] bins.
    pub fn new(bands: [crate::Band; B], sample_rate_hz: f32, num_samples: usize) -> Self {
        let resolution = sample_rate_hz / num_samples as f32;
        let bands = bands.map(|band| {
            let mut bins = BandBins { coeffs: [0.0; MAX_BINS_PER_BAND], count: 0 };
            for k in 0..num_samples / 2 {
                let freq = resolution * k as f32;
                if freq >= band.min_hz && freq <= band.max_hz {
                    assert!(bins.count < MAX_BINS_PER_BAND, "band covers too many bins");
                    let omega = 2.0 * core::f32::consts::PI * k as f32 / num_samples as f32;
                    bins.coeffs[bins.count] = 2.0 * cosf(omega);
                    bins.count += 1;
                }
            }
            bins
        });
        BandDetector { bands, num_samples }
    }

    /// Number of DFT bins selected for each band.
    pub fn bins_per_band(&self) -> [usize; B] {
        self.bands.map(|b| b.count)
    }

    /// Returns the magnitude of each band in `samples`, which must hold the capture length given to [`Self::new`].
    pub fn process(&self, samples: &[u8]) -> [f32; B] {
        debug_assert_eq!(samples.len(), self.num_samples);

        // Remove DC like fill_fft_input did. It does not change any non-zero bin, but keeps the filter state small.
        let sum: u32 = samples.iter().map(|&s| s as u32).sum();
        let mean = sum as f32 / samples.len() as f32;

        self.bands.map(|band| {
            let mut power = 0.0;
            for &coeff in &band.coeffs[..band.count] {
                let (mut s1, mut s2) = (0.0f32, 0.0f32);
                for &sample in samples {
                    let s0 = (sample as f32 - mean) + coeff * s1 - s2;
                    s2 = s1;
                    s1 = s0;
                }
                power += s1 * s1 + s2 * s2 - coeff * s1 * s2;
            }
            // Rounding can push an empty bin a hair below zero
            sqrtf(power.max(0.0))
        })
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::{BANDS, NUM_SAMPLES, SAMPLE_RATE_HZ, TALOS_THRESHOLDS, classify};
    use std::vec::Vec;

    /// The C pipeline in f64: subtract mean, full DFT, sum power of in-band bins below Nyquist, square root.
    fn dft_band_magnitudes(samples: &[u8], bands: &[Band; 2]) -> [f64; 2] {
        let n = samples.len();
        let mean = samples.iter().map(|&s| s as f64).sum::<f64>() / n as f64;
        let res = SAMPLE_RATE_HZ as f64 / n as f64;
        let mut out = [0.0; 2];
        for k in 0..n / 2 {
            let (mut re, mut im) = (0.0, 0.0);
            for (i, &s) in samples.iter().enumerate() {
                let angle = -2.0 * std::f64::consts::PI * (k * i) as f64 / n as f64;
                re += (s as f64 - mean) * angle.cos();
                im += (s as f64 - mean) * angle.sin();
            }
            let freq = res * k as f64;
            for (j, band) in bands.iter().enumerate() {
                if freq >= band.min_hz as f64 && freq <= band.max_hz as f64 {
                    out[j] += re * re + im * im;
                    break;
                }
            }
        }
        out.map(f64::sqrt)
    }

    /// An 8-bit ADC capture of a tone riding on mid-scale, with a little deterministic noise.
    fn tone(freq_hz: f64, amplitude: f64, phase: f64) -> Vec<u8> {
        let mut noise: u32 = 0x1234_5678;
        (0..NUM_SAMPLES)
            .map(|i| {
                noise = noise.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let jitter = ((noise >> 24) as f64 / 255.0 - 0.5) * 2.0;
                let t = i as f64 / SAMPLE_RATE_HZ as f64;
                let v = 128.0 + amplitude * (2.0 * std::f64::consts::PI * freq_hz * t + phase).sin() + jitter;
                v.round().clamp(0.0, 255.0) as u8
            })
            .collect()
    }

    #[test]
    fn selects_same_bins_as_fft_version() {
        let detector = BandDetector::new(BANDS, SAMPLE_RATE_HZ, NUM_SAMPLES);
        // 1 kHz resolution: only 17 kHz falls in 16.5..=17.5 kHz and only 19 kHz in 18.5..=19.5 kHz
        assert_eq!(detector.bins_per_band(), [1, 1]);
    }

    #[test]
    fn matches_reference_dft() {
        let detector = BandDetector::new(BANDS, SAMPLE_RATE_HZ, NUM_SAMPLES);
        for (freq, amp, phase) in
            [(17_000.0, 3.0, 0.3), (19_000.0, 40.0, 1.1), (18_000.0, 20.0, 2.0), (17_200.0, 5.0, 0.0)]
        {
            let samples = tone(freq, amp, phase);
            let expected = dft_band_magnitudes(&samples, &BANDS);
            let got = detector.process(&samples);
            for j in 0..2 {
                let tolerance = 1e-3 * expected[j].max(1.0) + 0.05;
                assert!(
                    (got[j] as f64 - expected[j]).abs() < tolerance,
                    "{freq} Hz band {j}: {} vs {}",
                    got[j],
                    expected[j]
                );
            }
        }
    }

    #[test]
    fn detects_symbols() {
        let detector = BandDetector::new(BANDS, SAMPLE_RATE_HZ, NUM_SAMPLES);
        assert_eq!(classify(detector.process(&tone(17_000.0, 10.0, 0.0)), TALOS_THRESHOLDS), Some(false));
        assert_eq!(classify(detector.process(&tone(19_000.0, 10.0, 0.0)), TALOS_THRESHOLDS), Some(true));
        assert_eq!(classify(detector.process(&tone(19_000.0, 0.0, 0.0)), TALOS_THRESHOLDS), None);
    }
}
