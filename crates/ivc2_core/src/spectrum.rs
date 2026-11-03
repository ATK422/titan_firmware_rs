//! Band magnitudes and the noise floor (`fft/fft.c`).
//!
//! The C firmware ran a 1024 point real FFT over each filtered, decimated, DC-free and Hann windowed capture, summed
//! the power of every bin whose frequency fell inside a band, and took the square root. With 122 Hz bin spacing and
//! ±61 Hz bands, every band holds exactly one bin (see the tests), so [`Analyzer`] evaluates only those bins with the
//! Goertzel algorithm. The magnitude is the same `|X[k]|` of the unscaled DFT that kiss_fftr produced, for a fraction
//! of the work on a core without an FPU.
//!
//! After every capture the mean magnitude of the reference bands updates an exponential moving average, the noise
//! floor. It counts as settled after [`NOISE_SETTLE_FRAMES`] captures; until then no symbol is detected.

use libm::{cosf, sqrtf};

use crate::filter::{FilterSelect, Fir};
use crate::{BIN_RANGE_HZ, DECIMATED_RATE_HZ, NUM_SAMPLES_DEC};

/// Most DFT bins a single band may cover. A ±61 Hz band can never span more than one 122 Hz bin; two leaves room.
pub const MAX_BINS_PER_BAND: usize = 2;
/// Weight of each new reference measurement in the noise floor (`EMA_ALPHA`).
pub const NOISE_EMA_ALPHA: f32 = 0.05;
/// Captures processed before the noise floor is trusted (`num_frames_processed >= 50`).
pub const NOISE_SETTLE_FRAMES: u32 = 50;

/// An inclusive frequency range in whole hertz (`frequency_bin_t`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Band {
    pub min_hz: i32,
    pub max_hz: i32,
    /// Counts towards the noise floor (`is_reference_bin`).
    pub is_reference: bool,
}

impl Band {
    /// The band `centre ± BIN_RANGE_HZ`.
    pub const fn around(centre_hz: i32, is_reference: bool) -> Band {
        Band { min_hz: centre_hz - BIN_RANGE_HZ, max_hz: centre_hz + BIN_RANGE_HZ, is_reference }
    }
}

/// Frequency of DFT bin `k`, computed in single precision like `calculate_frequencies_dec`.
fn bin_frequency(k: usize) -> f32 {
    (DECIMATED_RATE_HZ as f32 / NUM_SAMPLES_DEC as f32) * k as f32
}

#[derive(Clone, Copy, Debug)]
struct BandBins {
    bins: [u16; MAX_BINS_PER_BAND],
    /// Goertzel coefficient `2 cos(2 pi k / N)` of each bin.
    coeffs: [f32; MAX_BINS_PER_BAND],
    count: usize,
    is_reference: bool,
}

/// `B` bands with their DFT bins worked out.
#[derive(Clone, Debug)]
pub struct BandSet<const B: usize> {
    bands: [BandBins; B],
}

impl<const B: usize> BandSet<B> {
    /// Assigns every bin `k < N / 2` to the first band containing its frequency, the selection
    /// `accumulate_fft_window` made over the FFT output.
    ///
    /// # Panics
    /// If a band covers more than [`MAX_BINS_PER_BAND`] bins.
    pub fn new(bands: [Band; B]) -> Self {
        let mut out = [BandBins {
            bins: [0; MAX_BINS_PER_BAND],
            coeffs: [0.0; MAX_BINS_PER_BAND],
            count: 0,
            is_reference: false,
        }; B];
        for (slot, band) in out.iter_mut().zip(bands.iter()) {
            slot.is_reference = band.is_reference;
        }
        for k in 0..NUM_SAMPLES_DEC / 2 {
            let freq = bin_frequency(k);
            if let Some(j) = bands.iter().position(|b| freq >= b.min_hz as f32 && freq <= b.max_hz as f32) {
                let slot = &mut out[j];
                assert!(slot.count < MAX_BINS_PER_BAND, "band covers too many bins");
                let omega = 2.0 * core::f32::consts::PI * k as f32 / NUM_SAMPLES_DEC as f32;
                slot.bins[slot.count] = k as u16;
                slot.coeffs[slot.count] = 2.0 * cosf(omega);
                slot.count += 1;
            }
        }
        BandSet { bands: out }
    }

    /// The DFT bins of band `index`.
    pub fn bins(&self, index: usize) -> &[u16] {
        let band = &self.bands[index];
        &band.bins[..band.count]
    }
}

/// Turns captures into band magnitudes and keeps the noise floor (the C `fft.c` state).
#[derive(Clone, Debug)]
pub struct Analyzer {
    fir: Fir,
    window: [f32; NUM_SAMPLES_DEC],
    work: [f32; NUM_SAMPLES_DEC],
    noise_floor: f32,
    frames: u32,
}

impl Default for Analyzer {
    fn default() -> Self {
        Self::new()
    }
}

impl Analyzer {
    pub fn new() -> Self {
        let mut window = [0.0; NUM_SAMPLES_DEC];
        for (i, w) in window.iter_mut().enumerate() {
            // precompute_hann_window: the angle is computed in double and handed to cosf
            let angle = 2.0 * core::f64::consts::PI * i as f64 / (NUM_SAMPLES_DEC - 1) as f64;
            *w = 0.5 * (1.0 - cosf(angle as f32));
        }
        Analyzer { fir: Fir::new(), window, work: [0.0; NUM_SAMPLES_DEC], noise_floor: 0.0, frames: 0 }
    }

    /// First half of `fft_process`: filter and decimate `capture` (which must hold [`crate::NUM_SAMPLES`] samples),
    /// remove DC and apply the Hann window. Split from [`Self::measure`] so firmware can yield in between.
    pub fn prepare(&mut self, capture: &[u8], filter: FilterSelect) {
        self.fir.filter_decimate(filter, capture, &mut self.work);
        let mean = self.work.iter().sum::<f32>() / NUM_SAMPLES_DEC as f32;
        for (x, w) in self.work.iter_mut().zip(self.window.iter()) {
            *x = (*x - mean) * w;
        }
    }

    /// Second half of `fft_process`: the magnitude of each band in the prepared capture. Updates the noise floor
    /// from the reference bands, if `bands` has any.
    pub fn measure<const B: usize>(&mut self, bands: &BandSet<B>) -> [f32; B] {
        let magnitudes = bands.bands.map(|band| {
            let mut power = 0.0;
            for &coeff in &band.coeffs[..band.count] {
                let (mut s1, mut s2) = (0.0f32, 0.0f32);
                for &x in &self.work {
                    let s0 = x + coeff * s1 - s2;
                    s2 = s1;
                    s1 = s0;
                }
                power += s1 * s1 + s2 * s2 - coeff * s1 * s2;
            }
            // Rounding can push an empty bin a hair below zero
            sqrtf(power.max(0.0))
        });

        let mut reference_sum = 0.0;
        let mut references = 0;
        for (band, &magnitude) in bands.bands.iter().zip(magnitudes.iter()) {
            if band.is_reference {
                reference_sum += magnitude;
                references += 1;
            }
        }
        if references > 0 {
            let mean = reference_sum / references as f32;
            self.noise_floor = NOISE_EMA_ALPHA * mean + (1.0 - NOISE_EMA_ALPHA) * self.noise_floor;
        }
        self.frames = self.frames.saturating_add(1);
        magnitudes
    }

    /// [`Self::prepare`] followed by [`Self::measure`].
    pub fn process<const B: usize>(&mut self, capture: &[u8], filter: FilterSelect, bands: &BandSet<B>) -> [f32; B] {
        self.prepare(capture, filter);
        self.measure(bands)
    }

    /// The noise floor once it has settled (`fft_get_noise_floor`).
    pub fn noise_floor(&self) -> Option<f32> {
        (self.frames >= NOISE_SETTLE_FRAMES).then_some(self.noise_floor)
    }
}

#[cfg(test)]
pub(crate) mod tests {
    extern crate std;
    use super::*;
    use crate::detect::comms;
    use crate::filter::{COMMS, SCALE_SHIFT, TAPS};
    use crate::pinger::PingerFreq;
    use crate::{DECIMATE_BY, NUM_SAMPLES, SAMPLE_RATE_HZ};
    use std::vec::Vec;

    /// An 8-bit capture: tones riding on mid-scale plus deterministic white noise of the given peak amplitude.
    /// `start` is the index of the first sample, so consecutive captures continue the same waveform.
    pub(crate) fn capture(tones: &[(f64, f64)], noise: f64, seed: u32, start: usize) -> Vec<u8> {
        let mut state = seed;
        (start..start + NUM_SAMPLES)
            .map(|i| {
                state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let jitter = ((state >> 8) as f64 / (1u64 << 24) as f64 - 0.5) * 2.0 * noise;
                let t = i as f64 / SAMPLE_RATE_HZ as f64;
                let tone: f64 = tones.iter().map(|&(hz, amp)| amp * (2.0 * std::f64::consts::PI * hz * t).sin()).sum();
                (128.0 + tone + jitter).round().clamp(0.0, 255.0) as u8
            })
            .collect()
    }

    /// The C pipeline in f64 from a fresh filter state: FIR on every sample, keep every fourth, remove DC, Hann
    /// window, full DFT, sum in-band power below Nyquist, square root.
    fn reference_magnitudes(samples: &[u8], bands: &[Band]) -> Vec<f64> {
        let mut history = [0i16; TAPS];
        let mut index = 0;
        let mut decimated = Vec::new();
        for (i, &s) in samples.iter().enumerate() {
            history[index] = s as i16;
            let mut sum: i32 = 0;
            for t in 0..TAPS {
                sum += COMMS[t] as i32 * history[(index + TAPS - t) % TAPS] as i32;
            }
            index = (index + 1) % TAPS;
            if i % DECIMATE_BY == 0 {
                decimated.push((sum >> SCALE_SHIFT) as f64);
            }
        }
        let n = decimated.len();
        let mean = decimated.iter().sum::<f64>() / n as f64;
        let x: Vec<f64> = decimated
            .iter()
            .enumerate()
            .map(|(i, v)| (v - mean) * 0.5 * (1.0 - (2.0 * std::f64::consts::PI * i as f64 / (n - 1) as f64).cos()))
            .collect();
        let resolution = DECIMATED_RATE_HZ as f64 / n as f64;
        let mut out = std::vec![0.0; bands.len()];
        for k in 0..n / 2 {
            let freq = resolution * k as f64;
            if let Some(j) = bands.iter().position(|b| freq >= b.min_hz as f64 && freq <= b.max_hz as f64) {
                let (mut re, mut im) = (0.0, 0.0);
                for (i, v) in x.iter().enumerate() {
                    let angle = -2.0 * std::f64::consts::PI * (k * i) as f64 / n as f64;
                    re += v * angle.cos();
                    im += v * angle.sin();
                }
                out[j] += re * re + im * im;
            }
        }
        out.into_iter().map(f64::sqrt).collect()
    }

    #[test]
    fn every_band_holds_exactly_one_bin() {
        let comms = BandSet::new(comms::BANDS);
        let bins: Vec<u16> = (0..comms::BANDS.len()).map(|i| comms.bins(i)[0]).collect();
        for i in 0..comms::BANDS.len() {
            assert_eq!(comms.bins(i).len(), 1, "comms band {i}");
        }
        // 49, 49.5, 51, 51.5, 50.5, 52, 52.5 kHz; 49.5 kHz falls just outside bin 405 (49438.5 Hz) into 406
        assert_eq!(bins, [401, 406, 418, 422, 414, 426, 430]);
        for freq in PingerFreq::ALL {
            let set = BandSet::new(freq.bands());
            for i in 0..crate::pinger::NUM_BANDS {
                assert_eq!(set.bins(i).len(), 1, "{freq:?} band {i}");
            }
        }
    }

    #[test]
    fn matches_reference_pipeline() {
        let bands = BandSet::new(comms::BANDS);
        for tones in [
            &[(51_000.0, 20.0)][..],
            &[(51_500.0, 5.0), (49_000.0, 3.0)][..],
            &[(50_500.0, 40.0)][..],
            &[(50_700.0, 10.0)][..],
        ] {
            let samples = capture(tones, 2.0, 99, 0);
            let mut analyzer = Analyzer::new();
            let got = analyzer.process(&samples, FilterSelect::Comms, &bands);
            let expected = reference_magnitudes(&samples, &comms::BANDS);
            for j in 0..got.len() {
                let tolerance = 2e-3 * expected[j].max(1.0) + 0.5;
                assert!(
                    (got[j] as f64 - expected[j]).abs() < tolerance,
                    "{tones:?} band {j}: {} vs {}",
                    got[j],
                    expected[j]
                );
            }
        }
    }

    #[test]
    fn noise_floor_settles_after_fifty_frames() {
        let bands = BandSet::new(comms::BANDS);
        let mut analyzer = Analyzer::new();
        let samples = capture(&[], 5.0, 3, 0);
        let mut expected = 0.0f32;
        for frame in 0..NOISE_SETTLE_FRAMES {
            assert_eq!(analyzer.noise_floor(), None, "frame {frame}");
            let m = analyzer.process(&samples, FilterSelect::Comms, &bands);
            let refs = [m[comms::REF1], m[comms::REF2], m[comms::REF3], m[comms::REF4]];
            expected = NOISE_EMA_ALPHA * (refs.iter().sum::<f32>() / 4.0) + (1.0 - NOISE_EMA_ALPHA) * expected;
        }
        let floor = analyzer.noise_floor().unwrap();
        assert!((floor - expected).abs() < 1e-3 * expected.max(1.0), "{floor} vs {expected}");
        assert!(floor > 0.0);
    }

    #[test]
    fn pinger_band_picks_up_pinger() {
        let freq = PingerFreq::K30;
        let bands = BandSet::new(freq.bands());
        let mut analyzer = Analyzer::new();
        let quiet = analyzer.process(&capture(&[], 2.0, 5, 0), FilterSelect::Pinger(freq), &bands);
        let loud = analyzer.process(&capture(&[(30_000.0, 20.0)], 2.0, 5, 0), FilterSelect::Pinger(freq), &bands);
        let signal = crate::pinger::SIGNAL_INDEX;
        assert!(loud[signal] > 10.0 * quiet[signal], "{loud:?} vs {quiet:?}");
        for (i, &m) in loud.iter().enumerate() {
            if i != signal {
                assert!(loud[signal] > 5.0 * m, "{loud:?}");
            }
        }
    }
}
