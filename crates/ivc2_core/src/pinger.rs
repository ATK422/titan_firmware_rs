//! Pinger listening mode (`ivc/pinger/*`, `rx_handle_pinger`).
//!
//! While enabled, every capture goes through the band-pass filter of the selected pinger frequency and the board
//! streams the amplitude of the pinger band. Comms (receive and transmit) pause meanwhile.

use crate::filter::{self, Coeffs};
use crate::spectrum::Band;

/// Index of the pinger band within [`PingerFreq::bands`] (`FFT_PINGER_SIGNAL_IDX`).
pub const SIGNAL_INDEX: usize = 2;
/// Number of pinger bands (`NUM_PINGER_FFT_BINS`).
pub const NUM_BANDS: usize = 5;
/// How often the selected frequency is published on `ivc/pinger/selected_freq_khz` while the mode is on.
pub const FREQ_REPORT_PERIOD_MS: u64 = 1000;

/// A selectable pinger frequency (`pinger_freq_t`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PingerFreq {
    K20,
    K25,
    K30,
    K35,
    K37,
    K40,
}

impl PingerFreq {
    /// Selected at boot (`rx_init`).
    pub const DEFAULT: PingerFreq = PingerFreq::K30;

    /// Parses an `ivc/pinger/set_freq_khz` value. Other values are ignored, like `set_pinger_mode`.
    pub const fn from_khz(khz: i32) -> Option<PingerFreq> {
        Some(match khz {
            20 => PingerFreq::K20,
            25 => PingerFreq::K25,
            30 => PingerFreq::K30,
            35 => PingerFreq::K35,
            37 => PingerFreq::K37,
            40 => PingerFreq::K40,
            _ => return None,
        })
    }

    /// The value published on `ivc/pinger/selected_freq_khz` (`rx_get_pinger_mode_khz`).
    pub const fn khz(self) -> i32 {
        match self {
            PingerFreq::K20 => 20,
            PingerFreq::K25 => 25,
            PingerFreq::K30 => 30,
            PingerFreq::K35 => 35,
            PingerFreq::K37 => 37,
            PingerFreq::K40 => 40,
        }
    }

    /// The band-pass filter for this pinger (`pinger_coeffs_map`).
    pub fn coeffs(self) -> &'static Coeffs {
        match self {
            PingerFreq::K20 => &filter::PINGER_20K,
            PingerFreq::K25 => &filter::PINGER_25K,
            PingerFreq::K30 => &filter::PINGER_30K,
            PingerFreq::K35 => &filter::PINGER_35K,
            PingerFreq::K37 => &filter::PINGER_37K,
            PingerFreq::K40 => &filter::PINGER_40K,
        }
    }

    /// Two reference bands 1 kHz and 500 Hz below the pinger, the pinger band, and two references above
    /// (`fft_bins_pinger_*`).
    pub const fn bands(self) -> [Band; NUM_BANDS] {
        let hz = self.khz() * 1000;
        [
            Band::around(hz - 1000, true),
            Band::around(hz - 500, true),
            Band::around(hz, false),
            Band::around(hz + 500, true),
            Band::around(hz + 1000, true),
        ]
    }

    /// Every pinger frequency.
    pub const ALL: [PingerFreq; 6] =
        [PingerFreq::K20, PingerFreq::K25, PingerFreq::K30, PingerFreq::K35, PingerFreq::K37, PingerFreq::K40];

    /// Position in [`Self::ALL`].
    pub const fn index(self) -> usize {
        self as usize
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn khz_round_trip() {
        for freq in PingerFreq::ALL {
            assert_eq!(PingerFreq::from_khz(freq.khz()), Some(freq));
            assert_eq!(PingerFreq::ALL[freq.index()], freq);
        }
        assert_eq!(PingerFreq::from_khz(0), None);
        assert_eq!(PingerFreq::from_khz(36), None);
    }

    #[test]
    fn bands_match_c_tables() {
        // fft_bins_pinger_37k: refs at 36, 36.5, 37.5 and 38 kHz, pinger band in the middle
        let bands = PingerFreq::K37.bands();
        let centres = bands.map(|b| (b.min_hz + b.max_hz) / 2);
        assert_eq!(centres, [36_000, 36_500, 37_000, 37_500, 38_000]);
        assert_eq!(bands.map(|b| b.is_reference), [true, true, false, true, true]);
        assert_eq!(bands[SIGNAL_INDEX].min_hz, 37_000 - crate::BIN_RANGE_HZ);
    }
}
