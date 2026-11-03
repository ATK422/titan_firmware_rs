//! Hardware independent core of the second generation IVC (inter-vehicle communication) acoustic modem, shared by
//! `talos/IVCBoard2` and `talos/IVCDAC`.
//!
//! This is a different modem from the original `talos/IVCBoard` (see `ivc_core`); nothing on the wire is shared:
//!
//! | | IVCBoard (`ivc_core`) | IVCBoard2 / IVCDAC (this crate) |
//! |---|---|---|
//! | Tones | 0 = 17 kHz, 1 = 19 kHz | 0 = 51 kHz, 1 = 51.5 kHz, sync = 50.5 kHz |
//! | Symbol | 50 ms | 500 ms |
//! | Packet | start, heartbeat flag, 8 data bits, CRC-4 (MSB first) | sync before and after every data bit (LSB first), no CRC |
//! | Clocking | fixed schedule from the start symbol | self-clocked: a symbol is decoded when the tone changes |
//! | ACK | one ACK/NACK symbol at a fixed time | none (`send_ack`/`process_packet` exist in C but are never called) |
//! | Capture | 500 samples (1 ms), 500 kS/s | 4096 samples (8.2 ms), 500 kS/s, 64 tap FIR, decimated to 125 kS/s |
//! | Detector | two band magnitudes against fixed thresholds | eight band SNRs against a noise floor tracked from reference bands |
//!
//! The pieces, in signal order:
//!
//! * [`filter`]: the integer FIR band-pass filters and the decimation by four
//! * [`spectrum`]: DC removal, Hann window and band magnitudes, plus the noise floor EMA
//! * [`detect`]: turns band magnitudes into a [`Symbol`] observation
//! * [`consensus`]: debounces observations and emits a symbol when the tone changes
//! * [`receiver`]: assembles emitted symbols into bytes
//! * [`tx`]: the transmit schedule
//! * [`pinger`]: the pinger listening mode (amplitude of a selectable 20 to 40 kHz pinger)
//! * [`storage`]: the flash recorder (`ivc/storage/configure`)
//!
//! # Timing and the faster detector
//!
//! `ivc_core::timing` had to re-anchor the original board's receive and ACK schedule, because those were fixed delays
//! that only lined up with the C sender thanks to the C detector's latency. That reasoning does not carry over here:
//! this receiver has no fixed schedule and sends no ACK. It decodes a symbol when [`consensus::Consensus`] sees the tone
//! change, so detection latency only delays each decoded symbol by the same amount and never moves a sample point.
//!
//! What the detector speed does change is how many observations land in each 500 ms symbol. The C firmware
//! re-processed the newest capture once per main loop pass, with a 4096 point FIR and a 1024 point software FFT that
//! take an estimated 40 to 50 ms on the Cortex-M0+. The Rust firmware filters only the samples it keeps after
//! decimation and evaluates just the bins inside the bands with the Goertzel algorithm, so a capture plus its
//! processing should take about 25 ms (both figures are estimates; neither has been measured on a board). The
//! consensus thresholds ([`consensus::MIN_STABLE_SAMPLES`] = 3 and [`consensus::MAX_SAMPLE_GAP`] = 2) count
//! observations, so they now span roughly half the time they did in C. A symbol needs four observations (two to
//! switch to it, two more to become stable) before the next one can be recognised, which caps the observation period
//! near 100 ms; at 25 ms a symbol gets about twenty. The glitch rejection the thresholds exist for (a single stray
//! observation never changes the symbol) does not depend on time. The tests in [`receiver`] decode packets at observation periods from 10 to 80 ms,
//! with jitter and stray observations, to show this. The noise floor EMA ([`spectrum::NOISE_EMA_ALPHA`]) and its
//! settling count are also per frame, so the floor now adapts about twice as fast in wall clock time.
//!
//! The transmitted waveform is unchanged, so a Rust board and a C board can talk to each other.

#![no_std]

pub mod consensus;
pub mod detect;
pub mod filter;
pub mod pinger;
pub mod receiver;
pub mod spectrum;
pub mod storage;
pub mod tx;

/// ADC sample rate (`FSAMP`): conversions back to back with a clock divider of 0.
pub const SAMPLE_RATE_HZ: u32 = 500_000;
/// Samples per capture (`NSAMP`).
pub const NUM_SAMPLES: usize = 4096;
/// Decimation after the FIR filter (`DECIMATE_BY`).
pub const DECIMATE_BY: usize = 4;
/// Sample rate after decimation (`FSAMP_DEC`).
pub const DECIMATED_RATE_HZ: u32 = SAMPLE_RATE_HZ / DECIMATE_BY as u32;
/// Samples per capture after decimation, which is the transform length (`NSAMP_DEC`).
pub const NUM_SAMPLES_DEC: usize = NUM_SAMPLES / DECIMATE_BY;
/// Duration of one capture in microseconds (8192).
pub const CAPTURE_US: u64 = NUM_SAMPLES as u64 * 1_000_000 / SAMPLE_RATE_HZ as u64;

/// Half width of every detection band (`FFT_BIN_RANGE`). At 122 Hz bin spacing each band holds a single bin.
pub const BIN_RANGE_HZ: i32 = 61;
/// Sync tone, sent before and after every data bit.
pub const FREQ_SYNC_HZ: i32 = 50_500;
/// Tone of a `0` bit.
pub const FREQ_LOW_HZ: i32 = 51_000;
/// Tone of a `1` bit.
pub const FREQ_HIGH_HZ: i32 = 51_500;
/// Reference bands at the edges of the comms filter's pass band, used to track the noise floor
/// (`FREQ_NOISE_REF_1..4`).
pub const FREQ_NOISE_REF_HZ: [i32; 4] = [49_000, 49_500, 52_000, 52_500];

/// Duration of one transmitted symbol.
pub const SYMBOL_PERIOD_MS: u64 = 500;
/// Data bits per packet.
pub const DATA_SIZE: usize = 8;
/// Depth of the transmit queue (`tx_msg_queue`). Bytes are dropped when it is full.
pub const TX_QUEUE_DEPTH: usize = 8;
/// Minimum symbol SNR until `ivc/set_min_snr` changes it (`DEFAULT_MIN_SYMBOL_SNR`).
pub const DEFAULT_MIN_SNR: f32 = 4.0;
/// How long `ivc/debug/send_freq` (`ivc_dac/send_freq`) plays its tone.
pub const DEBUG_TONE_MS: u64 = 500;

/// One symbol observation, or a decoded symbol.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Symbol {
    /// A `0` bit ([`FREQ_LOW_HZ`]).
    Low,
    /// A `1` bit ([`FREQ_HIGH_HZ`]).
    High,
    /// The separator ([`FREQ_SYNC_HZ`]).
    Sync,
    /// Nothing clearly above the noise.
    None,
}

impl Symbol {
    /// Frequency transmitted for this symbol, if any.
    pub const fn tone_hz(self) -> Option<i32> {
        match self {
            Symbol::Low => Some(FREQ_LOW_HZ),
            Symbol::High => Some(FREQ_HIGH_HZ),
            Symbol::Sync => Some(FREQ_SYNC_HZ),
            Symbol::None => None,
        }
    }

    /// The tone selected by an `ivc/debug/send_freq` value, as `debug_tx` decoded it: 2 is sync, 0 is a `0` bit and
    /// anything else a `1` bit.
    pub const fn from_debug_value(value: u8) -> Symbol {
        match value {
            2 => Symbol::Sync,
            0 => Symbol::Low,
            _ => Symbol::High,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn capture_geometry_matches_c() {
        assert_eq!(NUM_SAMPLES_DEC, 1024);
        assert_eq!(DECIMATED_RATE_HZ, 125_000);
        assert_eq!(CAPTURE_US, 8192);
    }

    #[test]
    fn debug_values() {
        assert_eq!(Symbol::from_debug_value(0), Symbol::Low);
        assert_eq!(Symbol::from_debug_value(1), Symbol::High);
        assert_eq!(Symbol::from_debug_value(2), Symbol::Sync);
        assert_eq!(Symbol::from_debug_value(3), Symbol::High);
        assert_eq!(Symbol::from_debug_value(-1i8 as u8), Symbol::High);
    }
}
