//! Packet timing, in milliseconds from the onset of the start symbol.
//!
//! The sender's schedule is the C one: the start symbol at 0, symbol `i` of the packet at `50 * (i + 1)`, and the
//! transmitter off at [`TX_END_MS`] (700 ms).
//!
//! The C receiver started its symbol clock 15 ms after *noticing* the start symbol, sampled each symbol 50 ms
//! later, and sent its ACK one symbol after the last sample. The C sender sampled the ACK at 775 ms. Those two only
//! line up because the C receiver needs roughly 10 ms to notice the start symbol: its 500-point software FFT is
//! that slow. The ACK window is `[t + 715, t + 765]` for a detection latency `t`, so it only covers 775 when
//! `t >= 10`. The Rust detector finishes in about 1.5 ms, so copying the C delays would put the Rust receiver's ACK
//! before the C sender ever looks for it.
//!
//! So the Rust receiver anchors its schedule to when the detecting capture ended, not to when processing finished.
//! It samples every symbol in the middle and holds the ACK for two symbols centred on 775 ms. The Rust sender
//! samples the ACK at [`ACK_SAMPLE_MS`] (760 ms), which lies inside the C receiver's ACK for any latency from -5 to
//! 45 ms and inside the Rust receiver's ACK too. The tests below check those overlaps.

use crate::{PACKET_SYMBOLS, SYMBOL_PERIOD_MS};

/// The sender switches the transmitter off after the start symbol and every packet symbol.
pub const TX_END_MS: u64 = (PACKET_SYMBOLS as u64 + 1) * SYMBOL_PERIOD_MS;
/// When the sender samples the ACK.
pub const ACK_SAMPLE_MS: u64 = TX_END_MS + 60;
/// When the C sender sampled the ACK (1.5 symbols after the transmitter turned off).
pub const C_ACK_SAMPLE_MS: u64 = TX_END_MS + SYMBOL_PERIOD_MS * 3 / 2;

/// When the receiver samples packet symbol `i` (0 = heartbeat flag): in the middle of its slot.
pub const fn rx_symbol_sample_ms(i: usize) -> u64 {
    (i as u64 + 1) * SYMBOL_PERIOD_MS + SYMBOL_PERIOD_MS / 2
}

/// When the receiver starts its ACK/NACK symbol, centring it on the C sender's sample point.
pub const RX_ACK_START_MS: u64 = C_ACK_SAMPLE_MS - RX_ACK_HOLD_MS / 2;
/// How long the receiver holds its ACK/NACK symbol.
pub const RX_ACK_HOLD_MS: u64 = 2 * SYMBOL_PERIOD_MS;

/// Duration of one capture (500 samples at 500 kS/s), in microseconds.
pub const CAPTURE_US: u64 = 1000;

/// Estimates the onset of the start symbol from the end of the capture that detected it, in microseconds on the
/// same clock as `capture_end_us`. The tone has to fill most of a capture to cross the threshold, so take half a
/// capture before its end.
pub const fn start_onset_us(capture_end_us: u64) -> u64 {
    capture_end_us.saturating_sub(CAPTURE_US / 2)
}

#[cfg(test)]
mod tests {
    use super::*;

    const C_RX_START_DELAY_MS: u64 = SYMBOL_PERIOD_MS * 3 / 10;

    /// ACK window of the C receiver for a given detection latency.
    fn c_receiver_ack(latency_ms: u64) -> (u64, u64) {
        let start = latency_ms + C_RX_START_DELAY_MS + (PACKET_SYMBOLS as u64 + 1) * SYMBOL_PERIOD_MS;
        (start, start + SYMBOL_PERIOD_MS)
    }

    #[test]
    fn schedule_matches_c_sender() {
        assert_eq!(TX_END_MS, 700);
        assert_eq!(C_ACK_SAMPLE_MS, 775);
    }

    #[test]
    fn rust_sender_hits_c_receiver_ack() {
        for latency in 0..45 {
            let (start, end) = c_receiver_ack(latency);
            assert!(start < ACK_SAMPLE_MS && ACK_SAMPLE_MS < end, "latency {latency} ms: {start}..{end}");
        }
    }

    #[test]
    fn c_sender_needs_slow_c_receiver() {
        // The reason the receive side is re-anchored: with a fast detector the C ACK is gone by 775 ms
        let (_, end) = c_receiver_ack(2);
        assert!(end < C_ACK_SAMPLE_MS);
    }

    #[test]
    fn rust_receiver_ack_covers_both_senders_with_margin() {
        // Allow for the onset estimate being off by up to 20 ms either way
        for error in [-20i64, 0, 20] {
            let start = RX_ACK_START_MS as i64 + error;
            let end = start + RX_ACK_HOLD_MS as i64;
            for sample in [ACK_SAMPLE_MS, C_ACK_SAMPLE_MS] {
                assert!(start < sample as i64 && (sample as i64) < end, "error {error}: sample {sample}");
            }
        }
        // ...and never starts while the sender is still transmitting
        assert!(RX_ACK_START_MS > TX_END_MS);
    }

    #[test]
    fn rust_receiver_samples_inside_each_slot() {
        for i in 0..PACKET_SYMBOLS {
            let slot_start = (i as u64 + 1) * SYMBOL_PERIOD_MS;
            let sample = rx_symbol_sample_ms(i);
            assert!(sample >= slot_start + 20 && sample <= slot_start + SYMBOL_PERIOD_MS - 20);
        }
    }
}
