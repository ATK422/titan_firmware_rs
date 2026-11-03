//! Packet timing, in milliseconds from the onset of the start symbol.
//!
//! The sender's schedule is the C one: the start symbol at 0, symbol `i` of the packet at `50 * (i + 1)`, and the
//! transmitter off at [`TX_END_MS`] (700 ms).
//!
//! # Receiver
//!
//! The C receiver started its symbol clock 15 ms after *noticing* the start symbol, sampled each symbol 50 ms
//! later, and sent its ACK one symbol after the last sample. The C sender sampled the ACK at 775 ms. Those two only
//! line up because the C receiver needs roughly 10 ms to notice the start symbol: its 500-point software FFT is
//! that slow. The ACK window is `[t + 715, t + 765]` for a detection latency `t`, so it only covers 775 when
//! `t >= 10`. The Rust detector is much faster, so copying the C delays would put the Rust receiver's ACK
//! before the C sender ever looks for it.
//!
//! So the Rust receiver anchors its schedule to when the detecting capture ended, not to when processing finished.
//! It samples every symbol in the middle and holds the ACK for two symbols centred on 775 ms, from
//! [`RX_ACK_START_MS`] to [`RX_ACK_END_MS`].
//!
//! # Sender, and the propagation delay
//!
//! Everything above is in the frame of whoever is speaking. Sound takes about 0.7 ms per metre, so at a range of
//! `r` the sender hears the receiver's ACK `2 r / c` later than the schedule says: 33 ms at 25 m, 120 ms at 90 m.
//! A sender that samples the ACK at one point in time therefore has a range limit. The Rust sender used to sample
//! once at 760 ms, 35 ms into a Rust ACK that began at 725 ms, so it only got ACKs from up to 17 ms (about 25 m)
//! away, while data got through to about 90 m. The C sender, sampling once at [`C_ACK_SAMPLE_MS`], gets ACKs from
//! a C receiver up to about 37 m.
//!
//! The Rust sender now listens from [`ACK_LISTEN_START_MS`] (the old sample point) to [`ACK_LISTEN_END_MS`], which
//! covers the round trip to [`MAX_RANGE_M`] plus a symbol, and takes the first symbol it hears steadily as the
//! answer (see [`crate::AckListener`]). A receiver answers before it may send anything else, so the first symbol is
//! the answer. The window opens no earlier than the single sample did, so the sender's own echo is no more of a
//! risk than it was. The next packet still goes out at [`NEXT_TX_MS`] (1410 ms), as before, however late the
//! answer came.
//!
//! # Interoperability with the C firmware
//!
//! The waveform, the sender's schedule and the packet period are unchanged. For a one-way delay `d` (in ms):
//!
//! | Sender | Receiver | ACK at the sender | ACK heard for |
//! |---|---|---|---|
//! | Rust | Rust | `[2d + 725, 2d + 825]`, listened for in `[760, 1096]` | `d` up to [`MAX_PROPAGATION_MS`] (200 m) |
//! | Rust | C (latency `t`, about 10 ms) | `[2d + t + 715, 2d + t + 765]` | the same |
//! | C (samples at 775) | Rust | `[2d + 725, 2d + 825]` | `d < 25`: about 37 m |
//! | C | C | `[2d + t + 715, 2d + t + 765]` | `2d + t < 60`: about 37 m |
//!
//! A C sender keeps its single sample and so its range. The receiving side could stretch it a little (to about
//! 50 m) by starting the ACK as soon as the packet ends instead of at 725 ms; that was tried and left out, because
//! it did slightly worse in the emulator's reverberant pool and reaching a C sender from 37 to 50 m is a niche. The
//! tests below check each row, for detector speeds from 1.5 ms per capture (the original estimate) to about 7 ms
//! (what the emulator shows).

use crate::{INTERFRAME_SPACE_MS, PACKET_SYMBOLS, SYMBOL_PERIOD_MS};

/// The sender switches the transmitter off after the start symbol and every packet symbol.
pub const TX_END_MS: u64 = (PACKET_SYMBOLS as u64 + 1) * SYMBOL_PERIOD_MS;
/// When the C sender sampled the ACK (1.5 symbols after the transmitter turned off).
pub const C_ACK_SAMPLE_MS: u64 = TX_END_MS + SYMBOL_PERIOD_MS * 3 / 2;

/// Range up to which a Rust sender waits for the ACK.
pub const MAX_RANGE_M: u64 = 200;
/// Speed of sound used for [`MAX_RANGE_M`]: the slow end for cold fresh water, so the window is long enough
/// anywhere.
pub const SOUND_SPEED_M_PER_S: u64 = 1400;
/// One-way propagation delay at [`MAX_RANGE_M`] (143 ms).
pub const MAX_PROPAGATION_MS: u64 = (MAX_RANGE_M * 1000).div_ceil(SOUND_SPEED_M_PER_S);
/// When the sender starts listening for the ACK: 60 ms after its transmitter turned off, where it used to take its
/// single sample.
pub const ACK_LISTEN_START_MS: u64 = TX_END_MS + 60;
/// When the sender gives up listening: the start shifted by the longest round trip, plus one symbol so a C
/// receiver's late ACK still fits at full range.
pub const ACK_LISTEN_END_MS: u64 = ACK_LISTEN_START_MS + 2 * MAX_PROPAGATION_MS + SYMBOL_PERIOD_MS;

/// When the sender may start its next packet (or the retransmission): [`crate::INTERFRAME_SPACE_MS`] after the old
/// single ACK sample, whenever the answer came, so packets follow each other every 1410 ms as before.
pub const NEXT_TX_MS: u64 = ACK_LISTEN_START_MS + INTERFRAME_SPACE_MS;

/// When the receiver samples packet symbol `i` (0 = heartbeat flag): in the middle of its slot.
pub const fn rx_symbol_sample_ms(i: usize) -> u64 {
    (i as u64 + 1) * SYMBOL_PERIOD_MS + SYMBOL_PERIOD_MS / 2
}

/// When the receiver starts its ACK/NACK symbol, centring it on the C sender's sample point.
pub const RX_ACK_START_MS: u64 = C_ACK_SAMPLE_MS - RX_ACK_HOLD_MS / 2;
/// How long the receiver holds its ACK/NACK symbol.
pub const RX_ACK_HOLD_MS: u64 = 2 * SYMBOL_PERIOD_MS;
/// When the receiver ends its ACK/NACK symbol.
pub const RX_ACK_END_MS: u64 = RX_ACK_START_MS + RX_ACK_HOLD_MS;

/// Duration of one capture (500 samples at 500 kS/s), in microseconds.
pub const CAPTURE_US: u64 = 1000;

/// Estimates the onset of the start symbol from the end of the capture that detected it, in microseconds on the
/// same clock as `capture_end_us`. The tone has to fill most of a capture to cross the threshold, so take half a
/// capture before its end.
pub const fn start_onset_us(capture_end_us: u64) -> u64 {
    capture_end_us.saturating_sub(CAPTURE_US / 2)
}

/// One-way propagation delay over `range_m`, in milliseconds, at [`SOUND_SPEED_M_PER_S`].
pub const fn propagation_ms(range_m: u64) -> u64 {
    (range_m * 1000).div_ceil(SOUND_SPEED_M_PER_S)
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::{AckListener, Response};
    use std::format;
    use std::vec::Vec;

    const C_RX_START_DELAY_MS: u64 = SYMBOL_PERIOD_MS * 3 / 10;
    /// Detection latencies of the C receiver to allow for. Its FFT takes about 10 ms; C to C only worked above that.
    const C_LATENCIES_MS: [u64; 4] = [10, 20, 30, 45];

    /// ACK of the C receiver for a given detection latency, in its own frame.
    fn c_receiver_ack(latency_ms: u64) -> (u64, u64) {
        let start = latency_ms + C_RX_START_DELAY_MS + (PACKET_SYMBOLS as u64 + 1) * SYMBOL_PERIOD_MS;
        (start, start + SYMBOL_PERIOD_MS)
    }

    /// The Rust receiver's ACK in its own frame, for an onset estimate that is `error_ms` late.
    fn rust_receiver_ack(error_ms: i64) -> (u64, u64) {
        ((RX_ACK_START_MS as i64 + error_ms) as u64, (RX_ACK_END_MS as i64 + error_ms) as u64)
    }

    /// What a Rust sender's listener makes of an answer that is on the air `[start, end)` in the receiver's frame,
    /// heard `2 * delay_ms` later, with captures ending every `period_us` (starting at `phase_us`).
    fn listen(answer: bool, (start, end): (u64, u64), delay_ms: u64, period_us: u64, phase_us: u64) -> Response {
        let (start, end) = ((start + 2 * delay_ms) * 1000, (end + 2 * delay_ms) * 1000);
        let mut listener = AckListener::new();
        let mut t = ACK_LISTEN_START_MS * 1000 + phase_us;
        loop {
            // A capture only sees the answer if it lies within it
            let symbol = (t >= start + CAPTURE_US && t <= end).then_some(answer);
            if let Some(response) = listener.push(t, symbol) {
                return response;
            }
            t += period_us;
        }
    }

    /// One-way delays from 0 to the maximum range.
    fn delays() -> Vec<u64> {
        (0..=MAX_PROPAGATION_MS).step_by(3).chain([MAX_PROPAGATION_MS]).collect()
    }

    /// How often the sender classifies a capture, and when the first one in the window ends: from the original
    /// estimate of the detector's speed to what the emulator shows (about 7 ms), at a few phases each.
    fn capture_rates() -> Vec<(u64, u64)> {
        [1500, 2000, 4000, 7000]
            .into_iter()
            .flat_map(|p| (0..p).step_by(p as usize / 5).map(move |ph| (p, ph)))
            .collect()
    }

    #[test]
    fn schedule_matches_c_sender() {
        assert_eq!(TX_END_MS, 700);
        assert_eq!(C_ACK_SAMPLE_MS, 775);
        assert_eq!(MAX_PROPAGATION_MS, 143);
        assert_eq!(propagation_ms(MAX_RANGE_M), MAX_PROPAGATION_MS);
        assert_eq!(ACK_LISTEN_END_MS, 1096);
        // The window closes well before the next packet
        assert_eq!(NEXT_TX_MS, 1410);
        const { assert!(ACK_LISTEN_END_MS + 100 < NEXT_TX_MS) };
    }

    #[test]
    fn rust_sender_hears_rust_receiver_to_full_range() {
        for delay in delays() {
            // The onset estimate is late by up to a capture plus its processing (the emulator shows 9 ms in the pool)
            for error in [0, 5, 10] {
                for (period, phase) in capture_rates() {
                    for answer in [true, false] {
                        let got = listen(answer, rust_receiver_ack(error), delay, period, phase);
                        let expected = if answer { Response::Ack } else { Response::Nack };
                        assert_eq!(got, expected, "delay {delay} ms, error {error} ms, period {period} us");
                    }
                }
            }
        }
    }

    #[test]
    fn rust_sender_hears_c_receiver_to_full_range() {
        for delay in delays() {
            for latency in C_LATENCIES_MS {
                for (period, phase) in capture_rates() {
                    let got = listen(true, c_receiver_ack(latency), delay, period, phase);
                    let msg = format!("delay {delay} ms, latency {latency} ms, period {period} us, phase {phase} us");
                    assert_eq!(got, Response::Ack, "{msg}");
                }
            }
        }
    }

    #[test]
    fn rust_sender_gives_up_beyond_the_window() {
        let beyond = (ACK_LISTEN_END_MS - RX_ACK_START_MS) / 2 + 1;
        assert_eq!(listen(true, rust_receiver_ack(0), beyond, 2000, 0), Response::Silence);
    }

    /// Range up to which a sender sampling once at `sample` (a capture ending then) sees an answer `[start, end)`.
    fn single_sample_range_ms(sample: u64, (start, end): (u64, u64)) -> u64 {
        (0..500).take_while(|d| start + 2 * d < sample && sample <= end + 2 * d).last().unwrap()
    }

    #[test]
    fn c_sender_range_is_unchanged() {
        // The C sender samples once at 775 ms. It hears a Rust receiver's ACK from 24 ms (about 37 m) away, the
        // same as a C receiver's (10 ms latency)...
        let rust = single_sample_range_ms(C_ACK_SAMPLE_MS, rust_receiver_ack(0));
        let c = single_sample_range_ms(C_ACK_SAMPLE_MS, c_receiver_ack(10));
        assert_eq!((rust, c), (24, 24));
        // ...while the old Rust sender's single sample at 760 ms only heard it from 17 ms (24-26 m)
        assert_eq!(single_sample_range_ms(760, rust_receiver_ack(0)), 17);
    }

    #[test]
    fn c_sender_hears_rust_receiver_up_close() {
        // At zero range the C sender's sample at 775 ms falls inside the Rust ACK even when the onset estimate is off
        for error in -20..=20 {
            let (start, end) = rust_receiver_ack(error);
            assert!(start < C_ACK_SAMPLE_MS && C_ACK_SAMPLE_MS < end, "error {error}");
        }
    }

    #[test]
    fn c_sender_needs_slow_c_receiver() {
        // The reason the receive side is re-anchored: with a fast detector the C ACK is gone by 775 ms
        let (_, end) = c_receiver_ack(2);
        assert!(end < C_ACK_SAMPLE_MS);
    }

    #[test]
    fn rust_receiver_answers_after_the_packet() {
        // The ACK starts well after the last symbol sample and the end of the packet, so the receiver never
        // transmits over the sender, and ends well after the C sender's sample point
        const { assert!(RX_ACK_START_MS > TX_END_MS) };
        const { assert!(RX_ACK_START_MS > rx_symbol_sample_ms(PACKET_SYMBOLS - 1) + 20) };
        const { assert!(RX_ACK_END_MS > C_ACK_SAMPLE_MS + 20) };
        assert_eq!((RX_ACK_START_MS, RX_ACK_END_MS), (725, 825));
    }

    #[test]
    fn listening_starts_where_the_single_sample_was() {
        // No earlier than the old sample point, so the sender's own last symbol has died away as before
        assert_eq!(ACK_LISTEN_START_MS, 760);
        // Even with no delay the answer is still on the air when listening starts, from both kinds of receiver
        assert!(rust_receiver_ack(0).1 > ACK_LISTEN_START_MS + 50);
        assert!(c_receiver_ack(10).1 >= ACK_LISTEN_START_MS + 15);
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
