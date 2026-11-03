//! Listening for the receiver's answer to a packet.
//!
//! The sender listens from [`timing::ACK_LISTEN_START_MS`] to [`timing::ACK_LISTEN_END_MS`] after the onset of its
//! start symbol and classifies every capture it gets to. The answer is the first symbol detected steadily: in
//! consecutive captures, at least [`ACK_MIN_CAPTURES`] of them, whose ends are at least [`ACK_MIN_SPAN_US`] apart.
//! A `1` is an ACK, a `0` a NACK. A capture without a symbol, or with the other one, starts the run again, so the
//! odd capture that noise pushes over a threshold doesn't decide anything. The first steady symbol is the right one
//! to take because a receiver answers before it sends anything else: SSFB may start a packet of its own (which
//! begins with a `1`) right after a NACK.
//!
//! The rule is in time as well as in captures because how often the sender gets to classify a capture depends on
//! how long the detector takes, which hasn't been measured on a board: the emulator shows about 7 ms per capture,
//! the original estimate was 1.5 ms. At 7 ms two captures make an answer; at 1.5 ms it takes five.
//!
//! In noise this is about as likely as the old single sample to take noise for an answer when the receiver says
//! nothing (it looks many times, but needs the symbol twice in a row), and much less likely once the stray
//! detections drop below a few per cent of captures; see the tests.

use crate::timing;

/// Fewest consecutive captures of one symbol that make an answer.
pub const ACK_MIN_CAPTURES: u8 = 2;
/// Shortest time between the ends of the first and last capture of a run that makes an answer. Even at zero range
/// a C receiver's ACK is still on the air for 15 ms when listening starts, which leaves room for two captures 7 ms
/// apart.
pub const ACK_MIN_SPAN_US: u64 = 5000;

/// What the sender heard after a packet.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Response {
    /// The receiver got the packet and its CRC matched.
    Ack,
    /// The receiver heard a packet but its CRC did not match (or a symbol was missing).
    Nack,
    /// Nothing steady was heard before the end of the window.
    Silence,
}

impl Response {
    /// True for an ACK. A NACK and silence both make the sender retransmit.
    pub fn is_ack(self) -> bool {
        self == Response::Ack
    }
}

/// Decides the answer from a stream of classified captures.
#[derive(Clone, Copy, Default, Debug)]
pub struct AckListener {
    /// The symbol of the current run, if any.
    symbol: Option<bool>,
    /// Captures in the current run.
    count: u8,
    /// When the first capture of the current run ended.
    run_start_us: u64,
}

impl AckListener {
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds a capture that ended `capture_end_us` after the onset of the packet's start symbol, classified as
    /// `symbol` (see [`crate::classify`]). Returns the answer once it is known: captures that ended before the window
    /// opened are ignored, and one that ended after it closed gives [`Response::Silence`].
    pub fn push(&mut self, capture_end_us: u64, symbol: Option<bool>) -> Option<Response> {
        if capture_end_us < timing::ACK_LISTEN_START_MS * 1000 {
            return None;
        }
        if capture_end_us > timing::ACK_LISTEN_END_MS * 1000 {
            return Some(Response::Silence);
        }
        if symbol.is_some() && symbol == self.symbol {
            self.count = self.count.saturating_add(1);
        } else {
            self.symbol = symbol;
            self.count = 1;
            self.run_start_us = capture_end_us;
        }
        let steady = self.count >= ACK_MIN_CAPTURES && capture_end_us - self.run_start_us >= ACK_MIN_SPAN_US;
        match self.symbol {
            Some(true) if steady => Some(Response::Ack),
            Some(false) if steady => Some(Response::Nack),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::timing::{ACK_LISTEN_END_MS, ACK_LISTEN_START_MS};
    use std::vec::Vec;

    const START_US: u64 = ACK_LISTEN_START_MS * 1000;

    /// Feeds `symbols` as captures every `period_us` from the start of the window.
    fn feed(period_us: u64, symbols: impl IntoIterator<Item = Option<bool>>) -> Option<Response> {
        let mut listener = AckListener::new();
        symbols.into_iter().enumerate().find_map(|(i, s)| listener.push(START_US + i as u64 * period_us, s))
    }

    /// A deterministic pseudo-random sequence in [0, 1).
    struct Lcg(u32);
    impl Lcg {
        fn next(&mut self) -> f64 {
            self.0 = self.0.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (self.0 >> 8) as f64 / (1u32 << 24) as f64
        }
    }

    #[test]
    fn steady_symbol_decides() {
        let (ack, nack) = (Some(true), Some(false));
        // Two captures are enough when they are 5 ms or more apart...
        assert_eq!(feed(7000, [ack; 2]), Some(Response::Ack));
        assert_eq!(feed(5000, [nack; 2]), Some(Response::Nack));
        assert_eq!(feed(7000, [ack]), None);
        // ...otherwise the run has to last 5 ms
        assert_eq!(feed(2000, [ack; 3]), None);
        assert_eq!(feed(2000, [ack; 4]), Some(Response::Ack));
        assert_eq!(feed(1500, [nack; 4]), None);
        assert_eq!(feed(1500, [nack; 5]), Some(Response::Nack));
        assert_eq!(feed(2000, [None, None, ack, ack, ack, ack]), Some(Response::Ack));
    }

    #[test]
    fn interrupted_runs_start_again() {
        let ack = Some(true);
        assert_eq!(feed(2000, [ack, ack, ack, None, ack, ack, ack]), None);
        assert_eq!(feed(2000, [ack, ack, ack, Some(false), ack, ack, ack]), None);
        assert_eq!(feed(2000, [ack, ack, ack, None, ack, ack, ack, ack]), Some(Response::Ack));
        assert_eq!(feed(7000, [ack, None, ack, Some(false), ack]), None);
    }

    #[test]
    fn first_steady_symbol_wins() {
        // A NACK followed straight away by the start symbol of a packet from the receiver
        for period in [1500, 2000, 7000] {
            let mut symbols = std::vec![Some(false); 20];
            symbols.extend([Some(true); 30]);
            assert_eq!(feed(period, symbols), Some(Response::Nack));
        }
    }

    #[test]
    fn window_bounds() {
        // Before the window: ignored, even a steady tone (the sender's own echo, say)
        let mut listener = AckListener::new();
        for t in (0..START_US).step_by(500) {
            assert_eq!(listener.push(t, Some(true)), None);
        }
        let mut listener = AckListener::new();
        for t in (START_US..=ACK_LISTEN_END_MS * 1000).step_by(500) {
            assert_eq!(listener.push(t, None), None, "{t}");
        }
        assert_eq!(listener.push(ACK_LISTEN_END_MS * 1000 + 1, Some(true)), Some(Response::Silence));
        // Silence and NACK are not ACKs
        assert!(Response::Ack.is_ack() && !Response::Nack.is_ack() && !Response::Silence.is_ack());
    }

    /// Classifies a capture: the answer `(bit, start_ms, end_ms)` if it is on the air and shows (probability `q`),
    /// else a stray `0` or `1` with probability `p` each.
    fn capture(rng: &mut Lcg, t_us: u64, answer: Option<(bool, u64, u64)>, p: f64, q: f64) -> Option<bool> {
        match answer.filter(|&(_, start, end)| (start * 1000..end * 1000).contains(&t_us)) {
            Some((bit, _, _)) if rng.next() < q => Some(bit),
            _ => match rng.next() {
                x if x < p => Some(false),
                x if x < 2.0 * p => Some(true),
                _ => None,
            },
        }
    }

    /// Counts the responses (ACK, NACK, silence) over 4000 windows with captures every `period_us`.
    fn trials(period_us: u64, answer: Option<(bool, u64, u64)>, p: f64, q: f64) -> [usize; 3] {
        let mut rng = Lcg(period_us as u32);
        let mut counts = [0; 3];
        for _ in 0..4000 {
            let mut listener = AckListener::new();
            let phase = (rng.next() * period_us as f64) as u64;
            let response = (0..)
                .map(|i| START_US + phase + i * period_us)
                .find_map(|t| listener.push(t, capture(&mut rng, t, answer, p, q)))
                .unwrap();
            counts[response as usize] += 1;
        }
        counts
    }

    #[test]
    fn noise_alone_rarely_decides() {
        // With stray detections of each symbol in a fraction p of captures, the old single sample took noise for
        // an ACK (or a NACK) with probability p. Listening over the window is no worse at any capture rate, and
        // much better at the faster ones.
        for p in [0.02, 0.005] {
            let single_sample = (p * 4000.0) as usize;
            for period in [1500, 2000, 4000, 7000] {
                let [ack, nack, _] = trials(period, None, p, 0.0);
                assert!(ack <= single_sample && nack <= single_sample, "{period} us, p {p}: {ack} ACK, {nack} NACK");
                if period <= 2000 {
                    assert_eq!(ack + nack, 0, "{period} us, p {p}");
                }
            }
        }
    }

    #[test]
    fn weak_answers_are_heard() {
        let periods = [1500, 2000, 4000, 7000];
        // A C receiver's 50 ms ACK at 900 ms that shows in 80 % of its captures, among 2 % stray detections, is heard
        // at least 96 % of the time
        for period in periods {
            let [ack, nack, silence] = trials(period, Some((true, 900, 950)), 0.02, 0.8);
            assert!(ack >= 3850, "{period} us: {ack} ACK, {nack} NACK, {silence} silent");
            let [ack, nack, silence] = trials(period, Some((false, 900, 950)), 0.02, 0.8);
            assert!(nack >= 3850, "{period} us: {ack} ACK, {nack} NACK, {silence} silent");
        }
        // A clean answer is always heard, even the tail of a nearby C receiver's ACK
        let clean: Vec<_> = periods.iter().map(|&period| trials(period, Some((true, 760, 775)), 0.0, 1.0)).collect();
        assert!(clean.iter().all(|&c| c == [4000, 0, 0]), "{clean:?}");
    }
}
