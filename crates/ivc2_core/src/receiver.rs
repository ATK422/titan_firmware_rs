//! Packet reception (`attempt_reading`, `handle_sample`, `rx_encode_sample`).
//!
//! A sync emitted by the [`Consensus`] starts a packet. After that every emitted `0` or `1` is shifted in, least
//! significant bit first, and the eighth completes the byte. There is no CRC and no ACK: the C protocol code for both
//! is never called.

use crate::consensus::Consensus;
use crate::{DATA_SIZE, Symbol};

/// Receive state (the `rx_control_t` fields `attempt_reading` uses).
#[derive(Clone, Copy, Debug, Default)]
pub struct Receiver {
    receiving: bool,
    data: u8,
    bits: u8,
}

impl Receiver {
    pub const fn new() -> Self {
        Receiver { receiving: false, data: 0, bits: 0 }
    }

    /// True between the opening sync and the eighth bit. The transmitter waits while this is set.
    pub fn is_receiving(&self) -> bool {
        self.receiving
    }

    /// Consumes a symbol from `consensus` and returns the byte it completes (`attempt_reading`). While our own
    /// transmitter is running (`is_writing`) the symbol is left in place, so the last one emitted during the
    /// transmission is handled once it ends, as in C.
    pub fn poll(&mut self, consensus: &mut Consensus, is_writing: bool) -> Option<u8> {
        if is_writing {
            return None;
        }
        let symbol = consensus.take()?;
        self.handle(symbol)
    }

    fn handle(&mut self, symbol: Symbol) -> Option<u8> {
        if !self.receiving {
            self.receiving = symbol == Symbol::Sync;
            return None;
        }
        let bit = match symbol {
            Symbol::Low => 0,
            Symbol::High => 1,
            Symbol::Sync | Symbol::None => return None,
        };
        self.data |= bit << self.bits;
        self.bits += 1;
        if self.bits as usize == DATA_SIZE {
            let data = self.data;
            *self = Receiver::new();
            Some(data)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::tx::{PACKET_DURATION_MS, packet_slots};
    use crate::{CAPTURE_US, SYMBOL_PERIOD_MS};
    use std::vec::Vec;

    struct Lcg(u32);

    impl Lcg {
        fn next(&mut self) -> u32 {
            self.0 = self.0.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            self.0 >> 8
        }

        /// Uniform in `0..n`.
        fn below(&mut self, n: u32) -> u32 {
            self.next() % n
        }
    }

    /// What a receiver observing the transmissions of `bytes` (back to back, with `gap_ms` of silence before
    /// each) decodes, with one observation every `period_us` ± `jitter_us`. Each observation sees the tone at the
    /// middle of its capture; observations whose capture straddles a tone change, and one in `glitch_one_in` of the
    /// rest, come out as a random symbol.
    fn decode(bytes: &[u8], gap_ms: u64, period_us: u64, jitter_us: u64, glitch_one_in: u32, seed: u32) -> Vec<u8> {
        // Tone timeline in microseconds
        let mut timeline = Vec::new();
        let mut t = 0;
        for &byte in bytes {
            t += gap_ms * 1000;
            for symbol in packet_slots(byte) {
                timeline.push((t, t + SYMBOL_PERIOD_MS * 1000, symbol));
                t += SYMBOL_PERIOD_MS * 1000;
            }
        }
        let end = t + 2_000_000;
        let tone_at = |t: u64| {
            timeline.iter().find(|&&(start, stop, _)| start <= t && t < stop).map_or(Symbol::None, |&(_, _, s)| s)
        };

        let mut rng = Lcg(seed);
        let mut consensus = Consensus::new();
        let mut receiver = Receiver::new();
        let mut out = Vec::new();
        let mut t = 0;
        while t < end {
            let first = tone_at(t);
            let last = tone_at(t + CAPTURE_US);
            let mut observation = if first == last { first } else { tone_at(t + CAPTURE_US / 2) };
            if first != last || rng.below(glitch_one_in) == 0 {
                observation = [Symbol::Low, Symbol::High, Symbol::Sync, Symbol::None][rng.below(4) as usize];
            }
            consensus.update(observation);
            if let Some(byte) = receiver.poll(&mut consensus, false) {
                out.push(byte);
            }
            t += period_us - jitter_us + rng.below(2 * jitter_us as u32 + 1) as u64;
        }
        out
    }

    #[test]
    fn decodes_every_byte() {
        let bytes: Vec<u8> = (0..=255).collect();
        for chunk in bytes.chunks(32) {
            assert_eq!(decode(chunk, 1000, 25_000, 5_000, 1_000_000, 1), chunk);
        }
    }

    #[test]
    fn decodes_across_observation_rates() {
        // From faster than the Rust firmware (about 25 ms) to slower than the C firmware (about 45 ms)
        let bytes = [0x00, 0xFF, 0xA5, 0x5A, 0x3C, 0x81];
        for period_ms in [10, 15, 25, 45, 60, 80] {
            let period = period_ms * 1000;
            for seed in 0..4 {
                assert_eq!(decode(&bytes, 700, period, period / 5, 1_000_000, seed), bytes, "{period_ms} ms");
            }
        }
    }

    #[test]
    fn tolerates_stray_observations() {
        // 1 in 40 observations wrong, on top of the ones straddling a tone change. Errors need two matching wrong
        // observations in a row, so they grow with the square of this rate.
        let bytes = [0x12, 0xED, 0x77, 0x88];
        for period_ms in [15, 25, 45] {
            let period = period_ms * 1000;
            for seed in 0..8 {
                assert_eq!(decode(&bytes, 1000, period, period / 5, 40, seed), bytes, "{period_ms} ms seed {seed}");
            }
        }
    }

    #[test]
    fn packet_ends_one_symbol_before_the_transmitter() {
        // The eighth bit is emitted as soon as the closing sync is recognised, so a few observations into the last
        // slot: the closing sync itself is never needed afterwards
        assert_eq!(PACKET_DURATION_MS, 18 * SYMBOL_PERIOD_MS);
        let mut consensus = Consensus::new();
        let mut receiver = Receiver::new();
        let mut decoded_at = None;
        for (slot, symbol) in packet_slots(0x96).into_iter().enumerate() {
            for i in 0..20 {
                consensus.update(symbol);
                if let Some(byte) = receiver.poll(&mut consensus, false) {
                    assert_eq!(byte, 0x96);
                    decoded_at = Some((slot, i));
                }
            }
        }
        assert_eq!(decoded_at, Some((17, 1)));
    }

    #[test]
    fn symbols_wait_while_transmitting() {
        let mut consensus = Consensus::new();
        let mut receiver = Receiver::new();
        for symbol in [Symbol::Sync; 3].into_iter().chain([Symbol::High; 2]) {
            consensus.update(symbol);
        }
        assert_eq!(receiver.poll(&mut consensus, true), None);
        assert!(!receiver.is_receiving());
        // The sync that ended during the transmission is handled afterwards
        assert_eq!(receiver.poll(&mut consensus, false), None);
        assert!(receiver.is_receiving());
    }
}
