//! Debouncing of symbol observations (`consensus.c`).
//!
//! An observation that repeats the current symbol makes it more stable. Once the current symbol has been seen
//! [`MIN_STABLE_SAMPLES`] times, [`MAX_SAMPLE_GAP`] consecutive observations of another symbol switch to it, and the
//! symbol that just ended is emitted. Observations of [`Symbol::None`] are ignored entirely, so silence neither ends
//! nor emits a symbol. A symbol is therefore emitted when the next one begins; that is why every data bit is followed
//! by a sync.

use crate::Symbol;

/// Consecutive observations before a symbol counts as stable (`MIN_STABLE_SAMPLES`).
pub const MIN_STABLE_SAMPLES: u8 = 3;
/// Consecutive observations of a new symbol needed to switch to it (`MAX_SAMPLE_GAP`).
pub const MAX_SAMPLE_GAP: u8 = 2;

/// The consensus state (`rx_consensus_t`). Starts zeroed like the C context, which makes the current symbol
/// [`Symbol::Low`].
#[derive(Clone, Copy, Debug)]
pub struct Consensus {
    stable_count: u8,
    gap_count: u8,
    last_symbol: Symbol,
    pending: Symbol,
    current: Symbol,
    ready: bool,
}

impl Default for Consensus {
    fn default() -> Self {
        Self::new()
    }
}

impl Consensus {
    pub const fn new() -> Self {
        Consensus {
            stable_count: 0,
            gap_count: 0,
            last_symbol: Symbol::Low,
            pending: Symbol::Low,
            current: Symbol::Low,
            ready: false,
        }
    }

    /// Adds one observation (`consensus_update`).
    pub fn update(&mut self, observation: Symbol) {
        if observation == Symbol::None {
            return;
        }
        if observation == self.current {
            // The C counter was a uint8_t that wrapped after 255 repeats (about 6 s of one tone at the Rust
            // observation rate), which made the next change restart instead of emitting. Saturate instead.
            self.stable_count = self.stable_count.saturating_add(1);
            self.gap_count = 0;
        } else if self.stable_count >= MIN_STABLE_SAMPLES {
            if observation == self.pending {
                self.gap_count += 1;
            } else {
                self.pending = observation;
                self.gap_count = 1;
            }
            if self.gap_count >= MAX_SAMPLE_GAP {
                self.last_symbol = self.current;
                self.current = self.pending;
                self.stable_count = 1;
                self.gap_count = 0;
                self.ready = true;
            }
        } else {
            self.stable_count = 1;
            self.gap_count = 0;
            self.current = observation;
            self.pending = observation;
        }
    }

    /// Takes the symbol that ended at the last transition, if it has not been taken yet (`sample_ready`).
    pub fn take(&mut self) -> Option<Symbol> {
        core::mem::take(&mut self.ready).then_some(self.last_symbol)
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use Symbol::*;
    use std::vec::Vec;

    fn run(observations: &[Symbol]) -> Vec<Symbol> {
        let mut c = Consensus::new();
        observations
            .iter()
            .filter_map(|&o| {
                c.update(o);
                c.take()
            })
            .collect()
    }

    #[test]
    fn emits_symbol_when_the_next_one_starts() {
        assert_eq!(run(&[Sync, Sync, Sync, High, High]), [Sync]);
        // The switch counts as the first stable observation of the new symbol, so it needs two more
        assert_eq!(run(&[Sync, Sync, Sync, High, High, High, High, Sync, Sync]), [Sync, High]);
        assert_eq!(run(&[Sync, Sync, Sync, High, High, High, Sync, Sync]), [Sync]);
    }

    #[test]
    fn needs_a_stable_symbol_before_switching() {
        // Two syncs are not stable yet, so the highs simply replace them
        assert_eq!(run(&[Sync, Sync, High, High, High, Sync, Sync]), [High]);
    }

    #[test]
    fn single_glitch_is_ignored() {
        assert_eq!(run(&[Sync, Sync, Sync, Low, Sync, Sync, Low, Sync, High, High]), [Sync]);
    }

    #[test]
    fn interrupted_gap_restarts() {
        // High, Low, High: the pending symbol keeps changing and never reaches two in a row
        assert_eq!(run(&[Sync, Sync, Sync, High, Low, High, Low]), []);
    }

    #[test]
    fn silence_is_ignored() {
        assert_eq!(run(&[Sync, None, Sync, None, None, Sync, None, High, None, High]), [Sync]);
    }

    #[test]
    fn starts_out_as_low() {
        // The zeroed C context: a run of lows is already the current symbol
        assert_eq!(run(&[Low, Low, Low, Sync, Sync]), [Low]);
    }

    #[test]
    fn long_tone_does_not_wrap() {
        let mut observations = std::vec![Sync; 300];
        observations.extend([High, High]);
        assert_eq!(run(&observations), [Sync]);
    }

    #[test]
    fn ready_symbol_waits_to_be_taken() {
        let mut c = Consensus::new();
        for o in [Sync, Sync, Sync, High, High, High, High] {
            c.update(o);
        }
        assert_eq!(c.take(), Some(Sync));
        assert_eq!(c.take(), Option::None);
    }
}
