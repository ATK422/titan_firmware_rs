//! The transmit schedule (`tx.c`).
//!
//! A packet is one byte, sent least significant bit first, with a sync tone before and after every bit:
//!
//! ```text
//! slot:  0     1     2    3     4    ...  16   17    (off at 18)
//! tone:  sync  sync  b0   sync  b1   ...  b7   sync
//! ```
//!
//! Each slot lasts [`crate::SYMBOL_PERIOD_MS`], so a packet takes 9 s. In C, `attempt_writing` enabled the PWM and
//! the 500 ms repeating timer only programmed the first sync at the end of slot 0, so slot 0 carried whatever
//! frequency the PWM was left at: the closing sync of the previous packet, or for the first packet after boot the
//! undivided 1.26 MHz square wave. The Rust transmitter sends sync in slot 0 too, which is what every packet but the
//! first already did; the receiver cannot tell the difference.

use crate::{DATA_SIZE, SYMBOL_PERIOD_MS, Symbol};

/// Slots in one packet.
pub const PACKET_SLOTS: usize = 2 * DATA_SIZE + 2;
/// Time from the start of a packet until the transmitter turns off.
pub const PACKET_DURATION_MS: u64 = PACKET_SLOTS as u64 * SYMBOL_PERIOD_MS;

/// The tone of every slot of the packet carrying `data`.
pub fn packet_slots(data: u8) -> [Symbol; PACKET_SLOTS] {
    let mut slots = [Symbol::Sync; PACKET_SLOTS];
    for bit in 0..DATA_SIZE {
        slots[2 * bit + 2] = if (data >> bit) & 1 != 0 { Symbol::High } else { Symbol::Low };
    }
    slots
}

#[cfg(test)]
mod tests {
    use super::*;
    use Symbol::*;

    #[test]
    fn layout() {
        assert_eq!(PACKET_DURATION_MS, 9000);
        assert_eq!(
            packet_slots(0b1000_0101),
            [Sync, Sync, High, Sync, Low, Sync, High, Sync, Low, Sync, Low, Sync, Low, Sync, Low, Sync, High, Sync]
        );
    }
}
