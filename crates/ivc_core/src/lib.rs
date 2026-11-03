//! Hardware independent core of the IVC (inter-vehicle communication) acoustic modem.
//!
//! The IVC board sends single bytes through the water with binary frequency shift keying: a 17 kHz tone is a `0`
//! symbol and a 19 kHz tone is a `1`. Every symbol lasts [`SYMBOL_PERIOD_MS`]. A packet on the wire is:
//!
//! ```text
//! start (1) | heartbeat flag | data[7:0] | crc4[3:0]       (14 symbols, MSB first after the start symbol)
//! ```
//!
//! After the last symbol the receiver answers with a single ACK (`1`) or NACK (`0`) symbol depending on the CRC,
//! which the sender listens for with an [`AckListener`].
//! Talos (board ID pin low) sends a heartbeat packet every [`HEARTBEAT_PERIOD_MS`]; the surface vehicle (SSFB) only
//! transmits within [`SSFB_TX_WINDOW_MS`] of answering a packet from Talos, so the two never talk over each other.
//!
//! The packet format, CRC, thresholds and transmit timing are a direct port of `talos/IVCBoard/src/ivc.c`, so a
//! Rust board interoperates with a C board. The receive and ACK timing is anchored differently; see [`timing`].

#![no_std]

mod ack;
mod detector;
pub mod noise;
pub mod timing;

pub use ack::{ACK_MIN_CAPTURES, ACK_MIN_SPAN_US, AckListener, Response};
pub use detector::{Band, BandDetector};
pub use noise::NoiseFloor;

/// Frequency of a `0` symbol.
pub const FREQ_LOW_HZ: f32 = 17_000.0;
/// Frequency of a `1` symbol.
pub const FREQ_HIGH_HZ: f32 = 19_000.0;
/// Duration of one symbol.
pub const SYMBOL_PERIOD_MS: u64 = 50;

/// CRC-4 polynomial x^4 + x^3 + x + 1.
pub const CRC_POLY: u8 = 0x1B;
/// Data bits per packet.
pub const DATA_SIZE: u32 = 8;
/// CRC bits per packet.
pub const CRC_SIZE: u32 = 4;
/// Data and CRC bits per packet. The heartbeat flag sits just above them, at bit `PACKET_SIZE`.
pub const PACKET_SIZE: u32 = DATA_SIZE + CRC_SIZE;
/// Symbols following the start symbol: heartbeat flag, data and CRC.
pub const PACKET_SYMBOLS: usize = PACKET_SIZE as usize + 1;
/// Retransmissions allowed while the link is established.
pub const MAX_MISSED_PACKETS: u8 = 5;

/// Period between Talos heartbeat packets.
pub const HEARTBEAT_PERIOD_MS: u64 = 5000;
/// Data byte carried by heartbeat packets.
pub const HEARTBEAT_DATA: u8 = 0x3F;
/// Depth of the transmit queue. Packets are dropped when it is full.
pub const TX_QUEUE_DEPTH: usize = 8;

/// Delay after sampling the ACK before the next transmission or retransmission (13 symbols). The Rust sender counts
/// it from where the single sample used to be; see [`timing::NEXT_TX_MS`].
pub const INTERFRAME_SPACE_MS: u64 = (PACKET_SIZE as u64 + 1) * SYMBOL_PERIOD_MS;
/// How long SSFB may transmit after answering a packet from Talos.
pub const SSFB_TX_WINDOW_MS: u64 = DATA_SIZE as u64 * SYMBOL_PERIOD_MS;

/// Sample rate of the receive ADC.
pub const SAMPLE_RATE_HZ: f32 = 500_000.0;
/// Samples per detection window (1 ms at [`SAMPLE_RATE_HZ`]).
pub const NUM_SAMPLES: usize = 500;
/// Half width of each detection band around the symbol frequencies.
pub const BIN_RANGE_HZ: f32 = 500.0;

/// Detection thresholds for one vehicle. A symbol is detected when its band is above its threshold while the other
/// band is below its own.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Thresholds {
    pub low: f32,
    pub high: f32,
}

/// Thresholds used by the surface vehicle (SSFB).
pub const SSFB_THRESHOLDS: Thresholds = Thresholds { low: 300.0, high: 300.0 };
/// Thresholds used by Talos.
pub const TALOS_THRESHOLDS: Thresholds = Thresholds { low: 300.0, high: 300.0 };

/// The two detection bands, low symbol first.
pub const BANDS: [Band; 2] = [
    Band { min_hz: FREQ_LOW_HZ - BIN_RANGE_HZ, max_hz: FREQ_LOW_HZ + BIN_RANGE_HZ },
    Band { min_hz: FREQ_HIGH_HZ - BIN_RANGE_HZ, max_hz: FREQ_HIGH_HZ + BIN_RANGE_HZ },
];

/// CRC-4 over one data byte, bit-for-bit identical to `calculate_crc` in the C firmware.
pub fn crc4(data: u8) -> u8 {
    let mut crc: u8 = 0;
    for i in (0..DATA_SIZE).rev() {
        crc <<= 1;
        crc |= (data >> i) & 0x01;
        if crc & 0x10 != 0 {
            crc ^= CRC_POLY;
        }
    }
    crc & 0x0F
}

/// Builds the 13-bit packet word: heartbeat flag, data byte and CRC.
pub fn encode_packet(data: u8, is_heartbeat: bool) -> u16 {
    ((is_heartbeat as u16) << PACKET_SIZE) | ((data as u16) << CRC_SIZE) | crc4(data) as u16
}

/// Iterates the symbols to transmit for `word`: the start symbol followed by bit 12 down to bit 0.
pub fn packet_symbols(word: u16) -> impl Iterator<Item = bool> {
    core::iter::once(true).chain((0..PACKET_SYMBOLS as u32).rev().map(move |bit| (word >> bit) & 1 != 0))
}

/// Classifies band magnitudes as a `0` symbol, a `1` symbol, or nothing (`None`).
pub fn classify(magnitudes: [f32; 2], thresholds: Thresholds) -> Option<bool> {
    let [low, high] = magnitudes;
    if low > thresholds.low && high < thresholds.high {
        Some(false)
    } else if high > thresholds.high && low < thresholds.low {
        Some(true)
    } else {
        None
    }
}

/// A packet assembled from received symbols.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ReceivedPacket {
    pub data: u8,
    pub crc: u8,
    pub is_heartbeat: bool,
    /// True if every symbol was detected and the CRC matched. The receiver ACKs exactly when this is set.
    pub valid: bool,
}

/// Assembles the [`PACKET_SYMBOLS`] symbols following a start symbol.
///
/// The C firmware OR'd the detector's `-1` "no symbol" result straight into the packet word, which smeared ones over
/// every higher bit. Here an undetected symbol simply invalidates the packet, which leads to the same NACK without
/// relying on that accident.
#[derive(Clone, Copy, Default, Debug)]
pub struct PacketAssembler {
    word: u16,
    count: usize,
    missed_symbol: bool,
}

impl PacketAssembler {
    pub fn new() -> Self {
        Self::default()
    }

    /// Adds the next symbol. Returns the packet once all symbols have been pushed.
    pub fn push(&mut self, symbol: Option<bool>) -> Option<ReceivedPacket> {
        if self.count >= PACKET_SYMBOLS {
            return None;
        }
        match symbol {
            Some(bit) => self.word |= (bit as u16) << (PACKET_SIZE as usize - self.count),
            None => self.missed_symbol = true,
        }
        self.count += 1;
        (self.count == PACKET_SYMBOLS).then(|| self.packet())
    }

    fn packet(&self) -> ReceivedPacket {
        let data = (self.word >> CRC_SIZE) as u8;
        let crc = (self.word & 0x0F) as u8;
        ReceivedPacket {
            data,
            crc,
            is_heartbeat: self.word & (1 << PACKET_SIZE) != 0,
            valid: !self.missed_symbol && crc == crc4(data),
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::vec::Vec;

    /// Straight copy of the C implementation, kept as the reference.
    fn crc4_reference(packet: u8) -> u8 {
        let mut crc: u8 = 0;
        let mut i: i32 = DATA_SIZE as i32 - 1;
        while i >= 0 {
            crc <<= 1;
            crc |= (packet >> i) & 0x01;
            if crc & 0x10 != 0 {
                crc ^= CRC_POLY;
            }
            i -= 1;
        }
        crc & 0x0F
    }

    #[test]
    fn crc_matches_reference_for_all_bytes() {
        for b in 0..=255u8 {
            assert_eq!(crc4(b), crc4_reference(b));
        }
    }

    #[test]
    fn timing_constants_match_c() {
        assert_eq!(INTERFRAME_SPACE_MS, 650);
        assert_eq!(SSFB_TX_WINDOW_MS, 400);
    }

    #[test]
    fn symbols_round_trip_through_assembler() {
        for data in 0..=255u8 {
            for hb in [false, true] {
                let word = encode_packet(data, hb);
                let symbols: Vec<bool> = packet_symbols(word).collect();
                assert_eq!(symbols.len(), 1 + PACKET_SYMBOLS);
                assert!(symbols[0], "start symbol is a one");

                let mut asm = PacketAssembler::new();
                let mut out = None;
                for &s in &symbols[1..] {
                    out = asm.push(Some(s));
                }
                assert_eq!(out, Some(ReceivedPacket { data, crc: crc4(data), is_heartbeat: hb, valid: true }));
            }
        }
    }

    #[test]
    fn heartbeat_word_layout() {
        // HEARTBEAT_DATA << CRC_SIZE | heartbeat << PACKET_SIZE, as built in ivc_tick
        assert_eq!(encode_packet(HEARTBEAT_DATA, true) & !0xF, (1 << 12) | (0x3F << 4));
    }

    #[test]
    fn flipped_or_missing_symbol_invalidates_packet() {
        let symbols: Vec<bool> = packet_symbols(encode_packet(0x5A, false)).skip(1).collect();
        // Symbol 0 is the heartbeat flag, which the CRC does not cover
        for idx in 1..symbols.len() {
            let mut asm = PacketAssembler::new();
            let mut flipped = asm_out(&mut asm, symbols.iter().enumerate().map(|(i, &s)| Some(s ^ (i == idx))));
            assert!(!flipped.take().unwrap().valid, "flip at {idx}");

            let mut asm = PacketAssembler::new();
            let missing = asm_out(&mut asm, symbols.iter().enumerate().map(|(i, &s)| (i != idx).then_some(s)));
            assert!(!missing.unwrap().valid, "missing at {idx}");
        }
    }

    fn asm_out(asm: &mut PacketAssembler, symbols: impl Iterator<Item = Option<bool>>) -> Option<ReceivedPacket> {
        symbols.fold(None, |_, s| asm.push(s))
    }

    #[test]
    fn classification() {
        let t = TALOS_THRESHOLDS;
        assert_eq!(classify([400.0, 10.0], t), Some(false));
        assert_eq!(classify([10.0, 400.0], t), Some(true));
        assert_eq!(classify([400.0, 400.0], t), None);
        assert_eq!(classify([10.0, 10.0], t), None);
    }
}
