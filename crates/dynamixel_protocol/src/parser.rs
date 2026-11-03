//! Streaming protocol 2.0 parser, ported from `parse_dxl2_0_packet`.
//!
//! Feed bytes one at a time with [`Parser::push`]. It hunts for the `FF FF FD` header, checks the reserved byte, ID
//! and length, removes byte stuffing from the parameters and checks the CRC. Errors reset it to header hunting, like
//! the C state machine.
//!
//! One deliberate difference: the C parser looked for the `FF FF FD FD` stuffing pattern in the parameters it had
//! already de-stuffed, so parameters containing `FF FF FD FD` (sent as `FF FF FD FD FD`) lost their second `FD` and
//! the packet never completed. This parser looks at the bytes as received, which is what the e-manual specifies.

use crate::Instruction;
use crate::crc::Crc16;

/// Why a packet was rejected. The names follow the C `DXL_LIB_ERROR_*` codes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParseError {
    /// The reserved byte after the header was 0xFD (`DXL_LIB_ERROR_WRONG_PACKET`).
    WrongPacket,
    /// The ID was 0xFD or 0xFF (`DXL_LIB_ERROR_INVAILD_ID`).
    InvalidId,
    /// The length field is too small for the packet type (`DXL_LIB_ERROR_LENGTH`).
    Length,
    /// The parameters don't fit the parameter buffer (`DXL_LIB_ERROR_BUFFER_OVERFLOW`).
    BufferOverflow,
    /// CRC mismatch (`DXL_LIB_ERROR_CRC`).
    Crc,
    /// [`parse_exact`] ran out of bytes before the packet completed.
    Incomplete,
    /// [`parse_exact`] found bytes after the end of the packet.
    TrailingBytes,
}

impl ParseError {
    /// The matching C `enum DXLLibErrorCode` value, for fault reports.
    pub fn lib_code(self) -> u8 {
        match self {
            ParseError::WrongPacket => 12,
            ParseError::InvalidId => 4,
            ParseError::Length => 7,
            ParseError::BufferOverflow => 10,
            ParseError::Crc => 14,
            // DXL_LIB_PROCEEDING / DXL_LIB_OK: the C comms code reported an unexpected parser return value
            ParseError::Incomplete => 1,
            ParseError::TrailingBytes => 0,
        }
    }
}

/// A received packet. Parameters are de-stuffed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Packet<'a> {
    pub id: u8,
    pub instruction: u8,
    /// The error byte of a status packet (0 for instruction packets). Bit 7 is the hardware alert flag, bits 0-6 a
    /// [`crate::PacketError`].
    pub error: u8,
    pub params: &'a [u8],
}

impl Packet<'_> {
    /// True for status packets (replies from a servo).
    pub fn is_status(&self) -> bool {
        self.instruction == Instruction::Status as u8
    }

    /// The servo has a hardware error (`err_idx & 0x80`): read the hardware error status register for details.
    pub fn alert(&self) -> bool {
        self.error & crate::ALERT_BIT != 0
    }

    /// The error reported for the instruction, without the alert bit.
    pub fn instruction_error(&self) -> u8 {
        self.error & !crate::ALERT_BIT
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Idle,
    Reserved,
    Id,
    LengthL,
    LengthH,
    Instruction,
    Error,
    Param,
    CrcL,
    CrcH,
}

/// The streaming parser. Parameters are written into the buffer given to [`Parser::new`].
pub struct Parser<'a> {
    params: &'a mut [u8],
    state: State,
    header: [u8; 3],
    header_count: usize,
    id: u8,
    instruction: u8,
    error: u8,
    /// Value of the length field.
    length: u16,
    /// Bytes received after the length field, including removed stuffing (not counting the CRC).
    received: u16,
    param_len: usize,
    /// The last three bytes received from the instruction byte on, before de-stuffing.
    window: [u8; 3],
    crc: Crc16,
    received_crc: u16,
}

impl<'a> Parser<'a> {
    pub fn new(params: &'a mut [u8]) -> Self {
        Parser {
            params,
            state: State::Idle,
            header: [0; 3],
            header_count: 0,
            id: 0,
            instruction: 0,
            error: 0,
            length: 0,
            received: 0,
            param_len: 0,
            window: [0; 3],
            crc: Crc16::new(),
            received_crc: 0,
        }
    }

    /// Goes back to hunting for a header.
    pub fn reset(&mut self) {
        self.state = State::Idle;
        self.header_count = 0;
    }

    /// Length of the parameter section that the packet type needs before it, plus the CRC.
    fn overhead(&self) -> u16 {
        if self.instruction == Instruction::Status as u8 { 4 } else { 3 }
    }

    fn fail(&mut self, error: ParseError) -> Result<bool, ParseError> {
        self.state = State::Idle;
        Err(error)
    }

    fn shift_window(&mut self, byte: u8) {
        self.window = [self.window[1], self.window[2], byte];
    }

    /// Feeds one byte. Returns `Ok(true)` when it completed a valid packet (read it with [`Parser::packet`]),
    /// `Ok(false)` while the packet is still in progress.
    pub fn push(&mut self, byte: u8) -> Result<bool, ParseError> {
        match self.state {
            State::Idle => {
                if self.header_count >= 3 {
                    self.header_count = 0;
                }
                self.header[self.header_count] = byte;
                self.header_count += 1;
                if self.header_count == 3 {
                    if self.header == [0xFF, 0xFF, 0xFD] {
                        self.crc = Crc16::new();
                        self.crc.update_slice(&self.header);
                        self.state = State::Reserved;
                    } else {
                        self.header = [self.header[1], self.header[2], 0];
                        self.header_count -= 1;
                    }
                }
            }
            State::Reserved => {
                if byte == 0xFD {
                    return self.fail(ParseError::WrongPacket);
                }
                self.crc.update(byte);
                self.state = State::Id;
            }
            State::Id => {
                if byte >= 0xFD && byte != crate::BROADCAST_ID {
                    return self.fail(ParseError::InvalidId);
                }
                self.id = byte;
                self.crc.update(byte);
                self.state = State::LengthL;
            }
            State::LengthL => {
                self.length = byte as u16;
                self.crc.update(byte);
                self.state = State::LengthH;
            }
            State::LengthH => {
                self.length |= (byte as u16) << 8;
                self.crc.update(byte);
                if self.length < 3 {
                    return self.fail(ParseError::Length);
                }
                self.state = State::Instruction;
            }
            State::Instruction => {
                self.instruction = byte;
                self.crc.update(byte);
                self.error = 0;
                self.param_len = 0;
                self.received = 1;
                self.window = [0, 0, byte];
                let overhead = self.overhead();
                if self.length < overhead {
                    return self.fail(ParseError::Length);
                }
                // Stuffing only ever adds bytes, so the length bounds the de-stuffed parameters
                if (self.length - overhead) as usize > self.params.len() {
                    return self.fail(ParseError::BufferOverflow);
                }
                self.state = if overhead == 4 {
                    State::Error
                } else if self.length == 3 {
                    State::CrcL
                } else {
                    State::Param
                };
            }
            State::Error => {
                self.error = byte;
                self.crc.update(byte);
                self.received += 1;
                self.shift_window(byte);
                self.state = if self.length == 4 { State::CrcL } else { State::Param };
            }
            State::Param => {
                self.crc.update(byte);
                self.received += 1;
                // `FF FF FD` followed by an inserted `FD`: drop the `FD`
                let stuffing = byte == 0xFD && self.window == [0xFF, 0xFF, 0xFD];
                if stuffing {
                    // The inserted byte can't start another pattern
                    self.window = [0; 3];
                } else {
                    self.params[self.param_len] = byte;
                    self.param_len += 1;
                    self.shift_window(byte);
                }
                if self.received + 2 == self.length {
                    self.state = State::CrcL;
                }
            }
            State::CrcL => {
                self.received_crc = byte as u16;
                self.state = State::CrcH;
            }
            State::CrcH => {
                self.received_crc |= (byte as u16) << 8;
                self.state = State::Idle;
                self.header_count = 0;
                return if self.received_crc == self.crc.value() { Ok(true) } else { Err(ParseError::Crc) };
            }
        }
        Ok(false)
    }

    /// The last completed packet. Only meaningful right after [`Parser::push`] returned `Ok(true)`.
    pub fn packet(&self) -> Packet<'_> {
        Packet { id: self.id, instruction: self.instruction, error: self.error, params: &self.params[..self.param_len] }
    }
}

/// Parses exactly one packet that must span all of `bytes`, as the C comms code did with a received reply: every
/// byte but the last must leave the packet in progress and the last must complete it.
pub fn parse_exact<'a>(bytes: &[u8], params: &'a mut [u8]) -> Result<Packet<'a>, ParseError> {
    let mut parser = Parser::new(params);
    let Some((&last, body)) = bytes.split_last() else {
        return Err(ParseError::Incomplete);
    };
    for &byte in body {
        if parser.push(byte)? {
            return Err(ParseError::TrailingBytes);
        }
    }
    if !parser.push(last)? {
        return Err(ParseError::Incomplete);
    }
    let Parser { params, id, instruction, error, param_len, .. } = parser;
    Ok(Packet { id, instruction, error, params: &params[..param_len] })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::packet::{self, PacketBuilder};

    fn parse_all<'a>(bytes: &[u8], buf: &'a mut [u8]) -> Result<Packet<'a>, ParseError> {
        parse_exact(bytes, buf)
    }

    #[test]
    fn parses_e_manual_status_packets() {
        let mut buf = [0u8; 16];
        // Ping reply from an XM430-W210 (model 1030, firmware 0x26)
        let p =
            parse_all(&[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00, 0x55, 0x00, 0x06, 0x04, 0x26, 0x65, 0x5D], &mut buf)
                .unwrap();
        assert_eq!((p.id, p.instruction, p.error, p.params), (1, 0x55, 0, &[0x06, 0x04, 0x26][..]));
        assert!(p.is_status());

        let mut buf = [0u8; 16];
        let p = parse_all(
            &[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x08, 0x00, 0x55, 0x00, 0xA6, 0x00, 0x00, 0x00, 0x8C, 0xC0],
            &mut buf,
        )
        .unwrap();
        assert_eq!(i32::from_le_bytes(p.params.try_into().unwrap()), 166);

        let mut buf = [0u8; 16];
        let p = parse_all(&[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x04, 0x00, 0x55, 0x00, 0xA1, 0x0C], &mut buf).unwrap();
        assert!(p.params.is_empty());
        assert!(!p.alert());
    }

    #[test]
    fn alert_bit_is_reported() {
        let mut buf = [0u8; 4];
        let p = parse_all(&[0xFF, 0xFF, 0xFD, 0x00, 0x02, 0x04, 0x00, 0x55, 0x80, 0x2A, 0x8F], &mut buf).unwrap();
        assert!(p.alert());
        assert_eq!(p.instruction_error(), 0);
    }

    #[test]
    fn parses_instruction_packets() {
        let mut out = [0u8; 32];
        let pkt = packet::read(&mut out, 3, 64, 83).unwrap();
        let mut buf = [0u8; 8];
        let p = parse_all(pkt, &mut buf).unwrap();
        assert_eq!((p.id, p.instruction, p.params), (3, Instruction::Read as u8, &[64, 0, 83, 0][..]));
        assert!(!p.is_status());

        let mut out = [0u8; 16];
        let pkt = packet::ping(&mut out, 7).unwrap();
        let mut buf = [0u8; 0];
        let p = parse_all(pkt, &mut buf).unwrap();
        assert_eq!((p.id, p.instruction, p.params.len()), (7, 1, 0));
    }

    #[test]
    fn round_trips_with_stuffing() {
        let cases: &[&[u8]] = &[
            &[],
            &[0xFF, 0xFF, 0xFD],
            &[0xFF, 0xFF, 0xFD, 0x01],
            &[0xFF, 0xFF, 0xFD, 0xFD],
            &[0xFF, 0xFF, 0xFD, 0xFD, 0xFD],
            &[0xFF, 0xFF, 0xFD, 0xFF, 0xFF, 0xFD],
            &[0x00, 0xFF, 0xFF, 0xFD, 0xFD, 0x07],
            &[0xFF, 0xFD, 0xFF, 0xFF, 0xFF, 0xFD, 0xFD, 0xFF],
        ];
        for &data in cases {
            for error in [None, Some(0x00), Some(0xFF), Some(0x80)] {
                let mut out = [0u8; 64];
                let builder = match error {
                    Some(e) => PacketBuilder::status(&mut out, 5, e).unwrap(),
                    None => PacketBuilder::instruction(&mut out, 5, Instruction::Write).unwrap(),
                };
                let pkt = builder.param(data).unwrap().finish();
                let mut buf = [0u8; 32];
                let p = parse_all(pkt, &mut buf).unwrap_or_else(|e| panic!("{data:02X?} {error:?}: {e:?}"));
                assert_eq!(p.params, data, "{error:?}");
                assert_eq!(p.error, error.unwrap_or(0));
            }
        }
    }

    #[test]
    fn exhaustive_short_round_trips() {
        // Every 4 byte parameter string over the interesting byte values
        let values = [0x00, 0xFD, 0xFF];
        for i in 0..81usize {
            let data = [values[i % 3], values[i / 3 % 3], values[i / 9 % 3], values[i / 27 % 3]];
            let mut out = [0u8; 32];
            let pkt = PacketBuilder::status(&mut out, 1, 0).unwrap().param(&data).unwrap().finish();
            let mut buf = [0u8; 8];
            assert_eq!(parse_all(pkt, &mut buf).unwrap().params, data);
        }
    }

    #[test]
    fn skips_garbage_before_header() {
        let mut stream = [0u8; 20];
        stream[..5].copy_from_slice(&[0x00, 0xFF, 0x12, 0xFF, 0xFF]);
        stream[5..16].copy_from_slice(&[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x04, 0x00, 0x55, 0x00, 0xA1, 0x0C]);
        let mut buf = [0u8; 4];
        let mut parser = Parser::new(&mut buf);
        let mut done = 0;
        for &b in &stream[..16] {
            if parser.push(b).unwrap() {
                done += 1;
            }
        }
        assert_eq!(done, 1);
        assert_eq!(parser.packet().id, 1);
    }

    #[test]
    fn detects_errors() {
        let good = [0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x08, 0x00, 0x55, 0x00, 0xA6, 0x00, 0x00, 0x00, 0x8C, 0xC0];
        let mut buf = [0u8; 8];

        let mut bad = good;
        bad[9] ^= 1;
        assert_eq!(parse_all(&bad, &mut buf), Err(ParseError::Crc));

        let mut bad = good;
        bad[3] = 0xFD;
        assert_eq!(parse_all(&bad, &mut buf), Err(ParseError::WrongPacket));

        let mut bad = good;
        bad[4] = 0xFF;
        assert_eq!(parse_all(&bad, &mut buf), Err(ParseError::InvalidId));

        let mut bad = good;
        bad[5] = 0x02;
        assert_eq!(parse_all(&bad, &mut buf), Err(ParseError::Length));

        // A status packet needs room for the error byte
        let mut bad = good;
        bad[5] = 0x03;
        assert_eq!(parse_all(&bad, &mut buf), Err(ParseError::Length));

        let mut small = [0u8; 3];
        assert_eq!(parse_all(&good, &mut small), Err(ParseError::BufferOverflow));

        assert_eq!(parse_all(&good[..14], &mut buf), Err(ParseError::Incomplete));
        let mut long = [0u8; 16];
        long[..15].copy_from_slice(&good);
        assert_eq!(parse_all(&long, &mut buf), Err(ParseError::TrailingBytes));
        assert_eq!(parse_all(&[], &mut buf), Err(ParseError::Incomplete));
    }

    #[test]
    fn recovers_after_error() {
        let good = [0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x04, 0x00, 0x55, 0x00, 0xA1, 0x0C];
        let mut bad = good;
        bad[9] = 0;
        let mut buf = [0u8; 4];
        let mut parser = Parser::new(&mut buf);
        let mut results = [None; 2];
        for (i, pkt) in [bad, good].iter().enumerate() {
            for &b in pkt {
                match parser.push(b) {
                    Ok(true) => results[i] = Some(Ok(())),
                    Err(e) => results[i] = Some(Err(e)),
                    Ok(false) => {}
                }
            }
        }
        assert_eq!(results, [Some(Err(ParseError::Crc)), Some(Ok(()))]);
    }
}
