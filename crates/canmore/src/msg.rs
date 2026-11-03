//! Message fragmentation and re-assembly.
//!
//! A message is sent as a sequence of frames numbered by the NOC field. Frame 0 is always extended and carries the
//! length and subtype. If the message needs more frames, the middle frames are standard and the last frame is
//! extended with the CRC-18 of the whole message in its extra field. Classic CAN rolls the sequence number from 15
//! back to 1; CAN FD never needs to (16 * 64 bytes covers the 1024 byte maximum).

use crate::id::{self, Direction, Fields, FrameType};
use crate::{Frame, MAX_FD_FRAME_SIZE, MAX_FRAME_SIZE, MAX_MSG_LENGTH, crc18, fd_frame_len};

const MAX_SEQ_NUM: u8 = 15;

/// Splits a message into frames. Iterate it to get the frames in transmit order.
pub struct MsgEncoder<'a> {
    client_id: u8,
    direction: Direction,
    use_canfd: bool,
    subtype: u8,
    buf: &'a [u8],
    position: usize,
    seq_num: u8,
}

impl<'a> MsgEncoder<'a> {
    /// Prepares `buf` for transmission. Messages longer than [`MAX_MSG_LENGTH`] are truncated, like the C encoder.
    pub fn new(client_id: u8, direction: Direction, use_canfd: bool, subtype: u8, buf: &'a [u8]) -> Self {
        let buf = &buf[..buf.len().min(MAX_MSG_LENGTH)];
        MsgEncoder { client_id, direction, use_canfd, subtype, buf, position: 0, seq_num: 0 }
    }
}

impl Iterator for MsgEncoder<'_> {
    type Item = Frame;

    fn next(&mut self) -> Option<Frame> {
        if self.position == self.buf.len() {
            return None;
        }

        let max_frame_len = if self.use_canfd { MAX_FD_FRAME_SIZE } else { MAX_FRAME_SIZE };
        let copy_size = (self.buf.len() - self.position).min(max_frame_len);
        let chunk = &self.buf[self.position..self.position + copy_size];
        self.position += copy_size;

        // CAN FD can only carry certain lengths, pad with zeros up to the next one
        let mut payload = [0u8; MAX_FD_FRAME_SIZE];
        payload[..copy_size].copy_from_slice(chunk);
        let frame_len = if self.use_canfd { fd_frame_len(copy_size) } else { copy_size };

        let is_last = self.position == self.buf.len();
        let (frame_id, extended) = if self.seq_num == 0 {
            let extra = id::msg_first_extra(is_last, self.buf.len(), self.subtype);
            (id::ext_id(self.client_id, FrameType::Msg, self.direction, 0, extra), true)
        } else if is_last {
            let crc = crc18::update(crc18::INITIAL, self.buf) & crc18::MASK;
            (id::ext_id(self.client_id, FrameType::Msg, self.direction, self.seq_num, crc), true)
        } else {
            (id::std_id(self.client_id, FrameType::Msg, self.direction, self.seq_num), false)
        };

        self.seq_num = if self.seq_num >= MAX_SEQ_NUM { 1 } else { self.seq_num + 1 };

        Frame::new(frame_id, extended, &payload[..frame_len])
    }
}

/// Why a frame was rejected by [`MsgDecoder`]. The partially decoded message is dropped on every error.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DecodeError {
    /// Sequence number did not follow the previous frame.
    BadSeqNum,
    /// A frame with sequence number 0 was not extended.
    SeqZeroNotExtended,
    /// The declared length was 0 or larger than [`MAX_MSG_LENGTH`], or more data arrived than declared.
    MsgTooLarge,
    /// A frame carried no data.
    ZeroLengthPacket,
    /// More data arrived than the declared length before the last frame.
    UnexpectedData,
    /// The last frame arrived before the declared length was reached.
    MsgTooSmall,
    /// The CRC of the re-assembled message did not match.
    CrcFail,
    /// A CAN FD message tried to roll the sequence number over.
    RolloverWithCanfd,
}

/// A fully re-assembled message.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Message<'a> {
    pub subtype: u8,
    pub data: &'a [u8],
}

/// Re-assembles messages from a single sender. Feed it every message frame from that sender in receive order.
pub struct MsgDecoder {
    use_canfd: bool,
    buf: [u8; MAX_MSG_LENGTH],
    decode_len: usize,
    expected_len: usize,
    subtype: u8,
    next_seq_num: u8,
    crc: u32,
}

impl MsgDecoder {
    pub const fn new(use_canfd: bool) -> Self {
        MsgDecoder {
            use_canfd,
            buf: [0; MAX_MSG_LENGTH],
            decode_len: 0,
            expected_len: 0,
            subtype: 0,
            next_seq_num: 0,
            crc: crc18::INITIAL,
        }
    }

    pub fn reset(&mut self) {
        self.crc = crc18::INITIAL;
        self.decode_len = 0;
        self.next_seq_num = 0;
    }

    fn fail(&mut self, err: DecodeError) -> Result<Option<Message<'_>>, DecodeError> {
        self.reset();
        Err(err)
    }

    /// Feeds one message frame. Returns the message once its last frame arrives.
    pub fn decode_frame(
        &mut self,
        can_id: u32,
        extended: bool,
        frame: &[u8],
    ) -> Result<Option<Message<'_>>, DecodeError> {
        let fields = Fields::decode(can_id, extended);
        let is_last;
        let is_single;
        let mut crc_expected = None;

        if fields.noc == 0 {
            // A start frame re-synchronizes the decoder, discarding any partial message
            self.reset();
            if !extended {
                return self.fail(DecodeError::SeqZeroNotExtended);
            }
            let len = fields.msg_len();
            if len == 0 || len > MAX_MSG_LENGTH {
                return self.fail(DecodeError::MsgTooLarge);
            }
            self.subtype = fields.msg_subtype();
            self.expected_len = len;
            is_last = fields.msg_single();
            is_single = is_last;
        } else {
            is_last = extended;
            is_single = false;
            if extended {
                crc_expected = Some(fields.extra);
            }
            if fields.noc != self.next_seq_num {
                return self.fail(DecodeError::BadSeqNum);
            }
        }

        if frame.is_empty() {
            return self.fail(DecodeError::ZeroLengthPacket);
        }

        let mut copy_len = frame.len();
        if frame.len() + self.decode_len > self.expected_len {
            // CAN FD pads the last frame up to a valid frame length, so trim it back down
            if !is_last || !self.use_canfd {
                return self.fail(DecodeError::UnexpectedData);
            }
            copy_len = self.expected_len - self.decode_len;
        }

        let chunk = &frame[..copy_len];
        if !is_single {
            self.crc = crc18::update(self.crc, chunk);
        }
        self.buf[self.decode_len..self.decode_len + copy_len].copy_from_slice(chunk);
        self.decode_len += copy_len;

        if is_last {
            if self.decode_len < self.expected_len {
                return self.fail(DecodeError::MsgTooSmall);
            }
            if let Some(crc_expected) = crc_expected
                && self.crc & crc18::MASK != crc_expected
            {
                return self.fail(DecodeError::CrcFail);
            }
            let len = self.decode_len;
            self.reset();
            Ok(Some(Message { subtype: self.subtype, data: &self.buf[..len] }))
        } else {
            if self.decode_len >= self.expected_len {
                return self.fail(DecodeError::MsgTooLarge);
            }
            if self.next_seq_num >= MAX_SEQ_NUM {
                if self.use_canfd {
                    return self.fail(DecodeError::RolloverWithCanfd);
                }
                self.next_seq_num = 1;
            } else {
                self.next_seq_num += 1;
            }
            Ok(None)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::vec::Vec;

    fn pattern(len: usize) -> Vec<u8> {
        (0..len).map(|i| (i * 7 + 3) as u8).collect()
    }

    fn round_trip(len: usize, use_canfd: bool) {
        let data = pattern(len);
        let frames: Vec<Frame> = MsgEncoder::new(8, Direction::ClientToAgent, use_canfd, 1, &data).collect();
        let mut decoder = MsgDecoder::new(use_canfd);
        let mut decoded = None;
        for (i, frame) in frames.iter().enumerate() {
            let out = decoder.decode_frame(frame.id, frame.extended, frame.data()).unwrap();
            if i + 1 == frames.len() {
                decoded = out.map(|m| (m.subtype, m.data.to_vec()));
            } else {
                assert!(out.is_none());
            }
        }
        assert_eq!(decoded, Some((1, data)), "len {len} canfd {use_canfd}");
    }

    #[test]
    fn round_trips() {
        for len in [1, 7, 8, 9, 63, 64, 65, 100, 127, 128, 129, 500, 1023, 1024] {
            round_trip(len, false);
            round_trip(len, true);
        }
    }

    #[test]
    fn frame_shapes() {
        let data = pattern(20);
        let frames: Vec<Frame> = MsgEncoder::new(8, Direction::ClientToAgent, false, 0, &data).collect();
        assert_eq!(frames.len(), 3);
        assert!(frames[0].extended && !frames[1].extended && frames[2].extended);
        assert_eq!(frames[2].data().len(), 4);

        // Single CAN FD frame, padded from 17 to 20 bytes
        let data = pattern(17);
        let frames: Vec<Frame> = MsgEncoder::new(8, Direction::ClientToAgent, true, 0, &data).collect();
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].data().len(), 20);
        assert!(Fields::decode(frames[0].id, true).msg_single());
    }

    #[test]
    fn classic_can_rolls_sequence_over() {
        let data = pattern(200);
        let seqs: Vec<u8> = MsgEncoder::new(1, Direction::AgentToClient, false, 0, &data)
            .map(|f| Fields::decode(f.id, f.extended).noc)
            .collect();
        assert_eq!(&seqs[..17], &[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 1]);
    }

    #[test]
    fn corrupted_message_is_rejected() {
        let data = pattern(30);
        let mut frames: Vec<Frame> = MsgEncoder::new(8, Direction::ClientToAgent, false, 0, &data).collect();
        let mut tampered = frames[1].data().to_vec();
        tampered[0] ^= 0xFF;
        frames[1] = Frame::new(frames[1].id, false, &tampered).unwrap();

        let mut decoder = MsgDecoder::new(false);
        let mut result = Ok(None);
        for frame in &frames {
            result = decoder.decode_frame(frame.id, frame.extended, frame.data()).map(|m| m.map(|m| m.data.len()));
        }
        assert_eq!(result, Err(DecodeError::CrcFail));
    }

    #[test]
    fn dropped_frame_is_rejected() {
        let data = pattern(30);
        let frames: Vec<Frame> = MsgEncoder::new(8, Direction::ClientToAgent, false, 0, &data).collect();
        let mut decoder = MsgDecoder::new(false);
        decoder.decode_frame(frames[0].id, true, frames[0].data()).unwrap();
        let err = decoder.decode_frame(frames[2].id, frames[2].extended, frames[2].data()).unwrap_err();
        assert_eq!(err, DecodeError::BadSeqNum);
    }
}
