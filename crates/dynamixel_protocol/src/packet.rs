//! Building protocol 2.0 packets, ported from `begin_make_dxl_packet`/`add_param_to_dxl_packet`/
//! `end_make_dxl_packet` and the `dynamixel_create_*_packet` helpers of the C driver.
//!
//! ```text
//! FF FF FD 00 | ID | LEN_L LEN_H | INST | [ERR] | PARAM... | CRC_L CRC_H
//! ```
//!
//! `LEN` counts everything after itself (instruction, error, stuffed parameters and CRC). Whenever `FF FF FD` shows
//! up after the header, an extra `FD` is inserted so the receiver can't mistake it for a new header. The CRC covers
//! the stuffed packet.

use crate::crc::crc16;
use crate::{BROADCAST_ID, HEADER, Instruction};

/// Index of the instruction byte; byte stuffing applies from here on.
const INST_INDEX: usize = 7;
/// Header (4), ID, length (2), instruction and CRC (2).
const INSTRUCTION_OVERHEAD: usize = 10;
/// [`INSTRUCTION_OVERHEAD`] plus the error byte of a status packet.
const STATUS_OVERHEAD: usize = 11;

/// Why a packet could not be built.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BuildError {
    /// IDs 0xFD and 0xFF are reserved (`DXL_LIB_ERROR_INVAILD_ID`).
    InvalidId,
    /// The packet does not fit the buffer (`DXL_LIB_ERROR_BUFFER_OVERFLOW`).
    BufferOverflow,
}

/// Largest possible packet with `param_len` parameter bytes: a status packet whose parameters need the most byte
/// stuffing (one extra byte per three). Size packet buffers with this.
pub const fn max_packet_len(param_len: usize) -> usize {
    STATUS_OVERHEAD + param_len + param_len / 3
}

/// Builds one packet into a caller supplied buffer.
pub struct PacketBuilder<'a> {
    buf: &'a mut [u8],
    /// Bytes written so far.
    len: usize,
}

impl<'a> PacketBuilder<'a> {
    /// Starts an instruction packet.
    pub fn instruction(buf: &'a mut [u8], id: u8, instruction: Instruction) -> Result<Self, BuildError> {
        Self::begin(buf, id, instruction as u8, None)
    }

    /// Starts a status packet (what a servo sends back). Mostly useful for tests and simulators.
    pub fn status(buf: &'a mut [u8], id: u8, error: u8) -> Result<Self, BuildError> {
        Self::begin(buf, id, Instruction::Status as u8, Some(error))
    }

    fn begin(buf: &'a mut [u8], id: u8, instruction: u8, error: Option<u8>) -> Result<Self, BuildError> {
        // The C code meant to reject both reserved IDs but tested `id == 0xFD && id == 0xFF`, which is never true
        if id == 0xFD || id == 0xFF {
            return Err(BuildError::InvalidId);
        }
        let overhead = if error.is_some() { STATUS_OVERHEAD } else { INSTRUCTION_OVERHEAD };
        if buf.len() < overhead {
            return Err(BuildError::BufferOverflow);
        }
        buf[..4].copy_from_slice(&HEADER);
        buf[4] = id;
        buf[INST_INDEX] = instruction;
        let mut len = INST_INDEX + 1;
        if let Some(error) = error {
            buf[len] = error;
            len += 1;
        }
        Ok(PacketBuilder { buf, len })
    }

    /// Appends parameter bytes, stuffing as needed.
    pub fn param(mut self, bytes: &[u8]) -> Result<Self, BuildError> {
        for &byte in bytes {
            self.push(byte)?;
            // Look back over the instruction (and error) byte too, as the C code does
            let n = self.len;
            if n - 3 >= INST_INDEX && self.buf[n - 3..n] == [0xFF, 0xFF, 0xFD] {
                self.push(0xFD)?;
            }
        }
        Ok(self)
    }

    fn push(&mut self, byte: u8) -> Result<(), BuildError> {
        // Leave room for the CRC
        if self.len + 1 + 2 > self.buf.len() {
            return Err(BuildError::BufferOverflow);
        }
        self.buf[self.len] = byte;
        self.len += 1;
        Ok(())
    }

    /// Fills in the length and CRC and returns the finished packet.
    pub fn finish(self) -> &'a [u8] {
        // The CRC was reserved by `push`, so it always fits
        let length = (self.len + 2 - INST_INDEX) as u16;
        self.buf[5..7].copy_from_slice(&length.to_le_bytes());
        let crc = crc16(&self.buf[..self.len]);
        self.buf[self.len..self.len + 2].copy_from_slice(&crc.to_le_bytes());
        &self.buf[..self.len + 2]
    }
}

/// Ping (`dynamixel_create_ping_packet`). The servo answers with its model number and firmware version.
pub fn ping(buf: &mut [u8], id: u8) -> Result<&[u8], BuildError> {
    Ok(PacketBuilder::instruction(buf, id, Instruction::Ping)?.finish())
}

/// Reboot (`dynamixel_create_reboot_packet`).
pub fn reboot(buf: &mut [u8], id: u8) -> Result<&[u8], BuildError> {
    Ok(PacketBuilder::instruction(buf, id, Instruction::Reboot)?.finish())
}

/// Read `length` bytes of the control table from `address` (`dynamixel_create_read_packet`).
pub fn read(buf: &mut [u8], id: u8, address: u16, length: u16) -> Result<&[u8], BuildError> {
    if id == BROADCAST_ID {
        // A broadcast read would have every servo answer at once
        return Err(BuildError::InvalidId);
    }
    Ok(PacketBuilder::instruction(buf, id, Instruction::Read)?
        .param(&address.to_le_bytes())?
        .param(&length.to_le_bytes())?
        .finish())
}

/// Write `data` to the control table at `address` (`dynamixel_create_write_packet`).
pub fn write<'a>(buf: &'a mut [u8], id: u8, address: u16, data: &[u8]) -> Result<&'a [u8], BuildError> {
    Ok(PacketBuilder::instruction(buf, id, Instruction::Write)?.param(&address.to_le_bytes())?.param(data)?.finish())
}

#[cfg(test)]
mod tests {
    use super::*;

    // Expected bytes produced by the C `dxl_packet.c` (compiled on the host); the plain ones also match the examples
    // in the Robotis protocol 2.0 e-manual.

    #[test]
    fn ping_matches_c() {
        let mut buf = [0u8; 16];
        assert_eq!(ping(&mut buf, 1).unwrap(), &[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01, 0x19, 0x4E]);
        assert_eq!(ping(&mut buf, 2).unwrap(), &[0xFF, 0xFF, 0xFD, 0x00, 0x02, 0x03, 0x00, 0x01, 0x19, 0x72]);
    }

    #[test]
    fn read_matches_c() {
        let mut buf = [0u8; 32];
        assert_eq!(
            read(&mut buf, 1, 132, 4).unwrap(),
            &[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00, 0x02, 0x84, 0x00, 0x04, 0x00, 0x1D, 0x15]
        );
        // The driver's EEPROM and RAM refresh requests
        assert_eq!(
            read(&mut buf, 2, 0, 61).unwrap(),
            &[0xFF, 0xFF, 0xFD, 0x00, 0x02, 0x07, 0x00, 0x02, 0x00, 0x00, 0x3D, 0x00, 0x2B, 0xE3]
        );
        assert_eq!(
            read(&mut buf, 3, 64, 83).unwrap(),
            &[0xFF, 0xFF, 0xFD, 0x00, 0x03, 0x07, 0x00, 0x02, 0x40, 0x00, 0x53, 0x00, 0x36, 0x17]
        );
    }

    #[test]
    fn write_matches_c() {
        let mut buf = [0u8; 32];
        assert_eq!(
            write(&mut buf, 1, 116, &512u32.to_le_bytes()).unwrap(),
            &[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x09, 0x00, 0x03, 0x74, 0x00, 0x00, 0x02, 0x00, 0x00, 0xCA, 0x89]
        );
        assert_eq!(
            write(&mut buf, 2, 64, &[0]).unwrap(),
            &[0xFF, 0xFF, 0xFD, 0x00, 0x02, 0x06, 0x00, 0x03, 0x40, 0x00, 0x00, 0xEE, 0xE5]
        );
    }

    #[test]
    fn reboot_matches_c() {
        let mut buf = [0u8; 16];
        assert_eq!(reboot(&mut buf, 1).unwrap(), &[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x08, 0x2F, 0x4E]);
    }

    #[test]
    fn stuffing_matches_c() {
        let mut buf = [0u8; 32];
        assert_eq!(
            write(&mut buf, 1, 116, &[0xFF, 0xFF, 0xFD, 0x00]).unwrap(),
            &[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x0A, 0x00, 0x03, 0x74, 0x00, 0xFF, 0xFF, 0xFD, 0xFD, 0x00, 0x21, 0xE7]
        );
        assert_eq!(
            write(&mut buf, 1, 116, &[0xFF, 0xFF, 0xFD, 0xFF, 0xFF, 0xFD]).unwrap(),
            &[
                0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x0D, 0x00, 0x03, 0x74, 0x00, 0xFF, 0xFF, 0xFD, 0xFD, 0xFF, 0xFF, 0xFD,
                0xFD, 0x4F, 0x39
            ]
        );
    }

    #[test]
    fn status_matches_c() {
        let mut buf = [0u8; 32];
        let ping = PacketBuilder::status(&mut buf, 1, 0).unwrap().param(&[0x06, 0x04, 0x26]).unwrap().finish();
        assert_eq!(ping, &[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00, 0x55, 0x00, 0x06, 0x04, 0x26, 0x65, 0x5D]);

        let read = PacketBuilder::status(&mut buf, 1, 0).unwrap().param(&166u32.to_le_bytes()).unwrap().finish();
        assert_eq!(read, &[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x08, 0x00, 0x55, 0x00, 0xA6, 0x00, 0x00, 0x00, 0x8C, 0xC0]);

        let write = PacketBuilder::status(&mut buf, 1, 0).unwrap().finish();
        assert_eq!(write, &[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x04, 0x00, 0x55, 0x00, 0xA1, 0x0C]);

        let alert = PacketBuilder::status(&mut buf, 2, 0x80).unwrap().finish();
        assert_eq!(alert, &[0xFF, 0xFF, 0xFD, 0x00, 0x02, 0x04, 0x00, 0x55, 0x80, 0x2A, 0x8F]);

        let stuffed = PacketBuilder::status(&mut buf, 1, 0).unwrap().param(&[0xFF, 0xFF, 0xFD, 0x01]).unwrap().finish();
        assert_eq!(
            stuffed,
            &[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x09, 0x00, 0x55, 0x00, 0xFF, 0xFF, 0xFD, 0xFD, 0x01, 0xDD, 0x1C]
        );
    }

    #[test]
    fn rejects_reserved_ids() {
        let mut buf = [0u8; 16];
        assert_eq!(ping(&mut buf, 0xFD), Err(BuildError::InvalidId));
        assert_eq!(ping(&mut buf, 0xFF), Err(BuildError::InvalidId));
        assert_eq!(read(&mut buf, BROADCAST_ID, 0, 1), Err(BuildError::InvalidId));
        assert!(ping(&mut buf, BROADCAST_ID).is_ok());
    }

    #[test]
    fn buffer_overflow_is_reported() {
        let mut buf = [0u8; 9];
        assert_eq!(ping(&mut buf, 1), Err(BuildError::BufferOverflow));
        let mut buf = [0u8; 10];
        assert!(ping(&mut buf, 1).is_ok());
        let mut buf = [0u8; 12];
        assert_eq!(write(&mut buf, 1, 64, &[1]), Err(BuildError::BufferOverflow));
        let mut buf = [0u8; 13];
        assert_eq!(write(&mut buf, 1, 64, &[1]).map(|p| p.len()), Ok(13));
        // A stuffing byte needs room too
        let mut buf = [0u8; 15];
        assert_eq!(write(&mut buf, 1, 64, &[0xFF, 0xFF, 0xFD]), Err(BuildError::BufferOverflow));
        let mut buf = [0u8; 16];
        assert!(write(&mut buf, 1, 64, &[0xFF, 0xFF, 0xFD]).is_ok());
    }

    #[test]
    fn max_len_covers_worst_case_stuffing() {
        let data = [0xFF, 0xFF, 0xFD].repeat(10);
        let mut buf = [0u8; 64];
        let len = PacketBuilder::status(&mut buf, 1, 0).unwrap().param(&data).unwrap().finish().len();
        assert_eq!(len, max_packet_len(data.len()));
    }
}
