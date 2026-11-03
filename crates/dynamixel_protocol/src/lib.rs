//! Hardware independent part of the Dynamixel driver: protocol 2.0 packets and the XL430 control table.
//!
//! * [`crc`]: the protocol's CRC-16
//! * [`packet`]: building instruction (and status) packets with byte stuffing, from the C `dxl_packet.c` and
//!   `dynamixel_comms.c` packet helpers
//! * [`parser`]: a streaming parser that removes the stuffing and checks the CRC (`parse_dxl2_0_packet`)
//! * [`control_table`]: register addresses and decoding of the EEPROM and RAM blocks (`dynamixel_controls.h`,
//!   `dynamixel_reg.c`)
//!
//! Only protocol 2.0 is supported, as only 2.0 was used by the C driver. See
//! <https://emanual.robotis.com/docs/en/dxl/protocol2/>.

#![cfg_attr(not(test), no_std)]

pub mod control_table;
pub mod crc;
pub mod packet;
pub mod parser;

pub use control_table::{Eeprom, Ram};
pub use packet::{BuildError, PacketBuilder, max_packet_len};
pub use parser::{Packet, ParseError, Parser, parse_exact};

/// Start of every packet: the three header bytes and the reserved byte.
pub const HEADER: [u8; 4] = [0xFF, 0xFF, 0xFD, 0x00];
/// Packets to this ID reach every servo (`DXL_BROADCAST_ID`).
pub const BROADCAST_ID: u8 = 0xFE;
/// Highest ID a servo can have.
pub const MAX_ID: u8 = 0xFC;
/// Bit 7 of a status packet's error byte: the servo has a hardware error.
pub const ALERT_BIT: u8 = 0x80;

/// Instruction byte (`enum DXLInstruction`, protocol 2.0 values).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Instruction {
    Ping = 0x01,
    Read = 0x02,
    Write = 0x03,
    RegWrite = 0x04,
    Action = 0x05,
    FactoryReset = 0x06,
    Reboot = 0x08,
    Clear = 0x10,
    /// A servo's reply.
    Status = 0x55,
    SyncRead = 0x82,
    SyncWrite = 0x83,
    FastSyncRead = 0x8A,
    BulkRead = 0x92,
    BulkWrite = 0x93,
}

/// Error reported in a status packet, without the alert bit (`enum DXL2_0PacketError`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum PacketError {
    ResultFail = 1,
    InstructionError = 2,
    CrcError = 3,
    DataRange = 4,
    DataLength = 5,
    DataLimit = 6,
    Access = 7,
}

impl PacketError {
    /// Decodes the error byte of a status packet; `None` when there is no error (the alert bit is ignored).
    pub fn from_status(error: u8) -> Option<Result<Self, u8>> {
        Some(match error & !ALERT_BIT {
            0 => return None,
            1 => Ok(PacketError::ResultFail),
            2 => Ok(PacketError::InstructionError),
            3 => Ok(PacketError::CrcError),
            4 => Ok(PacketError::DataRange),
            5 => Ok(PacketError::DataLength),
            6 => Ok(PacketError::DataLimit),
            7 => Ok(PacketError::Access),
            other => Err(other),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn packet_error_decoding() {
        assert_eq!(PacketError::from_status(0x00), None);
        assert_eq!(PacketError::from_status(0x80), None);
        assert_eq!(PacketError::from_status(0x87), Some(Ok(PacketError::Access)));
        assert_eq!(PacketError::from_status(0x04), Some(Ok(PacketError::DataRange)));
        assert_eq!(PacketError::from_status(0x09), Some(Err(9)));
    }

    #[test]
    fn eeprom_and_ram_reads_fit_the_driver_buffer() {
        // The C driver sized its buffers for 128 parameter bytes
        assert!(max_packet_len(control_table::RAM_LENGTH as usize) <= 128 + 11 + 128 / 3);
        const { assert!(control_table::RAM_LENGTH <= 128 && control_table::EEPROM_LENGTH <= 128) };
    }
}
