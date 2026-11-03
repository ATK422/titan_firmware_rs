//! Register mapped protocol used by the CANmore control interface (channel 14), ported from `canmore/reg_mapped`.
//!
//! The protocol exposes a reliable "read/write a 32-bit register" interface over an unreliable packet link (a CAN
//! utility channel or a UDP socket). Every register has a 16-bit address split into a page and an offset. Pages
//! group related registers: some pages are a set of individual registers (a magic number, a command trigger, ...),
//! others are a contiguous block of memory such as a flash buffer or a version string.
//!
//! The agent (the computer, running [`Client`]) sends a request and the client device (running [`Server`]) answers
//! every request with a response. Three request kinds exist:
//!
//! * **Single** reads and writes of one register.
//! * **Bulk** writes: a run of write requests with an increasing sequence number that the server only answers once,
//!   after the request flagged as the bulk end. This hides the round trip latency on classic CAN.
//! * **Multiword** reads and writes: several consecutive words of a memory page in one packet. Only useful when the
//!   link carries more than 8 bytes (CAN FD, UDP).
//!
//! All multibyte fields are little endian. The wire format and the server's error semantics match
//! `reg_mapped/protocol.h` and `reg_mapped_server.c`, so this server is a drop-in replacement for the C firmware as
//! far as `canmore_cli` and the upload tool are concerned, and this client talks to the existing C firmware.
//!
//! ```text
//! Request:  | Flags | Count | Page | Offset | Data word 0 (writes) | Data word 1 (multiword) | ...
//! Flags:    | Mode (7:5) | RFU (4) | Multiword (3) | Bulk end (2) | Bulk (1) | Write (0) |
//! Response: | Result | Data word (reads) / Seq no (bulk end) / Data words (multiword reads) |
//! ```
//!
//! The register layouts implemented on top of this protocol live in [`interface`].

mod client;
pub mod interface;
mod server;

pub use client::{Client, ClientError, TransferMode, Transport};
pub use server::{Access, MemoryPage, Perm, RegisterMap, Server};

/// Number of 32-bit words in a page.
pub const PAGE_NUM_WORDS: usize = 0x100;
/// Size of a page in bytes.
pub const PAGE_SIZE: usize = PAGE_NUM_WORDS * 4;

/// Length of a request header (flags, count, page, offset).
pub const REQUEST_HEADER_LEN: usize = 4;
/// Length of a read request.
pub const READ_REQUEST_LEN: usize = REQUEST_HEADER_LEN;
/// Length of a single or bulk write request.
pub const WRITE_REQUEST_LEN: usize = REQUEST_HEADER_LEN + 4;
/// Length of a write response.
pub const WRITE_RESPONSE_LEN: usize = 1;
/// Length of the response that ends a bulk write.
pub const BULK_RESPONSE_LEN: usize = 2;
/// Length of a single read response.
pub const READ_RESPONSE_LEN: usize = 5;
/// Smallest response buffer [`Server::handle_request`] accepts (one read response).
pub const MIN_RESPONSE_BUFFER_LEN: usize = READ_RESPONSE_LEN;

/// Length of a multiword write request carrying `word_count` words.
pub const fn multiword_request_len(word_count: usize) -> usize {
    REQUEST_HEADER_LEN + 4 * word_count
}

/// Length of a successful multiword read response carrying `word_count` words.
pub const fn multiword_response_len(word_count: usize) -> usize {
    1 + 4 * word_count
}

/// Raw packet sent on the control interface channel to a device in boot delay mode to stop it in the bootloader.
///
/// This is not a register mapped request: a device in [`ControlInterfaceMode::BootDelay`](crate::ControlInterfaceMode)
/// only waits for this exact packet.
pub const BOOTLOADER_REQUEST: [u8; 4] = [0xB0, 0x07, 0x10, 0xAD];

/// Request flag byte.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct Flags {
    pub write: bool,
    pub bulk: bool,
    pub bulk_end: bool,
    pub multiword: bool,
    /// The control interface mode the request is meant for (see [`crate::ControlInterfaceMode`]).
    pub mode: u8,
}

impl Flags {
    pub const fn to_byte(self) -> u8 {
        (self.write as u8)
            | (self.bulk as u8) << 1
            | (self.bulk_end as u8) << 2
            | (self.multiword as u8) << 3
            | (self.mode & 0b111) << 5
    }

    pub const fn from_byte(byte: u8) -> Self {
        Flags {
            write: byte & 1 != 0,
            bulk: byte & (1 << 1) != 0,
            bulk_end: byte & (1 << 2) != 0,
            multiword: byte & (1 << 3) != 0,
            mode: byte >> 5,
        }
    }
}

/// Error results a server reports in the response's result byte. A result of 0 means success.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ServerError {
    /// The request length did not match its flags, or the flags combined bulk and multiword.
    MalformedRequest = 1,
    /// A bulk request arrived out of sequence, or a non-bulk request arrived during a bulk transfer.
    BulkRequestSeqError = 2,
    /// The page or register does not exist.
    InvalidRegisterAddress = 3,
    /// The register does not allow this access (writing a read-only register, for example).
    InvalidRegisterMode = 4,
    /// The register rejected the written value (an unknown command, for example).
    InvalidData = 5,
    /// The request's mode field does not match the server's control interface mode.
    InvalidMode = 6,
    /// The server does not support multiword requests.
    MultiwordUnsupported = 7,
    /// The multiword read does not fit in the server's response buffer.
    MultiwordTooLarge = 8,
}

impl ServerError {
    /// Decodes a non-zero result byte. Returns `None` for 0 (success) and unknown codes.
    pub const fn from_code(code: u8) -> Option<Self> {
        Some(match code {
            1 => ServerError::MalformedRequest,
            2 => ServerError::BulkRequestSeqError,
            3 => ServerError::InvalidRegisterAddress,
            4 => ServerError::InvalidRegisterMode,
            5 => ServerError::InvalidData,
            6 => ServerError::InvalidMode,
            7 => ServerError::MultiwordUnsupported,
            8 => ServerError::MultiwordTooLarge,
            _ => return None,
        })
    }

    /// The name used for this result in the C headers (`REG_MAPPED_RESULT_*`).
    pub const fn name(self) -> &'static str {
        match self {
            ServerError::MalformedRequest => "REG_MAPPED_RESULT_MALFORMED_REQUEST",
            ServerError::BulkRequestSeqError => "REG_MAPPED_RESULT_BULK_REQUEST_SEQ_ERROR",
            ServerError::InvalidRegisterAddress => "REG_MAPPED_RESULT_INVALID_REGISTER_ADDRESS",
            ServerError::InvalidRegisterMode => "REG_MAPPED_RESULT_INVALID_REGISTER_MODE",
            ServerError::InvalidData => "REG_MAPPED_RESULT_INVALID_DATA",
            ServerError::InvalidMode => "REG_MAPPED_RESULT_INVALID_MODE",
            ServerError::MultiwordUnsupported => "REG_MAPPED_RESULT_MULTIWORD_UNSUPPORTED",
            ServerError::MultiwordTooLarge => "REG_MAPPED_RESULT_MULTIWORD_TOO_LARGE",
        }
    }
}

impl core::fmt::Display for ServerError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(self.name())
    }
}

#[cfg(test)]
mod tests;
