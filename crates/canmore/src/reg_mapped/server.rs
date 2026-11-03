//! Register mapped server: decodes requests, applies them to a [`RegisterMap`] and encodes the response.

use super::{
    BULK_RESPONSE_LEN, Flags, MIN_RESPONSE_BUFFER_LEN, READ_REQUEST_LEN, READ_RESPONSE_LEN, REQUEST_HEADER_LEN,
    ServerError, WRITE_REQUEST_LEN, WRITE_RESPONSE_LEN, multiword_request_len, multiword_response_len,
};

/// The registers a [`Server`] exposes.
///
/// Implement this for the state behind a control interface. A typical implementation matches on `(page, offset)`,
/// returns constants or fields for plain registers, runs an action for trigger registers, and delegates memory pages
/// (buffers, version strings) to a [`MemoryPage`]:
///
/// ```
/// use canmore::reg_mapped::{Access, MemoryPage, RegisterMap, ServerError};
///
/// struct Debug<'a> {
///     version: &'a [u8],
///     reboot_requested: bool,
/// }
///
/// impl RegisterMap for Debug<'_> {
///     fn read(&mut self, page: u8, offset: u8) -> Result<u32, ServerError> {
///         match (page, offset) {
///             (0, 0) => Ok(0x0DBAA1F0),
///             (0, 7) => Err(ServerError::InvalidRegisterMode), // write-only
///             (1, _) => MemoryPage::Bytes(Access::ReadOnly(self.version)).read(offset),
///             _ => Err(ServerError::InvalidRegisterAddress),
///         }
///     }
///
///     fn write(&mut self, page: u8, offset: u8, _data: u32) -> Result<(), ServerError> {
///         match (page, offset) {
///             (0, 7) => Ok(self.reboot_requested = true),
///             (0, 0) | (1, _) => Err(ServerError::InvalidRegisterMode), // read-only
///             _ => Err(ServerError::InvalidRegisterAddress),
///         }
///     }
/// }
/// ```
///
/// To match the C server, report a missing page or register with [`ServerError::InvalidRegisterAddress`], an access
/// the register doesn't permit with [`ServerError::InvalidRegisterMode`], and a rejected value with
/// [`ServerError::InvalidData`]. Check the address before the permission.
pub trait RegisterMap {
    /// Reads one register.
    fn read(&mut self, page: u8, offset: u8) -> Result<u32, ServerError>;

    /// Writes one register.
    fn write(&mut self, page: u8, offset: u8, data: u32) -> Result<(), ServerError>;

    /// Reads `out.len() / 4` consecutive words starting at `offset` into `out` (little endian).
    ///
    /// Only memory pages support multiword access, since register pages can be sparse. The default rejects every
    /// page, like a C register mapped page. Delegate to [`MemoryPage::read_multiword`] for memory pages.
    fn read_multiword(&mut self, page: u8, offset: u8, out: &mut [u8]) -> Result<(), ServerError> {
        let _ = (page, offset, out);
        Err(ServerError::InvalidRegisterAddress)
    }

    /// Writes `data.len() / 4` consecutive words starting at `offset` (little endian).
    ///
    /// The default rejects every page. Delegate to [`MemoryPage::write_multiword`] for memory pages.
    fn write_multiword(&mut self, page: u8, offset: u8, data: &[u8]) -> Result<(), ServerError> {
        let _ = (page, offset, data);
        Err(ServerError::InvalidRegisterAddress)
    }
}

impl<M: RegisterMap + ?Sized> RegisterMap for &mut M {
    fn read(&mut self, page: u8, offset: u8) -> Result<u32, ServerError> {
        (**self).read(page, offset)
    }
    fn write(&mut self, page: u8, offset: u8, data: u32) -> Result<(), ServerError> {
        (**self).write(page, offset, data)
    }
    fn read_multiword(&mut self, page: u8, offset: u8, out: &mut [u8]) -> Result<(), ServerError> {
        (**self).read_multiword(page, offset, out)
    }
    fn write_multiword(&mut self, page: u8, offset: u8, data: &[u8]) -> Result<(), ServerError> {
        (**self).write_multiword(page, offset, data)
    }
}

/// Access permission of a register or memory page.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Perm {
    ReadWrite,
    ReadOnly,
    WriteOnly,
}

impl Perm {
    pub const fn can_read(self) -> bool {
        !matches!(self, Perm::WriteOnly)
    }

    pub const fn can_write(self) -> bool {
        !matches!(self, Perm::ReadOnly)
    }
}

/// A slice exposed through a [`MemoryPage`], with its permission encoded in how it is borrowed.
#[derive(Debug)]
pub enum Access<'a, T> {
    ReadOnly(&'a [T]),
    WriteOnly(&'a mut [T]),
    ReadWrite(&'a mut [T]),
}

impl<T> Access<'_, T> {
    pub fn perm(&self) -> Perm {
        match self {
            Access::ReadOnly(_) => Perm::ReadOnly,
            Access::WriteOnly(_) => Perm::WriteOnly,
            Access::ReadWrite(_) => Perm::ReadWrite,
        }
    }

    pub fn len(&self) -> usize {
        match self {
            Access::ReadOnly(s) => s.len(),
            Access::WriteOnly(s) | Access::ReadWrite(s) => s.len(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    fn as_slice(&self) -> &[T] {
        match self {
            Access::ReadOnly(s) => s,
            Access::WriteOnly(s) | Access::ReadWrite(s) => s,
        }
    }

    fn as_mut_slice(&mut self) -> Option<&mut [T]> {
        match self {
            Access::ReadOnly(_) => None,
            Access::WriteOnly(s) | Access::ReadWrite(s) => Some(s),
        }
    }
}

/// A page backed by a contiguous block of memory, with the exact semantics of the C server's memory mapped pages.
///
/// * `Words` maps register `n` to word `n`.
/// * `Bytes` maps register `n` to bytes `4n..4n + 4` (little endian). The buffer doesn't have to be a multiple of 4
///   bytes long: the last register reads the missing bytes as zero and drops them on write. This is how version
///   strings and other text pages are exposed. Include the NUL terminator in string pages (`b"1.0.0\0"`): the C
///   client reads until it finds one, and fails on strings whose length is a multiple of 4 otherwise.
///
/// Multiword accesses must stay within the page. For byte pages they may run into the partial last word, but not
/// beyond it.
#[derive(Debug)]
pub enum MemoryPage<'a> {
    Words(Access<'a, u32>),
    Bytes(Access<'a, u8>),
}

impl MemoryPage<'_> {
    pub fn perm(&self) -> Perm {
        match self {
            MemoryPage::Words(a) => a.perm(),
            MemoryPage::Bytes(a) => a.perm(),
        }
    }

    /// Reads register `offset`.
    pub fn read(&self, offset: u8) -> Result<u32, ServerError> {
        let offset = offset as usize;
        match self {
            MemoryPage::Words(words) => {
                if offset >= words.len() {
                    return Err(ServerError::InvalidRegisterAddress);
                }
                if !words.perm().can_read() {
                    return Err(ServerError::InvalidRegisterMode);
                }
                Ok(words.as_slice()[offset])
            }
            MemoryPage::Bytes(bytes) => {
                let start = offset * 4;
                if start >= bytes.len() {
                    return Err(ServerError::InvalidRegisterAddress);
                }
                if !bytes.perm().can_read() {
                    return Err(ServerError::InvalidRegisterMode);
                }
                let end = (start + 4).min(bytes.len());
                let mut word = [0u8; 4];
                word[..end - start].copy_from_slice(&bytes.as_slice()[start..end]);
                Ok(u32::from_le_bytes(word))
            }
        }
    }

    /// Writes register `offset`.
    pub fn write(&mut self, offset: u8, data: u32) -> Result<(), ServerError> {
        let offset = offset as usize;
        match self {
            MemoryPage::Words(words) => {
                if offset >= words.len() {
                    return Err(ServerError::InvalidRegisterAddress);
                }
                let words = words.as_mut_slice().ok_or(ServerError::InvalidRegisterMode)?;
                words[offset] = data;
                Ok(())
            }
            MemoryPage::Bytes(bytes) => {
                let start = offset * 4;
                if start >= bytes.len() {
                    return Err(ServerError::InvalidRegisterAddress);
                }
                let bytes = bytes.as_mut_slice().ok_or(ServerError::InvalidRegisterMode)?;
                let end = (start + 4).min(bytes.len());
                bytes[start..end].copy_from_slice(&data.to_le_bytes()[..end - start]);
                Ok(())
            }
        }
    }

    /// Byte range of a multiword access, or the error the C server reports for it.
    fn multiword_range(&self, offset: u8, byte_len: usize) -> Result<(usize, usize), ServerError> {
        let start = offset as usize * 4;
        match self {
            MemoryPage::Words(words) => {
                if start + byte_len > words.len() * 4 {
                    return Err(ServerError::InvalidRegisterAddress);
                }
                Ok((start, byte_len))
            }
            MemoryPage::Bytes(bytes) => {
                // Partial last words are allowed (a 7 byte buffer accepts 2 words), anything further is not
                if start + byte_len >= bytes.len() + 4 {
                    return Err(ServerError::InvalidRegisterAddress);
                }
                // Only a zero length access can start past the end, clamp it so it copies nothing
                let start = start.min(bytes.len());
                Ok((start, byte_len.min(bytes.len() - start)))
            }
        }
    }

    /// Reads `out.len() / 4` words starting at register `offset`.
    pub fn read_multiword(&self, offset: u8, out: &mut [u8]) -> Result<(), ServerError> {
        let out_len = out.len() & !3;
        let (start, copy_len) = self.multiword_range(offset, out_len)?;
        if !self.perm().can_read() {
            return Err(ServerError::InvalidRegisterMode);
        }
        match self {
            MemoryPage::Words(words) => {
                for (chunk, word) in
                    out[..out_len].as_chunks_mut::<4>().0.iter_mut().zip(&words.as_slice()[start / 4..])
                {
                    *chunk = word.to_le_bytes();
                }
            }
            MemoryPage::Bytes(bytes) => {
                out[..copy_len].copy_from_slice(&bytes.as_slice()[start..start + copy_len]);
                out[copy_len..out_len].fill(0);
            }
        }
        Ok(())
    }

    /// Writes `data.len() / 4` words starting at register `offset`.
    pub fn write_multiword(&mut self, offset: u8, data: &[u8]) -> Result<(), ServerError> {
        let data = &data[..data.len() & !3];
        let (start, copy_len) = self.multiword_range(offset, data.len())?;
        match self {
            MemoryPage::Words(words) => {
                let words = words.as_mut_slice().ok_or(ServerError::InvalidRegisterMode)?;
                for (word, chunk) in words[start / 4..].iter_mut().zip(data.as_chunks::<4>().0) {
                    *word = u32::from_le_bytes(*chunk);
                }
            }
            MemoryPage::Bytes(bytes) => {
                let bytes = bytes.as_mut_slice().ok_or(ServerError::InvalidRegisterMode)?;
                bytes[start..start + copy_len].copy_from_slice(&data[..copy_len]);
            }
        }
        Ok(())
    }
}

/// Register mapped protocol server for one control interface mode.
///
/// The server is transport agnostic and allocation free: feed every packet received on the control interface into
/// [`handle_request`](Self::handle_request) and send back the response it encodes, if any. The only state it keeps
/// is the progress of a bulk write, so one server instance serves one agent at a time during bulk writes (as in C).
#[derive(Clone, Debug)]
pub struct Server {
    mode: u8,
    multiword: bool,
    in_bulk: bool,
    bulk_last_seq: u8,
    bulk_error: u8,
}

impl Server {
    /// Creates a server answering requests for control interface `mode` (see [`crate::ControlInterfaceMode`]).
    /// Multiword requests are supported, up to what fits in the response buffer.
    pub const fn new(mode: u8) -> Self {
        Server { mode, multiword: true, in_bulk: false, bulk_last_seq: 0, bulk_error: 0 }
    }

    /// Creates a server that rejects multiword requests with [`ServerError::MultiwordUnsupported`], like a C server
    /// without a multiword response buffer.
    pub const fn new_without_multiword(mode: u8) -> Self {
        Server { mode, multiword: false, in_bulk: false, bulk_last_seq: 0, bulk_error: 0 }
    }

    /// The control interface mode this server answers.
    pub const fn mode(&self) -> u8 {
        self.mode
    }

    /// True while a bulk write is in progress.
    pub const fn in_bulk(&self) -> bool {
        self.in_bulk
    }

    /// Handles one request and encodes the response into `resp`.
    ///
    /// Returns the length of the response to send, or 0 if nothing should be sent (an empty request, or a bulk
    /// write that hasn't ended yet). `resp` must be at least [`MIN_RESPONSE_BUFFER_LEN`] bytes. Its length also caps
    /// multiword reads: larger reads fail with [`ServerError::MultiwordTooLarge`]. Use the link's MTU (8 for classic
    /// CAN, 64 for CAN FD) so every response fits in one packet.
    pub fn handle_request<M: RegisterMap + ?Sized>(&mut self, map: &mut M, req: &[u8], resp: &mut [u8]) -> usize {
        if req.is_empty() {
            // Not enough data to know which response format to use
            return 0;
        }
        if resp.len() < MIN_RESPONSE_BUFFER_LEN {
            debug_assert!(false, "reg mapped response buffer too small");
            return 0;
        }

        let flags = Flags::from_byte(req[0]);
        let (result, read_data, multiword_count) = self.process(map, flags, req, resp);

        if flags.bulk {
            if flags.bulk_end {
                resp[0] = if self.bulk_error != 0 { self.bulk_error } else { result };
                resp[1] = self.bulk_last_seq;
                self.in_bulk = false;
                BULK_RESPONSE_LEN
            } else {
                if self.bulk_error == 0 {
                    self.bulk_error = result;
                }
                0
            }
        } else if let Some(count) = multiword_count
            && result == 0
        {
            // The data words were already written into resp by process()
            resp[0] = 0;
            multiword_response_len(count)
        } else if flags.write {
            resp[0] = result;
            WRITE_RESPONSE_LEN
        } else {
            resp[0] = result;
            resp[1..READ_RESPONSE_LEN].copy_from_slice(&read_data.to_le_bytes());
            READ_RESPONSE_LEN
        }
    }

    /// Validates and executes a request. Returns the result code, the data of a single read, and the word count of a
    /// multiword read (whose data is placed in `resp[1..]`).
    fn process<M: RegisterMap + ?Sized>(
        &mut self,
        map: &mut M,
        flags: Flags,
        req: &[u8],
        resp: &mut [u8],
    ) -> (u8, u32, Option<usize>) {
        const MALFORMED: u8 = ServerError::MalformedRequest as u8;
        const SEQ_ERROR: u8 = ServerError::BulkRequestSeqError as u8;

        let expected_len = if !flags.write {
            READ_REQUEST_LEN
        } else if flags.multiword {
            if req.len() < REQUEST_HEADER_LEN {
                return (MALFORMED, 0, None);
            }
            multiword_request_len(req[1] as usize)
        } else {
            WRITE_REQUEST_LEN
        };
        if req.len() != expected_len {
            return (MALFORMED, 0, None);
        }

        let count = req[1];
        let page = req[2];
        let offset = req[3];

        if self.in_bulk && !flags.bulk {
            // A normal request in the middle of a bulk transfer aborts the bulk transfer
            self.in_bulk = false;
            return (SEQ_ERROR, 0, None);
        }
        if (flags.multiword && flags.bulk) || (flags.bulk && !flags.write) || (flags.bulk_end && !flags.bulk) {
            return (MALFORMED, 0, None);
        }
        if flags.mode != self.mode {
            // A stray request meant for another control interface mode must not touch our state
            return (ServerError::InvalidMode as u8, 0, None);
        }

        if flags.multiword {
            if !self.multiword {
                return (ServerError::MultiwordUnsupported as u8, 0, None);
            }
            if flags.write {
                return (code(map.write_multiword(page, offset, &req[REQUEST_HEADER_LEN..])), 0, None);
            }
            let count = count as usize;
            if count > (resp.len() - 1) / 4 {
                return (ServerError::MultiwordTooLarge as u8, 0, None);
            }
            let result = code(map.read_multiword(page, offset, &mut resp[1..multiword_response_len(count)]));
            return (result, 0, Some(count));
        }

        if flags.bulk {
            if !self.in_bulk {
                if count != 0 {
                    // Bulk transfers must start at sequence number 0
                    return (SEQ_ERROR, 0, None);
                }
                self.in_bulk = true;
                self.bulk_error = 0;
                self.bulk_last_seq = 0;
            } else if self.bulk_error == 0 {
                // Once an error is latched, keep reporting the sequence number it happened on
                self.bulk_last_seq = self.bulk_last_seq.wrapping_add(1);
                if count != self.bulk_last_seq {
                    return (SEQ_ERROR, 0, None);
                }
            }
        }

        if self.in_bulk && self.bulk_error != 0 {
            // Requests after a failed bulk request are ignored
            return (MALFORMED, 0, None);
        }

        if flags.write {
            let data = u32::from_le_bytes([req[4], req[5], req[6], req[7]]);
            (code(map.write(page, offset, data)), 0, None)
        } else {
            match map.read(page, offset) {
                Ok(data) => (0, data, None),
                Err(e) => (e as u8, 0, None),
            }
        }
    }
}

fn code(result: Result<(), ServerError>) -> u8 {
    match result {
        Ok(()) => 0,
        Err(e) => e as u8,
    }
}
