//! Register mapped client: turns register accesses into requests and checks the server's responses.

use super::{
    BULK_RESPONSE_LEN, Flags, PAGE_NUM_WORDS, READ_REQUEST_LEN, READ_RESPONSE_LEN, REQUEST_HEADER_LEN, ServerError,
    WRITE_REQUEST_LEN, WRITE_RESPONSE_LEN, multiword_request_len, multiword_response_len,
};

/// Largest packet the client ever builds or accepts. Matches `CANMORE_ETH_UDP_MAX_LEN`, the largest MTU in use.
const MAX_PACKET_LEN: usize = 512;

/// Words staged per chunk by the byte oriented helpers.
const CHUNK_WORDS: usize = 64;

/// The packet link a [`Client`] talks over: one CAN utility channel to one device, or one UDP socket.
///
/// The transport decides where packets go and applies its own receive timeout.
pub trait Transport {
    type Error;

    /// The largest packet the link carries: 8 for classic CAN, 64 for CAN FD, the UDP buffer size for Ethernet.
    fn mtu(&self) -> usize;

    /// Discards any queued responses, so a late response to an earlier request can't be taken for the next one.
    fn clear_rx(&mut self) -> Result<(), Self::Error>;

    /// Sends one request packet.
    fn send(&mut self, data: &[u8]) -> Result<(), Self::Error>;

    /// Waits for the next response packet, copies it into `buf` and returns its length. A timeout is an error.
    fn recv(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error>;
}

impl<T: Transport + ?Sized> Transport for &mut T {
    type Error = T::Error;
    fn mtu(&self) -> usize {
        (**self).mtu()
    }
    fn clear_rx(&mut self) -> Result<(), Self::Error> {
        (**self).clear_rx()
    }
    fn send(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        (**self).send(data)
    }
    fn recv(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        (**self).recv(buf)
    }
}

/// How [`Client`] transfers arrays.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum TransferMode {
    /// One request and response per word.
    Single,
    /// Writes are sent as bulk transfers of up to `max_in_flight` requests per response. Reads fall back to
    /// [`Single`](Self::Single). `max_in_flight` must not exceed the server's receive queue, or requests get lost (the
    /// C tools use 32 on CAN, matching the MCP251XFD FIFO, and 8 on Ethernet).
    Bulk { max_in_flight: usize },
    /// As many words per request as the transport's MTU allows. Use for CAN FD and Ethernet.
    Multiword,
}

/// Why a [`Client`] access failed.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ClientError<E> {
    /// The transport failed to send or receive (including receive timeouts).
    Transport(E),
    /// The server rejected the request.
    Server(ServerError),
    /// The server answered with a result code this client doesn't know.
    UnknownResult(u8),
    /// The response was shorter than the request requires.
    BadResponseLength { expected: usize, got: usize },
    /// The access is invalid: it crosses a page boundary, or a buffer is too large.
    InvalidArgument,
    /// The bulk response acknowledged a different number of requests than were sent.
    BulkCountMismatch { sent: usize, acked: usize },
    /// The transport's MTU can't carry even a single word in multiword mode.
    MtuTooSmall,
}

impl<E> From<ServerError> for ClientError<E> {
    fn from(e: ServerError) -> Self {
        ClientError::Server(e)
    }
}

impl<E: core::fmt::Display> core::fmt::Display for ClientError<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ClientError::Transport(e) => write!(f, "transport error: {e}"),
            ClientError::Server(e) => write!(f, "server error: {e}"),
            ClientError::UnknownResult(code) => write!(f, "unknown server result {code}"),
            ClientError::BadResponseLength { expected, got } => {
                write!(f, "response too short ({got} bytes, expected {expected})")
            }
            ClientError::InvalidArgument => f.write_str("invalid argument"),
            ClientError::BulkCountMismatch { sent, acked } => {
                write!(f, "bulk transfer acknowledged {acked} requests, sent {sent}")
            }
            ClientError::MtuTooSmall => f.write_str("transport MTU too small for multiword transfers"),
        }
    }
}

impl<E: core::fmt::Debug + core::fmt::Display> core::error::Error for ClientError<E> {}

/// Register mapped protocol client.
///
/// Like the C client it is blocking and allocation free: each access sends its request(s) through the
/// [`Transport`] and waits for the response(s). The client stamps every request with its control interface mode,
/// which must match the mode the device reports in its heartbeat.
#[derive(Debug)]
pub struct Client<T> {
    transport: T,
    mode: u8,
    transfer_mode: TransferMode,
}

impl<T: Transport> Client<T> {
    /// Creates a client for control interface `mode` (see [`crate::ControlInterfaceMode`]).
    pub fn new(transport: T, mode: u8, transfer_mode: TransferMode) -> Self {
        Client { transport, mode, transfer_mode }
    }

    pub fn mode(&self) -> u8 {
        self.mode
    }

    /// Changes the control interface mode stamped on requests (for example after the device entered the bootloader).
    pub fn set_mode(&mut self, mode: u8) {
        self.mode = mode;
    }

    pub fn transfer_mode(&self) -> TransferMode {
        self.transfer_mode
    }

    pub fn set_transfer_mode(&mut self, transfer_mode: TransferMode) {
        self.transfer_mode = transfer_mode;
    }

    pub fn transport(&self) -> &T {
        &self.transport
    }

    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }

    pub fn into_transport(self) -> T {
        self.transport
    }

    fn header(&self, flags: Flags, count: u8, page: u8, offset: u8) -> [u8; REQUEST_HEADER_LEN] {
        [Flags { mode: self.mode, ..flags }.to_byte(), count, page, offset]
    }

    /// Receives a response and checks its result byte and length.
    fn recv_response(&mut self, buf: &mut [u8], expected_len: usize) -> Result<(), ClientError<T::Error>> {
        let len = self.transport.recv(buf).map_err(ClientError::Transport)?;
        if len == 0 {
            return Err(ClientError::BadResponseLength { expected: expected_len, got: 0 });
        }
        // Errors may come back in a shorter response than the success format (a failed multiword read, for example)
        if buf[0] != 0 {
            return Err(ServerError::from_code(buf[0]).map_or(ClientError::UnknownResult(buf[0]), ClientError::Server));
        }
        if len < expected_len {
            return Err(ClientError::BadResponseLength { expected: expected_len, got: len });
        }
        Ok(())
    }

    /// Sends a raw packet on the control interface, bypassing the protocol (see [`super::BOOTLOADER_REQUEST`]).
    pub fn send_raw(&mut self, data: &[u8]) -> Result<(), ClientError<T::Error>> {
        self.transport.send(data).map_err(ClientError::Transport)
    }

    /// Reads one register.
    pub fn read_register(&mut self, page: u8, offset: u8) -> Result<u32, ClientError<T::Error>> {
        let req = self.header(Flags::default(), 0, page, offset);
        self.transport.clear_rx().map_err(ClientError::Transport)?;
        self.transport.send(&req[..READ_REQUEST_LEN]).map_err(ClientError::Transport)?;
        let mut resp = [0u8; MAX_PACKET_LEN];
        self.recv_response(&mut resp, READ_RESPONSE_LEN)?;
        Ok(u32::from_le_bytes([resp[1], resp[2], resp[3], resp[4]]))
    }

    /// Writes one register.
    pub fn write_register(&mut self, page: u8, offset: u8, data: u32) -> Result<(), ClientError<T::Error>> {
        let mut req = [0u8; WRITE_REQUEST_LEN];
        req[..REQUEST_HEADER_LEN].copy_from_slice(&self.header(
            Flags { write: true, ..Flags::default() },
            0,
            page,
            offset,
        ));
        req[REQUEST_HEADER_LEN..].copy_from_slice(&data.to_le_bytes());
        self.transport.clear_rx().map_err(ClientError::Transport)?;
        self.transport.send(&req).map_err(ClientError::Transport)?;
        let mut resp = [0u8; MAX_PACKET_LEN];
        self.recv_response(&mut resp, WRITE_RESPONSE_LEN)
    }

    fn check_range(offset: u8, num_words: usize) -> Result<(), ClientError<T::Error>> {
        if offset as usize + num_words > PAGE_NUM_WORDS {
            return Err(ClientError::InvalidArgument);
        }
        Ok(())
    }

    fn packet_limit(&self) -> usize {
        self.transport.mtu().min(MAX_PACKET_LEN)
    }

    /// Reads `out.len()` consecutive registers starting at `offset`. The range must stay within the page.
    pub fn read_array(&mut self, page: u8, offset: u8, out: &mut [u32]) -> Result<(), ClientError<T::Error>> {
        Self::check_range(offset, out.len())?;

        if self.transfer_mode != TransferMode::Multiword {
            // Bulk transfers only exist for writes
            for (i, word) in out.iter_mut().enumerate() {
                *word = self.read_register(page, offset + i as u8)?;
            }
            return Ok(());
        }

        let max_words = self.packet_limit().saturating_sub(1) / 4;
        if max_words == 0 {
            return Err(ClientError::MtuTooSmall);
        }

        self.transport.clear_rx().map_err(ClientError::Transport)?;
        let mut resp = [0u8; MAX_PACKET_LEN];
        let mut done = 0;
        for chunk in out.chunks_mut(max_words) {
            let req = self.header(
                Flags { multiword: true, ..Flags::default() },
                chunk.len() as u8,
                page,
                (offset as usize + done) as u8,
            );
            self.transport.send(&req).map_err(ClientError::Transport)?;
            self.recv_response(&mut resp, multiword_response_len(chunk.len()))?;
            for (word, bytes) in chunk.iter_mut().zip(resp[1..].as_chunks::<4>().0) {
                *word = u32::from_le_bytes(*bytes);
            }
            done += chunk.len();
        }
        Ok(())
    }

    /// Writes `data` to consecutive registers starting at `offset`. The range must stay within the page.
    pub fn write_array(&mut self, page: u8, offset: u8, data: &[u32]) -> Result<(), ClientError<T::Error>> {
        Self::check_range(offset, data.len())?;

        match self.transfer_mode {
            TransferMode::Single => {
                for (i, &word) in data.iter().enumerate() {
                    self.write_register(page, offset + i as u8, word)?;
                }
                Ok(())
            }
            TransferMode::Bulk { max_in_flight } => self.write_array_bulk(page, offset, data, max_in_flight),
            TransferMode::Multiword => self.write_array_multiword(page, offset, data),
        }
    }

    fn write_array_multiword(&mut self, page: u8, offset: u8, data: &[u32]) -> Result<(), ClientError<T::Error>> {
        let max_words = self.packet_limit().saturating_sub(REQUEST_HEADER_LEN) / 4;
        if max_words == 0 {
            return Err(ClientError::MtuTooSmall);
        }

        self.transport.clear_rx().map_err(ClientError::Transport)?;
        let mut req = [0u8; MAX_PACKET_LEN];
        let mut resp = [0u8; MAX_PACKET_LEN];
        let mut done = 0;
        for chunk in data.chunks(max_words) {
            let flags = Flags { write: true, multiword: true, ..Flags::default() };
            req[..REQUEST_HEADER_LEN].copy_from_slice(&self.header(
                flags,
                chunk.len() as u8,
                page,
                (offset as usize + done) as u8,
            ));
            for (bytes, word) in req[REQUEST_HEADER_LEN..].as_chunks_mut::<4>().0.iter_mut().zip(chunk) {
                *bytes = word.to_le_bytes();
            }
            self.transport.send(&req[..multiword_request_len(chunk.len())]).map_err(ClientError::Transport)?;
            self.recv_response(&mut resp, WRITE_RESPONSE_LEN)?;
            done += chunk.len();
        }
        Ok(())
    }

    fn write_array_bulk(
        &mut self,
        page: u8,
        offset: u8,
        data: &[u32],
        max_in_flight: usize,
    ) -> Result<(), ClientError<T::Error>> {
        // The sequence number is 8 bits, so at most 256 requests can be outstanding
        let max_in_flight = max_in_flight.clamp(1, 256);

        self.transport.clear_rx().map_err(ClientError::Transport)?;
        let mut resp = [0u8; MAX_PACKET_LEN];
        for (batch_idx, batch) in data.chunks(max_in_flight).enumerate() {
            let batch_offset = offset as usize + batch_idx * max_in_flight;
            for (seq, &word) in batch.iter().enumerate() {
                // The last request of the batch ends the bulk transfer, which makes the server respond
                let flags = Flags { write: true, bulk: true, bulk_end: seq + 1 == batch.len(), ..Flags::default() };
                let mut req = [0u8; WRITE_REQUEST_LEN];
                req[..REQUEST_HEADER_LEN].copy_from_slice(&self.header(
                    flags,
                    seq as u8,
                    page,
                    (batch_offset + seq) as u8,
                ));
                req[REQUEST_HEADER_LEN..].copy_from_slice(&word.to_le_bytes());
                self.transport.send(&req).map_err(ClientError::Transport)?;
            }

            self.recv_response(&mut resp, BULK_RESPONSE_LEN)?;
            let acked = resp[1] as usize + 1;
            if acked != batch.len() {
                return Err(ClientError::BulkCountMismatch { sent: batch.len(), acked });
            }
        }
        Ok(())
    }

    /// Reads `out.len()` bytes from consecutive registers starting at `offset` (little endian words).
    pub fn read_bytes(&mut self, page: u8, offset: u8, out: &mut [u8]) -> Result<(), ClientError<T::Error>> {
        Self::check_range(offset, out.len().div_ceil(4))?;
        let mut words = [0u32; CHUNK_WORDS];
        for (i, chunk) in out.chunks_mut(CHUNK_WORDS * 4).enumerate() {
            let num_words = chunk.len().div_ceil(4);
            self.read_array(page, offset + (i * CHUNK_WORDS) as u8, &mut words[..num_words])?;
            for (bytes, word) in chunk.chunks_mut(4).zip(&words) {
                bytes.copy_from_slice(&word.to_le_bytes()[..bytes.len()]);
            }
        }
        Ok(())
    }

    /// Writes `data` to consecutive registers starting at `offset` (little endian words, the last word zero padded).
    pub fn write_bytes(&mut self, page: u8, offset: u8, data: &[u8]) -> Result<(), ClientError<T::Error>> {
        self.write_byte_iter(page, offset, data.iter().copied(), data.len())
    }

    /// Writes `len` bytes from `bytes` in chunks, so no buffer the size of the whole transfer is needed.
    fn write_byte_iter(
        &mut self,
        page: u8,
        offset: u8,
        mut bytes: impl Iterator<Item = u8>,
        len: usize,
    ) -> Result<(), ClientError<T::Error>> {
        let total_words = len.div_ceil(4);
        Self::check_range(offset, total_words)?;
        let mut words = [0u32; CHUNK_WORDS];
        let mut written = 0;
        while written < total_words {
            let num_words = (total_words - written).min(CHUNK_WORDS);
            for word in &mut words[..num_words] {
                let mut le = [0u8; 4];
                for b in &mut le {
                    *b = bytes.next().unwrap_or(0);
                }
                *word = u32::from_le_bytes(le);
            }
            self.write_array(page, offset + written as u8, &words[..num_words])?;
            written += num_words;
        }
        Ok(())
    }

    /// Writes `s` followed by a NUL terminator to the start of a page.
    pub fn write_string_page(&mut self, page: u8, s: &[u8]) -> Result<(), ClientError<T::Error>> {
        self.write_byte_iter(page, 0, s.iter().copied().chain(core::iter::once(0)), s.len() + 1)
    }

    /// Reads a NUL terminated string page into `out`, returning the string length (without terminator).
    ///
    /// Reads word by word, since the server rejects reads past the end of the string. Stops early once `out` is
    /// full, and at the end of the page if the string isn't terminated.
    pub fn read_string_page(&mut self, page: u8, out: &mut [u8]) -> Result<usize, ClientError<T::Error>> {
        let mut len = 0;
        for offset in 0..PAGE_NUM_WORDS {
            if len == out.len() {
                break;
            }
            let word = match self.read_register(page, offset as u8) {
                Ok(word) => word,
                // A string filling its page exactly, without a terminator, ends where the page does. The C client
                // fails here; accepting it keeps unterminated byte pages (`MemoryPage::Bytes(b"...")`) readable.
                Err(ClientError::Server(ServerError::InvalidRegisterAddress)) if offset > 0 => return Ok(len),
                Err(e) => return Err(e),
            };
            for byte in word.to_le_bytes() {
                if byte == 0 || len == out.len() {
                    return Ok(len);
                }
                out[len] = byte;
                len += 1;
            }
        }
        Ok(len)
    }
}
