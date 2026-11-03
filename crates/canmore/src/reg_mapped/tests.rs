//! Client against server, in memory.

// Flag bytes are written grouped by field: mode | RFU | multiword, bulk end, bulk, write
#![allow(clippy::unusual_byte_groupings)]

extern crate std;

use super::*;
use std::collections::VecDeque;
use std::vec::Vec;

const MODE: u8 = crate::ControlInterfaceMode::Normal as u8;
const MAGIC: u32 = 0x0DBAA1F0;
const VERSION: &[u8] = b"hello world";

/// A device exposing every kind of page.
///
/// * page 0: registers. 0 = magic (RO), 1 = scratch (RW), 2 = trigger (WO, rejects 0xBAD), 3 = unimplemented
/// * page 1: version string (RO bytes, 11 long)
/// * page 2: 256 word buffer (RW)
/// * page 3: 7 byte buffer (RW)
/// * page 4: 4 word buffer (WO)
/// * page 5: unterminated 8 byte string (RO)
struct TestMap {
    scratch: u32,
    triggers: Vec<u32>,
    words: [u32; PAGE_NUM_WORDS],
    bytes: [u8; 7],
    wo: [u32; 4],
}

impl TestMap {
    fn new() -> Self {
        TestMap { scratch: 0, triggers: Vec::new(), words: [0; PAGE_NUM_WORDS], bytes: [0; 7], wo: [0; 4] }
    }

    fn page(&mut self, page: u8) -> Option<MemoryPage<'_>> {
        Some(match page {
            1 => MemoryPage::Bytes(Access::ReadOnly(VERSION)),
            2 => MemoryPage::Words(Access::ReadWrite(&mut self.words)),
            3 => MemoryPage::Bytes(Access::ReadWrite(&mut self.bytes)),
            4 => MemoryPage::Words(Access::WriteOnly(&mut self.wo)),
            5 => MemoryPage::Bytes(Access::ReadOnly(b"abcdefgh")),
            _ => return None,
        })
    }
}

impl RegisterMap for TestMap {
    fn read(&mut self, page: u8, offset: u8) -> Result<u32, ServerError> {
        match (page, offset) {
            (0, 0) => Ok(MAGIC),
            (0, 1) => Ok(self.scratch),
            (0, 2) => Err(ServerError::InvalidRegisterMode),
            (0, _) => Err(ServerError::InvalidRegisterAddress),
            _ => self.page(page).ok_or(ServerError::InvalidRegisterAddress)?.read(offset),
        }
    }

    fn write(&mut self, page: u8, offset: u8, data: u32) -> Result<(), ServerError> {
        match (page, offset) {
            (0, 0) => Err(ServerError::InvalidRegisterMode),
            (0, 1) => {
                self.scratch = data;
                Ok(())
            }
            (0, 2) if data == 0xBAD => Err(ServerError::InvalidData),
            (0, 2) => {
                self.triggers.push(data);
                Ok(())
            }
            (0, _) => Err(ServerError::InvalidRegisterAddress),
            _ => self.page(page).ok_or(ServerError::InvalidRegisterAddress)?.write(offset, data),
        }
    }

    fn read_multiword(&mut self, page: u8, offset: u8, out: &mut [u8]) -> Result<(), ServerError> {
        self.page(page).ok_or(ServerError::InvalidRegisterAddress)?.read_multiword(offset, out)
    }

    fn write_multiword(&mut self, page: u8, offset: u8, data: &[u8]) -> Result<(), ServerError> {
        self.page(page).ok_or(ServerError::InvalidRegisterAddress)?.write_multiword(offset, data)
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
struct Timeout;

/// Delivers requests straight to a server and queues its responses.
struct Loopback {
    server: Server,
    map: TestMap,
    mtu: usize,
    /// Size of the server's response buffer.
    resp_buf_len: usize,
    responses: VecDeque<Vec<u8>>,
    /// Requests seen so far, and the index of a request to drop (simulating a lost frame).
    sent: usize,
    drop_request: Option<usize>,
}

impl Loopback {
    fn new(mtu: usize) -> Self {
        Loopback {
            server: Server::new(MODE),
            map: TestMap::new(),
            mtu,
            resp_buf_len: mtu,
            responses: VecDeque::new(),
            sent: 0,
            drop_request: None,
        }
    }

    fn handle(&mut self, req: &[u8]) -> Option<Vec<u8>> {
        let mut resp = std::vec![0u8; self.resp_buf_len];
        let len = self.server.handle_request(&mut self.map, req, &mut resp);
        (len > 0).then(|| resp[..len].to_vec())
    }
}

impl Transport for Loopback {
    type Error = Timeout;

    fn mtu(&self) -> usize {
        self.mtu
    }

    fn clear_rx(&mut self) -> Result<(), Timeout> {
        self.responses.clear();
        Ok(())
    }

    fn send(&mut self, data: &[u8]) -> Result<(), Timeout> {
        assert!(data.len() <= self.mtu, "request of {} bytes exceeds MTU {}", data.len(), self.mtu);
        let idx = self.sent;
        self.sent += 1;
        if self.drop_request == Some(idx) {
            return Ok(());
        }
        if let Some(resp) = self.handle(data) {
            assert!(resp.len() <= self.mtu, "response of {} bytes exceeds MTU {}", resp.len(), self.mtu);
            self.responses.push_back(resp);
        }
        Ok(())
    }

    fn recv(&mut self, buf: &mut [u8]) -> Result<usize, Timeout> {
        let resp = self.responses.pop_front().ok_or(Timeout)?;
        buf[..resp.len()].copy_from_slice(&resp);
        Ok(resp.len())
    }
}

fn client(mtu: usize, transfer_mode: TransferMode) -> Client<Loopback> {
    Client::new(Loopback::new(mtu), MODE, transfer_mode)
}

const ALL_MODES: [(usize, TransferMode); 5] = [
    (8, TransferMode::Single),
    (8, TransferMode::Bulk { max_in_flight: 32 }),
    (8, TransferMode::Bulk { max_in_flight: 256 }),
    (64, TransferMode::Multiword),
    (512, TransferMode::Multiword),
];

fn pattern(len: usize) -> Vec<u32> {
    (0..len as u32).map(|i| i.wrapping_mul(0x9E37_79B9) ^ 0x5A5A_0000).collect()
}

#[test]
fn flags_round_trip() {
    for byte in 0..=255u8 {
        let flags = Flags::from_byte(byte);
        // Bit 4 is reserved and dropped
        assert_eq!(flags.to_byte(), byte & !0x10);
    }
    let flags = Flags { write: true, bulk: false, bulk_end: false, multiword: true, mode: 2 };
    assert_eq!(flags.to_byte(), 0b010_0_1001);
}

#[test]
fn single_register_access() {
    let mut c = client(8, TransferMode::Single);
    assert_eq!(c.read_register(0, 0), Ok(MAGIC));
    c.write_register(0, 1, 0x1234_5678).unwrap();
    assert_eq!(c.read_register(0, 1), Ok(0x1234_5678));
    c.write_register(0, 2, 7).unwrap();
    assert_eq!(c.transport().map.triggers, [7]);
}

#[test]
fn register_errors() {
    let mut c = client(8, TransferMode::Single);
    assert_eq!(c.read_register(0, 3), Err(ClientError::Server(ServerError::InvalidRegisterAddress)));
    assert_eq!(c.read_register(9, 0), Err(ClientError::Server(ServerError::InvalidRegisterAddress)));
    assert_eq!(c.write_register(0, 0, 1), Err(ClientError::Server(ServerError::InvalidRegisterMode)));
    assert_eq!(c.read_register(0, 2), Err(ClientError::Server(ServerError::InvalidRegisterMode)));
    assert_eq!(c.write_register(0, 2, 0xBAD), Err(ClientError::Server(ServerError::InvalidData)));
    assert_eq!(c.write_register(1, 0, 1), Err(ClientError::Server(ServerError::InvalidRegisterMode)));
    assert_eq!(c.read_register(4, 0), Err(ClientError::Server(ServerError::InvalidRegisterMode)));
    // Past the end of the 11 byte version string
    assert_eq!(c.read_register(1, 3), Err(ClientError::Server(ServerError::InvalidRegisterAddress)));

    c.set_mode(crate::ControlInterfaceMode::Bootloader as u8);
    assert_eq!(c.read_register(0, 0), Err(ClientError::Server(ServerError::InvalidMode)));
}

#[test]
fn lost_response_times_out() {
    let mut c = client(8, TransferMode::Single);
    c.transport_mut().drop_request = Some(0);
    assert_eq!(c.read_register(0, 0), Err(ClientError::Transport(Timeout)));
    // The next access works again
    assert_eq!(c.read_register(0, 0), Ok(MAGIC));
}

#[test]
fn byte_page_partial_words() {
    let mut c = client(8, TransferMode::Single);
    // "hello world" = 68 65 6c 6c | 6f 20 77 6f | 72 6c 64
    assert_eq!(c.read_register(1, 0), Ok(u32::from_le_bytes(*b"hell")));
    assert_eq!(c.read_register(1, 2), Ok(u32::from_le_bytes([b'r', b'l', b'd', 0])));

    // The 7 byte buffer drops the byte past its end
    c.write_register(3, 1, 0xAABB_CCDD).unwrap();
    assert_eq!(c.transport().map.bytes, [0, 0, 0, 0, 0xDD, 0xCC, 0xBB]);
    assert_eq!(c.read_register(3, 1), Ok(0x00BB_CCDD));
    assert_eq!(c.write_register(3, 2, 0), Err(ClientError::Server(ServerError::InvalidRegisterAddress)));
}

#[test]
fn string_pages() {
    let mut c = client(8, TransferMode::Single);
    let mut out = [0u8; PAGE_SIZE];
    let len = c.read_string_page(1, &mut out).unwrap();
    assert_eq!(&out[..len], VERSION);

    // An unterminated string ending on a word boundary ends with the page
    assert_eq!(c.read_string_page(5, &mut out), Ok(8));
    assert_eq!(&out[..8], b"abcdefgh");
    // A missing page is still an error
    assert_eq!(c.read_string_page(9, &mut out), Err(ClientError::Server(ServerError::InvalidRegisterAddress)));

    // A short buffer stops the read early
    let mut short = [0u8; 4];
    assert_eq!(c.read_string_page(1, &mut short), Ok(4));
    assert_eq!(&short, b"hell");

    for (mtu, mode) in ALL_MODES {
        let mut c = client(mtu, mode);
        c.write_string_page(2, b"remote\0arg\0\0").unwrap();
        let len = c.read_string_page(2, &mut out).unwrap();
        assert_eq!(&out[..len], b"remote");
        let mut raw = [0u8; 13];
        c.read_bytes(2, 0, &mut raw).unwrap();
        assert_eq!(&raw, b"remote\0arg\0\0\0");
    }
}

#[test]
fn array_round_trips() {
    for (mtu, mode) in ALL_MODES {
        for (offset, len) in [(0usize, 1usize), (0, 64), (5, 100), (0, 256), (255, 1), (17, 239)] {
            let mut c = client(mtu, mode);
            let data = pattern(len);
            c.write_array(2, offset as u8, &data).unwrap();
            assert_eq!(&c.transport().map.words[offset..offset + len], &data[..], "{mode:?} mtu {mtu}");
            let mut out = std::vec![0u32; len];
            c.read_array(2, offset as u8, &mut out).unwrap();
            assert_eq!(out, data, "{mode:?} mtu {mtu} offset {offset} len {len}");
        }
    }
}

#[test]
fn array_transfers_use_expected_packet_counts() {
    // 256 words in bulk batches of 32: 256 requests, 8 responses
    let mut c = client(8, TransferMode::Bulk { max_in_flight: 32 });
    c.write_array(2, 0, &pattern(256)).unwrap();
    assert_eq!(c.transport().sent, 256);

    // CAN FD: 15 words per multiword write, 15 per read
    let mut c = client(64, TransferMode::Multiword);
    c.write_array(2, 0, &pattern(256)).unwrap();
    assert_eq!(c.transport().sent, 256usize.div_ceil(15));
    let mut out = [0u32; 256];
    c.transport_mut().sent = 0;
    c.read_array(2, 0, &mut out).unwrap();
    assert_eq!(c.transport().sent, 256usize.div_ceil(15));
}

#[test]
fn array_crossing_page_is_rejected() {
    let mut c = client(8, TransferMode::Single);
    assert_eq!(c.write_array(2, 200, &pattern(57)), Err(ClientError::InvalidArgument));
    let mut out = [0u32; 2];
    assert_eq!(c.read_array(2, 255, &mut out), Err(ClientError::InvalidArgument));
    assert_eq!(c.transport().sent, 0);
}

#[test]
fn bulk_write_reports_failing_sequence_number() {
    let mut c = client(8, TransferMode::Bulk { max_in_flight: 32 });
    // Offset 0 (magic) is read only: the first request fails and the rest are ignored
    assert_eq!(c.write_array(0, 0, &[1, 2, 3]), Err(ClientError::Server(ServerError::InvalidRegisterMode)));
    assert_eq!(c.transport().map.scratch, 0);

    // Fails on the third request (offset 3 is unimplemented); the first two land
    let mut c = client(8, TransferMode::Bulk { max_in_flight: 32 });
    let mut server = Server::new(MODE);
    let mut map = TestMap::new();
    let mut resp = [0u8; 8];
    let reqs: [[u8; 8]; 4] = [
        [0b001_0_0011, 0, 0, 1, 9, 0, 0, 0],
        [0b001_0_0011, 1, 0, 2, 5, 0, 0, 0],
        [0b001_0_0011, 2, 0, 3, 0, 0, 0, 0],
        [0b001_0_0111, 3, 0, 4, 0, 0, 0, 0],
    ];
    for req in &reqs[..3] {
        assert_eq!(server.handle_request(&mut map, req, &mut resp), 0);
    }
    assert_eq!(server.handle_request(&mut map, &reqs[3], &mut resp), 2);
    assert_eq!(resp[..2], [ServerError::InvalidRegisterAddress as u8, 2]);
    assert_eq!((map.scratch, &map.triggers[..]), (9, &[5][..]));
    assert!(!server.in_bulk());

    // Losing the request that ends the bulk transfer means no response ever arrives
    c.transport_mut().drop_request = Some(31);
    assert_eq!(c.write_array(2, 0, &pattern(32)), Err(ClientError::Transport(Timeout)));
}

#[test]
fn bulk_lost_request_is_a_sequence_error() {
    let mut c = client(8, TransferMode::Bulk { max_in_flight: 32 });
    c.transport_mut().drop_request = Some(5);
    assert_eq!(c.write_array(2, 0, &pattern(20)), Err(ClientError::Server(ServerError::BulkRequestSeqError)));
    // The server recovered and accepts the retry
    c.write_array(2, 0, &pattern(20)).unwrap();
    assert_eq!(&c.transport().map.words[..20], &pattern(20)[..]);
}

#[test]
fn normal_request_during_bulk_aborts_it() {
    let mut server = Server::new(MODE);
    let mut map = TestMap::new();
    let mut resp = [0u8; 8];
    let bulk_start = [0b001_0_0011, 0, 2, 0, 1, 0, 0, 0];
    assert_eq!(server.handle_request(&mut map, &bulk_start, &mut resp), 0);
    assert!(server.in_bulk());

    let read = [0b001_0_0000, 0, 0, 0];
    assert_eq!(server.handle_request(&mut map, &read, &mut resp), READ_RESPONSE_LEN);
    assert_eq!(resp[..5], [ServerError::BulkRequestSeqError as u8, 0, 0, 0, 0]);
    assert!(!server.in_bulk());

    // A bulk continuation without a start is out of sequence
    let bulk_cont = [0b001_0_0111, 1, 2, 1, 1, 0, 0, 0];
    assert_eq!(server.handle_request(&mut map, &bulk_cont, &mut resp), BULK_RESPONSE_LEN);
    assert_eq!(resp[0], ServerError::BulkRequestSeqError as u8);

    // And normal requests work again
    assert_eq!(server.handle_request(&mut map, &read, &mut resp), READ_RESPONSE_LEN);
    assert_eq!(resp[..5], [0, 0xF0, 0xA1, 0xBA, 0x0D]);
}

#[test]
fn malformed_requests() {
    let mut server = Server::new(MODE);
    let mut map = TestMap::new();
    let mut resp = [0u8; 8];
    let malformed = ServerError::MalformedRequest as u8;

    // Empty requests get no response
    assert_eq!(server.handle_request(&mut map, &[], &mut resp), 0);

    let cases: [(&[u8], usize); 7] = [
        (&[0b001_0_0000, 0, 0], READ_RESPONSE_LEN),                 // short read
        (&[0b001_0_0000, 0, 0, 0, 0], READ_RESPONSE_LEN),           // long read
        (&[0b001_0_0001, 0, 0, 1, 0, 0, 0], WRITE_RESPONSE_LEN),    // short write
        (&[0b001_0_1001, 2, 2, 0, 0, 0, 0, 0], WRITE_RESPONSE_LEN), // multiword write missing a word
        (&[0b001_0_1001, 0, 2], WRITE_RESPONSE_LEN),                // multiword write without a header
        (&[0b001_0_0010, 0, 2, 0], 0),                              // bulk read (bulk, so no response)
        (&[0b001_0_0101, 0, 0, 1, 0, 0, 0, 0], WRITE_RESPONSE_LEN), // bulk end without bulk
    ];
    for (req, resp_len) in cases {
        assert_eq!(server.handle_request(&mut map, req, &mut resp), resp_len, "{req:02x?}");
        if resp_len > 0 {
            assert_eq!(resp[0], malformed, "{req:02x?}");
        }
    }
    assert_eq!(map.scratch, 0);

    // Bulk and multiword together
    let req = [0b001_0_1111, 1, 2, 0, 0, 0, 0, 0];
    assert_eq!(server.handle_request(&mut map, &req, &mut resp), BULK_RESPONSE_LEN);
    assert_eq!(resp[0], malformed);
}

#[test]
fn multiword_errors() {
    // Register pages don't support multiword
    let mut c = client(64, TransferMode::Multiword);
    let mut out = [0u32; 2];
    assert_eq!(c.read_array(0, 0, &mut out), Err(ClientError::Server(ServerError::InvalidRegisterAddress)));
    assert_eq!(c.write_array(0, 1, &[1, 2]), Err(ClientError::Server(ServerError::InvalidRegisterAddress)));

    // Permissions
    assert_eq!(c.write_array(1, 0, &[1, 2]), Err(ClientError::Server(ServerError::InvalidRegisterMode)));
    assert_eq!(c.read_array(4, 0, &mut out), Err(ClientError::Server(ServerError::InvalidRegisterMode)));
    c.write_array(4, 2, &[7, 8]).unwrap();
    assert_eq!(c.transport().map.wo, [0, 0, 7, 8]);
    assert_eq!(c.write_array(4, 3, &[1, 2]), Err(ClientError::Server(ServerError::InvalidRegisterAddress)));

    // A response buffer smaller than the client's MTU limits the read size
    c.transport_mut().resp_buf_len = 8;
    let mut out = [0u32; 4];
    assert_eq!(c.read_array(2, 0, &mut out), Err(ClientError::Server(ServerError::MultiwordTooLarge)));

    // Servers without multiword support
    let mut c = client(64, TransferMode::Multiword);
    c.transport_mut().server = Server::new_without_multiword(MODE);
    assert_eq!(c.read_array(2, 0, &mut out), Err(ClientError::Server(ServerError::MultiwordUnsupported)));
    assert_eq!(c.write_array(2, 0, &out), Err(ClientError::Server(ServerError::MultiwordUnsupported)));

    // An MTU that can't carry a word
    let mut c = client(7, TransferMode::Multiword);
    assert_eq!(c.write_array(2, 0, &[1]), Err(ClientError::MtuTooSmall));
}

#[test]
fn multiword_byte_page_partial_words() {
    let mut c = client(64, TransferMode::Multiword);
    // 7 byte buffer: 2 words fit (the last one partially), 3 don't
    c.write_array(3, 0, &[0x4433_2211, 0x8877_6655]).unwrap();
    assert_eq!(c.transport().map.bytes, [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77]);
    let mut out = [0xFFFF_FFFFu32; 2];
    c.read_array(3, 0, &mut out).unwrap();
    assert_eq!(out, [0x4433_2211, 0x0077_6655]);
    let mut out = [0u32; 3];
    assert_eq!(c.read_array(3, 0, &mut out), Err(ClientError::Server(ServerError::InvalidRegisterAddress)));
    assert_eq!(c.write_array(3, 1, &[0, 0]), Err(ClientError::Server(ServerError::InvalidRegisterAddress)));

    // The version string, read in one go
    let mut out = [0u32; 3];
    c.read_array(1, 0, &mut out).unwrap();
    assert_eq!(out[2], u32::from_le_bytes([b'r', b'l', b'd', 0]));
}

#[test]
fn bytes_helpers() {
    for (mtu, mode) in ALL_MODES {
        let mut c = client(mtu, mode);
        let data: Vec<u8> = (0..=255u8).cycle().take(1021).collect();
        c.write_bytes(2, 0, &data).unwrap();
        let mut out = std::vec![0u8; 1021];
        c.read_bytes(2, 0, &mut out).unwrap();
        assert_eq!(out, data, "{mode:?}");
        // The padding of the last word is zero
        assert_eq!(c.transport().map.words[255] >> 8, 0);
        assert_eq!(c.write_bytes(2, 1, &data), Err(ClientError::InvalidArgument));
    }
}
