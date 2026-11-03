//! Remote TTY over CAN (`canmore/remote_tty_interface.h`), ported from `canmore_cpp/RemoteTTYStream.cpp` and
//! `canmore_dbg_cpp/RemoteTTYClient.cpp`.
//!
//! A shell on a Linux machine is bridged over CANmore utility channel 13. Extended frames carry a 2-bit subchannel
//! (control, stdin, stdout, stderr) and a 16-bit sequence number or command in the extra field. Stream packets are
//! acknowledged in batches and retransmitted when an acknowledgement repeats, turning the bus into a reliable
//! stream. Frames are padded to a valid CAN FD length with zeros, which the receiver strips again (so trailing NUL
//! bytes in a packet are lost, as in the C++ implementation).

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use anyhow::{Result, bail};
use canmore::id::{self, Direction, Fields, FrameType};

use super::can::{CAN_EFF_FLAG, CAN_RTR_FLAG, CanBus, RawFrame};

pub const SUBCH_CONTROL: u8 = 0;
pub const SUBCH_STDIN: u8 = 1;
pub const SUBCH_STDOUT: u8 = 2;
pub const SUBCH_STDERR: u8 = 3;

pub const CMD_DISCONNECT: u16 = 0;
pub const CMD_ACK: u16 = 1;
pub const CMD_WINDOW_SIZE: u16 = 2;

pub const DEFAULT_MAX_IN_FLIGHT: usize = 24;
pub const DEFAULT_RX_PACKETS_PER_ACK: u16 = 16;
pub const DEFAULT_ACK_INTERVAL: Duration = Duration::from_millis(500);

/// The extended identifier of a remote TTY frame.
pub fn frame_id(client_id: u8, direction: Direction, subch: u8, seq_cmd: u16) -> u32 {
    let extra = ((subch as u32 & 0b11) << 16) | seq_cmd as u32;
    id::ext_id(client_id, FrameType::Util, direction, canmore::CHAN_REMOTE_TTY, extra)
}

/// Splits an identifier into `(subchannel, seq/cmd)`.
pub fn decode_id(id: u32) -> (u8, u16) {
    (((id >> 16) & 0b11) as u8, id as u16)
}

/// The next sequence number after `seq`, skipping 0 (reserved for "reset").
fn next_seq(seq: u16) -> u16 {
    match seq.wrapping_add(1) {
        0 => 1,
        n => n,
    }
}

/// Transmit side of a stream: numbers packets and keeps them until acknowledged.
pub struct TxScheduler {
    pub max_in_flight: usize,
    last_acked: u16,
    unacked: VecDeque<(u8, Vec<u8>)>,
}

impl TxScheduler {
    pub fn new(max_in_flight: usize) -> Self {
        TxScheduler { max_in_flight, last_acked: 0, unacked: VecDeque::new() }
    }

    pub fn space_available(&self) -> bool {
        self.unacked.len() < self.max_in_flight
    }

    fn seq_at(&self, index: usize) -> u16 {
        let mut seq = self.last_acked;
        for _ in 0..=index {
            seq = next_seq(seq);
        }
        seq
    }

    /// Queues `data` on `stream`. Returns the packet to transmit as `(stream, seq, padded data)`, or `None` if the
    /// data was all zeros (nothing to send).
    pub fn write(&mut self, stream: u8, data: &[u8]) -> Result<Option<(u8, u16, Vec<u8>)>> {
        if !self.space_available() {
            bail!("Cannot write to full TTY transmit buffer");
        }
        let Some(last) = data.iter().rposition(|&b| b != 0) else { return Ok(None) };
        let len = last + 1;
        let frame_len = canmore::fd_frame_len(len);
        if len > canmore::MAX_FD_FRAME_SIZE {
            bail!("Attempting to transmit remote tty packet greater than max frame size");
        }
        let mut payload = data[..len].to_vec();
        payload.resize(frame_len, 0);
        self.unacked.push_back((stream, payload.clone()));
        Ok(Some((stream, self.seq_at(self.unacked.len() - 1), payload)))
    }

    /// Handles an acknowledgement. Returns packets to retransmit (when the ack repeats the last one) and whether
    /// buffer space was freed.
    pub fn notify_ack(&mut self, seq: u16) -> (Vec<(u8, u16, Vec<u8>)>, bool) {
        if seq == 0 {
            // The receiver (re)started: forget everything in flight
            self.last_acked = 0;
            self.unacked.clear();
            return (Vec::new(), true);
        }
        if seq == self.last_acked {
            // Nothing new arrived since the last ack: something was lost, send everything again
            let resend = self.unacked.iter().enumerate().map(|(i, (s, d))| (*s, self.seq_at(i), d.clone())).collect();
            return (resend, false);
        }
        let mut acked_idx = seq.wrapping_sub(self.last_acked).wrapping_sub(1) as usize;
        if seq < self.last_acked {
            // Rolled over past 0, which isn't a valid sequence number
            acked_idx = acked_idx.wrapping_sub(1) & 0xFFFF;
        }
        if acked_idx >= self.unacked.len() {
            // Unknown sequence number: the receiver lost its state, resynchronize to it
            self.unacked.clear();
        } else {
            self.unacked.drain(..=acked_idx);
        }
        self.last_acked = seq;
        (Vec::new(), true)
    }
}

/// Receive side of a stream: accepts packets in order and decides when to acknowledge.
pub struct RxScheduler {
    pub max_before_ack: u16,
    pub ack_interval: Duration,
    unacked: u16,
    last_received: u16,
    next_ack: Instant,
}

impl RxScheduler {
    pub fn new(max_before_ack: u16, ack_interval: Duration) -> Self {
        RxScheduler { max_before_ack, ack_interval, unacked: 0, last_received: 0, next_ack: Instant::now() }
    }

    /// Checks a received packet's sequence number. Returns `(accept, ack to send)`.
    pub fn check_packet(&mut self, seq: u16) -> (bool, Option<u16>) {
        if seq != next_seq(self.last_received) {
            return (false, None);
        }
        self.last_received = seq;
        self.unacked += 1;
        self.next_ack = Instant::now() + self.ack_interval;
        if self.unacked >= self.max_before_ack {
            self.unacked = 0;
            return (true, Some(seq));
        }
        (true, None)
    }

    /// Call periodically. Returns an ack to send if the interval passed without a valid packet, and the time until
    /// the next call is needed.
    pub fn handle_timer(&mut self) -> (Option<u16>, Duration) {
        let now = Instant::now();
        if self.next_ack <= now {
            self.next_ack = now + self.ack_interval;
            self.unacked = 0;
            (Some(self.last_received), self.ack_interval)
        } else {
            (None, (self.next_ack - now).max(Duration::from_millis(1)))
        }
    }
}

/// What a received frame meant.
#[derive(Debug, PartialEq, Eq)]
pub enum TtyEvent {
    Stdout(Vec<u8>),
    Stderr(Vec<u8>),
    /// The host disconnected (`true` if because of an error).
    Disconnected(bool),
    /// Transmit buffer space became available.
    StdinReady,
    None,
}

/// The agent side of a remote TTY session on a CAN bus.
pub struct RemoteTtyClient {
    bus: CanBus,
    client_id: u8,
    tx: TxScheduler,
    rx: RxScheduler,
    disconnected: Option<bool>,
}

impl RemoteTtyClient {
    pub fn open(interface: &str, client_id: u8) -> Result<Self> {
        let bus = CanBus::open(interface)?;
        // Match client, type, direction and channel; any subchannel and sequence number
        let filter_id = frame_id(client_id, Direction::ClientToAgent, 0, 0) | CAN_EFF_FLAG;
        let filter_mask = CAN_EFF_FLAG | CAN_RTR_FLAG | (0x1F << 24) | (1 << 23) | (1 << 22) | (0xF << 18);
        bus.set_filters(&[(filter_id, filter_mask)])?;
        let mut client = RemoteTtyClient {
            bus,
            client_id,
            tx: TxScheduler::new(DEFAULT_MAX_IN_FLIGHT),
            rx: RxScheduler::new(DEFAULT_RX_PACKETS_PER_ACK, DEFAULT_ACK_INTERVAL),
            disconnected: None,
        };
        client.tick()?;
        Ok(client)
    }

    pub fn bus(&self) -> &CanBus {
        &self.bus
    }

    /// Largest stdin chunk per frame.
    pub fn max_frame_size(&self) -> usize {
        if self.bus.is_fd() { canmore::MAX_FD_FRAME_SIZE } else { canmore::MAX_FRAME_SIZE }
    }

    pub fn disconnected(&self) -> Option<bool> {
        self.disconnected
    }

    fn send(&self, subch: u8, seq_cmd: u16, data: &[u8]) -> Result<()> {
        let id = frame_id(self.client_id, Direction::AgentToClient, subch, seq_cmd);
        self.bus.write(id, true, data)?;
        Ok(())
    }

    fn send_control(&self, cmd: u16, data: &[u8]) -> Result<()> {
        if self.disconnected.is_some() {
            return Ok(());
        }
        self.send(SUBCH_CONTROL, cmd, data)
    }

    pub fn disconnect(&mut self, is_error: bool) -> Result<()> {
        if self.disconnected.is_none() {
            self.send_control(CMD_DISCONNECT, &[is_error as u8])?;
            self.disconnected = Some(is_error);
        }
        Ok(())
    }

    pub fn send_window_size(&self, rows: u16, cols: u16) -> Result<()> {
        let mut data = [0u8; 4];
        data[..2].copy_from_slice(&rows.to_le_bytes());
        data[2..].copy_from_slice(&cols.to_le_bytes());
        self.send_control(CMD_WINDOW_SIZE, &data)
    }

    pub fn stdin_can_write(&self) -> bool {
        self.tx.space_available()
    }

    pub fn stdin_write(&mut self, data: &[u8]) -> Result<()> {
        if self.disconnected.is_some() {
            return Ok(());
        }
        for chunk in data.chunks(self.max_frame_size()) {
            if let Some((stream, seq, payload)) = self.tx.write(SUBCH_STDIN, chunk)? {
                self.send(stream, seq, &payload)?;
            }
        }
        Ok(())
    }

    /// Sends a pending periodic ack. Returns how long until the next tick is due.
    pub fn tick(&mut self) -> Result<Duration> {
        let (ack, next) = self.rx.handle_timer();
        if let Some(seq) = ack {
            self.send_control(CMD_ACK, &seq.to_le_bytes())?;
        }
        Ok(next)
    }

    /// Waits up to `timeout` for a frame and handles it.
    pub fn poll(&mut self, timeout: Duration) -> Result<TtyEvent> {
        match self.bus.read(timeout)? {
            Some(frame) => self.handle_frame(&frame),
            None => Ok(TtyEvent::None),
        }
    }

    pub fn handle_frame(&mut self, frame: &RawFrame) -> Result<TtyEvent> {
        let fields = Fields::decode(frame.id, frame.extended);
        if !frame.extended || fields.client_id != self.client_id || fields.noc != canmore::CHAN_REMOTE_TTY {
            return Ok(TtyEvent::None);
        }
        let (subch, seq_cmd) = decode_id(frame.id);
        let data = &frame.data;
        if subch == SUBCH_CONTROL {
            return Ok(match (seq_cmd, data.len()) {
                (CMD_DISCONNECT, 1) => {
                    self.disconnected = Some(data[0] != 0);
                    TtyEvent::Disconnected(data[0] != 0)
                }
                (CMD_ACK, 2) => {
                    let (resend, freed) = self.tx.notify_ack(u16::from_le_bytes([data[0], data[1]]));
                    for (stream, seq, payload) in resend {
                        self.send(stream, seq, &payload)?;
                    }
                    if freed { TtyEvent::StdinReady } else { TtyEvent::None }
                }
                _ => TtyEvent::None,
            });
        }
        let (accept, ack) = self.rx.check_packet(seq_cmd);
        if let Some(seq) = ack {
            self.send_control(CMD_ACK, &seq.to_le_bytes())?;
        }
        if !accept {
            return Ok(TtyEvent::None);
        }
        let len = data.iter().rposition(|&b| b != 0).map_or(0, |i| i + 1);
        let payload = data[..len].to_vec();
        Ok(match subch {
            _ if payload.is_empty() => TtyEvent::None,
            SUBCH_STDOUT => TtyEvent::Stdout(payload),
            SUBCH_STDERR => TtyEvent::Stderr(payload),
            _ => TtyEvent::None,
        })
    }
}

impl Drop for RemoteTtyClient {
    fn drop(&mut self) {
        let _ = self.disconnect(true);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids() {
        let id = frame_id(7, Direction::ClientToAgent, SUBCH_STDERR, 0xBEEF);
        let fields = Fields::decode(id, true);
        assert_eq!((fields.client_id, fields.noc, fields.frame_type), (7, canmore::CHAN_REMOTE_TTY, FrameType::Util));
        assert_eq!(decode_id(id), (SUBCH_STDERR, 0xBEEF));
    }

    #[test]
    fn tx_pads_and_acks() {
        let mut tx = TxScheduler::new(3);
        let (_, seq, data) = tx.write(SUBCH_STDIN, b"0123456789\0\0").unwrap().unwrap();
        assert_eq!(seq, 1);
        assert_eq!(data.len(), 12, "padded to the next CAN FD length");
        assert_eq!(&data[..10], b"0123456789");
        assert_eq!(tx.write(SUBCH_STDIN, &[0, 0]).unwrap(), None);
        assert_eq!(tx.write(SUBCH_STDIN, b"a").unwrap().unwrap().1, 2);
        assert_eq!(tx.write(SUBCH_STDIN, b"b").unwrap().unwrap().1, 3);
        assert!(!tx.space_available());
        assert!(tx.write(SUBCH_STDIN, b"c").is_err());

        // Repeated ack of the last acked number resends everything outstanding
        let (resend, freed) = tx.notify_ack(0);
        assert!(resend.is_empty() && freed && tx.space_available());
        tx.write(SUBCH_STDIN, b"x").unwrap();
        tx.write(SUBCH_STDIN, b"y").unwrap();
        let (resend, _) = tx.notify_ack(0);
        assert!(resend.is_empty(), "ack 0 resets");
        tx.write(SUBCH_STDIN, b"x").unwrap();
        tx.write(SUBCH_STDIN, b"y").unwrap();
        let (_, freed) = tx.notify_ack(1);
        assert!(freed);
        let (resend, freed) = tx.notify_ack(1);
        assert!(!freed);
        assert_eq!(resend, vec![(SUBCH_STDIN, 2, b"y".to_vec())]);
    }

    #[test]
    fn tx_sequence_skips_zero() {
        let mut tx = TxScheduler::new(4);
        tx.last_acked = 0xFFFE;
        assert_eq!(tx.write(SUBCH_STDIN, b"a").unwrap().unwrap().1, 0xFFFF);
        assert_eq!(tx.write(SUBCH_STDIN, b"b").unwrap().unwrap().1, 1);
        // Acking across the rollover frees both
        let (_, freed) = tx.notify_ack(1);
        assert!(freed);
        assert!(tx.unacked.is_empty());
    }

    #[test]
    fn rx_in_order_and_acks() {
        let mut rx = RxScheduler::new(2, Duration::from_secs(60));
        assert_eq!(rx.check_packet(2), (false, None));
        assert_eq!(rx.check_packet(1), (true, None));
        assert_eq!(rx.check_packet(2), (true, Some(2)));
        assert_eq!(rx.check_packet(2), (false, None));
        let (ack, next) = rx.handle_timer();
        assert_eq!(ack, None);
        assert!(next > Duration::from_secs(50));
    }
}
