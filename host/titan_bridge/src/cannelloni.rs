//! CAN over UDP in [cannelloni](https://github.com/mguentner/cannelloni)'s framing, so the bridge reaches a CAN
//! bus without SocketCAN: `--interface cannelloni:HOST:PORT` sends to a cannelloni **hub** at that address (the
//! firmware simulator's `--cannelloni BUS=HOST:PORT`) and takes the bus's frames back. That runs where there is no
//! `vcan` (WSL2, macOS, a laptop without sudo).
//!
//! A hub learns its peers from the datagrams they send and forwards every bus frame (and every other peer's) to all
//! of them, so this side filters for its client the way the SocketCAN filters do ([`crate::can::accepts`]), and
//! sends an empty datagram at least every [`KEEPALIVE`] so the hub keeps it while there is nothing to say.
//!
//! A datagram is a five byte header, then the frames:
//!
//! ```text
//! version (2) | op code (0 = data) | sequence number | frame count (u16, big endian)
//! per frame:  can_id (u32, big endian, Linux flags: 0x80000000 extended, 0x40000000 remote, 0x20000000 error)
//!             len (u8, | 0x80 for a CAN FD frame) | [CAN FD flags (u8: 1 = BRS, 2 = ESI)] | data (none if remote)
//! ```

use std::collections::VecDeque;
use std::io::ErrorKind;
use std::net::{SocketAddr, ToSocketAddrs, UdpSocket};
use std::sync::Mutex;
use std::sync::atomic::{AtomicU8, Ordering};
use std::time::{Duration, Instant};

use anyhow::{Context, Result, anyhow, bail};

use crate::can::{self, Frame, FrameIo};

/// How `--interface` names CAN over UDP to a cannelloni hub: `cannelloni:HOST:PORT`.
pub const PREFIX: &str = "cannelloni:";
/// A peer that has sent nothing for this long sends an empty datagram, so the hub keeps it (it forgets peers it
/// has not heard from for 10 s).
pub const KEEPALIVE: Duration = Duration::from_secs(1);
/// How long [`FrameIo::read`] waits for a datagram.
const RECEIVE_TIMEOUT: Duration = Duration::from_millis(250);

const VERSION: u8 = 2;
const OP_DATA: u8 = 0;
const HEADER_LEN: usize = 5;
const CAN_EFF_FLAG: u32 = 0x8000_0000;
const CAN_RTR_FLAG: u32 = 0x4000_0000;
const CAN_ERR_FLAG: u32 = 0x2000_0000;
const CANFD_FRAME: u8 = 0x80;
const CANFD_BRS: u8 = 0x01;

/// One datagram with `frames` (CAN FD frames with bit rate switching, as SocketCAN sends them here).
pub fn encode(frames: &[Frame], seq: u8) -> Result<Vec<u8>> {
    let mut p = vec![VERSION, OP_DATA, seq];
    p.extend_from_slice(&u16::try_from(frames.len()).context("too many frames")?.to_be_bytes());
    for f in frames {
        if f.data.len() > if f.fd { 64 } else { 8 } {
            bail!("{} byte CAN{} frame", f.data.len(), if f.fd { " FD" } else { "" });
        }
        let id = if f.extended { f.id & 0x1FFF_FFFF | CAN_EFF_FLAG } else { f.id & 0x7FF };
        p.extend_from_slice(&id.to_be_bytes());
        if f.fd {
            p.push(f.data.len() as u8 | CANFD_FRAME);
            p.push(CANFD_BRS);
        } else {
            p.push(f.data.len() as u8);
        }
        p.extend_from_slice(&f.data);
    }
    Ok(p)
}

/// The data frames in a datagram (remote and error frames are left out).
pub fn decode(p: &[u8]) -> Result<Vec<Frame>> {
    if p.len() < HEADER_LEN {
        bail!("{} byte datagram", p.len());
    }
    if p[0] != VERSION || p[1] != OP_DATA {
        bail!("not a cannelloni v2 data datagram (version {}, op code {})", p[0], p[1]);
    }
    let count = u16::from_be_bytes([p[3], p[4]]) as usize;
    let mut at = HEADER_LEN;
    let mut take = |n: usize| -> Result<&[u8]> {
        let s = p.get(at..at + n).ok_or_else(|| anyhow!("datagram ends inside a frame ({} bytes)", p.len()))?;
        at += n;
        Ok(s)
    };
    let mut frames = Vec::with_capacity(count);
    for _ in 0..count {
        let id = u32::from_be_bytes(take(4)?.try_into().unwrap());
        let len_byte = take(1)?[0];
        let fd = len_byte & CANFD_FRAME != 0;
        let len = (len_byte & !CANFD_FRAME) as usize;
        if fd {
            take(1)?;
        }
        if len > if fd { 64 } else { 8 } {
            bail!("{len} byte frame");
        }
        // As cannelloni's parser: no data follows an id with the remote flag
        if id & CAN_RTR_FLAG != 0 {
            continue;
        }
        let data = take(len)?.to_vec();
        if id & CAN_ERR_FLAG != 0 {
            continue;
        }
        let extended = id & CAN_EFF_FLAG != 0;
        frames.push(Frame { id: if extended { id & 0x1FFF_FFFF } else { id & 0x7FF }, extended, fd, data });
    }
    Ok(frames)
}

/// `HOST:PORT`, resolved now.
pub fn parse_addr(spec: &str) -> Result<SocketAddr> {
    spec.to_socket_addrs()
        .ok()
        .and_then(|mut a| a.next())
        .ok_or_else(|| anyhow!("cannelloni hub address {spec:?}: expected {PREFIX}HOST:PORT"))
}

/// The bus through a cannelloni hub, for one client.
pub struct CannelloniIo {
    socket: UdpSocket,
    hub: SocketAddr,
    client_id: u8,
    seq: AtomicU8,
    last_tx: Mutex<Option<Instant>>,
    /// Frames for this client from datagrams already read.
    queue: Mutex<VecDeque<Frame>>,
}

impl CannelloniIo {
    /// Talks to the hub at `addr` (`HOST:PORT`) from a port of its own, for `client_id`'s frames.
    pub fn open(addr: &str, client_id: u8) -> Result<Self> {
        let hub = parse_addr(addr)?;
        let any: SocketAddr = if hub.is_ipv4() { ([0, 0, 0, 0], 0).into() } else { "[::]:0".parse().unwrap() };
        let socket = UdpSocket::bind(any).with_context(|| format!("binding a UDP socket for {hub}"))?;
        // Connected: only the hub's datagrams arrive
        socket.connect(hub).with_context(|| format!("connecting to {hub}"))?;
        socket.set_read_timeout(Some(RECEIVE_TIMEOUT))?;
        let io = CannelloniIo {
            socket,
            hub,
            client_id,
            seq: AtomicU8::new(0),
            last_tx: Mutex::new(None),
            queue: Mutex::new(VecDeque::new()),
        };
        // Introduce ourselves, so the hub sends the bus's frames from now on
        io.keepalive();
        Ok(io)
    }

    #[cfg(test)]
    pub fn local_addr(&self) -> SocketAddr {
        self.socket.local_addr().unwrap()
    }

    fn send(&self, frames: &[Frame]) -> Result<()> {
        let p = encode(frames, self.seq.fetch_add(1, Ordering::Relaxed))?;
        *self.last_tx.lock().unwrap() = Some(Instant::now());
        match self.socket.send(&p) {
            Ok(_) => Ok(()),
            // An earlier datagram's ICMP port unreachable: the hub is not up (yet). Send once more, as the error
            // is consumed; if that fails too, the frames are lost like frames on a bus nobody listens to
            Err(e) if e.kind() == ErrorKind::ConnectionRefused => match self.socket.send(&p) {
                Err(e) if e.kind() == ErrorKind::ConnectionRefused => Ok(()),
                r => r.map(|_| ()).with_context(|| format!("sending to {}", self.hub)),
            },
            Err(e) => Err(e).with_context(|| format!("sending to {}", self.hub)),
        }
    }

    /// An empty datagram if nothing went to the hub for [`KEEPALIVE`].
    fn keepalive(&self) {
        let due = self.last_tx.lock().unwrap().is_none_or(|t| t.elapsed() >= KEEPALIVE);
        if due {
            let _ = self.send(&[]);
        }
    }
}

impl FrameIo for CannelloniIo {
    fn write(&self, frames: &[Frame]) -> Result<()> {
        self.send(frames)
    }

    fn read(&self) -> Result<Option<Frame>> {
        if let Some(f) = self.queue.lock().unwrap().pop_front() {
            return Ok(Some(f));
        }
        self.keepalive();
        let mut buf = [0u8; 65536];
        let n = match self.socket.recv(&mut buf) {
            Ok(n) => n,
            Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => return Ok(None),
            // The hub is not up (yet)
            Err(e) if matches!(e.kind(), ErrorKind::ConnectionRefused | ErrorKind::ConnectionReset) => return Ok(None),
            Err(e) => return Err(e).with_context(|| format!("receiving from {}", self.hub)),
        };
        let frames = decode(&buf[..n]).with_context(|| format!("datagram from {}", self.hub))?;
        let mut queue = self.queue.lock().unwrap();
        queue.extend(frames.into_iter().filter(|f| can::accepts(self.client_id, f.id, f.extended)));
        Ok(queue.pop_front())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use canmore::id::{Direction, util_id};

    fn hub() -> UdpSocket {
        let hub = UdpSocket::bind("127.0.0.1:0").unwrap();
        hub.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
        hub
    }

    fn recv(hub: &UdpSocket) -> (Vec<Frame>, SocketAddr) {
        let mut buf = [0u8; 2048];
        let (n, from) = hub.recv_from(&mut buf).unwrap();
        (decode(&buf[..n]).unwrap(), from)
    }

    /// The layout of the emulator's and cannelloni's datagrams.
    #[test]
    fn datagram_layout() {
        let frames = [
            Frame { id: 0x123, extended: false, fd: false, data: vec![0xAA, 0xBB] },
            Frame { id: 0x1234567, extended: true, fd: true, data: vec![1; 12] },
        ];
        let p = encode(&frames, 7).unwrap();
        assert_eq!(&p[..12], &[2, 0, 7, 0, 2, 0, 0, 0x01, 0x23, 2, 0xAA, 0xBB]);
        assert_eq!(&p[12..18], &[0x81, 0x23, 0x45, 0x67, 0x80 | 12, CANFD_BRS]);
        assert_eq!(decode(&p).unwrap(), frames);
        assert_eq!(decode(&encode(&[], 0).unwrap()).unwrap(), [], "a keepalive");
        // A remote frame (no data follows) and an error frame are left out
        let p = [2, 0, 0, 0, 3, 0x40, 0, 0, 0x10, 4, 0x20, 0, 0, 0, 1, 0, 0, 0, 0, 0x11, 1, 9];
        assert_eq!(decode(&p).unwrap(), [Frame { id: 0x11, extended: false, fd: false, data: vec![9] }]);
        assert!(decode(&[2, 0, 0, 0, 1, 0, 0, 1, 0x23, 2, 0xAA]).is_err(), "cut short");
        assert!(decode(&[3, 0, 0, 0, 0]).is_err(), "another version");
        assert!(encode(&[Frame { id: 1, extended: false, fd: false, data: vec![0; 9] }], 0).is_err());
        assert!(parse_addr("localhost:20000").is_ok());
        assert!(parse_addr("127.0.0.1").is_err());
    }

    /// The bridge introduces itself, sends its frames to the hub in one datagram per message, and keeps only its
    /// client's frames to the agent from what the hub forwards.
    #[test]
    fn talks_to_a_hub() {
        let hub = hub();
        let io = CannelloniIo::open(&hub.local_addr().unwrap().to_string(), 2).unwrap();
        let (hello, from) = recv(&hub);
        assert!(hello.is_empty(), "a keepalive first");
        assert_eq!(from, io.local_addr());
        let msg = [
            Frame { id: 0x10, extended: false, fd: true, data: vec![1; 20] },
            Frame { id: 0x11, extended: false, fd: true, data: vec![2; 3] },
        ];
        io.write(&msg).unwrap();
        assert_eq!(recv(&hub).0, msg);

        let mine = Frame { id: util_id(2, Direction::ClientToAgent, 0), extended: false, fd: false, data: vec![1] };
        let other = Frame { id: util_id(3, Direction::ClientToAgent, 0), extended: false, fd: false, data: vec![1] };
        let command = Frame { id: util_id(2, Direction::AgentToClient, 0), extended: false, fd: false, data: vec![0] };
        let p = encode(&[other, mine.clone(), command, mine.clone()], 0).unwrap();
        hub.send_to(&p, from).unwrap();
        assert_eq!(io.read().unwrap(), Some(mine.clone()));
        assert_eq!(io.read().unwrap(), Some(mine), "the rest of the datagram");
        assert_eq!(io.read().unwrap(), None);
        hub.send_to(b"junk", from).unwrap();
        assert!(io.read().is_err());
    }

    /// A quiet bridge keeps itself known to the hub; a hub that is not up loses frames without failing the link.
    #[test]
    fn keepalives_and_a_missing_hub() {
        let hub = hub();
        let io = CannelloniIo::open(&hub.local_addr().unwrap().to_string(), 2).unwrap();
        recv(&hub);
        *io.last_tx.lock().unwrap() = Some(Instant::now() - KEEPALIVE);
        assert_eq!(io.read().unwrap(), None);
        assert!(recv(&hub).0.is_empty(), "a keepalive while reading");

        let addr = hub.local_addr().unwrap();
        drop(hub);
        let io = CannelloniIo::open(&addr.to_string(), 2).unwrap();
        for _ in 0..3 {
            io.write(&[Frame { id: 1, extended: false, fd: false, data: vec![] }]).unwrap();
            assert_eq!(io.read().unwrap(), None);
        }
    }
}
