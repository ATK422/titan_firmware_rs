//! `titan_link` over UDP, for boards on Ethernet (the Puddles backplane).
//!
//! One message per datagram, framed as described in `puddles_core::wire`: the bridge listens on the bind address
//! of `--interface` (default `0.0.0.0:2203`), sends to the board's address (default `192.168.1.42:2203`), and only
//! accepts datagrams from the board's IP. There are no CANmore heartbeats or utility frames on this transport.

use std::io::ErrorKind;
use std::net::{SocketAddr, UdpSocket};
use std::sync::Mutex;
use std::time::Duration;

use anyhow::{Context, Result, anyhow, bail};
use puddles_core::wire::{self, LinkSpec, MAX_DATAGRAM_LEN};
use titan_link::HostToDevice;

use crate::link::{Event, Link};

/// How long [`Link::receive`] waits before returning [`Event::Idle`].
const RECEIVE_TIMEOUT: Duration = Duration::from_millis(250);

pub struct UdpLink {
    socket: UdpSocket,
    /// Where the board is. The port follows the board's datagrams; the IP is fixed by the spec.
    board: Mutex<SocketAddr>,
}

impl UdpLink {
    /// Opens the link described by an `--interface` spec (see [`LinkSpec`]).
    pub fn open(spec: &str) -> Result<Self> {
        let spec = LinkSpec::parse(spec).map_err(|_| {
            anyhow!("invalid UDP interface {spec:?}: expected [BIND_IP[:PORT] | :PORT][,BOARD_IP[:PORT]]")
        })?;
        Self::bind(spec.bind.into(), spec.board.into())
    }

    /// Listens on `bind` and talks to the board at `board`.
    pub fn bind(bind: SocketAddr, board: SocketAddr) -> Result<Self> {
        let socket = UdpSocket::bind(bind).with_context(|| format!("binding UDP socket to {bind}"))?;
        socket.set_read_timeout(Some(RECEIVE_TIMEOUT))?;
        Ok(UdpLink { socket, board: Mutex::new(board) })
    }

    /// The address the bridge listens on.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn local_addr(&self) -> Result<SocketAddr> {
        Ok(self.socket.local_addr()?)
    }
}

impl Link for UdpLink {
    fn send(&self, msg: &HostToDevice) -> Result<()> {
        let mut buf = [0u8; MAX_DATAGRAM_LEN];
        let datagram = wire::encode(msg, &mut buf).map_err(|err| anyhow!("encoding {msg:?}: {err:?}"))?;
        let board = *self.board.lock().unwrap();
        self.socket.send_to(datagram, board).with_context(|| format!("sending to {board}"))?;
        Ok(())
    }

    fn receive(&self) -> Result<Event> {
        let mut buf = [0u8; MAX_DATAGRAM_LEN + 1];
        let (len, from) = match self.socket.recv_from(&mut buf) {
            Ok(received) => received,
            Err(err) if matches!(err.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => return Ok(Event::Idle),
            // Linux reports an ICMP port unreachable from an earlier send this way; the board isn't up yet
            Err(err) if err.kind() == ErrorKind::ConnectionRefused => return Ok(Event::Idle),
            Err(err) => return Err(err).context("receiving UDP datagram"),
        };
        let mut board = self.board.lock().unwrap();
        if from.ip() != board.ip() {
            bail!("ignoring datagram from {from} (the board is {})", board.ip());
        }
        let msg = wire::decode(&buf[..len]).map_err(|err| anyhow!("undecodable datagram from {from}: {err:?}"))?;
        // Answer to wherever the board sends from
        *board = from;
        Ok(Event::Message(msg))
    }
}

#[cfg(test)]
mod tests {
    //! Loopback tests: a plain UDP socket plays the board.

    use super::*;
    use std::net::Ipv4Addr;

    use titan_link::puddles_backplane::{self, RpmFeedback};
    use titan_link::{DeviceToHost, MAX_ENCODED_LEN};

    fn loopback() -> (UdpLink, UdpSocket) {
        let board = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        board.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
        let link = UdpLink::bind((Ipv4Addr::LOCALHOST, 0).into(), board.local_addr().unwrap()).unwrap();
        (link, board)
    }

    #[test]
    fn messages_cross_in_both_directions() {
        let (link, board) = loopback();

        let ping = HostToDevice::Ping { protocol_version: titan_link::PROTOCOL_VERSION };
        link.send(&ping).unwrap();
        let mut buf = [0u8; 512];
        let (len, from) = board.recv_from(&mut buf).unwrap();
        assert_eq!(from, link.local_addr().unwrap());
        assert_eq!(&buf[..2], b"TL");
        assert_eq!(wire::decode::<HostToDevice>(&buf[..len]).unwrap(), ping);

        let reply = DeviceToHost::PuddlesBackplane(puddles_backplane::DeviceToHost::Rpm(RpmFeedback {
            valid_mask: 0x81,
            rpm: [1200, 0, 0, 0, 0, 0, 0, -1200],
        }));
        let datagram = wire::encode(&reply, &mut buf).unwrap();
        board.send_to(datagram, link.local_addr().unwrap()).unwrap();
        match link.receive().unwrap() {
            Event::Message(msg) => assert_eq!(msg, reply),
            _ => panic!("expected a message"),
        }
    }

    #[test]
    fn replies_follow_the_board_port() {
        let (link, _old) = loopback();
        let moved = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        moved.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
        let mut buf = [0u8; 512];
        let heartbeat = wire::encode(&DeviceToHost::Heartbeat, &mut buf).unwrap();
        moved.send_to(heartbeat, link.local_addr().unwrap()).unwrap();
        assert!(matches!(link.receive().unwrap(), Event::Message(DeviceToHost::Heartbeat)));

        link.send(&HostToDevice::KillSwitch { asserting_kill: true }).unwrap();
        let (len, _) = moved.recv_from(&mut buf).unwrap();
        assert_eq!(
            wire::decode::<HostToDevice>(&buf[..len]).unwrap(),
            HostToDevice::KillSwitch { asserting_kill: true }
        );
    }

    #[test]
    fn rejects_garbage_and_strangers() {
        let board = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        // The board lives on another address, so loopback traffic is from a stranger
        let link = UdpLink::bind((Ipv4Addr::LOCALHOST, 0).into(), "192.0.2.1:2203".parse().unwrap()).unwrap();
        let mut buf = [0u8; MAX_DATAGRAM_LEN];
        let datagram = wire::encode(&DeviceToHost::Heartbeat, &mut buf).unwrap();
        board.send_to(datagram, link.local_addr().unwrap()).unwrap();
        assert!(link.receive().is_err());

        let (link, board) = loopback();
        board.send_to(b"not titan_link", link.local_addr().unwrap()).unwrap();
        assert!(link.receive().is_err());
        let mut oversized = vec![0u8; MAX_ENCODED_LEN + 3];
        oversized[..2].copy_from_slice(b"TL");
        board.send_to(&oversized, link.local_addr().unwrap()).unwrap();
        assert!(link.receive().is_err());
    }

    #[test]
    fn idle_when_quiet() {
        let (link, _board) = loopback();
        assert!(matches!(link.receive().unwrap(), Event::Idle));
    }

    #[test]
    fn interface_specs() {
        assert!(UdpLink::open("127.0.0.1:0").is_ok());
        let err = UdpLink::open("can0").err().unwrap();
        assert!(format!("{err:#}").contains("invalid UDP interface"));
    }
}
