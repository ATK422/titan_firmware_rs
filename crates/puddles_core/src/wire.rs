//! `titan_link` over UDP: the Puddles transport between the backplane and `titan_bridge`.
//!
//! The C backplane ran micro-ROS (DDS-XRCE) over UDP to an agent on the robot computer. The Rust backplane sends
//! `titan_link` messages instead, over the same Ethernet link:
//!
//! * **Endpoints.** The backplane (`192.168.1.42`, [`BOARD_IP`]) and the bridge on the robot computer
//!   (`192.168.1.22`, `titan_rp2040::puddles::ROBOT_COMPUTER_IP`) both use UDP port [`PORT`] (2203). It sits next
//!   to the CANmore Ethernet ports (heartbeat broadcast 2201, control interface 2202), below the DDS range, and
//!   apart from the micro-ROS agent's 8888, so a C board and the bridge can't be confused with each other.
//! * **Datagrams.** Each datagram carries exactly one message: the two magic bytes [`MAGIC`] (`"TL"`) followed by
//!   the `postcard` encoding of one `titan_link::HostToDevice` (bridge to board) or `titan_link::DeviceToHost`
//!   (board to bridge), with no trailing bytes. Datagrams are at most [`MAX_DATAGRAM_LEN`] bytes. Anything else is
//!   dropped, so stray traffic (a micro-ROS client, a port scan) can't be mistaken for a message.
//! * **Addressing.** The board accepts datagrams only from the robot computer's IP and answers to the port the last
//!   one came from (initially [`PORT`]). The bridge sends to the board's address and accepts datagrams only from
//!   the board's IP. Like everything on the link, delivery is best effort: the 100 ms ping and the periodic
//!   messages cover losses, exactly as they do over CAN.

use core::net::{Ipv4Addr, SocketAddrV4};

use serde::{Deserialize, Serialize};

/// UDP port of the `titan_link` socket, on both the board and the bridge.
pub const PORT: u16 = 2203;
/// Static IP of the backplane (`ETHERNET_IP` in `boards/mk2/puddles_backplane.h`).
pub const BOARD_IP: [u8; 4] = [192, 168, 1, 42];
/// First bytes of every datagram.
pub const MAGIC: [u8; 2] = *b"TL";
/// Largest datagram: the magic plus the largest `titan_link` message.
pub const MAX_DATAGRAM_LEN: usize = MAGIC.len() + titan_link::MAX_ENCODED_LEN;

/// Why a datagram couldn't be built or read.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WireError {
    /// The datagram doesn't start with [`MAGIC`] (or is shorter than it).
    BadMagic,
    /// The payload doesn't fit [`MAX_DATAGRAM_LEN`] or the given buffer.
    TooLong,
    /// The payload isn't a valid `titan_link` message.
    Undecodable,
}

/// Builds a datagram around an already encoded `titan_link` message.
pub fn frame<'a>(payload: &[u8], buf: &'a mut [u8]) -> Result<&'a [u8], WireError> {
    let len = MAGIC.len() + payload.len();
    if payload.len() > titan_link::MAX_ENCODED_LEN || len > buf.len() {
        return Err(WireError::TooLong);
    }
    buf[..MAGIC.len()].copy_from_slice(&MAGIC);
    buf[MAGIC.len()..len].copy_from_slice(payload);
    Ok(&buf[..len])
}

/// Returns the encoded `titan_link` message inside a datagram, without decoding it.
pub fn unframe(datagram: &[u8]) -> Result<&[u8], WireError> {
    let payload = datagram.strip_prefix(&MAGIC).ok_or(WireError::BadMagic)?;
    if payload.len() > titan_link::MAX_ENCODED_LEN {
        return Err(WireError::TooLong);
    }
    Ok(payload)
}

/// Encodes `msg` as a datagram.
pub fn encode<'a, T: Serialize>(msg: &T, buf: &'a mut [u8]) -> Result<&'a [u8], WireError> {
    if buf.len() < MAGIC.len() {
        return Err(WireError::TooLong);
    }
    let (magic, rest) = buf.split_at_mut(MAGIC.len());
    magic.copy_from_slice(&MAGIC);
    let room = rest.len().min(titan_link::MAX_ENCODED_LEN);
    let len = titan_link::encode(msg, &mut rest[..room]).map_err(|_| WireError::TooLong)?.len();
    Ok(&buf[..MAGIC.len() + len])
}

/// Decodes the message in a datagram.
pub fn decode<'a, T: Deserialize<'a>>(datagram: &'a [u8]) -> Result<T, WireError> {
    titan_link::decode(unframe(datagram)?).map_err(|_| WireError::Undecodable)
}

/// True if `payload` is a `HostToDevice::KillSwitch`, i.e. the bridge forwarding `state/kill`.
///
/// On Puddles the backplane *publishes* `state/kill`, so the only thing the bridge's generic `state/kill`
/// subscription ever sees is the board's own kill state coming back. Fed into the firmware it would land in kill
/// switch 0, which on Puddles is the physical switch (`KILL_SWITCH_PHYSICAL`), and a killed vehicle would keep
/// itself killed. The C backplane had no such subscription, so the board drops these.
pub fn is_ros_kill_switch(payload: &[u8]) -> bool {
    matches!(titan_link::decode(payload), Ok(titan_link::HostToDevice::KillSwitch { .. }))
}

/// The bridge's `--interface` value for the UDP transport: `[BIND][,BOARD]`.
///
/// * `BIND` is where the bridge listens: `IP`, `IP:PORT` or `:PORT`. Defaults to `0.0.0.0:2203`.
/// * `BOARD` is where the backplane is: `IP` or `IP:PORT`. Defaults to `192.168.1.42:2203`.
///
/// For example `192.168.1.22`, `0.0.0.0:2203,192.168.1.42` or `127.0.0.1:2300,127.0.0.1:2301`. An empty spec
/// means both defaults.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct LinkSpec {
    pub bind: SocketAddrV4,
    pub board: SocketAddrV4,
}

/// The `--interface` value didn't parse.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct SpecError;

impl LinkSpec {
    pub const DEFAULT: LinkSpec = LinkSpec {
        bind: SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, PORT),
        board: SocketAddrV4::new(Ipv4Addr::new(BOARD_IP[0], BOARD_IP[1], BOARD_IP[2], BOARD_IP[3]), PORT),
    };

    pub fn parse(spec: &str) -> Result<Self, SpecError> {
        let (bind, board) = spec.split_once(',').unwrap_or((spec, ""));
        Ok(LinkSpec {
            bind: parse_endpoint(bind.trim(), Self::DEFAULT.bind)?,
            board: parse_endpoint(board.trim(), Self::DEFAULT.board)?,
        })
    }
}

/// `IP`, `IP:PORT`, `:PORT` or empty, filling in what's missing from `default`.
fn parse_endpoint(text: &str, default: SocketAddrV4) -> Result<SocketAddrV4, SpecError> {
    if text.is_empty() {
        return Ok(default);
    }
    if let Some(port) = text.strip_prefix(':') {
        return Ok(SocketAddrV4::new(*default.ip(), port.parse().map_err(|_| SpecError)?));
    }
    if let Ok(addr) = text.parse::<SocketAddrV4>() {
        return Ok(addr);
    }
    let ip: Ipv4Addr = text.parse().map_err(|_| SpecError)?;
    Ok(SocketAddrV4::new(ip, default.port()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use titan_link::{DeviceToHost, HostToDevice, MAX_ENCODED_LEN, puddles_backplane};

    #[test]
    fn datagram_is_magic_then_postcard() {
        let msg = HostToDevice::Ping { protocol_version: 7 };
        let mut buf = [0u8; MAX_DATAGRAM_LEN];
        let datagram = encode(&msg, &mut buf).unwrap();
        // Variant 0 (Ping), then the protocol version
        assert_eq!(datagram, b"TL\x00\x07");
        assert_eq!(decode::<HostToDevice>(datagram).unwrap(), msg);

        let mut payload = [0u8; MAX_ENCODED_LEN];
        let encoded = titan_link::encode(&msg, &mut payload).unwrap();
        let mut framed = [0u8; MAX_DATAGRAM_LEN];
        assert_eq!(frame(encoded, &mut framed).unwrap(), datagram);
        assert_eq!(unframe(datagram).unwrap(), encoded);
    }

    #[test]
    fn board_messages_round_trip() {
        let msg =
            DeviceToHost::PuddlesBackplane(puddles_backplane::DeviceToHost::Rpm(puddles_backplane::RpmFeedback {
                valid_mask: 0b1010_0101,
                rpm: [1, -2, 3, -4, 5, -6, i16::MAX, 0],
            }));
        let mut buf = [0u8; MAX_DATAGRAM_LEN];
        let datagram = encode(&msg, &mut buf).unwrap();
        assert_eq!(decode::<DeviceToHost>(datagram).unwrap(), msg);
    }

    #[test]
    fn rejects_foreign_and_malformed_datagrams() {
        assert_eq!(unframe(b""), Err(WireError::BadMagic));
        assert_eq!(unframe(b"T"), Err(WireError::BadMagic));
        // A DDS-XRCE message from a micro-ROS client
        assert_eq!(unframe(&[0x81, 0x00, 0x00, 0x00, 0x0B, 0x01]), Err(WireError::BadMagic));
        // Magic with nothing after it, an unknown variant, and trailing bytes
        assert_eq!(decode::<HostToDevice>(b"TL"), Err(WireError::Undecodable));
        assert_eq!(decode::<HostToDevice>(b"TL\x7F"), Err(WireError::Undecodable));
        assert_eq!(decode::<HostToDevice>(b"TL\x00\x01\x00"), Err(WireError::Undecodable));

        let mut oversized = [0u8; MAX_DATAGRAM_LEN + 1];
        oversized[..2].copy_from_slice(&MAGIC);
        assert_eq!(unframe(&oversized), Err(WireError::TooLong));
        assert_eq!(frame(&oversized[2..], &mut [0u8; 2 * MAX_DATAGRAM_LEN]), Err(WireError::TooLong));
        assert_eq!(frame(b"abc", &mut [0u8; 4]), Err(WireError::TooLong));
        assert_eq!(encode(&HostToDevice::Ping { protocol_version: 1 }, &mut [0u8; 3]), Err(WireError::TooLong));
    }

    #[test]
    fn kill_switch_echo_is_recognised() {
        let mut buf = [0u8; MAX_ENCODED_LEN];
        let kill = titan_link::encode(&HostToDevice::KillSwitch { asserting_kill: false }, &mut buf).unwrap();
        assert!(is_ros_kill_switch(kill));
        let ping = titan_link::encode(&HostToDevice::Ping { protocol_version: 1 }, &mut buf).unwrap();
        assert!(!is_ros_kill_switch(ping));
        let dshot = HostToDevice::PuddlesBackplane(puddles_backplane::HostToDevice::Dshot([0; 8]));
        assert!(!is_ros_kill_switch(titan_link::encode(&dshot, &mut buf).unwrap()));
        assert!(!is_ros_kill_switch(b"garbage"));
    }

    #[test]
    fn interface_spec() {
        let board = Ipv4Addr::new(192, 168, 1, 42);
        assert_eq!(LinkSpec::parse("").unwrap(), LinkSpec::DEFAULT);
        assert_eq!(LinkSpec::DEFAULT.board, SocketAddrV4::new(board, 2203));
        assert_eq!(LinkSpec::DEFAULT.bind, SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, 2203));

        let spec = LinkSpec::parse("192.168.1.22").unwrap();
        assert_eq!(spec.bind, SocketAddrV4::new(Ipv4Addr::new(192, 168, 1, 22), 2203));
        assert_eq!(spec.board, LinkSpec::DEFAULT.board);

        let spec = LinkSpec::parse(":3000, 10.0.0.5").unwrap();
        assert_eq!(spec.bind, SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, 3000));
        assert_eq!(spec.board, SocketAddrV4::new(Ipv4Addr::new(10, 0, 0, 5), 2203));

        let spec = LinkSpec::parse("127.0.0.1:2300,127.0.0.1:2301").unwrap();
        assert_eq!(spec.bind, SocketAddrV4::new(Ipv4Addr::LOCALHOST, 2300));
        assert_eq!(spec.board, SocketAddrV4::new(Ipv4Addr::LOCALHOST, 2301));

        let spec = LinkSpec::parse(",192.168.1.50:9").unwrap();
        assert_eq!(spec.bind, LinkSpec::DEFAULT.bind);
        assert_eq!(spec.board, SocketAddrV4::new(Ipv4Addr::new(192, 168, 1, 50), 9));

        for bad in ["can0", "1.2.3", ":99999", "1.2.3.4:x", "::1", "a,b"] {
            assert_eq!(LinkSpec::parse(bad), Err(SpecError), "{bad}");
        }
    }
}
