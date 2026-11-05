//! The agent side of CANmore, carrying `titan_link` messages for one client, over a CAN frame transport: SocketCAN
//! (Linux), or CAN over UDP in cannelloni's framing ([`crate::cannelloni`]), which needs no CAN interface at all.

use std::sync::{Arc, Mutex};

use anyhow::Result;
use canmore::id::{Direction, Fields, FrameType};
use canmore::{Heartbeat, MsgDecoder, MsgEncoder};
use titan_link::HostToDevice;

use crate::cannelloni::{self, CannelloniIo};
use crate::link::{Event, Link};

/// A CAN frame as the transports carry it (remote and error frames never reach the link).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Frame {
    pub id: u32,
    pub extended: bool,
    /// A CAN FD frame (sent with bit rate switching).
    pub fd: bool,
    pub data: Vec<u8>,
}

/// Moves CAN frames to and from the bus.
pub trait FrameIo: Send + Sync {
    /// Sends frames in order (one CANmore message, or one utility frame).
    fn write(&self, frames: &[Frame]) -> Result<()>;

    /// Waits a short while for the next frame from the board; `None` if nothing came. Only frames that
    /// [`accepts`] lets through are returned (SocketCAN filters in the kernel, cannelloni here).
    fn read(&self) -> Result<Option<Frame>>;
}

/// True for the frames the bridge for `client_id` takes: that client's message frames (standard and extended) and
/// its standard utility frames, to the agent.
pub fn accepts(client_id: u8, id: u32, extended: bool) -> bool {
    let fields = Fields::decode(id, extended);
    fields.client_id == client_id
        && fields.direction == Direction::ClientToAgent
        && (fields.frame_type == FrameType::Msg || !extended)
}

/// Opens `interface` for `client_id`: `cannelloni:HOST:PORT` for CAN over UDP to a cannelloni hub (the firmware
/// simulator's `--cannelloni BUS=HOST:PORT`), anything else a SocketCAN interface (Linux only).
pub fn open(interface: &str, client_id: u8, use_canfd: bool) -> Result<Arc<dyn Link>> {
    if let Some(addr) = interface.strip_prefix(cannelloni::PREFIX) {
        let io = CannelloniIo::open(addr, client_id)?;
        return Ok(Arc::new(CanmoreLink::new(io, client_id, use_canfd)));
    }
    open_socketcan(interface, client_id, use_canfd)
}

#[cfg(target_os = "linux")]
fn open_socketcan(interface: &str, client_id: u8, use_canfd: bool) -> Result<Arc<dyn Link>> {
    let io = socketcan_io::SocketCanIo::open(interface, client_id)?;
    Ok(Arc::new(CanmoreLink::new(io, client_id, use_canfd)))
}

#[cfg(not(target_os = "linux"))]
fn open_socketcan(interface: &str, _client_id: u8, _use_canfd: bool) -> Result<Arc<dyn Link>> {
    anyhow::bail!(
        "CAN interface {interface}: SocketCAN needs Linux; use --interface {}HOST:PORT (CAN over UDP)",
        cannelloni::PREFIX
    )
}

/// CANmore for one client over any [`FrameIo`].
pub struct CanmoreLink<F> {
    io: F,
    client_id: u8,
    use_canfd: bool,
    decoder: Mutex<MsgDecoder>,
}

impl<F: FrameIo> CanmoreLink<F> {
    pub fn new(io: F, client_id: u8, use_canfd: bool) -> Self {
        CanmoreLink { io, client_id, use_canfd, decoder: Mutex::new(MsgDecoder::new(use_canfd)) }
    }
}

impl<F: FrameIo> Link for CanmoreLink<F> {
    fn send(&self, msg: &HostToDevice) -> Result<()> {
        let mut buf = [0u8; titan_link::MAX_ENCODED_LEN];
        let encoded = titan_link::encode(msg, &mut buf).map_err(|_| anyhow::anyhow!("encoding {msg:?}"))?;
        let frames: Vec<Frame> = MsgEncoder::new(
            self.client_id,
            Direction::AgentToClient,
            self.use_canfd,
            canmore::MSG_SUBTYPE_TITAN_LINK,
            encoded,
        )
        .map(|f| Frame { id: f.id, extended: f.extended, fd: self.use_canfd, data: f.data().to_vec() })
        .collect();
        self.io.write(&frames)
    }

    fn send_utility(&self, channel: u8, data: &[u8]) -> Result<()> {
        anyhow::ensure!(data.len() <= canmore::MAX_FRAME_SIZE, "utility frames carry at most 8 bytes");
        // Utility frames are classic CAN frames, as in the C firmware
        let id = canmore::id::util_id(self.client_id, Direction::AgentToClient, channel);
        self.io.write(&[Frame { id, extended: false, fd: false, data: data.to_vec() }])
    }

    fn receive(&self) -> Result<Event> {
        let Some(frame) = self.io.read()? else { return Ok(Event::Idle) };
        let fields = Fields::decode(frame.id, frame.extended);
        if fields.frame_type == FrameType::Util {
            return Ok(match (fields.noc, frame.data.as_slice()) {
                (canmore::CHAN_HEARTBEAT, [byte]) => Event::Heartbeat(Heartbeat::from_byte(*byte)),
                (channel, data) => Event::Utility { channel, data: data.to_vec() },
            });
        }

        let mut decoder = self.decoder.lock().unwrap();
        match decoder.decode_frame(frame.id, frame.extended, &frame.data) {
            Ok(Some(msg)) if msg.subtype == canmore::MSG_SUBTYPE_TITAN_LINK => match titan_link::decode(msg.data) {
                Ok(decoded) => Ok(Event::Message(decoded)),
                Err(_) => anyhow::bail!("undecodable titan_link message {:02x?}", msg.data),
            },
            Ok(Some(msg)) => anyhow::bail!("unexpected CANmore message subtype {} (micro-ROS firmware?)", msg.subtype),
            Ok(None) => Ok(Event::Idle),
            Err(err) => anyhow::bail!("CANmore decode error: {err:?}"),
        }
    }
}

#[cfg(target_os = "linux")]
mod socketcan_io {
    //! SocketCAN, filtered down to one client in the kernel.

    use std::time::Duration;

    use anyhow::{Context, Result};
    use canmore::id::{self, Direction, FrameType};
    use socketcan::id::FdFlags;
    use socketcan::{
        CanAnyFrame, CanDataFrame, CanFdFrame, CanFdSocket, CanFilter, EmbeddedFrame, ExtendedId, Frame as _, Id,
        Socket, SocketOptions, StandardId,
    };

    use super::{Frame, FrameIo};

    const CAN_EFF_FLAG: u32 = 0x8000_0000;

    pub struct SocketCanIo {
        socket: CanFdSocket,
    }

    impl SocketCanIo {
        /// Opens `interface` and filters it down to frames from `client_id` (as [`super::accepts`]).
        pub fn open(interface: &str, client_id: u8) -> Result<Self> {
            let socket = CanFdSocket::open(interface).with_context(|| format!("opening CAN interface {interface}"))?;
            let c2a = Direction::ClientToAgent;
            socket.set_filters(&[
                // Message frames: standard middle frames and extended first/last frames
                CanFilter::new(
                    id::std_id(client_id, FrameType::Msg, c2a, 0),
                    id::STD_CLIENT_TYPE_DIRECTION_MASK | CAN_EFF_FLAG,
                ),
                CanFilter::new(
                    id::ext_id(client_id, FrameType::Msg, c2a, 0, 0) | CAN_EFF_FLAG,
                    id::EXT_CLIENT_TYPE_DIRECTION_MASK | CAN_EFF_FLAG,
                ),
                // Utility frames: heartbeats, to report whether the board is on the bus, and board specific channels
                CanFilter::new(
                    id::std_id(client_id, FrameType::Util, c2a, 0),
                    id::STD_CLIENT_TYPE_DIRECTION_MASK | CAN_EFF_FLAG,
                ),
            ])?;
            socket.set_read_timeout(Duration::from_millis(250))?;
            Ok(SocketCanIo { socket })
        }

        fn write_one(&self, f: &Frame) -> Result<()> {
            let id = if f.extended {
                Id::Extended(ExtendedId::new(f.id).context("invalid extended id")?)
            } else {
                Id::Standard(StandardId::new(f.id as u16).context("invalid standard id")?)
            };
            if f.fd {
                let frame = CanFdFrame::with_flags(id, &f.data, FdFlags::BRS).context("building CAN FD frame")?;
                self.socket.write_frame(&frame)?;
            } else {
                let frame = CanDataFrame::new(id, &f.data).context("building CAN frame")?;
                self.socket.write_frame(&frame)?;
            }
            Ok(())
        }
    }

    impl FrameIo for SocketCanIo {
        fn write(&self, frames: &[Frame]) -> Result<()> {
            frames.iter().try_for_each(|f| self.write_one(f))
        }

        fn read(&self) -> Result<Option<Frame>> {
            let frame = match self.socket.read_frame() {
                Ok(frame) => frame,
                Err(err) if matches!(err.kind(), std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut) => {
                    return Ok(None);
                }
                Err(err) => return Err(err).context("reading CAN frame"),
            };
            Ok(match &frame {
                CanAnyFrame::Normal(f) => {
                    Some(Frame { id: f.raw_id(), extended: f.is_extended(), fd: false, data: f.data().to_vec() })
                }
                CanAnyFrame::Fd(f) => {
                    Some(Frame { id: f.raw_id(), extended: f.is_extended(), fd: true, data: f.data().to_vec() })
                }
                CanAnyFrame::Remote(_) | CanAnyFrame::Error(_) => None,
            })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;

    use canmore::id::{ext_id, std_id, util_id};
    use titan_link::DeviceToHost;

    /// Frames written are kept; frames to read are queued.
    #[derive(Default)]
    struct Fake {
        written: Mutex<Vec<Frame>>,
        to_read: Mutex<VecDeque<Frame>>,
    }

    impl FrameIo for Arc<Fake> {
        fn write(&self, frames: &[Frame]) -> Result<()> {
            self.written.lock().unwrap().extend_from_slice(frames);
            Ok(())
        }

        fn read(&self) -> Result<Option<Frame>> {
            Ok(self.to_read.lock().unwrap().pop_front())
        }
    }

    #[test]
    fn the_filter_takes_one_clients_frames_to_the_agent() {
        let c2a = Direction::ClientToAgent;
        let a2c = Direction::AgentToClient;
        assert!(accepts(3, std_id(3, FrameType::Msg, c2a, 2), false));
        assert!(accepts(3, ext_id(3, FrameType::Msg, c2a, 0, 0x1234), true));
        assert!(accepts(3, util_id(3, c2a, canmore::CHAN_HEARTBEAT), false));
        assert!(!accepts(3, util_id(4, c2a, canmore::CHAN_HEARTBEAT), false), "another board");
        assert!(!accepts(3, util_id(3, a2c, 0), false), "a thruster command to the board");
        assert!(!accepts(3, std_id(3, FrameType::Msg, a2c, 2), false), "another agent's message");
        assert!(!accepts(3, ext_id(3, FrameType::Util, c2a, 0, 0), true), "utility frames are standard");
    }

    /// Messages and utility frames go out as frames in the board's mode and come back as events.
    #[test]
    fn messages_round_trip_through_frames() {
        let fake = Arc::new(Fake::default());
        let link = CanmoreLink::new(fake.clone(), 2, true);
        link.send(&HostToDevice::Ping { protocol_version: titan_link::PROTOCOL_VERSION }).unwrap();
        link.send_utility(0, &[0, 1, 2, 3, 4, 5, 6, 7]).unwrap();
        let written = fake.written.lock().unwrap().clone();
        assert!(written[..written.len() - 1].iter().all(|f| f.fd), "messages in CAN FD");
        let util = written.last().unwrap();
        assert_eq!((util.id, util.extended, util.fd), (util_id(2, Direction::AgentToClient, 0), false, false));

        // The board answers with a heartbeat utility frame and a titan_link heartbeat message
        let mut buf = [0u8; titan_link::MAX_ENCODED_LEN];
        let encoded = titan_link::encode(&DeviceToHost::Heartbeat, &mut buf).unwrap();
        let mut queue = fake.to_read.lock().unwrap();
        queue.push_back(Frame {
            id: util_id(2, Direction::ClientToAgent, canmore::CHAN_HEARTBEAT),
            extended: false,
            fd: false,
            data: vec![0x01],
        });
        for f in MsgEncoder::new(2, Direction::ClientToAgent, true, canmore::MSG_SUBTYPE_TITAN_LINK, encoded) {
            queue.push_back(Frame { id: f.id, extended: f.extended, fd: true, data: f.data().to_vec() });
        }
        drop(queue);
        assert!(matches!(link.receive().unwrap(), Event::Heartbeat(_)));
        let mut got = None;
        for _ in 0..8 {
            match link.receive().unwrap() {
                Event::Message(m) => {
                    got = Some(m);
                    break;
                }
                Event::Idle => {}
                _ => panic!("unexpected event"),
            }
        }
        assert_eq!(got, Some(DeviceToHost::Heartbeat));
        assert!(matches!(link.receive().unwrap(), Event::Idle));
    }

    #[test]
    fn interfaces() {
        let err = open("cannelloni:nowhere", 2, true).err().unwrap();
        assert!(format!("{err:#}").contains("HOST:PORT"), "{err:#}");
        assert!(open("cannelloni:127.0.0.1:20000", 2, true).is_ok());
        #[cfg(not(target_os = "linux"))]
        assert!(format!("{:#}", open("can0", 2, true).err().unwrap()).contains("cannelloni:"));
    }
}
