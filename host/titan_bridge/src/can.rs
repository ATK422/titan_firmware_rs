//! The agent side of CANmore over SocketCAN, carrying `titan_link` messages for one client.

use std::sync::Mutex;
use std::time::Duration;

use anyhow::{Context, Result};
use canmore::id::{self, Direction, Fields, FrameType};
use canmore::{Heartbeat, MsgDecoder, MsgEncoder};
use socketcan::id::FdFlags;
use socketcan::{
    CanAnyFrame, CanDataFrame, CanFdFrame, CanFdSocket, CanFilter, EmbeddedFrame, ExtendedId, Frame, Id, Socket,
    SocketOptions, StandardId,
};
use titan_link::HostToDevice;

use crate::link::{Event, Link};

const CAN_EFF_FLAG: u32 = 0x8000_0000;

pub struct CanLink {
    socket: CanFdSocket,
    client_id: u8,
    use_canfd: bool,
    decoder: Mutex<MsgDecoder>,
}

impl CanLink {
    /// Opens `interface` and filters it down to frames from `client_id`.
    pub fn open(interface: &str, client_id: u8, use_canfd: bool) -> Result<Self> {
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
            // Utility frames: heartbeats, to report whether the board is on the bus, and any board specific channels
            CanFilter::new(
                id::std_id(client_id, FrameType::Util, c2a, 0),
                id::STD_CLIENT_TYPE_DIRECTION_MASK | CAN_EFF_FLAG,
            ),
        ])?;
        socket.set_read_timeout(Duration::from_millis(250))?;
        Ok(CanLink { socket, client_id, use_canfd, decoder: Mutex::new(MsgDecoder::new(use_canfd)) })
    }

    fn write(&self, id: u32, extended: bool, data: &[u8], fd: bool) -> Result<()> {
        let id = if extended {
            Id::Extended(ExtendedId::new(id).context("invalid extended id")?)
        } else {
            Id::Standard(StandardId::new(id as u16).context("invalid standard id")?)
        };
        if fd {
            let frame = CanFdFrame::with_flags(id, data, FdFlags::BRS).context("building CAN FD frame")?;
            self.socket.write_frame(&frame)?;
        } else {
            let frame = CanDataFrame::new(id, data).context("building CAN frame")?;
            self.socket.write_frame(&frame)?;
        }
        Ok(())
    }
}

impl Link for CanLink {
    fn send(&self, msg: &HostToDevice) -> Result<()> {
        let mut buf = [0u8; titan_link::MAX_ENCODED_LEN];
        let encoded = titan_link::encode(msg, &mut buf).map_err(|_| anyhow::anyhow!("encoding {msg:?}"))?;
        let frames = MsgEncoder::new(
            self.client_id,
            Direction::AgentToClient,
            self.use_canfd,
            canmore::MSG_SUBTYPE_TITAN_LINK,
            encoded,
        );
        for frame in frames {
            self.write(frame.id, frame.extended, frame.data(), self.use_canfd)?;
        }
        Ok(())
    }

    fn send_utility(&self, channel: u8, data: &[u8]) -> Result<()> {
        anyhow::ensure!(data.len() <= canmore::MAX_FRAME_SIZE, "utility frames carry at most 8 bytes");
        // Utility frames are classic CAN frames, as in the C firmware
        self.write(id::util_id(self.client_id, Direction::AgentToClient, channel), false, data, false)
    }

    fn receive(&self) -> Result<Event> {
        let frame = match self.socket.read_frame() {
            Ok(frame) => frame,
            Err(err) if matches!(err.kind(), std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut) => {
                return Ok(Event::Idle);
            }
            Err(err) => return Err(err).context("reading CAN frame"),
        };
        let (raw_id, extended, data) = match &frame {
            CanAnyFrame::Normal(f) => (f.raw_id(), f.is_extended(), f.data()),
            CanAnyFrame::Fd(f) => (f.raw_id(), f.is_extended(), f.data()),
            CanAnyFrame::Remote(_) | CanAnyFrame::Error(_) => return Ok(Event::Idle),
        };

        let fields = Fields::decode(raw_id, extended);
        if fields.frame_type == FrameType::Util {
            return Ok(match (fields.noc, data) {
                (canmore::CHAN_HEARTBEAT, [byte]) => Event::Heartbeat(Heartbeat::from_byte(*byte)),
                (channel, data) => Event::Utility { channel, data: data.to_vec() },
            });
        }

        let mut decoder = self.decoder.lock().unwrap();
        match decoder.decode_frame(raw_id, extended, data) {
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
