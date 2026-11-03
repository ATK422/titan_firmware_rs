//! SocketCAN access for CANmore, ported from `canmore_cpp/CANSocket.cpp` and `RegMappedCANClient.cpp`.

use std::io;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use canmore::id::{self, Direction};
use canmore::reg_mapped;
use socketcan::id::FdFlags;
use socketcan::{
    CanAnyFrame, CanDataFrame, CanFdFrame, CanFdSocket, CanFilter, CanFrame, CanSocket, EmbeddedFrame, ExtendedId,
    Frame, Id, Socket, SocketOptions, StandardId,
};

pub const CAN_EFF_FLAG: u32 = 0x8000_0000;
pub const CAN_RTR_FLAG: u32 = 0x4000_0000;
pub const CAN_SFF_MASK: u32 = 0x0000_07FF;
pub const CAN_EFF_MASK: u32 = 0x1FFF_FFFF;

/// Timeout for one register mapped request (the W25Q16JV takes up to 400 ms to erase a sector).
pub const REG_MAPPED_TIMEOUT: Duration = Duration::from_millis(1000);
/// Bulk requests in flight on CAN, matching the 32 frame FIFO the firmware configures in the MCP251XFD.
pub const REG_MAPPED_MAX_IN_FLIGHT: usize = 32;

/// The CAN interfaces present on this machine (`canN`, plus `vcanN` for testing).
pub fn list_interfaces() -> Vec<String> {
    let mut names: Vec<String> = std::fs::read_dir("/sys/class/net")
        .map(|dir| {
            dir.filter_map(|e| e.ok()?.file_name().into_string().ok())
                .filter(|name| is_can_interface_name(name))
                .collect()
        })
        .unwrap_or_default();
    names.sort();
    names
}

fn is_can_interface_name(name: &str) -> bool {
    let name = name.strip_prefix('v').unwrap_or(name);
    name.strip_prefix("can").is_some_and(|n| !n.is_empty() && n.chars().all(|c| c.is_ascii_digit()))
}

/// True if the interface is configured for CAN FD (its MTU is 72 instead of 16).
pub fn interface_is_fd(name: &str) -> bool {
    std::fs::read_to_string(format!("/sys/class/net/{name}/mtu"))
        .ok()
        .and_then(|mtu| mtu.trim().parse::<u32>().ok())
        .is_some_and(|mtu| mtu >= 72)
}

/// A received frame.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RawFrame {
    pub id: u32,
    pub extended: bool,
    pub data: Vec<u8>,
}

enum Sock {
    Classic(CanSocket),
    Fd(CanFdSocket),
}

/// A raw CAN socket bound to one interface. Opens in CAN FD mode when the interface supports it.
pub struct CanBus {
    sock: Sock,
    name: String,
}

impl CanBus {
    pub fn open(interface: &str) -> Result<Self> {
        let sock = if interface_is_fd(interface) {
            Sock::Fd(CanFdSocket::open(interface).with_context(|| format!("opening CAN FD interface {interface}"))?)
        } else {
            Sock::Classic(CanSocket::open(interface).with_context(|| format!("opening CAN interface {interface}"))?)
        };
        Ok(CanBus { sock, name: interface.to_owned() })
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn is_fd(&self) -> bool {
        matches!(self.sock, Sock::Fd(_))
    }

    /// Only receive frames matching one of the `(id, mask)` filters (raw SocketCAN filter values).
    pub fn set_filters(&self, filters: &[(u32, u32)]) -> io::Result<()> {
        let filters: Vec<CanFilter> = filters.iter().map(|&(id, mask)| CanFilter::new(id, mask)).collect();
        match &self.sock {
            Sock::Classic(s) => s.set_filters(&filters),
            Sock::Fd(s) => s.set_filters(&filters),
        }
    }

    /// Waits up to `timeout` for a data frame. Returns `None` on timeout.
    pub fn read(&self, timeout: Duration) -> io::Result<Option<RawFrame>> {
        // A zero timeout would mean "block forever" to SO_RCVTIMEO
        let timeout = timeout.max(Duration::from_micros(1));
        let result = match &self.sock {
            Sock::Classic(s) => s.set_read_timeout(timeout).and_then(|_| s.read_frame()).map(|f| match f {
                CanFrame::Data(f) => {
                    Some(RawFrame { id: f.raw_id(), extended: f.is_extended(), data: f.data().into() })
                }
                _ => None,
            }),
            Sock::Fd(s) => s.set_read_timeout(timeout).and_then(|_| s.read_frame()).map(|f| match f {
                CanAnyFrame::Normal(f) => {
                    Some(RawFrame { id: f.raw_id(), extended: f.is_extended(), data: f.data().into() })
                }
                CanAnyFrame::Fd(f) => {
                    Some(RawFrame { id: f.raw_id(), extended: f.is_extended(), data: f.data().into() })
                }
                _ => None,
            }),
        };
        match result {
            Ok(frame) => Ok(frame),
            Err(e) if matches!(e.kind(), io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut) => Ok(None),
            Err(e) => Err(e),
        }
    }

    /// Discards every queued frame.
    pub fn drain(&self) -> io::Result<()> {
        while self.read(Duration::from_micros(1))?.is_some() {}
        Ok(())
    }

    /// Sends a frame: a classic frame if the data fits in 8 bytes, otherwise a CAN FD frame with bit rate switching.
    pub fn write(&self, id: u32, extended: bool, data: &[u8]) -> io::Result<()> {
        let invalid = |msg: &str| io::Error::new(io::ErrorKind::InvalidInput, msg.to_owned());
        let id = if extended {
            Id::Extended(ExtendedId::new(id).ok_or_else(|| invalid("invalid extended id"))?)
        } else {
            Id::Standard(StandardId::new(id as u16).ok_or_else(|| invalid("invalid standard id"))?)
        };
        if data.len() <= canmore::MAX_FRAME_SIZE {
            let frame = CanDataFrame::new(id, data).ok_or_else(|| invalid("invalid CAN frame"))?;
            match &self.sock {
                Sock::Classic(s) => s.write_frame(&frame),
                Sock::Fd(s) => s.write_frame(&frame),
            }
        } else {
            let Sock::Fd(s) = &self.sock else {
                return Err(invalid("frame longer than 8 bytes on a classic CAN interface"));
            };
            let frame =
                CanFdFrame::with_flags(id, data, FdFlags::BRS).ok_or_else(|| invalid("invalid CAN FD frame"))?;
            s.write_frame(&frame)
        }
    }
}

impl std::os::fd::AsRawFd for CanBus {
    fn as_raw_fd(&self) -> std::os::fd::RawFd {
        match &self.sock {
            Sock::Classic(s) => s.as_raw_fd(),
            Sock::Fd(s) => s.as_raw_fd(),
        }
    }
}

/// Register mapped protocol over a CANmore utility channel to one client, ported from `RegMappedCANClient`.
///
/// Requests are classic CAN frames even on a CAN FD bus. The C bootloader is built without multiword support, and
/// a CAN FD receiver can't tell padding from data in requests whose length isn't a valid FD length, so the tools use
/// bulk transfers on CAN (as the C++ tools do on classic CAN).
pub struct CanTransport {
    bus: CanBus,
    client_id: u8,
    channel: u8,
    timeout: Duration,
}

impl CanTransport {
    pub fn open(interface: &str, client_id: u8, channel: u8) -> Result<Self> {
        let bus = CanBus::open(interface)?;
        let rx_id = id::util_id(client_id, Direction::ClientToAgent, channel);
        bus.set_filters(&[(rx_id, CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK)])?;
        Ok(CanTransport { bus, client_id, channel, timeout: REG_MAPPED_TIMEOUT })
    }

    pub fn transfer_mode() -> reg_mapped::TransferMode {
        reg_mapped::TransferMode::Bulk { max_in_flight: REG_MAPPED_MAX_IN_FLIGHT }
    }
}

impl reg_mapped::Transport for CanTransport {
    type Error = io::Error;

    fn mtu(&self) -> usize {
        canmore::MAX_FRAME_SIZE
    }

    fn clear_rx(&mut self) -> io::Result<()> {
        self.bus.drain()
    }

    fn send(&mut self, data: &[u8]) -> io::Result<()> {
        self.bus.write(id::util_id(self.client_id, Direction::AgentToClient, self.channel), false, data)
    }

    fn recv(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let deadline = Instant::now() + self.timeout;
        let expected_id = id::util_id(self.client_id, Direction::ClientToAgent, self.channel);
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Err(io::Error::new(io::ErrorKind::TimedOut, "register mapped response timed out"));
            }
            if let Some(frame) = self.bus.read(remaining)?
                && !frame.extended
                && frame.id == expected_id
            {
                let len = frame.data.len().min(buf.len());
                buf[..len].copy_from_slice(&frame.data[..len]);
                return Ok(len);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn interface_names() {
        assert!(super::is_can_interface_name("can0"));
        assert!(super::is_can_interface_name("vcan12"));
        assert!(!super::is_can_interface_name("can"));
        assert!(!super::is_can_interface_name("canary"));
        assert!(!super::is_can_interface_name("eth0"));
    }
}
