//! CANmore over UDP (`canmore/ethernet_defs.h`), ported from `RegMappedEthernetClient.cpp`.
//!
//! Ethernet devices broadcast their one byte heartbeat to UDP port [`HEARTBEAT_BROADCAST_PORT`] and serve the
//! register mapped control interface on [`CONTROL_INTERFACE_PORT`], one request per datagram. The CANmore client ID
//! of an Ethernet device is the last byte of its IP address.

use std::io;
use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};
use std::time::Duration;

use anyhow::{Context, Result};
use canmore::reg_mapped;

pub const BROADCAST_IP: Ipv4Addr = Ipv4Addr::BROADCAST;
pub const HEARTBEAT_BROADCAST_PORT: u16 = 2201;
pub const CONTROL_INTERFACE_PORT: u16 = 2202;
/// Largest datagram exchanged with a device (sizes the devices' static buffers).
pub const UDP_MAX_LEN: usize = 512;
/// Bulk requests in flight on Ethernet (unused while multiword transfers are available).
pub const REG_MAPPED_MAX_IN_FLIGHT: usize = 8;

/// Register mapped protocol over UDP to one device.
pub struct UdpTransport {
    socket: UdpSocket,
}

impl UdpTransport {
    pub fn open(addr: Ipv4Addr, port: u16) -> Result<Self> {
        let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0)).context("binding UDP socket")?;
        // Connecting filters out datagrams from anyone but the device
        socket.connect(SocketAddrV4::new(addr, port)).with_context(|| format!("connecting to {addr}:{port}"))?;
        socket.set_read_timeout(Some(super::can::REG_MAPPED_TIMEOUT))?;
        Ok(UdpTransport { socket })
    }

    pub fn transfer_mode() -> reg_mapped::TransferMode {
        reg_mapped::TransferMode::Multiword
    }
}

impl reg_mapped::Transport for UdpTransport {
    type Error = io::Error;

    fn mtu(&self) -> usize {
        UDP_MAX_LEN
    }

    fn clear_rx(&mut self) -> io::Result<()> {
        self.socket.set_nonblocking(true)?;
        let mut buf = [0u8; UDP_MAX_LEN];
        let result = loop {
            match self.socket.recv(&mut buf) {
                Ok(_) => continue,
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => break Ok(()),
                // A previous send to a closed port reports ECONNREFUSED here, which isn't interesting
                Err(e) if e.kind() == io::ErrorKind::ConnectionRefused => continue,
                Err(e) => break Err(e),
            }
        };
        self.socket.set_nonblocking(false)?;
        result
    }

    fn send(&mut self, data: &[u8]) -> io::Result<()> {
        let sent = self.socket.send(data)?;
        if sent != data.len() {
            return Err(io::Error::new(io::ErrorKind::WriteZero, "short UDP send"));
        }
        Ok(())
    }

    fn recv(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self.socket.recv(buf) {
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => {
                Err(io::Error::new(io::ErrorKind::TimedOut, "register mapped response timed out"))
            }
            other => other,
        }
    }
}

/// A socket receiving the heartbeats broadcast by Ethernet devices. Several programs can listen at once.
pub fn heartbeat_socket(read_timeout: Duration) -> Result<UdpSocket> {
    use socket2::{Domain, Protocol, Socket, Type};
    let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
    socket.set_reuse_port(true)?;
    socket.set_reuse_address(true)?;
    socket
        .bind(&SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, HEARTBEAT_BROADCAST_PORT).into())
        .with_context(|| format!("binding UDP heartbeat port {HEARTBEAT_BROADCAST_PORT}"))?;
    let socket: UdpSocket = socket.into();
    socket.set_read_timeout(Some(read_timeout))?;
    Ok(socket)
}

#[cfg(test)]
mod tests {
    use super::*;
    use canmore::reg_mapped::{Client, Server, Transport};

    /// The UDP transport against a register mapped server on localhost.
    #[test]
    fn loopback_server() {
        let server_sock = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        let port = server_sock.local_addr().unwrap().port();
        let handle = std::thread::spawn(move || {
            struct Regs([u32; 64]);
            impl reg_mapped::RegisterMap for Regs {
                fn read(&mut self, page: u8, offset: u8) -> Result<u32, reg_mapped::ServerError> {
                    self.mem(page)?.read(offset)
                }
                fn write(&mut self, page: u8, offset: u8, data: u32) -> Result<(), reg_mapped::ServerError> {
                    self.mem(page)?.write(offset, data)
                }
                fn read_multiword(&mut self, p: u8, o: u8, out: &mut [u8]) -> Result<(), reg_mapped::ServerError> {
                    self.mem(p)?.read_multiword(o, out)
                }
                fn write_multiword(&mut self, p: u8, o: u8, data: &[u8]) -> Result<(), reg_mapped::ServerError> {
                    self.mem(p)?.write_multiword(o, data)
                }
            }
            impl Regs {
                fn mem(&mut self, page: u8) -> Result<reg_mapped::MemoryPage<'_>, reg_mapped::ServerError> {
                    match page {
                        0 => Ok(reg_mapped::MemoryPage::Words(reg_mapped::Access::ReadWrite(&mut self.0))),
                        _ => Err(reg_mapped::ServerError::InvalidRegisterAddress),
                    }
                }
            }
            let mut server = Server::new(1);
            let mut regs = Regs([0; 64]);
            let mut buf = [0u8; UDP_MAX_LEN];
            let mut resp = [0u8; UDP_MAX_LEN];
            // Serve until the client has done its two transfers (2 requests each, at 127 words per packet)
            for _ in 0..2 {
                let (len, from) = server_sock.recv_from(&mut buf).unwrap();
                let resp_len = server.handle_request(&mut regs, &buf[..len], &mut resp);
                server_sock.send_to(&resp[..resp_len], from).unwrap();
            }
        });

        let mut transport = UdpTransport::open(Ipv4Addr::LOCALHOST, port).unwrap();
        transport.clear_rx().unwrap();
        let mut client = Client::new(transport, 1, UdpTransport::transfer_mode());
        let data: Vec<u32> = (0..64).map(|i| i * 3).collect();
        client.write_array(0, 0, &data).unwrap();
        let mut out = [0u32; 64];
        client.read_array(0, 0, &mut out).unwrap();
        assert_eq!(out.as_slice(), data.as_slice());
        handle.join().unwrap();
    }
}
