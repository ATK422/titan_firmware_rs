//! CANmore host side: transports, discovery and control interface clients.

pub mod bootloader;
pub mod camera;
pub mod can;
pub mod debug;
pub mod discovery;
pub mod file_transfer;
pub mod gdb;
pub mod linux;
pub mod remote_tty;
pub mod udp;

use std::fmt;
use std::io;
use std::net::Ipv4Addr;

use anyhow::{Result, anyhow};
use canmore::ControlInterfaceMode;
use canmore::reg_mapped::{self, ClientError};

pub use bootloader::BootloaderClient;
pub use debug::DebugClient;
pub use discovery::{CanmoreDevice, DeviceMode, Discovery};
pub use linux::LinuxClient;

/// Where a CANmore device is reachable.
#[derive(Clone, PartialEq, Eq, Hash, Debug)]
pub enum Link {
    /// A SocketCAN interface, by name.
    Can(String),
    /// A device on the network, by IP address.
    Udp(Ipv4Addr),
}

impl fmt::Display for Link {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Link::Can(name) => f.write_str(name),
            Link::Udp(_) => f.write_str("UDP"),
        }
    }
}

/// The register mapped transport for a [`Link`].
pub enum LinkTransport {
    Can(can::CanTransport),
    Udp(udp::UdpTransport),
}

impl LinkTransport {
    /// Opens the control interface channel of `client_id` on `link` (the client ID is ignored for UDP).
    pub fn open(link: &Link, client_id: u8) -> Result<Self> {
        Ok(match link {
            Link::Can(interface) => {
                LinkTransport::Can(can::CanTransport::open(interface, client_id, canmore::CHAN_CONTROL_INTERFACE)?)
            }
            Link::Udp(addr) => LinkTransport::Udp(udp::UdpTransport::open(*addr, udp::CONTROL_INTERFACE_PORT)?),
        })
    }

    fn transfer_mode(&self) -> reg_mapped::TransferMode {
        match self {
            LinkTransport::Can(_) => can::CanTransport::transfer_mode(),
            LinkTransport::Udp(_) => udp::UdpTransport::transfer_mode(),
        }
    }
}

impl reg_mapped::Transport for LinkTransport {
    type Error = io::Error;

    fn mtu(&self) -> usize {
        match self {
            LinkTransport::Can(t) => t.mtu(),
            LinkTransport::Udp(t) => t.mtu(),
        }
    }
    fn clear_rx(&mut self) -> io::Result<()> {
        match self {
            LinkTransport::Can(t) => t.clear_rx(),
            LinkTransport::Udp(t) => t.clear_rx(),
        }
    }
    fn send(&mut self, data: &[u8]) -> io::Result<()> {
        match self {
            LinkTransport::Can(t) => t.send(data),
            LinkTransport::Udp(t) => t.send(data),
        }
    }
    fn recv(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self {
            LinkTransport::Can(t) => t.recv(buf),
            LinkTransport::Udp(t) => t.recv(buf),
        }
    }
}

/// Name of a control interface mode, as the C++ tools print it.
pub fn mode_name(mode: u8) -> String {
    match mode {
        m if m == ControlInterfaceMode::Normal as u8 => "Normal".into(),
        m if m == ControlInterfaceMode::BootDelay as u8 => "Boot Delay".into(),
        m if m == ControlInterfaceMode::Bootloader as u8 => "Bootloader".into(),
        m if m == ControlInterfaceMode::Linux as u8 => "Linux".into(),
        m => format!("Unknown Mode {m}"),
    }
}

/// A register mapped client over a [`Link`], turning protocol errors into descriptive `anyhow` errors like the C++
/// `RegMappedClientError`.
pub struct RegClient {
    inner: reg_mapped::Client<LinkTransport>,
}

impl RegClient {
    pub fn open(link: &Link, client_id: u8, mode: ControlInterfaceMode) -> Result<Self> {
        Ok(Self::new(LinkTransport::open(link, client_id)?, mode as u8))
    }

    pub fn new(transport: LinkTransport, mode: u8) -> Self {
        let transfer_mode = transport.transfer_mode();
        RegClient { inner: reg_mapped::Client::new(transport, mode, transfer_mode) }
    }

    pub fn mode(&self) -> u8 {
        self.inner.mode()
    }

    fn err(&self, e: ClientError<io::Error>, what: fmt::Arguments) -> anyhow::Error {
        anyhow!("Register Mapped Access Error ({what} - mode: {}): {e}", mode_name(self.inner.mode()))
    }

    pub fn read(&mut self, page: u8, offset: u8) -> Result<u32> {
        self.inner
            .read_register(page, offset)
            .map_err(|e| self.err(e, format_args!("Register Direct Read - page: {page}, offset: {offset}")))
    }

    pub fn write(&mut self, page: u8, offset: u8, data: u32) -> Result<()> {
        self.inner
            .write_register(page, offset, data)
            .map_err(|e| self.err(e, format_args!("Register Direct Write - page: {page}, offset: {offset}")))
    }

    pub fn read_array(&mut self, page: u8, offset: u8, out: &mut [u32]) -> Result<()> {
        let len = out.len();
        self.inner
            .read_array(page, offset, out)
            .map_err(|e| self.err(e, format_args!("Array Read - page: {page}, offset: {offset}, length: {len}")))
    }

    pub fn write_array(&mut self, page: u8, offset: u8, data: &[u32]) -> Result<()> {
        self.inner.write_array(page, offset, data).map_err(|e| {
            self.err(e, format_args!("Array Write - page: {page}, offset: {offset}, length: {}", data.len()))
        })
    }

    pub fn read_bytes(&mut self, page: u8, offset: u8, out: &mut [u8]) -> Result<()> {
        let len = out.len();
        self.inner
            .read_bytes(page, offset, out)
            .map_err(|e| self.err(e, format_args!("Array Read - page: {page}, offset: {offset}, length: {len} bytes")))
    }

    pub fn write_bytes(&mut self, page: u8, offset: u8, data: &[u8]) -> Result<()> {
        self.inner.write_bytes(page, offset, data).map_err(|e| {
            self.err(e, format_args!("Array Write - page: {page}, offset: {offset}, length: {} bytes", data.len()))
        })
    }

    pub fn read_string_page(&mut self, page: u8) -> Result<String> {
        let mut buf = [0u8; reg_mapped::PAGE_SIZE];
        let len = self
            .inner
            .read_string_page(page, &mut buf)
            .map_err(|e| self.err(e, format_args!("String Access - page: {page}")))?;
        Ok(String::from_utf8_lossy(&buf[..len]).into_owned())
    }

    pub fn write_string_page(&mut self, page: u8, s: &[u8]) -> Result<()> {
        self.inner.write_string_page(page, s).map_err(|e| self.err(e, format_args!("String Write - page: {page}")))
    }

    pub fn send_raw(&mut self, data: &[u8]) -> Result<()> {
        self.inner.send_raw(data).map_err(|e| self.err(e, format_args!("Raw Access - length: {}", data.len())))
    }
}

/// Combines the two flash ID registers into the 64-bit board serial number.
pub fn flash_id_from_words(lower: u32, upper: u32) -> u64 {
    (upper as u64) << 32 | lower as u64
}
