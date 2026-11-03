//! Camera feed TCP server, ported from `canmore_cli/src/camera_socket_listener.cpp`.
//!
//! Receives JPEGs from a device's CANmore camera feed and forwards each one to every connected TCP client, framed
//! as `A5 5A 0F F0 <u32 big endian length> <jpeg>`. Clients send `<cmd> <param len> <params>` control packets:
//! 1 = enable stream, 2 = quality, 3 = keypress, 4 = stream select, 5 = max dimension (u16 big endian). The CAN
//! stream is enabled while at least one client is connected. Runs until Enter or Ctrl+C.

use std::io::{ErrorKind, Read, Write};
use std::net::{Ipv6Addr, SocketAddr, TcpListener, TcpStream};
use std::time::Duration;

use anyhow::{Context, Result};
use titan_tools::canmore::camera::ImageReceiver;
use titan_tools::ui::{COLOR_NAME, COLOR_RESET};

const CTRL_STREAM_ENABLE: u8 = 1;
const CTRL_QUALITY: u8 = 2;
const CTRL_KEYPRESS: u8 = 3;
const CTRL_STREAM_SELECT: u8 = 4;
const CTRL_MAX_DIMENSION: u8 = 5;

pub const DEFAULT_PORT: u16 = 3005;

struct Client {
    stream: TcpStream,
    addr: SocketAddr,
    pending: Vec<u8>,
}

impl Client {
    /// Reads control packets. Returns false once the connection closed.
    fn service(&mut self, rx: &ImageReceiver) -> bool {
        let mut buf = [0u8; 64];
        loop {
            match self.stream.read(&mut buf) {
                Ok(0) => return false,
                Ok(n) => self.pending.extend_from_slice(&buf[..n]),
                Err(e) if e.kind() == ErrorKind::WouldBlock => break,
                Err(_) => return false,
            }
        }
        while self.pending.len() >= 2 && self.pending.len() >= 2 + self.pending[1] as usize {
            let len = 2 + self.pending[1] as usize;
            let packet: Vec<u8> = self.pending.drain(..len).collect();
            let params = &packet[2..];
            let result = match (packet[0], params) {
                (CTRL_STREAM_ENABLE, []) => rx.set_stream_enabled(true),
                (CTRL_KEYPRESS, [key, ..]) => rx.send_keypress(*key),
                (CTRL_QUALITY, [quality]) => rx.set_quality(*quality),
                (CTRL_STREAM_SELECT, [stream]) => rx.set_stream_id(*stream),
                (CTRL_MAX_DIMENSION, [hi, lo]) => rx.set_max_dimension(u16::from_be_bytes([*hi, *lo])),
                _ => Ok(()),
            };
            if let Err(e) = result {
                println!("({}) Failed to forward control packet: {e:#}", self.addr);
            }
        }
        true
    }

    fn send_jpeg(&mut self, jpeg: &[u8]) -> bool {
        let mut header = [0xA5, 0x5A, 0x0F, 0xF0, 0, 0, 0, 0];
        header[4..].copy_from_slice(&(jpeg.len() as u32).to_be_bytes());
        // Blocking writes, so a whole image goes out
        let _ = self.stream.set_nonblocking(false);
        let ok = self.stream.write_all(&header).and_then(|_| self.stream.write_all(jpeg)).is_ok();
        let _ = self.stream.set_nonblocking(true);
        if !ok {
            println!("({}) Failed to write - Closing connection...", self.addr);
        }
        ok
    }
}

/// Serves the camera feed of `client_id` on `interface` to TCP clients on `port` until Enter is pressed.
pub fn run(interface: &str, client_id: u8, port: u16) -> Result<()> {
    let mut rx = ImageReceiver::open(interface, client_id)?;
    let listener = TcpListener::bind((Ipv6Addr::UNSPECIFIED, port)).context("binding camera server port")?;
    listener.set_nonblocking(true)?;
    println!("{COLOR_NAME}Listening for connections on port {port} (Press Enter or Ctrl+C to exit){COLOR_RESET}");

    let mut clients: Vec<Client> = Vec::new();
    while !crate::gdbserver::stdin_interrupt() {
        loop {
            match listener.accept() {
                Ok((stream, addr)) => {
                    stream.set_nonblocking(true)?;
                    println!("({addr}) Accepted connection");
                    if clients.is_empty() {
                        rx.set_stream_enabled(true)?;
                    }
                    clients.push(Client { stream, addr, pending: Vec::new() });
                }
                Err(e) if e.kind() == ErrorKind::WouldBlock => break,
                Err(e) => return Err(e).context("accepting camera client"),
            }
        }

        let before = clients.len();
        clients.retain_mut(|c| {
            let alive = c.service(&rx);
            if !alive {
                println!("({}) Connection closed", c.addr);
            }
            alive
        });

        if let Some(jpeg) = rx.poll(Duration::from_millis(20))? {
            clients.retain_mut(|c| c.send_jpeg(&jpeg));
        }

        if before > 0 && clients.is_empty() {
            // Nobody is watching: stop the stream to give the bus its bandwidth back
            rx.set_stream_enabled(false)?;
        }
    }
    println!("{COLOR_NAME}Server Stopped{COLOR_RESET}");
    Ok(())
}
