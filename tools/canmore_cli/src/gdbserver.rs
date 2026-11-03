//! GDB server over the CANmore GDB stub, ported from `canmore_cli/src/gdbserver.cpp`.
//!
//! Listens on localhost, serving one GDB connection at a time, until Enter or Ctrl+C is pressed. The protocol
//! handling lives in [`titan_tools::canmore::gdb`].

use std::io::{ErrorKind, Read, Write};
use std::net::{Ipv4Addr, TcpListener, TcpStream};
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use titan_tools::canmore::gdb::{GdbHandler, GdbPacketParser, GdbTarget};
use titan_tools::ui::{COLOR_ERROR, COLOR_HEADER, COLOR_NAME, COLOR_NOTICE, COLOR_RESET, decode_u32};

use crate::cli::usage;

pub const DEFAULT_PORT: u16 = 3333;

pub fn parse_port(args: &[String]) -> Result<u16> {
    match args.first() {
        None => Ok(DEFAULT_PORT),
        Some(arg) => match decode_u32(arg, u16::MAX as u32) {
            Some(port) => Ok(port as u16),
            None => usage("Invalid port number"),
        },
    }
}

/// True if Enter (or Ctrl+C) was typed. Doesn't block.
pub fn stdin_interrupt() -> bool {
    let mut fds = [libc::pollfd { fd: libc::STDIN_FILENO, events: libc::POLLIN, revents: 0 }];
    // SAFETY: fds is a valid array of one pollfd for the duration of the call
    let ready = unsafe { libc::poll(fds.as_mut_ptr(), 1, 0) };
    if ready <= 0 || fds[0].revents & libc::POLLIN == 0 {
        return false;
    }
    let mut byte = [0u8; 1];
    // SAFETY: reading one byte into a valid one byte buffer
    let n = unsafe { libc::read(libc::STDIN_FILENO, byte.as_mut_ptr().cast(), 1) };
    n <= 0 || byte[0] == b'\n' || byte[0] == 0x03
}

/// Pings the device every second, returning false (and reporting) if it stopped answering.
struct Pinger(Instant);

impl Pinger {
    fn check(&mut self, handler: &mut GdbHandler) -> bool {
        if self.0.elapsed() < Duration::from_secs(1) {
            return true;
        }
        self.0 = Instant::now();
        match handler.ping() {
            Ok(()) => true,
            Err(e) => {
                println!("{COLOR_ERROR}Failed to ping device: {e:#}{COLOR_RESET}");
                false
            }
        }
    }
}

/// Serves GDB on `localhost:port` until interrupted.
pub fn run(port: u16, target: &mut dyn GdbTarget) {
    if let Err(e) = serve(port, target) {
        println!("{COLOR_ERROR}Exception caught while running GDB Server:{COLOR_RESET}");
        println!("{COLOR_ERROR}  what(): {e:#}{COLOR_RESET}");
    }
}

fn serve(port: u16, target: &mut dyn GdbTarget) -> Result<()> {
    let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, port)).context("TCP Socket Bind")?;
    listener.set_nonblocking(true)?;
    println!("{COLOR_NAME}Listening for connections on localhost:{port} (Press Enter or Ctrl+C to exit){COLOR_RESET}");
    let mut handler = GdbHandler::new(target);
    let mut pinger = Pinger(Instant::now());
    loop {
        let (stream, addr) = loop {
            if stdin_interrupt() || !pinger.check(&mut handler) {
                println!("{COLOR_NOTICE}Break{COLOR_RESET}");
                return Ok(());
            }
            match listener.accept() {
                Ok(conn) => break conn,
                Err(e) if e.kind() == ErrorKind::WouldBlock => std::thread::sleep(Duration::from_millis(50)),
                Err(e) => return Err(e).context("TCP Socket Accept"),
            }
        };
        println!("{COLOR_HEADER}Accepted new connection from {addr}{COLOR_RESET}");
        match serve_connection(stream, &mut handler, &mut pinger) {
            Ok(true) => println!("{COLOR_HEADER}Connection closed{COLOR_RESET}"),
            Ok(false) => {
                println!("{COLOR_NOTICE}Break{COLOR_RESET}");
                return Ok(());
            }
            // Connection errors only end this connection
            Err(e) => println!("{COLOR_ERROR}Connection Error: {e:#}{COLOR_RESET}"),
        }
    }
}

/// Serves one connection. Returns false if the user asked to stop the server.
fn serve_connection(mut stream: TcpStream, handler: &mut GdbHandler, pinger: &mut Pinger) -> Result<bool> {
    stream.set_read_timeout(Some(Duration::from_millis(50)))?;
    handler.reset_state();
    let mut parser = GdbPacketParser::default();
    let mut buf = [0u8; 1024];
    loop {
        if stdin_interrupt() || !pinger.check(handler) {
            let _ = stream.shutdown(std::net::Shutdown::Both);
            return Ok(false);
        }
        let n = match stream.read(&mut buf) {
            Ok(0) => return Ok(true),
            Ok(n) => n,
            Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => continue,
            Err(e) => return Err(e).context("Error During Read"),
        };
        let response = parser.process(handler, &buf[..n]);
        for error in handler.errors.drain(..) {
            println!("{COLOR_ERROR}{error}{COLOR_RESET}");
        }
        stream.write_all(&response).context("Error During Write")?;
        if handler.detach_received {
            let _ = stream.shutdown(std::net::Shutdown::Both);
            return Ok(true);
        }
    }
}
