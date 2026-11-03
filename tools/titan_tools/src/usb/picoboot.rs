//! The RP2040 bootrom's PICOBOOT USB protocol, ported from `pico_usb/picoboot_connection` (picotool).

use std::time::Duration;

use anyhow::{Context, Result, anyhow, bail};
use rusb::{DeviceHandle, Direction, Recipient, RequestType, UsbContext};

pub const PICOBOOT_MAGIC: u32 = 0x431f_d10b;

const IF_RESET: u8 = 0x41;
const IF_CMD_STATUS: u8 = 0x42;

const PC_EXCLUSIVE_ACCESS: u8 = 0x01;
const PC_REBOOT: u8 = 0x02;
const PC_FLASH_ERASE: u8 = 0x03;
const PC_READ: u8 = 0x84;
const PC_WRITE: u8 = 0x05;
const PC_EXIT_XIP: u8 = 0x06;
const PC_ENTER_CMD_XIP: u8 = 0x07;
const PC_EXEC: u8 = 0x08;

pub const NOT_EXCLUSIVE: u8 = 0;
pub const EXCLUSIVE: u8 = 1;

pub const ROM_START: u32 = 0x0000_0000;
pub const FLASH_START: u32 = 0x1000_0000;
pub const FLASH_END: u32 = 0x1100_0000;
pub const SRAM_START: u32 = 0x2000_0000;
pub const SRAM_END: u32 = 0x2004_2000;

const CMD_TIMEOUT: Duration = Duration::from_millis(3000);
const DATA_TIMEOUT: Duration = Duration::from_millis(10000);
const CONTROL_TIMEOUT: Duration = Duration::from_millis(1000);

/// Status codes reported by the bootrom for a failed command.
pub fn status_name(code: u32) -> &'static str {
    match code {
        0 => "ok",
        1 => "unknown cmd",
        2 => "invalid cmd length",
        3 => "invalid transfer length",
        4 => "invalid address",
        5 => "bad address alignment",
        6 => "interleaved write",
        7 => "rebooting",
        _ => "<unknown>",
    }
}

/// The PICOBOOT interface of a bootrom device.
#[derive(Clone, Copy, Debug)]
pub struct PicobootInterface {
    pub interface: u8,
    pub out_ep: u8,
    pub in_ep: u8,
}

impl PicobootInterface {
    /// Finds the vendor class interface with two bulk endpoints (interface 1 if the mass storage interface is
    /// enabled, otherwise 0).
    pub fn find<T: UsbContext>(handle: &DeviceHandle<T>) -> Result<Self> {
        let config = handle.device().active_config_descriptor()?;
        let index = if config.num_interfaces() == 1 { 0 } else { 1 };
        let interface = config.interfaces().nth(index).context("PICOBOOT interface missing")?;
        let alt = interface.descriptors().next().context("PICOBOOT interface has no descriptor")?;
        let endpoints: Vec<u8> = alt.endpoint_descriptors().map(|e| e.address()).collect();
        if alt.class_code() != 0xFF || endpoints.len() != 2 || endpoints[0] & 0x80 != 0 || endpoints[1] & 0x80 == 0 {
            bail!("Could not locate PICOBOOT interface");
        }
        Ok(PicobootInterface { interface: alt.interface_number(), out_ep: endpoints[0], in_ep: endpoints[1] })
    }
}

/// An open PICOBOOT connection. Takes exclusive access on open and gives it back on drop.
pub struct Picoboot<T: UsbContext> {
    handle: DeviceHandle<T>,
    itf: PicobootInterface,
    token: u32,
}

enum DataPhase<'a> {
    None,
    In(&'a mut [u8]),
    Out(&'a [u8]),
}

impl<T: UsbContext> Picoboot<T> {
    /// Claims the PICOBOOT interface, resets it in case it was left in a bad state and takes exclusive access.
    pub fn open(handle: DeviceHandle<T>) -> Result<Self> {
        let itf = PicobootInterface::find(&handle)?;
        handle.claim_interface(itf.interface).context("claiming PICOBOOT interface")?;
        let mut conn = Picoboot { handle, itf, token: 1 };
        conn.reset()?;
        conn.exclusive_access(EXCLUSIVE)?;
        Ok(conn)
    }

    pub fn handle(&self) -> &DeviceHandle<T> {
        &self.handle
    }

    fn is_halted(&self, ep: u8) -> bool {
        let mut status = [0u8; 2];
        let request_type = rusb::request_type(Direction::In, RequestType::Standard, Recipient::Endpoint);
        matches!(self.handle.read_control(request_type, 0x00, 0, ep as u16, &mut status, CONTROL_TIMEOUT), Ok(2))
            && status[0] & 1 != 0
    }

    /// Resets the PICOBOOT interface state machine.
    pub fn reset(&mut self) -> Result<()> {
        for ep in [self.itf.in_ep, self.itf.out_ep] {
            if self.is_halted(ep) {
                let _ = self.handle.clear_halt(ep);
            }
        }
        let request_type = rusb::request_type(Direction::Out, RequestType::Vendor, Recipient::Interface);
        self.handle
            .write_control(request_type, IF_RESET, 0, self.itf.interface as u16, &[], CONTROL_TIMEOUT)
            .context("PICOBOOT interface reset")?;
        Ok(())
    }

    /// Reads the status of the last command: `(token, status code, command id, in progress)`.
    pub fn cmd_status(&self) -> Result<(u32, u32, u8, bool)> {
        let mut status = [0u8; 16];
        let request_type = rusb::request_type(Direction::In, RequestType::Vendor, Recipient::Interface);
        let len = self.handle.read_control(
            request_type,
            IF_CMD_STATUS,
            0,
            self.itf.interface as u16,
            &mut status,
            CONTROL_TIMEOUT,
        )?;
        if len != status.len() {
            bail!("short PICOBOOT status response");
        }
        let word = |i: usize| u32::from_le_bytes(status[i..i + 4].try_into().unwrap());
        Ok((word(0), word(4), status[8], status[9] != 0))
    }

    fn cmd(&mut self, id: u8, args: &[u8], data: DataPhase) -> Result<()> {
        self.cmd_inner(id, args, data).map_err(|e| {
            // Ask the bootrom why it failed, like picotool's wrap_call
            match self.cmd_status() {
                Ok((_, code, _, _)) if code != 0 => {
                    anyhow!("PICOBOOT command 0x{id:02x} failed: {} ({code})", status_name(code))
                }
                _ => e.context(format!("PICOBOOT command 0x{id:02x} connection error")),
            }
        })
    }

    fn cmd_inner(&mut self, id: u8, args: &[u8], data: DataPhase) -> Result<()> {
        let transfer_len = match &data {
            DataPhase::None => 0,
            DataPhase::In(buf) => buf.len(),
            DataPhase::Out(buf) => buf.len(),
        } as u32;

        let mut cmd = [0u8; 32];
        cmd[0..4].copy_from_slice(&PICOBOOT_MAGIC.to_le_bytes());
        cmd[4..8].copy_from_slice(&self.token.to_le_bytes());
        self.token = self.token.wrapping_add(1);
        cmd[8] = id;
        cmd[9] = args.len() as u8;
        cmd[12..16].copy_from_slice(&transfer_len.to_le_bytes());
        cmd[16..16 + args.len()].copy_from_slice(args);
        let sent = self.handle.write_bulk(self.itf.out_ep, &cmd, CMD_TIMEOUT)?;
        if sent != cmd.len() {
            bail!("short PICOBOOT command write");
        }

        match data {
            DataPhase::None => {}
            DataPhase::In(buf) => {
                let mut received = 0;
                while received < buf.len() {
                    let n = self.handle.read_bulk(self.itf.in_ep, &mut buf[received..], DATA_TIMEOUT)?;
                    if n == 0 {
                        bail!("PICOBOOT data phase ended early ({received}/{} bytes)", buf.len());
                    }
                    received += n;
                }
            }
            DataPhase::Out(buf) => {
                let sent = self.handle.write_bulk(self.itf.out_ep, buf, DATA_TIMEOUT)?;
                if sent != buf.len() {
                    bail!("short PICOBOOT data write ({sent}/{} bytes)", buf.len());
                }
            }
        }

        // The acknowledgement goes in the opposite direction of the command (sizes as in picotool)
        let ack_timeout = if transfer_len == 0 { DATA_TIMEOUT } else { CMD_TIMEOUT };
        let mut spoon = [0u8; 64];
        if id & 0x80 != 0 {
            self.handle.write_bulk(self.itf.out_ep, &spoon[..1], ack_timeout)?;
        } else {
            self.handle.read_bulk(self.itf.in_ep, &mut spoon[..1], ack_timeout)?;
        }
        Ok(())
    }

    fn range(addr: u32, len: u32) -> [u8; 8] {
        let mut args = [0u8; 8];
        args[..4].copy_from_slice(&addr.to_le_bytes());
        args[4..].copy_from_slice(&len.to_le_bytes());
        args
    }

    pub fn exclusive_access(&mut self, exclusive: u8) -> Result<()> {
        self.cmd(PC_EXCLUSIVE_ACCESS, &[exclusive], DataPhase::None)
    }

    pub fn enter_cmd_xip(&mut self) -> Result<()> {
        self.cmd(PC_ENTER_CMD_XIP, &[], DataPhase::None)
    }

    pub fn exit_xip(&mut self) -> Result<()> {
        self.cmd(PC_EXIT_XIP, &[], DataPhase::None)
    }

    pub fn reboot(&mut self, pc: u32, sp: u32, delay_ms: u32) -> Result<()> {
        let mut args = [0u8; 12];
        args[0..4].copy_from_slice(&pc.to_le_bytes());
        args[4..8].copy_from_slice(&sp.to_le_bytes());
        args[8..12].copy_from_slice(&delay_ms.to_le_bytes());
        self.cmd(PC_REBOOT, &args, DataPhase::None)
    }

    pub fn exec(&mut self, addr: u32) -> Result<()> {
        self.cmd(PC_EXEC, &addr.to_le_bytes(), DataPhase::None)
    }

    pub fn flash_erase(&mut self, addr: u32, len: u32) -> Result<()> {
        self.cmd(PC_FLASH_ERASE, &Self::range(addr, len), DataPhase::None)
    }

    pub fn write(&mut self, addr: u32, data: &[u8]) -> Result<()> {
        self.cmd(PC_WRITE, &Self::range(addr, data.len() as u32), DataPhase::Out(data))
    }

    pub fn read(&mut self, addr: u32, buf: &mut [u8]) -> Result<()> {
        let len = buf.len() as u32;
        self.cmd(PC_READ, &Self::range(addr, len), DataPhase::In(buf))
    }
}

impl<T: UsbContext> Drop for Picoboot<T> {
    fn drop(&mut self) {
        if self.exclusive_access(NOT_EXCLUSIVE).is_err() {
            let _ = self.reset();
        }
        let _ = self.handle.release_interface(self.itf.interface);
    }
}

/// Stub run from the end of SRAM that reads the flash unique ID and JEDEC ID into the 12 bytes after itself
/// (`pico_usb/flash_getid_stub/flash_getid.S`, prebuilt).
pub const FLASH_GETID_BIN: [u8; 192] = [
    0xc0, 0xb5, 0x14, 0x26, 0x36, 0x88, 0x18, 0x27, 0x3f, 0x88, 0x30, 0x1c, 0x27, 0x49, 0xb8, 0x47, 0x80, 0x47, 0x30,
    0x1c, 0x26, 0x49, 0xb8, 0x47, 0x80, 0x47, 0x00, 0x20, 0x00, 0xf0, 0x2c, 0xf8, 0x4b, 0x20, 0x00, 0xf0, 0x36, 0xf8,
    0x04, 0x27, 0x00, 0x20, 0x00, 0xf0, 0x32, 0xf8, 0x01, 0x3f, 0xfa, 0xd1, 0x23, 0xa6, 0x08, 0x27, 0x00, 0x20, 0x00,
    0xf0, 0x2b, 0xf8, 0x30, 0x70, 0x01, 0x36, 0x01, 0x3f, 0xf8, 0xd1, 0x01, 0x20, 0x00, 0xf0, 0x17, 0xf8, 0x80, 0x27,
    0x01, 0x3f, 0xfd, 0xd1, 0x00, 0x20, 0x00, 0xf0, 0x11, 0xf8, 0x9f, 0x20, 0x00, 0xf0, 0x1b, 0xf8, 0x03, 0x27, 0x00,
    0x20, 0x00, 0xf0, 0x17, 0xf8, 0x30, 0x70, 0x01, 0x36, 0x01, 0x3f, 0xf8, 0xd1, 0x00, 0x20, 0x30, 0x70, 0x01, 0x20,
    0x00, 0xf0, 0x01, 0xf8, 0xc0, 0xbd, 0x00, 0x28, 0x01, 0xd0, 0x03, 0x20, 0x00, 0xe0, 0x02, 0x20, 0x00, 0x02, 0x0b,
    0x49, 0x0c, 0x4a, 0x0b, 0x68, 0x13, 0x40, 0x03, 0x43, 0x0b, 0x60, 0xf7, 0x46, 0x0a, 0x49, 0x02, 0x23, 0x8a, 0x6a,
    0x1a, 0x40, 0xfc, 0xd0, 0x08, 0x66, 0x08, 0x23, 0x8a, 0x6a, 0x1a, 0x40, 0xfc, 0xd0, 0x08, 0x6e, 0xf7, 0x46, 0x00,
    0x00, 0x49, 0x46, 0x00, 0x00, 0x45, 0x58, 0x00, 0x00, 0x0c, 0x80, 0x01, 0x40, 0xff, 0xfc, 0xff, 0xff, 0x00, 0x00,
    0x00, 0x18,
];

/// Stub that sets the watchdog scratch registers so the firmware reports a clean boot from the USB bootrom
/// (`pico_usb/flash_getid_stub/cfg_watchdog.S`, prebuilt).
pub const CFG_WATCHDOG_BIN: [u8; 24] = [
    0x02, 0x48, 0x03, 0x49, 0xc8, 0x60, 0x03, 0x48, 0x08, 0x61, 0x70, 0x47, 0x00, 0x50, 0x03, 0x01, 0x00, 0x80, 0x05,
    0x40, 0x07, 0x30, 0x19, 0x07,
];

/// Where the stubs are loaded: the end of SRAM, reserved for the boot2 copy during boot.
pub const STUB_MAX_LEN: u32 = 256;
pub const STUB_LOCATION: u32 = SRAM_END - STUB_MAX_LEN;

const _: () = assert!(FLASH_GETID_BIN.len() + 12 <= STUB_MAX_LEN as usize);
const _: () = assert!(CFG_WATCHDOG_BIN.len() <= STUB_MAX_LEN as usize);
