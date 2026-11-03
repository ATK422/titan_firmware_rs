//! The gauge debug commands the C firmware registered with the CANmore remote command interface
//! (`bq40_canmore_cmds.h`, used through `canmore_cli`): argument parsing, execution through a [`Gauge`], and the
//! text output.
//!
//! The firmware runs these through [`run`] with the command line split into words. The gauge operations go to the
//! gauge task through the [`Gauge`] trait (the C version passed them to core 1 through the SIO FIFO); everything
//! else is formatted from a [`Snapshot`].

use core::fmt::{self, Write};

use crate::sbs::{self, BitName};
use crate::screens::{Snapshot, format_millivolts};
use crate::smbus::{BqError, BqErrorKind};

/// Integer width of a read (`enum read_width`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Width {
    W8,
    W16,
    W32,
}

/// Largest SMBus block the debug commands move.
pub const BLOCK_LEN: usize = 32;

/// A gauge operation (`enum dbg_cmd_type`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DebugOp {
    /// SBS word (or dword block) read.
    SbsReadInt {
        cmd: u8,
        width: Width,
    },
    SbsReadBlock {
        cmd: u8,
    },
    MfgReadBlock {
        cmd: u16,
    },
    /// Manufacturer access command without data.
    MfgCmd {
        cmd: u16,
    },
    /// Data flash write through manufacturer access.
    DfWrite {
        addr: u16,
        data: [u8; BLOCK_LEN],
        len: u8,
    },
}

/// Result of a [`DebugOp`] (`enum dbg_result_type` and its payload).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DebugResult {
    Int(u32),
    Block {
        data: [u8; BLOCK_LEN],
        len: u8,
    },
    Ok,
    Err(BqError),
    /// The gauge task didn't answer in time.
    Timeout,
}

/// Status registers read on request (`FIFO_CMD_READ_GAUGING_STATUS` / `..._CHARGING_STATUS`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StatusRegister {
    Gauging,
    Charging,
}

/// Access to the gauge task.
#[allow(async_fn_in_trait)] // Only used within the firmware, with a single executor
pub trait Gauge {
    async fn execute(&mut self, op: DebugOp) -> DebugResult;
    /// Reads a 3 byte status register; `None` if the read failed.
    async fn read_status(&mut self, register: StatusRegister) -> Option<u32>;
}

/// A remote command: name, argument synopsis and help (`debug_remote_cmd_register`).
pub struct CommandInfo {
    pub name: &'static str,
    pub args: &'static str,
    pub help: &'static str,
}

/// Every command, in the C registration order.
pub const COMMANDS: &[CommandInfo] = &[
    CommandInfo {
        name: "bq40_sbsreadint",
        args: "[cmd] [type]",
        help: "[DEBUG CMD] Sends the requested sbs command to the bq40 to read that int\n  See the bq40 reference \
               manual sections 18.2 through 18.84 for list of commands\n  [type] can be of: u8, s8, u16, s16, u32, \
               or s32",
    },
    CommandInfo {
        name: "bq40_sbsreadblock",
        args: "[cmd]",
        help: "[DEBUG CMD] Sends the requested sbs command to the bq40 to read that block\n  See the bq40 reference \
               manual sections 18.2 through 18.84 for list of commands",
    },
    CommandInfo {
        name: "bq40_mfgreadblock",
        args: "[cmd]",
        help: "[DEBUG CMD] Sends the requested manufacturing command to the bq40 to read that block\n  See the bq40 \
               reference manual section 18.1 for list of commands",
    },
    CommandInfo {
        name: "bq40_dfreadint",
        args: "[addr] [type]",
        help: "[DEBUG CMD] Reads an integer from bq40 dataflash\n  See the bq40 reference manual sections 19.17 for \
               list of dataflash addresses\n  [type] can be of: u8, s8, u16, s16, u32, or s32",
    },
    CommandInfo {
        name: "bq40_mfgruncmd",
        args: "[cmd]",
        help: "[DEBUG CMD] Sends the requested manufacturing command to the bq40 to run that command (no \
               readback)\n  See the bq40 reference manual section 18.1 for list of commands",
    },
    CommandInfo {
        name: "bq40_mfginfo",
        args: "",
        help: "Dumps the manufacturing info pulled from the device at startup",
    },
    CommandInfo { name: "bq40_state", args: "", help: "Dumps the general state of the bq40" },
    CommandInfo { name: "bq40_cell_state", args: "", help: "Dumps the state of individual cells on the bq40" },
    CommandInfo {
        name: "bq40_status_flags",
        args: "[register]",
        help: "Dumps the requested status flags register. Valid options for register are:\n - operation: Reports \
               the general operation status of the bq40\n - battery: Reports the standardized Smart Battery System \
               status\n - manufacturing: Reports the enable status of the various subsystems on the B40\n - \
               gauging: Reports the state of gas gauging system\n - charging: Reports the state of advanced charge \
               algorithm system",
    },
    CommandInfo {
        name: "bq40_dfwrite",
        args: "[--iknowwhatimdoingiswear] [flash addr] [hex data...]",
        help: "[DEBUG CMD] Writes the requested data to the bq40 dataflash at the requested address\n!!! WARNING !!! \
               This command can modify behavior of the battery charge chip!\nTHIS MAY PUT IT IN AN UNSAFE STATE! DO \
               NOT USE THIS COMMAND UNLESS YOU KNOW WHAT YOU\nARE DOING! This command can be used in place of \
               disassembling the battery to connect\nit to bq studio. DO NOT USE IF YOU HAVEN'T THOROUGHLY READ THE \
               BQ40 REFERENCE MANUAL!\n  [flash addr]: The starting flash address to write (in decimal, or prefixed \
               with 0x hex)\n  [hex data...]: String of hex characters to write to dataflash (up to 32 bytes, 64 \
               chars, no spaces or 0x)",
    },
];

/// Guard flag `bq40_dfwrite` insists on.
pub const DF_WRITE_GUARD: &str = "--iknowwhatimdoingiswear";

/// Runs one command, writing its output to `out`, and returns the C return code (0 on success).
pub async fn run<G: Gauge, W: Write>(argv: &[&str], gauge: &mut G, s: &Snapshot, out: &mut W) -> i32 {
    // Output errors only happen when the response buffer is full; the C version truncated silently
    run_inner(argv, gauge, s, out).await.unwrap_or(1)
}

type Res = Result<i32, fmt::Error>;

async fn run_inner<G: Gauge, W: Write>(argv: &[&str], gauge: &mut G, s: &Snapshot, out: &mut W) -> Res {
    let Some(name) = argv.first() else {
        writeln!(out, "Invalid Syntax!")?;
        return Ok(1);
    };
    match *name {
        "bq40_sbsreadint" => sbs_readint(argv, gauge, out).await,
        "bq40_sbsreadblock" => readblock(argv, gauge, out, false).await,
        "bq40_mfgreadblock" => readblock(argv, gauge, out, true).await,
        "bq40_mfgruncmd" => mfg_runcmd(argv, gauge, out).await,
        "bq40_dfreadint" => df_readint(argv, gauge, out).await,
        "bq40_dfwrite" => df_write(argv, gauge, out).await,
        "bq40_mfginfo" => mfginfo(s, out),
        "bq40_state" => state(s, out),
        "bq40_cell_state" => cell_state(s, out),
        "bq40_status_flags" => status_flags(argv, gauge, s, out).await,
        other => {
            writeln!(out, "Unknown command '{other}'")?;
            Ok(1)
        }
    }
}

/// Parses a decimal or `0x` hex number in `0..=max` (`dbg_cmd_get_uint`).
fn parse_uint<W: Write>(out: &mut W, s: &str, max: i64) -> Result<Option<i64>, fmt::Error> {
    let parsed = match s.strip_prefix("0x") {
        Some(hex) => i64::from_str_radix(hex, 16),
        None => s.parse::<i64>(),
    };
    match parsed {
        Err(_) => {
            writeln!(out, "Invalid Number: '{s}'")?;
            Ok(None)
        }
        Ok(v) if !(0..=max).contains(&v) => {
            writeln!(out, "Integer '{v}' out of range")?;
            Ok(None)
        }
        Ok(v) => Ok(Some(v)),
    }
}

/// Parses `u8`/`s8`/`u16`/`s16`/`u32`/`s32` (`dbg_cmd_get_int_type`).
fn parse_int_type<W: Write>(out: &mut W, s: &str) -> Result<Option<(Width, bool)>, fmt::Error> {
    Ok(Some(match s {
        "u8" => (Width::W8, false),
        "s8" => (Width::W8, true),
        "u16" => (Width::W16, false),
        "s16" => (Width::W16, true),
        "u32" => (Width::W32, false),
        "s32" => (Width::W32, true),
        _ => {
            writeln!(out, "Invalid Integer Type: '{s}'")?;
            return Ok(None);
        }
    }))
}

/// Writes an integer in hex and decimal (`dbg_cmd_show_int_result`). Like C's `%lX` of a `long`, negative values
/// show as 32 bit two's complement.
pub fn show_int<W: Write>(out: &mut W, val: u32, width: Width, signed: bool) -> fmt::Result {
    let digits = match width {
        Width::W8 => 2,
        Width::W16 => 4,
        Width::W32 => 8,
    };
    if signed {
        let v = match width {
            Width::W8 => val as u8 as i8 as i32,
            Width::W16 => val as u16 as i16 as i32,
            Width::W32 => val as i32,
        };
        writeln!(out, "0x{:0digits$X} ({})", v as u32, v)
    } else {
        let v = match width {
            Width::W8 => val as u8 as u32,
            Width::W16 => val as u16 as u32,
            Width::W32 => val,
        };
        writeln!(out, "0x{v:0digits$X} ({v})")
    }
}

/// Lists the bits of a register (`canmore_print_bit_array`): every named bit with its value if `list_all`,
/// otherwise only the set bits.
pub fn print_bits<W: Write>(out: &mut W, val: u32, names: &[BitName], list_all: bool) -> fmt::Result {
    for i in 0..32 {
        let set = sbs::bit(val, i);
        match (sbs::bit_name(names, i), list_all) {
            (Some(name), true) => writeln!(out, " - {name}: {}", set as u8)?,
            (None, true) if set => writeln!(out, " - Unknown Bit {i}: 1")?,
            (Some(name), false) if set => writeln!(out, " - {name}")?,
            (None, false) if set => writeln!(out, " - Unknown ({i})")?,
            _ => {}
        }
    }
    Ok(())
}

/// Describes a gauge error (`bq40_decode_err`). C printed the source line; this prints the failing command.
pub fn decode_error<W: Write>(out: &mut W, err: &BqError) -> fmt::Result {
    write!(out, "Error Executing Command: [cmd 0x{:04X}] ", err.command)?;
    match err.kind {
        BqErrorKind::InvalidDeviceType => {
            writeln!(out, "BQ_ERROR_INVALID_DEVICE_TYPE (Found 0x{:04X} instead)", err.arg)
        }
        BqErrorKind::InvalidSerial => writeln!(out, "BQ_ERROR_INVALID_SERIAL (Read {})", err.arg),
        BqErrorKind::SerialChanged => writeln!(out, "BQ_ERROR_SERIAL_CHANGED (Now {})", err.arg),
        BqErrorKind::InvalidState => writeln!(out, "BQ_ERROR_INVALID_STATE"),
        BqErrorKind::InvalidMacRespLen => writeln!(out, "BQ_ERROR_INVALID_MAC_RESP_LEN (got {} instead)", err.arg),
        BqErrorKind::BattStatusError => writeln!(
            out,
            "BQ_ERROR_BATT_STATUS_ERROR (BQ40 returned error: {})",
            sbs::battery_status_error_name(err.arg)
        ),
        BqErrorKind::BadDbgCommand => writeln!(out, "Unknown Error ({}, arg: {})", err.kind.code(), err.arg),
        BqErrorKind::Smbus(e) => writeln!(out, "{}", e.name()),
    }
}

/// Runs a gauge operation and checks the result kind (`dbg_cmd_xfer_and_wait`). Returns the C error code on failure.
async fn transfer<G: Gauge, W: Write>(
    gauge: &mut G,
    out: &mut W,
    op: DebugOp,
    expect: fn(&DebugResult) -> bool,
) -> Result<Result<DebugResult, i32>, fmt::Error> {
    let result = gauge.execute(op).await;
    Ok(match result {
        DebugResult::Timeout => {
            writeln!(out, "Timed out waiting for response from core 1!")?;
            Err(1)
        }
        DebugResult::Err(err) => {
            decode_error(out, &err)?;
            Err(2)
        }
        r if expect(&r) => Ok(r),
        _ => {
            writeln!(out, "Invalid Result Type from Core 1")?;
            Err(1)
        }
    })
}

fn is_int(r: &DebugResult) -> bool {
    matches!(r, DebugResult::Int(_))
}

fn is_block(r: &DebugResult) -> bool {
    matches!(r, DebugResult::Block { .. })
}

fn is_ok(r: &DebugResult) -> bool {
    matches!(r, DebugResult::Ok)
}

macro_rules! tri {
    ($e:expr, $code:expr) => {
        match $e? {
            Some(v) => v,
            None => return Ok($code),
        }
    };
}

async fn sbs_readint<G: Gauge, W: Write>(argv: &[&str], gauge: &mut G, out: &mut W) -> Res {
    if argv.len() != 3 {
        writeln!(out, "Invalid Syntax!")?;
        return Ok(1);
    }
    let cmd = tri!(parse_uint(out, argv[1], u8::MAX as i64), -1) as u8;
    let (width, signed) = tri!(parse_int_type(out, argv[2]), 1);
    match transfer(gauge, out, DebugOp::SbsReadInt { cmd, width }, is_int).await? {
        Ok(DebugResult::Int(v)) => {
            write!(out, "SBS Reg[0x{cmd:02X}]: ")?;
            show_int(out, v, width, signed)?;
            Ok(0)
        }
        Ok(_) => Ok(1),
        Err(code) => Ok(code),
    }
}

async fn readblock<G: Gauge, W: Write>(argv: &[&str], gauge: &mut G, out: &mut W, mfg: bool) -> Res {
    if argv.len() != 2 {
        writeln!(out, "Invalid Syntax!")?;
        return Ok(1);
    }
    let max = if mfg { u16::MAX as i64 } else { u8::MAX as i64 };
    let cmd = tri!(parse_uint(out, argv[1], max), -1);
    let op = if mfg { DebugOp::MfgReadBlock { cmd: cmd as u16 } } else { DebugOp::SbsReadBlock { cmd: cmd as u8 } };
    match transfer(gauge, out, op, is_block).await? {
        Ok(DebugResult::Block { data, len }) => {
            if mfg {
                write!(out, "MFG Reg[0x{cmd:04X}] (Len: {len}): ")?;
            } else {
                write!(out, "SBS Reg[0x{cmd:02X}] (Len: {len}): ")?;
            }
            for b in &data[..len as usize] {
                write!(out, "{b:02X}")?;
            }
            writeln!(out)?;
            Ok(0)
        }
        Ok(_) => Ok(1),
        Err(code) => Ok(code),
    }
}

async fn mfg_runcmd<G: Gauge, W: Write>(argv: &[&str], gauge: &mut G, out: &mut W) -> Res {
    if argv.len() != 2 {
        writeln!(out, "Invalid Syntax!")?;
        return Ok(1);
    }
    let cmd = tri!(parse_uint(out, argv[1], u16::MAX as i64), -1) as u16;
    match transfer(gauge, out, DebugOp::MfgCmd { cmd }, is_ok).await? {
        Ok(_) => {
            writeln!(out, "OK")?;
            Ok(0)
        }
        Err(code) => Ok(code),
    }
}

async fn df_readint<G: Gauge, W: Write>(argv: &[&str], gauge: &mut G, out: &mut W) -> Res {
    if argv.len() != 3 {
        writeln!(out, "Invalid Syntax!")?;
        return Ok(1);
    }
    let addr = tri!(parse_uint(out, argv[1], u16::MAX as i64), -1) as u16;
    let (width, signed) = tri!(parse_int_type(out, argv[2]), 1);
    let (data, len) = match transfer(gauge, out, DebugOp::MfgReadBlock { cmd: addr }, is_block).await? {
        Ok(DebugResult::Block { data, len }) => (data, len as usize),
        Ok(_) => return Ok(1),
        Err(code) => return Ok(code),
    };
    let needed = match width {
        Width::W8 => 1,
        Width::W16 => 2,
        Width::W32 => 4,
    };
    if len < needed {
        writeln!(out, "Block read didn't return enough bytes: {len}")?;
        return Ok(1);
    }
    let mut bytes = [0u8; 4];
    bytes[..needed].copy_from_slice(&data[..needed]);
    write!(out, "Data Flash [0x{addr:02X}]: ")?;
    show_int(out, u32::from_le_bytes(bytes), width, signed)?;
    Ok(0)
}

/// Decodes the `bq40_dfwrite` hex string into bytes; `None` if it isn't an even number of hex digits.
pub fn parse_hex(s: &str, buf: &mut [u8; BLOCK_LEN]) -> Result<usize, &'static str> {
    if s.is_empty() || !s.len().is_multiple_of(2) {
        return Err("Invalid hex string");
    }
    let len = s.len() / 2;
    if len > BLOCK_LEN {
        return Err("Can only write up to 32 bytes at a time");
    }
    for (i, pair) in s.as_bytes().chunks(2).enumerate() {
        let nibble = |c: u8| (c as char).to_digit(16).ok_or("Invalid hex string");
        buf[i] = (nibble(pair[0])? << 4 | nibble(pair[1])?) as u8;
    }
    Ok(len)
}

async fn df_write<G: Gauge, W: Write>(argv: &[&str], gauge: &mut G, out: &mut W) -> Res {
    if argv.len() < 3 {
        writeln!(out, "Invalid Syntax!")?;
        return Ok(1);
    }
    if argv[1] != DF_WRITE_GUARD {
        writeln!(out, "Missing Guard Flag! Refusing to execute!")?;
        return Ok(1);
    }
    if argv.len() < 4 {
        writeln!(out, "Invalid Syntax!")?;
        return Ok(1);
    }
    let addr = tri!(parse_uint(out, argv[2], u16::MAX as i64), -1) as u16;
    let mut data = [0u8; BLOCK_LEN];
    let len = match parse_hex(argv[3], &mut data) {
        Ok(len) => len,
        Err(msg) => {
            writeln!(out, "{msg}")?;
            return Ok(1);
        }
    };
    match transfer(gauge, out, DebugOp::DfWrite { addr, data, len: len as u8 }, is_ok).await? {
        Ok(_) => {
            writeln!(out, "OK")?;
            Ok(0)
        }
        Err(code) => Ok(code),
    }
}

fn not_connected<W: Write>(s: &Snapshot, out: &mut W) -> Result<bool, fmt::Error> {
    if s.state.is_connected() {
        Ok(false)
    } else {
        writeln!(out, "Not Connected to BQ40!")?;
        Ok(true)
    }
}

fn mfginfo<W: Write>(s: &Snapshot, out: &mut W) -> Res {
    if not_connected(s, out)? {
        return Ok(1);
    }
    let m = &s.mfg;
    // C printed "%dd" here, a stray 'd'
    writeln!(out, "Serial: {}", m.serial)?;
    writeln!(out, "Manufacture Date: {}/{}/{}", m.mfg_month, m.mfg_day, m.mfg_year)?;
    writeln!(out, "Device Type: 0x{:04X}", m.device_type)?;
    write!(out, "Firmware Version: ")?;
    for b in m.firmware_version {
        write!(out, "{b:02X}")?;
    }
    writeln!(out)?;
    writeln!(out, "Scale Factor: {}", m.scale_factor)?;
    Ok(0)
}

fn write_time<W: Write>(out: &mut W, label: &str, minutes: u16) -> fmt::Result {
    if minutes == u16::MAX {
        writeln!(out, "{label}: <A Long Time>")
    } else {
        writeln!(out, "{label}: {}h {}m", minutes / 60, minutes % 60)
    }
}

fn state<W: Write>(s: &Snapshot, out: &mut W) -> Res {
    writeln!(out, "Batt State: {}", s.state.name())?;
    if !s.state.is_connected() {
        return Ok(0);
    }
    let i = &s.info;
    writeln!(out, "SOC: {}%", i.relative_soc)?;
    writeln!(out, "SOC Max Error: {}%", i.max_error)?;
    writeln!(out, "Temperature: {:.1} C", i.temperature as f64 / 10.0 - 273.2)?;
    write_time(out, "Time to Empty", i.time_to_empty)?;
    write_time(out, "Time to Full", i.time_to_full)?;
    writeln!(out, "Batt Voltage: {}", format_millivolts(i.da_status1.batt_voltage).as_str())?;
    writeln!(out, "Pack Voltage: {}", format_millivolts(i.da_status1.pack_voltage).as_str())?;
    writeln!(out, "Current Now: {:.3}A", i.current as f64 / 1000.0)?;
    writeln!(out, "Avg Current: {:.3}A", i.avg_current as f64 / 1000.0)?;
    writeln!(out, "Requested Charging Voltage: {}", format_millivolts(i.charging_voltage).as_str())?;
    writeln!(out, "Requested Charging Current: {:.3}A", i.charging_current as f64 / 1000.0)?;
    if i.safety_status != 0 {
        writeln!(out, "Safety Status: 0x{:08X}", i.safety_status)?;
        print_bits(out, i.safety_status, sbs::SAFETY_STATUS_NAMES, false)?;
    } else {
        writeln!(out, "Safety Status: No Faults")?;
    }
    if i.pf_status != 0 {
        writeln!(out, "PF Status: 0x{:08X}", i.pf_status)?;
        print_bits(out, i.pf_status, sbs::PF_STATUS_NAMES, false)?;
    } else {
        writeln!(out, "PF Status: No Faults")?;
    }
    Ok(0)
}

fn cell_state<W: Write>(s: &Snapshot, out: &mut W) -> Res {
    if not_connected(s, out)? {
        return Ok(1);
    }
    writeln!(out, "Cell Voltages:")?;
    for (n, mv) in s.info.cell_voltages().iter().enumerate() {
        writeln!(out, "  {}: {}", n + 1, format_millivolts(*mv).as_str())?;
    }
    writeln!(out, "Cell Currents:")?;
    for (n, ma) in s.info.da_status1.cell_current.iter().enumerate() {
        writeln!(out, "  {}: {:.3}A", n + 1, *ma as f64 / 1000.0)?;
    }
    writeln!(out, "  5: Not Monitored")?;
    Ok(0)
}

async fn status_flags<G: Gauge, W: Write>(argv: &[&str], gauge: &mut G, s: &Snapshot, out: &mut W) -> Res {
    if argv.len() != 2 {
        writeln!(out, "Invalid Syntax!")?;
        return Ok(1);
    }
    if not_connected(s, out)? {
        return Ok(1);
    }
    match argv[1] {
        "manufacturing" => {
            let v = s.mfg.manufacturing_status;
            writeln!(out, "Manufacturing Status: 0x{v:04X}")?;
            print_bits(out, v as u32, sbs::MANUFACTURING_STATUS_NAMES, true)?;
        }
        "operation" => {
            let v = s.info.operation_status;
            writeln!(out, "Operation Status: 0x{v:08X}")?;
            print_bits(out, v, sbs::OPERATION_STATUS_NAMES, true)?;
        }
        "battery" => {
            let v = s.info.battery_status;
            // C labelled this "Operation Status" too
            writeln!(out, "Battery Status: 0x{v:04X}")?;
            print_bits(out, v as u32, sbs::BATTERY_STATUS_NAMES, true)?;
        }
        reg @ ("gauging" | "charging") => {
            let (register, label, names) = if reg == "gauging" {
                (StatusRegister::Gauging, "Gauging", sbs::GAUGING_STATUS_NAMES)
            } else {
                (StatusRegister::Charging, "Charging", sbs::CHARGING_STATUS_NAMES)
            };
            let Some(v) = gauge.read_status(register).await else {
                writeln!(out, "Failed to read {reg} status register!")?;
                return Ok(1);
            };
            writeln!(out, "{label} Status: 0x{v:08X}")?;
            print_bits(out, v, names, true)?;
        }
        other => {
            writeln!(out, "Unknown Status Register: '{other}'")?;
            return Ok(1);
        }
    }
    Ok(0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::smbus::SmbusError;
    use core::future::Future;
    use core::pin::pin;
    use core::task::{Context, Poll, Waker};
    use std::string::String;
    use std::vec::Vec;

    /// Runs a future that never waits.
    fn block_on<F: Future>(f: F) -> F::Output {
        let mut f = pin!(f);
        let mut cx = Context::from_waker(Waker::noop());
        loop {
            if let Poll::Ready(v) = f.as_mut().poll(&mut cx) {
                return v;
            }
        }
    }

    #[derive(Default)]
    struct MockGauge {
        ops: Vec<DebugOp>,
        reply: Option<DebugResult>,
        status: Option<u32>,
    }

    impl Gauge for MockGauge {
        async fn execute(&mut self, op: DebugOp) -> DebugResult {
            self.ops.push(op);
            self.reply.unwrap_or(DebugResult::Timeout)
        }

        async fn read_status(&mut self, _register: StatusRegister) -> Option<u32> {
            self.status
        }
    }

    fn exec(argv: &[&str], gauge: &mut MockGauge, s: &Snapshot) -> (i32, String) {
        let mut out = String::new();
        let code = block_on(run(argv, gauge, s, &mut out));
        (code, out)
    }

    use crate::monitor::BattState;

    fn connected() -> Snapshot {
        Snapshot { state: BattState::Discharging, ..Default::default() }
    }

    #[test]
    fn sbs_readint_parses_and_formats() {
        let mut g = MockGauge { reply: Some(DebugResult::Int(0xFFFE)), ..Default::default() };
        let (code, out) = exec(&["bq40_sbsreadint", "0x0a", "s16"], &mut g, &connected());
        assert_eq!(code, 0);
        assert_eq!(g.ops, [DebugOp::SbsReadInt { cmd: 0x0A, width: Width::W16 }]);
        assert_eq!(out, "SBS Reg[0x0A]: 0xFFFFFFFE (-2)\n");

        let (_, out) = exec(&["bq40_sbsreadint", "13", "u8"], &mut g, &connected());
        assert_eq!(out, "SBS Reg[0x0D]: 0xFE (254)\n");

        let (code, out) = exec(&["bq40_sbsreadint", "256", "u8"], &mut g, &connected());
        assert_eq!((code, out.as_str()), (-1, "Integer '256' out of range\n"));
        let (code, out) = exec(&["bq40_sbsreadint", "12x", "u8"], &mut g, &connected());
        assert_eq!((code, out.as_str()), (-1, "Invalid Number: '12x'\n"));
        let (code, out) = exec(&["bq40_sbsreadint", "1", "u64"], &mut g, &connected());
        assert_eq!((code, out.as_str()), (1, "Invalid Integer Type: 'u64'\n"));
        let (code, out) = exec(&["bq40_sbsreadint", "1"], &mut g, &connected());
        assert_eq!((code, out.as_str()), (1, "Invalid Syntax!\n"));
    }

    #[test]
    fn errors_and_timeouts() {
        let mut g = MockGauge {
            reply: Some(DebugResult::Err(BqError::smbus(SmbusError::AddrNak, 0x44))),
            ..Default::default()
        };
        let (code, out) = exec(&["bq40_mfgruncmd", "0x0041"], &mut g, &connected());
        assert_eq!(code, 2);
        assert_eq!(out, "Error Executing Command: [cmd 0x0044] PIO_SMBUS_ERR_ADDR_NAK\n");

        let mut g = MockGauge::default();
        let (code, out) = exec(&["bq40_mfgruncmd", "0x0041"], &mut g, &connected());
        assert_eq!((code, out.as_str()), (1, "Timed out waiting for response from core 1!\n"));

        let mut g = MockGauge { reply: Some(DebugResult::Int(1)), ..Default::default() };
        let (code, out) = exec(&["bq40_mfgruncmd", "1"], &mut g, &connected());
        assert_eq!((code, out.as_str()), (1, "Invalid Result Type from Core 1\n"));
    }

    #[test]
    fn blocks_and_data_flash() {
        let mut data = [0u8; BLOCK_LEN];
        data[..3].copy_from_slice(&[0xE8, 0x4A, 0x01]);
        let mut g = MockGauge { reply: Some(DebugResult::Block { data, len: 3 }), ..Default::default() };
        let (_, out) = exec(&["bq40_mfgreadblock", "0x0002"], &mut g, &connected());
        assert_eq!(out, "MFG Reg[0x0002] (Len: 3): E84A01\n");
        let (_, out) = exec(&["bq40_sbsreadblock", "0x56"], &mut g, &connected());
        assert_eq!(out, "SBS Reg[0x56] (Len: 3): E84A01\n");
        let (code, out) = exec(&["bq40_dfreadint", "0x4AE8", "u16"], &mut g, &connected());
        assert_eq!((code, out.as_str()), (0, "Data Flash [0x4AE8]: 0x4AE8 (19176)\n"));
        assert_eq!(g.ops.last(), Some(&DebugOp::MfgReadBlock { cmd: 0x4AE8 }));
        let (code, out) = exec(&["bq40_dfreadint", "0x4AE8", "u32"], &mut g, &connected());
        assert_eq!((code, out.as_str()), (1, "Block read didn't return enough bytes: 3\n"));
    }

    #[test]
    fn df_write_needs_the_guard() {
        let mut g = MockGauge { reply: Some(DebugResult::Ok), ..Default::default() };
        let (code, out) = exec(&["bq40_dfwrite", "0x4AE8", "01"], &mut g, &connected());
        assert_eq!((code, out.as_str()), (1, "Missing Guard Flag! Refusing to execute!\n"));
        assert!(g.ops.is_empty());
        let (code, out) = exec(&["bq40_dfwrite", DF_WRITE_GUARD, "0x4AE8", "0aFf"], &mut g, &connected());
        assert_eq!((code, out.as_str()), (0, "OK\n"));
        let mut data = [0u8; BLOCK_LEN];
        data[..2].copy_from_slice(&[0x0A, 0xFF]);
        assert_eq!(g.ops, [DebugOp::DfWrite { addr: 0x4AE8, data, len: 2 }]);
        let (code, out) = exec(&["bq40_dfwrite", DF_WRITE_GUARD, "1", "abc"], &mut g, &connected());
        assert_eq!((code, out.as_str()), (1, "Invalid hex string\n"));
        let long: String = core::iter::repeat_n('0', 66).collect();
        let (_, out) = exec(&["bq40_dfwrite", DF_WRITE_GUARD, "1", &long], &mut g, &connected());
        assert_eq!(out, "Can only write up to 32 bytes at a time\n");
    }

    #[test]
    fn state_dumps() {
        let mut g = MockGauge::default();
        let (code, out) = exec(&["bq40_cell_state"], &mut g, &Snapshot::default());
        assert_eq!((code, out.as_str()), (1, "Not Connected to BQ40!\n"));
        let (code, out) = exec(&["bq40_state"], &mut g, &Snapshot::default());
        assert_eq!((code, out.as_str()), (0, "Batt State: Uninitialized\n"));

        let mut s = connected();
        s.info.relative_soc = 80;
        s.info.temperature = 2982;
        s.info.time_to_empty = 125;
        s.info.time_to_full = u16::MAX;
        s.info.da_status1.batt_voltage = 18_050;
        s.info.current = -1500;
        s.info.safety_status = 1 << 1;
        let (code, out) = exec(&["bq40_state"], &mut g, &s);
        assert_eq!(code, 0);
        assert!(out.contains("Batt State: Discharging\n"));
        assert!(out.contains("Temperature: 25.0 C\n"));
        assert!(out.contains("Time to Empty: 2h 5m\n"));
        assert!(out.contains("Time to Full: <A Long Time>\n"));
        assert!(out.contains("Batt Voltage: 18.050V\n"));
        assert!(out.contains("Current Now: -1.500A\n"));
        assert!(out.contains("Safety Status: 0x00000002\n - Cell overvoltage [COV]\n"));
        assert!(out.contains("PF Status: No Faults\n"));
    }

    #[test]
    fn status_flags_list_every_named_bit() {
        let mut g = MockGauge { status: Some((1 << 6) | (1 << 9)), ..Default::default() };
        let (code, out) = exec(&["bq40_status_flags", "gauging"], &mut g, &connected());
        assert_eq!(code, 0);
        assert!(out.starts_with("Gauging Status: 0x00000240\n"));
        assert!(out.contains(" - Discharge/relax [DSG]: 1\n"));
        assert!(out.contains(" - Fully discharged [FD]: 0\n"));
        assert!(out.contains(" - Unknown Bit 9: 1\n"));
        g.status = None;
        let (code, out) = exec(&["bq40_status_flags", "charging"], &mut g, &connected());
        assert_eq!((code, out.as_str()), (1, "Failed to read charging status register!\n"));
        let (code, _) = exec(&["bq40_status_flags", "bogus"], &mut g, &connected());
        assert_eq!(code, 1);
    }

    #[test]
    fn every_command_is_dispatched() {
        let mut g = MockGauge::default();
        for c in COMMANDS {
            let (_, out) = exec(&[c.name], &mut g, &Snapshot::default());
            assert!(!out.starts_with("Unknown command"), "{}", c.name);
        }
    }
}
