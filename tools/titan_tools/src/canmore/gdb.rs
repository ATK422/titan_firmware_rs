//! What a GDB server needs from a debug stub, ported from `canmore_cpp/GDBClient.hpp`.

use anyhow::Result;

/// Word level memory access and the stub's registers, exposed by the debug and bootloader interfaces.
pub trait GdbTarget {
    /// Reads the word at `addr` (flash, RAM or ROM).
    fn read_memory(&mut self, addr: u32) -> Result<u32>;
    /// Writes the word at `addr` (RAM only).
    fn write_memory(&mut self, addr: u32, data: u32) -> Result<()>;
    /// PC of the debug stub.
    fn stub_pc(&mut self) -> Result<u32>;
    /// SP of the debug stub.
    fn stub_sp(&mut self) -> Result<u32>;
    /// LR of the debug stub.
    fn stub_lr(&mut self) -> Result<u32>;
    /// Checks the device still answers.
    fn ping(&mut self) -> Result<()>;
}

const ERR_EINVAL: &str = "E16";

/// Target description reported to GDB: a Cortex-M0+ with the M-profile register set.
pub const TARGET_XML: &str = concat!(
    "<target version=\"1.0\">",
    "<architecture>armv6s-m</architecture>",
    "<feature name=\"org.gnu.gdb.arm.m-profile\">",
    "<reg name=\"r0\" bitsize=\"32\"/>",
    "<reg name=\"r1\" bitsize=\"32\"/>",
    "<reg name=\"r2\" bitsize=\"32\"/>",
    "<reg name=\"r3\" bitsize=\"32\"/>",
    "<reg name=\"r4\" bitsize=\"32\"/>",
    "<reg name=\"r5\" bitsize=\"32\"/>",
    "<reg name=\"r6\" bitsize=\"32\"/>",
    "<reg name=\"r7\" bitsize=\"32\"/>",
    "<reg name=\"r8\" bitsize=\"32\"/>",
    "<reg name=\"r9\" bitsize=\"32\"/>",
    "<reg name=\"r10\" bitsize=\"32\"/>",
    "<reg name=\"r11\" bitsize=\"32\"/>",
    "<reg name=\"r12\" bitsize=\"32\"/>",
    "<reg name=\"sp\" bitsize=\"32\" type=\"data_ptr\"/>",
    "<reg name=\"lr\" bitsize=\"32\"/>",
    "<reg name=\"pc\" bitsize=\"32\" type=\"code_ptr\"/>",
    "<reg name=\"xpsr\" bitsize=\"32\"/>",
    "</feature>",
    "</target>"
);

fn parse_hex(s: &str) -> Option<u64> {
    if s.is_empty() || s.len() > 16 {
        return None;
    }
    u64::from_str_radix(s, 16).ok()
}

fn word_hex(word: u32) -> String {
    word.to_le_bytes().iter().map(|b| format!("{b:02X}")).collect()
}

/// Answers GDB remote serial protocol packets from a [`GdbTarget`], ported from `gdbserver.cpp`.
///
/// The debug stub can only read and write memory and report where it is, so the target is presented as a stopped
/// single thread: continue and step return immediately, and only SP, LR and PC are known.
pub struct GdbHandler<'a> {
    target: &'a mut dyn GdbTarget,
    pub detach_received: bool,
    pub send_ack: bool,
    /// Errors to show the user (register reads failing, for example).
    pub errors: Vec<String>,
}

impl<'a> GdbHandler<'a> {
    pub fn new(target: &'a mut dyn GdbTarget) -> Self {
        GdbHandler { target, detach_received: false, send_ack: true, errors: Vec::new() }
    }

    pub fn reset_state(&mut self) {
        self.detach_received = false;
        self.send_ack = true;
    }

    pub fn ping(&mut self) -> Result<()> {
        self.target.ping()
    }

    /// The response to one packet (without framing).
    pub fn process_packet(&mut self, req: &str) -> String {
        let Some(cmd) = req.chars().next() else { return String::new() };
        let rest = &req[1..];
        match cmd {
            '?' | 'c' => "S00".into(),
            's' => "T05".into(),
            'D' | 'k' => {
                self.detach_received = true;
                "OK".into()
            }
            'H' => "OK".into(),
            'q' => self.query(rest),
            'Q' if rest == "StartNoAckMode" => {
                self.send_ack = false;
                "OK".into()
            }
            'g' => match self.registers() {
                Ok(regs) => regs,
                Err(e) => {
                    self.errors.push(format!("Error reading registers: {e:#}"));
                    "E00".into()
                }
            },
            'G' => "E00".into(),
            'm' => self.memory_read(rest),
            'M' => self.memory_write(rest),
            // Unknown packets must get an empty response
            _ => String::new(),
        }
    }

    fn registers(&mut self) -> Result<String> {
        let pc = self.target.stub_pc()?;
        let sp = self.target.stub_sp()?;
        let lr = self.target.stub_lr()?;
        // r0-r12 unknown
        Ok(format!("{}{}{}{}00000000", "x".repeat(13 * 8), word_hex(sp), word_hex(lr), word_hex(pc)))
    }

    fn query(&self, sub: &str) -> String {
        if sub.starts_with("Supported") {
            "PacketSize=4000;qXfer:features:read+;QStartNoAckMode+".into()
        } else if sub == "Attached" {
            "1".into()
        } else if sub == "fThreadInfo" {
            "m 1".into()
        } else if sub == "sThreadInfo" {
            "l".into()
        } else if sub == "C" {
            "T01;".into()
        } else if sub.starts_with("Xfer") {
            Self::xfer(sub)
        } else {
            String::new()
        }
    }

    /// `Xfer:features:read:target.xml:offset,length`
    fn xfer(sub: &str) -> String {
        let parts: Vec<&str> = sub.split(':').collect();
        let [_, object, op, annex, range] = parts.as_slice() else { return "E00".into() };
        let Some((offset, len)) = range.split_once(',') else { return "E00".into() };
        let (Some(offset), Some(len)) = (parse_hex(offset), parse_hex(len)) else { return "E00".into() };
        if *op != "read" || *object != "features" {
            return String::new();
        }
        if *annex != "target.xml" {
            return "E00".into();
        }
        let (offset, len) = (offset as usize, len as usize);
        if offset > TARGET_XML.len() {
            "l".into()
        } else if offset + len > TARGET_XML.len() {
            format!("l{}", &TARGET_XML[offset..])
        } else {
            format!("m{}", &TARGET_XML[offset..offset + len])
        }
    }

    fn memory_read(&mut self, sub: &str) -> String {
        let Some((addr, len)) = sub.split_once(',') else { return ERR_EINVAL.into() };
        let (Some(addr), Some(len)) = (parse_hex(addr), parse_hex(len)) else { return ERR_EINVAL.into() };
        if addr + len > u32::MAX as u64 {
            return ERR_EINVAL.into();
        }
        let mut out = String::new();
        let mut cached: Option<(u32, u32)> = None;
        for a in addr..addr + len {
            let word_addr = (a & !3) as u32;
            let word = match cached {
                Some((cached_addr, word)) if cached_addr == word_addr => word,
                _ => match self.target.read_memory(word_addr) {
                    Ok(word) => {
                        cached = Some((word_addr, word));
                        word
                    }
                    // A partial read is still useful to GDB
                    Err(_) if !out.is_empty() => return out,
                    Err(_) => return ERR_EINVAL.into(),
                },
            };
            out += &format!("{:02X}", (word >> ((a % 4) * 8)) as u8);
        }
        out
    }

    fn memory_write(&mut self, sub: &str) -> String {
        let Some((header, data)) = sub.split_once(':') else { return ERR_EINVAL.into() };
        let Some((addr, len)) = header.split_once(',') else { return ERR_EINVAL.into() };
        let (Some(addr), Some(len)) = (parse_hex(addr), parse_hex(len)) else { return ERR_EINVAL.into() };
        if addr + len > u32::MAX as u64 || (data.len() as u64) < len * 2 {
            return ERR_EINVAL.into();
        }
        let bytes: Option<Vec<u8>> = (0..len as usize)
            .map(|i| data.get(i * 2..i * 2 + 2).and_then(|b| u8::from_str_radix(b, 16).ok()))
            .collect();
        let Some(bytes) = bytes else { return ERR_EINVAL.into() };

        let result = (|| -> Result<()> {
            let mut a = addr as u32;
            let mut remaining = &bytes[..];
            while !remaining.is_empty() {
                let word_addr = a & !3;
                let first = (a % 4) as usize;
                let count = (4 - first).min(remaining.len());
                // Partial words keep the bytes not being written
                let mut word =
                    if first == 0 && count == 4 { [0; 4] } else { self.target.read_memory(word_addr)?.to_le_bytes() };
                word[first..first + count].copy_from_slice(&remaining[..count]);
                self.target.write_memory(word_addr, u32::from_le_bytes(word))?;
                remaining = &remaining[count..];
                a += count as u32;
            }
            Ok(())
        })();
        match result {
            Ok(()) => "OK".into(),
            Err(_) => ERR_EINVAL.into(),
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum ParseState {
    StartOfPacket,
    Data,
    EscapedData,
    Checksum1,
    Checksum2,
}

/// Frames and unframes GDB remote protocol packets.
pub struct GdbPacketParser {
    state: ParseState,
    buf: Vec<u8>,
    calc_checksum: u8,
    exp_checksum: u8,
}

impl Default for GdbPacketParser {
    fn default() -> Self {
        GdbPacketParser { state: ParseState::StartOfPacket, buf: Vec::new(), calc_checksum: 0, exp_checksum: 0 }
    }
}

impl GdbPacketParser {
    /// Feeds received bytes, returning the bytes to send back (acks and responses).
    pub fn process(&mut self, handler: &mut GdbHandler, data: &[u8]) -> Vec<u8> {
        let mut out = Vec::new();
        for &c in data {
            let mut fail = false;
            let mut complete = false;
            match (c, self.state) {
                (b'$', _) => {
                    self.state = ParseState::Data;
                    self.buf.clear();
                    self.calc_checksum = 0;
                }
                (b'#', ParseState::Data) => self.state = ParseState::Checksum1,
                (b'#', _) => fail = true,
                (b'}', ParseState::Data) => {
                    self.calc_checksum = self.calc_checksum.wrapping_add(c);
                    self.state = ParseState::EscapedData;
                }
                (b'}', _) => fail = true,
                (_, ParseState::Data) => {
                    self.calc_checksum = self.calc_checksum.wrapping_add(c);
                    self.buf.push(c);
                }
                (_, ParseState::EscapedData) => {
                    self.calc_checksum = self.calc_checksum.wrapping_add(c);
                    self.buf.push(c ^ 0x20);
                    self.state = ParseState::Data;
                }
                (_, ParseState::Checksum1) => match (c as char).to_digit(16) {
                    Some(v) => {
                        self.exp_checksum = (v as u8) << 4;
                        self.state = ParseState::Checksum2;
                    }
                    None => fail = true,
                },
                (_, ParseState::Checksum2) => match (c as char).to_digit(16) {
                    Some(v) if self.exp_checksum | v as u8 == self.calc_checksum => complete = true,
                    _ => fail = true,
                },
                // Garbage between packets (including acks) is ignored
                (_, ParseState::StartOfPacket) => {}
            }

            if fail {
                if self.state != ParseState::StartOfPacket {
                    self.state = ParseState::StartOfPacket;
                    if handler.send_ack {
                        out.push(b'-');
                    }
                }
            } else if complete {
                if handler.send_ack {
                    out.push(b'+');
                }
                let req = String::from_utf8_lossy(&self.buf).into_owned();
                let resp = handler.process_packet(&req);
                out.extend(frame_packet(&resp));
                self.state = ParseState::StartOfPacket;
            }
        }
        out
    }
}

/// `$data#cs`, escaping the special characters.
pub fn frame_packet(data: &str) -> Vec<u8> {
    let mut out = vec![b'$'];
    for &b in data.as_bytes() {
        if matches!(b, b'$' | b'#' | b'*' | b'}') {
            out.extend([b'}', b ^ 0x20]);
        } else {
            out.push(b);
        }
    }
    // The checksum covers the escaped data (the C++ server summed the unescaped data, which only mattered for
    // responses containing special characters)
    let checksum = out[1..].iter().fold(0u8, |acc, &b| acc.wrapping_add(b));
    out.extend(format!("#{checksum:02X}").into_bytes());
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    #[derive(Default)]
    struct FakeTarget {
        mem: HashMap<u32, u32>,
    }

    impl GdbTarget for FakeTarget {
        fn read_memory(&mut self, addr: u32) -> Result<u32> {
            assert_eq!(addr % 4, 0);
            self.mem.get(&addr).copied().ok_or_else(|| anyhow::anyhow!("bad address"))
        }
        fn write_memory(&mut self, addr: u32, data: u32) -> Result<()> {
            assert_eq!(addr % 4, 0);
            self.mem.insert(addr, data);
            Ok(())
        }
        fn stub_pc(&mut self) -> Result<u32> {
            Ok(0x1000_0100)
        }
        fn stub_sp(&mut self) -> Result<u32> {
            Ok(0x2004_0000)
        }
        fn stub_lr(&mut self) -> Result<u32> {
            Ok(0xFFFF_FFF9)
        }
        fn ping(&mut self) -> Result<()> {
            Ok(())
        }
    }

    #[test]
    fn packets() {
        let mut target = FakeTarget::default();
        target.mem.insert(0x2000_0000, 0x4433_2211);
        target.mem.insert(0x2000_0004, 0x8877_6655);
        let mut handler = GdbHandler::new(&mut target);

        assert_eq!(handler.process_packet("m20000001,6"), "223344556677");
        assert_eq!(handler.process_packet("m30000000,4"), ERR_EINVAL);
        // Partial read up to the unmapped word
        assert_eq!(handler.process_packet("m20000006,4"), "7788");
        assert_eq!(handler.process_packet("M20000003,2:AABB"), "OK");
        let regs = handler.process_packet("g");
        assert!(regs.ends_with("00000420F9FFFFFF0001001000000000"), "{regs}");
        assert!(regs.starts_with(&"x".repeat(104)));
        assert_eq!(handler.process_packet("qXfer:features:read:target.xml:0,10"), format!("m{}", &TARGET_XML[..16]));
        assert_eq!(handler.process_packet("qXfer:features:read:other.xml:0,10"), "E00");
        assert_eq!(handler.process_packet("vMustReplyEmpty"), "");
        assert_eq!(handler.process_packet("QStartNoAckMode"), "OK");
        assert!(!handler.send_ack);
        assert_eq!(handler.process_packet("D"), "OK");
        assert!(handler.detach_received);
        assert_eq!(target.mem[&0x2000_0000], 0xAA33_2211);
        assert_eq!(target.mem[&0x2000_0004], 0x8877_66BB);
    }

    #[test]
    fn framing() {
        let mut target = FakeTarget::default();
        let mut handler = GdbHandler::new(&mut target);
        let mut parser = GdbPacketParser::default();
        // "?" has checksum 0x3F
        assert_eq!(parser.process(&mut handler, b"+$?#3f"), b"+$S00#B3");
        // Bad checksum is NAKed
        assert_eq!(parser.process(&mut handler, b"$?#00"), b"-");
        assert_eq!(frame_packet("a#b"), b"$a}\x03b#43");
    }
}
