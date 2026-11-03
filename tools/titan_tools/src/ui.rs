//! Terminal output helpers shared by the tools (`TerminalDraw.hpp`, `BinaryInfoDisplay.cpp`, `cli_core.cpp`).

use std::fmt::Write as _;
use std::io::{BufRead, Write};

use crate::binary_info::AppInfo;

pub const CLEAR_TERMINAL: &str = "\x1b[2J";
pub const CURSOR_GOTO_START: &str = "\x1b[1;1H";
pub const CLEAR_LINE_AFTER: &str = "\x1b[0K";
pub const COLOR_RESET: &str = "\x1b[0m";
pub const COLOR_TITLE: &str = "\x1b[1;35m";
pub const COLOR_HEADER: &str = "\x1b[90m";
pub const COLOR_NAME: &str = "\x1b[1;94m";
pub const COLOR_BODY: &str = "\x1b[0;32m";
pub const COLOR_NOTICE: &str = "\x1b[0;93m";
pub const COLOR_ERROR: &str = "\x1b[1;31m";
pub const COLOR_PROMPT: &str = "\x1b[1;32m";
/// Colors used by the upload tool and binary info dumps.
pub const COLOR_ID: &str = "\x1b[0m";
pub const COLOR_INFO_HEADER: &str = "\x1b[1;90m";
pub const COLOR_INFO_NAME: &str = "\x1b[0;33m";

/// `=====Title=====` centered in `width` columns.
pub fn header(title: &str, width: usize) -> String {
    let padding = width.saturating_sub(title.len());
    let before = padding / 2;
    let after = padding - before;
    format!("{COLOR_TITLE}{}{title}{}{COLOR_RESET}{CLEAR_LINE_AFTER}", "=".repeat(before), "=".repeat(after))
}

pub fn render_header(title: &str) {
    println!("{}", header(title, 40));
}

pub fn render_name(name: &str) {
    println!("{COLOR_NAME}   {name}{COLOR_RESET}{CLEAR_LINE_AFTER}");
}

pub fn field(name: &str, value: &str) -> String {
    let pad = 16usize.saturating_sub(name.len() + 1);
    format!("{COLOR_HEADER}        {name}:{}{COLOR_BODY}{value}{COLOR_RESET}{CLEAR_LINE_AFTER}", " ".repeat(pad))
}

pub fn render_field(name: &str, value: &str) {
    println!("{}", field(name, value));
}

/// Parses a decimal or `0x` prefixed hex integer no larger than `max`.
pub fn decode_u32(s: &str, max: u32) -> Option<u32> {
    let value = match s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        Some(hex) if !hex.is_empty() => u32::from_str_radix(hex, 16).ok()?,
        Some(_) => return None,
        None if s.chars().all(|c| c.is_ascii_digit()) && !s.is_empty() => s.parse().ok()?,
        None => return None,
    };
    (value <= max).then_some(value)
}

/// Hex dump with an ASCII column, 16 bytes per line, addresses starting at `address`.
pub fn hex_dump(address: u32, data: &[u8]) -> String {
    let mut out = String::new();
    for (line_no, line) in data.chunks(16).enumerate() {
        let _ = write!(out, "0x{:08X}: ", address as usize + line_no * 16);
        for i in 0..16 {
            match line.get(i) {
                Some(b) => {
                    let _ = write!(out, "{b:02X} ");
                }
                None => out.push_str("   "),
            }
            if i == 7 {
                out.push(' ');
            }
        }
        out.push_str(" |  ");
        out.extend(line.iter().map(|&b| if (b' '..=b'~').contains(&b) { b as char } else { '.' }));
        out.push('\n');
    }
    out
}

/// Asks a yes/no question, defaulting to no.
pub fn confirm(msg: &str) -> bool {
    print!("{msg} [y/N]: ");
    let _ = std::io::stdout().flush();
    let mut input = String::new();
    if std::io::stdin().lock().read_line(&mut input).is_err() {
        return false;
    }
    matches!(input.trim(), "y" | "Y")
}

/// Reads a line from stdin, or `None` on end of input.
pub fn read_line(prompt: &str) -> Option<String> {
    print!("{prompt}");
    let _ = std::io::stdout().flush();
    let mut input = String::new();
    match std::io::stdin().lock().read_line(&mut input) {
        Ok(0) | Err(_) => None,
        Ok(_) => Some(input.trim().to_owned()),
    }
}

fn info_row(out: &mut String, name: &str, value: &str, indent: bool) {
    if value.is_empty() {
        return;
    }
    if indent {
        out.push('\t');
    }
    let sep = if name.len() < 7 { ":\t" } else { ":" };
    let _ = write!(out, "{COLOR_INFO_HEADER}{name}{sep}");
    if !indent {
        out.push('\t');
    }
    let _ = writeln!(out, "{COLOR_INFO_NAME}{value}{COLOR_RESET}");
}

fn info_list(out: &mut String, name: &str, values: &[String], indent: bool) {
    match values {
        [] => {}
        [one] => info_row(out, name, one, indent),
        _ => {
            if indent {
                out.push('\t');
            }
            let _ = writeln!(out, "{COLOR_INFO_HEADER}{name}:{COLOR_RESET}");
            for value in values {
                let _ = writeln!(out, "{COLOR_INFO_NAME}\t\t{value}{COLOR_RESET}");
            }
        }
    }
}

/// Formats an image's binary info like `BinaryInfo::dumpAppInfo`.
pub fn format_app_info(app: &AppInfo) -> String {
    let mut out = String::new();
    info_row(&mut out, "Name", &app.program_name, false);
    info_row(&mut out, "Description", &app.program_description, false);
    info_row(&mut out, "Version", &app.program_version, false);
    info_row(&mut out, "Board Name", &app.board_type, false);
    info_row(&mut out, "Device IP Addr", &app.device_ip_address, false);
    info_row(&mut out, "Agent IP Addr", &app.agent_ip_address, false);
    if app.agent_port != 0 {
        info_row(&mut out, "Agent Port", &app.agent_port.to_string(), false);
    }
    match app.client_ids.as_slice() {
        [] => {}
        [id] => info_row(&mut out, "Client ID", &id.to_string(), false),
        ids => {
            let _ = writeln!(out, "{COLOR_INFO_HEADER}Client ID List:{COLOR_RESET}");
            for id in ids {
                let _ = writeln!(out, "{COLOR_INFO_NAME}\t\t{id}{COLOR_RESET}");
            }
        }
    }
    info_row(&mut out, "Build Date", &app.program_build_date, false);
    info_row(&mut out, "URL", &app.program_url, false);
    info_row(&mut out, "SDK Version", &app.sdk_version, false);
    info_row(&mut out, "Boot 2 Name", &app.boot2_name, false);
    info_row(&mut out, "Binary Start", &format!("0x{:08x}", app.binary_start), false);
    if app.binary_end != 0 {
        info_row(&mut out, "Binary End", &format!("0x{:08x}", app.binary_end), false);
    }
    if app.bl_app_base != 0 {
        info_row(&mut out, "Next App Addr", &format!("0x{:08x}", app.bl_app_base), false);
    }
    info_list(&mut out, "Build Attrs", &app.build_attributes, false);
    info_list(&mut out, "Features", &app.program_features, false);
    for (group, values) in &app.named_feature_group_values {
        info_list(&mut out, group, values, false);
    }
    if !app.pins.is_empty() {
        let _ = writeln!(out, "{COLOR_INFO_HEADER}Pins:{COLOR_RESET}");
        for (pin, names) in &app.pins {
            info_list(&mut out, &pin.to_string(), names, true);
        }
    }
    info_row(&mut out, "Message", &app.message, false);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decode() {
        assert_eq!(decode_u32("0x10", u32::MAX), Some(16));
        assert_eq!(decode_u32("0xffffffff", u32::MAX), Some(u32::MAX));
        assert_eq!(decode_u32("4294967296", u32::MAX), None);
        assert_eq!(decode_u32("31", 31), Some(31));
        assert_eq!(decode_u32("32", 31), None);
        assert_eq!(decode_u32("", 31), None);
        assert_eq!(decode_u32("0x", 31), None);
        assert_eq!(decode_u32("-1", 31), None);
        assert_eq!(decode_u32("+1", 31), None);
    }

    #[test]
    fn dump() {
        let dump = hex_dump(0x1000_0000, b"Hello, world!\x00\x01\x02ABC");
        let lines: Vec<&str> = dump.lines().collect();
        assert_eq!(lines.len(), 2);
        assert!(lines[0].starts_with("0x10000000: 48 65 6C 6C 6F 2C 20 77  6F"));
        assert!(lines[0].ends_with("|  Hello, world!..."));
        assert!(lines[1].starts_with("0x10000010: 41 42 43 "));
        assert!(lines[1].ends_with("|  ABC"));
    }
}
