//! Board and flash chip database, ported from `rp2040_flash_itf/DeviceMap.cpp`.
//!
//! The database is `DeviceList.jsonc` (copied from `titan_firmware/tools`), embedded at build time like the C++
//! tools do. It maps flash unique IDs (the board serial numbers) to a friendly name and board type, and flash chip
//! JEDEC IDs to their capacity.

use std::sync::OnceLock;

use anyhow::{Context, Result, bail, ensure};
use serde_json::Value;

use crate::flash::MAX_FLASH_SIZE;

const EMBEDDED_DEVICE_LIST: &str = include_str!("../DeviceList.jsonc");

/// A board in the database (or a placeholder for an unknown serial number).
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct DeviceIdentifier {
    pub is_unknown: bool,
    pub name: String,
    pub board_type: String,
    pub serial_number: u64,
}

impl DeviceIdentifier {
    fn unknown(serial_number: u64) -> Self {
        DeviceIdentifier { is_unknown: true, name: "Unknown Board".into(), board_type: "unknown".into(), serial_number }
    }

    /// The serial number as printed in the database (first byte first).
    pub fn hex_serial(&self) -> String {
        hex_serial(self.serial_number)
    }
}

/// Formats a flash unique ID the way the database stores it: byte 0 (the LSB) first, upper case.
pub fn hex_serial(serial: u64) -> String {
    serial.to_le_bytes().iter().map(|b| format!("{b:02X}")).collect()
}

/// A flash chip in the database (or a placeholder for an unknown JEDEC ID).
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct FlashChipInfo {
    pub is_unknown: bool,
    pub name: String,
    pub id: u32,
    pub capacity: u32,
}

impl FlashChipInfo {
    /// The JEDEC ID as printed in the database (manufacturer byte first).
    pub fn hex_id(&self) -> String {
        self.id.to_le_bytes()[..3].iter().map(|b| format!("{b:02X}")).collect()
    }
}

pub struct DeviceMap {
    devices: Vec<DeviceIdentifier>,
    flash_chips: Vec<FlashChipInfo>,
}

impl DeviceMap {
    /// The embedded database, parsed once.
    pub fn get() -> &'static DeviceMap {
        static MAP: OnceLock<DeviceMap> = OnceLock::new();
        MAP.get_or_init(|| DeviceMap::parse(EMBEDDED_DEVICE_LIST).expect("embedded DeviceList.jsonc is invalid"))
    }

    pub fn parse(jsonc: &str) -> Result<Self> {
        let json: Value = serde_json::from_str(&strip_jsonc(jsonc)).context("parsing device list")?;
        let mut map = DeviceMap { devices: Vec::new(), flash_chips: Vec::new() };

        for entry in json["serialMap"].as_array().context("serialMap missing")? {
            let name = str_field(entry, "name")?;
            let board_type = str_field(entry, "boardType")?;
            let serial = str_field(entry, "serial")?;
            ensure!(serial.len() == 16, "invalid serial number length: {serial}");
            let serial_number = u64::from_le_bytes(parse_hex_bytes(&serial)?.try_into().unwrap());
            ensure!(
                map.devices.iter().all(|d| d.serial_number != serial_number),
                "duplicate serial number {serial} in device map"
            );
            map.devices.push(DeviceIdentifier { is_unknown: false, name, board_type, serial_number });
        }

        for entry in json["jedecIdMap"].as_array().context("jedecIdMap missing")? {
            let name = str_field(entry, "name")?;
            let id_str = str_field(entry, "id")?;
            ensure!(id_str.len() == 6, "invalid JEDEC ID length: {id_str}");
            let bytes = parse_hex_bytes(&id_str)?;
            let id = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], 0]);
            let capacity = entry["capacity"].as_u64().context("flash capacity missing")?;
            ensure!(capacity <= MAX_FLASH_SIZE as u64, "flash '{name}' is larger than the RP2040 can address");
            ensure!(map.flash_chips.iter().all(|c| c.id != id), "duplicate JEDEC ID {id_str} in device map");
            map.flash_chips.push(FlashChipInfo { is_unknown: false, name, id, capacity: capacity as u32 });
        }
        Ok(map)
    }

    pub fn lookup_serial(&self, serial_number: u64) -> DeviceIdentifier {
        self.devices
            .iter()
            .find(|d| d.serial_number == serial_number)
            .cloned()
            .unwrap_or_else(|| DeviceIdentifier::unknown(serial_number))
    }

    /// Looks up the JEDEC ID read from the flash chip (manufacturer in the lowest byte).
    pub fn lookup_flash_chip(&self, jedec_id: u32) -> FlashChipInfo {
        self.flash_chips.iter().find(|c| c.id == jedec_id).cloned().unwrap_or(FlashChipInfo {
            is_unknown: true,
            name: "Unknown Flash Chip".into(),
            id: jedec_id,
            capacity: MAX_FLASH_SIZE,
        })
    }

    /// Finds the serial number of the board called `name`.
    pub fn lookup_serial_by_name(&self, name: &str) -> Result<u64> {
        let mut matches = self.devices.iter().filter(|d| d.name == name);
        let Some(first) = matches.next() else {
            bail!("device database does not have any devices with name: {name}");
        };
        ensure!(matches.next().is_none(), "multiple devices in database with name: {name}");
        Ok(first.serial_number)
    }

    pub fn devices(&self) -> &[DeviceIdentifier] {
        &self.devices
    }
}

fn str_field(entry: &Value, name: &str) -> Result<String> {
    Ok(entry[name].as_str().with_context(|| format!("field {name} missing in {entry}"))?.to_owned())
}

fn parse_hex_bytes(s: &str) -> Result<Vec<u8>> {
    ensure!(s.len().is_multiple_of(2), "odd length hex string {s}");
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).with_context(|| format!("invalid hex string {s}")))
        .collect()
}

/// Turns JSON with comments (and trailing commas) into plain JSON.
fn strip_jsonc(input: &str) -> String {
    let mut out = String::with_capacity(input.len());
    let mut chars = input.chars().peekable();
    let mut in_string = false;
    while let Some(c) = chars.next() {
        if in_string {
            out.push(c);
            match c {
                '\\' => out.extend(chars.next()),
                '"' => in_string = false,
                _ => {}
            }
            continue;
        }
        match (c, chars.peek()) {
            ('"', _) => {
                in_string = true;
                out.push(c);
            }
            ('/', Some('/')) => {
                for c in chars.by_ref() {
                    if c == '\n' {
                        out.push('\n');
                        break;
                    }
                }
            }
            ('/', Some('*')) => {
                chars.next();
                let mut prev = ' ';
                for c in chars.by_ref() {
                    if prev == '*' && c == '/' {
                        break;
                    }
                    prev = c;
                }
            }
            _ => out.push(c),
        }
    }

    // Drop commas directly followed (ignoring whitespace) by a closing bracket
    let mut cleaned = String::with_capacity(out.len());
    let bytes: Vec<char> = out.chars().collect();
    let mut in_string = false;
    for (i, &c) in bytes.iter().enumerate() {
        if c == '"' && (i == 0 || bytes[i - 1] != '\\') {
            in_string = !in_string;
        }
        if c == ',' && !in_string {
            let next = bytes[i + 1..].iter().find(|c| !c.is_whitespace());
            if matches!(next, Some(']') | Some('}')) {
                continue;
            }
        }
        cleaned.push(c);
    }
    cleaned
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn embedded_map_parses() {
        let map = DeviceMap::get();
        assert!(!map.devices().is_empty());
        let serial = map.lookup_serial_by_name("Talos IVC Board").unwrap();
        assert_eq!(hex_serial(serial), "E46214B0630D2A32");
        assert_eq!(serial & 0xFF, 0xE4);
        let dev = map.lookup_serial(serial);
        assert_eq!(dev.board_type, "talos/ivc_board");
        assert!(map.lookup_serial(1).is_unknown);

        // W25Q16JV, read from the chip as EF 40 15
        let chip = map.lookup_flash_chip(0x15_40EF);
        assert_eq!(chip.capacity, 2 * 1024 * 1024);
        assert_eq!(chip.hex_id(), "EF4015");
        assert!(map.lookup_flash_chip(0x123456).is_unknown);
    }

    #[test]
    fn jsonc_stripping() {
        let input = r#"{ "a": "x // not a comment", /* block */ "b": [1, 2,], // trailing
            "c": { "d": "\"," , }, }"#;
        let v: Value = serde_json::from_str(&strip_jsonc(input)).unwrap();
        assert_eq!(v["a"], "x // not a comment");
        assert_eq!(v["b"].as_array().unwrap().len(), 2);
        assert_eq!(v["c"]["d"], "\",");
    }
}
