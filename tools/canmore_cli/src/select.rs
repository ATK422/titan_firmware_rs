//! Device selection, ported from `canmore_cli/src/discovery.cpp`.
//!
//! The C++ tool redraws a live menu navigated with the arrow keys. This port lists the devices and asks for a
//! number instead (Enter refreshes the list), which needs no raw terminal handling.

use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;

use titan_tools::canmore::{CanmoreDevice, Discovery, Link};
use titan_tools::device_map::DeviceMap;
use titan_tools::ui::{COLOR_ERROR, COLOR_HEADER, COLOR_NAME, COLOR_RESET, read_line, render_field, render_header};

/// Caches flash IDs, since reading one costs a few register accesses.
#[derive(Default)]
pub struct FlashIdCache(HashMap<(Link, u8), u64>);

impl FlashIdCache {
    pub fn get(&mut self, dev: &mut CanmoreDevice) -> Option<u64> {
        let key = (dev.link.clone(), dev.client_id);
        if let Some(&id) = self.0.get(&key) {
            return Some(id);
        }
        let id = dev.read_flash_id().ok().filter(|&id| id != 0)?;
        self.0.insert(key, id);
        Some(id)
    }
}

pub fn all_devices(sources: &[Arc<Discovery>]) -> Vec<CanmoreDevice> {
    let mut devices: Vec<CanmoreDevice> = sources.iter().flat_map(|s| s.devices()).collect();
    devices.sort_by_key(|a| (a.link.to_string(), a.client_id));
    devices
}

fn render_device(index: usize, dev: &mut CanmoreDevice, cache: &mut FlashIdCache) {
    let flash_id = if dev.mode.supports_flash_id() { cache.get(dev) } else { None };
    let descr = DeviceMap::get().lookup_serial(flash_id.unwrap_or(0));
    println!("{COLOR_NAME} [{index}] {}{COLOR_RESET}", descr.name);
    if !descr.is_unknown {
        render_field("Board Type", &descr.board_type);
    } else if flash_id.is_some() {
        render_field("Unique ID", &descr.hex_serial());
    } else if !dev.mode.supports_flash_id() {
        render_field("Unique ID", "<ID Retrieval Not Supported>");
    } else {
        render_field("Unique ID", "<Error Retrieving ID>");
    }
    render_field("Interface", &dev.interface_description());
    render_field("Mode", &dev.mode.name());
    let error = if dev.heartbeat.error { format!("{COLOR_ERROR}Fault Present") } else { "Normal".into() };
    render_field("Error State", &error);
    if dev.heartbeat.term_valid {
        render_field("Term Resistor", if dev.heartbeat.term_enabled { "On" } else { "Off" });
    }
    println!();
}

/// Lists the discovered devices and asks which one to connect to. `None` quits.
pub fn select_device(sources: &[Arc<Discovery>], cache: &mut FlashIdCache) -> Option<CanmoreDevice> {
    // Give the discovery threads a moment to hear the first heartbeats
    std::thread::sleep(Duration::from_millis(600));
    loop {
        let mut devices = all_devices(sources);
        println!();
        render_header("Select Device");
        if devices.is_empty() {
            println!("{COLOR_HEADER}   No devices found. Discovering...{COLOR_RESET}\n");
        }
        for (i, dev) in devices.iter_mut().enumerate() {
            render_device(i + 1, dev, cache);
        }
        let input = read_line("Select a device [number, Enter to refresh, q to quit]: ")?;
        match input.as_str() {
            "" => continue,
            "q" | "quit" | "exit" => return None,
            n => match n.parse::<usize>() {
                Ok(i) if (1..=devices.len()).contains(&i) => return Some(devices.swap_remove(i - 1)),
                _ => println!("Invalid selection: {n}"),
            },
        }
    }
}

/// Waits until a device with flash ID `serial` shows up.
pub fn wait_for_target(sources: &[Arc<Discovery>], serial: u64) -> CanmoreDevice {
    let mut cache = FlashIdCache::default();
    loop {
        for mut dev in all_devices(sources) {
            if dev.mode.supports_flash_id() && cache.get(&mut dev) == Some(serial) {
                return dev;
            }
        }
        std::thread::sleep(Duration::from_secs(1));
    }
}
