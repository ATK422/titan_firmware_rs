//! Fan switch and the power stage commands from the bridge.
//!
//! The C boards turned the fan on at boot and only touched it again for `ElectricalCommand::{ENABLE,DISABLE}_FANS`.
//! The fan tachometer pin is unused, as in C.

use core::cell::RefCell;

use embassy_rp::gpio::{Level, Output};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use power_core::electrical::{ElectricalCommand, FAN_ON_AT_BOOT};
use titan_link::power;

use crate::kill;

static FAN: Mutex<CriticalSectionRawMutex, RefCell<Option<Output<'static>>>> = Mutex::new(RefCell::new(None));

/// The level to create the fan output with.
pub const FAN_BOOT_LEVEL: Level = if FAN_ON_AT_BOOT { Level::High } else { Level::Low };

/// Hands the fan switch output (created at [`FAN_BOOT_LEVEL`]) to [`handle_power_message`].
pub fn init_fan(pin: Output<'static>) {
    FAN.lock(|f| *f.borrow_mut() = Some(pin));
}

fn set_fan(on: bool) {
    FAN.lock(|f| {
        if let Some(pin) = f.borrow_mut().as_mut() {
            pin.set_level(Level::from(on));
        }
    });
}

/// Handles a power stage message from the bridge. Short enough to run in the link task, like the C executor
/// callbacks it replaces.
pub fn handle_power_message(msg: power::HostToDevice, bad_command_fault: u8) {
    match msg {
        power::HostToDevice::SoftwareKill(cmd) => kill::handle_software_kill(&cmd, bad_command_fault),
        power::HostToDevice::ElectricalCommand(command) => {
            match ElectricalCommand::from_u8(command).and_then(ElectricalCommand::fan_switch) {
                Some(on) => {
                    defmt::info!("Fans {}", if on { "enabled" } else { "disabled" });
                    set_fan(on);
                }
                None => defmt::debug!("Ignoring electrical command {}", command),
            }
        }
    }
}
