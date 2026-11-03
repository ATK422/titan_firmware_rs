//! MCU power latch and board serial number.
//!
//! The MCU runs from a regulator that only stays on while GPIO 9 (`PWR_CTRL_PIN`) is high, or while the reed switch
//! is held. The C firmware latched it in an assembly `__pre_init` hook right after the SDK reset the IO bank, so the
//! regulator capacitors only had to bridge a few microseconds.
//!
//! `embassy_rp::init` resets the IO bank inside its clock setup, so the earliest point Rust can latch the pin is
//! right after it returns, which leaves the pin released for the clock bring-up (crystal and PLL start, about
//! 2 ms). That only matters when nothing else holds the power on (the C CAN bootloader used to, but Rust boards
//! don't use it, and on a fresh start the reed switch is held). Verify on hardware that a watchdog reset with the
//! switch released doesn't drop the board; if it does, the latch needs a hook inside embassy's init.

use embassy_rp::Peri;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::pac;
use embassy_rp::pac::io::vals::{Oeover, Outover};
use embassy_rp::peripherals::PIN_9;

/// GPIO number of the power latch (`PWR_CTRL_PIN`).
const PWR_CTRL_PIN: usize = 9;

/// XIP address of the board serial number in the last flash sector (written by `id_gen/id.uf2`).
const SERIAL_ADDRESS: usize = smart_battery_core::uf2::ID_ADDRESS as usize;

/// Latches the power on. Call first thing after `embassy_rp::init`.
pub fn latch(pin: Peri<'static, PIN_9>) -> Output<'static> {
    // Force the pin high through the IO bank overrides immediately, like `early_pin_set.S`
    pac::IO_BANK0.gpio(PWR_CTRL_PIN).ctrl().write(|w| {
        w.set_outover(Outover::HIGH);
        w.set_oeover(Oeover::ENABLE);
    });
    // Have SIO drive it high too, so handing the pin to the GPIO driver (which clears the overrides) is seamless
    pac::SIO.gpio_out(0).value_set().write_value(1 << PWR_CTRL_PIN);
    pac::SIO.gpio_oe(0).value_set().write_value(1 << PWR_CTRL_PIN);
    Output::new(pin, Level::High)
}

/// The serial number programmed into this board, which is also its CANmore client ID and must match the gauge
/// serial number. 0 or 0xFF means unprogrammed.
pub fn board_serial() -> u8 {
    // SAFETY: flash is memory mapped (XIP) and this sector is never written by the firmware
    unsafe { core::ptr::read_volatile(SERIAL_ADDRESS as *const u8) }
}
