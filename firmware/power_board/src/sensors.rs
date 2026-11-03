//! The board I2C bus (I2C0) and the sensors every power board has on it: the SHT41 temperature/humidity sensor and
//! the MCP3426 rail ADC at 0x68. The PoHalf board also has a depth sensor on it.
//!
//! The C drivers queued their transfers on `driver_async_i2c`. Here the bus is an async `embassy_rp` I2C behind an
//! `embassy_embedded_hal` shared bus mutex, and each driver gets its own [`SharedI2c`] device.

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_rp::i2c::{Async, I2c};
use embassy_rp::peripherals::I2C0;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use power_core::electrical::{RailMap, RailVoltages};
use static_cell::StaticCell;
use titan_link::power;
use titan_rp2040::drivers::Latest;
use titan_rp2040::drivers::mcp3426::{self, Channel, Mcp3426};
use titan_rp2040::drivers::sht41::{self, Sht41};
use titan_rp2040::safety;

use crate::{Wrap, send};

/// The board I2C bus.
pub type I2cBus = I2c<'static, I2C0, Async>;
/// One driver's handle on the shared board bus.
pub type SharedI2c = I2cDevice<'static, CriticalSectionRawMutex, I2cBus>;

/// I2C clock. The C boards asked for 2 MHz, which the RP2040 I2C block (Fast-mode Plus, 1 MHz at most) can't do
/// within spec and `embassy_rp` refuses; 1 MHz is the closest valid rate. The PoHalf board uses 200 kHz.
pub const TALOS_I2C_FREQUENCY: u32 = 1_000_000;
/// Address of the rail ADC.
pub const MCP3426_ADDRESS: u8 = 0x68;

static BUS: StaticCell<Mutex<CriticalSectionRawMutex, I2cBus>> = StaticCell::new();

/// Puts the bus behind the shared bus mutex. Call once; hand [`device`]s to the drivers.
pub fn share_bus(i2c: I2cBus) -> &'static Mutex<CriticalSectionRawMutex, I2cBus> {
    BUS.init(Mutex::new(i2c))
}

/// A driver's handle on the shared bus.
pub fn device(bus: &'static Mutex<CriticalSectionRawMutex, I2cBus>) -> SharedI2c {
    I2cDevice::new(bus)
}

/// Latest SHT41 reading (`sht41_read_temp`, `sht41_read_rh`, `sht41_is_valid`).
pub static SHT41: Latest<sht41::Reading> = Latest::new(sht41::VALID_FOR);
/// Latest voltage on each MCP3426 channel selection (`mcp3426_read_voltage`).
pub static RAIL_ADC: mcp3426::Voltages = mcp3426::Voltages::new();

/// Polls the SHT41, raising `fault` (the board's `FAULT_SHT41_ERROR`) on errors.
#[embassy_executor::task]
pub async fn sht41_task(mut sensor: Sht41<SharedI2c>, fault: u8) -> ! {
    sensor.run(&SHT41, |err| safety::raise_fault(fault, err.code())).await
}

/// Cycles the MCP3426 channels, raising `fault` (the board's `FAULT_ADC_ERROR`) on errors.
#[embassy_executor::task]
pub async fn mcp3426_task(mut adc: Mcp3426<SharedI2c>, fault: u8) -> ! {
    adc.run(&RAIL_ADC, |err| safety::raise_fault(fault, err.code())).await
}

/// The rail voltages, NaN for channels without a reading.
pub fn rail_voltages(map: &RailMap) -> RailVoltages {
    map.convert(Channel::ALL.map(|channel| RAIL_ADC.get(channel)))
}

/// `state/electrical` contents: the rails from the MCP3426 plus the battery measurements the board has.
pub fn electrical_readings(
    map: &RailMap,
    port_voltage: f32,
    stbd_voltage: f32,
    port_powering: bool,
    stbd_powering: bool,
) -> power::ElectricalReadings {
    let rails = rail_voltages(map);
    power::ElectricalReadings {
        port_voltage,
        stbd_voltage,
        balanced_voltage: rails.balanced,
        twelve_volt_voltage: rails.twelve_volt,
        five_volt_voltage: rails.five_volt,
        three_volt_voltage: rails.three_volt,
        stbd_powering,
        port_powering,
    }
}

/// Publishes a new SHT41 reading if one arrived since the last call and is still valid
/// (`ros_update_temp_humidity_publisher` on `sht41_temp_rh_set_on_read`).
pub fn send_new_temp_humidity(wrap: Wrap) {
    if SHT41.take_new().is_none() {
        return;
    }
    if let Some(reading) = SHT41.get() {
        send(
            wrap,
            power::DeviceToHost::TempHumidity {
                temperature_c: reading.temperature_c,
                humidity_rh: reading.humidity_rh,
            },
        );
    }
}
