//! Talos smart battery housing MCU (`mk2/sbh_mcu`): Rust port of `talos/SmartBattery`.
//!
//! The board lives in each battery pack. It watches the pack's BQ40Z80 gas gauge, shows the pack state on a small
//! OLED (woken with a magnet on the reed switch), drives the pack's SOC LEDs, keeps itself powered only while the
//! pack is in use or the display is on, and reports to ROS over the external (classic) CAN bus.
//!
//! * Core 1 polls the gauge over SMBus on PIO ([`gauge`], [`bq40z80`], [`pio_smbus`])
//! * Core 0 runs the CAN link, safety and status LED ([`titan_rp2040::board`]), the display and power latch
//!   ([`background`]) and the ROS traffic plus the SHT41 ([`ros`])
//!
//! The CANmore client ID is the serial number programmed into the last flash sector (see
//! `smart_battery_core::uf2` and the `smart_battery_id` tool), matching the gauge's serial number; run the bridge
//! with `titan_bridge --board smart_battery --client-id <serial>`.
//!
//! Pins (`boards/mk2/sbh_mcu.h`): board I2C (OLED, SHT41) on GPIO 0 (SDA) and 1 (SCL); SOC LEDs red/yellow/green on
//! GPIO 2/3/4; gauge SMBus SCL on GPIO 6 and SDA on GPIO 7 (swapped relative to the I2C pin mux, hence PIO); gauge
//! wake on GPIO 8; power latch on GPIO 9; reed switch on GPIO 10.

#![no_std]
#![no_main]

mod background;
mod bq40z80;
mod debug_cmd;
mod fault;
mod gauge;
mod pio_smbus;
mod power;
mod ros;
mod ssd1306;

use defmt_rtt as _;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::{Executor, Spawner};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::multicore::{Stack, spawn_core1};
use embassy_rp::peripherals::{DMA_CH1, DMA_CH2, I2C0, PIO0};
use embassy_rp::pio::{self, Pio};
use embassy_rp::{bind_interrupts, dma, i2c};
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;

use smart_battery_core::uf2;
use titan_rp2040::board::{self, BoardConfig, Bus};
use titan_rp2040::drivers::sht41::Sht41;

use crate::background::I2cBus;
use crate::bq40z80::Bq40z80;
use crate::pio_smbus::{PioSmbus, SmbusProgram};

bind_interrupts!(struct Irqs {
    // DMA_CH1/2 belong to the CAN block's SPI
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH1>, dma::InterruptHandler<DMA_CH2>;
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
    // Enabled on core 1, which owns the gauge SMBus
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

/// The board I2C bus (`async_i2c_init(PERIPH_SDA_PIN, PERIPH_SCL_PIN, -1, -1, 400000, 20)`).
static I2C_BUS: OnceLock<I2cBus> = OnceLock::new();
const I2C_FREQUENCY: u32 = 400_000;

static mut CORE1_STACK: Stack<8192> = Stack::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    // Before anything else: the regulator may be running on its capacitors
    let power = power::latch(p.PIN_9);

    let switch = Input::new(p.PIN_10, Pull::Down);
    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = I2C_FREQUENCY;
    let bus = I2C_BUS.get_or_init(|| Mutex::new(i2c::I2c::new_async(p.I2C0, p.PIN_1, p.PIN_0, Irqs, i2c_config)));

    let serial = power::board_serial();
    if !uf2::valid_id(serial) {
        background::show_early_message(I2cDevice::new(bus), "No Ser# Err", "").await;
        panic!("Unprogrammed serial number");
    }
    defmt::info!("Board serial number {}", serial);

    board::start(
        spawner,
        titan_rp2040::can_block!(p, Irqs),
        BoardConfig {
            name: "mk2/sbh_mcu",
            client_id: serial,
            bus: Bus::External,
            version: titan_rp2040::version!(),
            fault_names: &fault::FAULT_NAMES,
            // Nothing to disable on kill: the pack's FETs are the gauge's business, and the status LED follows the
            // kill state on its own (`safety_handle_kill` only set the LED)
            on_board_message: ros::handle_board_message,
            ..BoardConfig::DEFAULT
        },
    );

    // The gauge gets core 1 to itself, like `core1_init`
    let gauge_pins = gauge::Pins {
        leds: [Output::new(p.PIN_2, Level::Low), Output::new(p.PIN_3, Level::Low), Output::new(p.PIN_4, Level::Low)],
        wake: Output::new(p.PIN_8, Level::Low),
    };
    let (pio0, sda, scl) = (p.PIO0, p.PIN_7, p.PIN_6);
    spawner.spawn(gauge::core1_watchdog_task().unwrap());
    // SAFETY: the only reference to the stack, taken once
    let stack = unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) };
    spawn_core1(p.CORE1, stack, move || {
        let executor: &'static mut Executor = cortex_m::singleton!(: Executor = Executor::new()).unwrap();
        executor.run(|spawner| {
            // Created on core 1 so the PIO interrupt is enabled there
            let Pio { mut common, sm0, irq0, .. } = Pio::new(pio0, Irqs);
            let program = SmbusProgram::new(&mut common);
            let smbus = PioSmbus::new(&mut common, sm0, irq0, sda, scl, &program);
            spawner.spawn(gauge::gauge_task(Bq40z80::new(smbus), gauge_pins, serial).unwrap());
        })
    });

    spawner.spawn(background::background_task(I2cDevice::new(bus), switch, power).unwrap());
    spawner.spawn(ros::sht41_task(Sht41::new(I2cDevice::new(bus))).unwrap());
    spawner.spawn(ros::battery_status_task().unwrap());
    spawner.spawn(ros::temp_humidity_task().unwrap());
}
