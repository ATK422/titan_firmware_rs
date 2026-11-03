//! Talos camera cage breakout board: depth sensor, cage temperature/humidity, the status LED strip, fans and the
//! Orin power switch.
//!
//! Rust port of `talos/CameraCageBB`. ROS topics (served by `titan_bridge --board camera_cage`):
//! * `state/depth/raw` (`riptide_msgs2/Depth`, sensor data QoS, published): depth as each new reading arrives
//! * `state/depth/temp` (`std_msgs/Float32`, sensor data QoS, published): water temperature every second
//! * `state/temp/cameracage`, `state/humidity/cameracage` (`std_msgs/Float32`, best effort, published): SHT41
//! * `state/leak` (`std_msgs/Bool`, published): advertised but never published, as in C
//! * `command/led` (`riptide_msgs2/LedCommand`, subscribed): status strip colour and mode
//! * `state/physkill_notify` (`std_msgs/Bool`, subscribed): flashes the strip when the kill switch moves
//! * `command/electrical` (`riptide_msgs2/ElectricalCommand`, subscribed): computer power cycle, fans, strip
//!   enable, depth re-zeroing
//! * plus the standard `state/firmware`, `state/kill` and heartbeat. The C board published its heartbeat on
//!   `state/fw_heartbeat`; the bridge publishes every board's on `heartbeat` until it can be set per board.
//!
//! Pins (`boards/mk2/camera_cage_bb.h`): fan switch GPIO 0, leak sense GPIO 1, Orin power switch GPIO 6, status
//! strip data GPIO 9, I2C0 on GPIO 28 (SDA) / 29 (SCL) with the MS5837-02BA and the SHT41.

#![no_std]
#![no_main]

mod cage;

use defmt_rtt as _;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, DMA_CH2, I2C0, PIO0};
use embassy_rp::pio::Pio;
use embassy_rp::pio_programs::ws2812::{PioWs2812, PioWs2812Program};
use embassy_rp::{bind_interrupts, dma, i2c, pio};
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;

use canmore::client_ids::CLIENT_ID_CAMERA_CAGE_BB;
use titan_rp2040::board::{self, BoardConfig, Bus};
use titan_rp2040::drivers::depth::{Model, Ms5837};
use titan_rp2040::drivers::sht41::Sht41;

bind_interrupts!(struct Irqs {
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    // DMA_CH0 feeds the status strip; DMA_CH1/2 belong to the CAN block's SPI
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>, dma::InterruptHandler<DMA_CH2>;
});

/// The board I2C bus, shared by the depth sensor and the SHT41 (`async_i2c_init(.., 200000, 10)`).
static I2C_BUS: OnceLock<cage::I2cBus> = OnceLock::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    board::start(
        spawner,
        titan_rp2040::can_block!(p, Irqs),
        BoardConfig {
            name: "mk2/camera_cage_bb",
            client_id: CLIENT_ID_CAMERA_CAGE_BB,
            bus: Bus::Internal,
            version: titan_rp2040::version!(),
            fault_names: &cage::FAULT_NAMES,
            // No actuators to disable; the status LED follows the kill state on its own
            on_board_message: cage::handle_board_message,
            ..BoardConfig::DEFAULT
        },
    );

    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = 200_000;
    let bus = I2C_BUS.get_or_init(|| Mutex::new(i2c::I2c::new_async(p.I2C0, p.PIN_29, p.PIN_28, Irqs, i2c_config)));
    spawner.spawn(cage::depth_task(Ms5837::new(I2cDevice::new(bus), Model::Ms5837_02ba)).unwrap());
    spawner.spawn(cage::sht41_task(Sht41::new(I2cDevice::new(bus))).unwrap());

    let Pio { mut common, sm0, .. } = Pio::new(p.PIO0, Irqs);
    let program = PioWs2812Program::new(&mut common);
    let leds = PioWs2812::new(&mut common, sm0, p.DMA_CH0, Irqs, p.PIN_9, &program);
    cage::STRIP.clear();
    spawner.spawn(cage::strip_task(leds).unwrap());

    // Fans on and the Orin powered from boot, as in C
    let fan = Output::new(p.PIN_0, Level::High);
    let orin = Output::new(p.PIN_6, Level::High);
    spawner.spawn(cage::power_task(fan, orin).unwrap());

    // Pulled down to prevent false positives. The C leak publisher was commented out, so nothing reads it yet.
    let leak_sense = Input::new(p.PIN_1, Pull::Down);
    spawner.spawn(cage::ros_task(leak_sense).unwrap());
}
