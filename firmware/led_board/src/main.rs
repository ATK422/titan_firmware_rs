//! Talos LED board: drives the aluminium LED module (RGBW, four buck converters in two ST L99LD20 controllers).
//!
//! Rust port of `talos/LEDBoard`. ROS topics (served by `titan_bridge --board led`):
//! * `command/led` (`riptide_msgs2/LedCommand`, subscribed): colour and pattern, for commands targeting `TARGET_ALU`
//! * `state/physkill_notify` (`std_msgs/Bool`, subscribed): flashes red when the kill switch is removed, blue when
//!   it is inserted
//! * `state/depth/raw` (`riptide_msgs2/Depth`, subscribed, best effort): full brightness only under water
//! * plus the standard `heartbeat`, `state/firmware` and `state/kill`
//!
//! Pins (`boards/talos/led_board.h`): controller SPI1 on GPIO 10 (SCK), 11 (MOSI), 12 (MISO) with chip selects on
//! GPIO 9 (LEDC1) and 6 (LEDC2), DIN1/DIN2 on GPIO 1/0, the controllers' 204.8 kHz PWMCLK on GPIO 7 and the module
//! thermistor on GPIO 26 (ADC 0).

#![no_std]
#![no_main]

mod leds;

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::adc::{self, Adc};
use embassy_rp::gpio::{Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH1, DMA_CH2};
use embassy_rp::spi::{self, Spi};
use embassy_rp::{bind_interrupts, dma};
use embassy_time::Delay;

use canmore::client_ids::CLIENT_ID_LED_BOARD;
use ledc_core::driver::Ledc;
use titan_rp2040::board::{self, BoardConfig, Bus};
use titan_rp2040::safety::STANDARD_FAULT_NAMES;

bind_interrupts!(struct Irqs {
    // DMA_CH1/2 belong to the CAN block's SPI
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH1>, dma::InterruptHandler<DMA_CH2>;
});

/// Fault names by ID, matching `safety_interface.h`. Only `FAULT_LED_OVERTEMP` is ever raised, as in C.
static FAULT_NAMES: [&str; 8] = [
    STANDARD_FAULT_NAMES[0],
    STANDARD_FAULT_NAMES[1],
    STANDARD_FAULT_NAMES[2],
    STANDARD_FAULT_NAMES[3],
    STANDARD_FAULT_NAMES[4],
    "FAULT_LED_TEMP_WARN",
    "FAULT_LED_OVERTEMP",
    "FAULT_BUCK_OVERTEMP",
];
const _: () = assert!(leds::FAULT_LED_OVERTEMP == 6);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    board::start(
        spawner,
        titan_rp2040::can_block!(p, Irqs),
        BoardConfig {
            name: "talos/led_board",
            client_id: CLIENT_ID_LED_BOARD,
            bus: Bus::Internal,
            version: titan_rp2040::version!(),
            fault_names: &FAULT_NAMES,
            // Like the C safety_handle_kill, killing only changes the status LED, which follows the kill state on
            // its own; the module stays under ROS control
            on_board_message: leds::handle_board_message,
            ..BoardConfig::DEFAULT
        },
    );

    let mut spi_config = spi::Config::default();
    spi_config.frequency = 4_000_000;
    let spi = Spi::new_blocking(p.SPI1, p.PIN_10, p.PIN_11, p.PIN_12, spi_config);
    let ledc = Ledc::new(spi, Output::new(p.PIN_9, Level::High), Output::new(p.PIN_6, Level::High), Delay);

    let hardware = leds::Hardware {
        ledc,
        // The bucks are dimmed by the controllers' PWM generators, so DIN is only held low
        _din: [Output::new(p.PIN_1, Level::Low), Output::new(p.PIN_0, Level::Low)],
        _pwm_clock: leds::start_pwm_clock(p.PWM_SLICE3, p.PIN_7),
        adc: Adc::new_blocking(p.ADC, adc::Config::default()),
        thermistor: adc::Channel::new_pin(p.PIN_26, Pull::None),
    };
    spawner.spawn(leds::led_task(hardware).unwrap());
}
