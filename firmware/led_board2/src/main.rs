//! Talos LED board, second revision firmware: Rust port of `talos/LEDBoard2`.
//!
//! The C firmware is a rewrite of the controller bring-up that was still in progress: it puts both ST L99LD20
//! controllers into active mode with PWM dimming at the normal peak current and keeps their watchdogs fed, but has
//! no colour control yet. Its ROS node (`titan_bridge --board led2`) has only the standard `heartbeat`,
//! `state/firmware` and `state/kill`. Its CANmore remote commands are in [`commands`].
//!
//! Pins (`boards/talos/led_board.h`, same board as `led_board`): controller SPI1 on GPIO 10 (SCK), 11 (MOSI),
//! 12 (MISO) with chip selects on GPIO 9 (LEDC1) and 6 (LEDC2), DIN1/DIN2 on GPIO 1/0 (left as inputs), the
//! controllers' 204.8 kHz PWMCLK on GPIO 7 and the module thermistor on GPIO 26 (ADC 0).

#![no_std]
#![no_main]

mod commands;
mod ledc;

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::adc::{self, Adc};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH1, DMA_CH2};
use embassy_rp::spi::{self, Spi};
use embassy_rp::{bind_interrupts, dma};
use embassy_time::Delay;

use canmore::client_ids::CLIENT_ID_LED_BOARD;
use ledc_core::driver::Ledc;
use titan_rp2040::board::{self, BoardConfig, Bus};

bind_interrupts!(struct Irqs {
    // DMA_CH1/2 belong to the CAN block's SPI
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH1>, dma::InterruptHandler<DMA_CH2>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    board::start(
        spawner,
        titan_rp2040::can_block!(p, Irqs),
        BoardConfig {
            // Same PICO_BOARD and client ID as LEDBoard; only the node name differs (led_board2)
            name: "talos/led_board",
            client_id: CLIENT_ID_LED_BOARD,
            bus: Bus::Internal,
            version: titan_rp2040::version!(),
            remote_commands: &commands::COMMANDS,
            // safety_interface.h defines only the standard faults
            ..BoardConfig::DEFAULT
        },
    );

    let mut spi_config = spi::Config::default();
    spi_config.frequency = 4_000_000;
    let spi = Spi::new_blocking(p.SPI1, p.PIN_10, p.PIN_11, p.PIN_12, spi_config);
    let ledc = Ledc::new(spi, Output::new(p.PIN_9, Level::High), Output::new(p.PIN_6, Level::High), Delay);

    let hardware = ledc::Hardware {
        ledc,
        // gpio_init only: inputs, keeping the pads' reset pull-downs
        _din: [Input::new(p.PIN_1, Pull::Down), Input::new(p.PIN_0, Pull::Down)],
        _pwm_clock: ledc::start_pwm_clock(p.PWM_SLICE3, p.PIN_7),
        adc: Adc::new_blocking(p.ADC, adc::Config::default()),
        thermistor: adc::Channel::new_pin(p.PIN_26, Pull::None),
    };
    spawner.spawn(ledc::ledc_task(hardware).unwrap());
}
