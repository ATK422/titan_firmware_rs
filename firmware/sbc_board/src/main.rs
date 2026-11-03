//! Talos SBC (smart battery charger) board: Rust port of `SBCBoard`.
//!
//! The C firmware is a skeleton: it brings up safety, the status LED, the board I2C bus and CAN, and nothing
//! else. The BQ25730 charger, USB-PD controller and screen drivers were never written (`bq25730.c` is empty).
//! The C `main` called `canbus_init` rather than `transport_can_init` and never started micro-ROS, so it had no
//! ROS node. The Rust board always runs the titan link, but it stays idle (safety uninitialized, no heartbeat or
//! status published) unless `titan_bridge --board sbc` is started for it.
//!
//! Pins (`boards/talos/sbc_board.h`): board I2C0 on GPIO 28 (SDA) and 29 (SCL). The header also names audio
//! (GPIO 0-2), screen (GPIO 4-6, 9-12) and charger (GPIO 23-27) pins that the C firmware never touched.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::i2c::{self, I2c};
use embassy_rp::peripherals::{DMA_CH1, DMA_CH2, I2C0};
use embassy_rp::{bind_interrupts, dma};

use canmore::client_ids::CLIENT_ID_SBC_BOARD;
use titan_rp2040::board::{self, BoardConfig, Bus};
use titan_rp2040::safety::STANDARD_FAULT_NAMES;

bind_interrupts!(struct Irqs {
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
    // DMA_CH1/2 belong to the CAN block's SPI
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH1>, dma::InterruptHandler<DMA_CH2>;
});

/// Fault names by ID. The C `safety_interface.h` put `FAULT_BQ25730_ERROR` at ID 3, which the Rust safety core
/// reserves for `FAULT_ROS_ERROR`, so it moves to the first board fault ID. Nothing raises it yet.
static FAULT_NAMES: [&str; 6] = [
    STANDARD_FAULT_NAMES[0],
    STANDARD_FAULT_NAMES[1],
    STANDARD_FAULT_NAMES[2],
    STANDARD_FAULT_NAMES[3],
    STANDARD_FAULT_NAMES[4],
    "FAULT_BQ25730_ERROR",
];

/// Board I2C clock (`async_i2c_init(..., 200000, ...)`).
const I2C_FREQUENCY: u32 = 200_000;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    // TODO (C FIXME): canbus_utility_frame_register_cb()? Utility frames arrive on canbus::UTIL_RX if needed.
    board::start(
        spawner,
        titan_rp2040::can_block!(p, Irqs),
        BoardConfig {
            name: "talos/sbc_board",
            client_id: CLIENT_ID_SBC_BOARD,
            bus: Bus::Internal,
            version: titan_rp2040::version!(),
            fault_names: &FAULT_NAMES,
            ..BoardConfig::DEFAULT
        },
    );

    // Internal pull-ups enabled, as async_i2c_init did
    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = I2C_FREQUENCY;
    let _i2c = I2c::new_async(p.I2C0, p.PIN_29, p.PIN_28, Irqs, i2c_config);
    // TODO: Initialize bq25730 (raise FAULT_BQ25730_ERROR on failure)
    // TODO: Initialize USB-PD chip
    // TODO: Initialize screen
    // The bus stays configured after `main` returns; hand `_i2c` to those drivers' tasks once they exist.
}
