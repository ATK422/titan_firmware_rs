//! Template board: Rust port of `examples/micro_ros_template`.
//!
//! Starting point for a new board, like the C `micro_ros_template`: copy this crate, set the board name, client ID
//! and bus, add the board's messages to `titan_link` and its topics to `titan_bridge`, then add board tasks after
//! `board::start`.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::peripherals::{DMA_CH1, DMA_CH2};
use embassy_rp::{bind_interrupts, dma};

use canmore::client_ids::CLIENT_ID_POWER_BOARD;
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
            name: "mk2/power_board",
            client_id: CLIENT_ID_POWER_BOARD,
            bus: Bus::Internal,
            version: titan_rp2040::version!(),
            ..BoardConfig::DEFAULT
        },
    );
}
