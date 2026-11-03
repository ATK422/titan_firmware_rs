//! Talos IVC board: an acoustic modem that sends single bytes between Talos and the surface vehicle (SSFB).
//!
//! Rust port of `talos/IVCBoard`. ROS topics (served by `titan_bridge --board ivc`):
//! * `ivc/tx` (`std_msgs/Int8`, subscribed): byte to send
//! * `ivc/rx` (`std_msgs/Int8`, published): byte received
//! * `ivc/debug_tx_frequency` (`std_msgs/Float32`, subscribed): play a test tone for 1 s
//! * `ivc/debug_rx_mag` (`std_msgs/Float32`, published): low band magnitude, `calibration` builds only
//! * plus the standard `heartbeat`, `state/firmware` and `state/kill`
//!
//! Pins (`boards/talos/ivc_board.h`): transducer drive on GPIO 7, filter output select on GPIO 3, hydrophone on
//! GPIO 26 (ADC 0), board ID on GPIO 6 (low on Talos).

#![no_std]
#![no_main]

mod ivc;

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::adc::{self, Adc};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, DMA_CH2};
use embassy_rp::{bind_interrupts, dma};

use canmore::client_ids::CLIENT_ID_IVC_BOARD;
use titan_rp2040::board::{self, BoardConfig, Bus};

bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO => adc::InterruptHandler;
    // DMA_CH0 feeds the ADC; DMA_CH1/2 belong to the CAN block's SPI
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>, dma::InterruptHandler<DMA_CH2>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    board::start(
        spawner,
        titan_rp2040::can_block!(p, Irqs),
        BoardConfig {
            name: "talos/ivc_board",
            client_id: CLIENT_ID_IVC_BOARD,
            bus: Bus::Internal,
            version: titan_rp2040::version!(),
            // The IVC board has no actuators to disable; the status LED follows the kill state on its own
            on_board_message: ivc::handle_board_message,
            ..BoardConfig::DEFAULT
        },
    );

    let output_select = Output::new(p.PIN_3, Level::High);
    // The C firmware left the pad's reset pull-down enabled, so a floating ID pin reads as Talos
    let is_talos = Input::new(p.PIN_6, Pull::Down).is_low();
    defmt::info!("This board {} talos", if is_talos { "is" } else { "is not" });

    let adc = Adc::new(p.ADC, Irqs, adc::Config::default());
    let hydrophone = adc::Channel::new_pin(p.PIN_26, Pull::None);
    let adc_dma = dma::Channel::new(p.DMA_CH0, Irqs);
    spawner.spawn(ivc::sampler_task(adc, hydrophone, adc_dma).unwrap());

    let transmitter = ivc::Transmitter::new(p.PWM_SLICE3, p.PIN_7);
    spawner.spawn(ivc::ivc_task(ivc::Modem::new(transmitter, output_select, is_talos)).unwrap());
}
