//! Talos IVC board, second revision firmware: Rust port of `talos/IVCBoard2`.
//!
//! A slower, more robust acoustic modem than `talos/IVCBoard`, plus a pinger listening mode and a flash recorder.
//! Bytes go out at 500 ms per symbol on 50.5/51/51.5 kHz tones with a sync between bits; see `ivc2_core`. ROS topics
//! (served by `titan_bridge --board ivc2`, node `IVCBoard2`):
//! * `ivc/enqueue_data` (`std_msgs/Int8`, subscribed): byte to send
//! * `ivc/rx_new` (`std_msgs/Int8`, published): byte received
//! * `ivc/pinger/enable` (`std_msgs/Bool`), `ivc/pinger/set_freq_khz` (`std_msgs/Int32`, subscribed): pinger mode
//! * `ivc/pinger/selected_freq_amp_stream` (`std_msgs/Float32`), `ivc/pinger/selected_freq_khz` (`std_msgs/Int32`,
//!   published while in pinger mode)
//! * `ivc/storage/configure` (`std_msgs/Int8`, subscribed): flash recorder, 0 off, 1 write, 2 dump to the log
//! * `ivc/set_min_snr` (`std_msgs/Float32`, subscribed): detection threshold
//! * `ivc/debug/send_freq` (`std_msgs/Int8`, subscribed, best effort): play one tone for 500 ms
//! * silent publishers kept for parity: `ivc/adc_dump`, `ivc/new_transducer`, `ivc/rx_sample_stream`,
//!   `ivc/rx_sample_snr_stream`
//! * plus the standard `heartbeat`, `state/firmware` and `state/kill`
//!
//! Pins (`boards/talos/ivc_board.h`): transducer drive on GPIO 7, filter output select on GPIO 3, board ID on GPIO 6.
//! The hydrophone is read on ADC 1 (GPIO 27): the C `fft.c` samples `CAPTURE_CHANNEL` 1, not the header's `RX_PIN`.
//!
//! The C firmware used the micro-ROS USB transport; this one talks to the bridge over the internal CAN bus like the
//! other Rust boards, with the board header's client ID.

#![no_std]
#![no_main]

mod hw;
mod modem;
mod recorder;

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::adc::{self, Adc};
use embassy_rp::flash::Flash;
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
            // No actuators to disable; the status LED follows the kill state on its own
            on_board_message: modem::handle_board_message,
            ..BoardConfig::DEFAULT
        },
    );

    // Selects the low pass filtered transducer output
    let output_select = Output::new(p.PIN_3, Level::High);
    // Only logged: unlike IVCBoard, this protocol treats both vehicles alike. The pad's reset pull-down stays on
    // like in C, so a floating ID pin reads as Talos.
    let is_talos = Input::new(p.PIN_6, Pull::Down).is_low();
    defmt::info!("This board {} talos", if is_talos { "is" } else { "is not" });

    let adc = Adc::new(p.ADC, Irqs, adc::Config::default());
    let hydrophone = adc::Channel::new_pin(p.PIN_27, Pull::None);
    let adc_dma = dma::Channel::new(p.DMA_CH0, Irqs);
    let recorder = recorder::Recorder::new(Flash::new_blocking(p.FLASH));
    spawner.spawn(modem::rx_task(hw::Hydrophone::new(adc, hydrophone, adc_dma), recorder).unwrap());
    spawner.spawn(modem::tx_task(hw::Transmitter::new(p.PWM_SLICE3, p.PIN_7), output_select).unwrap());
}
