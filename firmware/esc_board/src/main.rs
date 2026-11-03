//! Talos ESC board: drives four thrusters with bidirectional DShot600 and closes an RPM loop around them.
//!
//! Rust port of `talos/ESCBoard`. Talos has two ESC boards; the board detection pin picks which half of the eight
//! thrusters this one drives and its CANmore client ID (ESC_BOARD0 or ESC_BOARD1), before CAN comes up.
//!
//! * Core 1 runs the control loop ([`core1`]): DShot frames out and eRPM replies in over PIO0 ([`dshot`]), one tick
//!   per 400 µs frame.
//! * Core 0 runs embassy: safety, CAN and the link, the ESC rail monitor ([`vcc`]), the UART telemetry receivers on
//!   PIO1, and the ROS side ([`ros`]).
//!
//! Target RPMs arrive as CANmore utility frames on channel 0 from the robot computer, or on the
//! `command/thruster_rpm` topic through `titan_bridge --board esc0|esc1`. Without a fresh target every 500 ms, or
//! while killed, unpowered or unconfigured, the thrusters get neutral.
//!
//! Pins (`boards/mk2/esc_board.h`): ESC1-4 DShot on GPIO 6, 4, 2, 0 and telemetry on GPIO 7, 5, 3, 1; VCC sense
//! on GPIO 28 (ADC 2); board detection on GPIO 29 (low = board 0).
//!
//! The `esc_firmware_patching/` directory of the C tree is not firmware for this board: it is a binary patch (and
//! its notes) that adds 3D mode to the APD 80F3 ESCs' own firmware, applied with APD's tools.

#![no_std]
#![no_main]

mod core1;
mod dshot;
mod ros;
mod vcc;

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::adc::{self, Adc};
use embassy_rp::gpio::{Input, Pull};
use embassy_rp::peripherals::{DMA_CH1, DMA_CH2, PIO0, PIO1};
use embassy_rp::pio::{self, Pio};
use embassy_rp::{bind_interrupts, dma};
use embassy_time::Duration;

use canmore::client_ids::{CLIENT_ID_ESC_BOARD0, CLIENT_ID_ESC_BOARD1};
use titan_rp2040::board::{self, BoardConfig, Bus};
use titan_rp2040::safety::{self, FIRST_BOARD_FAULT, STANDARD_FAULT_NAMES};

bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO => adc::InterruptHandler;
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    PIO1_IRQ_0 => pio::InterruptHandler<PIO1>;
    // DMA_CH1/2 belong to the CAN block's SPI
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH1>, dma::InterruptHandler<DMA_CH2>;
});

// Board faults, same IDs as `safety_interface.h`
/// A ROS command was invalid.
pub const FAULT_ROS_BAD_COMMAND: u8 = FIRST_BOARD_FAULT;
/// The target RPM went stale while the thrusters were moving.
pub const FAULT_RPM_CMD_TIMEOUT: u8 = FIRST_BOARD_FAULT + 1;
/// The controller is bypassed (raw throttle mode).
pub const FAULT_RAW_MODE: u8 = FIRST_BOARD_FAULT + 2;

static FAULT_NAMES: [&str; 8] = [
    STANDARD_FAULT_NAMES[0],
    STANDARD_FAULT_NAMES[1],
    STANDARD_FAULT_NAMES[2],
    STANDARD_FAULT_NAMES[3],
    STANDARD_FAULT_NAMES[4],
    "FAULT_ROS_BAD_COMMAND",
    "FAULT_RPM_CMD_TIMEOUT",
    "FAULT_RAW_MODE",
];

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Board detection (`load_client_id`): pulled down, so a floating pin reads as board 0
    let board_num = {
        let detect = Input::new(p.PIN_29, Pull::Down);
        embassy_time::block_for(Duration::from_micros(5));
        detect.is_high() as u8
    };
    let client_id = if board_num == 1 { CLIENT_ID_ESC_BOARD1 } else { CLIENT_ID_ESC_BOARD0 };
    defmt::info!(
        "ESC board {} (thrusters {}-{}), client ID {}",
        board_num,
        board_num * 4,
        board_num * 4 + 3,
        client_id
    );
    ros::set_board_num(board_num);

    board::start(
        spawner,
        titan_rp2040::can_block!(p, Irqs),
        BoardConfig {
            name: "mk2/esc_board",
            client_id,
            bus: Bus::Internal,
            version: titan_rp2040::version!(),
            fault_names: &FAULT_NAMES,
            on_kill: core1::on_kill,
            on_board_message: ros::handle_board_message,
            ..BoardConfig::DEFAULT
        },
    );
    debug_assert_eq!(safety::fault_name(FAULT_RAW_MODE), "FAULT_RAW_MODE");

    let adc = Adc::new(p.ADC, Irqs, adc::Config::default());
    let vcc_sense = adc::Channel::new_pin(p.PIN_28, Pull::None);
    spawner.spawn(vcc::vcc_task(adc, vcc_sense).unwrap());

    let telemetry_rx = dshot::UartTelemetryRx::new(Pio::new(p.PIO1, Irqs), p.PIN_7, p.PIN_5, p.PIN_3, p.PIN_1);
    spawner.spawn(dshot::uart_telemetry_task(telemetry_rx).unwrap());

    let dshot = dshot::Dshot::new(Pio::new(p.PIO0, Irqs), p.PIN_6, p.PIN_4, p.PIN_2, p.PIN_0);
    spawner.spawn(core1::core1_watchdog_task().unwrap());
    core1::launch(p.CORE1, dshot);

    spawner.spawn(ros::thruster_command_task().unwrap());
    spawner.spawn(ros::rpm_feedback_task().unwrap());
    spawner.spawn(ros::telemetry_task().unwrap());
}
