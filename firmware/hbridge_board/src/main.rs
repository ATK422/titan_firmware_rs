//! Lilboat H-bridge board: drives the thrusters and drive motors through four DRV8243 H-bridges, and the claw's
//! Hiwonder bus servos.
//!
//! Rust port of `lilboat/HBridgeBoard`. ROS topics (served by `titan_bridge --board hbridge`, namespace
//! `liltank`):
//! * `command/thruster/port`, `command/thruster/stbd`, `command/motor/stbd`, `command/motor/port`,
//!   `command/bridge2/unused` (`std_msgs/Float32`, subscribed): bridge outputs in `[-1, 1]`
//! * `command/actuator/arm` (`std_msgs/Bool`, subscribed): arm or disarm the servos
//! * `command/actuator/move_speed_for_time` (`std_msgs/Int32`, subscribed): run the claw grip for 1 s
//! * `state/actuator/status` (`riptide_msgs2/ActuatorStatus`, best effort, published every 500 ms)
//! * `state/actuator/degrees` (`std_msgs/Int32`, published every 30 ms)
//! * `state/actuator/cmd_feedback` (`std_msgs/String`), `state/actuator/cmd_status` (`std_msgs/Bool`):
//!   published after each actuator command
//! * plus the standard `heartbeat`, `state/firmware` and `state/kill`
//!
//! Pins (`boards/lilboat/hbridge_board.h`): bridge phase/enable pairs H0 0/1, H1 2/3, H3 6/7 (inverted), H4 8/9;
//! nSLEEP 25, DRVOFF 12; CD74HC4051 select 23/28/29 and output 26 (ADC 0) carrying the nFAULT lines of H0, H1 and
//! H3; H4's nFAULT on GPIO 11; servo bus on GPIO 24 (PIO0 SM0).

#![no_std]
#![no_main]

mod actuators;
mod bridges;
mod multidrop;

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::adc::{self, Adc};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH1, DMA_CH2, DMA_CH3, DMA_CH4, PIO0};
use embassy_rp::pio::{self, Pio};
use embassy_rp::pwm::Pwm;
use embassy_rp::{bind_interrupts, dma};

use canmore::client_ids::CLIENT_ID_ESC_BOARD0;
use titan_link::{HostToDevice, hbridge};
use titan_rp2040::board::{self, BoardConfig, Bus};
use titan_rp2040::drivers::cd74hc4051::Cd74hc4051;
use titan_rp2040::safety::STANDARD_FAULT_NAMES;

use crate::bridges::{BridgePins, Hardware, Nfault};

bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO => adc::InterruptHandler;
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    // DMA_CH1/2 belong to the CAN block's SPI, DMA_CH3/4 move the servo UART's bytes
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH1>, dma::InterruptHandler<DMA_CH2>, dma::InterruptHandler<DMA_CH3>,
        dma::InterruptHandler<DMA_CH4>;
});

static FAULT_NAMES: [&str; 6] = [
    STANDARD_FAULT_NAMES[0],
    STANDARD_FAULT_NAMES[1],
    STANDARD_FAULT_NAMES[2],
    STANDARD_FAULT_NAMES[3],
    STANDARD_FAULT_NAMES[4],
    "FAULT_HBRIDGE_DISCONNECTED",
];
const _: () = assert!(bridges::FAULT_HBRIDGE_DISCONNECTED as usize == FAULT_NAMES.len() - 1);

/// Link hook for H-bridge messages from the bridge (the C executor callbacks).
fn handle_board_message(msg: HostToDevice) {
    match msg {
        HostToDevice::HBridge(hbridge::HostToDevice::SetTarget { bridge, percent }) => {
            bridges::set_target(bridge, percent)
        }
        HostToDevice::HBridge(hbridge::HostToDevice::ActuatorArm(arm)) => {
            actuators::queue_command(actuators::Command::Arm(arm))
        }
        HostToDevice::HBridge(hbridge::HostToDevice::MoveSpeedForTime(speed)) => {
            actuators::queue_command(actuators::Command::MoveSpeedForTime(speed))
        }
        other => defmt::warn!("Unexpected message for the H-bridge board: {}", other),
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    board::start(
        spawner,
        titan_rp2040::can_block!(p, Irqs),
        BoardConfig {
            name: "lilboat/hbridge_board",
            client_id: CLIENT_ID_ESC_BOARD0,
            bus: Bus::Internal,
            version: titan_rp2040::version!(),
            fault_names: &FAULT_NAMES,
            on_kill: bridges::on_kill,
            on_enable: bridges::on_enable,
            on_board_message: handle_board_message,
        },
    );

    // Servo bus (init_servos): claw grip ID 3, claw rack ID 4
    let Pio { mut common, sm0, irq0, irq1, irq_flags, .. } = Pio::new(p.PIO0, Irqs);
    let uart = multidrop::MultidropUart::new(
        &mut common,
        sm0,
        irq0,
        irq1,
        irq_flags,
        p.PIN_24,
        dma::Channel::new(p.DMA_CH3, Irqs),
        dma::Channel::new(p.DMA_CH4, Irqs),
        hbridge_core::hiwonder::BAUD,
    );
    spawner.spawn(actuators::uart_task(uart).unwrap());
    spawner.spawn(actuators::actuator_task().unwrap());

    // hbridge_init: bridges asleep and drivers off until the vehicle is enabled
    let nsleep = Output::new(p.PIN_25, Level::Low);
    bridges::init(Output::new(p.PIN_12, Level::High));

    // hbridge_create: phase low, PWM enable at 0
    let pwm = bridges::pwm_config();
    let hw = Hardware {
        bridges: [
            BridgePins {
                phase: Output::new(p.PIN_0, Level::Low),
                pwm: Pwm::new_output_b(p.PWM_SLICE0, p.PIN_1, pwm.clone()),
                nfault: Nfault::Mux(0),
            },
            BridgePins {
                phase: Output::new(p.PIN_2, Level::Low),
                pwm: Pwm::new_output_b(p.PWM_SLICE1, p.PIN_3, pwm.clone()),
                nfault: Nfault::Mux(1),
            },
            BridgePins {
                phase: Output::new(p.PIN_6, Level::Low),
                pwm: Pwm::new_output_b(p.PWM_SLICE3, p.PIN_7, pwm.clone()),
                nfault: Nfault::Mux(2),
            },
            BridgePins {
                phase: Output::new(p.PIN_8, Level::Low),
                pwm: Pwm::new_output_b(p.PWM_SLICE4, p.PIN_9, pwm),
                // gpio_init leaves the reset pull-down on
                nfault: Nfault::Gpio(Input::new(p.PIN_11, Pull::Down)),
            },
        ],
        nsleep,
        mux: Cd74hc4051::new(p.PIN_26, p.PIN_23, p.PIN_28, p.PIN_29),
        adc: Adc::new(p.ADC, Irqs, adc::Config::default()),
    };
    spawner.spawn(bridges::hbridge_task(hw).unwrap());
}
