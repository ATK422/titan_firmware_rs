//! Talos actuator board (Mk2): torpedoes, marker droppers and the claw. Rust port of `talos/Actuator`.
//!
//! Two actuator generations are supported, as in C:
//! * **V2** (default, `USE_V2_ACTUATORS`): two Dynamixel XL430 servos on the TTL bus on GPIO 10, one turning the
//!   torpedo/marker cam (ID 2) and one the claw (ID 3). See [`v2`].
//! * **V1** (`--features v1`): coil gun torpedoes sequenced by PIO with an ADC watching the capacitor charge, GPIO
//!   droppers and a timed DC motor claw. See [`v1`].
//!
//! ROS interface (served by `titan_bridge --board actuator`), as in the C `ros.c`:
//! * `state/actuator/status` (`riptide_msgs2/ActuatorStatus`, best effort) and `state/actuator/busy`
//!   (`std_msgs/Bool`, best effort) every second
//! * `state/actuator/dynamixel` (`riptide_msgs2/DynamixelStatus`, best effort), V2 only, one per connected servo
//! * services `command/actuator/{arm,claw}` (`std_srvs/SetBool`) and `command/actuator/{torpedo,dropper,
//!   notify_reload}` (`std_srvs/Trigger`)
//! * V2 services `command/actuator/torpedo_marker/{go_home,set_home}`, `command/actuator/claw/set_closed_pos`
//!   (`std_srvs/Trigger`)
//! * V1 parameters `claw_timing_ms`, `dropper_active_timing_ms`, `torpedo_coil*_timing_us`
//! * plus the standard `heartbeat`, `state/firmware` and `state/kill`
//!
//! The WS2812 status strip on GPIO 12 plays its startup animation and is otherwise kept dark, as in C.
//!
//! Pins (`boards/mk2/actuator_board.h`): claw 0-5, 9, 10 and 29, status strip 12, torpedo select 6/7, drain 8, arm 11,
//! coils 25-27, charge level 28 (ADC 2), droppers 23/24.

#![no_std]
#![no_main]

mod control;
#[cfg(not(feature = "v1"))]
mod dynamixel_cmds;
#[cfg(feature = "v1")]
mod v1;
#[cfg(not(feature = "v1"))]
mod v2;

#[cfg(feature = "v1")]
use v1 as actuators;
#[cfg(not(feature = "v1"))]
use v2 as actuators;

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, DMA_CH2, PIO0};
use embassy_rp::pio::{self, Pio};
use embassy_rp::pio_programs::ws2812::{Grb, PioWs2812, PioWs2812Program};
use embassy_rp::{bind_interrupts, dma};

use canmore::client_ids::CLIENT_ID_ACTUATOR_BOARD;
use titan_rp2040::board::{self, BoardConfig, Bus};
use titan_rp2040::drivers::status_strip::{self, StatusStripControl};
use titan_rp2040::safety::{self, STANDARD_FAULT_NAMES};

#[cfg(not(feature = "v1"))]
bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    // DMA_CH0 feeds the status strip, DMA_CH1/2 belong to the CAN block's SPI, DMA_CH3 is the Dynamixel UART
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>, dma::InterruptHandler<DMA_CH2>,
        dma::InterruptHandler<embassy_rp::peripherals::DMA_CH3>;
});

#[cfg(feature = "v1")]
bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    ADC_IRQ_FIFO => embassy_rp::adc::InterruptHandler;
    // DMA_CH0 feeds the status strip, DMA_CH1/2 belong to the CAN block's SPI
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>, dma::InterruptHandler<DMA_CH2>;
});

/// An actuator command could not be carried out because of a hardware or firmware problem.
pub const FAULT_ACTUATOR_FAILURE: u8 = safety::FIRST_BOARD_FAULT;
/// A Dynamixel reports a hardware error.
pub const FAULT_ACTUATOR_HW_FAULT: u8 = 6;
/// A Dynamixel is unplugged; cleared once every servo is connected.
pub const FAULT_ACTUATOR_UNPLUGGED: u8 = 7;

static FAULT_NAMES: [&str; 8] = [
    STANDARD_FAULT_NAMES[0],
    STANDARD_FAULT_NAMES[1],
    STANDARD_FAULT_NAMES[2],
    STANDARD_FAULT_NAMES[3],
    STANDARD_FAULT_NAMES[4],
    "FAULT_ACTUATOR_FAILURE",
    "FAULT_ACTUATOR_HW_FAULT",
    "FAULT_ACTUATOR_UNPLUGGED",
];

/// The status strip along the top of the cage.
pub static STRIP: StatusStripControl = StatusStripControl::new();
/// The strip's data input is at the front of the cage (`status_strip_init(pio0, 0, RGB_DATA_PIN, false)`).
const FIRST_PIXEL_IS_REAR: bool = false;

#[embassy_executor::task]
async fn strip_task(mut leds: PioWs2812<'static, PIO0, 0, { status_strip::NUM_LEDS }, Grb>) -> ! {
    status_strip::run(&mut leds, &STRIP, FIRST_PIXEL_IS_REAR).await
}

/// Kill: stop every actuator (`safety_handle_kill`). Runs in a critical section; only touches pins and queues.
fn on_kill() {
    actuators::disarm();
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    board::start(
        spawner,
        titan_rp2040::can_block!(p, Irqs),
        BoardConfig {
            name: "mk2/actuator_board",
            client_id: CLIENT_ID_ACTUATOR_BOARD,
            bus: Bus::Internal,
            version: titan_rp2040::version!(),
            fault_names: &FAULT_NAMES,
            on_kill,
            // The status LED follows the kill state on its own; nothing else to restore
            on_board_message: control::handle_board_message,
            // The C board registered remote commands only with the V2 actuators
            #[cfg(not(feature = "v1"))]
            remote_commands: &v2::REMOTE_COMMANDS,
            ..BoardConfig::DEFAULT
        },
    );

    // Status strip on PIO0 state machine 0, cleared like the C main
    let Pio { mut common, sm0, sm1, irq_flags, irq0, irq1, irq2, .. } = Pio::new(p.PIO0, Irqs);
    let strip_program = PioWs2812Program::new(&mut common);
    let leds = PioWs2812::new(&mut common, sm0, p.DMA_CH0, Irqs, p.PIN_12, &strip_program);
    STRIP.clear();
    spawner.spawn(strip_task(leds).unwrap());

    #[cfg(not(feature = "v1"))]
    {
        let _ = irq0;
        v2::init(
            spawner,
            v2::Hardware {
                common: &mut common,
                sm: sm1,
                irq_flags,
                tx_irq: irq1,
                err_irq: irq2,
                pin: p.PIN_10,
                pwm_pin: p.PIN_9,
                dma: dma::Channel::new(p.DMA_CH3, Irqs),
            },
        );
    }

    #[cfg(feature = "v1")]
    {
        let _ = (irq_flags, irq1, irq2);
        v1::init(
            spawner,
            v1::Hardware {
                common: &mut common,
                sm: sm1,
                fired_irq: irq0,
                coil3: p.PIN_25,
                coil2: p.PIN_26,
                coil1: p.PIN_27,
                select1: p.PIN_7,
                select2: p.PIN_6,
                arm: p.PIN_11,
                drain: p.PIN_8,
                adc: p.ADC,
                charge_level: p.PIN_28,
                irqs: Irqs,
                dropper1: p.PIN_24,
                dropper2: p.PIN_23,
                claw_enable: p.PIN_0,
                claw_phase: p.PIN_2,
                claw_mode2: p.PIN_5,
            },
        );
    }

    spawner.spawn(control::command_task().unwrap());
    spawner.spawn(control::status_task().unwrap());
}
