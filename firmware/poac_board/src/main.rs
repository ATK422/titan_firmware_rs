//! Talos Poac board: Rust port of `talos/PoacBoard`, the power board with the torpedo/dropper servo added.
//!
//! Despite the "acoustics" in the name, the C firmware has nothing acoustic: it is the power board (kill switch
//! source, rail monitoring, fan) without the battery voltage inputs, plus a Hiwonder bus servo on a PIO half duplex
//! UART that fires the torpedoes and drops the markers.
//!
//! ROS topics (served by `titan_bridge --board poac`), besides `heartbeat` and `state/firmware`:
//! * `state/kill`, `state/physkill_notify`, `command/software_kill`, `state/electrical`,
//!   `state/batteries_balanced`, `state/aux`, `command/electrical`: as on the power board
//! * `state/temp/poacboard`, `state/humidity/poacboard` (`std_msgs/Float32`, best effort)
//! * `command/actuator/{torpedo,dropper,notify_reload}`, `command/actuator/torpedo_marker/{go_home,set_home}`
//!   (`std_msgs/Empty`) and `command/actuator/arm` (`std_msgs/Bool`)
//! * `state/actuator/busy` (`std_msgs/Bool`) and `state/actuator/status` (`riptide_msgs2/ActuatorStatus`), best
//!   effort, every 500 ms; `state/actuator/dynamixel` is advertised but never published, as in C
//! * `state/actuator/cmd_feedback` (`std_msgs/String`) and `state/actuator/cmd_status` (`std_msgs/Bool`): outcome of
//!   each command
//!
//! Pins (`boards/talos/poac_board.h`): aux switch 0, physical kill switch 1, software kill 2, I2C0 on 4/5 (SHT41,
//! MCP3426), port/starboard powering status 10/11, servo bus 27 (PIO0 state machine 0), fan switch 28.

#![no_std]
#![no_main]

mod actuators;
mod remote_cmd;
mod uart;

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{self, I2c};
use embassy_rp::peripherals::{DMA_CH1, DMA_CH2, I2C0, PIO0};
use embassy_rp::pio::{self, Pio};
use embassy_rp::{bind_interrupts, dma};
use embassy_time::{Duration, Instant};

use canmore::client_ids::CLIENT_ID_POWER_BOARD;
use power_board::kill::{self, KillPublisher};
use power_board::{Interval, ROS_TICK, fan, sensors};
use power_core::actuator::Command;
use power_core::electrical::TALOS_RAILS;
use titan_link::{DeviceToHost, HostToDevice, poac, power};
use titan_rp2040::board::{self, BoardConfig, Bus};
use titan_rp2040::drivers::mcp3426::Mcp3426;
use titan_rp2040::drivers::sht41::Sht41;
use titan_rp2040::{link, safety};

bind_interrupts!(struct Irqs {
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    // DMA_CH1/2 belong to the CAN block's SPI
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH1>, dma::InterruptHandler<DMA_CH2>;
});

// Board faults, numbered as in `safety_interface.h`
const FAULT_ROS_BAD_COMMAND: u8 = 5;
const FAULT_ADC_ERROR: u8 = 6;
const FAULT_SHT41_ERROR: u8 = 7;

static FAULT_NAMES: [&str; 8] = {
    let s = safety::STANDARD_FAULT_NAMES;
    [s[0], s[1], s[2], s[3], s[4], "FAULT_ROS_BAD_COMMAND", "FAULT_ADC_ERROR", "FAULT_SHT41_ERROR"]
};

/// `ELECTRICAL_READINGS_INTERVAL`
const ELECTRICAL_PERIOD: Duration = Duration::from_millis(1000);
/// `AUXSWITCH_INTERVAL`
const AUX_PERIOD: Duration = Duration::from_millis(1000);
/// `ACTUATOR_STATUS_TIME_MS`
const ACTUATOR_STATUS_PERIOD: Duration = Duration::from_millis(500);
/// `SERVO_UPDATE_CMD_STATUS_PERIOD_MS`
const CMD_FEEDBACK_PERIOD: Duration = Duration::from_millis(100);

fn wrap(msg: power::DeviceToHost) -> DeviceToHost {
    DeviceToHost::Poac(poac::DeviceToHost::Power(msg))
}

fn send(msg: poac::DeviceToHost) {
    if !link::send(&DeviceToHost::Poac(msg)) {
        defmt::debug!("Transport queue full, dropped actuator message");
    }
}

fn handle_board_message(msg: HostToDevice) {
    let HostToDevice::Poac(msg) = msg else {
        defmt::warn!("Not a Poac board message: {}", msg);
        return;
    };
    let command = match msg {
        poac::HostToDevice::Power(msg) => return fan::handle_power_message(msg, FAULT_ROS_BAD_COMMAND),
        poac::HostToDevice::FireTorpedo => Command::FireTorpedo,
        poac::HostToDevice::DropMarker => Command::DropMarker,
        poac::HostToDevice::NotifyReload => Command::NotifyReload,
        poac::HostToDevice::Arm(arm) => Command::Arm(arm),
        poac::HostToDevice::GoHome => Command::GoHome,
        poac::HostToDevice::SetHome => Command::SetHome,
    };
    actuators::handle_command(command);
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Kill until safety says otherwise, before anything can call the kill hooks
    kill::init_soft_kill(Output::new(p.PIN_2, Level::Low));

    board::start(
        spawner,
        titan_rp2040::can_block!(p, Irqs),
        BoardConfig {
            name: "talos/poac_board",
            client_id: CLIENT_ID_POWER_BOARD,
            bus: Bus::Internal,
            version: titan_rp2040::version!(),
            fault_names: &FAULT_NAMES,
            // As in C, a kill doesn't disarm the servo: firing and arming check the kill state instead
            on_kill: kill::on_kill,
            on_enable: kill::on_enable,
            on_board_message: handle_board_message,
            remote_commands: &remote_cmd::COMMANDS,
        },
    );

    fan::init_fan(Output::new(p.PIN_28, fan::FAN_BOOT_LEVEL));
    spawner.spawn(kill::physical_kill_task(Input::new(p.PIN_1, Pull::Up)).unwrap());

    // The C firmware left the reset pull-down on the aux input and disabled the pulls on the status inputs
    let aux = Input::new(p.PIN_0, Pull::Down);
    let port_stat = Input::new(p.PIN_10, Pull::None);
    let stbd_stat = Input::new(p.PIN_11, Pull::None);

    let uart = uart::MultidropUart::new(Pio::new(p.PIO0, Irqs), p.PIN_27, actuators::BAUD, actuators::RX_TIMEOUT);
    spawner.spawn(actuators::servo_task(uart).unwrap());

    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = sensors::TALOS_I2C_FREQUENCY;
    let bus = sensors::share_bus(I2c::new_async(p.I2C0, p.PIN_5, p.PIN_4, Irqs, i2c_config));
    let rail_adc = Mcp3426::new(sensors::device(bus), sensors::MCP3426_ADDRESS);
    spawner.spawn(sensors::mcp3426_task(rail_adc, FAULT_ADC_ERROR).unwrap());
    spawner.spawn(sensors::sht41_task(Sht41::new(sensors::device(bus)), FAULT_SHT41_ERROR).unwrap());

    spawner.spawn(ros_task(aux, port_stat, stbd_stat).unwrap());
}

/// Publishes the board topics while ROS is connected (`tick_ros_tasks`).
#[embassy_executor::task]
async fn ros_task(aux: Input<'static>, port_stat: Input<'static>, stbd_stat: Input<'static>) -> ! {
    loop {
        power_board::wait_connected().await;
        // A new ROS session forgets earlier bad commands
        safety::lower_fault(FAULT_ROS_BAD_COMMAND);
        let mut kill_publisher = KillPublisher::new();
        let mut electrical = Interval::starting_in(ELECTRICAL_PERIOD, true);
        let mut aux_interval = Interval::starting_in(AUX_PERIOD, true);
        let mut actuator_status = Interval::starting_in(ACTUATOR_STATUS_PERIOD, true);
        let mut cmd_feedback = Interval::starting_in(CMD_FEEDBACK_PERIOD, false);

        while link::is_connected() {
            let refreshed = kill::next_tick(ROS_TICK).await;
            let now = Instant::now();
            kill_publisher.poll(now, refreshed, wrap);

            if electrical.fire(now) {
                // No battery voltage inputs on this board
                let readings =
                    sensors::electrical_readings(&TALOS_RAILS, 0.0, 0.0, port_stat.is_high(), stbd_stat.is_high());
                power_board::send(wrap, power::DeviceToHost::Electrical(readings));
            }

            if aux_interval.fire(now) {
                power_board::send(wrap, power::DeviceToHost::AuxSwitch(aux.is_high()));
            }

            sensors::send_new_temp_humidity(wrap);

            if actuator_status.fire(now) {
                let (busy, status) = actuators::status();
                send(poac::DeviceToHost::Status { busy, status });
            }

            if cmd_feedback.fire(now)
                && let Some(feedback) = actuators::take_feedback()
            {
                send(poac::DeviceToHost::CommandFeedback(feedback));
            }
        }
    }
}
