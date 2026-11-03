//! Talos power board: Rust port of `talos/PowerBoard`, the vehicle's kill switch source and battery monitor.
//!
//! ROS topics (served by `titan_bridge --board power`), besides `state/firmware`:
//! * `state/fw_heartbeat` (`std_msgs/Int8`): the heartbeat; this board doesn't use `heartbeat`
//! * `state/kill` (`std_msgs/Bool`, published): kill state, every 150 ms and on every change
//! * `state/physkill_notify` (`std_msgs/Bool`): the physical kill switch changed
//! * `command/software_kill` (`riptide_msgs2/KillSwitchReport`, best effort): software kill switches 1-3
//! * `state/electrical` (`riptide_msgs2/ElectricalReadings`) and `state/batteries_balanced` (`std_msgs/Bool`):
//!   every second
//! * `state/aux` (`std_msgs/Bool`): aux switch, every second
//! * `command/electrical` (`riptide_msgs2/ElectricalCommand`): fan on/off
//! * `state/temp/powerboard`, `state/humidity/powerboard` (`std_msgs/Float32`, best effort): each SHT41 reading
//!
//! Pins (`boards/mk2/power_board.h`): aux switch 0, physical kill switch 1, software kill 2, I2C0 on 4/5 (SHT41,
//! MCP3426), starboard/port powering status 10/11, port/starboard battery measurement 26/27 (ADC 0/1), fan switch
//! 28, fan tachometer 29 (unused).

#![no_std]
#![no_main]

use core::cell::Cell;

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::adc::{self, Adc};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{self, I2c};
use embassy_rp::peripherals::{DMA_CH1, DMA_CH2, I2C0};
use embassy_rp::{bind_interrupts, dma};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, Ticker};

use canmore::client_ids::CLIENT_ID_POWER_BOARD;
use power_board::kill::{self, KillPublisher};
use power_board::{Interval, ROS_TICK, fan, sensors};
use power_core::electrical::{self, TALOS_RAILS};
use titan_link::{DeviceToHost, HostToDevice, power};
use titan_rp2040::board::{self, BoardConfig, Bus};
use titan_rp2040::drivers::mcp3426::Mcp3426;
use titan_rp2040::drivers::sht41::Sht41;
use titan_rp2040::{link, safety};

bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO => adc::InterruptHandler;
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
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
/// `ADC_TIMER_TIME_MS`: each battery is read every other period
const BATTERY_ADC_PERIOD: Duration = Duration::from_millis(100);

/// Latest (port, starboard) battery voltages; 0 V until read, as in C.
static BATTERIES: Mutex<CriticalSectionRawMutex, Cell<(f32, f32)>> = Mutex::new(Cell::new((0.0, 0.0)));

fn wrap(msg: power::DeviceToHost) -> DeviceToHost {
    DeviceToHost::Power(msg)
}

fn handle_board_message(msg: HostToDevice) {
    match msg {
        HostToDevice::Power(msg) => fan::handle_power_message(msg, FAULT_ROS_BAD_COMMAND),
        other => defmt::warn!("Not a power board message: {}", other),
    }
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
            name: "mk2/power_board",
            client_id: CLIENT_ID_POWER_BOARD,
            bus: Bus::Internal,
            version: titan_rp2040::version!(),
            fault_names: &FAULT_NAMES,
            on_kill: kill::on_kill,
            on_enable: kill::on_enable,
            on_board_message: handle_board_message,
        },
    );

    fan::init_fan(Output::new(p.PIN_28, fan::FAN_BOOT_LEVEL));
    spawner.spawn(kill::physical_kill_task(Input::new(p.PIN_1, Pull::Up)).unwrap());

    // The C firmware left the reset pull-down on the aux input and disabled the pulls on the status inputs
    let aux = Input::new(p.PIN_0, Pull::Down);
    let stbd_stat = Input::new(p.PIN_10, Pull::None);
    let port_stat = Input::new(p.PIN_11, Pull::None);

    let adc = Adc::new(p.ADC, Irqs, adc::Config::default());
    let port_meas = adc::Channel::new_pin(p.PIN_26, Pull::None);
    let stbd_meas = adc::Channel::new_pin(p.PIN_27, Pull::None);
    spawner.spawn(battery_task(adc, port_meas, stbd_meas).unwrap());

    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = sensors::TALOS_I2C_FREQUENCY;
    let bus = sensors::share_bus(I2c::new_async(p.I2C0, p.PIN_5, p.PIN_4, Irqs, i2c_config));
    let rail_adc = Mcp3426::new(sensors::device(bus), sensors::MCP3426_ADDRESS);
    spawner.spawn(sensors::mcp3426_task(rail_adc, FAULT_ADC_ERROR).unwrap());
    spawner.spawn(sensors::sht41_task(Sht41::new(sensors::device(bus)), FAULT_SHT41_ERROR).unwrap());

    spawner.spawn(ros_task(aux, port_stat, stbd_stat).unwrap());
}

/// Reads the port and starboard battery voltages alternately every 100 ms (`analog_io.c`).
#[embassy_executor::task]
async fn battery_task(
    mut adc: Adc<'static, adc::Async>,
    mut port: adc::Channel<'static>,
    mut stbd: adc::Channel<'static>,
) {
    let mut ticker = Ticker::every(BATTERY_ADC_PERIOD);
    // Starboard is selected first
    let mut read_stbd = true;
    loop {
        ticker.next().await;
        let channel = if read_stbd { &mut stbd } else { &mut port };
        match adc.read(channel).await {
            Ok(raw) => {
                let volts = electrical::battery_voltage(raw);
                BATTERIES.lock(|b| {
                    let (port_v, stbd_v) = b.get();
                    b.set(if read_stbd { (port_v, volts) } else { (volts, stbd_v) });
                });
            }
            Err(err) => defmt::warn!("Battery ADC read failed: {}", err),
        }
        read_stbd = !read_stbd;
    }
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

        while link::is_connected() {
            let refreshed = kill::next_tick(ROS_TICK).await;
            let now = Instant::now();
            kill_publisher.poll(now, refreshed, wrap);

            if electrical.fire(now) {
                let (port_v, stbd_v) = BATTERIES.lock(Cell::get);
                let readings = sensors::electrical_readings(
                    &TALOS_RAILS,
                    port_v,
                    stbd_v,
                    port_stat.is_high(),
                    stbd_stat.is_high(),
                );
                power_board::send(wrap, power::DeviceToHost::Electrical(readings));
            }

            if aux_interval.fire(now) {
                power_board::send(wrap, power::DeviceToHost::AuxSwitch(aux.is_high()));
            }

            sensors::send_new_temp_humidity(wrap);
        }
    }
}
