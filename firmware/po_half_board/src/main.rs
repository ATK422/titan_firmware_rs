//! Lilboat PoHalf board: Rust port of `lilboat/PoHalfBoard`. The vehicle's kill switch source, with solenoids and a
//! pump, two analog pressure transducers and two MS5837 depth sensors.
//!
//! ROS topics (served by `titan_bridge --board po_half`, namespace `liltank`), besides `heartbeat` and
//! `state/firmware`:
//! * `state/kill`, `state/physkill_notify` (published) and `command/software_kill` (best effort), as on the power
//!   board
//! * `state/electrical` (`riptide_msgs2/ElectricalReadings`): rails only, every second
//! * `state/temp/pohalf`, `state/humidity/pohalf` (`std_msgs/Float32`, best effort)
//! * `command/solenoid/{water,pressure,exhaust}`, `command/pump` (`std_msgs/Bool`, best effort) and
//!   `state/solenoid/{water,pressure,exhaust}`, `state/pump` (`std_msgs/Bool`, every 500 ms)
//! * `command/fart_for_ms` (`std_msgs/Int32`): run a solenoid cycle
//! * `state/pressure/tank`, `state/pressure/regulated` (`std_msgs/Float32`): transducers, every 500 ms
//! * `state/depth/raw` (`riptide_msgs2/Depth`, sensor data QoS): each water depth reading (20 Hz)
//! * `state/pressure/regulator_housing` (`std_msgs/Float32`): regulator housing pressure, every 500 ms
//! * `command/depth/recalibrate` (`std_msgs/Int8`): zero depth sensor 0 or 1 again
//!
//! Pins (`boards/lilboat/po_half.h`): solenoids 4/5/9 (water, pressure, exhaust), pump 6, aux switch 10, Orin
//! power switch 11, software kill 12, physical kill switch 23, I2C0 on 24/25 (water depth sensor, MCP3426, SHT41),
//! I2C1 on 26/27 (regulator housing depth sensor), pressure transducers 28/29 (ADC 2/3).

#![no_std]
#![no_main]

mod solenoids;

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::adc::{self, Adc};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{self, Async, I2c};
use embassy_rp::peripherals::{DMA_CH1, DMA_CH2, I2C0, I2C1};
use embassy_rp::{bind_interrupts, dma};
use embassy_time::{Duration, Instant, Ticker};

use canmore::client_ids::CLIENT_ID_POWER_BOARD;
use power_board::kill::{self, KillPublisher};
use power_board::{Interval, ROS_TICK, fan, sensors};
use power_core::electrical::{self, PO_HALF_RAILS};
use titan_link::{DeviceToHost, HostToDevice, po_half, power};
use titan_rp2040::board::{self, BoardConfig, Bus};
use titan_rp2040::drivers::depth::{DepthState, Model, Ms5837, SurfaceStore};
use titan_rp2040::drivers::mcp3426::Mcp3426;
use titan_rp2040::drivers::sht41::Sht41;
use titan_rp2040::{link, safety};

bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO => adc::InterruptHandler;
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
    // DMA_CH1/2 belong to the CAN block's SPI
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH1>, dma::InterruptHandler<DMA_CH2>;
});

// Board faults, numbered as in `safety_interface.h`
const FAULT_ROS_BAD_COMMAND: u8 = 5;
const FAULT_DEPTH_INIT_ERROR: u8 = 6;
const FAULT_DEPTH_ERROR: u8 = 7;
const FAULT_ADC_ERROR: u8 = 8;
const FAULT_SHT41_ERROR: u8 = 9;

static FAULT_NAMES: [&str; 10] = {
    let s = safety::STANDARD_FAULT_NAMES;
    [
        s[0],
        s[1],
        s[2],
        s[3],
        s[4],
        "FAULT_ROS_BAD_COMMAND",
        "FAULT_DEPTH_INIT_ERROR",
        "FAULT_DEPTH_ERROR",
        "FAULT_ADC_ERROR",
        "FAULT_SHT41_ERROR",
    ]
};

/// I2C clock of both buses, as passed to `async_i2c_init`.
const I2C_FREQUENCY: u32 = 200_000;
/// `ELECTRICAL_READINGS_INTERVAL_MS`
const ELECTRICAL_PERIOD: Duration = Duration::from_millis(1000);
/// `PRESSURE_PUB_INTERVAL_MS`: transducer reads and regulator housing pressure
const PRESSURE_PERIOD: Duration = Duration::from_millis(500);
/// `SOLENOID_STATE_PUB_INTERVAL_MS`
const SOLENOID_STATE_PERIOD: Duration = Duration::from_millis(500);

/// Water depth sensor (`WATER_DEPTH_NUM`, on I2C0).
static WATER_DEPTH: DepthState = DepthState::new();
#[unsafe(link_section = ".uninit.depth0")]
static WATER_SURFACE: SurfaceStore = SurfaceStore::new();
/// Regulator housing pressure sensor (`REGHOUSING_DEPTH_NUM`, on I2C1).
static REGHOUSING_DEPTH: DepthState = DepthState::new();
#[unsafe(link_section = ".uninit.depth1")]
static REGHOUSING_SURFACE: SurfaceStore = SurfaceStore::new();

fn wrap(msg: power::DeviceToHost) -> DeviceToHost {
    DeviceToHost::PoHalf(po_half::DeviceToHost::Power(msg))
}

fn send(msg: po_half::DeviceToHost) {
    if !link::send(&DeviceToHost::PoHalf(msg)) {
        defmt::debug!("Transport queue full, dropped PoHalf message");
    }
}

fn on_kill() {
    kill::on_kill();
    solenoids::start_kill_routine();
}

fn handle_board_message(msg: HostToDevice) {
    let HostToDevice::PoHalf(msg) = msg else {
        defmt::warn!("Not a PoHalf board message: {}", msg);
        return;
    };
    match msg {
        po_half::HostToDevice::Power(msg) => fan::handle_power_message(msg, FAULT_ROS_BAD_COMMAND),
        po_half::HostToDevice::Solenoid { index, open } => solenoids::set(index as usize, open),
        po_half::HostToDevice::SolenoidCycle(ms) => solenoids::request_cycle(ms),
        po_half::HostToDevice::DepthRecalibrate(0) => WATER_DEPTH.request_recalibration(),
        po_half::HostToDevice::DepthRecalibrate(1) => REGHOUSING_DEPTH.request_recalibration(),
        po_half::HostToDevice::DepthRecalibrate(other) => defmt::warn!("No depth sensor {}", other),
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Kill until safety says otherwise, before anything can call the kill hooks
    kill::init_soft_kill(Output::new(p.PIN_12, Level::Low));

    board::start(
        spawner,
        titan_rp2040::can_block!(p, Irqs),
        BoardConfig {
            name: "lilboat/po_half",
            client_id: CLIENT_ID_POWER_BOARD,
            bus: Bus::Internal,
            version: titan_rp2040::version!(),
            fault_names: &FAULT_NAMES,
            on_kill,
            on_enable: kill::on_enable,
            on_board_message: handle_board_message,
            remote_commands: &[],
        },
    );

    spawner.spawn(kill::physical_kill_task(Input::new(p.PIN_23, Pull::Up)).unwrap());

    // Water, pressure, exhaust, pump: closed at boot
    let outputs = [
        Output::new(p.PIN_4, Level::Low),
        Output::new(p.PIN_5, Level::Low),
        Output::new(p.PIN_9, Level::Low),
        Output::new(p.PIN_6, Level::Low),
    ];
    spawner.spawn(solenoids::solenoid_task(outputs).unwrap());

    // The Orin power switch is switched on at boot and never touched again. The aux switch is configured as an
    // input but never read, as in C. Both pins keep their configuration for good.
    core::mem::forget(Output::new(p.PIN_11, Level::High));
    core::mem::forget(Input::new(p.PIN_10, Pull::Down));

    let adc = Adc::new(p.ADC, Irqs, adc::Config::default());
    let tank = adc::Channel::new_pin(p.PIN_28, Pull::None);
    let regulated = adc::Channel::new_pin(p.PIN_29, Pull::None);
    spawner.spawn(pressure_task(adc, tank, regulated).unwrap());

    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = I2C_FREQUENCY;
    let bus0 = sensors::share_bus(I2c::new_async(p.I2C0, p.PIN_25, p.PIN_24, Irqs, i2c_config));
    let bus1 = I2c::new_async(p.I2C1, p.PIN_27, p.PIN_26, Irqs, i2c_config);
    spawner.spawn(water_depth_task(Ms5837::new(sensors::device(bus0), Model::Ms5837_02ba)).unwrap());
    spawner.spawn(reghousing_depth_task(Ms5837::new(bus1, Model::Ms5837_02ba)).unwrap());
    let rail_adc = Mcp3426::new(sensors::device(bus0), sensors::MCP3426_ADDRESS);
    spawner.spawn(sensors::mcp3426_task(rail_adc, FAULT_ADC_ERROR).unwrap());
    spawner.spawn(sensors::sht41_task(Sht41::new(sensors::device(bus0)), FAULT_SHT41_ERROR).unwrap());

    spawner.spawn(ros_task().unwrap());
}

/// Runs a depth sensor until it fails fatally. Like the C board, a failed sensor stays down until reset.
async fn run_depth<I: embedded_hal_async::i2c::I2c>(
    sensor: &mut Ms5837<I>,
    state: &DepthState,
    surface: &SurfaceStore,
) -> ! {
    let err = sensor.run(state, surface, |err| safety::raise_fault(FAULT_DEPTH_ERROR, err.code())).await;
    safety::raise_fault(FAULT_DEPTH_INIT_ERROR, err.code());
    core::future::pending().await
}

#[embassy_executor::task]
async fn water_depth_task(mut sensor: Ms5837<sensors::SharedI2c>) -> ! {
    run_depth(&mut sensor, &WATER_DEPTH, &WATER_SURFACE).await
}

#[embassy_executor::task]
async fn reghousing_depth_task(mut sensor: Ms5837<I2c<'static, I2C1, Async>>) -> ! {
    run_depth(&mut sensor, &REGHOUSING_DEPTH, &REGHOUSING_SURFACE).await
}

/// Reads the tank and regulated pressure transducers every 500 ms and publishes them while ROS is connected
/// (`pressure_read_adc` in the background tick, published from the ROS tick).
#[embassy_executor::task]
async fn pressure_task(
    mut adc: Adc<'static, adc::Async>,
    mut tank: adc::Channel<'static>,
    mut regulated: adc::Channel<'static>,
) -> ! {
    let mut ticker = Ticker::every(PRESSURE_PERIOD);
    loop {
        let (tank_raw, regulated_raw) = (adc.read(&mut tank).await, adc.read(&mut regulated).await);
        // pressure_read_adc returned NaN only for an invalid ADC number; a failed read is the closest thing here
        let reading = |raw: Result<u16, adc::Error>| raw.map_or(f32::NAN, electrical::pressure_sensor_reading);
        if link::is_connected() {
            send(po_half::DeviceToHost::AdcPressures { tank: reading(tank_raw), regulated: reading(regulated_raw) });
        }
        ticker.next().await;
    }
}

/// Publishes the board topics while ROS is connected (`tick_ros_tasks`).
#[embassy_executor::task]
async fn ros_task() -> ! {
    loop {
        power_board::wait_connected().await;
        let mut kill_publisher = KillPublisher::new();
        let mut reghousing = Interval::starting_in(PRESSURE_PERIOD, true);
        let mut electrical = Interval::starting_in(ELECTRICAL_PERIOD, true);
        let mut solenoid_states = Interval::starting_in(SOLENOID_STATE_PERIOD, true);

        while link::is_connected() {
            let refreshed = kill::next_tick(ROS_TICK).await;
            let now = Instant::now();
            kill_publisher.poll(now, refreshed, wrap);

            // Each new water depth reading straight away (depth_set_on_read)
            if WATER_DEPTH.take_new().is_some()
                && let Some(reading) = WATER_DEPTH.reading()
            {
                send(po_half::DeviceToHost::Depth(-reading.depth_m() as f32));
            }

            // Regulator housing pressure at most every 500 ms, if a new reading arrived
            if reghousing.fire(now)
                && REGHOUSING_DEPTH.take_new().is_some()
                && let Some(reading) = REGHOUSING_DEPTH.reading()
            {
                send(po_half::DeviceToHost::RegulatorHousingPressure(reading.pressure_pa as f32));
            }

            sensors::send_new_temp_humidity(wrap);

            if electrical.fire(now) {
                // Rails only: no battery voltage or powering inputs on this board
                let readings = sensors::electrical_readings(&PO_HALF_RAILS, 0.0, 0.0, false, false);
                power_board::send(wrap, power::DeviceToHost::Electrical(readings));
            }

            if solenoid_states.fire(now) {
                send(po_half::DeviceToHost::SolenoidStates(solenoids::states()));
            }
        }
    }
}
