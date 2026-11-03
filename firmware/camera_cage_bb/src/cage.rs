//! Board tasks, ported from `main.c`, `ros.c` and `safety_interface.h`.

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_futures::select::{Either, select};
use embassy_rp::gpio::{Input, Level, Output};
use embassy_rp::i2c::{self, I2c};
use embassy_rp::peripherals::{I2C0, PIO0};
use embassy_rp::pio_programs::ws2812::{Grb, PioWs2812};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker, Timer};
use portable_atomic::{AtomicBool, Ordering};

use titan_link::camera_cage::{self, electrical, led};
use titan_link::{DeviceToHost, HostToDevice};
use titan_rp2040::drivers::Latest;
use titan_rp2040::drivers::depth::{DepthState, Ms5837, SurfaceStore};
use titan_rp2040::drivers::sht41::{self, Sht41};
use titan_rp2040::drivers::status_strip::{self, Mode, RGB8, StatusStripControl};
use titan_rp2040::link;
use titan_rp2040::safety::{self, STANDARD_FAULT_NAMES};

// Board faults, same IDs as the C `safety_interface.h`
/// A command that can't be carried out right now (a second computer power cycle while one runs).
pub const FAULT_ROS_BAD_COMMAND: u8 = 5;
/// The depth sensor failed to initialize or zero; it stays off until the next reset.
pub const FAULT_DEPTH_INIT_ERROR: u8 = 6;
/// Depth readings are failing or overrunning.
pub const FAULT_DEPTH_ERROR: u8 = 7;
/// SHT41 measurements are failing.
pub const FAULT_SHT41_ERROR: u8 = 8;

pub static FAULT_NAMES: [&str; 9] = [
    STANDARD_FAULT_NAMES[0],
    STANDARD_FAULT_NAMES[1],
    STANDARD_FAULT_NAMES[2],
    STANDARD_FAULT_NAMES[3],
    STANDARD_FAULT_NAMES[4],
    "FAULT_ROS_BAD_COMMAND",
    "FAULT_DEPTH_INIT_ERROR",
    "FAULT_DEPTH_ERROR",
    "FAULT_SHT41_ERROR",
];

pub type I2cBus = Mutex<CriticalSectionRawMutex, I2c<'static, I2C0, i2c::Async>>;
type BusDevice = I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, I2C0, i2c::Async>>;

/// `WATER_TEMP_PUBLISH_INTERVAL_MS`
const WATER_TEMP_PERIOD: Duration = Duration::from_millis(1000);
/// How often the ROS task looks for new readings. The C main loop polled continuously; 10 ms is well below the
/// 50 ms depth and 500 ms SHT41 poll periods.
const ROS_POLL_PERIOD: Duration = Duration::from_millis(10);
/// Delay before the Orin is switched off for a power cycle.
const CYCLE_START_DELAY: Duration = Duration::from_millis(10);
/// How long the Orin stays off during a power cycle.
const CYCLE_OFF_TIME: Duration = Duration::from_millis(5000);
/// Data input of the strip is at the rear of the cage (`status_strip_init(.., true)`).
const FIRST_PIXEL_IS_REAR: bool = true;

pub static DEPTH: DepthState = DepthState::new();
/// Survives watchdog resets so a sensor that is already underwater isn't zeroed there.
#[unsafe(link_section = ".uninit.depth")]
static SURFACE: SurfaceStore = SurfaceStore::new();
pub static SHT41: Latest<sht41::Reading> = Latest::new(sht41::VALID_FOR);
pub static STRIP: StatusStripControl = StatusStripControl::new();

#[derive(Clone, Copy, defmt::Format)]
enum PowerCommand {
    Fans(bool),
    CycleComputer,
}

static POWER_COMMANDS: Channel<CriticalSectionRawMutex, PowerCommand, 4> = Channel::new();
/// A computer power cycle is scheduled or running (`toggle_pwr_alarm_id != 0`).
static CYCLE_ACTIVE: AtomicBool = AtomicBool::new(false);

fn queue_power(command: PowerCommand) {
    if POWER_COMMANDS.try_send(command).is_err() {
        defmt::warn!("Power command queue full, dropping {}", command);
    }
}

/// Link hook for camera cage messages from the bridge (the C executor callbacks).
pub fn handle_board_message(msg: HostToDevice) {
    match msg {
        HostToDevice::CameraCage(camera_cage::HostToDevice::LedCommand(cmd)) => {
            // Ignore commands not for us
            if cmd.target & led::TARGET_CCB == 0 {
                return;
            }
            let mode = match cmd.mode {
                led::MODE_SOLID => Mode::Solid,
                led::MODE_SLOW_FLASH => Mode::SlowFlash,
                led::MODE_FAST_FLASH => Mode::FastFlash,
                led::MODE_BREATH => Mode::Breath,
                _ => return STRIP.clear(),
            };
            STRIP.set(mode, RGB8 { r: cmd.red, g: cmd.green, b: cmd.blue });
        }
        HostToDevice::CameraCage(camera_cage::HostToDevice::PhysKillNotify(removed)) => {
            // The thrusters don't chime while software kill is active, so the strip is the only feedback for
            // whoever moves the kill switch: red when it's removed, blue when it's inserted
            STRIP.status_flash(if removed { RGB8 { r: 255, g: 0, b: 0 } } else { RGB8 { r: 0, g: 0, b: 255 } });
        }
        HostToDevice::CameraCage(camera_cage::HostToDevice::ElectricalCommand(command)) => match command {
            electrical::CYCLE_COMPUTER => {
                if CYCLE_ACTIVE.swap(true, Ordering::Relaxed) {
                    defmt::warn!("Attempting to request multiple cycles in a row");
                    safety::raise_fault(FAULT_ROS_BAD_COMMAND, 0);
                } else {
                    queue_power(PowerCommand::CycleComputer);
                }
            }
            electrical::ENABLE_FANS => queue_power(PowerCommand::Fans(true)),
            electrical::DISABLE_FANS => queue_power(PowerCommand::Fans(false)),
            electrical::ENABLE_LEDS => STRIP.enable(),
            electrical::DISABLE_LEDS => STRIP.disable(),
            electrical::CLEAR_DEPTH => DEPTH.request_recalibration(),
            // Commands for other boards
            _ => {}
        },
        other => defmt::warn!("Unexpected message for the camera cage board: {}", other),
    }
}

#[embassy_executor::task]
pub async fn depth_task(mut sensor: Ms5837<BusDevice>) -> ! {
    // Fatal errors end the driver. The C board never called depth_init again, so neither does this one.
    let err = sensor.run(&DEPTH, &SURFACE, |err| safety::raise_fault(FAULT_DEPTH_ERROR, err.code())).await;
    safety::raise_fault(FAULT_DEPTH_INIT_ERROR, err.code());
    core::future::pending().await
}

#[embassy_executor::task]
pub async fn sht41_task(mut sensor: Sht41<BusDevice>) -> ! {
    sensor.run(&SHT41, |err| safety::raise_fault(FAULT_SHT41_ERROR, err.code())).await
}

#[embassy_executor::task]
pub async fn strip_task(mut leds: PioWs2812<'static, PIO0, 0, { status_strip::NUM_LEDS }, Grb>) -> ! {
    status_strip::run(&mut leds, &STRIP, FIRST_PIXEL_IS_REAR).await
}

/// Owns the fan and Orin power switches. Replaces the C power cycle alarm.
#[embassy_executor::task]
pub async fn power_task(mut fan: Output<'static>, mut orin: Output<'static>) -> ! {
    // The pending power cycle step: whether it turns the Orin back on, and when
    let mut cycle: Option<(bool, Instant)> = None;
    loop {
        let deadline = cycle.map_or(Instant::MAX, |(_, at)| at);
        match select(POWER_COMMANDS.receive(), Timer::at(deadline)).await {
            Either::First(PowerCommand::Fans(on)) => fan.set_level(Level::from(on)),
            Either::First(PowerCommand::CycleComputer) => cycle = Some((false, Instant::now() + CYCLE_START_DELAY)),
            Either::Second(()) => match cycle {
                Some((false, at)) => {
                    orin.set_low();
                    cycle = Some((true, at + CYCLE_OFF_TIME));
                }
                Some((true, _)) => {
                    orin.set_high();
                    cycle = None;
                    CYCLE_ACTIVE.store(false, Ordering::Relaxed);
                }
                None => {}
            },
        }
    }
}

/// Publishes the sensor readings while ROS is connected (`tick_ros_tasks`) and clears the strip when the
/// connection drops.
#[embassy_executor::task]
pub async fn ros_task(_leak_sense: Input<'static>) -> ! {
    let mut ticker = Ticker::every(ROS_POLL_PERIOD);
    let mut connected = false;
    let mut next_water_temp = Instant::MAX;
    loop {
        ticker.next().await;
        let now = Instant::now();
        match (connected, link::is_connected()) {
            (false, true) => {
                connected = true;
                // start_ros_timers
                next_water_temp = now + WATER_TEMP_PERIOD;
            }
            (true, false) => {
                connected = false;
                // Clear the status strip on teardown of ROS
                STRIP.clear();
                continue;
            }
            (false, false) => continue,
            (true, true) => {}
        }

        // Send depth as soon as a new reading comes in
        if DEPTH.take_new().is_some()
            && let Some(reading) = DEPTH.reading()
        {
            link::send(&DeviceToHost::CameraCage(camera_cage::DeviceToHost::Depth(-reading.depth_m() as f32)));
        }

        // The C timer was only checked while the reading was valid, so missed periods are skipped silently
        if let Some(reading) = DEPTH.reading()
            && now >= next_water_temp
        {
            while next_water_temp <= now {
                next_water_temp += WATER_TEMP_PERIOD;
            }
            link::send(&DeviceToHost::CameraCage(camera_cage::DeviceToHost::WaterTemp(reading.temperature_c())));
        }

        if SHT41.take_new().is_some()
            && let Some(reading) = SHT41.get()
        {
            link::send(&DeviceToHost::CameraCage(camera_cage::DeviceToHost::TempHumidity {
                temperature_c: reading.temperature_c,
                humidity_rh: reading.humidity_rh,
            }));
        }
    }
}
