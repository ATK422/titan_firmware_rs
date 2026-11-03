//! The board's side of its ROS topics, ported from `ros.c` and `tick_ros_tasks` in `main.c`.
//!
//! [`handle_board_message`] takes the subscriptions (`command/dshot`, `command/software_kill`,
//! `command/electrical`). [`ros_task`] publishes, while the bridge is connected:
//! * `state/kill` every 150 ms and whenever the kill state changes
//! * `state/depth/raw` for every new depth reading
//! * `state/depth/temp` every second while the depth reading is valid
//! * `state/electrical` every 500 ms
//!
//! `state/thrusters/rpm` goes out from [`crate::dshot`] after each thruster command, and `state/firmware` comes from
//! the link. [`power_cycle_task`] switches the computer and acoustics rails for `command/electrical`.

use embassy_rp::gpio::Output;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Ticker, Timer};
use portable_atomic::{AtomicBool, Ordering};

use puddles_core::{Interval, electrical};
use titan_link::puddles_backplane::{DeviceToHost as Board, ElectricalCommand, HostToDevice as Command};
use titan_link::{DeviceToHost, HostToDevice};
use titan_rp2040::drivers::mcp3426::Channel;
use titan_rp2040::{link, safety};

use crate::safety_interface::{self, FAULT_ROS_BAD_COMMAND, FAULT_THRUSTER_TIMEOUT};
use crate::sensors::{DEPTH, VOLTAGES};
use crate::{dshot, safety_interface::KILL_REFRESHED};

/// Client ID reported in the heartbeat and firmware status.
pub const CLIENT_ID: u8 = 1;

const KILLSWITCH_PUBLISH_PERIOD_US: u64 = 150_000;
const WATER_TEMP_PUBLISH_PERIOD_US: u64 = 1_000_000;
const ADC_PERIOD_US: u64 = 500_000;
/// How often [`ros_task`] looks for work. The C main loop polled continuously; nothing here needs finer timing.
const POLL_PERIOD: Duration = Duration::from_millis(10);

/// Link hook for the backplane's messages. Runs in the link task, so it only queues work.
pub fn handle_board_message(msg: HostToDevice) {
    match msg {
        HostToDevice::PuddlesBackplane(Command::Dshot(throttles)) => dshot::command(throttles),
        HostToDevice::PuddlesBackplane(Command::SoftwareKill(kill)) => safety_interface::handle_software_kill(&kill),
        HostToDevice::PuddlesBackplane(Command::Electrical(ElectricalCommand::ClearDepth)) => {
            DEPTH.request_recalibration()
        }
        HostToDevice::PuddlesBackplane(Command::Electrical(ElectricalCommand::CycleComputer)) => {
            request_cycle(&COMPUTER_POWER)
        }
        HostToDevice::PuddlesBackplane(Command::Electrical(ElectricalCommand::CycleAcoustics)) => {
            request_cycle(&ACOUSTICS_POWER)
        }
        other => defmt::warn!("Unexpected message for the backplane: {}", other),
    }
}

fn send(msg: Board) {
    link::send(&DeviceToHost::PuddlesBackplane(msg));
}

/// The timers started when ROS connects (`start_ros_timers`).
struct Timers {
    killswitch: Interval,
    water_temp: Interval,
    adc: Interval,
}

/// Fires a timer whose missed runs are a fault (`timer_ready(..., true)`).
fn fire_critical(timer: &mut Interval, now: u64) -> bool {
    match timer.fire(now) {
        Some(0) => true,
        Some(missed) => {
            defmt::warn!("Missed {} runs of a critical timer", missed);
            safety::raise_fault(safety::FAULT_TIMER_MISSED, missed);
            true
        }
        None => false,
    }
}

/// Publishes the backplane's topics while the bridge is connected.
#[embassy_executor::task]
pub async fn ros_task() -> ! {
    let mut ticker = Ticker::every(POLL_PERIOD);
    let mut timers: Option<Timers> = None;
    loop {
        ticker.next().await;
        if !link::is_connected() {
            timers = None;
            continue;
        }
        let now = Instant::now().as_micros();
        let t = timers.get_or_insert_with(|| {
            // A new ROS connection: earlier command errors no longer apply (the link lowers FAULT_ROS_ERROR)
            safety::lower_fault(FAULT_ROS_BAD_COMMAND);
            safety::lower_fault(FAULT_THRUSTER_TIMEOUT);
            Timers {
                killswitch: Interval::starting_at(now, KILLSWITCH_PUBLISH_PERIOD_US),
                water_temp: Interval::starting_at(now, WATER_TEMP_PUBLISH_PERIOD_US),
                adc: Interval::starting_at(now, ADC_PERIOD_US),
            }
        });

        // Evaluate both: the timer keeps its schedule even when a kill state change publishes first
        let killswitch_due = fire_critical(&mut t.killswitch, now);
        if KILL_REFRESHED.swap(false, Ordering::Relaxed) || killswitch_due {
            send(Board::Kill(safety::asserting_kill()));
        }

        // Depth as soon as a new reading comes in
        if DEPTH.take_new().is_some()
            && let Some(reading) = DEPTH.reading()
        {
            send(Board::Depth(-reading.depth_m() as f32));
        }

        // The C timer only advances while the reading is valid
        if let Some(reading) = DEPTH.reading()
            && t.water_temp.fire(now).is_some()
        {
            send(Board::WaterTemp(reading.temperature_c()));
        }

        if t.adc.fire(now).is_some() {
            let channels = Channel::ALL.map(|channel| VOLTAGES.get(channel));
            send(Board::Electrical(electrical::readings(channels)));
        }
    }
}

/// A switched power rail and its pending cycle.
pub struct PowerRail {
    request: Signal<CriticalSectionRawMutex, ()>,
    cycling: AtomicBool,
    name: &'static str,
}

impl PowerRail {
    const fn new(name: &'static str) -> Self {
        PowerRail { request: Signal::new(), cycling: AtomicBool::new(false), name }
    }
}

/// The robot computer's power (`PWR_CTL_CPU`, GPIO 26).
pub static COMPUTER_POWER: PowerRail = PowerRail::new("computer");
/// The acoustics' power (`PWR_CTL_ACC`, GPIO 10).
pub static ACOUSTICS_POWER: PowerRail = PowerRail::new("acoustics");

/// Delay before the rail switches off (`add_alarm_in_ms(10, ...)`).
const CYCLE_DELAY: Duration = Duration::from_millis(10);
/// How long the rail stays off.
const CYCLE_OFF_TIME: Duration = Duration::from_millis(1000);

/// Starts a power cycle; a second request while one is running is a bad command, as in C.
fn request_cycle(rail: &'static PowerRail) {
    if rail.cycling.swap(true, Ordering::Relaxed) {
        defmt::warn!("Attempting to request multiple {} power cycles in a row", rail.name);
        safety::raise_fault(FAULT_ROS_BAD_COMMAND, 0);
    } else {
        rail.request.signal(());
    }
}

/// Switches a rail off for [`CYCLE_OFF_TIME`] on each request, then back on.
///
/// The C alarm callback re-armed itself with the same "off" argument, so after a cycle request the rail stayed off
/// for good (re-written every second) and every later request was rejected. The comments there say the rail
/// should come back on after a second, which is what this does.
#[embassy_executor::task(pool_size = 2)]
pub async fn power_cycle_task(rail: &'static PowerRail, mut output: Output<'static>) -> ! {
    loop {
        rail.request.wait().await;
        Timer::after(CYCLE_DELAY).await;
        defmt::info!("Power cycling the {}", rail.name);
        output.set_low();
        Timer::after(CYCLE_OFF_TIME).await;
        output.set_high();
        rail.cycling.store(false, Ordering::Relaxed);
    }
}
