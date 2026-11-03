//! The board's half of the ROS interface (`ros_actuators.c`, `actuator_v2_dynamixel.c`, `actuator_v1_parameters.c`
//! and the actuator timer in `main.c`).
//!
//! Service calls arrive as [`Command`]s and are carried out by [`command_task`], which answers each with the
//! result, like the C service callbacks did. [`status_task`] publishes the actuator status every second while ROS is
//! connected and clears the status strip when the connection drops.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Ticker};

use titan_link::actuator::{self, Command};
use titan_link::{DeviceToHost, HostToDevice};
use titan_rp2040::{link, safety};

use crate::actuators;

/// Service calls waiting for [`command_task`].
static COMMANDS: Channel<CriticalSectionRawMutex, (u8, Command), 4> = Channel::new();

/// How often the status goes out (`ACTUATOR_STATUS_TIME_MS`).
const STATUS_PERIOD: Duration = Duration::from_millis(1000);
/// How often [`status_task`] checks the connection.
const POLL_PERIOD: Duration = Duration::from_millis(100);

/// Link hook for actuator messages from the bridge.
pub fn handle_board_message(msg: HostToDevice) {
    match msg {
        HostToDevice::Actuator(actuator::HostToDevice::Command { seq, command }) => {
            if COMMANDS.try_send((seq, command)).is_err() {
                // The bridge times the call out and reports it failed
                defmt::warn!("Actuator command queue full, dropping {}", command);
            }
        }
        other => defmt::warn!("Unexpected message for the actuator board: {}", other),
    }
}

/// Carries out service calls one at a time and answers each.
#[embassy_executor::task]
pub async fn command_task() -> ! {
    loop {
        let (seq, command) = COMMANDS.receive().await;
        let result = actuators::execute(command);
        match result {
            Ok(()) => defmt::info!("Actuator command {} done", command),
            Err(err) => defmt::info!("Actuator command {} failed: {}", command, err.message()),
        }
        if !link::send(&DeviceToHost::Actuator(actuator::DeviceToHost::CommandResult { seq, result })) {
            defmt::warn!("Link queue full, dropped the result of {}", command);
        }
    }
}

/// Publishes the status every second while ROS is connected, starting a second after it connects.
#[embassy_executor::task]
pub async fn status_task() -> ! {
    let mut ticker = Ticker::every(POLL_PERIOD);
    let mut connected = false;
    let mut next_status = Instant::MAX;
    loop {
        ticker.next().await;
        let now = Instant::now();
        match (connected, link::is_connected()) {
            (false, true) => {
                connected = true;
                // Staggered from bring-up like start_ros_timers
                next_status = now + STATUS_PERIOD;
                actuators::on_ros_connected();
            }
            (true, false) => {
                connected = false;
                next_status = Instant::MAX;
                crate::STRIP.clear();
            }
            _ => {}
        }

        if now < next_status {
            continue;
        }
        // Like timer_ready: advance, counting whole periods that were skipped
        let mut missed = 0u32;
        next_status += STATUS_PERIOD;
        while next_status <= now {
            next_status += STATUS_PERIOD;
            missed += 1;
        }
        if missed > 0 {
            safety::raise_fault(safety::FAULT_TIMER_MISSED, missed);
        }

        let status = actuators::status();
        if !link::send(&DeviceToHost::Actuator(actuator::DeviceToHost::Status(status))) {
            defmt::debug!("Link queue full, skipping the actuator status");
            continue;
        }
        actuators::send_extra_status();
    }
}
