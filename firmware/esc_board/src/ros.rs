//! The board's ROS interface (`ros.c`) and the CANmore thruster command channel (`thruster_cmd_callback` in
//! `main.c`).
//!
//! Through `titan_bridge --board esc0|esc1`:
//! * `command/thruster_rpm` (`riptide_msgs2/DshotCommand`, best effort) → [`HostToDevice::ThrusterRpm`]; each
//!   one is answered with the RPM feedback
//! * `dshot_tune/*` → the `Set*` messages, forwarded to the control loop as tuning parameters
//! * `state/thrusters/rpm` and `state/thrusters/cmd` ← [`esc::DeviceToHost::Rpm`]
//! * `state/thrusters/telemetry` ← [`esc::DeviceToHost::Telemetry`], every 500 ms
//!
//! Directly over CAN, the robot computer's controller sends target RPMs as utility frames on
//! [`canmore::CHAN_THRUSTER_CMDS`]: four big-endian `i16`, one per thruster of this board.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Ticker, Timer};
use portable_atomic::{AtomicU8, Ordering};

use esc_core::NUM_THRUSTERS;
use esc_core::control_loop::Param;
use titan_link::{DeviceToHost, HostToDevice, esc};
use titan_rp2040::{canbus, link, safety};

use crate::dshot::UART_TELEMETRY;
use crate::{FAULT_ROS_BAD_COMMAND, FAULT_RPM_CMD_TIMEOUT, core1, vcc};

const TELEMETRY_INTERVAL: Duration = Duration::from_millis(500);

static BOARD_NUM: AtomicU8 = AtomicU8::new(0);
/// Set when a ROS thruster command arrives, so the RPM feedback goes out (`dshot_command_received`).
static RPM_FEEDBACK_REQUESTED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Records which board this is (0 or 1, from the board detection pin). Call before the link starts.
pub fn set_board_num(num: u8) {
    BOARD_NUM.store(num, Ordering::Relaxed);
}

fn start_thruster() -> u8 {
    BOARD_NUM.load(Ordering::Relaxed) * NUM_THRUSTERS as u8
}

/// Link hook for ESC board messages from the bridge.
pub fn handle_board_message(msg: HostToDevice) {
    let HostToDevice::Esc(msg) = msg else {
        defmt::warn!("Unexpected message for the ESC board: {}", msg);
        return;
    };
    match msg {
        esc::HostToDevice::ThrusterRpm(values) => {
            let start = start_thruster() as usize;
            let mut rpm = [0; NUM_THRUSTERS];
            rpm.copy_from_slice(&values[start..start + NUM_THRUSTERS]);
            core1::update_target_rpm(rpm);
            RPM_FEEDBACK_REQUESTED.signal(());
        }
        // Like the C callbacks, out of range values are dropped (set_param logs them)
        esc::HostToDevice::SetRawMode(on) => _ = core1::set_param(Param::RawMode(on)),
        esc::HostToDevice::SetPGain(v) => _ = core1::set_param(Param::PGain(v)),
        esc::HostToDevice::SetIGain(v) => _ = core1::set_param(Param::IGain(v)),
        esc::HostToDevice::SetIBound(v) => _ = core1::set_param(Param::IBound(v)),
        esc::HostToDevice::SetHardLimit(v) => _ = core1::set_param(Param::HardLimit(v)),
        esc::HostToDevice::SetMinCommand(v) => _ = core1::set_param(Param::MinCommand(v)),
    }
}

/// Target RPMs from the robot computer on the thruster command utility channel.
#[embassy_executor::task]
pub async fn thruster_command_task() -> ! {
    loop {
        let frame = canbus::UTIL_RX.receive().await;
        if frame.channel != canmore::CHAN_THRUSTER_CMDS {
            defmt::debug!("Ignoring utility frame on channel {}", frame.channel);
            continue;
        }
        if frame.data.len() != NUM_THRUSTERS * 2 {
            defmt::debug!("Dropping invalid thruster command with length {}", frame.data.len());
            continue;
        }
        // Big-endian per canmore/protocol.h. The C callback OR'd the high byte into itself instead of using the
        // low byte; this decodes the documented format.
        let rpm = core::array::from_fn(|i| i16::from_be_bytes([frame.data[2 * i], frame.data[2 * i + 1]]));
        core1::update_target_rpm(rpm);
        // Like C, no RPM feedback for these (it was disabled there since nobody used it)
    }
}

/// Sends the RPM feedback after each ROS thruster command (`ros_send_rpm`).
#[embassy_executor::task]
pub async fn rpm_feedback_task() -> ! {
    loop {
        RPM_FEEDBACK_REQUESTED.wait().await;
        if !link::is_connected() {
            continue;
        }
        let t = core1::telemetry(false);
        let mut feedback = esc::RpmFeedback { start_thruster: start_thruster(), ..Default::default() };
        for (i, thruster) in t.thrusters.iter().enumerate() {
            if thruster.rpm_valid {
                feedback.valid_mask |= 1 << i;
                feedback.rpm[i] = thruster.rpm;
            }
            feedback.cmd[i] = thruster.cmd;
        }
        link::send(&DeviceToHost::Esc(esc::DeviceToHost::Rpm(feedback)));
    }
}

/// Sends the telemetry every 500 ms while ROS is connected (`ros_send_telemetry`), starting 500 ms after it
/// connects like `start_ros_timers`.
#[embassy_executor::task]
pub async fn telemetry_task() -> ! {
    loop {
        while !link::is_connected() {
            Timer::after(safety::TICK).await;
        }
        // A new ROS context: the earlier command faults no longer apply (FAULT_ROS_ERROR is lowered by the link)
        safety::lower_fault(FAULT_ROS_BAD_COMMAND);
        safety::lower_fault(FAULT_RPM_CMD_TIMEOUT);

        // Start the tick rate measurement from here
        let _ = core1::telemetry(true);
        let mut last = embassy_time::Instant::now();
        let mut ticker = Ticker::every(TELEMETRY_INTERVAL);
        loop {
            ticker.next().await;
            if !link::is_connected() {
                break;
            }
            let now = embassy_time::Instant::now();
            let delta_us = (now - last).as_micros().min(u32::MAX as u64) as u32;
            last = now;
            link::send(&DeviceToHost::Esc(esc::DeviceToHost::Telemetry(telemetry(delta_us))));
        }
    }
}

fn telemetry(packet_delta_us: u32) -> esc::Telemetry {
    let control = core1::telemetry(true);
    let vcc_mv = vcc::vcc_mv();
    let mut msg = esc::Telemetry {
        start_thruster: start_thruster(),
        escs_powered: esc_core::esc_board_on(vcc_mv),
        thrusters_moving: control.thrusters_on,
        disabled_flags: control.disabled_flags,
        vcc_mv,
        packet_delta_us,
        tick_cnt: control.tick_cnt,
        escs: Default::default(),
    };
    for (i, out) in msg.escs.iter_mut().enumerate() {
        let uart = UART_TELEMETRY.lock(|t| t.borrow_mut().take(i));
        let thruster = &control.thrusters[i];
        if uart.valid {
            // Online only with both RPM feedback and UART telemetry coming in
            out.esc_online = thruster.rpm_valid;
            out.temperature_c = uart.temperature;
            out.voltage_cv = uart.voltage;
            out.current_ca = uart.current;
            out.consumption_mah = uart.consumption;
            out.rpm = uart.mechanical_rpm();
        }
        out.uart_disabled_cnt = uart.total_invalid;
        out.uart_missed_cnt = uart.total_missed;
        out.uart_success_cnt = uart.total_success;
        out.thruster_ready = thruster.thruster_ready;
        out.dshot_missed_cnt = thruster.ticks_missed;
        out.dshot_offline_cnt = thruster.ticks_offline;
    }
    msg
}
