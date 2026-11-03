//! Board specific safety, ported from `safety_interface.c` and the software kill callback in `ros.c`.
//!
//! * Faults: the C board numbered its faults after only two library faults. The Rust safety core fixes IDs 0-4
//!   (`FAULT_CAN_INTERNAL_ERROR` at 2, `FAULT_ROS_ERROR` at 3, `FAULT_TIMER_MISSED` at 4), so the board faults keep
//!   their order but start at [`safety::FIRST_BOARD_FAULT`], one above their C IDs.
//! * Physical kill switch: GPIO 12, pulled up, high (switch removed) asserts kill. It feeds kill switch 0
//!   (`KILL_SWITCH_PHYSICAL`) as a switch that needs updates, refreshed on every edge and every safety tick like
//!   `safety_interface_tick`, and tells the thrusters about it for their wake-up holdoff.
//! * Software kill switches 1-3 come from `command/software_kill`, with the sender locks of
//!   [`puddles_core::kill`].
//! * Kill hooks stop the thrusters and flag a kill state change so `state/kill` is published right away.

use core::cell::RefCell;

use embassy_futures::select::select;
use embassy_rp::gpio::Input;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Timer;
use portable_atomic::{AtomicBool, Ordering};

use puddles_core::kill::{KILL_SWITCH_PHYSICAL, SoftwareKillLocks};
use titan_link::puddles_backplane::SoftwareKill;
use titan_rp2040::safety::{self, FIRST_BOARD_FAULT, STANDARD_FAULT_NAMES};

use crate::dshot;

/// An invalid command arrived from ROS (C ID 4).
pub const FAULT_ROS_BAD_COMMAND: u8 = FIRST_BOARD_FAULT;
/// Thruster commands stopped arriving while the thrusters ran (C ID 5).
pub const FAULT_THRUSTER_TIMEOUT: u8 = FIRST_BOARD_FAULT + 1;
/// The depth sensor failed to initialize (C ID 6).
pub const FAULT_DEPTH_INIT_ERROR: u8 = FIRST_BOARD_FAULT + 2;
/// Depth sensor reads are failing (C ID 7).
pub const FAULT_DEPTH_ERROR: u8 = FIRST_BOARD_FAULT + 3;
/// The MCP3426 rail voltage ADC is failing (C ID 8).
pub const FAULT_ADC_ERROR: u8 = FIRST_BOARD_FAULT + 4;

/// Fault names by ID.
pub static FAULT_NAMES: [&str; 10] = [
    STANDARD_FAULT_NAMES[0],
    STANDARD_FAULT_NAMES[1],
    STANDARD_FAULT_NAMES[2],
    STANDARD_FAULT_NAMES[3],
    STANDARD_FAULT_NAMES[4],
    "FAULT_ROS_BAD_COMMAND",
    "FAULT_THRUSTER_TIMEOUT",
    "FAULT_DEPTH_INIT_ERROR",
    "FAULT_DEPTH_ERROR",
    "FAULT_ADC_ERROR",
];

/// Kill state seen by the hooks; starts false like C's `prev_kill_state`, so the first kill counts as a change.
static PREV_KILLED: AtomicBool = AtomicBool::new(false);
/// Set when the kill state changes (`safety_interface_kill_switch_refreshed`); cleared by the `state/kill`
/// publisher.
pub static KILL_REFRESHED: AtomicBool = AtomicBool::new(false);

/// `safety_handle_kill`: runs in a critical section, possibly from any task.
pub fn on_kill() {
    dshot::stop_thrusters();
    if !PREV_KILLED.swap(true, Ordering::Relaxed) {
        KILL_REFRESHED.store(true, Ordering::Relaxed);
    }
}

/// `safety_handle_enable`.
pub fn on_enable() {
    if PREV_KILLED.swap(false, Ordering::Relaxed) {
        KILL_REFRESHED.store(true, Ordering::Relaxed);
    }
}

/// Keeps the physical kill switch up to date, on every edge and at least every safety tick.
#[embassy_executor::task]
pub async fn kill_switch_task(mut sense: Input<'static>) -> ! {
    loop {
        // The switch pulls the line low when inserted
        let asserting_kill = sense.is_high();
        safety::kill_switch_update(KILL_SWITCH_PHYSICAL, asserting_kill, true);
        dshot::notify_physical_kill_switch(asserting_kill);
        select(sense.wait_for_any_edge(), Timer::after(safety::TICK)).await;
    }
}

static SOFTWARE_KILL_LOCKS: Mutex<CriticalSectionRawMutex, RefCell<SoftwareKillLocks>> =
    Mutex::new(RefCell::new(SoftwareKillLocks::new()));

/// Applies a `command/software_kill` update (`software_kill_subscription_callback`).
pub fn handle_software_kill(msg: &SoftwareKill) {
    let report = safety::kill_switch_report();
    let bit = 1u32.checked_shl(msg.kill_switch_id as u32).unwrap_or(0);
    let switch_killing = report.enabled & report.asserting_kill & bit != 0;
    match SOFTWARE_KILL_LOCKS.lock(|locks| locks.borrow_mut().check(msg, switch_killing)) {
        Ok(()) => safety::kill_switch_update(msg.kill_switch_id, msg.asserting_kill, msg.needs_update),
        Err(rejected) => {
            defmt::warn!("Software kill on switch {} rejected: {}", msg.kill_switch_id, rejected);
            safety::raise_fault(FAULT_ROS_BAD_COMMAND, rejected.fault_arg());
        }
    }
}
