//! Kill switch interface, ported from the power boards' `safety_interface.c` and the kill parts of `ros.c`.
//!
//! * The physical kill switch input (pulled up) reads low while the switch asserts kill. [`physical_kill_task`] feeds it to kill switch
//!   [`KILL_SWITCH_PHYSICAL`] on every edge and at least every safety tick, like the C GPIO interrupt plus
//!   `safety_interface_tick`. It needs updates, so a stuck task kills the vehicle.
//! * The software kill output overrides the kill switch signal to the ESCs. It is driven low (kill) at boot and in
//!   [`on_kill`], high in [`on_enable`].
//! * `command/software_kill` drives kill switches 1-3, each locked to the sender that killed it
//!   ([`handle_software_kill`]).
//! * [`KillPublisher`] publishes `state/kill` every 150 ms and straight away when the kill state flips, and
//!   `state/physkill_notify` when the physical switch changes.

use core::cell::{Cell, RefCell};

use embassy_futures::select::{Either, select};
use embassy_rp::gpio::{Input, Level, Output};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use portable_atomic::{AtomicBool, Ordering};
use power_core::kill::{KILL_SWITCH_PHYSICAL, KillEdge, PhysKillNotifier, SoftwareKill, SoftwareKillLocks};
use titan_link::power;
use titan_rp2040::safety;

use crate::{Interval, Wrap, send};

/// How often `state/kill` is published (`KILLSWITCH_PUBLISH_TIME_MS`).
pub const KILL_PUBLISH_PERIOD: Duration = Duration::from_millis(150);

/// Software kill output level that kills (`SOFTKILL_STATE_KILL`).
const SOFT_KILL_KILL: Level = Level::Low;
/// Software kill output level that lets the kill switch decide (`SOFTKILL_STATE_RUN`).
const SOFT_KILL_RUN: Level = Level::High;

static SOFT_KILL: Mutex<CriticalSectionRawMutex, RefCell<Option<Output<'static>>>> = Mutex::new(RefCell::new(None));
static EDGE: Mutex<CriticalSectionRawMutex, Cell<KillEdge>> = Mutex::new(Cell::new(KillEdge::new()));
/// Set when the kill state flips (`safety_interface_kill_switch_refreshed`).
static REFRESHED: Signal<CriticalSectionRawMutex, ()> = Signal::new();
/// `safety_interface_physical_kill_asserting_kill`
static PHYSICAL_ASSERTING_KILL: AtomicBool = AtomicBool::new(true);
static LOCKS: Mutex<CriticalSectionRawMutex, RefCell<SoftwareKillLocks>> =
    Mutex::new(RefCell::new(SoftwareKillLocks::new()));

/// Creates the software kill output, killing, and hands it to [`on_kill`]/[`on_enable`]. Call before
/// `board::start` so the output is controlled from the first kill callback.
pub fn init_soft_kill(pin: Output<'static>) {
    SOFT_KILL.lock(|p| *p.borrow_mut() = Some(pin));
    drive_soft_kill(SOFT_KILL_KILL);
}

fn drive_soft_kill(level: Level) {
    SOFT_KILL.lock(|p| {
        if let Some(pin) = p.borrow_mut().as_mut() {
            pin.set_level(level);
        }
    });
}

/// The kill callback (`safety_handle_kill`). Runs in a critical section.
pub fn on_kill() {
    drive_soft_kill(SOFT_KILL_KILL);
    let flipped = EDGE.lock(|e| {
        let mut edge = e.get();
        let flipped = edge.on_kill();
        e.set(edge);
        flipped
    });
    if flipped {
        REFRESHED.signal(());
    }
}

/// The enable callback (`safety_handle_enable`). Runs in a critical section.
pub fn on_enable() {
    drive_soft_kill(SOFT_KILL_RUN);
    let flipped = EDGE.lock(|e| {
        let mut edge = e.get();
        let flipped = edge.on_enable();
        e.set(edge);
        flipped
    });
    if flipped {
        REFRESHED.signal(());
    }
}

/// True while the physical kill switch is removed.
pub fn physical_asserting_kill() -> bool {
    PHYSICAL_ASSERTING_KILL.load(Ordering::Relaxed)
}

/// Feeds the physical kill switch (input pulled up, low asserts kill) to the safety core forever.
#[embassy_executor::task]
pub async fn physical_kill_task(mut pin: Input<'static>) -> ! {
    loop {
        // Low asserts kill (`!gpio_get(PHYS_KILLSWITCH_PIN)`)
        let asserting_kill = pin.is_low();
        PHYSICAL_ASSERTING_KILL.store(asserting_kill, Ordering::Relaxed);
        safety::kill_switch_update(KILL_SWITCH_PHYSICAL, asserting_kill, true);
        select(pin.wait_for_any_edge(), Timer::after(safety::TICK)).await;
    }
}

/// Applies a `command/software_kill` message (`software_kill_subscription_callback`), raising `bad_command_fault`
/// if it is refused.
pub fn handle_software_kill(cmd: &power::SoftwareKill, bad_command_fault: u8) {
    let report = safety::kill_switch_report();
    let bit = 1u32.checked_shl(cmd.kill_switch_id as u32).unwrap_or(0);
    let request = SoftwareKill {
        kill_switch_id: cmd.kill_switch_id,
        asserting_kill: cmd.asserting_kill,
        needs_update: cmd.needs_update,
        sender_id: &cmd.sender_id,
    };
    let result =
        LOCKS.lock(|l| l.borrow_mut().check(&request, report.enabled & bit != 0, report.asserting_kill & bit != 0));
    match result {
        Ok(()) => safety::kill_switch_update(cmd.kill_switch_id, cmd.asserting_kill, cmd.needs_update),
        Err(rejected) => {
            defmt::warn!("Refused software kill on switch {}: {}", cmd.kill_switch_id, rejected);
            safety::raise_fault(bad_command_fault, rejected.fault_extra());
        }
    }
}

/// Waits for the kill state to flip. Returns at once if it flipped since the last call.
pub async fn wait_refreshed() {
    REFRESHED.wait().await
}

/// Publishes `state/kill` and `state/physkill_notify` while ROS is connected (`ros_publish_killswitch`).
pub struct KillPublisher {
    interval: Interval,
    notifier: PhysKillNotifier,
}

impl KillPublisher {
    /// Starts publishing; call when ROS connects.
    pub fn new() -> Self {
        KillPublisher { interval: Interval::starting_in(KILL_PUBLISH_PERIOD, true), notifier: PhysKillNotifier::new() }
    }

    /// Publishes if the period elapsed or the kill state flipped (`refreshed`).
    pub fn poll(&mut self, now: Instant, refreshed: bool, wrap: Wrap) {
        // Both checked, as in C: the timer advances even when a refresh triggers the publish
        let fired = self.interval.fire(now);
        if !(fired || refreshed) {
            return;
        }
        send(wrap, power::DeviceToHost::KillState(safety::asserting_kill()));
        if let Some(asserting_kill) = self.notifier.update(physical_asserting_kill()) {
            send(wrap, power::DeviceToHost::PhysicalKillNotify(asserting_kill));
        }
    }
}

impl Default for KillPublisher {
    fn default() -> Self {
        Self::new()
    }
}

/// Waits for the next tick of a publishing loop: `tick` elapsing, or the kill state flipping. Returns true for a
/// flip.
pub async fn next_tick(tick: Duration) -> bool {
    matches!(select(Timer::after(tick), wait_refreshed()).await, Either::Second(()))
}
