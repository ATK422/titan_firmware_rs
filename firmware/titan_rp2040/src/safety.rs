//! Safety core, ported from `titan_safety`.
//!
//! * **Faults**: up to 32 fault bits. Any raised fault turns the status LED red-blinking, sets the error bit in the
//!   CAN heartbeat and is reported in the firmware status. Faults 0-4 are standard across boards; boards number
//!   their own faults from [`FIRST_BOARD_FAULT`].
//! * **Kill switches**: the vehicle is killed unless at least one switch is enabled and none assert kill or have
//!   timed out. Switches are only honoured once [`init`] runs (when ROS connects); [`deinit`] kills.
//! * **Watchdog**: [`safety_task`] feeds the hardware watchdog every [`TICK`]. A task that hogs the executor stops
//!   the feeding and resets the chip. The timeout is 3 s before ROS connects and 250 ms after.
//! * **Crash records**: the fault list and uptime are mirrored into watchdog scratch registers so the reason for a
//!   watchdog or panic reset can be reported on the next boot.

use core::cell::{Cell, RefCell};

use embassy_rp::Peri;
use embassy_rp::peripherals::WATCHDOG;
use embassy_rp::watchdog::{ResetReason, Watchdog};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, Ticker};
use portable_atomic::{AtomicBool, AtomicU32, Ordering};

/// The watchdog reset the chip (or a panic forced a reset) on the previous run.
pub const FAULT_WATCHDOG_RESET: u8 = 0;
/// The safety tick ran dangerously close to the watchdog timeout.
pub const FAULT_WATCHDOG_WARNING: u8 = 1;
/// The CAN controller reported an error.
pub const FAULT_CAN_INTERNAL_ERROR: u8 = 2;
/// The ROS link reported an error.
pub const FAULT_ROS_ERROR: u8 = 3;
/// A critical periodic task missed one or more runs.
pub const FAULT_TIMER_MISSED: u8 = 4;
/// First fault ID available to board specific faults.
pub const FIRST_BOARD_FAULT: u8 = 5;
/// Largest valid fault ID.
pub const MAX_FAULT_ID: u8 = 31;

/// Names of the standard faults, indexed by ID. Boards extend this list in [`SafetyConfig::fault_names`].
pub const STANDARD_FAULT_NAMES: [&str; 5] = [
    "FAULT_WATCHDOG_RESET",
    "FAULT_WATCHDOG_WARNING",
    "FAULT_CAN_INTERNAL_ERROR",
    "FAULT_ROS_ERROR",
    "FAULT_TIMER_MISSED",
];

/// Kill switch driven by the `state/kill` topic.
pub const KILL_SWITCH_ROS: u8 = 0;
/// Number of kill switch slots.
pub const MAX_KILL_SWITCHES: usize = 8;
/// A switch that needs updates times out (asserts kill) this long after its last update.
pub const KILL_SWITCH_TIMEOUT: Duration = Duration::from_millis(500);

/// How often [`safety_task`] runs.
pub const TICK: Duration = Duration::from_millis(10);
const WATCHDOG_SETUP_TIMEOUT: Duration = Duration::from_millis(3000);
const WATCHDOG_ACTIVE_TIMEOUT: Duration = Duration::from_millis(250);
const WATCHDOG_SETUP_WARN_MARGIN: Duration = Duration::from_millis(500);
const WATCHDOG_ACTIVE_WARN_MARGIN: Duration = Duration::from_millis(50);

// Watchdog scratch registers 0-3 hold the crash record. 4-7 belong to the boot ROM.
const SCRATCH_RECORD: usize = 0;
const SCRATCH_FAULTS: usize = 1;
const SCRATCH_UPTIME: usize = 2;
const SCRATCH_PANIC: usize = 3;
const RECORD_MAGIC: u32 = 0x7174;
pub(crate) const PANIC_MARKER: u32 = 0x9A41_C0DE;

/// Board specific safety configuration.
pub struct SafetyConfig {
    /// Fault names indexed by fault ID, used in log messages. Usually [`STANDARD_FAULT_NAMES`] plus board faults.
    pub fault_names: &'static [&'static str],
    /// Called when the vehicle becomes killed. Runs inside a critical section and may run from any context, so it
    /// must be short. This is where actuator boards disable their outputs.
    pub on_kill: fn(),
    /// Called when the vehicle becomes enabled. Same constraints as `on_kill`.
    pub on_enable: fn(),
}

#[derive(Clone, Copy)]
struct FaultRecord {
    time: Instant,
    extra: u32,
    file: &'static str,
    line: u32,
    multiple_fires: bool,
}

#[derive(Clone, Copy)]
struct KillSwitch {
    enabled: bool,
    asserting_kill: bool,
    needs_update: bool,
    update_timeout: Instant,
}

impl KillSwitch {
    const DISABLED: Self =
        KillSwitch { enabled: false, asserting_kill: false, needs_update: false, update_timeout: Instant::MIN };

    fn timed_out(&self, now: Instant) -> bool {
        self.needs_update && now >= self.update_timeout
    }
}

struct KillState {
    switches: [KillSwitch; MAX_KILL_SWITCHES],
    asserting_kill: bool,
    last_change: Instant,
}

fn noop() {}

static FAULT_LIST: AtomicU32 = AtomicU32::new(0);
static FAULT_RECORDS: Mutex<CriticalSectionRawMutex, RefCell<[Option<FaultRecord>; 32]>> =
    Mutex::new(RefCell::new([None; 32]));
static FAULT_NAMES: Mutex<CriticalSectionRawMutex, Cell<&'static [&'static str]>> =
    Mutex::new(Cell::new(&STANDARD_FAULT_NAMES));
/// `(on_kill, on_enable)` from [`SafetyConfig`].
type Hooks = (fn(), fn());
static HOOKS: Mutex<CriticalSectionRawMutex, Cell<Hooks>> = Mutex::new(Cell::new((noop, noop)));

static INITIALIZED: AtomicBool = AtomicBool::new(false);
static KILL: Mutex<CriticalSectionRawMutex, RefCell<KillState>> = Mutex::new(RefCell::new(KillState {
    switches: [KillSwitch::DISABLED; MAX_KILL_SWITCHES],
    asserting_kill: true,
    last_change: Instant::MIN,
}));

static CRASH_COUNTS: Mutex<CriticalSectionRawMutex, Cell<CrashCounts>> =
    Mutex::new(Cell::new(CrashCounts { watchdog_timeouts: 0, panics: 0 }));

/// Resets recorded since power on.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct CrashCounts {
    pub watchdog_timeouts: u8,
    pub panics: u8,
}

// ========================================
// Faults
// ========================================

/// Looks up the name of a fault for logging.
pub fn fault_name(id: u8) -> &'static str {
    FAULT_NAMES.lock(|names| names.get().get(id as usize).copied().unwrap_or("UNKNOWN"))
}

/// Raises a fault, recording `extra` and the caller's location. Safe to call from any context.
#[track_caller]
pub fn raise_fault(id: u8, extra: u32) {
    assert!(id <= MAX_FAULT_ID, "invalid fault id");
    let location = core::panic::Location::caller();
    let newly_raised = FAULT_RECORDS.lock(|records| {
        let mut records = records.borrow_mut();
        let previous = records[id as usize];
        let was_raised = FAULT_LIST.load(Ordering::Relaxed) & (1 << id) != 0;
        records[id as usize] = Some(FaultRecord {
            time: Instant::now(),
            extra,
            file: location.file(),
            line: location.line(),
            multiple_fires: previous.is_some(),
        });
        FAULT_LIST.fetch_or(1 << id, Ordering::Relaxed);
        !was_raised
    });
    if newly_raised {
        defmt::error!(
            "Fault {} raised (arg {=u32:#x}) at {}:{}",
            fault_name(id),
            extra,
            location.file(),
            location.line()
        );
    }
}

/// Lowers a fault. Its record stays available for debugging.
pub fn lower_fault(id: u8) {
    assert!(id <= MAX_FAULT_ID, "invalid fault id");
    if FAULT_LIST.fetch_and(!(1 << id), Ordering::Relaxed) & (1 << id) != 0 {
        defmt::info!("Fault {} lowered", fault_name(id));
    }
}

/// Bitmask of the currently raised faults.
pub fn faults() -> u32 {
    FAULT_LIST.load(Ordering::Relaxed)
}

/// Logs every fault that has ever fired, like `safety_print_crash_log` did for the fault table.
pub fn log_fault_records() {
    FAULT_RECORDS.lock(|records| {
        for (id, record) in records.borrow().iter().enumerate() {
            if let Some(r) = record {
                let active = faults() & (1 << id) != 0;
                defmt::info!(
                    "{} {}: arg {=u32:#x} at {}:{} ({} ms){}",
                    if active { "ACTIVE" } else { "cleared" },
                    fault_name(id as u8),
                    r.extra,
                    r.file,
                    r.line,
                    r.time.as_millis(),
                    if r.multiple_fires { ", fired multiple times" } else { "" }
                );
            }
        }
    });
}

// ========================================
// Kill switches
// ========================================

fn handle_kill(state: &mut KillState) {
    state.asserting_kill = true;
    state.last_change = Instant::now();
    (HOOKS.lock(Cell::get).0)();
}

/// Updates a kill switch, enabling it if it was not already.
///
/// `needs_update` makes the switch assert kill if it is not updated again within [`KILL_SWITCH_TIMEOUT`].
pub fn kill_switch_update(switch: u8, asserting_kill: bool, needs_update: bool) {
    assert!((switch as usize) < MAX_KILL_SWITCHES, "invalid kill switch");
    KILL.lock(|state| {
        let mut state = state.borrow_mut();
        state.switches[switch as usize] = KillSwitch {
            enabled: true,
            asserting_kill,
            needs_update,
            update_timeout: Instant::now() + KILL_SWITCH_TIMEOUT,
        };
        if INITIALIZED.load(Ordering::Relaxed) && asserting_kill {
            handle_kill(&mut state);
        }
    });
}

fn refresh_kill_switches() {
    let now = Instant::now();
    KILL.lock(|state| {
        let mut state = state.borrow_mut();
        let mut enabled = 0;
        let mut asserting_kill = false;
        for switch in state.switches.iter().filter(|s| s.enabled) {
            enabled += 1;
            if switch.asserting_kill || switch.timed_out(now) {
                asserting_kill = true;
                break;
            }
        }
        if enabled == 0 {
            asserting_kill = true;
        }

        if state.asserting_kill != asserting_kill {
            if asserting_kill {
                handle_kill(&mut state);
            } else {
                state.asserting_kill = false;
                state.last_change = now;
                (HOOKS.lock(Cell::get).1)();
            }
        }
    });
}

/// True if the vehicle is currently killed.
pub fn asserting_kill() -> bool {
    KILL.lock(|state| state.borrow().asserting_kill)
}

/// When the kill state last changed.
pub fn kill_last_change() -> Instant {
    KILL.lock(|state| state.borrow().last_change)
}

/// Kill switch bitmasks as reported in the firmware status.
#[derive(Clone, Copy, Default, Debug)]
pub struct KillSwitchReport {
    pub enabled: u32,
    pub asserting_kill: u32,
    pub needs_update: u32,
    pub timed_out: u32,
}

pub fn kill_switch_report() -> KillSwitchReport {
    let now = Instant::now();
    KILL.lock(|state| {
        let mut report = KillSwitchReport::default();
        for (i, s) in state.borrow().switches.iter().enumerate() {
            let bit = 1 << i;
            if s.enabled {
                report.enabled |= bit;
            }
            if s.asserting_kill {
                report.asserting_kill |= bit;
            }
            if s.needs_update {
                report.needs_update |= bit;
            }
            if s.timed_out(now) {
                report.timed_out |= bit;
            }
        }
        report
    })
}

// ========================================
// Lifetime
// ========================================

/// Owns the watchdog. Created by [`setup`] and handed to [`safety_task`].
pub struct Safety {
    watchdog: Watchdog,
}

/// Starts the watchdog and evaluates the crash record. Call this first thing in `main`.
pub fn setup(watchdog: Peri<'static, WATCHDOG>, config: SafetyConfig) -> Safety {
    FAULT_NAMES.lock(|names| names.set(config.fault_names));
    HOOKS.lock(|hooks| hooks.set((config.on_kill, config.on_enable)));

    let mut watchdog = Watchdog::new(watchdog);
    let reason = watchdog.reset_reason();
    let record = watchdog.get_scratch(SCRATCH_RECORD);
    let mut counts = if record >> 16 == RECORD_MAGIC {
        CrashCounts { watchdog_timeouts: (record & 0xFF) as u8, panics: (record >> 8) as u8 }
    } else {
        CrashCounts { watchdog_timeouts: 0, panics: 0 }
    };
    let valid_record = record >> 16 == RECORD_MAGIC;
    let last_faults = watchdog.get_scratch(SCRATCH_FAULTS);
    let last_uptime = watchdog.get_scratch(SCRATCH_UPTIME);
    let panicked = watchdog.get_scratch(SCRATCH_PANIC) == PANIC_MARKER;

    // The panic handler marks scratch3 before anything else, so a watchdog timeout with the marker set is a panic
    // whose handler never got to its forced reset (it stalled, or faulted while logging)
    match reason {
        Some(ResetReason::Forced) if panicked => {
            counts.panics = counts.panics.wrapping_add(1);
            defmt::error!("Reset by panic after {} ms (faults {=u32:#x})", last_uptime, last_faults);
            raise_fault(FAULT_WATCHDOG_RESET, last_faults);
        }
        Some(ResetReason::TimedOut) if panicked => {
            counts.panics = counts.panics.wrapping_add(1);
            defmt::error!(
                "Reset by panic after {} ms (faults {=u32:#x}); the watchdog expired inside the panic handler",
                last_uptime,
                last_faults
            );
            raise_fault(FAULT_WATCHDOG_RESET, last_faults);
        }
        Some(ResetReason::TimedOut) if valid_record => {
            counts.watchdog_timeouts = counts.watchdog_timeouts.wrapping_add(1);
            defmt::error!("Watchdog timeout after {} ms (faults {=u32:#x})", last_uptime, last_faults);
            raise_fault(FAULT_WATCHDOG_RESET, last_faults);
        }
        Some(ResetReason::TimedOut) => {
            defmt::error!("Watchdog timeout with no crash record");
            raise_fault(FAULT_WATCHDOG_RESET, 0);
        }
        _ => {}
    }
    CRASH_COUNTS.lock(|c| c.set(counts));
    defmt::info!("Crash counts since power on: {}", counts);

    watchdog.set_scratch(
        SCRATCH_RECORD,
        RECORD_MAGIC << 16 | (counts.panics as u32) << 8 | counts.watchdog_timeouts as u32,
    );
    watchdog.set_scratch(SCRATCH_PANIC, 0);
    watchdog.pause_on_debug(true);
    watchdog.start(WATCHDOG_SETUP_TIMEOUT);

    Safety { watchdog }
}

/// Resets recorded since power on.
pub fn crash_counts() -> CrashCounts {
    CRASH_COUNTS.lock(Cell::get)
}

/// True once [`init`] has run and kill switches are being honoured.
pub fn is_initialized() -> bool {
    INITIALIZED.load(Ordering::Relaxed)
}

/// Starts honouring kill switches and tightens the watchdog. Called by the link when ROS connects.
pub fn init() {
    KILL.lock(|state| state.borrow_mut().last_change = Instant::now());
    INITIALIZED.store(true, Ordering::Relaxed);
}

/// Kills the vehicle and relaxes the watchdog. Called by the link when ROS disconnects.
pub fn deinit() {
    INITIALIZED.store(false, Ordering::Relaxed);
    KILL.lock(|state| handle_kill(&mut state.borrow_mut()));
}

/// Runs the safety tick forever: kill switch timeouts, crash record and watchdog.
#[embassy_executor::task]
pub async fn safety_task(mut safety: Safety) -> ! {
    let mut ticker = Ticker::every(TICK);
    let mut last_feed = Instant::now();
    // The timeout the watchdog was last fed with, and the warning margin that goes with it
    let (mut timeout, mut margin) = (WATCHDOG_SETUP_TIMEOUT, WATCHDOG_SETUP_WARN_MARGIN);
    loop {
        ticker.next().await;
        let now = Instant::now();

        if is_initialized() {
            refresh_kill_switches();
        }

        safety.watchdog.set_scratch(SCRATCH_FAULTS, faults());
        safety.watchdog.set_scratch(SCRATCH_UPTIME, now.as_millis() as u32);

        // Check how close the countdown that is running now came to the end, against the margin for its own
        // timeout. Using the margin of the timeout about to be fed raised a warning on every ROS disconnect: the
        // 500 ms setup margin against the 250 ms active countdown.
        let elapsed = now - last_feed;
        if elapsed + margin > timeout {
            let remaining = timeout.checked_sub(elapsed).unwrap_or(Duration::from_ticks(0));
            raise_fault(FAULT_WATCHDOG_WARNING, remaining.as_micros() as u32);
        }
        (timeout, margin) = if is_initialized() {
            (WATCHDOG_ACTIVE_TIMEOUT, WATCHDOG_ACTIVE_WARN_MARGIN)
        } else {
            (WATCHDOG_SETUP_TIMEOUT, WATCHDOG_SETUP_WARN_MARGIN)
        };
        safety.watchdog.feed(timeout);
        last_feed = now;
    }
}
