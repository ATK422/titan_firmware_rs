//! The thruster control loop on core 1, ported from `core1.c`.
//!
//! Core 1 runs a plain blocking loop (no executor), one tick per DShot frame, at most every
//! [`DSHOT_MIN_FRAME_TIME_US`]. The decisions are made by [`esc_core::control_loop::ControlLoop`]; this module does
//! the I/O around it and the hand-off with core 0:
//! * target RPMs: [`update_target_rpm`] (shared under a lock, with a 500 ms expiry)
//! * tuning parameters: [`set_param`] (a queue drained at the start of each tick, replacing the multicore FIFO,
//!   which embassy-rp reserves for pausing core 1)
//! * telemetry: [`telemetry`]
//! * kill: [`on_kill`] latches a flag the next tick sees even if the kill was brief; the loop also polls the kill
//!   state every tick
//!
//! Like `safety_core1_checkin`, core 1 checks in every tick and [`core1_watchdog_task`] on core 0 resets the board
//! with a `CORE1_TIMEOUT` crash record if it stops for 250 ms.

use core::cell::{Cell, RefCell};

use embassy_rp::Peri;
use embassy_rp::multicore::{Stack, spawn_core1};
use embassy_rp::peripherals::CORE1;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Ticker};
use portable_atomic::{AtomicBool, AtomicU64, Ordering};

use esc_core::control_loop::{ControlLoop, ControllerTelemetry, Inputs, Param};
use esc_core::{DSHOT_MIN_FRAME_TIME_US, DSHOT_MIN_UPDATE_RATE_MS, NUM_THRUSTERS};
use titan_rp2040::safety;

use crate::dshot::Dshot;
use crate::{FAULT_RAW_MODE, FAULT_RPM_CMD_TIMEOUT, vcc};

/// Core 1 must check in at least this often (`SAFETY_CORE1_CHECKIN_INTERVAL_MS`).
const CHECKIN_INTERVAL: Duration = Duration::from_millis(250);

#[derive(Clone, Copy)]
struct Target {
    rpm: [i16; NUM_THRUSTERS],
    /// The target is stale from this time on.
    expiration: Instant,
}

/// Starts out expired, so the thrusters stay off until the first command.
static TARGET: Mutex<CriticalSectionRawMutex, Cell<Target>> =
    Mutex::new(Cell::new(Target { rpm: [0; NUM_THRUSTERS], expiration: Instant::from_ticks(0) }));
static PARAMS: Channel<CriticalSectionRawMutex, Param, 16> = Channel::new();
static TELEMETRY: Mutex<CriticalSectionRawMutex, RefCell<ControllerTelemetry>> =
    Mutex::new(RefCell::new(ControllerTelemetry::new()));
static KILL_LATCH: AtomicBool = AtomicBool::new(false);
static CORE1_RUNNING: AtomicBool = AtomicBool::new(false);
/// `Instant` ticks of core 1's last check-in.
static LAST_CHECKIN: AtomicU64 = AtomicU64::new(0);

static mut CORE1_STACK: Stack<8192> = Stack::new();

/// Sets new target RPMs (or raw throttle in raw mode) for this board's thrusters (`core1_update_target_rpm`). They
/// expire after [`DSHOT_MIN_UPDATE_RATE_MS`].
pub fn update_target_rpm(rpm: [i16; NUM_THRUSTERS]) {
    let expiration = Instant::now() + Duration::from_millis(DSHOT_MIN_UPDATE_RATE_MS);
    TARGET.lock(|t| t.set(Target { rpm, expiration }));
}

/// Queues a tuning parameter. Returns false if the value is out of range or the queue is full.
pub fn set_param(param: Param) -> bool {
    let Some(param) = param.validate() else {
        defmt::warn!("Ignoring out of range controller parameter {}", param);
        return false;
    };
    if PARAMS.try_send(param).is_err() {
        defmt::warn!("Controller parameter queue full, dropping {}", param);
        return false;
    }
    true
}

/// Copies the control loop telemetry, optionally clearing its counters (`core1_get_telem`).
pub fn telemetry(clear: bool) -> ControllerTelemetry {
    TELEMETRY.lock(|t| t.borrow_mut().snapshot(clear))
}

/// `BoardConfig::on_kill`: makes the next control loop tick (within about half a millisecond) send neutral.
pub fn on_kill() {
    KILL_LATCH.store(true, Ordering::Relaxed);
}

/// Starts the control loop on core 1 with the default tuning (`core1_init`).
pub fn launch(core1: Peri<'static, CORE1>, dshot: Dshot) {
    for param in Param::DEFAULTS {
        set_param(param);
    }
    LAST_CHECKIN.store(Instant::now().as_ticks(), Ordering::Relaxed);
    CORE1_RUNNING.store(true, Ordering::Relaxed);
    // SAFETY: the only reference ever taken to the stack, handed to core 1 for good
    let stack = unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) };
    spawn_core1(core1, stack, move || core1_main(dshot));
}

/// Panics if core 1 stops checking in (`safety_internal_multicore_tick`).
#[embassy_executor::task]
pub async fn core1_watchdog_task() -> ! {
    let mut ticker = Ticker::every(safety::TICK);
    loop {
        ticker.next().await;
        if CORE1_RUNNING.load(Ordering::Relaxed) {
            let last = Instant::from_ticks(LAST_CHECKIN.load(Ordering::Relaxed));
            let silent = Instant::now().saturating_duration_since(last);
            if silent > CHECKIN_INTERVAL {
                // Recorded as CORE1_TIMEOUT in the crash log, like `safety_internal_multicore_tick`
                titan_rp2040::crash::core1_timeout(silent.as_millis() as u32);
            }
        }
    }
}

fn core1_main(mut dshot: Dshot) -> ! {
    defmt::info!("Control loop running on core 1");
    let mut control = ControlLoop::new(Instant::now().as_micros());

    loop {
        LAST_CHECKIN.store(Instant::now().as_ticks(), Ordering::Relaxed);

        // Phase 1: copy in the external commands
        let target = TARGET.lock(Cell::get);
        let target_expired = Instant::now() >= target.expiration;
        while let Ok(param) = PARAMS.try_receive() {
            match control.apply_param(param) {
                Some(true) => safety::raise_fault(FAULT_RAW_MODE, 0),
                Some(false) => safety::lower_fault(FAULT_RAW_MODE),
                None => {}
            }
        }

        // Phases 2-3: kill conditions and the commands to send
        let killed = KILL_LATCH.swap(false, Ordering::Relaxed) | safety::asserting_kill();
        let esc_board_off = vcc::take_was_off() | !vcc::esc_board_on();
        let now = Instant::now();
        let tick = control
            .begin_tick(now.as_micros(), Inputs { killed, esc_board_off, target_rpm: target.rpm, target_expired });
        if tick.command_timed_out {
            safety::raise_fault(FAULT_RPM_CMD_TIMEOUT, 0);
        }

        // Phase 4: transmit
        dshot.send(&tick.commands);
        let min_frame_time = Instant::now() + Duration::from_micros(DSHOT_MIN_FRAME_TIME_US);

        // Phase 5: collect the eRPM replies until all arrived or the frame time is up. `serviced` gives one more
        // pass after a slow controller tick, so a reply that arrived meanwhile isn't dropped.
        let mut waiting: u8 = (1 << NUM_THRUSTERS) - 1;
        let mut rpm = [None; NUM_THRUSTERS];
        let mut serviced = false;
        while waiting != 0 && (Instant::now() < min_frame_time || serviced) {
            serviced = false;
            for (i, rpm) in rpm.iter_mut().enumerate() {
                if !dshot.rpm_available(i) {
                    continue;
                }
                // Decode even if this thruster was already serviced, to drain noise from the FIFO
                if let Some(value) = dshot.read_rpm(i)
                    && waiting & (1 << i) != 0
                {
                    *rpm = Some(value);
                    control.on_rpm(i, value, Instant::now().as_micros());
                }
                waiting &= !(1 << i);
                serviced = true;
            }
        }

        // Phase 6: telemetry writeback
        TELEMETRY.lock(|t| control.end_tick(rpm, &mut t.borrow_mut()));

        // Phase 7: don't send the next frame too early
        while Instant::now() < min_frame_time {}
    }
}
