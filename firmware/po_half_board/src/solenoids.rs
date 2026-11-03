//! Solenoid and pump outputs: the hardware side of `solenoid.c`. The sequencing is `power_core::solenoid`.
//!
//! [`solenoid_task`] runs the state machine whenever it has a deadline or is woken by a command or a kill, and
//! drives the four outputs after every step. The C code ran `solenoid_tick` on every pass of its main loop.

use core::cell::RefCell;

use embassy_futures::select::select;
use embassy_rp::gpio::{Level, Output};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Instant, Timer};
use power_core::solenoid::{SOLENOID_COUNT, Solenoids};
use titan_rp2040::safety;

static SOLENOIDS: Mutex<CriticalSectionRawMutex, RefCell<Solenoids>> = Mutex::new(RefCell::new(Solenoids::new()));
static WAKE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Starts the kill routine (`solenoid_start_kill_routine`). Called from the kill callback, in a critical section.
pub fn start_kill_routine() {
    SOLENOIDS.lock(|s| s.borrow_mut().start_kill_routine());
    WAKE.signal(());
}

/// Opens or closes an output from ROS (`solenoid_set`); ignored while a sequence runs.
pub fn set(index: usize, open: bool) {
    if !SOLENOIDS.lock(|s| s.borrow_mut().set(index, open)) {
        defmt::info!("Solenoid {} command ignored: busy or invalid", index);
    }
    WAKE.signal(());
}

/// Requests a cycle with the pressure solenoid open for `pressure_ms` (`solenoid_request_cycle`).
pub fn request_cycle(pressure_ms: i32) {
    let killed = safety::asserting_kill();
    SOLENOIDS.lock(|s| s.borrow_mut().request_cycle(pressure_ms, killed));
    WAKE.signal(());
}

/// Output states for `state/solenoid/*` and `state/pump` (`solenoid_get`).
pub fn states() -> [bool; SOLENOID_COUNT] {
    SOLENOIDS.lock(|s| s.borrow().states())
}

/// Drives the outputs (water, pressure, exhaust, pump; created closed) forever.
#[embassy_executor::task]
pub async fn solenoid_task(mut outputs: [Output<'static>; SOLENOID_COUNT]) -> ! {
    loop {
        let now = Instant::now();
        let (states, deadline) = SOLENOIDS.lock(|s| {
            let mut s = s.borrow_mut();
            s.tick(now.as_millis());
            (s.states(), s.next_deadline())
        });
        for (output, open) in outputs.iter_mut().zip(states) {
            output.set_level(Level::from(open));
        }
        let wake_at = deadline.map_or(Instant::MAX, Instant::from_millis);
        select(Timer::at(wake_at), WAKE.wait()).await;
    }
}
