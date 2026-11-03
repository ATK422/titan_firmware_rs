//! Lilboat PoHalf solenoids and pump, ported from `lilboat/PoHalfBoard/src/solenoid.c`.
//!
//! Four outputs: the water, pressure and exhaust solenoids and the pump. ROS can open and close each one directly
//! unless a sequence is running. Two sequences take over the outputs:
//!
//! * a **cycle** (`command/fart_for_ms`): close everything, wait [`SPACING_MS`], open the pressure solenoid for the
//!   requested time, close, wait [`SPACING_MS`], open the water solenoid for [`WATER_OPEN_MS`], close
//! * the **kill routine**, started by every kill callback: [`KILL_NUM_CYCLES`] cycles back to back with the
//!   pressure solenoid open for [`KILL_PRESSURE_OPEN_MS`]
//!
//! The C code ran `solenoid_tick` on every pass of the main loop. [`Solenoids::tick`] is the same state machine
//! with an explicit clock, and [`Solenoids::next_deadline`] says when it next has work, so the firmware can sleep in
//! between.
//!
//! Quirks kept from C:
//! * The safety core calls the kill callback for every kill switch update that asserts kill (the physical switch
//!   alone does so every 10 ms), and a new kill routine starts whenever none is active. So while the vehicle stays
//!   killed, the kill routine repeats.
//! * The cycle length comes from a signed `Int32` and is used as unsigned, so a negative request holds the pressure
//!   solenoid open for about 49 days (until the next kill or reset).

/// Number of outputs.
pub const SOLENOID_COUNT: usize = 4;
/// Index of the water solenoid (`WATER_SOLENOID_NUM`).
pub const WATER: usize = 0;
/// Index of the pressure solenoid (`PRESSURE_SOLENOID_NUM`).
pub const PRESSURE: usize = 1;
/// Index of the exhaust solenoid (`EXHAUST_SOLENOID_NUM`).
pub const EXHAUST: usize = 2;
/// Index of the pump (`PUMP_ACCESS_NUM`).
pub const PUMP: usize = 3;

/// Pressure solenoid open time during the kill routine (`KILL_PRESSURE_OPEN_TIME_MS`).
pub const KILL_PRESSURE_OPEN_MS: u32 = 1000;
/// Water solenoid open time in every cycle (`KILL_WATER_OPEN_TIME_MS`).
pub const WATER_OPEN_MS: u64 = 3000;
/// Pause with everything closed before each solenoid opens (`KILL_SOLENOID_SPACING_TIME_MS`).
pub const SPACING_MS: u64 = 100;
/// Cycles in one kill routine (`KILL_NUM_CYCLES`).
pub const KILL_NUM_CYCLES: u32 = 5;

/// The solenoid state machine. Times are milliseconds on any monotonic clock; `None` is "never".
#[derive(Clone, Debug)]
pub struct Solenoids {
    states: [bool; SOLENOID_COUNT],

    kill_requested: bool,
    kill_active: bool,
    num_cycles: u32,

    cycle_requested: bool,
    cycle_active: bool,
    cycle_idx: u32,
    cycle_pressure_ms: u32,

    next_pressure: Option<u64>,
    next_water: Option<u64>,
    next_wait: Option<u64>,
}

fn reached(deadline: Option<u64>, now: u64) -> bool {
    deadline.is_some_and(|t| now >= t)
}

impl Solenoids {
    /// Everything closed and idle (`solenoid_init`).
    pub const fn new() -> Self {
        Solenoids {
            states: [false; SOLENOID_COUNT],
            kill_requested: false,
            kill_active: false,
            num_cycles: 0,
            cycle_requested: false,
            cycle_active: false,
            cycle_idx: 0,
            cycle_pressure_ms: 0,
            next_pressure: None,
            next_water: None,
            next_wait: None,
        }
    }

    /// The level each output should be driven to (open = high).
    pub fn states(&self) -> [bool; SOLENOID_COUNT] {
        self.states
    }

    /// Whether output `index` is open (`solenoid_get`).
    pub fn get(&self, index: usize) -> bool {
        self.states.get(index).copied().unwrap_or(false)
    }

    /// True while a cycle or the kill routine owns the outputs.
    pub fn busy(&self) -> bool {
        self.kill_active || self.kill_requested || self.cycle_active || self.cycle_requested
    }

    fn set_ignoring_kill(&mut self, index: usize, open: bool) {
        if let Some(state) = self.states.get_mut(index) {
            *state = open;
        }
    }

    fn close_all(&mut self) {
        self.states = [false; SOLENOID_COUNT];
    }

    /// Opens or closes output `index` from ROS (`solenoid_set`). Ignored while a sequence runs or for an invalid
    /// index. Returns whether it was applied.
    pub fn set(&mut self, index: usize, open: bool) -> bool {
        if self.busy() || index >= SOLENOID_COUNT {
            return false;
        }
        self.set_ignoring_kill(index, open);
        true
    }

    /// Starts the kill routine unless one is running (`solenoid_start_kill_routine`). Cancels any cycle. Safe to
    /// call from the kill callback; the work happens in the next [`Solenoids::tick`].
    pub fn start_kill_routine(&mut self) {
        if self.kill_active {
            return;
        }
        self.kill_requested = true;
        self.cycle_active = false;
        self.cycle_requested = false;
        self.cycle_pressure_ms = KILL_PRESSURE_OPEN_MS;
    }

    fn request_cycle_ignoring_kill(&mut self, pressure_ms: u32) {
        if self.cycle_active || self.cycle_requested {
            return;
        }
        self.cycle_pressure_ms = pressure_ms;
        self.cycle_requested = true;
    }

    /// Requests a cycle with the pressure solenoid open for `pressure_ms` (`solenoid_request_cycle`). Ignored while
    /// the vehicle is killed or a cycle is already pending or running.
    pub fn request_cycle(&mut self, pressure_ms: i32, vehicle_killed: bool) {
        if vehicle_killed {
            return;
        }
        // The C code stored the signed request in an unsigned variable
        self.request_cycle_ignoring_kill(pressure_ms as u32);
    }

    fn tick_kill(&mut self) {
        if self.kill_requested && !self.kill_active {
            self.num_cycles = 0;
            self.close_all();
            self.kill_active = true;
            self.kill_requested = false;
        }

        if self.num_cycles >= KILL_NUM_CYCLES {
            self.kill_active = false;
        }

        if self.kill_active && !self.cycle_active && !self.cycle_requested {
            self.request_cycle_ignoring_kill(KILL_PRESSURE_OPEN_MS);
            self.num_cycles += 1;
        }
    }

    fn tick_cycle(&mut self, now: u64) {
        if self.cycle_requested && !self.cycle_active {
            self.cycle_idx = 0;
            self.close_all();
            self.next_water = None;
            self.next_pressure = None;
            self.next_wait = Some(now);
            self.cycle_active = true;
            self.cycle_requested = false;
        }

        if reached(self.next_wait, now) {
            self.close_all();
            match self.cycle_idx {
                0 => {
                    self.next_pressure = Some(now + SPACING_MS);
                    self.next_water = None;
                }
                1 => {
                    self.next_water = Some(now + SPACING_MS);
                    self.next_pressure = None;
                }
                _ => self.cycle_active = false,
            }
            self.next_wait = None;
        }

        if reached(self.next_pressure, now) {
            self.set_ignoring_kill(PRESSURE, true);
            self.next_wait = Some(now + self.cycle_pressure_ms as u64);
            self.cycle_idx += 1;
            self.next_pressure = None;
        }

        if reached(self.next_water, now) {
            self.set_ignoring_kill(WATER, true);
            self.next_wait = Some(now + WATER_OPEN_MS);
            self.cycle_idx += 1;
            self.next_water = None;
        }
    }

    /// Advances the sequences to `now` (`solenoid_tick`).
    pub fn tick(&mut self, now: u64) {
        self.tick_kill();
        self.tick_cycle(now);
    }

    /// When [`Solenoids::tick`] next has something to do. `Some(0)` means straight away, `None` means not until
    /// another call changes the state.
    pub fn next_deadline(&self) -> Option<u64> {
        if self.kill_requested || self.cycle_requested || (self.kill_active && !self.cycle_active) {
            return Some(0);
        }
        [self.next_pressure, self.next_water, self.next_wait].into_iter().flatten().min()
    }
}

impl Default for Solenoids {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use std::vec::Vec;

    use super::*;

    /// Runs the state machine like the firmware does, recording every change of the outputs until `end`.
    fn run(s: &mut Solenoids, start: u64, end: u64) -> Vec<(u64, [bool; SOLENOID_COUNT])> {
        let mut changes = Vec::new();
        let mut now = start;
        let mut last = s.states();
        loop {
            s.tick(now);
            if s.states() != last {
                last = s.states();
                changes.push((now, last));
            }
            match s.next_deadline() {
                Some(t) if t <= end => now = t.max(now),
                _ => break,
            }
        }
        changes
    }

    const CLOSED: [bool; 4] = [false; 4];
    const PRESSURE_OPEN: [bool; 4] = [false, true, false, false];
    const WATER_OPEN: [bool; 4] = [true, false, false, false];

    #[test]
    fn direct_control_when_idle() {
        let mut s = Solenoids::new();
        assert!(s.set(EXHAUST, true));
        assert!(s.set(PUMP, true));
        assert_eq!(s.states(), [false, false, true, true]);
        assert!(s.get(EXHAUST));
        assert!(!s.set(4, true));
        assert_eq!(s.next_deadline(), None);
    }

    #[test]
    fn cycle_sequence() {
        let mut s = Solenoids::new();
        s.set(PUMP, true);
        s.request_cycle(500, false);
        assert_eq!(s.next_deadline(), Some(0));
        let changes = run(&mut s, 1000, 100_000);
        assert_eq!(
            changes,
            [(1000, CLOSED), (1100, PRESSURE_OPEN), (1600, CLOSED), (1700, WATER_OPEN), (4700, CLOSED),]
        );
        assert!(!s.busy());
        assert!(s.set(WATER, true));
    }

    #[test]
    fn commands_ignored_during_cycle() {
        let mut s = Solenoids::new();
        s.request_cycle(200, false);
        s.tick(0);
        assert!(!s.set(EXHAUST, true));
        // A second request while one runs is dropped
        s.request_cycle(5000, false);
        let changes = run(&mut s, 0, 100_000);
        assert_eq!(changes, [(100, PRESSURE_OPEN), (300, CLOSED), (400, WATER_OPEN), (3400, CLOSED)]);
    }

    #[test]
    fn no_cycle_while_killed() {
        let mut s = Solenoids::new();
        s.request_cycle(200, true);
        assert_eq!(s.next_deadline(), None);
        assert!(!s.busy());
    }

    #[test]
    fn kill_routine_runs_five_cycles() {
        let mut s = Solenoids::new();
        s.set(EXHAUST, true);
        s.start_kill_routine();
        assert!(!s.set(PUMP, true));
        let changes = run(&mut s, 0, 1_000_000);

        let mut expected = Vec::new();
        expected.push((0, CLOSED));
        // Each cycle: 100 ms closed, 1 s pressure, 100 ms closed, 3 s water
        for cycle in 0..KILL_NUM_CYCLES as u64 {
            let t = cycle * 4200;
            expected.push((t + 100, PRESSURE_OPEN));
            expected.push((t + 1100, CLOSED));
            expected.push((t + 1200, WATER_OPEN));
            expected.push((t + 4200, CLOSED));
        }
        assert_eq!(changes, expected);
        assert!(!s.busy());
        assert_eq!(s.next_deadline(), None);
    }

    #[test]
    fn kill_cancels_cycle_and_ignores_repeats() {
        let mut s = Solenoids::new();
        s.request_cycle(10_000, false);
        s.tick(0);
        s.tick(100);
        assert_eq!(s.states(), PRESSURE_OPEN);

        s.start_kill_routine();
        s.tick(150);
        assert_eq!(s.states(), CLOSED);
        // The safety core keeps calling the kill callback; the routine isn't restarted while active
        for t in (160..2000).step_by(10) {
            s.start_kill_routine();
            s.tick(t);
        }
        // First kill cycle: pressure opened at 250 for the kill time, not the 10 s request
        assert_eq!(s.states(), WATER_OPEN);
    }

    #[test]
    fn kill_routine_repeats_while_killed() {
        let mut s = Solenoids::new();
        s.start_kill_routine();
        run(&mut s, 0, 21_000);
        assert!(!s.busy());
        // The next kill callback (the physical switch updates every safety tick) starts another routine
        s.start_kill_routine();
        assert_eq!(s.next_deadline(), Some(0));
        s.tick(21_010);
        assert!(s.busy());
    }

    #[test]
    fn negative_cycle_length_wraps() {
        let mut s = Solenoids::new();
        s.request_cycle(-1, false);
        s.tick(0);
        s.tick(100);
        assert_eq!(s.states(), PRESSURE_OPEN);
        assert_eq!(s.next_deadline(), Some(100 + u32::MAX as u64));
    }
}
