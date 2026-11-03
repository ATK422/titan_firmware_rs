//! Original actuators (`actuators_v1`): coil gun torpedoes, solenoid droppers and a DC motor claw, all timed.
//!
//! This is the hardware independent part: timing bookkeeping, the capacitor charge hysteresis, the claw's state
//! machine and the status rules. The firmware owns the pins, the PIO coil sequencer, the ADC and the timers.

use titan_link::actuator::{ActuatorError, ClawState, DropperState, TorpedoState, TorpedoTiming};

/// Torpedoes per load (one select line each).
pub const NUM_TORPEDOES: usize = 2;
/// Coils per torpedo.
pub const NUM_COILS: usize = 3;
/// Timings per torpedo: on, delay, on, delay, on.
pub const NUM_TIMINGS: usize = 5;
const _: () = assert!(NUM_TIMINGS + 1 == NUM_COILS * 2);
/// Droppers per load.
pub const NUM_DROPPERS: usize = 2;

/// Microseconds the PIO program spends pulling and loading a coil-on time, taken off the requested time.
pub const ON_SUBTRACT: u16 = 2;
/// The same for a delay between coils.
pub const OFF_SUBTRACT: u16 = 4;

/// Below this the capacitor counts as discharged again (hysteresis).
pub const TORPEDO_DISCHARGED_THRESHOLD: f32 = 20.25;
/// Above this the capacitor counts as charged.
pub const TORPEDO_CHARGED_THRESHOLD: f32 = 21.75;
/// ADC counts per volt on the charge level divider.
pub const TORPEDO_VDIV_CAL: f32 = 160.5022;

/// Index of a timing in the PIO sequence.
pub fn timing_index(timing: TorpedoTiming) -> usize {
    match timing {
        TorpedoTiming::Coil1On => 0,
        TorpedoTiming::Coil1To2Delay => 1,
        TorpedoTiming::Coil2On => 2,
        TorpedoTiming::Coil2To3Delay => 3,
        TorpedoTiming::Coil3On => 4,
    }
}

/// Converts a raw 12-bit ADC reading of the charge level pin to volts (`torpedo_adc_cb`).
pub fn charge_volts(raw: u16) -> f32 {
    raw as f32 / TORPEDO_VDIV_CAL
}

/// PIO loop counts for each torpedo's coil sequence (`torpedo_timings`). Zero means not set.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TorpedoTimings {
    counts: [[u16; NUM_TIMINGS]; NUM_TORPEDOES],
}

impl TorpedoTimings {
    pub const fn new() -> Self {
        TorpedoTimings { counts: [[0; NUM_TIMINGS]; NUM_TORPEDOES] }
    }

    /// Sets one timing of torpedo `torpedo_num` (1-based) to `time_us` (`torpedo_set_timings`). Refused while
    /// armed, for unknown torpedoes and for times shorter than the PIO overhead.
    pub fn set(&mut self, torpedo_num: u8, timing: TorpedoTiming, time_us: u16, armed: bool) -> bool {
        if torpedo_num < 1 || torpedo_num as usize > NUM_TORPEDOES {
            return false;
        }
        let index = timing_index(timing);
        let compensation = if index.is_multiple_of(2) { ON_SUBTRACT } else { OFF_SUBTRACT };
        if time_us < compensation || armed {
            return false;
        }
        self.counts[torpedo_num as usize - 1][index] = time_us - compensation;
        true
    }

    /// All timings of all torpedoes are set (`torpedo_timings_valid`).
    pub fn valid(&self) -> bool {
        self.counts.iter().flatten().all(|&c| c != 0)
    }

    /// The PIO words for torpedo `index` (0-based), or `None` if any is missing.
    pub fn sequence(&self, index: usize) -> Option<[u16; NUM_TIMINGS]> {
        let counts = *self.counts.get(index)?;
        counts.iter().all(|&c| c != 0).then_some(counts)
    }
}

/// Capacitor charge with hysteresis (`torpedo_check_charged`).
#[derive(Clone, Copy, Debug, Default)]
pub struct ChargeMonitor {
    charged: bool,
}

impl ChargeMonitor {
    pub const fn new() -> Self {
        ChargeMonitor { charged: false }
    }

    /// Updates with the latest voltage and returns whether the capacitor counts as charged.
    pub fn check(&mut self, volts: f32) -> bool {
        let threshold = if self.charged { TORPEDO_DISCHARGED_THRESHOLD } else { TORPEDO_CHARGED_THRESHOLD };
        self.charged = volts >= threshold;
        self.charged
    }
}

/// `torpedo_get_state`.
pub fn torpedo_state(timings_valid: bool, armed: bool, firing: bool, charged: bool) -> TorpedoState {
    if !timings_valid {
        TorpedoState::Error
    } else if !armed {
        TorpedoState::Disarmed
    } else if firing {
        TorpedoState::Firing
    } else if charged {
        TorpedoState::Charged
    } else {
        TorpedoState::Charging
    }
}

/// `dropper_get_state`. The C code never reported `DROPPED`.
pub fn dropper_state(active_time_ms: u16, armed: bool, dropping: bool) -> DropperState {
    if active_time_ms == 0 {
        DropperState::Error
    } else if !armed {
        DropperState::Disarmed
    } else if dropping {
        DropperState::Dropping
    } else {
        DropperState::Ready
    }
}

/// A claw move to start: drive the motor in this direction for this long.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ClawMove {
    pub forward: bool,
    pub duration_ms: u16,
}

/// Opening drives the motor forward (`OPEN_DIRECTION_IS_FORWARD`).
pub const OPEN_DIRECTION_IS_FORWARD: bool = true;

/// The timed claw (`actuators_v1/claw.c`). The C file was marked "must be fixed before production use": it doesn't
/// check the arm state, only the kill switch. This keeps that behaviour.
#[derive(Clone, Copy, Debug)]
pub struct Claw {
    state: ClawState,
    open_time_ms: u16,
    close_time_ms: u16,
}

impl Default for Claw {
    fn default() -> Self {
        Self::new()
    }
}

impl Claw {
    pub const fn new() -> Self {
        Claw { state: ClawState::Unknown, open_time_ms: 0, close_time_ms: 0 }
    }

    /// `claw_set_timings`.
    pub fn set_timings(&mut self, open_time_ms: u16, close_time_ms: u16) -> bool {
        if open_time_ms == 0 || close_time_ms == 0 {
            return false;
        }
        self.open_time_ms = open_time_ms;
        self.close_time_ms = close_time_ms;
        true
    }

    /// `claw_get_state`.
    pub fn state(&self) -> ClawState {
        if self.open_time_ms == 0 || self.close_time_ms == 0 { ClawState::Error } else { self.state }
    }

    /// A move is in progress (`claw_get_busy`).
    pub fn busy(&self) -> bool {
        matches!(self.state, ClawState::Opening | ClawState::Closing)
    }

    /// `claw_open`: returns the move to start, or `None` if it's already open or opening.
    pub fn open(&mut self, killed: bool) -> Result<Option<ClawMove>, ActuatorError> {
        self.begin(killed, true)
    }

    /// `claw_close`.
    pub fn close(&mut self, killed: bool) -> Result<Option<ClawMove>, ActuatorError> {
        self.begin(killed, false)
    }

    fn begin(&mut self, killed: bool, open: bool) -> Result<Option<ClawMove>, ActuatorError> {
        if killed {
            return Err(ActuatorError::KillPulled);
        }
        let (done, moving, opposite, duration_ms) = if open {
            (ClawState::Opened, ClawState::Opening, ClawState::Closed, self.open_time_ms)
        } else {
            (ClawState::Closed, ClawState::Closing, ClawState::Opened, self.close_time_ms)
        };
        match self.state() {
            s if s == done || s == moving => return Ok(None),
            s if s == opposite || s == ClawState::Unknown => {}
            // The C close path returned false with no message here; report the open path's message for both
            _ => return Err(ActuatorError::InvalidState),
        }
        self.state = moving;
        Ok(Some(ClawMove { forward: open == OPEN_DIRECTION_IS_FORWARD, duration_ms }))
    }

    /// The move timer expired (`claw_finish_callback`).
    pub fn finish(&mut self) {
        self.state = match self.state {
            ClawState::Opening => ClawState::Opened,
            ClawState::Closing => ClawState::Closed,
            other => other,
        };
    }

    /// Stops the claw (`claw_safety_disable`). A move in progress leaves the position unknown.
    pub fn safety_disable(&mut self) {
        if self.busy() {
            self.state = ClawState::Unknown;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn timings_need_every_value() {
        let mut t = TorpedoTimings::new();
        assert!(!t.valid());
        for timing in TorpedoTiming::ALL {
            assert!(t.set(1, timing, timing.default_us(), false));
        }
        assert!(!t.valid());
        assert_eq!(t.sequence(0), Some([23000 - 2, 250 - 4, 15000 - 2, 250 - 4, 13000 - 2]));
        assert_eq!(t.sequence(1), None);
        for timing in TorpedoTiming::ALL {
            assert!(t.set(2, timing, timing.default_us(), false));
        }
        assert!(t.valid());
        assert_eq!(t.sequence(2), None);
    }

    #[test]
    fn timing_rejections() {
        let mut t = TorpedoTimings::new();
        assert!(!t.set(0, TorpedoTiming::Coil1On, 100, false));
        assert!(!t.set(3, TorpedoTiming::Coil1On, 100, false));
        assert!(!t.set(1, TorpedoTiming::Coil1On, 1, false));
        // Exactly the overhead is accepted but leaves a zero count, which reads as missing
        assert!(t.set(1, TorpedoTiming::Coil1On, 2, false));
        assert_eq!(t.sequence(0), None);
        assert!(!t.set(1, TorpedoTiming::Coil1To2Delay, 3, false));
        assert!(!t.set(1, TorpedoTiming::Coil1On, 100, true));
    }

    #[test]
    fn charge_hysteresis() {
        let mut c = ChargeMonitor::new();
        assert!(!c.check(21.0));
        assert!(c.check(21.75));
        assert!(c.check(20.5));
        assert!(!c.check(20.0));
        assert!(!c.check(21.5));
        assert!((charge_volts(3500) - 21.806).abs() < 0.01);
    }

    #[test]
    fn states() {
        assert_eq!(torpedo_state(false, true, false, true), TorpedoState::Error);
        assert_eq!(torpedo_state(true, false, false, true), TorpedoState::Disarmed);
        assert_eq!(torpedo_state(true, true, true, true), TorpedoState::Firing);
        assert_eq!(torpedo_state(true, true, false, true), TorpedoState::Charged);
        assert_eq!(torpedo_state(true, true, false, false), TorpedoState::Charging);
        assert_eq!(dropper_state(0, true, false), DropperState::Error);
        assert_eq!(dropper_state(250, false, true), DropperState::Disarmed);
        assert_eq!(dropper_state(250, true, true), DropperState::Dropping);
        assert_eq!(dropper_state(250, true, false), DropperState::Ready);
    }

    #[test]
    fn claw_state_machine() {
        let mut c = Claw::new();
        assert_eq!(c.state(), ClawState::Error);
        assert_eq!(c.open(false), Err(ActuatorError::InvalidState));
        assert!(!c.set_timings(0, 10));
        assert!(c.set_timings(4500, 4500));
        assert_eq!(c.state(), ClawState::Unknown);
        assert_eq!(c.open(true), Err(ActuatorError::KillPulled));
        assert_eq!(c.open(false), Ok(Some(ClawMove { forward: true, duration_ms: 4500 })));
        assert!(c.busy());
        assert_eq!(c.open(false), Ok(None));
        c.finish();
        assert_eq!(c.state(), ClawState::Opened);
        assert_eq!(c.close(false), Ok(Some(ClawMove { forward: false, duration_ms: 4500 })));
        c.safety_disable();
        assert_eq!(c.state(), ClawState::Unknown);
        c.safety_disable();
        assert_eq!(c.state(), ClawState::Unknown);
    }
}
