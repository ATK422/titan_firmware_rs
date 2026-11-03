//! The thruster control loop of `core1.c`, with the hardware access taken out.
//!
//! The firmware runs one tick every [`DSHOT_MIN_FRAME_TIME_US`](crate::DSHOT_MIN_FRAME_TIME_US):
//! 1. apply queued tuning parameters ([`ControlLoop::apply_param`])
//! 2. [`ControlLoop::begin_tick`] checks the kill conditions and returns the commands to send now. In the normal
//!    path these are the controller outputs computed from the previous tick's RPM feedback.
//! 3. send them over DShot, then feed every eRPM reply that arrives within the frame time to
//!    [`ControlLoop::on_rpm`], which runs the controller and stores the command for the next tick
//! 4. [`ControlLoop::end_tick`] records telemetry and takes thrusters with too many missed replies offline

use crate::controller::ThrusterController;
use crate::dshot::{THROTTLE_MAX, throttle_command};
use crate::{DSHOT_UPDATE_DISABLE_TIME_MS, ESC_POWERUP_DELAY_MS, NUM_THRUSTERS, TELEM_MAX_MISSED_DSHOT_PACKETS};

/// Bit numbers of [`TickOutput::disabled_flags`] (`CORE1_DISABLED_*`). Any set bit forces all commands to zero.
pub mod disabled {
    /// The vehicle is killed.
    pub const SAFETY_KILLED: u8 = 0;
    /// The ESC rail is unpowered (or dropped out since the last tick).
    pub const ESC_BOARD_OFF: u8 = 1;
    /// The ESCs powered up less than `ESC_POWERUP_DELAY_MS` ago and may still be chiming.
    pub const ESC_POWERON_DELAY: u8 = 2;
    /// No target RPM arrived within `DSHOT_MIN_UPDATE_RATE_MS`.
    pub const TGT_RPM_STALE: u8 = 3;
    /// The target went stale while the thrusters were moving; commands are ignored for a while.
    pub const TGT_RPM_STALE_PENALTY: u8 = 4;
    /// Not every tuning parameter has been set yet.
    pub const CONTROLLER_UNCONFIGURED: u8 = 5;
}

/// Largest tuning value accepted (the C firmware packed them into 24 bits of a multicore FIFO word).
pub const PARAM_VALUE_LIMIT: i32 = 1 << 24;

/// A controller tuning parameter, applied to all thrusters at the start of a tick.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Param {
    PGain(i32),
    IGain(i32),
    IBound(i32),
    HardLimit(i32),
    MinCommand(i32),
    /// Bit `i` inverts thruster `i`.
    InvertMask(u8),
    /// Send the target values straight to the ESCs as throttle (-999..=999), bypassing the controller.
    RawMode(bool),
}

const PARAM_COUNT: u32 = 7;

impl Param {
    fn index(self) -> u32 {
        match self {
            Param::PGain(_) => 0,
            Param::IGain(_) => 1,
            Param::IBound(_) => 2,
            Param::HardLimit(_) => 3,
            Param::MinCommand(_) => 4,
            Param::InvertMask(_) => 5,
            Param::RawMode(_) => 6,
        }
    }

    /// Returns the parameter if its value is in range, like the `core1_set_*` checks: numeric values must be in
    /// `0..PARAM_VALUE_LIMIT` and the invert mask must only cover existing thrusters.
    pub fn validate(self) -> Option<Param> {
        let ok = match self {
            Param::PGain(v) | Param::IGain(v) | Param::IBound(v) | Param::HardLimit(v) | Param::MinCommand(v) => {
                (0..PARAM_VALUE_LIMIT).contains(&v)
            }
            Param::InvertMask(mask) => (mask as usize) < (1 << NUM_THRUSTERS),
            Param::RawMode(_) => true,
        };
        ok.then_some(self)
    }

    /// The parameters `core1_init` sets at boot. The inverted mask is the same for both boards (thrusters 1 and 4,
    /// or 5 and 8).
    pub const DEFAULTS: [Param; PARAM_COUNT as usize] = [
        Param::InvertMask(0b1001),
        Param::PGain(100_000),
        Param::IGain(1000),
        Param::IBound(300),
        Param::HardLimit(725),
        Param::MinCommand(0),
        Param::RawMode(false),
    ];
}

/// What the loop needs to know about the outside world at the start of a tick.
#[derive(Clone, Copy, Debug, Default)]
pub struct Inputs {
    /// The vehicle is killed.
    pub killed: bool,
    /// The ESC rail is off now, or was seen off since the last tick.
    pub esc_board_off: bool,
    /// Latest target RPM (or raw throttle in raw mode).
    pub target_rpm: [i16; NUM_THRUSTERS],
    /// The target is older than `DSHOT_MIN_UPDATE_RATE_MS`.
    pub target_expired: bool,
}

/// Result of [`ControlLoop::begin_tick`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TickOutput {
    /// Throttle to send to each thruster now (-999..=999).
    pub commands: [i16; NUM_THRUSTERS],
    /// See [`disabled`].
    pub disabled_flags: u8,
    /// The target timed out while the thrusters were moving: raise `FAULT_RPM_CMD_TIMEOUT`.
    pub command_timed_out: bool,
}

/// Per-thruster telemetry kept by the control loop (`struct thruster_state`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ThrusterTelemetry {
    /// The command sent this tick.
    pub cmd: i16,
    /// Last RPM feedback (only meaningful while `rpm_valid`).
    pub rpm: i16,
    pub rpm_valid: bool,
    pub thruster_ready: bool,
    /// eRPM replies missed in a row.
    pub rpm_missed_count: u8,
    /// Ticks with a missed reply while the thruster was still online, since the last clearing snapshot.
    pub ticks_missed: u16,
    /// Ticks the thruster spent offline, since the last clearing snapshot.
    pub ticks_offline: u16,
}

/// Telemetry shared from the control loop with the rest of the firmware (`core1_telem_shared_mem`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ControllerTelemetry {
    /// Ticks since the last clearing snapshot (saturating).
    pub tick_cnt: u16,
    /// See [`disabled`].
    pub disabled_flags: u8,
    /// Any thruster is being sent a non-neutral command (`dshot_thrusters_on`).
    pub thrusters_on: bool,
    pub thrusters: [ThrusterTelemetry; NUM_THRUSTERS],
}

impl ControllerTelemetry {
    pub const fn new() -> Self {
        const ZERO: ThrusterTelemetry = ThrusterTelemetry {
            cmd: 0,
            rpm: 0,
            rpm_valid: false,
            thruster_ready: false,
            rpm_missed_count: 0,
            ticks_missed: 0,
            ticks_offline: 0,
        };
        ControllerTelemetry { tick_cnt: 0, disabled_flags: 0, thrusters_on: false, thrusters: [ZERO; NUM_THRUSTERS] }
    }

    /// Copies the telemetry out, optionally clearing the tick and miss counters (`core1_get_telem`).
    pub fn snapshot(&mut self, clear: bool) -> ControllerTelemetry {
        let out = *self;
        if clear {
            self.tick_cnt = 0;
            for t in &mut self.thrusters {
                t.ticks_missed = 0;
                t.ticks_offline = 0;
            }
        }
        out
    }
}

/// The control loop state (the locals of `core1_main`).
#[derive(Clone, Debug)]
pub struct ControlLoop {
    controllers: [ThrusterController; NUM_THRUSTERS],
    raw_mode: bool,
    /// The throttle for the next send; overwritten by the controller as RPM feedback arrives.
    throttle: [i16; NUM_THRUSTERS],
    /// The target RPM cached for this tick (zeroed in raw mode).
    target: [i16; NUM_THRUSTERS],
    last_tick_us: Option<u64>,
    time_difference_us: i32,
    esc_startup_finished_us: u64,
    command_timeout_penalty_us: u64,
    missing_params: u32,
    thrusters_on: bool,
    disabled_flags: u8,
    sent: [i16; NUM_THRUSTERS],
}

impl ControlLoop {
    /// A loop started at `now_us`, unconfigured and with the ESC power-up delay running.
    pub fn new(now_us: u64) -> Self {
        ControlLoop {
            controllers: core::array::from_fn(|_| ThrusterController::new()),
            raw_mode: false,
            throttle: [0; NUM_THRUSTERS],
            target: [0; NUM_THRUSTERS],
            last_tick_us: None,
            time_difference_us: 0,
            esc_startup_finished_us: now_us + ESC_POWERUP_DELAY_MS * 1000,
            command_timeout_penalty_us: 0,
            missing_params: (1 << PARAM_COUNT) - 1,
            thrusters_on: false,
            disabled_flags: 0,
            sent: [0; NUM_THRUSTERS],
        }
    }

    /// Applies a (validated) tuning parameter to all thrusters and zeroes the controllers. Returns `Some(raw)` for
    /// [`Param::RawMode`], so the caller can raise or lower `FAULT_RAW_MODE`.
    pub fn apply_param(&mut self, param: Param) -> Option<bool> {
        let mut raw = None;
        for (i, c) in self.controllers.iter_mut().enumerate() {
            match param {
                Param::PGain(v) => c.p_gain = v,
                Param::IGain(v) => c.i_gain = v,
                Param::IBound(v) => c.i_bound = v,
                // Truncated to 16 bits like the C struct fields
                Param::HardLimit(v) => c.hard_limit = v as i16,
                Param::MinCommand(v) => c.min_command = v as i16,
                Param::InvertMask(mask) => c.inverted = mask & (1 << i) != 0,
                Param::RawMode(on) => raw = Some(on),
            }
            c.zero();
        }
        if let Some(on) = raw {
            self.raw_mode = on;
        }
        self.missing_params &= !(1 << param.index());
        raw
    }

    pub fn raw_mode(&self) -> bool {
        self.raw_mode
    }

    /// Checks the kill conditions and decides the commands to send this tick (phases 2-4 of `core1_main`).
    pub fn begin_tick(&mut self, now_us: u64, inputs: Inputs) -> TickOutput {
        self.target = inputs.target_rpm;
        self.time_difference_us = 0;

        let mut flags = 0u8;
        let mut command_timed_out = false;
        if inputs.killed {
            flags |= 1 << disabled::SAFETY_KILLED;
        }
        if inputs.esc_board_off {
            flags |= 1 << disabled::ESC_BOARD_OFF;
            // Restart the power-up delay so commands wait for the ESCs to chime once power returns
            self.esc_startup_finished_us = now_us + ESC_POWERUP_DELAY_MS * 1000;
        } else if now_us < self.esc_startup_finished_us {
            flags |= 1 << disabled::ESC_POWERON_DELAY;
        }
        if inputs.target_expired {
            flags |= 1 << disabled::TGT_RPM_STALE;
            // Timing out while moving means the commander isn't keeping up: fault, and lock out to prevent jitter
            if self.thrusters_on {
                command_timed_out = true;
                self.command_timeout_penalty_us = now_us + DSHOT_UPDATE_DISABLE_TIME_MS * 1000;
            }
        }
        if now_us < self.command_timeout_penalty_us {
            flags |= 1 << disabled::TGT_RPM_STALE_PENALTY;
        }
        if self.missing_params != 0 {
            flags |= 1 << disabled::CONTROLLER_UNCONFIGURED;
        }

        if flags != 0 {
            // Start from a clean slate once re-enabled
            self.throttle = [0; NUM_THRUSTERS];
            self.controllers.iter_mut().for_each(ThrusterController::zero);
            self.last_tick_us = None;
        } else if self.raw_mode {
            for i in 0..NUM_THRUSTERS {
                let throttle = self.target[i];
                self.throttle[i] = if (-THROTTLE_MAX..=THROTTLE_MAX).contains(&throttle) { throttle } else { 0 };
                self.target[i] = 0;
                self.controllers[i].zero();
            }
            self.last_tick_us = None;
        } else {
            for i in 0..NUM_THRUSTERS {
                if self.target[i] == 0 {
                    self.throttle[i] = 0;
                    self.controllers[i].zero();
                }
            }
            // The integrator only runs with a real time difference to the previous tick
            if let Some(last) = self.last_tick_us {
                self.time_difference_us = now_us.saturating_sub(last) as i32;
            }
            self.last_tick_us = Some(now_us);
        }

        self.sent = self.throttle;
        self.thrusters_on = self.sent.iter().any(|&t| throttle_command(t).reversed.is_some());
        self.disabled_flags = flags;
        TickOutput { commands: self.sent, disabled_flags: flags, command_timed_out }
    }

    /// Feeds the RPM feedback of `thruster` received during this tick to its controller, which computes the
    /// command for the next tick.
    pub fn on_rpm(&mut self, thruster: usize, rpm: i32, now_us: u64) {
        self.throttle[thruster] = self.controllers[thruster].step(
            self.target[thruster] as i32,
            rpm,
            self.time_difference_us,
            self.throttle[thruster],
            now_us,
        );
    }

    /// Records this tick's telemetry. `rpm[i]` is the decoded feedback of thruster `i`, if any arrived. Thrusters
    /// that missed more than [`TELEM_MAX_MISSED_DSHOT_PACKETS`] replies in a row are taken offline: their command
    /// is cleared until they answer again.
    pub fn end_tick(&mut self, rpm: [Option<i32>; NUM_THRUSTERS], telemetry: &mut ControllerTelemetry) {
        telemetry.disabled_flags = self.disabled_flags;
        telemetry.thrusters_on = self.thrusters_on;
        telemetry.tick_cnt = telemetry.tick_cnt.saturating_add(1);
        for i in 0..NUM_THRUSTERS {
            let t = &mut telemetry.thrusters[i];
            t.cmd = self.sent[i];
            if let Some(rpm) = rpm[i] {
                t.rpm = rpm.clamp(i16::MIN as i32, i16::MAX as i32) as i16;
                t.rpm_missed_count = 0;
                t.rpm_valid = true;
                t.thruster_ready = true;
            } else if t.rpm_missed_count < TELEM_MAX_MISSED_DSHOT_PACKETS {
                t.rpm_missed_count += 1;
                t.ticks_missed = t.ticks_missed.saturating_add(1);
            } else {
                t.rpm = 0;
                t.rpm_valid = false;
                t.thruster_ready = false;
                t.ticks_offline = t.ticks_offline.saturating_add(1);
                // Running an old command through a few missed replies is fine, but not indefinitely
                self.throttle[i] = 0;
                self.controllers[i].zero();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: u64 = 1000;

    fn configured(now_us: u64) -> ControlLoop {
        let mut l = ControlLoop::new(now_us);
        for p in Param::DEFAULTS {
            l.apply_param(p);
        }
        l
    }

    fn enabled_inputs(target: [i16; 4]) -> Inputs {
        Inputs { killed: false, esc_board_off: false, target_rpm: target, target_expired: false }
    }

    /// Runs one tick in which every thruster reports `rpm`.
    fn tick(
        l: &mut ControlLoop,
        now: u64,
        inputs: Inputs,
        rpm: Option<i32>,
        t: &mut ControllerTelemetry,
    ) -> TickOutput {
        let out = l.begin_tick(now, inputs);
        if let Some(rpm) = rpm {
            for i in 0..NUM_THRUSTERS {
                l.on_rpm(i, rpm, now + 100);
            }
        }
        l.end_tick([rpm; NUM_THRUSTERS], t);
        out
    }

    #[test]
    fn param_validation() {
        assert_eq!(Param::PGain(5).validate(), Some(Param::PGain(5)));
        assert_eq!(Param::PGain(-1).validate(), None);
        assert_eq!(Param::IBound(PARAM_VALUE_LIMIT).validate(), None);
        assert_eq!(Param::InvertMask(0xF).validate(), Some(Param::InvertMask(0xF)));
        assert_eq!(Param::InvertMask(0x10).validate(), None);
        assert_eq!(Param::RawMode(true).validate(), Some(Param::RawMode(true)));
    }

    #[test]
    fn disabled_until_configured_powered_and_commanded() {
        let mut l = ControlLoop::new(0);
        let mut t = ControllerTelemetry::new();
        let out = tick(&mut l, 0, Inputs { target_expired: true, ..enabled_inputs([500; 4]) }, None, &mut t);
        assert_eq!(
            out.disabled_flags,
            1 << disabled::ESC_POWERON_DELAY | 1 << disabled::TGT_RPM_STALE | 1 << disabled::CONTROLLER_UNCONFIGURED
        );
        assert_eq!(out.commands, [0; 4]);
        assert!(!out.command_timed_out);

        for p in &Param::DEFAULTS[..6] {
            l.apply_param(*p);
        }
        let out = tick(&mut l, 6000 * MS, enabled_inputs([500; 4]), None, &mut t);
        assert_eq!(out.disabled_flags, 1 << disabled::CONTROLLER_UNCONFIGURED);
        assert_eq!(l.apply_param(Param::RawMode(false)), Some(false));
        let out = tick(&mut l, 6001 * MS, enabled_inputs([500; 4]), None, &mut t);
        assert_eq!(out.disabled_flags, 0);
        assert_eq!(t.disabled_flags, 0);
    }

    #[test]
    fn kill_and_power_loss_zero_the_commands() {
        let mut l = configured(0);
        let mut t = ControllerTelemetry::new();
        let now = 6000 * MS;
        tick(&mut l, now, enabled_inputs([1000; 4]), Some(0), &mut t);
        // The controller computed a command from the feedback; it goes out on the next tick. Thrusters 0 and 3 are
        // inverted by the default mask.
        let out = tick(&mut l, now + 400, enabled_inputs([1000; 4]), Some(0), &mut t);
        assert!(out.commands[0] < 0 && out.commands[1] > 0 && out.commands[2] > 0 && out.commands[3] < 0);
        assert!(t.thrusters_on);

        let out = tick(&mut l, now + 800, Inputs { killed: true, ..enabled_inputs([1000; 4]) }, Some(0), &mut t);
        assert_eq!(out.disabled_flags, 1 << disabled::SAFETY_KILLED);
        assert_eq!(out.commands, [0; 4]);
        assert!(!t.thrusters_on);

        // Power loss restarts the power-up delay
        let out =
            tick(&mut l, now + 1200, Inputs { esc_board_off: true, ..enabled_inputs([1000; 4]) }, Some(0), &mut t);
        assert_eq!(out.disabled_flags, 1 << disabled::ESC_BOARD_OFF);
        let out = tick(&mut l, now + 1600, enabled_inputs([1000; 4]), Some(0), &mut t);
        assert_eq!(out.disabled_flags, 1 << disabled::ESC_POWERON_DELAY);
        assert_eq!(out.commands, [0; 4]);
        let out = tick(&mut l, now + 1200 + ESC_POWERUP_DELAY_MS * MS, enabled_inputs([1000; 4]), Some(0), &mut t);
        assert_eq!(out.disabled_flags, 0);
    }

    #[test]
    fn stale_command_while_moving_faults_and_locks_out() {
        let mut l = configured(0);
        let mut t = ControllerTelemetry::new();
        let now = 6000 * MS;
        tick(&mut l, now, enabled_inputs([1000; 4]), Some(0), &mut t);
        tick(&mut l, now + 400, enabled_inputs([1000; 4]), Some(0), &mut t);
        let stale = Inputs { target_expired: true, ..enabled_inputs([1000; 4]) };
        let out = tick(&mut l, now + 800, stale, Some(0), &mut t);
        assert!(out.command_timed_out);
        assert_eq!(out.disabled_flags, 1 << disabled::TGT_RPM_STALE | 1 << disabled::TGT_RPM_STALE_PENALTY);
        // Not moving any more, so further stale ticks don't fault again
        let out = tick(&mut l, now + 1200, stale, Some(0), &mut t);
        assert!(!out.command_timed_out);
        // Fresh commands are still locked out for a second
        let out = tick(&mut l, now + 1600, enabled_inputs([1000; 4]), Some(0), &mut t);
        assert_eq!(out.disabled_flags, 1 << disabled::TGT_RPM_STALE_PENALTY);
        let out =
            tick(&mut l, now + 800 + DSHOT_UPDATE_DISABLE_TIME_MS * MS, enabled_inputs([1000; 4]), Some(0), &mut t);
        assert_eq!(out.disabled_flags, 0);
    }

    #[test]
    fn raw_mode_passes_throttle_through() {
        let mut l = configured(0);
        assert_eq!(l.apply_param(Param::RawMode(true)), Some(true));
        let mut t = ControllerTelemetry::new();
        let out = tick(&mut l, 6000 * MS, enabled_inputs([100, -999, 1000, 0]), Some(0), &mut t);
        assert_eq!(out.commands, [100, -999, 0, 0]);
        assert_eq!(t.thrusters[1].cmd, -999);
    }

    #[test]
    fn zero_target_forces_zero_command() {
        let mut l = configured(0);
        let mut t = ControllerTelemetry::new();
        tick(&mut l, 6000 * MS, enabled_inputs([1000, 1000, 1000, 1000]), Some(0), &mut t);
        let out = tick(&mut l, 6000 * MS + 400, enabled_inputs([1000, 0, 1000, 0]), Some(0), &mut t);
        assert_eq!(out.commands[1], 0);
        assert_eq!(out.commands[3], 0);
        assert!(out.commands[0] != 0 && out.commands[2] != 0);
    }

    #[test]
    fn missing_feedback_takes_thrusters_offline() {
        let mut l = configured(0);
        let mut t = ControllerTelemetry::new();
        let mut now = 6000 * MS;
        tick(&mut l, now, enabled_inputs([1000; 4]), Some(1234), &mut t);
        assert!(t.thrusters[0].rpm_valid && t.thrusters[0].thruster_ready);
        assert_eq!(t.thrusters[0].rpm, 1234);

        // Up to 20 misses keep the last command going
        for _ in 0..TELEM_MAX_MISSED_DSHOT_PACKETS {
            now += 400;
            let out = tick(&mut l, now, enabled_inputs([1000; 4]), None, &mut t);
            assert!(out.commands.iter().all(|&c| c != 0));
        }
        assert!(t.thrusters[0].rpm_valid);
        assert_eq!(t.thrusters[0].ticks_missed, TELEM_MAX_MISSED_DSHOT_PACKETS as u16);

        // The next one takes them offline and clears the command
        now += 400;
        tick(&mut l, now, enabled_inputs([1000; 4]), None, &mut t);
        assert!(!t.thrusters[0].rpm_valid && !t.thrusters[0].thruster_ready);
        assert_eq!(t.thrusters[0].ticks_offline, 1);
        now += 400;
        let out = tick(&mut l, now, enabled_inputs([1000; 4]), None, &mut t);
        assert_eq!(out.commands, [0; 4]);

        let snap = t.snapshot(true);
        assert_eq!(snap.tick_cnt, TELEM_MAX_MISSED_DSHOT_PACKETS as u16 + 3);
        assert_eq!(snap.thrusters[0].ticks_offline, 2);
        assert_eq!(t.tick_cnt, 0);
        assert_eq!(t.thrusters[0].ticks_missed, 0);
        assert_eq!(t.thrusters[0].rpm_missed_count, TELEM_MAX_MISSED_DSHOT_PACKETS);
    }

    #[test]
    fn telemetry_rpm_saturates() {
        let mut l = configured(0);
        let mut t = ControllerTelemetry::new();
        tick(&mut l, 6000 * MS, enabled_inputs([0; 4]), Some(60_000_000 / 7), &mut t);
        assert_eq!(t.thrusters[2].rpm, i16::MAX);
    }
}
