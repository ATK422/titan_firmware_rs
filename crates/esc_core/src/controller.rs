//! The thruster RPM controller, ported from `Thruster_Controller/ThrusterController.c`.
//!
//! The output is a feed-forward term (a quadratic fit of DShot throttle against RPM) plus a P term on the error
//! against a 100-sample rolling average of the measured RPM, plus an I term that only integrates while the error is
//! within `i_bound`. Gains are fixed point: P is scaled by 1e6 and I by 1e9 (error in RPM times microseconds).
//!
//! Integer arithmetic follows the C code, which relied on two's complement wrapping in a few places. The Rust port
//! uses explicit wrapping operations there, with two deliberate exceptions:
//! * the feed-forward polynomial is evaluated in 64 bits: the C code overflowed `int32_t` for targets above about
//!   7000 RPM and could flip the sign of the output (no difference in the normal operating range)
//! * zero P or I gains skip the overflow clamps instead of dividing by zero

/// Length of the rolling average of the measured RPM.
pub const CONTROL_AVG_LENGTH: usize = 100;

/// Below this target RPM the feed-forward term is zero.
const FEED_FORWARD_DEAD_ZONE: i32 = 50;

/// The I term resets when the target moves by this much from where it was last reset.
const I_RESET_TARGET_CHANGE: i32 = 50;

/// After the controller output changes sign, the output is held at zero for this long.
pub const DIRECTION_CHANGE_HOLD_US: u64 = 2000;

/// Feed-forward DShot throttle for a target RPM.
pub fn feed_forward(target_rpm: i32) -> i32 {
    if target_rpm < FEED_FORWARD_DEAD_ZONE && target_rpm > -FEED_FORWARD_DEAD_ZONE {
        return 0;
    }
    // Coefficients are scaled by 1e6. The C file also has a disabled 100-segment cubic spline fit, not ported.
    // Clamped so the polynomial can't overflow; far beyond any reachable RPM, where the output is hard limited anyway
    let t = target_rpm.clamp(-1_000_000, 1_000_000) as i64;
    let value =
        if target_rpm > 0 { 12_369_874 + 103_410 * t + 42 * t * t } else { -14_791_378 + 98_938 * t - 42 * t * t };
    (value / 1_000_000) as i32
}

/// State of one thruster's controller (`thruster_controller_state_t`).
#[derive(Clone, Debug)]
pub struct ThrusterController {
    /// P gain, scaled by 1e6.
    pub p_gain: i32,
    /// I gain, scaled by 1e9 per (RPM · µs).
    pub i_gain: i32,
    /// The I term only integrates while the averaged error is within ± this many RPM.
    pub i_bound: i32,
    /// Output limit in DShot throttle units.
    pub hard_limit: i16,
    /// Outputs smaller than this (but not zero) are rounded to zero.
    pub min_command: i16,
    /// The thruster spins the other way for positive throttle.
    pub inverted: bool,

    sum_of_error: i64,
    last_i_reset: i32,
    avg_buffer: [i32; CONTROL_AVG_LENGTH],
    value_to_set: usize,
    rolling_avg: i32,
    avg_buffer_filled: bool,
    /// Output is held at zero until this time (µs since boot).
    direction_change_timeout_us: u64,
}

impl Default for ThrusterController {
    fn default() -> Self {
        Self::new()
    }
}

impl ThrusterController {
    pub const fn new() -> Self {
        ThrusterController {
            p_gain: 0,
            i_gain: 0,
            i_bound: 0,
            hard_limit: 0,
            min_command: 0,
            inverted: false,
            sum_of_error: 0,
            last_i_reset: 0,
            avg_buffer: [0; CONTROL_AVG_LENGTH],
            value_to_set: 0,
            rolling_avg: 0,
            avg_buffer_filled: false,
            direction_change_timeout_us: 0,
        }
    }

    /// Clears the integrator and the rolling average (`thruster_controller_zero`). Gains and the direction change
    /// hold are kept.
    pub fn zero(&mut self) {
        self.sum_of_error = 0;
        self.value_to_set = 0;
        self.rolling_avg = 0;
        self.avg_buffer_filled = false;
        self.last_i_reset = 0;
    }

    fn update_rolling_average(&mut self, current_rpm: i32) {
        // Scaled by 1e4 for resolution
        let rpm = current_rpm.wrapping_mul(10_000);
        let len = CONTROL_AVG_LENGTH as i32;
        if self.avg_buffer_filled {
            let delta = rpm.wrapping_sub(self.avg_buffer[self.value_to_set]) / len;
            self.rolling_avg = self.rolling_avg.wrapping_add(delta);
        } else {
            self.rolling_avg = self.rolling_avg.wrapping_add(rpm / len);
            if self.value_to_set == CONTROL_AVG_LENGTH - 1 {
                self.avg_buffer_filled = true;
            }
        }
        self.avg_buffer[self.value_to_set] = rpm;
        self.value_to_set = (self.value_to_set + 1) % CONTROL_AVG_LENGTH;
    }

    /// Runs the controller once (`thruster_controller_tick`). `delta_time_us` is the time since the previous tick,
    /// or 0 if there is none (disables integration for this tick). Returns a DShot throttle in ±`hard_limit`.
    pub fn tick(&mut self, target_rpm: i32, measured_rpm: i32, delta_time_us: i32) -> i16 {
        let current_rpm = if self.inverted { measured_rpm.wrapping_neg() } else { measured_rpm };

        self.update_rolling_average(current_rpm);
        let control_value = self.rolling_avg / 10_000;

        let instant_error = target_rpm.wrapping_sub(current_rpm);
        let mut current_error = target_rpm.wrapping_sub(control_value);
        // Keep the P product within i32
        if self.p_gain > 0 {
            let limit = i32::MAX / self.p_gain;
            if current_error > limit {
                current_error = limit;
            } else if current_error < -limit {
                current_error = -limit;
            }
        }

        let ff_control = feed_forward(target_rpm);
        let p_control = current_error.wrapping_mul(self.p_gain) / 1_000_000;

        let mut i_control = 0;
        if current_error < self.i_bound && current_error > self.i_bound.wrapping_neg() {
            // The C code multiplied in 32 bits before widening
            self.sum_of_error = self.sum_of_error.saturating_add(instant_error.wrapping_mul(delta_time_us) as i64);
            if self.i_gain > 0 {
                let max = i64::MAX / self.i_gain as i64;
                let min = i64::MIN / self.i_gain as i64;
                if self.sum_of_error > max {
                    self.sum_of_error = max;
                } else if self.sum_of_error < min {
                    self.sum_of_error = min;
                }
            }
            i_control = (self.sum_of_error.wrapping_mul(self.i_gain as i64) / 1_000_000_000) as i32;
        } else {
            // The averaged error left the bound
            self.sum_of_error = 0;
            self.last_i_reset = target_rpm;
        }

        let target_change = self.last_i_reset.wrapping_sub(target_rpm);
        if !(target_change < I_RESET_TARGET_CHANGE && target_change > -I_RESET_TARGET_CHANGE) {
            self.sum_of_error = 0;
            self.last_i_reset = target_rpm;
        }

        let mut control = i_control.wrapping_add(p_control).wrapping_add(ff_control);
        let hard_limit = self.hard_limit as i32;
        if control > hard_limit {
            control = hard_limit;
        } else if control < -hard_limit {
            control = -hard_limit;
        }
        let min_command = self.min_command as i32;
        if (control > 0 && control < min_command) || (control < 0 && control > -min_command) {
            control = 0;
        }
        if self.inverted {
            control = -control;
        }
        // The hard limit is an i16, so the cast only truncates if the limit itself was negative (as in C)
        control as i16
    }

    /// Runs [`tick`](Self::tick), then applies the direction change hold from `core1.c`: when the output would
    /// change sign relative to `previous_command`, the output is zero for [`DIRECTION_CHANGE_HOLD_US`] and the
    /// controller is zeroed.
    pub fn step(
        &mut self,
        target_rpm: i32,
        measured_rpm: i32,
        delta_time_us: i32,
        previous_command: i16,
        now_us: u64,
    ) -> i16 {
        let command = self.tick(target_rpm, measured_rpm, delta_time_us);
        let sign_changed = (command > 0 && previous_command < 0) || (command < 0 && previous_command > 0);
        if now_us < self.direction_change_timeout_us {
            self.zero();
            0
        } else if sign_changed {
            self.direction_change_timeout_us = now_us + DIRECTION_CHANGE_HOLD_US;
            self.zero();
            0
        } else {
            command
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tuned() -> ThrusterController {
        // The defaults core1_init configures
        ThrusterController {
            p_gain: 100_000,
            i_gain: 1000,
            i_bound: 300,
            hard_limit: 725,
            min_command: 0,
            ..ThrusterController::new()
        }
    }

    #[test]
    fn feed_forward_matches_fit() {
        assert_eq!(feed_forward(0), 0);
        assert_eq!(feed_forward(49), 0);
        assert_eq!(feed_forward(-49), 0);
        // (12369874 + 103410 * 1000 + 42 * 1000^2) / 1e6 = 157.78
        assert_eq!(feed_forward(1000), 157);
        // (-14791378 - 98938000 - 42000000) / 1e6 = -155.73
        assert_eq!(feed_forward(-1000), -155);
        assert_eq!(feed_forward(50), 17);
        // Above ~7150 RPM the C int32 arithmetic overflowed; the 64-bit version keeps growing
        assert!(feed_forward(10_000) > feed_forward(7000));
        assert!(feed_forward(-10_000) < feed_forward(-7000));
    }

    #[test]
    fn first_tick_from_standstill() {
        let mut c = tuned();
        // Error 1000 - 0 (average of one sample of 0): P = 1000 * 0.1 = 100, FF = 157, no I without a time delta
        assert_eq!(c.tick(1000, 0, 0), 257);
        let mut c = tuned();
        assert_eq!(c.tick(-1000, 0, 0), -255);
    }

    #[test]
    fn hard_limit_and_min_command() {
        let mut c = tuned();
        assert_eq!(c.tick(30_000, 0, 0), 725);
        let mut c = tuned();
        assert_eq!(c.tick(-30_000, 0, 0), -725);

        let mut c = ThrusterController { min_command: 300, ..tuned() };
        assert_eq!(c.tick(1000, 0, 0), 0);
        let mut c = ThrusterController { min_command: 200, ..tuned() };
        assert_eq!(c.tick(1000, 0, 0), 257);
    }

    #[test]
    fn inverted_thruster_mirrors_output() {
        let mut normal = tuned();
        let mut inverted = ThrusterController { inverted: true, ..tuned() };
        for i in 0..50 {
            let rpm = i * 10;
            assert_eq!(inverted.tick(800, -rpm, 1000), -normal.tick(800, rpm, 1000));
        }
    }

    #[test]
    fn rolling_average_converges() {
        let mut c = ThrusterController { p_gain: 1_000_000, i_gain: 0, i_bound: 0, hard_limit: 2000, ..tuned() };
        // With P = 1 and the target in the dead zone (no FF), the output is target - average
        for _ in 0..CONTROL_AVG_LENGTH {
            c.tick(0, 400, 0);
        }
        assert_eq!(c.rolling_avg / 10_000, 400);
        assert_eq!(c.tick(0, 400, 0), -400);
        // The window slides: after another full window of 200 the average is 200
        for _ in 0..CONTROL_AVG_LENGTH {
            c.tick(0, 200, 0);
        }
        assert_eq!(c.tick(0, 200, 0), -200);
        c.zero();
        assert_eq!(c.tick(0, 200, 0), -2);
    }

    /// A tuned controller whose rolling average has settled at `rpm` while targeting 1000 RPM.
    fn settled(rpm: i32) -> ThrusterController {
        let mut c = tuned();
        for _ in 0..CONTROL_AVG_LENGTH {
            c.tick(1000, rpm, 0);
        }
        c
    }

    #[test]
    fn integrator_accumulates_within_bound() {
        let mut c = settled(900);
        assert_eq!(c.sum_of_error, 0);
        // Error of 100 RPM (within the 300 RPM bound) for 1000 us per tick
        let mut last = 0;
        for _ in 0..10 {
            last = c.tick(1000, 900, 1000);
        }
        // sum = 10 * 100 * 1000 = 1e6 -> I = 1e6 * 1000 / 1e9 = 1; P = 100 * 0.1 = 10; FF = 157
        assert_eq!(c.sum_of_error, 1_000_000);
        assert_eq!(last, 168);

        // One sample of 0 RPM pulls the average down by only 9 RPM, so the averaged error stays within the bound and
        // the instantaneous error (1000 RPM) is integrated
        c.tick(1000, 0, 1000);
        assert_eq!(c.sum_of_error, 2_000_000);
        // A fresh controller sees the full 1000 RPM error, outside the bound, and doesn't integrate
        let mut far = tuned();
        far.tick(1000, 0, 1000);
        assert_eq!(far.sum_of_error, 0);

        // A target change of 50 RPM or more also resets it
        let mut c = settled(900);
        c.tick(1000, 900, 1000);
        assert_eq!(c.sum_of_error, 100_000);
        c.tick(1060, 960, 1000);
        assert_eq!(c.sum_of_error, 0);
    }

    #[test]
    fn zero_gains_do_not_divide_by_zero() {
        let mut c = ThrusterController { p_gain: 0, i_gain: 0, i_bound: 1_000_000, hard_limit: 725, ..tuned() };
        assert_eq!(c.tick(1000, 0, 1000), 157);
        // Extreme inputs wrap instead of panicking
        let mut c = ThrusterController { i_bound: i32::MAX, ..tuned() };
        c.tick(i32::MAX, i32::MIN, i32::MAX);
        c.tick(i32::MIN, 60_000_000 / 7, i32::MAX);
    }

    #[test]
    fn direction_change_is_held_at_zero() {
        let mut c = tuned();
        // Commanding reverse while the last command was forward: zeroed and held for 2 ms
        assert_eq!(c.step(-1000, 0, 0, 200, 10_000), 0);
        assert_eq!(c.step(-1000, 0, 0, 0, 11_000), 0);
        assert_eq!(c.step(-1000, 0, 0, 0, 11_999), 0);
        assert_eq!(c.step(-1000, 0, 0, 0, 12_000), -255);
        // Same direction passes straight through
        assert_eq!(c.step(-1000, 0, 0, -255, 12_500), -255);
    }
}
