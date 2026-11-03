//! H-bridge control math, ported from `hbridge.c`.
//!
//! Each bridge is driven by a phase (direction) GPIO and a PWM enable output. Commands set a target in
//! `[-1, 1]`; every [`SLEW_PERIOD_MS`] the current output moves towards the target by at most [`SLEW_MAX_DIFF`],
//! and the PWM level is the wrap value scaled by the magnitude. A bridge marked inverted has its sign flipped
//! before it reaches the pins.

/// PWM frequency of the enable outputs (`PWM_FREQ_HZ`).
pub const PWM_FREQ_HZ: u32 = 25_000;
/// Time between slew steps (`SLEW_PERIOD_MS`).
pub const SLEW_PERIOD_MS: u64 = 20;
/// Largest change of the output per slew step (`SLEW_MAX_DIFF`).
pub const SLEW_MAX_DIFF: f32 = 0.1;
/// Output limits (`PCT_MIN`, `PCT_MAX`).
pub const PCT_MIN: f32 = -1.0;
pub const PCT_MAX: f32 = 1.0;

/// nSLEEP low time that puts the bridges to sleep before a wake attempt (`TIME_SLEEP_US`).
pub const TIME_SLEEP_US: u64 = 120;
/// Time after nSLEEP rises before nFAULT is valid, and again before the reset pulse (`WAKEUP_TIME_FOR_COM_US`).
pub const WAKEUP_TIME_FOR_COM_US: u64 = 500;
/// Length of the nSLEEP reset pulse that clears the wake-up fault indication (`WAKEUP_TIME_RESET_US`).
pub const WAKEUP_TIME_RESET_US: u64 = 50;
/// Wake attempts before giving up on missing bridges (`MAX_WAKE_ATTEMPTS`).
pub const MAX_WAKE_ATTEMPTS: u32 = 3;

/// PWM slice settings for the enable outputs.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PwmParams {
    /// Integer part of the clock divider.
    pub div_int: u8,
    /// Fractional part of the clock divider, in sixteenths.
    pub div_frac: u8,
    /// Counter wrap (TOP) value. A level of `wrap` is the largest output.
    pub wrap: u16,
}

impl PwmParams {
    /// The divider as 8.4 fixed point bits, the format of the RP2040 `DIV` register.
    pub fn divider_bits(&self) -> u16 {
        (self.div_int as u16) << 4 | self.div_frac as u16
    }
}

/// Divider and wrap for [`PWM_FREQ_HZ`] at the given system clock, computed exactly as `hbridge_pwm_init`.
///
/// The C formula picks the smallest divider (in sixteenths) that keeps the wrap within 12 bits, rounded up, but
/// never below 1. At 125 MHz that is a divider of 1 and a wrap of 4999.
pub fn pwm_params(sys_clk_hz: u32) -> PwmParams {
    let clock = sys_clk_hz as u64;
    let freq = PWM_FREQ_HZ as u64;
    let mut divider16 = clock / freq / 4096 + !clock.is_multiple_of(freq * 4096) as u64;
    if divider16 / 16 == 0 {
        divider16 = 16;
    }
    let wrap = clock * 16 / divider16 / freq - 1;
    PwmParams { div_int: (divider16 / 16) as u8, div_frac: (divider16 & 0xF) as u8, wrap: wrap as u16 }
}

/// What one bridge's pins should be set to.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Output {
    /// Phase (direction) pin level: high for negative outputs.
    pub phase: bool,
    /// PWM compare level of the enable pin.
    pub level: u16,
}

/// Pin state for an output of `pct` (`hbridge_set_curr_duty`).
pub fn output(pct: f32, inverted: bool, wrap: u16) -> Output {
    let pct = if inverted { -pct } else { pct };
    // Float to integer casts truncate towards zero like the C conversion, and saturate instead of being undefined
    Output { phase: pct < 0.0, level: (wrap as f32 * pct.abs()) as u16 }
}

/// Moves `current` one slew step towards `target` (`hbridge_slew`).
pub fn slew(current: f32, target: f32) -> f32 {
    let error = target - current;
    if error.abs() < SLEW_MAX_DIFF {
        target
    } else if error > 0.0 {
        current + SLEW_MAX_DIFF
    } else {
        current - SLEW_MAX_DIFF
    }
}

/// Why a target was refused.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TargetError {
    /// The bridges are disabled (vehicle killed); the C firmware dropped these silently too.
    Disabled,
    /// No bridge with that index exists.
    NoSuchBridge,
    /// The target is NaN.
    NotANumber,
}

#[derive(Clone, Copy, Debug)]
struct Bridge {
    target: f32,
    current: f32,
    inverted: bool,
}

/// The state of all bridges, shared between the command handler, the kill hook and the slew task.
#[derive(Debug)]
pub struct Bridges<const N: usize> {
    bridges: [Bridge; N],
    enabled: bool,
}

impl<const N: usize> Bridges<N> {
    /// Bridges at rest and disabled. `inverted[i]` flips the sign of bridge `i`.
    pub const fn new(inverted: [bool; N]) -> Self {
        let mut bridges = [Bridge { target: 0.0, current: 0.0, inverted: false }; N];
        let mut i = 0;
        while i < N {
            bridges[i].inverted = inverted[i];
            i += 1;
        }
        Bridges { bridges, enabled: false }
    }

    /// Sets the target of bridge `idx`, clamped to `[-1, 1]` (`hbridge_set_target`).
    ///
    /// Ignored while disabled, as in C. The C code wrote past the last bridge for out of range indices (harmless
    /// there because the array had a spare slot) and let NaN through to the PWM; both are refused here.
    pub fn set_target(&mut self, idx: usize, pct: f32) -> Result<(), TargetError> {
        if !self.enabled {
            return Err(TargetError::Disabled);
        }
        if pct.is_nan() {
            return Err(TargetError::NotANumber);
        }
        let bridge = self.bridges.get_mut(idx).ok_or(TargetError::NoSuchBridge)?;
        bridge.target = pct.clamp(PCT_MIN, PCT_MAX);
        Ok(())
    }

    /// Zeroes every target and enables or disables commands (`hbridge_set_enabled`). The caller drives DRVOFF.
    pub fn set_enabled(&mut self, enabled: bool) {
        for bridge in &mut self.bridges {
            bridge.target = 0.0;
        }
        self.enabled = enabled;
    }

    /// Disables the bridges and drops their outputs to zero at once instead of slewing down.
    ///
    /// The C kill handler only zeroed the targets; DRVOFF already cuts the outputs, so dropping the duty as well
    /// changes nothing visible while killed, but means nothing is driven if DRVOFF fails.
    pub fn kill(&mut self) {
        self.set_enabled(false);
        for bridge in &mut self.bridges {
            bridge.current = 0.0;
        }
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// The current output of bridge `idx`, before inversion.
    pub fn current(&self, idx: usize) -> f32 {
        self.bridges[idx].current
    }

    /// Runs one slew step for every bridge and returns their pin states.
    pub fn slew(&mut self, wrap: u16) -> [Output; N] {
        core::array::from_fn(|i| {
            let bridge = &mut self.bridges[i];
            bridge.current = slew(bridge.current, bridge.target);
            output(bridge.current, bridge.inverted, wrap)
        })
    }
}

/// Counts the bridges that woke correctly in one wake attempt (`hbridge_wake`).
///
/// A DRV8243 that wakes properly pulls nFAULT low to report the wake-up, then releases it after the nSLEEP reset
/// pulse. `nfault_before_reset[i]` and `nfault_after_reset[i]` are bridge `i`'s nFAULT levels (high = true) read
/// before and after the pulse.
pub fn count_awake(nfault_before_reset: &[bool], nfault_after_reset: &[bool]) -> usize {
    nfault_before_reset.iter().zip(nfault_after_reset).filter(|&(&before, &after)| !before && after).count()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pwm_params_match_c_at_125_mhz() {
        let p = pwm_params(125_000_000);
        assert_eq!(p, PwmParams { div_int: 1, div_frac: 0, wrap: 4999 });
        assert_eq!(p.divider_bits(), 16);
        // 125 MHz / 1 / (4999 + 1) = 25 kHz
        assert_eq!(125_000_000 / (p.wrap as u32 + 1), PWM_FREQ_HZ);
    }

    #[test]
    fn pwm_params_follow_the_c_formula_at_other_clocks() {
        // 48 MHz: 48e6 / 25e3 / 4096 = 0, remainder -> 1, below 16 -> 16
        assert_eq!(pwm_params(48_000_000), PwmParams { div_int: 1, div_frac: 0, wrap: 1919 });
        // 200 MHz: 200e6 / 25e3 = 8000 / 4096 = 1, + 1 = 2 -> 16
        assert_eq!(pwm_params(200_000_000), PwmParams { div_int: 1, div_frac: 0, wrap: 7999 });
    }

    #[test]
    fn output_sets_phase_and_level() {
        assert_eq!(output(0.0, false, 4999), Output { phase: false, level: 0 });
        assert_eq!(output(1.0, false, 4999), Output { phase: false, level: 4999 });
        assert_eq!(output(-0.5, false, 4999), Output { phase: true, level: 2499 });
        assert_eq!(output(0.5, true, 4999), Output { phase: true, level: 2499 });
        assert_eq!(output(-0.25, true, 4000), Output { phase: false, level: 1000 });
    }

    #[test]
    fn slew_is_rate_limited() {
        assert_eq!(slew(0.0, 0.05), 0.05);
        assert!((slew(0.0, 1.0) - 0.1).abs() < 1e-6);
        assert!((slew(0.5, -1.0) - 0.4).abs() < 1e-6);
        let mut x = 0.0;
        let mut steps = 0;
        while x != 1.0 {
            x = slew(x, 1.0);
            steps += 1;
            assert!(steps < 20);
        }
        // 0.1 per step from 0 to 1, the last step snapping within 0.1 of the target
        assert!((10..=11).contains(&steps));
    }

    #[test]
    fn targets_are_clamped_and_ignored_while_disabled() {
        let mut b = Bridges::new([false, false, true, false]);
        assert_eq!(b.set_target(0, 0.5), Err(TargetError::Disabled));
        b.set_enabled(true);
        assert_eq!(b.set_target(0, 3.0), Ok(()));
        assert_eq!(b.set_target(1, -7.0), Ok(()));
        assert_eq!(b.set_target(4, 0.5), Err(TargetError::NoSuchBridge));
        assert_eq!(b.set_target(2, f32::NAN), Err(TargetError::NotANumber));
        for _ in 0..20 {
            b.slew(4999);
        }
        assert_eq!(b.current(0), 1.0);
        assert_eq!(b.current(1), -1.0);
        assert_eq!(b.current(2), 0.0);
    }

    #[test]
    fn slew_drives_inverted_bridges_backwards() {
        let mut b = Bridges::new([false, true]);
        b.set_enabled(true);
        b.set_target(0, 0.3).unwrap();
        b.set_target(1, 0.3).unwrap();
        let out = b.slew(1000);
        assert_eq!(out[0], Output { phase: false, level: 100 });
        assert_eq!(out[1], Output { phase: true, level: 100 });
    }

    #[test]
    fn disabling_zeroes_targets_and_kill_zeroes_outputs() {
        let mut b = Bridges::new([false]);
        b.set_enabled(true);
        b.set_target(0, 1.0).unwrap();
        for _ in 0..5 {
            b.slew(100);
        }
        assert!(b.current(0) > 0.4);
        b.set_enabled(false);
        // Like C, disabling ramps down
        let out = b.slew(100);
        assert!(out[0].level > 0);
        b.set_enabled(true);
        b.set_target(0, 1.0).unwrap();
        b.slew(100);
        b.kill();
        assert!(!b.is_enabled());
        assert_eq!(b.slew(100)[0], Output { phase: false, level: 0 });
        assert_eq!(b.set_target(0, 1.0), Err(TargetError::Disabled));
    }

    #[test]
    fn wake_check_needs_low_then_high_nfault() {
        assert_eq!(count_awake(&[false, false, false, false], &[true, true, true, true]), 4);
        // Bridge 1 never pulled nFAULT low (missing), bridge 3 stayed in fault after the reset pulse
        assert_eq!(count_awake(&[false, true, false, false], &[true, true, true, false]), 2);
        assert_eq!(count_awake(&[], &[]), 0);
    }
}
