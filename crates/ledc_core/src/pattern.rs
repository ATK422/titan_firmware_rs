//! Colour patterns of the `LEDBoard`, ported from `update_led_status` in `ledc_driver.c`.
//!
//! The pattern advances one step per refresh ([`REFRESH_PERIOD_MS`]). A status flash (two quick pulses of another
//! colour) can be laid over it without losing the colour command.

/// Time between refreshes (`LED_UPDATE_INTERVAL_MS`).
pub const REFRESH_PERIOD_MS: u64 = 50;
/// Refreshes per breath cycle and period of the pattern timer (`LED_TIMER_PERIOD_TICKS`).
pub const TIMER_PERIOD: u32 = 120;
/// Refreshes per fast flash cycle.
pub const FAST_FLASH_PERIOD: u32 = 6;
/// Refreshes per slow flash cycle.
pub const SLOW_FLASH_PERIOD: u32 = 40;
/// Refreshes per status flash pulse.
pub const FLASH_PULSE_PERIOD: u32 = 6;
/// Pulses per status flash.
pub const FLASH_PULSE_COUNT: u32 = 2;

/// An 8 bit per channel colour.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Rgb {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl Rgb {
    pub const OFF: Rgb = Rgb::new(0, 0, 0);
    pub const RED: Rgb = Rgb::new(255, 0, 0);
    pub const BLUE: Rgb = Rgb::new(0, 0, 255);

    pub const fn new(r: u8, g: u8, b: u8) -> Self {
        Rgb { r, g, b }
    }

    fn map(self, f: impl Fn(u8) -> u8) -> Self {
        Rgb { r: f(self.r), g: f(self.g), b: f(self.b) }
    }

    /// Scales each channel by `num / den`.
    fn scale(self, num: u32, den: u32) -> Self {
        self.map(|c| (c as u32 * num / den) as u8)
    }
}

/// How the colour is shown (`enum status_mode`).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Mode {
    /// Steady colour.
    #[default]
    Solid,
    /// On and off, 1 s each.
    SlowFlash,
    /// On and off, 150 ms each.
    FastFlash,
    /// Fades in and out over 6 s.
    Breath,
}

/// Triangle fade over `period` steps: up from off to `color` in the first half, back down in the second.
fn fade(color: Rgb, tick: u32, period: u32) -> Rgb {
    let half = period / 2;
    if tick % period < half {
        color.scale(tick, half)
    } else {
        let down = color.scale(tick - half, half);
        Rgb { r: color.r - down.r, g: color.g - down.g, b: color.b - down.b }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct Flash {
    color: Rgb,
    timer: u32,
    count: u32,
}

/// The pattern state, advanced once per refresh.
#[derive(Clone, Debug, Default)]
pub struct Animator {
    mode: Mode,
    target: Rgb,
    timer: u32,
    flash: Option<Flash>,
}

impl Animator {
    pub const fn new() -> Self {
        Animator { mode: Mode::Solid, target: Rgb::OFF, timer: 0, flash: None }
    }

    /// Shows `color` in `mode`, restarting the pattern (`led_set`).
    pub fn set(&mut self, mode: Mode, color: Rgb) {
        self.timer = 0;
        self.mode = mode;
        self.target = color;
    }

    /// Flashes `color` twice over the pattern, restarting any flash in progress (`led_flash`).
    pub fn flash(&mut self, color: Rgb) {
        self.flash = Some(Flash { color, timer: 0, count: 0 });
    }

    /// Advances one refresh and returns the colour to show.
    pub fn tick(&mut self) -> Rgb {
        let color = self.mode_color();
        self.timer = (self.timer + 1) % TIMER_PERIOD;
        match &mut self.flash {
            Some(flash) => {
                let color = fade(flash.color, flash.timer, FLASH_PULSE_PERIOD);
                flash.timer += 1;
                if flash.timer >= FLASH_PULSE_PERIOD {
                    flash.timer = 0;
                    flash.count += 1;
                    if flash.count >= FLASH_PULSE_COUNT {
                        self.flash = None;
                    }
                }
                color
            }
            None => color,
        }
    }

    fn mode_color(&self) -> Rgb {
        match self.mode {
            Mode::Solid => self.target,
            Mode::FastFlash | Mode::SlowFlash => {
                let period = if self.mode == Mode::FastFlash { FAST_FLASH_PERIOD } else { SLOW_FLASH_PERIOD };
                if self.timer % period < period / 2 { self.target } else { Rgb::OFF }
            }
            // Squared to make the fade look smoother
            Mode::Breath => fade(self.target, self.timer, TIMER_PERIOD).map(|c| ((c as u32 * c as u32) >> 8) as u8),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(animator: &mut Animator, n: usize) -> [Rgb; 256] {
        assert!(n <= 256);
        let mut out = [Rgb::OFF; 256];
        for slot in out.iter_mut().take(n) {
            *slot = animator.tick();
        }
        out
    }

    const C: Rgb = Rgb::new(200, 100, 40);

    #[test]
    fn starts_off_and_solid_holds() {
        let mut a = Animator::new();
        assert_eq!(a.tick(), Rgb::OFF);
        a.set(Mode::Solid, C);
        assert!(run(&mut a, 200).iter().take(200).all(|&c| c == C));
    }

    #[test]
    fn flash_modes_have_half_duty() {
        for (mode, period) in [(Mode::FastFlash, FAST_FLASH_PERIOD), (Mode::SlowFlash, SLOW_FLASH_PERIOD)] {
            let mut a = Animator::new();
            a.set(mode, C);
            let out = run(&mut a, 240);
            for (i, &c) in out.iter().take(240).enumerate() {
                let on = (i as u32 % TIMER_PERIOD) % period < period / 2;
                assert_eq!(c, if on { C } else { Rgb::OFF }, "{mode:?} step {i}");
            }
        }
    }

    #[test]
    fn breath_rises_and_falls_squared() {
        let mut a = Animator::new();
        a.set(Mode::Breath, Rgb::new(255, 128, 0));
        let out = run(&mut a, 121);
        assert_eq!(out[0], Rgb::OFF);
        // Halfway up: 255 * 30 / 60 = 127, squared 63
        assert_eq!(out[30], Rgb::new(63, 16, 0));
        // Peak at the middle of the cycle
        assert_eq!(out[60], Rgb::new(254, 64, 0));
        // Falling: 255 - 255 * 30 / 60 = 128, squared 64
        assert_eq!(out[90], Rgb::new(64, 16, 0));
        // And the cycle restarts
        assert_eq!(out[120], Rgb::OFF);
        assert!(out[1..60].windows(2).all(|w| w[0].r <= w[1].r));
        assert!(out[60..120].windows(2).all(|w| w[0].r >= w[1].r));
    }

    #[test]
    fn set_restarts_pattern() {
        let mut a = Animator::new();
        a.set(Mode::SlowFlash, C);
        run(&mut a, 25);
        a.set(Mode::SlowFlash, C);
        assert_eq!(a.tick(), C);
    }

    #[test]
    fn status_flash_pulses_twice_then_restores() {
        let mut a = Animator::new();
        a.set(Mode::Solid, C);
        a.tick();
        a.flash(Rgb::BLUE);
        let out = run(&mut a, 13);
        let pulse = [0, 85, 170, 255, 170, 85];
        for (i, &c) in out.iter().take(12).enumerate() {
            assert_eq!(c, Rgb::new(0, 0, pulse[i % 6]), "step {i}");
        }
        assert_eq!(out[12], C);
    }
}
