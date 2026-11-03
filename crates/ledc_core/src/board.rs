//! `LEDBoard` light behaviour around the [`Animator`], ported from `ledc_driver.c`, `led_set_rgb` in
//! `ledc_commands.c` and the subscription callbacks in `ros.c`.
//!
//! The firmware calls [`LedBoard::refresh`] every [`REFRESH_PERIOD_MS`](crate::pattern::REFRESH_PERIOD_MS),
//! [`LedBoard::singleton_tick`] every [`SINGLETON_PERIOD_MS`], [`LedBoard::check_depth`] every
//! [`DEPTH_MONITOR_PERIOD_MS`] and [`LedBoard::set_temperature`] every [`TEMPERATURE_MONITOR_PERIOD_MS`], and sends
//! every [`Output`] they return to the controllers.
//!
//! Brightness depends on where the board is: [`BENCH_MAX_BRIGHTNESS`] unless a depth reading in the last second
//! says it is under water, derated as the module warms past 30 °C and off above 50 °C.

use crate::l99ld20::{Buck, Chip, Cr1};
use crate::pattern::{Animator, Mode, Rgb};
use crate::thermal;

/// Period of the singleton flash timer (`SINGLETON_FLASH_PERIOD_MS`).
pub const SINGLETON_PERIOD_MS: u64 = 10;
/// Refreshes after a singleton flash before the next one may show (`LOOPS_PER_SINGLETON`).
pub const LOOPS_PER_SINGLETON: u32 = 5;
/// Period of the depth staleness check (`DEPTH_MONITOR_PERIOD_MS`).
pub const DEPTH_MONITOR_PERIOD_MS: u64 = 1000;
/// Period of the temperature check (`TEMPERATURE_MONITOR_PERIOD_MS`).
pub const TEMPERATURE_MONITOR_PERIOD_MS: u64 = 1000;
/// Period of the controller watchdog trigger (`CONTROLLER_WATCHDOG_PERIOD_MS`).
pub const CONTROLLER_WATCHDOG_PERIOD_MS: u64 = 3;
/// Peak inductor current select for all bucks (`NORMAL_OPERATION_PEAK_CURRENT`).
pub const PEAK_CURRENT: u8 = 45;

/// Brightness limit under water.
pub const WATER_MAX_BRIGHTNESS: f32 = 1.0;
/// Brightness limit out of the water, where the module has no cooling.
pub const BENCH_MAX_BRIGHTNESS: f32 = 0.01;
/// Depths below this (more negative) count as under water.
pub const UNDERWATER_MIN_DEPTH: f32 = -0.05;

/// `riptide_msgs2/msg/LedCommand` constants.
pub mod led_command {
    pub const MODE_SOLID: u8 = 0;
    pub const MODE_SLOW_FLASH: u8 = 1;
    pub const MODE_FAST_FLASH: u8 = 2;
    pub const MODE_BREATH: u8 = 3;
    pub const SINGLETON_FLASH: u8 = 4;
    pub const TARGET_NONE: u8 = 0;
    pub const TARGET_CCB: u8 = 1;
    /// The aluminium LED module driven by this board.
    pub const TARGET_ALU: u8 = 2;
    pub const TARGET_ALL: u8 = 3;
}

/// The bucks driving each colour of the module.
pub const RED: (Chip, Buck) = (Chip::Ledc1, Buck::Buck1);
pub const GREEN: (Chip, Buck) = (Chip::Ledc1, Buck::Buck2);
pub const BLUE: (Chip, Buck) = (Chip::Ledc2, Buck::Buck2);
pub const WHITE: (Chip, Buck) = (Chip::Ledc2, Buck::Buck1);

/// Something the light was asked to do.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Command {
    /// Show a colour in a mode (`led_set`; `led_clear` is solid black).
    Set(Mode, Rgb),
    /// Blink a colour briefly, for vision detections (`led_singleton`).
    Singleton(Rgb),
    /// Two quick pulses over the current pattern (`led_flash`).
    Flash(Rgb),
}

impl Command {
    /// Interprets a `command/led` message (`led_subscription_callback`). `None` if it is not for the aluminium
    /// module. Unknown modes turn the light off.
    pub fn from_led_command(color: Rgb, mode: u8, target: u8) -> Option<Command> {
        use led_command::*;
        if target & TARGET_ALU == 0 {
            return None;
        }
        Some(match mode {
            MODE_SOLID => Command::Set(Mode::Solid, color),
            MODE_SLOW_FLASH => Command::Set(Mode::SlowFlash, color),
            MODE_FAST_FLASH => Command::Set(Mode::FastFlash, color),
            MODE_BREATH => Command::Set(Mode::Breath, color),
            SINGLETON_FLASH => Command::Singleton(color),
            _ => Command::Set(Mode::Solid, Rgb::OFF),
        })
    }

    /// Feedback for the person at the physical kill switch (`physkill_notify_subscription_callback`): red when the
    /// switch is removed (`asserting_kill`), blue when it is inserted. The thrusters do not chime while software
    /// kill is active, so this is the only feedback they get.
    pub fn from_physkill_notify(asserting_kill: bool) -> Command {
        Command::Flash(if asserting_kill { Rgb::RED } else { Rgb::BLUE })
    }
}

/// Splits out the white component (`rgb_to_rgbw`).
pub fn rgb_to_rgbw(color: Rgb) -> (Rgb, u8) {
    let w = color.r.min(color.g).min(color.b);
    (Rgb::new(color.r - w, color.g - w, color.b - w), w)
}

/// Converts an 8 bit channel and a brightness scale to a 10 bit buck duty, truncating like the C double maths.
pub fn duty(channel: u8, brightness: f32) -> u16 {
    let duty = channel as f64 * (1023.0 / 255.0) * brightness as f64;
    (duty as u32).min(Cr1::MAX_DUTY as u32) as u16
}

/// What the module should show.
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Output {
    pub color: Rgb,
    /// Scale applied to every channel, 0 to 1.
    pub brightness: f32,
}

impl Output {
    pub const OFF: Output = Output { color: Rgb::OFF, brightness: 0.0 };

    /// Buck duties indexed by `[chip][buck]` (`led_set_rgb`).
    pub fn duties(&self) -> [[u16; 2]; 2] {
        let (rgb, w) = rgb_to_rgbw(self.color);
        let mut duties = [[0; 2]; 2];
        for ((chip, buck), channel) in [(RED, rgb.r), (GREEN, rgb.g), (BLUE, rgb.b), (WHITE, w)] {
            duties[chip as usize][buck as usize] = duty(channel, self.brightness);
        }
        duties
    }
}

/// The light state of an `LEDBoard`.
#[derive(Clone, Debug)]
pub struct LedBoard {
    animator: Animator,
    /// A singleton flash waiting to show (`do_singleton`).
    singleton: Option<Rgb>,
    /// The last singleton step showed the flash colour (`is_in_singleton`).
    in_singleton: bool,
    /// Refreshes until the next singleton may show (`next_singleton`).
    singleton_cooldown: u32,
    /// The last refresh's output.
    last: Output,
    underwater: bool,
    depth_stale: bool,
    got_new_depth: bool,
    temperature: f32,
    overtemp: bool,
}

impl Default for LedBoard {
    fn default() -> Self {
        Self::new()
    }
}

impl LedBoard {
    pub const fn new() -> Self {
        LedBoard {
            animator: Animator::new(),
            singleton: None,
            in_singleton: false,
            singleton_cooldown: LOOPS_PER_SINGLETON,
            last: Output::OFF,
            underwater: false,
            depth_stale: true,
            got_new_depth: false,
            temperature: 0.0,
            overtemp: false,
        }
    }

    pub fn command(&mut self, command: Command) {
        match command {
            Command::Set(mode, color) => self.animator.set(mode, color),
            Command::Singleton(color) => self.singleton = Some(color),
            Command::Flash(color) => self.animator.flash(color),
        }
    }

    /// Records a depth reading (`led_depth_set`).
    pub fn set_depth(&mut self, depth: f32) {
        self.underwater = depth < UNDERWATER_MIN_DEPTH;
        self.got_new_depth = true;
    }

    /// Marks the depth stale unless a reading arrived since the last check (`monitor_depth`).
    pub fn check_depth(&mut self) {
        self.depth_stale = !self.got_new_depth;
        self.got_new_depth = false;
    }

    /// Records the module temperature. Returns true while it is over [`thermal::MAX_OPERATING_TEMPERATURE_C`], when
    /// the caller raises `FAULT_LED_OVERTEMP` (`monitor_temperature`).
    pub fn set_temperature(&mut self, celsius: f32) -> bool {
        self.temperature = celsius;
        self.overtemp = thermal::is_overtemp(celsius);
        self.overtemp
    }

    /// Brightness limit before temperature derating.
    pub fn max_brightness(&self) -> f32 {
        if self.overtemp {
            0.0
        } else if self.underwater && !self.depth_stale {
            WATER_MAX_BRIGHTNESS
        } else {
            BENCH_MAX_BRIGHTNESS
        }
    }

    /// Advances the pattern and returns what to show (`update_led_status`).
    pub fn refresh(&mut self) -> Output {
        let color = self.animator.tick();
        self.last = Output { color, brightness: self.max_brightness() * thermal::derating(self.temperature) };
        self.singleton_cooldown = self.singleton_cooldown.saturating_sub(1);
        self.last
    }

    /// Shows a pending singleton flash once the cooldown allows (`handle_singleton_flash`).
    ///
    /// As in C, the flash colour stays until the next [`LedBoard::refresh`], and a singleton that arrives while the
    /// previous one is still marked as showing only restores the pattern colour.
    pub fn singleton_tick(&mut self) -> Option<Output> {
        if self.singleton_cooldown > 0 {
            return None;
        }
        let color = self.singleton.take()?;
        self.singleton_cooldown = LOOPS_PER_SINGLETON;
        self.in_singleton = !self.in_singleton;
        // C used the brightness before temperature derating here; derating it too keeps a hot module from flashing
        // at full brightness
        Some(if self.in_singleton { Output { color, ..self.last } } else { self.last })
    }
}

#[cfg(test)]
mod tests {
    use super::led_command::*;
    use super::*;

    const C: Rgb = Rgb::new(10, 20, 30);

    #[test]
    fn led_command_mapping() {
        assert_eq!(Command::from_led_command(C, MODE_SOLID, TARGET_CCB), None);
        assert_eq!(Command::from_led_command(C, MODE_SOLID, TARGET_NONE), None);
        assert_eq!(Command::from_led_command(C, MODE_SOLID, TARGET_ALU), Some(Command::Set(Mode::Solid, C)));
        assert_eq!(Command::from_led_command(C, MODE_BREATH, TARGET_ALL), Some(Command::Set(Mode::Breath, C)));
        assert_eq!(Command::from_led_command(C, MODE_SLOW_FLASH, 2), Some(Command::Set(Mode::SlowFlash, C)));
        assert_eq!(Command::from_led_command(C, MODE_FAST_FLASH, 2), Some(Command::Set(Mode::FastFlash, C)));
        assert_eq!(Command::from_led_command(C, SINGLETON_FLASH, 2), Some(Command::Singleton(C)));
        assert_eq!(Command::from_led_command(C, 9, 2), Some(Command::Set(Mode::Solid, Rgb::OFF)));
        assert_eq!(Command::from_physkill_notify(true), Command::Flash(Rgb::RED));
        assert_eq!(Command::from_physkill_notify(false), Command::Flash(Rgb::BLUE));
    }

    #[test]
    fn rgbw_and_duties() {
        assert_eq!(rgb_to_rgbw(Rgb::new(200, 100, 50)), (Rgb::new(150, 50, 0), 50));
        assert_eq!(duty(255, 1.0), 1023);
        assert_eq!(duty(255, 0.01), 10);
        assert_eq!(duty(0, 1.0), 0);
        assert_eq!(duty(255, 2.0), 1023);
        let out = Output { color: Rgb::new(255, 128, 64), brightness: 1.0 };
        // White takes 64, leaving red 191, green 64, blue 0
        assert_eq!(out.duties(), [[duty(191, 1.0), duty(64, 1.0)], [duty(64, 1.0), 0]]);
        assert_eq!(out.duties()[0][0], 766);
        assert_eq!(Output::OFF.duties(), [[0; 2]; 2]);
    }

    #[test]
    fn brightness_follows_depth() {
        let mut board = LedBoard::new();
        board.command(Command::Set(Mode::Solid, C));
        assert_eq!(board.refresh().brightness, BENCH_MAX_BRIGHTNESS);
        // A reading only counts once the monitor has seen it arrive
        board.set_depth(-1.0);
        assert_eq!(board.refresh().brightness, BENCH_MAX_BRIGHTNESS);
        board.check_depth();
        assert_eq!(board.refresh().brightness, WATER_MAX_BRIGHTNESS);
        // Surfacing
        board.set_depth(0.0);
        assert_eq!(board.refresh().brightness, BENCH_MAX_BRIGHTNESS);
        // Readings stop: stale after the next check
        board.set_depth(-1.0);
        board.check_depth();
        assert_eq!(board.refresh().brightness, WATER_MAX_BRIGHTNESS);
        board.check_depth();
        assert_eq!(board.refresh().brightness, BENCH_MAX_BRIGHTNESS);
    }

    #[test]
    fn temperature_derates_and_cuts_off() {
        let mut board = LedBoard::new();
        board.set_depth(-1.0);
        board.check_depth();
        assert!(!board.set_temperature(40.0));
        assert!((board.refresh().brightness - 0.5).abs() < 1e-6);
        assert!(board.set_temperature(55.0));
        assert_eq!(board.refresh().brightness, 0.0);
        assert!(!board.set_temperature(20.0));
        assert_eq!(board.refresh().brightness, 1.0);
    }

    #[test]
    fn singleton_waits_for_cooldown_and_alternates() {
        let mut board = LedBoard::new();
        board.command(Command::Set(Mode::Solid, C));
        board.command(Command::Singleton(Rgb::RED));
        // Five refreshes must pass after start up
        for _ in 0..4 {
            board.refresh();
            assert_eq!(board.singleton_tick(), None);
        }
        let last = board.refresh();
        assert_eq!(board.singleton_tick(), Some(Output { color: Rgb::RED, ..last }));
        // Only once per request
        assert_eq!(board.singleton_tick(), None);
        // The next request after the cooldown restores the pattern colour instead of flashing
        board.command(Command::Singleton(Rgb::RED));
        for _ in 0..5 {
            assert_eq!(board.singleton_tick(), None);
            board.refresh();
        }
        assert_eq!(board.singleton_tick(), Some(Output { color: C, ..last }));
        board.command(Command::Singleton(Rgb::BLUE));
        for _ in 0..5 {
            board.refresh();
        }
        assert_eq!(board.singleton_tick().map(|o| o.color), Some(Rgb::BLUE));
    }

    #[test]
    fn physkill_flash_overrides_pattern() {
        let mut board = LedBoard::new();
        board.command(Command::Set(Mode::Solid, C));
        board.command(Command::from_physkill_notify(true));
        assert_eq!(board.refresh().color, Rgb::OFF);
        assert_eq!(board.refresh().color, Rgb::new(85, 0, 0));
        for _ in 0..10 {
            board.refresh();
        }
        assert_eq!(board.refresh().color, C);
    }
}
