//! The OLED menu state machine of the C `display.c` (`display_tick`), without drawing or I2C.
//!
//! The firmware calls [`Display::tick`] every few milliseconds with the reed switch level and the pack state, then
//! carries out the returned [`Actions`] in order: display on, contrast, then the drawing (rendered by
//! [`crate::screens`]).
//!
//! The reed switch is the only input. A tap (release within [`SWITCH_HOLD_TIME_MS`]) wakes the screen, brightens a
//! dimmed screen, opens the menu or moves the menu selection. Holding it selects a menu entry. Screens time out
//! after [`HOLD_MS`] (dimming [`DIM_TIME_MS`] before), after which the display turns off.

use crate::monitor::BattState;

/// Redraw period when nothing happens (`DISPLAY_UPDATE_INTERVAL_MS`).
pub const UPDATE_INTERVAL_MS: u64 = 300;
/// How long the splash screen shows (`DISPLAY_SPLASH_TIME_MS`).
pub const SPLASH_TIME_MS: u64 = 1000;
/// How long an interactive screen stays up (`DISPLAY_HOLD_MS`).
pub const HOLD_MS: u64 = 15_000;
/// How long before the timeout the screen dims (`DISPLAY_DIM_TIME_MS`).
pub const DIM_TIME_MS: u64 = 5000;
/// Contrast while dimmed (`DISPLAY_DIM_CONTRAST`).
pub const DIM_CONTRAST: u8 = 0x10;
/// How long a message stays up (`DISPLAY_MESSAGE_TIME_MS`).
pub const MESSAGE_TIME_MS: u64 = 2000;
/// How long the switch must be held to count as a hold (`DISPLAY_SWITCH_HOLD_TIME_MS`).
pub const SWITCH_HOLD_TIME_MS: u64 = 2000;

/// Never (`at_the_end_of_time`).
const END_OF_TIME: u64 = u64::MAX;

/// Screens (`enum screen_type`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Screen {
    Soc,
    Current,
    PackStatus,
    /// Never implemented in C; shows "Invalid Screen".
    LifeStats,
    CellVoltages,
    BqMissing,
    BqInitializing,
    BqPermFail,
    BqShutdown,
    BqPowerCycle,
    BqXdsg,
}

/// Menus (`enum menu_type`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Menu {
    Main,
    More,
}

/// Menu actions (`enum action_type`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Action {
    /// Never implemented in C; shows "Not Yet Implemented".
    PowerCycle,
}

/// What the display shows (`display_target_t`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Target {
    Startup,
    Message,
    Splash,
    Screen(Screen),
    Menu(Menu),
    DoAction(Action),
    Off,
}

/// Number of entries in every menu (`NUM_MENU_ENTRIES`).
pub const MENU_ENTRIES: usize = 4;

/// The menu entries and where they lead (`menu_def_main`, `menu_def_more`).
pub fn menu_entries(menu: Menu) -> &'static [(&'static str, Target); MENU_ENTRIES] {
    const MAIN: [(&str, Target); MENU_ENTRIES] = [
        ("1. Relative SOC", Target::Screen(Screen::Soc)),
        ("2. Current/Voltage", Target::Screen(Screen::Current)),
        ("3. Pack Status", Target::Screen(Screen::PackStatus)),
        ("4. More...", Target::Menu(Menu::More)),
    ];
    const MORE: [(&str, Target); MENU_ENTRIES] = [
        ("Detailed Stats", Target::Screen(Screen::LifeStats)),
        ("Cell Voltages", Target::Screen(Screen::CellVoltages)),
        ("Power Cycle Robot", Target::DoAction(Action::PowerCycle)),
        ("Back <-", Target::Menu(Menu::Main)),
    ];
    match menu {
        Menu::Main => &MAIN,
        Menu::More => &MORE,
    }
}

/// What to draw.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Draw {
    Splash,
    Screen(Screen),
    /// The menu with the selected entry and the hold progress circle (degrees, 0..=360).
    Menu {
        menu: Menu,
        selected: usize,
        sweep: u16,
    },
    NotImplemented,
    /// Clear the screen, then turn the display off.
    ClearAndOff,
}

/// What the firmware must do after a tick, in field order.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Actions {
    /// Send the display on command.
    pub turn_on: bool,
    /// Set the contrast.
    pub contrast: Option<u8>,
    pub draw: Option<Draw>,
}

/// Inputs to one tick.
#[derive(Clone, Copy, Debug)]
pub struct Inputs {
    pub now: u64,
    /// Reed switch level (high while the magnet is held near it).
    pub switch: bool,
    pub battery: BattState,
    /// `FAULT_BQ40_SAFETY_STATUS` is raised.
    pub safety_fault: bool,
}

/// Display state (`struct display_state`).
#[derive(Clone, Debug)]
pub struct Display {
    last_batt_state: Option<BattState>,
    cur_target: Target,
    next_state_time: u64,
    next_redraw: u64,
    btn_hold_timeout: Option<u64>,
    last_switch_state: bool,
    selected_idx: usize,
    last_contrast: u8,
}

impl Display {
    /// Starts in the startup state; the first tick shows the splash screen (`display_init`).
    pub const fn new(switch: bool) -> Self {
        Display {
            // C forced the first tick to see a battery state change
            last_batt_state: None,
            cur_target: Target::Startup,
            next_state_time: 0,
            next_redraw: 0,
            btn_hold_timeout: None,
            last_switch_state: switch,
            selected_idx: 0,
            last_contrast: 0,
        }
    }

    /// The display is showing something (`display_check_on`).
    pub fn is_on(&self) -> bool {
        self.cur_target != Target::Off
    }

    pub fn target(&self) -> Target {
        self.cur_target
    }

    /// Runs the state machine once. `message_pending` forces the message state; the caller has already drawn the
    /// message (`display_show_msg`).
    pub fn tick(&mut self, input: Inputs, message_pending: bool) -> Actions {
        let now = input.now;
        let reached = |t: u64| now >= t;
        let mut target = self.cur_target;
        let mut new_selected_idx = self.selected_idx;
        let mut battery_state = input.battery;

        // Stage 1: current state
        let mut should_dim = matches!(self.cur_target, Target::Menu(_) | Target::Screen(_))
            && self.next_state_time < now.saturating_add(DIM_TIME_MS);

        let switch = input.switch;
        let mut tap = false;
        let mut hold = false;
        if switch != self.last_switch_state {
            let was_hold = self.btn_hold_timeout.is_none_or(reached);
            if !switch && !was_hold {
                tap = true;
            }
            self.btn_hold_timeout = Some(now + SWITCH_HOLD_TIME_MS);
        }
        if let Some(timeout) = self.btn_hold_timeout
            && reached(timeout)
        {
            hold = switch;
            // Don't fire again until the switch is pressed again
            self.btn_hold_timeout = None;
        }
        self.last_switch_state = switch;

        // A tap keeps interactive screens alive
        if tap && matches!(self.cur_target, Target::Screen(_) | Target::Menu(_)) {
            self.next_state_time = now + HOLD_MS;
        }

        let mut show_default = self.last_batt_state != Some(battery_state);
        if self.cur_target == Target::Splash && !reached(self.next_state_time) {
            // Let the splash screen stay up for a bit
            show_default = false;
            if let Some(last) = self.last_batt_state {
                battery_state = last;
            }
        }

        // Stage 2: next target from timeouts and the switch
        if reached(self.next_state_time) {
            match self.cur_target {
                Target::Startup => target = Target::Splash,
                Target::Splash | Target::Message => show_default = true,
                _ => target = Target::Off,
            }
        }

        if tap {
            match self.cur_target {
                Target::Menu(_) => new_selected_idx = (new_selected_idx + 1) % MENU_ENTRIES,
                Target::Screen(screen) => {
                    // A dimmed screen just brightens; a bright one opens the menu
                    if !should_dim {
                        match screen {
                            Screen::BqInitializing
                            | Screen::BqMissing
                            | Screen::BqPermFail
                            | Screen::BqShutdown
                            | Screen::BqPowerCycle => {}
                            Screen::BqXdsg | Screen::Soc | Screen::Current | Screen::PackStatus => {
                                target = Target::Menu(Menu::Main)
                            }
                            Screen::LifeStats | Screen::CellVoltages => target = Target::Menu(Menu::More),
                        }
                    } else {
                        should_dim = false;
                    }
                }
                Target::Off => show_default = true,
                _ => {}
            }
        }

        if hold {
            match self.cur_target {
                Target::Menu(menu) => target = menu_entries(menu)[self.selected_idx].1,
                Target::Off => show_default = true,
                _ => {}
            }
        }

        if show_default {
            target = default_target(battery_state, input.safety_fault);
        }

        if message_pending {
            target = Target::Message;
        }

        // Stage 3: what changed
        let transitioned = target != self.cur_target;
        let redraw = reached(self.next_redraw) || transitioned || new_selected_idx != self.selected_idx;

        let mut contrast = if target == Target::Off { 0 } else { 0xFF };
        if !transitioned && should_dim {
            contrast = DIM_CONTRAST;
        }

        // Stage 4: update the display
        let mut actions = Actions { turn_on: transitioned && self.cur_target == Target::Off, ..Actions::default() };
        if self.last_contrast != contrast {
            actions.contrast = Some(contrast);
            self.last_contrast = contrast;
        }

        if redraw {
            match target {
                Target::Splash => {
                    if transitioned {
                        self.next_state_time = now + SPLASH_TIME_MS;
                        actions.draw = Some(Draw::Splash);
                    }
                }
                Target::Message => {
                    // The caller drew the message
                    if transitioned {
                        self.next_state_time = now + MESSAGE_TIME_MS;
                    }
                }
                Target::Screen(screen) => {
                    if transitioned {
                        self.next_state_time = now + HOLD_MS;
                    }
                    actions.draw = Some(Draw::Screen(screen));
                }
                Target::Menu(menu) => {
                    if transitioned {
                        new_selected_idx = 0;
                        self.next_state_time = now + HOLD_MS;
                    }
                    let sweep = match self.btn_hold_timeout {
                        Some(timeout) if switch => hold_sweep(now, timeout),
                        _ => 0,
                    };
                    actions.draw = Some(Draw::Menu { menu, selected: new_selected_idx, sweep });
                }
                Target::DoAction(_) => {
                    if transitioned {
                        self.next_state_time = now + HOLD_MS;
                    }
                    actions.draw = Some(Draw::NotImplemented);
                }
                Target::Off | Target::Startup => {
                    if transitioned {
                        // Only the switch leaves the off state
                        self.next_state_time = END_OF_TIME;
                        actions.draw = Some(Draw::ClearAndOff);
                    }
                }
            }
            self.next_redraw = now + UPDATE_INTERVAL_MS;
        }

        // Stage 5: save state
        self.cur_target = target;
        self.selected_idx = new_selected_idx;
        self.last_batt_state = Some(battery_state);
        actions
    }
}

/// The screen for a pack state.
fn default_target(state: BattState, safety_fault: bool) -> Target {
    match state {
        BattState::Uninitialized => Target::Splash,
        BattState::Disconnected => Target::Screen(Screen::BqMissing),
        BattState::NeedsShutdown => Target::Screen(Screen::BqShutdown),
        BattState::Initializing => Target::Screen(Screen::BqInitializing),
        BattState::PermanentFail => Target::Screen(Screen::BqPermFail),
        BattState::PowerCycle => Target::Screen(Screen::BqPowerCycle),
        BattState::Xdsg => Target::Screen(Screen::BqXdsg),
        // The C TODO for latched off falls through to the normal screens
        BattState::LatchOff | BattState::Removed | BattState::Discharging | BattState::Charging => {
            Target::Screen(if safety_fault { Screen::PackStatus } else { Screen::Soc })
        }
    }
}

/// How far around the hold progress circle is, in degrees.
fn hold_sweep(now: u64, timeout: u64) -> u16 {
    let left_ms = timeout as i64 - now as i64;
    let held_ms = SWITCH_HOLD_TIME_MS as i64 - left_ms;
    (held_ms.max(0) * 360 / SWITCH_HOLD_TIME_MS as i64).clamp(0, 360) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Harness {
        display: Display,
        now: u64,
        switch: bool,
        battery: BattState,
        safety_fault: bool,
    }

    impl Harness {
        fn new() -> Self {
            Harness {
                display: Display::new(false),
                now: 0,
                switch: false,
                battery: BattState::Uninitialized,
                safety_fault: false,
            }
        }

        fn tick(&mut self) -> Actions {
            let input =
                Inputs { now: self.now, switch: self.switch, battery: self.battery, safety_fault: self.safety_fault };
            self.display.tick(input, false)
        }

        /// Ticks every 10 ms for `ms`, returning every draw.
        fn run(&mut self, ms: u64) -> std::vec::Vec<Draw> {
            let mut draws = std::vec::Vec::new();
            for _ in 0..ms / 10 {
                self.now += 10;
                if let Some(d) = self.tick().draw {
                    draws.push(d);
                }
            }
            draws
        }

        fn tap(&mut self) -> std::vec::Vec<Draw> {
            self.switch = true;
            let mut draws = self.run(100);
            self.switch = false;
            draws.extend(self.run(20));
            draws
        }

        fn hold(&mut self) -> std::vec::Vec<Draw> {
            self.switch = true;
            let mut draws = self.run(2100);
            self.switch = false;
            draws.extend(self.run(20));
            draws
        }
    }

    #[test]
    fn boot_shows_splash_then_the_pack_screen_then_turns_off() {
        let mut h = Harness::new();
        let first = h.tick();
        assert_eq!(first.draw, Some(Draw::Splash));
        assert_eq!(first.contrast, Some(0xFF));
        assert!(h.display.is_on());

        // The pack connects during the splash: the splash stays for its full second
        h.battery = BattState::Discharging;
        let draws = h.run(990);
        assert!(draws.is_empty(), "{draws:?}");
        let draws = h.run(20);
        assert_eq!(draws[0], Draw::Screen(Screen::Soc));
        assert_eq!(h.display.target(), Target::Screen(Screen::Soc));

        // Redrawn every 300 ms, dimmed for the last 5 s, then off after 15 s
        let mut dimmed_at = None;
        let mut off_at = None;
        for _ in 0..1600 {
            h.now += 10;
            let a = h.tick();
            if a.contrast == Some(DIM_CONTRAST) {
                dimmed_at = Some(h.now);
            }
            if a.draw == Some(Draw::ClearAndOff) {
                off_at = Some(h.now);
                assert_eq!(a.contrast, Some(0));
            }
        }
        let start = 1000;
        assert_eq!(off_at, Some(start + HOLD_MS));
        assert_eq!(dimmed_at, Some(start + HOLD_MS - DIM_TIME_MS + 10));
        assert!(!h.display.is_on());
    }

    #[test]
    fn tap_wakes_and_opens_menu_and_hold_selects() {
        let mut h = Harness::new();
        h.battery = BattState::Charging;
        h.run(30_000);
        assert!(!h.display.is_on());

        // Wake on tap: display on and the default screen
        h.switch = true;
        h.now += 10;
        h.tick();
        h.switch = false;
        h.now += 10;
        let a = h.tick();
        assert!(a.turn_on);
        assert_eq!(a.draw, Some(Draw::Screen(Screen::Soc)));

        // Tap on a bright screen opens the main menu at the top
        let draws = h.tap();
        assert_eq!(draws.last(), Some(&Draw::Menu { menu: Menu::Main, selected: 0, sweep: 0 }));
        // Taps move the selection, wrapping around
        h.tap();
        let draws = h.tap();
        assert_eq!(draws.last(), Some(&Draw::Menu { menu: Menu::Main, selected: 2, sweep: 0 }));

        // Holding shows the progress circle, then selects "3. Pack Status"
        let draws = h.hold();
        assert!(draws.iter().any(|d| matches!(d, Draw::Menu { sweep, .. } if *sweep > 90 && *sweep < 360)));
        assert_eq!(h.display.target(), Target::Screen(Screen::PackStatus));

        // From the pack screens a tap opens the main menu; "4. More..." then "Cell Voltages"
        h.tap();
        for _ in 0..3 {
            h.tap();
        }
        h.hold();
        assert_eq!(h.display.target(), Target::Menu(Menu::More));
        h.tap();
        h.hold();
        assert_eq!(h.display.target(), Target::Screen(Screen::CellVoltages));
        // A tap here opens the More menu
        h.tap();
        assert_eq!(h.display.target(), Target::Menu(Menu::More));
    }

    #[test]
    fn tap_on_dimmed_screen_only_brightens() {
        let mut h = Harness::new();
        h.battery = BattState::Discharging;
        h.run(1010);
        h.run(11_000);
        assert_eq!(h.display.last_contrast, DIM_CONTRAST);
        h.tap();
        assert_eq!(h.display.target(), Target::Screen(Screen::Soc));
        assert_eq!(h.display.last_contrast, 0xFF);
        // ...and restarts the timeout
        h.run(5000);
        assert!(h.display.is_on());
        // A second tap opens the menu
        h.tap();
        assert_eq!(h.display.target(), Target::Menu(Menu::Main));
    }

    #[test]
    fn battery_state_changes_switch_screens() {
        let mut h = Harness::new();
        h.battery = BattState::Disconnected;
        h.run(1100);
        assert_eq!(h.display.target(), Target::Screen(Screen::BqMissing));
        // Taps on the error screens do nothing
        h.tap();
        assert_eq!(h.display.target(), Target::Screen(Screen::BqMissing));
        h.battery = BattState::Initializing;
        h.run(20);
        assert_eq!(h.display.target(), Target::Screen(Screen::BqInitializing));
        h.safety_fault = true;
        h.battery = BattState::Removed;
        h.run(20);
        assert_eq!(h.display.target(), Target::Screen(Screen::PackStatus));
        h.battery = BattState::Xdsg;
        h.run(20);
        assert_eq!(h.display.target(), Target::Screen(Screen::BqXdsg));
        // XDSG offers the main menu
        h.tap();
        assert_eq!(h.display.target(), Target::Menu(Menu::Main));
    }

    #[test]
    fn messages_hold_then_return_to_default() {
        let mut h = Harness::new();
        h.battery = BattState::Discharging;
        h.run(1100);
        h.now += 10;
        let input = Inputs { now: h.now, switch: false, battery: h.battery, safety_fault: false };
        let a = h.display.tick(input, true);
        assert_eq!(a.draw, None);
        assert_eq!(h.display.target(), Target::Message);
        let draws = h.run(1980);
        assert!(draws.is_empty());
        let draws = h.run(20);
        assert_eq!(draws, [Draw::Screen(Screen::Soc)]);
    }

    #[test]
    fn hold_sweep_clamps() {
        assert_eq!(hold_sweep(0, 2000), 0);
        assert_eq!(hold_sweep(1000, 2000), 180);
        assert_eq!(hold_sweep(2500, 2000), 360);
    }
}
