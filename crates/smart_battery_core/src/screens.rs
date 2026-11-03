//! Screen layouts of the C `display.c`, drawn into a [`Framebuffer`].

use core::fmt::Write;

use crate::bitmaps;
use crate::display::{self, Draw, Menu, Screen};
use crate::faults;
use crate::fonts::{FONT_6X8, FONT_7X10, FONT_11X18};
use crate::gfx::{Color, Framebuffer, WIDTH};
use crate::monitor::BattState;
use crate::sbs::{BatteryInfo, MfgInfo};

/// Everything the screens show, sampled when drawing.
#[derive(Clone, Copy, Debug, Default)]
pub struct Snapshot {
    pub now: u64,
    pub state: BattState,
    pub info: BatteryInfo,
    /// Last manufacturer info read from the pack.
    pub mfg: MfgInfo,
    /// The pack is ready and its manufacturer info was read (`mfg_readout_okay`), shown as the CAN icon.
    pub mfg_readout_okay: bool,
    /// Raised faults (`*fault_list_reg`).
    pub faults: u32,
    /// The arguments last passed to the gauge faults (`safety_fault_data[...].extra_data`).
    pub not_connected_arg: u32,
    pub safety_status_arg: u32,
    pub pf_status_arg: u32,
}

impl Snapshot {
    fn fault(&self, id: u8) -> bool {
        self.faults & (1 << id) != 0
    }
}

/// Short safety status names for the pack status screen (`protection_names`).
pub const PROTECTION_NAMES: &[(u8, &str)] = &[
    (0, "Cell Undervolt"),
    (1, "Cell Overvolt"),
    (2, "Overcur. Chg1"),
    (3, "Overcur. Chg2"),
    (4, "Overcur. Dsg1"),
    (5, "Overcur. Dsg2"),
    (6, "Overld. Dsg"),
    (7, "OvrldDsg Latch"),
    (8, "ASCC"),
    (9, "ASCCL"),
    (10, "ASCD"),
    (11, "ASCDL"),
    (12, "OverTemp Chg"),
    (13, "OverTemp Dsg"),
    (14, "CUVC"),
    (16, "OverTemp FET"),
    (18, "PreChg Timeout"),
    (20, "Charge Timeout"),
    (22, "Overcharge"),
    (23, "Overcharge Cur"),
    (24, "Overcharge Volt"),
    (25, "OverPreChg Cur"),
    (26, "UnderTemp Chg"),
    (27, "UnderTemp Dsg"),
    (28, "OverVolt Latch"),
    (29, "OverCur. Dsg"),
];

/// Short permanent failure names for the pack status screen (`pf_names`).
pub const PF_NAMES: &[(u8, &str)] = &[
    (0, "Cell Undervolt"),
    (1, "Cell Overvolt"),
    (2, "Overcur. Chg"),
    (3, "Overcur. Dsg"),
    (4, "OverTemp Cell"),
    (5, "OverVolt Latch"),
    (6, "OverTemp FET"),
    (7, "QMax Imbal"),
    (8, "Cell Bal Fail"),
    (9, "Impedance Fail"),
    (10, "Capacity Deg."),
    (11, "VoltImbal@Rest"),
    (12, "VoltImbal@Act"),
    (13, "Ovrld. Dsg"),
    (14, "ASCCL"),
    (15, "ASCDL"),
    (16, "Chg FET Fail"),
    (17, "Dsg FET Fail"),
    (18, "Overcur. Dsg"),
    (19, "ChemFUSE Fail"),
    (20, "AFE Reg Fail"),
    (21, "AFE Comm Fail"),
    (22, "2ndLvl Prot"),
    (23, "PTC Fail"),
    (24, "Instr. Flash"),
    (26, "Data Flash"),
    (28, "Thermistor1"),
    (29, "Thermistor2"),
    (30, "Thermistor3"),
    (31, "Thermistor4"),
];

/// Message icons (`enum display_msg_icon`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Icon {
    None,
    Info,
    Alert,
}

/// A small fixed capacity string for formatting screen text.
pub type Text = heapless_str::Str<24>;

/// Minimal `heapless::String` stand-in so the crate has no dependencies beyond `libm`.
pub mod heapless_str {
    use core::fmt;

    /// A string of at most `N` bytes; writes past the end are truncated.
    #[derive(Clone, Copy)]
    pub struct Str<const N: usize> {
        buf: [u8; N],
        len: usize,
    }

    impl<const N: usize> Default for Str<N> {
        fn default() -> Self {
            Str { buf: [0; N], len: 0 }
        }
    }

    impl<const N: usize> Str<N> {
        pub fn as_str(&self) -> &str {
            // Only whole UTF-8 sequences are ever copied in
            core::str::from_utf8(&self.buf[..self.len]).unwrap_or("")
        }

        pub fn len(&self) -> usize {
            self.len
        }

        pub fn is_empty(&self) -> bool {
            self.len == 0
        }
    }

    impl<const N: usize> fmt::Write for Str<N> {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            for ch in s.chars() {
                let mut tmp = [0u8; 4];
                let bytes = ch.encode_utf8(&mut tmp).as_bytes();
                if self.len + bytes.len() > N {
                    return Err(fmt::Error);
                }
                self.buf[self.len..self.len + bytes.len()].copy_from_slice(bytes);
                self.len += bytes.len();
            }
            Ok(())
        }
    }

    impl<const N: usize> fmt::Debug for Str<N> {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            fmt::Debug::fmt(self.as_str(), f)
        }
    }

    impl<const N: usize> PartialEq<&str> for Str<N> {
        fn eq(&self, other: &&str) -> bool {
            self.as_str() == *other
        }
    }
}

macro_rules! text {
    ($($arg:tt)*) => {{
        let mut t = Text::default();
        let _ = write!(t, $($arg)*);
        t
    }};
}

/// Remaining time as shown on the SOC screen (`display_format_remaining_time`).
pub fn format_remaining_time(minutes: u16) -> Text {
    if minutes == u16::MAX {
        text!("N/A")
    } else if minutes > 10 * 60 {
        // Over 10 hours: whole hours
        text!("{} h", minutes / 60)
    } else if minutes > 60 {
        text!("{}.{} h", minutes / 60, (minutes % 60) / 6)
    } else {
        text!("{} m", minutes)
    }
}

/// Temperature in 0.1 K as degrees Celsius with one decimal.
///
/// The C code mangled negative temperatures ("-1.15C" for -1.5 C, "0.-5C" for -0.5 C); this prints them properly.
pub fn format_temperature(deci_kelvin: u16) -> Text {
    let deci_c = deci_kelvin as i32 - 2731;
    let sign = if deci_c < 0 { "-" } else { "" };
    let abs = deci_c.unsigned_abs();
    text!("{}{}.{}C", sign, abs / 10, abs % 10)
}

/// A voltage in mV as volts with three decimals.
///
/// C printed `"%d.%d"` of `mV / 1000` and `mV % 1000`, dropping leading zeros of the fraction (3050 mV showed as
/// "3.50V"); this pads it.
pub fn format_millivolts(mv: u16) -> Text {
    text!("{}.{:03}V", mv / 1000, mv % 1000)
}

/// Rotation through the active gauge faults on the pack status screen (the `static` variables in C).
#[derive(Clone, Copy, Debug, Default)]
pub struct Screens {
    alert_blink: bool,
    next_fault_increment: Option<u64>,
    target_fault_idx: u32,
}

impl Screens {
    pub const fn new() -> Self {
        Screens { alert_blink: false, next_fault_increment: None, target_fault_idx: 0 }
    }

    /// Renders a [`Draw`] from the display state machine. For [`Draw::ClearAndOff`] this only clears; the caller
    /// flushes and then turns the display off.
    pub fn render(&mut self, fb: &mut Framebuffer, draw: Draw, s: &Snapshot) {
        match draw {
            Draw::Splash => splash(fb),
            Draw::Screen(screen) => self.screen(fb, screen, s),
            Draw::Menu { menu, selected, sweep } => self::menu(fb, menu, selected, sweep),
            Draw::NotImplemented => {
                fb.fill(Color::Black);
                fb.set_cursor(0, 0);
                fb.write_str("Not Yet Implemented", &FONT_7X10, Color::White);
            }
            Draw::ClearAndOff => fb.fill(Color::Black),
        }
    }

    fn screen(&mut self, fb: &mut Framebuffer, screen: Screen, s: &Snapshot) {
        match screen {
            Screen::Soc | Screen::Current | Screen::PackStatus => self.main_screen(fb, screen, s),
            Screen::CellVoltages => cell_voltages(fb, s),
            Screen::BqMissing => {
                message_with_icon(fb, Icon::Alert, "No BQ40", text!("Err: 0x{:08X}", s.not_connected_arg).as_str())
            }
            Screen::BqPermFail => {
                message_with_icon(fb, Icon::Alert, "Pack PF", text!("Err: 0x{:08X}", s.pf_status_arg).as_str())
            }
            Screen::BqInitializing => message_with_icon(fb, Icon::Info, "Booting", "BQ40 in Init"),
            Screen::BqShutdown => message_with_icon(fb, Icon::Info, "Shutdown", "In Progress..."),
            Screen::BqPowerCycle => message_with_icon(fb, Icon::Info, "MFC Req", "Cycling Power"),
            Screen::BqXdsg => message_with_icon(fb, Icon::Alert, "Dsg Dis.", "Tap for Menu"),
            Screen::LifeStats => {
                fb.fill(Color::Black);
                fb.set_cursor(0, 0);
                fb.write_str("Invalid Screen", &FONT_7X10, Color::White);
            }
        }
    }

    /// The SOC, current and pack status screens (`display_show_main_screen`).
    fn main_screen(&mut self, fb: &mut Framebuffer, screen: Screen, s: &Snapshot) {
        fb.fill(Color::Black);

        // Pack year and set: odd serials are set 1, even serials set 2
        let subseries = if s.mfg.serial.is_multiple_of(2) { 2 } else { 1 };
        fb.set_cursor(0, 0);
        fb.write_str(text!("ID: {}-{}", s.mfg.mfg_year, subseries).as_str(), &FONT_6X8, Color::White);

        // Serial number in a box in the corner
        fb.set_cursor(120, 2);
        fb.write_str(text!("{}", s.mfg.serial).as_str(), &FONT_6X8, Color::White);
        fb.draw_rectangle(118, 0, 127, 11, Color::White);

        let icon = if s.mfg_readout_okay { &bitmaps::CAN_ONLINE } else { &bitmaps::CAN_OFFLINE };
        fb.draw_bitmap(85, 0, icon, Color::White);

        // Blinking alert while the gauge reports safety or permanent failure faults
        let alert = s.fault(faults::BQ40_SAFETY_STATUS) || s.fault(faults::BQ40_PF_STATUS);
        if !self.alert_blink && alert {
            self.alert_blink = true;
            fb.draw_bitmap(70, 0, &bitmaps::ALERT_SMALL, Color::White);
        } else {
            self.alert_blink = false;
        }

        let info = &s.info;
        if matches!(screen, Screen::Soc | Screen::Current) {
            fb.fill_rectangle(0, 11, 79, 32, Color::White);
            fb.set_cursor(2, 13);
            let big = if screen == Screen::Soc {
                text!("SOC {}%", info.relative_soc)
            } else {
                text!("{:.2} A", info.current as f32 / 1000.0)
            };
            fb.write_str(big.as_str(), &FONT_11X18, Color::Black);
        }

        match screen {
            Screen::Soc => {
                let (remaining, charging) = info.time_remaining();
                if remaining != u16::MAX {
                    fb.set_cursor(86, 13);
                    // Estimated time to full / to empty
                    fb.write_str(if charging { "ETtF:" } else { "ETtE:" }, &FONT_6X8, Color::White);
                    fb.set_cursor(86, 22);
                    fb.write_str(format_remaining_time(remaining).as_str(), &FONT_7X10, Color::White);
                } else {
                    fb.set_cursor(86, 22);
                    fb.write_str(text!("{:.2}V", info.voltage() as f32 / 1000.0).as_str(), &FONT_7X10, Color::White);
                }
            }
            Screen::Current => {
                fb.set_cursor(86, 22);
                fb.write_str(text!("{:.2}V", info.voltage() as f32 / 1000.0).as_str(), &FONT_7X10, Color::White);
            }
            Screen::PackStatus => {
                self.pack_status(fb, s);
                fb.set_cursor(97, 15);
                fb.write_str("Temp:", &FONT_6X8, Color::White);
                let temp = format_temperature(info.temperature);
                let x = match temp.len() {
                    5 => 97,
                    4 => 103,
                    _ => 91,
                };
                fb.set_cursor(x, 23);
                fb.write_str(temp.as_str(), &FONT_6X8, Color::White);
            }
            _ => {}
        }
    }

    /// Bottom left of the pack status screen: the active gauge faults one at a time, or the pack state.
    fn pack_status(&mut self, fb: &mut Framebuffer, s: &Snapshot) {
        let pf = s.fault(faults::BQ40_PF_STATUS);
        if !(pf || s.fault(faults::BQ40_SAFETY_STATUS)) {
            fb.set_cursor(2, 13);
            fb.write_str("Pack State:", &FONT_6X8, Color::White);
            fb.set_cursor(2, 22);
            fb.write_str(s.state.name(), &FONT_6X8, Color::White);
            return;
        }

        let (mut reg, title, names) = if pf {
            (s.pf_status_arg, "Permanent Fail:", PF_NAMES)
        } else {
            (s.safety_status_arg, "Safety Fault:", PROTECTION_NAMES)
        };

        // Pick the target_fault_idx'th set bit, wrapping to the first
        let mut displayed = None;
        let mut cur = 0;
        let mut bit = 0u8;
        while reg != 0 {
            if reg & 1 != 0 {
                if cur == 0 {
                    displayed = Some(bit);
                }
                if cur == self.target_fault_idx {
                    displayed = Some(bit);
                    break;
                }
                cur += 1;
            }
            bit += 1;
            reg >>= 1;
        }
        if cur != self.target_fault_idx {
            self.target_fault_idx = 0;
        }
        let name = displayed.and_then(|b| crate::sbs::bit_name(names, b)).unwrap_or("Unknown?");

        fb.set_cursor(2, 13);
        fb.write_str(title, &FONT_6X8, Color::White);
        fb.set_cursor(2, 22);
        fb.write_str(name, &FONT_6X8, Color::White);

        // Move on to the next fault once a second
        match self.next_fault_increment {
            None => self.next_fault_increment = Some(s.now + 1000),
            Some(t) if s.now >= t => {
                self.target_fault_idx += 1;
                self.next_fault_increment = Some(s.now + 1000);
            }
            Some(_) => {}
        }
    }
}

/// The splash screen (`display_show_splash_screen`).
pub fn splash(fb: &mut Framebuffer) {
    fb.fill(Color::Black);
    for (y, line) in [(0, "Ohio State"), (11, "Underwater"), (22, "Robotics")] {
        fb.set_cursor(0, y);
        fb.write_str(line, &FONT_7X10, Color::White);
    }
    fb.draw_bitmap(79, 0, &bitmaps::UWRT_LOGO, Color::White);
}

/// A menu with the selected entry highlighted (`display_show_menu`).
pub fn menu(fb: &mut Framebuffer, menu: Menu, selected: usize, sweep: u16) {
    fb.fill(Color::Black);
    let top = (selected * 8) as u8;
    fb.fill_rectangle(0, top, WIDTH as u8 - 1, top + 7, Color::White);
    // Progress circle while the switch is held
    if sweep > 30 {
        fb.draw_arc_with_radius_line(122, 4 + top, 4, 0, sweep, Color::Black);
    }
    for (i, (label, _)) in display::menu_entries(menu).iter().enumerate() {
        fb.set_cursor(0, (8 * i) as u8);
        fb.write_str(label, &FONT_6X8, if i == selected { Color::Black } else { Color::White });
    }
}

/// The cell voltage screen (`display_show_cell_voltage_screen`).
pub fn cell_voltages(fb: &mut Framebuffer, s: &Snapshot) {
    fb.fill(Color::Black);
    for (i, mv) in s.info.cell_voltages().iter().enumerate() {
        fb.set_cursor(if i > 2 { 66 } else { 0 }, ((i % 3) * 10) as u8);
        fb.write_str(text!("C{}: {}", i + 1, format_millivolts(*mv).as_str()).as_str(), &FONT_6X8, Color::White);
    }
}

/// A message with an icon (`display_draw_message`).
pub fn message_with_icon(fb: &mut Framebuffer, icon: Icon, msg: &str, submsg: &str) {
    fb.fill(Color::Black);
    match icon {
        Icon::Info => fb.draw_bitmap(4, 4, &bitmaps::INFO_LARGE, Color::White),
        Icon::Alert => fb.draw_bitmap(3, 3, &bitmaps::ALERT_LARGE, Color::White),
        Icon::None => {}
    }
    if !msg.is_empty() {
        fb.set_cursor(38, 0);
        fb.write_str(msg, &FONT_11X18, Color::White);
    }
    if !submsg.is_empty() {
        fb.set_cursor(38, 23);
        fb.write_str(submsg, &FONT_6X8, Color::White);
    }
}

/// A status message such as "ROS Connect" (`display_show_msg`).
pub fn status_message(fb: &mut Framebuffer, msg: &str, submsg: &str) {
    fb.fill(Color::Black);
    if !msg.is_empty() {
        fb.set_cursor(0, 0);
        fb.write_str(msg, &FONT_11X18, Color::White);
    }
    if !submsg.is_empty() {
        fb.set_cursor(0, 24);
        fb.write_str(submsg, &FONT_6X8, Color::White);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn white(fb: &Framebuffer, x0: u8, y0: u8, x1: u8, y1: u8) -> u32 {
        let mut n = 0;
        for y in y0..=y1 {
            for x in x0..=x1 {
                if fb.pixel(x, y) == Color::White {
                    n += 1;
                }
            }
        }
        n
    }

    #[test]
    fn remaining_time_formats() {
        assert_eq!(format_remaining_time(65535), "N/A");
        assert_eq!(format_remaining_time(601), "10 h");
        assert_eq!(format_remaining_time(600), "10.0 h");
        assert_eq!(format_remaining_time(95), "1.5 h");
        assert_eq!(format_remaining_time(60), "60 m");
        assert_eq!(format_remaining_time(0), "0 m");
    }

    #[test]
    fn temperatures() {
        assert_eq!(format_temperature(2981), "25.0C");
        assert_eq!(format_temperature(2731), "0.0C");
        assert_eq!(format_temperature(2726), "-0.5C");
        assert_eq!(format_temperature(2716), "-1.5C");
        assert_eq!(format_temperature(3731), "100.0C");
    }

    #[test]
    fn millivolts_are_padded() {
        assert_eq!(format_millivolts(3050), "3.050V");
        assert_eq!(format_millivolts(4150), "4.150V");
        assert_eq!(format_millivolts(18_400), "18.400V");
    }

    #[test]
    fn soc_screen_layout() {
        let mut fb = Framebuffer::new();
        let mut screens = Screens::new();
        let mut s = Snapshot { mfg_readout_okay: true, ..Default::default() };
        s.mfg.serial = 3;
        s.mfg.mfg_year = 2023;
        s.info.relative_soc = 87;
        s.info.time_to_empty = 95;
        s.info.battery_status = 1 << crate::sbs::battery_status::DSG;
        screens.render(&mut fb, Draw::Screen(Screen::Soc), &s);
        // The SOC box is white with black text
        assert!(white(&fb, 0, 11, 79, 31) > 1000);
        assert!(white(&fb, 0, 11, 79, 31) < 80 * 21);
        // The serial box outline
        assert_eq!(fb.pixel(118, 0), Color::White);
        assert_eq!(fb.pixel(127, 11), Color::White);
        // Time remaining is drawn on the right
        assert!(white(&fb, 86, 13, 127, 31) > 0);
    }

    #[test]
    fn alert_icon_blinks() {
        let mut fb = Framebuffer::new();
        let mut screens = Screens::new();
        let s = Snapshot { faults: 1 << faults::BQ40_SAFETY_STATUS, ..Default::default() };
        let mut shown = [false; 4];
        for seen in shown.iter_mut() {
            screens.render(&mut fb, Draw::Screen(Screen::Current), &s);
            *seen = white(&fb, 70, 0, 79, 9) > 0;
        }
        assert_eq!(shown, [true, false, true, false]);
    }

    #[test]
    fn pack_status_rotates_through_faults() {
        let mut screens = Screens::new();
        let mut s = Snapshot {
            faults: 1 << faults::BQ40_SAFETY_STATUS,
            safety_status_arg: (1 << 0) | (1 << 13),
            ..Default::default()
        };
        let mut shown = std::vec::Vec::new();
        for i in 0..5 {
            s.now = i * 1000;
            let mut fb = Framebuffer::new();
            let mut expected = Framebuffer::new();
            screens.render(&mut fb, Draw::Screen(Screen::PackStatus), &s);
            for name in ["Cell Undervolt", "OverTemp Dsg"] {
                expected.fill(Color::Black);
                expected.set_cursor(2, 22);
                expected.write_str(name, &FONT_6X8, Color::White);
                let same = (22..30).all(|y| (2..90).all(|x| fb.pixel(x, y) == expected.pixel(x, y)));
                if same {
                    shown.push(name);
                }
            }
        }
        // The first draw arms the timer and the index advances after drawing, so the first fault shows for two
        // seconds. Like C, the wrap shows the first fault twice more before starting over
        assert_eq!(shown, ["Cell Undervolt", "Cell Undervolt", "OverTemp Dsg", "Cell Undervolt", "Cell Undervolt"]);
    }

    #[test]
    fn menu_highlights_selection() {
        let mut fb = Framebuffer::new();
        menu(&mut fb, Menu::Main, 1, 0);
        // Row 8..16 is mostly white, the others mostly black
        assert!(white(&fb, 0, 8, 127, 15) > 700);
        assert!(white(&fb, 0, 0, 127, 7) < 300);
        let mut with_circle = Framebuffer::new();
        menu(&mut with_circle, Menu::Main, 1, 180);
        assert!(white(&with_circle, 118, 8, 127, 15) < white(&fb, 118, 8, 127, 15));
    }

    #[test]
    fn static_screens_render() {
        let mut fb = Framebuffer::new();
        let mut screens = Screens::new();
        let s = Snapshot { not_connected_arg: 0x0000_1CFF, ..Default::default() };
        screens.render(&mut fb, Draw::Screen(Screen::BqMissing), &s);
        assert!(white(&fb, 3, 3, 28, 27) > 0, "alert icon");
        assert!(white(&fb, 38, 23, 127, 30) > 0, "error code");
        screens.render(&mut fb, Draw::ClearAndOff, &s);
        assert_eq!(white(&fb, 0, 0, 127, 31), 0);
        let mut fb = Framebuffer::new();
        splash(&mut fb);
        assert!(white(&fb, 79, 0, 127, 31) > 50);
    }
}
