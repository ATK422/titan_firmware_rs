//! ST L99LD20 dual buck LED controller: SPI frames and register fields.
//!
//! Every SPI exchange is one 32 bit frame, MSB first. The host sends
//!
//! ```text
//! 31..30 opcode   29..24 address   23..1 data   0 parity
//! ```
//!
//! and the controller answers in the same frame with the global status byte followed by the 24 bit register
//! contents. Bit 0 of every frame is a parity bit that makes the number of ones in the whole frame odd.
//!
//! The C `LEDBoard` computed the parity over the data bits only, choosing even or odd per register so that the
//! whole frame came out odd for its writes, and sent reads without fixing the parity. `LEDBoard2` fixed it up over
//! the whole frame but could only set the bit, never clear it. [`encode_frame`] always recomputes it over the whole
//! frame, which produces the same bytes as both C versions for every write they made.

/// Highest register address.
pub const MAX_ADDRESS: u8 = 0x3F;
/// Bits of the register contents.
pub const DATA_MASK: u32 = 0x00FF_FFFF;
/// The parity bit, bit 0 of every frame.
pub const PARITY: u32 = 1;

/// Control register 1: PWM duties, high LED current configuration, unlock.
pub const CR1: u8 = 0x01;
/// Control register 2: peak inductor currents, VLED×TOFF, standby and enable.
pub const CR2: u8 = 0x02;
/// Control register 3: phases, DIN mapping, PWM sync, input voltage windows.
pub const CR3: u8 = 0x03;
/// Control register 4: holds the watchdog trigger bit.
pub const CR4: u8 = 0x04;
/// Status register 1 (read and clear).
pub const SR1: u8 = 0x05;
/// Status register 2 (read and clear), including the watchdog timer state.
pub const SR2: u8 = 0x06;
/// Status register 3: VLED measurements.
pub const SR3: u8 = 0x07;

/// Operation encoded in the top two bits of a frame.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Opcode {
    Write = 0b00,
    Read = 0b01,
    ReadClear = 0b10,
    /// Reads the device information ROM.
    ReadRom = 0b11,
}

/// One of the two controllers on the board.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Chip {
    Ledc1 = 0,
    Ledc2 = 1,
}

impl Chip {
    pub const ALL: [Chip; 2] = [Chip::Ledc1, Chip::Ledc2];
}

/// One of the two buck converters of a controller.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Buck {
    Buck1 = 0,
    Buck2 = 1,
}

impl Buck {
    pub const ALL: [Buck; 2] = [Buck::Buck1, Buck::Buck2];

    const fn pick(self, buck1: u32, buck2: u32) -> u32 {
        match self {
            Buck::Buck1 => buck1,
            Buck::Buck2 => buck2,
        }
    }
}

/// Frequency of the PWMCLK the boards feed the controllers' integrated PWM generators.
pub const PWM_CLOCK_HZ: u32 = 204_800;
/// The PWMCLK is a 4 cycle (wrap 3) square wave...
pub const PWM_CLOCK_TOP: u16 = 3;
/// ...high for two cycles.
pub const PWM_CLOCK_LEVEL: u16 = 2;

/// RP2040 PWM clock divider for [`PWM_CLOCK_HZ`], as 8.4 fixed point bits, with the fraction truncated like
/// `pwm_set_clkdiv`.
pub fn pwm_clock_divider(clk_sys_hz: u32) -> u16 {
    let divider = clk_sys_hz as f32 / (PWM_CLOCK_HZ * (PWM_CLOCK_TOP as u32 + 1)) as f32;
    let int = divider as u32;
    let frac = ((divider - int as f32) * 16.0) as u32;
    (int.min(255) << 4 | frac) as u16
}

/// Builds a frame. Data bit 0 is replaced by the parity bit.
pub fn encode_frame(opcode: Opcode, address: u8, data: u32) -> [u8; 4] {
    let word = ((opcode as u32) << 30) | (((address & MAX_ADDRESS) as u32) << 24) | (data & DATA_MASK & !PARITY);
    let word = if word.count_ones().is_multiple_of(2) { word | PARITY } else { word };
    word.to_be_bytes()
}

/// True if the frame has the odd parity the controller expects.
pub fn has_odd_parity(frame: [u8; 4]) -> bool {
    u32::from_be_bytes(frame).count_ones() % 2 == 1
}

/// The controller's answer to a frame.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Response {
    pub global_status: GlobalStatus,
    /// Register contents, including the controller's parity bit in bit 0.
    pub data: u32,
}

impl Response {
    pub fn from_bytes(frame: [u8; 4]) -> Self {
        Response { global_status: GlobalStatus(frame[0]), data: u32::from_be_bytes(frame) & DATA_MASK }
    }
}

/// The global status byte that starts every answer.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GlobalStatus(pub u8);

impl GlobalStatus {
    /// Flag names by bit, as printed by the C `@ledc` debug command.
    pub const FLAG_NAMES: [&'static str; 8] = [
        "Fail Safe",
        "Global Warning",
        "Device Error",
        "Functional Err 1",
        "Functional Err 2",
        "SPI Err",
        "Reset Bit",
        "Global Status Bit Not",
    ];

    fn bit(self, n: u8) -> bool {
        self.0 & (1 << n) != 0
    }

    /// The controller is in fail safe (limp home) mode.
    pub fn fail_safe(self) -> bool {
        self.bit(0)
    }

    pub fn global_warning(self) -> bool {
        self.bit(1)
    }

    pub fn device_error(self) -> bool {
        self.bit(2)
    }

    /// The previous frame was rejected (bad parity, length or address).
    pub fn spi_error(self) -> bool {
        self.bit(5)
    }

    /// The controller was reset since the status was last cleared.
    pub fn reset(self) -> bool {
        self.bit(6)
    }

    /// Names of the set flags.
    pub fn flags(self) -> impl Iterator<Item = &'static str> {
        (0..8).filter(move |&n| self.bit(n)).map(|n| Self::FLAG_NAMES[n as usize])
    }
}

const fn field(reg: u32, shift: u32, width: u32) -> u32 {
    (reg >> shift) & ((1 << width) - 1)
}

const fn with_field(reg: u32, shift: u32, width: u32, value: u32) -> u32 {
    let mask = ((1 << width) - 1) << shift;
    (reg & !mask) | ((value << shift) & mask)
}

const fn with_bit(reg: u32, bit: u32, on: bool) -> u32 {
    with_field(reg, bit, 1, on as u32)
}

/// Control register 1.
///
/// ```text
/// 23..14 DUTY1   13..4 DUTY2   3 HLEDCUR1   2 HLEDCUR2   1 UNLOCK   0 parity
/// ```
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Cr1(pub u32);

impl Cr1 {
    /// Largest PWM duty (10 bits).
    pub const MAX_DUTY: u16 = 0x3FF;

    pub const fn duty(self, buck: Buck) -> u16 {
        field(self.0, buck.pick(14, 4), 10) as u16
    }

    pub const fn with_duty(self, buck: Buck, duty: u16) -> Self {
        Cr1(with_field(self.0, buck.pick(14, 4), 10, duty as u32))
    }

    pub const fn high_led_current(self, buck: Buck) -> bool {
        field(self.0, buck.pick(3, 2), 1) != 0
    }

    pub const fn with_high_led_current(self, buck: Buck, on: bool) -> Self {
        Cr1(with_bit(self.0, buck.pick(3, 2), on))
    }

    /// Unlocks EN/GOSTBY in the next frame (clears itself).
    pub const fn unlock(self) -> bool {
        field(self.0, 1, 1) != 0
    }

    pub const fn with_unlock(self, on: bool) -> Self {
        Cr1(with_bit(self.0, 1, on))
    }
}

/// Control register 2.
///
/// ```text
/// 23..18 IL1_PEAK   17..12 IL2_PEAK   11..8 VLED_TOFF1   7..4 VLED_TOFF2   3 GOSTBY   2 EN   1 reserved   0 parity
/// ```
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Cr2(pub u32);

impl Cr2 {
    /// Peak inductor current select (6 bits).
    pub const fn peak_current(self, buck: Buck) -> u8 {
        field(self.0, buck.pick(18, 12), 6) as u8
    }

    pub const fn with_peak_current(self, buck: Buck, current: u8) -> Self {
        Cr2(with_field(self.0, buck.pick(18, 12), 6, current as u32))
    }

    /// Constant VLED×TOFF select (4 bits).
    pub const fn vled_toff(self, buck: Buck) -> u8 {
        field(self.0, buck.pick(8, 4), 4) as u8
    }

    /// Standby request (effective while EN is low).
    pub const fn go_standby(self) -> bool {
        field(self.0, 3, 1) != 0
    }

    pub const fn with_go_standby(self, on: bool) -> Self {
        Cr2(with_bit(self.0, 3, on))
    }

    /// Active mode enable. Only accepted in the frame after [`Cr1::with_unlock`].
    pub const fn enable(self) -> bool {
        field(self.0, 2, 1) != 0
    }

    pub const fn with_enable(self, on: bool) -> Self {
        Cr2(with_bit(self.0, 2, on))
    }
}

/// What controls a buck (CR3 DIN_MAP).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DinMap {
    AlwaysOff = 0,
    /// Dimmed by the integrated PWM generator at the CR1 duty.
    PwmDimming = 1,
    AlwaysOn = 2,
    /// Follows the DIN pin.
    FollowDin = 3,
}

impl DinMap {
    const fn from_bits(bits: u32) -> Self {
        match bits & 3 {
            0 => DinMap::AlwaysOff,
            1 => DinMap::PwmDimming,
            2 => DinMap::AlwaysOn,
            _ => DinMap::FollowDin,
        }
    }
}

/// Control register 3.
///
/// ```text
/// 23..20 PH1   19..16 PH2   15..14 DIN_MAP1   13..12 DIN_MAP2   11..7 reserved   6 PWM_SYNC   5..4 B_IN_W1
/// 3..2 B_IN_W2   1 reserved   0 parity
/// ```
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Cr3(pub u32);

impl Cr3 {
    /// Phase select, in sixteenths of a PWM period.
    pub const fn phase(self, buck: Buck) -> u8 {
        field(self.0, buck.pick(20, 16), 4) as u8
    }

    pub const fn din_map(self, buck: Buck) -> DinMap {
        DinMap::from_bits(field(self.0, buck.pick(14, 12), 2))
    }

    pub const fn with_din_map(self, buck: Buck, map: DinMap) -> Self {
        Cr3(with_field(self.0, buck.pick(14, 12), 2, map as u32))
    }

    pub const fn input_window(self, buck: Buck) -> u8 {
        field(self.0, buck.pick(4, 2), 2) as u8
    }
}

/// CR4 watchdog trigger bit. The controller drops to fail safe unless it toggles within the watchdog period.
pub const CR4_WATCHDOG_TRIGGER: u32 = 1 << 23;

/// Status register 2 (selected fields).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Sr2(pub u32);

impl Sr2 {
    /// Watchdog timer state in quarters of its period (bits 16..15): how close the controller is to timing out.
    pub const fn watchdog_quarters(self) -> u8 {
        field(self.0, 15, 2) as u8
    }

    /// VS undervoltage.
    pub const fn supply_undervoltage(self) -> bool {
        field(self.0, 13, 1) != 0
    }
}

/// Decoded controller state, as reported by the C `leds` debug command.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ControllerStatus {
    pub unlocked: bool,
    pub standby: bool,
    pub enabled: bool,
    /// See [`Sr2::watchdog_quarters`].
    pub watchdog_quarters: u8,
}

impl ControllerStatus {
    pub fn new(cr1: Cr1, cr2: Cr2, sr2: Sr2) -> Self {
        ControllerStatus {
            unlocked: cr1.unlock(),
            standby: cr2.go_standby(),
            enabled: cr2.enable(),
            watchdog_quarters: sr2.watchdog_quarters(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The C `LEDBoard` frame layout and its data-only parity (`correct_parity_bit`).
    fn c_ledboard_write(address: u8, data: u32, odd_parity: bool) -> [u8; 4] {
        let ones = (data >> 1 & 0x7F_FFFF).count_ones();
        let data = if ones.is_multiple_of(2) == odd_parity { data | 1 } else { data & !1 };
        ((address as u32 & 0x3F) << 24 | data).to_be_bytes()
    }

    #[test]
    fn frames_have_odd_parity() {
        for opcode in [Opcode::Write, Opcode::Read, Opcode::ReadClear, Opcode::ReadRom] {
            for address in 0..=MAX_ADDRESS {
                for data in [0, 1, 0x80_0001, 0xFF_FFFF, 0x12_3456, 0x5000] {
                    let frame = encode_frame(opcode, address, data);
                    assert!(has_odd_parity(frame), "{opcode:?} {address} {data:#x}");
                    assert_eq!(frame[0] >> 6, opcode as u8);
                    assert_eq!(frame[0] & 0x3F, address);
                    assert_eq!(u32::from_be_bytes(frame) & DATA_MASK & !1, data & !1 & DATA_MASK);
                }
            }
        }
    }

    #[test]
    fn writes_match_c_ledboard() {
        // CR1 and CR2 writes used even data parity, CR3 odd: both make the whole frame odd
        for data in [0u32, 0x3FF << 14, 0x2D << 18 | 0x2D << 12, 0x12_3456, 0xFF_FFFE] {
            assert_eq!(encode_frame(Opcode::Write, CR1, data), c_ledboard_write(CR1, data, false));
            assert_eq!(encode_frame(Opcode::Write, CR2, data), c_ledboard_write(CR2, data, false));
            assert_eq!(encode_frame(Opcode::Write, CR3, data), c_ledboard_write(CR3, data, true));
        }
        // The watchdog writes were raw values with the parity worked out by hand
        assert_eq!(encode_frame(Opcode::Write, CR4, 0x80_0000), [0x04, 0x80, 0x00, 0x01]);
        assert_eq!(encode_frame(Opcode::Write, CR4, 0), [0x04, 0x00, 0x00, 0x00]);
    }

    #[test]
    fn pwm_clock_divider_matches_pico_sdk() {
        // 125 MHz / 819.2 kHz = 152.5879: integer 152, fraction 9/16
        assert_eq!(pwm_clock_divider(125_000_000), 152 << 4 | 9);
        assert_eq!(pwm_clock_divider(819_200), 1 << 4);
    }

    #[test]
    fn response_splits_status_and_data() {
        let r = Response::from_bytes([0x41, 0x12, 0x34, 0x57]);
        assert_eq!(r.global_status, GlobalStatus(0x41));
        assert_eq!(r.data, 0x12_3457);
        assert!(r.global_status.fail_safe());
        assert!(r.global_status.reset());
        assert!(!r.global_status.spi_error());
        let mut flags = r.global_status.flags();
        assert_eq!(flags.next(), Some("Fail Safe"));
        assert_eq!(flags.next(), Some("Reset Bit"));
        assert_eq!(flags.next(), None);
    }

    #[test]
    fn cr1_fields() {
        let cr1 = Cr1(0).with_duty(Buck::Buck1, 0x3FF);
        assert_eq!(cr1.0, 0xFF_C000);
        let cr1 = cr1.with_duty(Buck::Buck2, 0x155);
        assert_eq!(cr1.0, 0xFF_C000 | 0x155 << 4);
        assert_eq!(cr1.duty(Buck::Buck1), 0x3FF);
        assert_eq!(cr1.duty(Buck::Buck2), 0x155);
        // Out of range duties cannot spill into the neighbouring field
        assert_eq!(Cr1(0).with_duty(Buck::Buck2, 0xFFFF).0, 0x3FF0);
        // Updating one duty keeps everything else (the C masks 0x3FFF and 0xFFC00F)
        let full = Cr1(0xFF_FFFF);
        assert_eq!(full.with_duty(Buck::Buck1, 0).0, 0x3FFF);
        assert_eq!(full.with_duty(Buck::Buck2, 0).0, 0xFF_C00F);
        let flags = Cr1(0).with_high_led_current(Buck::Buck1, true).with_high_led_current(Buck::Buck2, true);
        assert_eq!(flags.with_unlock(true).0, 0b1110);
        assert!(flags.high_led_current(Buck::Buck1) && flags.high_led_current(Buck::Buck2) && !flags.unlock());
    }

    #[test]
    fn cr2_fields() {
        let cr2 = Cr2(0).with_peak_current(Buck::Buck1, 45).with_peak_current(Buck::Buck2, 45);
        assert_eq!(cr2.0, 45 << 18 | 45 << 12);
        assert_eq!(cr2.peak_current(Buck::Buck1), 45);
        assert_eq!(cr2.peak_current(Buck::Buck2), 45);
        // Read-modify-write keeps the other fields, unlike the C LEDBoard2 decode_cr2/encode_cr2 pair
        let cr2 = Cr2(0x00_0FF4).with_peak_current(Buck::Buck1, 45);
        assert!(cr2.enable());
        assert_eq!(cr2.vled_toff(Buck::Buck1), 0xF);
        assert_eq!(cr2.vled_toff(Buck::Buck2), 0xF);
        let active = Cr2(0b1000).with_enable(true).with_go_standby(false);
        assert_eq!(active.0, 0b0100);
    }

    #[test]
    fn cr3_fields() {
        let cr3 = Cr3(0).with_din_map(Buck::Buck1, DinMap::PwmDimming).with_din_map(Buck::Buck2, DinMap::PwmDimming);
        // The value C LEDBoard2's enable_dimming writes
        assert_eq!(cr3.0, 0x5000);
        assert_eq!(cr3.din_map(Buck::Buck1), DinMap::PwmDimming);
        let cr3 = Cr3(0xFF_FFFF).with_din_map(Buck::Buck1, DinMap::AlwaysOff);
        assert_eq!(cr3.0, 0xFF_3FFF);
        assert_eq!(cr3.din_map(Buck::Buck2), DinMap::FollowDin);
        assert_eq!(Cr3(0xA5_0034).phase(Buck::Buck1), 0xA);
        assert_eq!(Cr3(0xA5_0034).phase(Buck::Buck2), 0x5);
        assert_eq!(Cr3(0xA5_0034).input_window(Buck::Buck1), 3);
        assert_eq!(Cr3(0xA5_0034).input_window(Buck::Buck2), 1);
    }

    #[test]
    fn controller_status() {
        let status = ControllerStatus::new(Cr1(0b10), Cr2(0b0100), Sr2(0b11 << 15));
        assert_eq!(status, ControllerStatus { unlocked: true, standby: false, enabled: true, watchdog_quarters: 3 });
        assert_eq!(Sr2(1 << 16).watchdog_quarters(), 2);
        assert!(Sr2(1 << 13).supply_undervoltage());
    }
}
