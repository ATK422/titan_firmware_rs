//! Voltage measurement and the `command/electrical` topic.
//!
//! * The Talos power board measures the port and starboard battery voltages on the RP2040's internal ADC through
//!   10k/1k dividers (`analog_io.c`): [`battery_voltage`].
//! * All three boards measure their rails on an MCP3426 behind 10k/10k (3.3 V rail) or 10k/1k dividers (the rest):
//!   [`RailMap::convert`]. The PoHalf board wires the 5 V and 12 V rails to the opposite channels from Talos.
//! * The PoHalf board reads two analog pressure transducers on the internal ADC: [`pressure_sensor_reading`].
//! * `riptide_msgs2/ElectricalCommand` only switches the fan; the other commands were never implemented on these
//!   boards: [`ElectricalCommand::fan_switch`].

/// Reference voltage of the RP2040 ADC and of the conversions in the C firmware.
pub const ADC_VREF: f32 = 3.3;
/// Largest 12-bit ADC code, the divisor `analog_io.c` used.
pub const ADC_MAX_CODE: u16 = 4095;
/// Number of codes of the 12-bit ADC, the divisor `pressure.c` used.
pub const ADC_CODES: u32 = 1 << 12;

/// A resistor divider in front of an ADC input.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Divider {
    pub top_ohms: f32,
    pub bottom_ohms: f32,
}

impl Divider {
    pub const fn new(top_ohms: f32, bottom_ohms: f32) -> Self {
        Divider { top_ohms, bottom_ohms }
    }

    /// The voltage at the divider's input given the voltage at its tap, computed like `calc_actual_voltage`.
    pub fn input_voltage(&self, tap_voltage: f32) -> f32 {
        tap_voltage * (1.0 / (self.bottom_ohms / (self.bottom_ohms + self.top_ohms)))
    }
}

/// Port and starboard battery measurement divider (Talos power board).
pub const BATTERY_DIVIDER: Divider = Divider::new(10_000.0, 1_000.0);
/// Divider in front of the MCP3426 on the 3.3 V rail.
pub const RAIL_3V3_DIVIDER: Divider = Divider::new(10_000.0, 10_000.0);
/// Divider in front of the MCP3426 on the balanced, 12 V and 5 V rails.
pub const RAIL_DIVIDER: Divider = Divider::new(10_000.0, 1_000.0);

/// Battery voltage from a raw internal ADC code, as `analog_io.c` computed it.
pub fn battery_voltage(raw: u16) -> f32 {
    let tap = raw as f32 * (ADC_VREF / ADC_MAX_CODE as f32);
    BATTERY_DIVIDER.input_voltage(tap)
}

/// Scale applied to the pressure transducer voltage (`PRESSURE_SCALE_SLOPE`). The C firmware never calibrated it,
/// so the topics carry volts.
pub const PRESSURE_SCALE_SLOPE: f32 = 1.0;
/// Offset applied to the pressure transducer voltage (`PRESSURE_SCALE_OFFSET`).
pub const PRESSURE_SCALE_OFFSET: f32 = 0.0;

/// Pressure transducer reading from a raw internal ADC code, as `pressure_read_adc` computed it.
pub fn pressure_sensor_reading(raw: u16) -> f32 {
    let volts = raw as f32 / ADC_CODES as f32 * ADC_VREF;
    volts * PRESSURE_SCALE_SLOPE + PRESSURE_SCALE_OFFSET
}

/// Rail voltages as published in `riptide_msgs2/ElectricalReadings`.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct RailVoltages {
    pub three_volt: f32,
    pub balanced: f32,
    pub twelve_volt: f32,
    pub five_volt: f32,
}

/// Which MCP3426 channel selection (0-3, as cycled by the driver) measures each rail.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct RailMap {
    pub three_volt: usize,
    pub balanced: usize,
    pub twelve_volt: usize,
    pub five_volt: usize,
}

/// Talos power and Poac boards (`ros_publish_electrical_readings` in their `ros.c`).
pub const TALOS_RAILS: RailMap = RailMap { three_volt: 0, balanced: 1, twelve_volt: 2, five_volt: 3 };
/// Lilboat PoHalf board: 12 V on channel 4 and 5 V on channel 3.
pub const PO_HALF_RAILS: RailMap = RailMap { three_volt: 0, balanced: 1, twelve_volt: 3, five_volt: 2 };

/// Converts an ADC voltage behind `divider`, giving NaN for a channel that hasn't been read (the C driver's value
/// for an invalid reading, which the topic has always carried).
pub fn rail_voltage(adc_voltage: Option<f32>, divider: Divider) -> f32 {
    adc_voltage.map_or(f32::NAN, |v| divider.input_voltage(v))
}

impl RailMap {
    /// Rail voltages from the latest voltage on each of the four MCP3426 channel selections.
    pub fn convert(&self, adc: [Option<f32>; 4]) -> RailVoltages {
        RailVoltages {
            three_volt: rail_voltage(adc[self.three_volt], RAIL_3V3_DIVIDER),
            balanced: rail_voltage(adc[self.balanced], RAIL_DIVIDER),
            twelve_volt: rail_voltage(adc[self.twelve_volt], RAIL_DIVIDER),
            five_volt: rail_voltage(adc[self.five_volt], RAIL_DIVIDER),
        }
    }
}

/// `riptide_msgs2/ElectricalCommand::command`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ElectricalCommand {
    CycleComputer = 0,
    CycleAccoustics = 1,
    KillAccoustics = 2,
    CycleRobot = 3,
    KillRobotPower = 4,
    EnableFans = 5,
    DisableFans = 6,
    ClearDepth = 7,
    DisableLeds = 8,
    EnableLeds = 9,
    CycleImu = 10,
}

impl ElectricalCommand {
    pub fn from_u8(command: u8) -> Option<Self> {
        use ElectricalCommand::*;
        Some(match command {
            0 => CycleComputer,
            1 => CycleAccoustics,
            2 => KillAccoustics,
            3 => CycleRobot,
            4 => KillRobotPower,
            5 => EnableFans,
            6 => DisableFans,
            7 => ClearDepth,
            8 => DisableLeds,
            9 => EnableLeds,
            10 => CycleImu,
            _ => return None,
        })
    }

    /// The fan switch level this command asks for, if it is a fan command. The power boards ignore every other
    /// command, as the C `elec_command_subscription_callback` did.
    pub fn fan_switch(self) -> Option<bool> {
        match self {
            ElectricalCommand::EnableFans => Some(true),
            ElectricalCommand::DisableFans => Some(false),
            _ => None,
        }
    }
}

/// The fan is switched on at power up and stays on until a `DISABLE_FANS` command.
pub const FAN_ON_AT_BOOT: bool = true;

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-4 * b.abs().max(1.0)
    }

    #[test]
    fn dividers_match_calc_actual_voltage() {
        assert!(close(RAIL_3V3_DIVIDER.input_voltage(1.65), 3.3));
        assert!(close(RAIL_DIVIDER.input_voltage(1.0), 11.0));
        assert!(close(BATTERY_DIVIDER.input_voltage(1.5), 16.5));
    }

    #[test]
    fn battery_voltage_from_raw() {
        assert_eq!(battery_voltage(0), 0.0);
        // Full scale is 3.3 V at the pin, 36.3 V at the battery
        assert!(close(battery_voltage(4095), 36.3));
        // A 16.8 V pack shows up as about 1.527 V at the pin
        let raw = (16.8 / 11.0 / 3.3 * 4095.0) as u16;
        assert!((battery_voltage(raw) - 16.8).abs() < 0.01);
    }

    #[test]
    fn pressure_reading_divides_by_4096() {
        assert_eq!(pressure_sensor_reading(0), 0.0);
        assert!(close(pressure_sensor_reading(2048), 1.65));
        // Unlike analog_io.c, pressure.c never reaches the full 3.3 V
        assert!(pressure_sensor_reading(4095) < 3.3);
    }

    #[test]
    fn rail_maps() {
        let adc = [Some(1.65), Some(1.5), Some(1.1), Some(0.45)];
        let talos = TALOS_RAILS.convert(adc);
        assert!(close(talos.three_volt, 3.3));
        assert!(close(talos.balanced, 16.5));
        assert!(close(talos.twelve_volt, 12.1));
        assert!(close(talos.five_volt, 4.95));

        let po_half = PO_HALF_RAILS.convert(adc);
        assert!(close(po_half.three_volt, 3.3));
        assert!(close(po_half.twelve_volt, 4.95));
        assert!(close(po_half.five_volt, 12.1));
    }

    #[test]
    fn missing_channels_are_nan() {
        let rails = TALOS_RAILS.convert([Some(1.0), None, None, None]);
        assert!(close(rails.three_volt, 2.0));
        assert!(rails.balanced.is_nan() && rails.twelve_volt.is_nan() && rails.five_volt.is_nan());
    }

    #[test]
    fn only_fan_commands_switch_the_fan() {
        assert_eq!(ElectricalCommand::from_u8(5).and_then(ElectricalCommand::fan_switch), Some(true));
        assert_eq!(ElectricalCommand::from_u8(6).and_then(ElectricalCommand::fan_switch), Some(false));
        for cmd in (0..=10).filter(|c| *c != 5 && *c != 6) {
            let parsed = ElectricalCommand::from_u8(cmd).expect("valid command");
            assert_eq!(parsed as u8, cmd);
            assert_eq!(parsed.fan_switch(), None);
        }
        assert_eq!(ElectricalCommand::from_u8(11), None);
    }
}
