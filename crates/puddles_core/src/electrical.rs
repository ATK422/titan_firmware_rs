//! Rail voltages from the MCP3426 on the board I2C bus (`calc_actual_voltage` and `adc_update` in `ros.c`).
//!
//! Each rail reaches the ADC through a resistor divider with a 10 kΩ bottom resistor: 23.2 kΩ on top for the
//! 3.3 V and 5 V rails, 100 kΩ for the 12 V and balanced battery rails.

use titan_link::puddles_backplane::ElectricalReadings;

/// The divider in front of an ADC channel.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Divider {
    /// 23.2 kΩ over 10 kΩ, for the 3.3 V and 5 V rails.
    Small,
    /// 100 kΩ over 10 kΩ, for the 12 V and balanced rails.
    Large,
}

const BOTTOM_OHMS: f32 = 10_000.0;

/// The rail voltage behind a divider, from the voltage the ADC measured.
pub fn rail_voltage(adc_volts: f32, divider: Divider) -> f32 {
    let top = match divider {
        Divider::Small => 23_200.0,
        Divider::Large => 100_000.0,
    };
    adc_volts / (BOTTOM_OHMS / (BOTTOM_OHMS + top))
}

/// `state/electrical` from the voltages on MCP3426 channels 1-4 (3.3 V, 5 V, 12 V, balanced). A channel that hasn't
/// been read publishes NaN, as `mcp3426_read_voltage` returned NaN in C.
pub fn readings(channels: [Option<f32>; 4]) -> ElectricalReadings {
    let rail = |channel: Option<f32>, divider| rail_voltage(channel.unwrap_or(f32::NAN), divider);
    ElectricalReadings {
        three_volt_voltage: rail(channels[0], Divider::Small),
        five_volt_voltage: rail(channels[1], Divider::Small),
        twelve_volt_voltage: rail(channels[2], Divider::Large),
        balanced_voltage: rail(channels[3], Divider::Large),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-4
    }

    #[test]
    fn divider_ratios() {
        // 3.32 x for the small divider, 11 x for the large one
        assert!(close(rail_voltage(1.0, Divider::Small), 3.32));
        assert!(close(rail_voltage(1.0, Divider::Large), 11.0));
        assert!(close(rail_voltage(0.0, Divider::Large), 0.0));
    }

    #[test]
    fn readings_map_channels_to_rails() {
        let r = readings([Some(1.0), Some(1.5), Some(1.1), None]);
        assert!(close(r.three_volt_voltage, 3.32));
        assert!(close(r.five_volt_voltage, 4.98));
        assert!(close(r.twelve_volt_voltage, 12.1));
        assert!(r.balanced_voltage.is_nan());
    }
}
