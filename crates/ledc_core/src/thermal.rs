//! Temperature of the aluminium LED module, ported from `al_read_temp` / `read_temp` and the derating in
//! `update_led_status`.
//!
//! The thermistor (22 kΩ at 25 °C, B25/85 = 3730 K) sits below a 10 kΩ series resistor from 3.3 V and is read by
//! the RP2040's 12 bit ADC.

/// Series resistor above the thermistor, in ohms.
pub const SERIES_RESISTANCE: f64 = 10_000.0;
/// Divider supply and ADC reference.
pub const V_REF: f64 = 3.3;
/// Thermistor resistance at 25 °C.
pub const R_25: f64 = 22_000.0;
/// Thermistor B constant between 25 and 85 °C.
pub const B_25_85: f64 = 3730.0;
/// 25 °C in kelvin.
pub const NOMINAL_TEMP_K: f64 = 298.15;

/// Below this the LEDs run at full brightness.
pub const BASE_OPERATING_TEMPERATURE_C: f32 = 30.0;
/// Above this the LEDs are dimmed to nothing and `FAULT_LED_OVERTEMP` is raised.
pub const MAX_OPERATING_TEMPERATURE_C: f32 = 50.0;

/// Converts a 12 bit ADC reading to degrees Celsius.
pub fn thermistor_celsius(raw: u16) -> f32 {
    let voltage = raw as f32 * (3.3f32 / 4096.0);
    let resistance = SERIES_RESISTANCE * voltage as f64 / (V_REF - voltage as f64);
    // The C code subtracts 273.0, not 273.15; kept so readings and thresholds match
    (1.0 / (libm::log(resistance / R_25) / B_25_85 + 1.0 / NOMINAL_TEMP_K) - 273.0) as f32
}

/// Brightness factor for the module temperature: 1 up to [`BASE_OPERATING_TEMPERATURE_C`], falling linearly to 0
/// at [`MAX_OPERATING_TEMPERATURE_C`].
pub fn derating(temperature_c: f32) -> f32 {
    let span = MAX_OPERATING_TEMPERATURE_C - BASE_OPERATING_TEMPERATURE_C;
    let factor = (-1.0 / span) * temperature_c + MAX_OPERATING_TEMPERATURE_C / span;
    factor.clamp(0.0, 1.0)
}

/// True if the module is too hot to light (`monitor_temperature`).
pub fn is_overtemp(temperature_c: f32) -> bool {
    temperature_c > MAX_OPERATING_TEMPERATURE_C
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The ADC reading for a thermistor at `celsius`, from the B equation.
    fn raw_for(celsius: f64) -> u16 {
        let kelvin = celsius + 273.0;
        let r = R_25 * libm::exp(B_25_85 * (1.0 / kelvin - 1.0 / NOMINAL_TEMP_K));
        let v = V_REF * r / (r + SERIES_RESISTANCE);
        libm::round(v / 3.3 * 4096.0) as u16
    }

    #[test]
    fn thermistor_curve() {
        // At 25 °C the divider sits at 22/32 of 3.3 V
        let raw = (4096.0 * 22.0 / 32.0) as u16;
        assert!((thermistor_celsius(raw) - 25.15).abs() < 0.01, "{}", thermistor_celsius(raw));
        for celsius in [0.0, 20.0, 30.0, 40.0, 50.0, 60.0] {
            let got = thermistor_celsius(raw_for(celsius));
            assert!((got as f64 - celsius).abs() < 0.2, "{celsius}: {got}");
        }
        // Hotter means lower resistance, so lower readings
        assert!(thermistor_celsius(1000) > thermistor_celsius(3000));
    }

    #[test]
    fn derating_ramp() {
        let close = |t: f32, want: f32| assert!((derating(t) - want).abs() < 1e-6, "{t}: {}", derating(t));
        close(0.0, 1.0);
        close(30.0, 1.0);
        close(35.0, 0.75);
        close(40.0, 0.5);
        close(50.0, 0.0);
        assert_eq!(derating(80.0), 0.0);
        assert_eq!(derating(-10.0), 1.0);
    }

    #[test]
    fn overtemp_threshold() {
        assert!(!is_overtemp(50.0));
        assert!(is_overtemp(50.1));
    }
}
