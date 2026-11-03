//! TI CD74HC4051 8:1 analog multiplexer feeding an RP2040 ADC input, ported from `driver_cd74hc4051`.
//!
//! Three GPIOs (S0 = LSB, S1, S2 = MSB) select one of the eight mux inputs, and the mux output goes to an ADC pin.
//! Reading an input sets the select pins and takes one ADC sample, with no settling delay, as in C.
//!
//! # Differences from C
//!
//! * The data pin is typed as an ADC pin, so the C runtime check (and its "setup failed" log) is gone.
//! * The ADC is passed in to each read rather than owned, so other drivers can sample other ADC inputs too.
//! * ADC conversion errors are returned instead of being read as a value.
//! * An input number above 7 panics instead of silently selecting `num & 7`.

use embassy_rp::Peri;
use embassy_rp::adc::{self, Adc, AdcPin};
use embassy_rp::gpio::{Level, Output, Pin, Pull};

/// ADC reference voltage.
const ADC_VREF: f32 = 3.3;
/// Full scale of the 12-bit ADC.
const ADC_FULL_SCALE: f32 = (1 << 12) as f32;
/// Inputs above this voltage read as high in [`Cd74hc4051::read_digital`] (`DIGITAL_LOGIC_HIGH_V`).
pub const DIGITAL_HIGH_V: f32 = 2.0;
/// Number of mux inputs.
pub const INPUTS: u8 = 8;

/// A CD74HC4051 whose output is wired to an ADC pin.
pub struct Cd74hc4051<'d> {
    select: [Output<'d>; 3],
    input: adc::Channel<'d>,
}

impl<'d> Cd74hc4051<'d> {
    /// Sets up the mux (`multiplexer_init`). Input 0 is selected to start with.
    pub fn new(
        data: Peri<'d, impl AdcPin + 'd>,
        s0: Peri<'d, impl Pin>,
        s1: Peri<'d, impl Pin>,
        s2: Peri<'d, impl Pin>,
    ) -> Self {
        Cd74hc4051 {
            select: [Output::new(s0, Level::Low), Output::new(s1, Level::Low), Output::new(s2, Level::Low)],
            input: adc::Channel::new_pin(data, Pull::None),
        }
    }

    /// Routes input `num` (0-7) to the output.
    pub fn select(&mut self, num: u8) {
        assert!(num < INPUTS, "CD74HC4051 has 8 inputs");
        for (bit, pin) in self.select.iter_mut().enumerate() {
            pin.set_level(Level::from(num & (1 << bit) != 0));
        }
    }

    /// Selects input `num` and returns its voltage, 0 to 3.3 V (`multiplexer_decode_analog`).
    pub async fn read_analog(&mut self, adc: &mut Adc<'_, adc::Async>, num: u8) -> Result<f32, adc::Error> {
        self.select(num);
        let raw = adc.read(&mut self.input).await?;
        Ok(raw as f32 * (ADC_VREF / ADC_FULL_SCALE))
    }

    /// Selects input `num` and returns whether it's above [`DIGITAL_HIGH_V`] (`multiplexer_decode_digital`).
    pub async fn read_digital(&mut self, adc: &mut Adc<'_, adc::Async>, num: u8) -> Result<bool, adc::Error> {
        Ok(self.read_analog(adc, num).await? > DIGITAL_HIGH_V)
    }
}
