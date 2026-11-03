//! Modem hardware shared by the IVCBoard2 and IVCDAC firmware: the transducer drive and the hydrophone ADC.
//!
//! `ivc_dac` compiles this same file (`#[path]` module), since both boards drive identical hardware and the firmware
//! workspace has no shared crate for board code.

use embassy_rp::Peri;
use embassy_rp::adc::{self, Adc};
use embassy_rp::dma;
use embassy_rp::peripherals::{PIN_7, PWM_SLICE3};
use embassy_rp::pwm::{self, Pwm};
use embassy_time::Instant;
use fixed::FixedU16;
use fixed::types::extra::U4;

use ivc2_core::{NUM_SAMPLES, Symbol};

/// The transducer drive: a 50 % square wave on GPIO 7 (PWM 3B) whose frequency selects the symbol (`tx.c`).
pub struct Transmitter {
    pwm: Pwm<'static>,
    config: pwm::Config,
    wrap: u16,
}

impl Transmitter {
    pub fn new(slice: Peri<'static, PWM_SLICE3>, pin: Peri<'static, PIN_7>) -> Self {
        // PWM_WRAP_VALUE: keeps the clock divider below 256 down to 5 kHz (99 at 125 MHz)
        let sysclk = embassy_rp::clocks::clk_sys_freq() as f32;
        let wrap = (sysclk / 5000.0 / 255.0) as u16 + 1;
        let mut config = pwm::Config::default();
        config.top = wrap - 1;
        config.compare_b = wrap / 2;
        config.enable = false;
        Transmitter { pwm: Pwm::new_output_b(slice, pin, config.clone()), config, wrap }
    }

    /// Sets the output frequency, leaving the output on or off. Returns false (and changes nothing) if the divider
    /// cannot reach it.
    pub fn set_frequency(&mut self, hz: f32) -> bool {
        let divider = embassy_rp::clocks::clk_sys_freq() as f32 / (hz * self.wrap as f32);
        if !(1.0..256.0).contains(&divider) {
            return false;
        }
        // 8.4 fixed point, fraction truncated like pwm_set_clkdiv
        self.config.divider = FixedU16::<U4>::from_bits((divider * 16.0) as u16);
        self.pwm.set_config(&self.config);
        true
    }

    /// Plays the tone of `symbol` (`tx_encode_bit`, `tx_encode_sync`), or turns the output off for
    /// [`Symbol::None`].
    pub fn play(&mut self, symbol: Symbol) {
        match symbol.tone_hz() {
            Some(hz) => {
                self.set_frequency(hz as f32);
                self.enable(true);
            }
            None => self.enable(false),
        }
    }

    pub fn enable(&mut self, on: bool) {
        self.config.enable = on;
        self.pwm.set_config(&self.config);
    }
}

/// The hydrophone: [`NUM_SAMPLES`] 8-bit samples at 500 kS/s per capture, through DMA (`fft_setup`, `fft_sample`).
pub struct Hydrophone {
    adc: Adc<'static, adc::Async>,
    input: adc::Channel<'static>,
    dma: dma::Channel<'static>,
}

impl Hydrophone {
    pub fn new(adc: Adc<'static, adc::Async>, input: adc::Channel<'static>, dma: dma::Channel<'static>) -> Self {
        Hydrophone { adc, input, dma }
    }

    /// Fills `samples` with one capture (8.2 ms) and returns when its last sample was taken.
    ///
    /// The C firmware let DMA run continuously into two swap buffers and processed whichever had filled last, so
    /// DMA could overwrite a buffer that was still being processed. Capturing on demand never races, and since the C
    /// code only ever processed some of the captures (processing takes longer than a capture), its FIR history did
    /// not join up from one processed capture to the next either.
    pub async fn capture(&mut self, samples: &mut [u8; NUM_SAMPLES]) -> Result<Instant, adc::Error> {
        // A divider of 0 runs conversions back to back: 48 MHz / 96 cycles = 500 kS/s
        self.adc.read_many(&mut self.input, samples, 0, &mut self.dma).await?;
        Ok(Instant::now())
    }
}
