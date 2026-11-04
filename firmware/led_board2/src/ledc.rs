//! Controller bring-up and watchdog, ported from `ledc_start.c`, `ledc_driver.c`, `ledc_commands.c` and the
//! watchdog timer in `main.c`.
//!
//! Deliberate difference from C: `decode_cr1`/`decode_cr2` in `ledc_registers.c` masked each field with its
//! in-place mask *after* shifting it down, so the read-modify-write cycles zeroed almost every field they meant to
//! keep. In particular `set_peak_current` cleared EN again right after `set_controllers_active_mode` set it, and
//! the unlock frame cleared buck 1's duty. The Rust port keeps the other fields, so the controllers end up as the C
//! code intended: active (EN set, GOSTBY clear), PWM dimmed at the duties already in CR1, peak current 45.

use embassy_futures::select::{Either, select};
use embassy_rp::Peri;
use embassy_rp::adc::{self, Adc};
use embassy_rp::gpio::{Input, Output};
use embassy_rp::peripherals::{PIN_7, PWM_SLICE3, SPI1};
use embassy_rp::pwm::{self, Pwm};
use embassy_rp::spi::{self, Spi};
use embassy_time::{Delay, Ticker, Timer};
use fixed::FixedU16;

use ledc_core::driver::Ledc;
use ledc_core::l99ld20::{self, Buck, CR1, CR2, CR3, Chip, Cr1, Cr2, Cr3, DinMap};
use ledc_core::thermal;
use titan_rp2040::control;

use crate::commands;

/// Peak inductor current select for every buck (`BUCK_PEAK_CURRENT`).
const PEAK_CURRENT: u8 = 45;
/// How often the controller watchdogs are fed (`LEDC_WATCHDOG_TIME_MS`).
const WATCHDOG_PERIOD_MS: u64 = 50;

pub type Controllers = Ledc<Spi<'static, SPI1, spi::Blocking>, Output<'static>, Delay>;

/// The board hardware [`ledc_task`] owns.
pub struct Hardware {
    pub ledc: Controllers,
    /// DIN1 and DIN2, left as inputs.
    pub _din: [Input<'static>; 2],
    /// Kept so the PWMCLK keeps running.
    pub _pwm_clock: Pwm<'static>,
    pub adc: Adc<'static, adc::Blocking>,
    pub thermistor: adc::Channel<'static>,
}

/// Starts the controllers' 204.8 kHz PWMCLK on GPIO 7 (PWM 3B), `init_pwm`.
pub fn start_pwm_clock(slice: Peri<'static, PWM_SLICE3>, pin: Peri<'static, PIN_7>) -> Pwm<'static> {
    let mut config = pwm::Config::default();
    config.top = l99ld20::PWM_CLOCK_TOP;
    config.compare_b = l99ld20::PWM_CLOCK_LEVEL;
    config.divider = FixedU16::from_bits(l99ld20::pwm_clock_divider(embassy_rp::clocks::clk_sys_freq()));
    Pwm::new_output_b(slice, pin, config)
}

/// `set_controllers_active_mode`: an unlock frame to each controller, then EN high and GOSTBY low.
pub async fn set_active_mode(ledc: &mut Controllers) {
    for chip in Chip::ALL {
        defmt::unwrap!(ledc.modify(chip, CR1, |cr1| Cr1(cr1).with_unlock(true).0));
    }
    Timer::after_millis(1).await;
    for chip in Chip::ALL {
        defmt::unwrap!(ledc.modify(chip, CR2, |cr2| Cr2(cr2).with_enable(true).with_go_standby(false).0));
    }
}

/// `enable_dimming`: both bucks dimmed by the integrated PWM generator. Like C, the rest of CR3 is written as zero.
fn enable_dimming(ledc: &mut Controllers) {
    let cr3 = Cr3(0).with_din_map(Buck::Buck1, DinMap::PwmDimming).with_din_map(Buck::Buck2, DinMap::PwmDimming);
    for chip in Chip::ALL {
        defmt::unwrap!(ledc.write(chip, CR3, cr3.0));
    }
}

/// `set_peak_current`.
pub async fn set_peak_current(ledc: &mut Controllers) {
    for chip in Chip::ALL {
        defmt::unwrap!(ledc.modify(chip, CR2, |cr2| {
            Cr2(cr2).with_peak_current(Buck::Buck1, PEAK_CURRENT).with_peak_current(Buck::Buck2, PEAK_CURRENT).0
        }));
        Timer::after_millis(1).await;
    }
}

/// Logs at start-up what the C `leds` and `temp` debug commands print (see [`crate::commands`]).
fn log_status(hw: &mut Hardware) {
    for chip in Chip::ALL {
        let (global, status) = defmt::unwrap!(hw.ledc.status(chip));
        defmt::info!("{}: {}, global status {=u8:#04x}", chip, status, global.0);
        if global.fail_safe() {
            defmt::error!("{} in limp home", chip);
        }
    }
    match hw.adc.blocking_read(&mut hw.thermistor) {
        Ok(raw) => defmt::info!("Aluminium board temperature: {} °C", thermal::thermistor_celsius(raw)),
        Err(err) => defmt::warn!("Thermistor read failed: {}", err),
    }
}

/// Brings the controllers up (`ledc_init`), then feeds their watchdogs forever (`pet_watchdog`), running the
/// board's remote commands in between: the task owns the controller SPI and the thermistor ADC they use.
#[embassy_executor::task]
pub async fn ledc_task(mut hw: Hardware) -> ! {
    set_active_mode(&mut hw.ledc).await;
    enable_dimming(&mut hw.ledc);
    set_peak_current(&mut hw.ledc).await;
    log_status(&mut hw);

    let mut ticker = Ticker::every(embassy_time::Duration::from_millis(WATCHDOG_PERIOD_MS));
    loop {
        for chip in Chip::ALL {
            defmt::unwrap!(hw.ledc.toggle_watchdog(chip));
        }
        match select(ticker.next(), control::next_deferred()).await {
            Either::First(()) => {}
            Either::Second(request) => commands::run(&mut hw, request).await,
        }
    }
}
