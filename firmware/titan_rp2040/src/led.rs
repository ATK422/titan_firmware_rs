//! RGB status LED, ported from `driver_led`.
//!
//! Colour priority:
//! 1. Blue: kill switch inserted (vehicle enabled)
//! 2. Green: ROS connected
//! 3. Yellow: network (CAN or Ethernet) online
//! 4. Red: network offline
//!
//! While any fault is raised the LED alternates between that colour and solid red every 500 ms.

use embassy_rp::Peri;
use embassy_rp::peripherals::{PIN_13, PIN_14, PIN_15, PWM_SLICE6, PWM_SLICE7};
use embassy_rp::pwm::{Config, Pwm, PwmOutput, SetDutyCycle};
use embassy_time::{Duration, Instant, Ticker};
use fixed::FixedU16;
use fixed::types::extra::U4;

use crate::{link, safety};

const TOGGLE_RATE_MS: u64 = 500;
const BRIGHTNESS_STEPS: u16 = 1024;
const FREQUENCY_HZ: u32 = 1000;
/// Yellow is red at full brightness plus a little green.
const YELLOW_GREEN_PERCENT: u8 = 22;
const UPDATE_PERIOD: Duration = Duration::from_millis(50);

#[derive(Clone, Copy, PartialEq, Eq)]
enum Level {
    Off,
    On,
    Percent(u8),
}

/// The status LED. Each colour is one PWM channel, driven active low.
pub struct StatusLed {
    red: PwmOutput<'static>,
    green: PwmOutput<'static>,
    blue: PwmOutput<'static>,
}

impl StatusLed {
    /// PWM configuration for an LED slice: one period per millisecond with [`BRIGHTNESS_STEPS`] counts, both
    /// channels inverted because the RGB LED is common anode.
    pub fn slice_config() -> Config {
        let mut config = Config::default();
        let divider = embassy_rp::clocks::clk_sys_freq() as f32 / (FREQUENCY_HZ * BRIGHTNESS_STEPS as u32) as f32;
        config.divider = FixedU16::<U4>::from_num(divider);
        config.top = BRIGHTNESS_STEPS - 1;
        config.invert_a = true;
        config.invert_b = true;
        config
    }

    /// Builds the LED from three PWM channels created with [`Self::slice_config`].
    pub fn new(red: PwmOutput<'static>, green: PwmOutput<'static>, blue: PwmOutput<'static>) -> Self {
        let mut led = StatusLed { red, green, blue };
        led.set(Level::On, Level::Off, Level::Off);
        led
    }

    /// The LED of the `rp2040_can_block`: GPIO 13 (red, PWM 6B), 14 (green, PWM 7A) and 15 (blue, PWM 7B).
    pub fn can_block(
        slice6: Peri<'static, PWM_SLICE6>,
        red: Peri<'static, PIN_13>,
        slice7: Peri<'static, PWM_SLICE7>,
        green: Peri<'static, PIN_14>,
        blue: Peri<'static, PIN_15>,
    ) -> Self {
        let (_, red) = Pwm::new_output_b(slice6, red, Self::slice_config()).split();
        let (green, blue) = Pwm::new_output_ab(slice7, green, blue, Self::slice_config()).split();
        Self::new(red.unwrap(), green.unwrap(), blue.unwrap())
    }

    fn set(&mut self, r: Level, g: Level, b: Level) {
        for (output, level) in [(&mut self.red, r), (&mut self.green, g), (&mut self.blue, b)] {
            // Duty cycles are always in range, so these cannot fail
            let _ = match level {
                Level::Off => output.set_duty_cycle_fully_off(),
                Level::On => output.set_duty_cycle_fully_on(),
                Level::Percent(p) => output.set_duty_cycle_percent(p),
            };
        }
    }

    fn update(&mut self) {
        let in_fault_phase = Instant::now().as_millis() % (TOGGLE_RATE_MS * 2) >= TOGGLE_RATE_MS;
        if safety::faults() != 0 && in_fault_phase {
            self.set(Level::On, Level::Off, Level::Off);
        } else if !safety::asserting_kill() {
            self.set(Level::Off, Level::Off, Level::On);
        } else if link::is_connected() {
            self.set(Level::Off, Level::On, Level::Off);
        } else if link::transport_online() {
            self.set(Level::On, Level::Percent(YELLOW_GREEN_PERCENT), Level::Off);
        } else {
            self.set(Level::On, Level::Off, Level::Off);
        }
    }
}

/// Keeps the LED in sync with the safety, link and network state.
#[embassy_executor::task]
pub async fn led_task(mut led: StatusLed) -> ! {
    let mut ticker = Ticker::every(UPDATE_PERIOD);
    loop {
        led.update();
        ticker.next().await;
    }
}
