//! The aluminium LED module, ported from `ledc_driver.c` and `ledc_commands.c`.
//!
//! The C firmware ran five repeating timers in interrupt context, each talking to the controllers over SPI. Here
//! [`led_task`] owns the bus and runs the same schedule: the controller watchdog every 3 ms, the singleton flash
//! check every 10 ms, the pattern refresh every 50 ms, and the depth and temperature checks every second. The
//! light logic itself is [`ledc_core::board::LedBoard`].

use embassy_futures::select::{Either, select};
use embassy_rp::Peri;
use embassy_rp::adc::{self, Adc};
use embassy_rp::gpio::Output;
use embassy_rp::peripherals::{PIN_7, PWM_SLICE3, SPI1};
use embassy_rp::pwm::{self, Pwm};
use embassy_rp::spi::{self, Spi};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Delay, Duration, Instant, Timer};
use fixed::FixedU16;

use ledc_core::board::{self, Command, LedBoard, Output as Light};
use ledc_core::driver::Ledc;
use ledc_core::l99ld20::{self, Buck, CR1, CR2, CR4, CR4_WATCHDOG_TRIGGER, Chip, Cr1, Cr2, DinMap, SR2};
use ledc_core::pattern::REFRESH_PERIOD_MS;
use ledc_core::{Rgb, thermal};
use titan_link::{HostToDevice, led};
use titan_rp2040::safety;

/// Raised while the module is above 50 °C, with the temperature in °C as its argument.
pub const FAULT_LED_OVERTEMP: u8 = safety::FIRST_BOARD_FAULT + 1;

pub type Controllers = Ledc<Spi<'static, SPI1, spi::Blocking>, Output<'static>, Delay>;

/// The board hardware [`led_task`] owns.
pub struct Hardware {
    pub ledc: Controllers,
    /// DIN1 and DIN2, held low.
    pub _din: [Output<'static>; 2],
    /// Kept so the PWMCLK keeps running.
    pub _pwm_clock: Pwm<'static>,
    pub adc: Adc<'static, adc::Blocking>,
    pub thermistor: adc::Channel<'static>,
}

enum Event {
    Command(Command),
    Depth(f32),
}

static EVENTS: Channel<CriticalSectionRawMutex, Event, 8> = Channel::new();

/// Link hook for LED board messages: the C executor callbacks.
pub fn handle_board_message(msg: HostToDevice) {
    let event = match msg {
        HostToDevice::Led(led::HostToDevice::Command(c)) => {
            match Command::from_led_command(Rgb::new(c.red, c.green, c.blue), c.mode, c.target) {
                Some(command) => Event::Command(command),
                // Not for the aluminium module
                None => return,
            }
        }
        HostToDevice::Led(led::HostToDevice::PhysKillNotify(asserting_kill)) => {
            Event::Command(Command::from_physkill_notify(asserting_kill))
        }
        HostToDevice::Led(led::HostToDevice::Depth(depth)) => Event::Depth(depth),
        other => {
            defmt::warn!("Unexpected message for the LED board: {}", other);
            return;
        }
    };
    if EVENTS.try_send(event).is_err() {
        defmt::warn!("LED event queue full, dropping a command");
    }
}

/// Starts the controllers' 204.8 kHz PWMCLK on GPIO 7 (PWM 3B).
pub fn start_pwm_clock(slice: Peri<'static, PWM_SLICE3>, pin: Peri<'static, PIN_7>) -> Pwm<'static> {
    let mut config = pwm::Config::default();
    config.top = l99ld20::PWM_CLOCK_TOP;
    config.compare_b = l99ld20::PWM_CLOCK_LEVEL;
    config.divider = FixedU16::from_bits(l99ld20::pwm_clock_divider(embassy_rp::clocks::clk_sys_freq()));
    Pwm::new_output_b(slice, pin, config)
}

/// A repeating deadline. Late runs are not made up, like the pico-sdk repeating timers.
struct Periodic {
    next: Instant,
    period: Duration,
}

impl Periodic {
    fn every_ms(start: Instant, ms: u64) -> Self {
        let period = Duration::from_millis(ms);
        Periodic { next: start + period, period }
    }

    fn fire(&mut self, now: Instant) -> bool {
        if now < self.next {
            return false;
        }
        self.next += self.period;
        if self.next <= now {
            self.next = now + self.period;
        }
        true
    }
}

/// Sends `light` to the bucks (`led_set_rgb`). The C code wrote each buck separately; writing both duties of a
/// controller in one read-modify-write ends in the same registers with half the frames.
fn show(ledc: &mut Controllers, light: Light) {
    let duties = light.duties();
    for chip in Chip::ALL {
        defmt::unwrap!(ledc.set_duties(chip, duties[chip as usize]));
    }
}

/// `ledc_init`: LEDs off, PWM dimming at the normal peak current, then each controller enabled.
async fn init(ledc: &mut Controllers) {
    show(ledc, Light::OFF);
    for chip in Chip::ALL {
        for buck in Buck::ALL {
            defmt::unwrap!(ledc.set_din_map(chip, buck, DinMap::PwmDimming));
            defmt::unwrap!(ledc.set_peak_current(chip, buck, board::PEAK_CURRENT));
            Timer::after_millis(1).await;
        }

        // controller_clear_watchdog_error
        defmt::unwrap!(ledc.read_clear(chip, SR2));

        // controller_enable: unlock (also setting both high LED current bits), then set EN
        Timer::after_millis(1).await;
        defmt::unwrap!(ledc.modify(chip, CR1, |cr1| {
            Cr1(cr1)
                .with_high_led_current(Buck::Buck1, true)
                .with_high_led_current(Buck::Buck2, true)
                .with_unlock(true)
                .0
        }));
        Timer::after_millis(2).await;
        defmt::unwrap!(ledc.modify(chip, CR2, |cr2| Cr2(cr2).with_enable(true).0));
        Timer::after_millis(1).await;
    }
}

/// Logs what the C `leds` debug command printed; the CANmore debug commands are not ported yet.
fn log_status(ledc: &mut Controllers) {
    for chip in Chip::ALL {
        let (global, status) = defmt::unwrap!(ledc.status(chip));
        defmt::info!("{}: {}, global status {=u8:#04x}", chip, status, global.0);
        for flag in global.flags() {
            defmt::info!("{}:  - {=str}", chip, flag);
        }
    }
}

/// The controller watchdog (`controller_satisfy_watchdog`): writes the trigger bit to both controllers, alternating
/// high and low, and reads LEDC1's CR4 back to check the write landed.
struct ControllerWatchdog {
    high: bool,
    previous: u32,
    /// Read-backs that did not change since the last check.
    failures: u32,
}

impl ControllerWatchdog {
    fn pet(&mut self, ledc: &mut Controllers) {
        self.high = !self.high;
        let value = if self.high { CR4_WATCHDOG_TRIGGER } else { 0 };
        for chip in Chip::ALL {
            defmt::unwrap!(ledc.write(chip, CR4, value));
        }
        let readback = defmt::unwrap!(ledc.read(Chip::Ledc1, CR4)).data;
        if readback == self.previous {
            self.failures += 1;
        }
        self.previous = readback;
    }
}

/// Runs the module forever.
#[embassy_executor::task]
pub async fn led_task(mut hw: Hardware) -> ! {
    init(&mut hw.ledc).await;
    log_status(&mut hw.ledc);

    let mut board = LedBoard::new();
    let mut watchdog = ControllerWatchdog { high: false, previous: 0, failures: 0 };

    let start = Instant::now();
    let mut watchdog_timer = Periodic::every_ms(start, board::CONTROLLER_WATCHDOG_PERIOD_MS);
    let mut singleton_timer = Periodic::every_ms(start, board::SINGLETON_PERIOD_MS);
    let mut refresh_timer = Periodic::every_ms(start, REFRESH_PERIOD_MS);
    let mut depth_timer = Periodic::every_ms(start, board::DEPTH_MONITOR_PERIOD_MS);
    let mut temperature_timer = Periodic::every_ms(start, board::TEMPERATURE_MONITOR_PERIOD_MS);

    loop {
        let wake = [&watchdog_timer, &singleton_timer, &refresh_timer, &depth_timer, &temperature_timer]
            .iter()
            .map(|t| t.next)
            .min()
            .unwrap();
        if let Either::First(event) = select(EVENTS.receive(), Timer::at(wake)).await {
            match event {
                Event::Command(command) => board.command(command),
                Event::Depth(depth) => board.set_depth(depth),
            }
        }

        let now = Instant::now();
        if watchdog_timer.fire(now) {
            watchdog.pet(&mut hw.ledc);
        }
        if temperature_timer.fire(now) {
            match hw.adc.blocking_read(&mut hw.thermistor) {
                Ok(raw) => {
                    let celsius = thermal::thermistor_celsius(raw);
                    if board.set_temperature(celsius) {
                        safety::raise_fault(FAULT_LED_OVERTEMP, celsius as u32);
                    } else {
                        safety::lower_fault(FAULT_LED_OVERTEMP);
                    }
                }
                Err(err) => defmt::warn!("Thermistor read failed: {}", err),
            }
            if watchdog.failures > 0 {
                defmt::debug!("LEDC1 CR4 read back unchanged {} times", watchdog.failures);
                watchdog.failures = 0;
            }
        }
        if depth_timer.fire(now) {
            board.check_depth();
        }
        if refresh_timer.fire(now) {
            show(&mut hw.ledc, board.refresh());
        }
        if singleton_timer.fire(now)
            && let Some(light) = board.singleton_tick()
        {
            show(&mut hw.ledc, light);
        }
    }
}
