//! The H-bridges, ported from `hbridge.c` and the kill/enable handlers in `safety_interface.c`.
//!
//! Four DRV8243 bridges are fitted (H0, H1, H3 and H4; the C firmware never set up H2). Each has a phase GPIO and
//! a 25 kHz PWM enable. All of them share nSLEEP (GPIO 25) and DRVOFF (GPIO 12). The nFAULT lines of H0, H1 and
//! H3 come in through mux inputs 0-2, H4's on GPIO 11.
//!
//! * Commands set targets directly from the link task ([`set_target`]), like the C executor callbacks.
//! * [`on_kill`] runs inside the safety critical section: it disables the bridges, zeroes their outputs and
//!   raises DRVOFF, which cuts the drivers at once.
//! * [`on_enable`] can't run the C wake-up sequence there (it takes over a millisecond and reads the ADC), so it
//!   only signals [`hbridge_task`], which wakes the bridges and then enables them unless the vehicle was killed
//!   again meanwhile. Commands in the ~1.5 ms between enable and the end of the wake-up are ignored.
//! * [`hbridge_task`] also runs the 20 ms slew timer, which ran in a repeating timer interrupt in C.

use core::cell::RefCell;

use embassy_futures::select::{Either, select};
use embassy_rp::adc::{self, Adc};
use embassy_rp::gpio::{Input, Level, Output};
use embassy_rp::pwm::{self, Pwm};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Ticker, Timer, block_for};
use fixed::FixedU16;
use fixed::types::extra::U4;

use hbridge_core::bridge::{self, Bridges, PwmParams, TargetError};
use titan_rp2040::drivers::cd74hc4051::Cd74hc4051;
use titan_rp2040::safety;

/// Bridges fitted (`hbridge_create` calls).
pub const NUM_BRIDGES: usize = 4;
/// Which bridges are wired backwards (`H*_INVERTED` of H0, H1, H3, H4).
const INVERTED: [bool; NUM_BRIDGES] = [false, false, true, false];

/// Board fault: fewer bridges than expected answered the wake-up (`FAULT_HBRIDGE_DISCONNECTED`). The argument is
/// the number missing.
pub const FAULT_HBRIDGE_DISCONNECTED: u8 = 5;

struct Shared {
    bridges: Bridges<NUM_BRIDGES>,
    /// High disables every driver output.
    drvoff: Option<Output<'static>>,
}

static SHARED: Mutex<CriticalSectionRawMutex, RefCell<Shared>> =
    Mutex::new(RefCell::new(Shared { bridges: Bridges::new(INVERTED), drvoff: None }));
static ENABLE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Safety kill hook (`safety_handle_kill`): stops every motor.
pub fn on_kill() {
    SHARED.lock(|shared| {
        let mut shared = shared.borrow_mut();
        shared.bridges.kill();
        if let Some(drvoff) = &mut shared.drvoff {
            drvoff.set_high();
        }
    });
}

/// Safety enable hook (`safety_handle_enable`): wakes and enables the bridges from [`hbridge_task`].
pub fn on_enable() {
    ENABLE.signal(());
}

/// Sets the target of the bridge behind command topic `idx` (`hbridge_set_target`).
pub fn set_target(idx: u8, percent: f32) {
    match SHARED.lock(|shared| shared.borrow_mut().bridges.set_target(idx as usize, percent)) {
        // Dropped silently in C: commands while killed, and the topic of the unfitted fifth bridge
        Ok(()) | Err(TargetError::Disabled) | Err(TargetError::NoSuchBridge) => {}
        Err(TargetError::NotANumber) => defmt::warn!("Ignoring NaN target for bridge {}", idx),
    }
}

/// Where a bridge's nFAULT line is read.
pub enum Nfault {
    /// Mux input number.
    Mux(u8),
    Gpio(Input<'static>),
}

/// One bridge's pins.
pub struct BridgePins {
    pub phase: Output<'static>,
    pub pwm: Pwm<'static>,
    pub nfault: Nfault,
}

/// Board hardware owned by [`hbridge_task`].
pub struct Hardware {
    pub bridges: [BridgePins; NUM_BRIDGES],
    pub nsleep: Output<'static>,
    pub mux: Cd74hc4051<'static>,
    pub adc: Adc<'static, adc::Async>,
}

/// PWM slice settings for an enable output, initially off (`hbridge_pwm_init`).
pub fn pwm_config() -> pwm::Config {
    let params: PwmParams = bridge::pwm_params(embassy_rp::clocks::clk_sys_freq());
    let mut config = pwm::Config::default();
    config.divider = FixedU16::<U4>::from_bits(params.divider_bits());
    config.top = params.wrap;
    config.compare_b = 0;
    config.enable = true;
    config
}

/// Hands DRVOFF to the kill hook. Call before ROS can connect, so before the kill hook can run.
pub fn init(drvoff: Output<'static>) {
    SHARED.lock(|shared| shared.borrow_mut().drvoff = Some(drvoff));
}

impl Hardware {
    /// Reads a bridge's nFAULT level (high = no fault). ADC errors read as `None`.
    async fn nfault(&mut self, idx: usize) -> Option<bool> {
        match &self.bridges[idx].nfault {
            Nfault::Gpio(pin) => Some(pin.is_high()),
            &Nfault::Mux(input) => match self.mux.read_digital(&mut self.adc, input).await {
                Ok(level) => Some(level),
                Err(err) => {
                    defmt::warn!("nFAULT read of bridge {} failed: {}", idx, err);
                    None
                }
            },
        }
    }

    /// Runs the DRV8243 wake-up sequence until every bridge reports in or the attempts run out, and raises or
    /// lowers the disconnected fault (`hbridge_wake`). Returns the number of bridges awake.
    async fn wake(&mut self) -> usize {
        let mut num_awake = 0;
        let mut attempts = 0;
        while num_awake < NUM_BRIDGES && attempts < bridge::MAX_WAKE_ATTEMPTS {
            // Put the bridges to sleep, then start the wake-up
            self.nsleep.set_low();
            Timer::after_micros(bridge::TIME_SLEEP_US).await;
            self.nsleep.set_high();
            Timer::after_micros(bridge::WAKEUP_TIME_FOR_COM_US).await;

            // A bridge that woke pulls nFAULT low. A failed read counts as not awake.
            let mut before = [true; NUM_BRIDGES];
            for (idx, level) in before.iter_mut().enumerate() {
                *level = self.nfault(idx).await.unwrap_or(true);
            }
            // Wait for the bridges to be ready. This wait is VERY IMPORTANT
            Timer::after_micros(bridge::WAKEUP_TIME_FOR_COM_US).await;

            // nSLEEP reset pulse acknowledges the wake-up. It must stay short or the bridges go back to sleep, so
            // it is busy-waited rather than left to the scheduler.
            self.nsleep.set_low();
            block_for(Duration::from_micros(bridge::WAKEUP_TIME_RESET_US));
            self.nsleep.set_high();

            let mut after = [false; NUM_BRIDGES];
            for (idx, level) in after.iter_mut().enumerate() {
                *level = self.nfault(idx).await.unwrap_or(false);
            }
            num_awake = bridge::count_awake(&before, &after);
            attempts += 1;
        }

        if num_awake < NUM_BRIDGES {
            safety::raise_fault(FAULT_HBRIDGE_DISCONNECTED, (NUM_BRIDGES - num_awake) as u32);
            defmt::warn!("Only {} hbridges found. Expected {}", num_awake, NUM_BRIDGES);
        } else {
            safety::lower_fault(FAULT_HBRIDGE_DISCONNECTED);
        }
        num_awake
    }

    /// Applies one slew step to the pins (`hbridge_slew`).
    fn slew(&mut self, config: &mut pwm::Config) {
        let outputs = SHARED.lock(|shared| shared.borrow_mut().bridges.slew(config.top));
        for (pins, output) in self.bridges.iter_mut().zip(outputs) {
            pins.phase.set_level(Level::from(output.phase));
            config.compare_b = output.level;
            pins.pwm.set_config(config);
        }
    }
}

/// Wakes and enables the bridges when the vehicle is enabled, and slews their outputs every 20 ms.
#[embassy_executor::task]
pub async fn hbridge_task(mut hw: Hardware) -> ! {
    let mut config = pwm_config();
    let mut ticker = Ticker::every(Duration::from_millis(bridge::SLEW_PERIOD_MS));
    loop {
        match select(ticker.next(), ENABLE.wait()).await {
            Either::First(()) => hw.slew(&mut config),
            Either::Second(()) => {
                hw.wake().await;
                // hbridge_set_enabled(true), unless a kill arrived during the wake-up. The check and the enable
                // share one critical section, so the kill hook can't slip in between.
                SHARED.lock(|shared| {
                    if !safety::asserting_kill() {
                        let mut shared = shared.borrow_mut();
                        shared.bridges.set_enabled(true);
                        if let Some(drvoff) = &mut shared.drvoff {
                            drvoff.set_low();
                        }
                    }
                });
            }
        }
    }
}
