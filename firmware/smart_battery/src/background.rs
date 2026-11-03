//! The core 0 background work of the C `main.c` loop (`tick_background_tasks`) and the display (`display.c`):
//! the OLED menu, the manufacturer info readout that gates ROS, the "ROS Connect"/"ROS Lost" messages and the auto
//! power off.

use core::cell::Cell;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_rp::gpio::{Input, Output};
use embassy_rp::i2c::{self, I2c};
use embassy_rp::peripherals::I2C0;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, Ticker};

use smart_battery_core::display::{Actions, Display, Draw, Inputs};
use smart_battery_core::faults;
use smart_battery_core::gfx::Framebuffer;
use smart_battery_core::sbs::MfgInfo;
use smart_battery_core::screens::{self, Screens, Snapshot};
use titan_rp2040::{link, safety};

use crate::fault;
use crate::gauge;
use crate::ssd1306::Ssd1306;

/// The board I2C bus (I2C0 on GPIO 0/1), shared by the display and the SHT41.
pub type I2cBus = embassy_sync::mutex::Mutex<CriticalSectionRawMutex, I2c<'static, I2C0, i2c::Async>>;
pub type BusDevice = I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, I2C0, i2c::Async>>;

/// How often the switch is sampled and the display state machine runs. The C main loop spun continuously.
const TICK: Duration = Duration::from_millis(10);

static MFG_READOUT_OKAY: AtomicBool = AtomicBool::new(false);
static MFG_INFO: Mutex<CriticalSectionRawMutex, Cell<Option<MfgInfo>>> = Mutex::new(Cell::new(None));

/// The pack's manufacturer info, once read while the pack is ready (`mfg_readout_okay` and `bq_mfg_info`). The C
/// firmware only brought ROS up while this was set.
pub fn mfg_readout() -> Option<MfgInfo> {
    if MFG_READOUT_OKAY.load(Ordering::Relaxed) { MFG_INFO.lock(Cell::get) } else { None }
}

/// "ROS connected" as the C firmware saw it: the bridge is there and the pack info is known.
fn ros_connected() -> bool {
    link::is_connected() && MFG_READOUT_OKAY.load(Ordering::Relaxed)
}

struct Ui {
    oled: Ssd1306<BusDevice>,
    fb: Framebuffer,
    display: Display,
    screens: Screens,
}

impl Ui {
    fn snapshot(&self) -> Snapshot {
        let shared = gauge::shared();
        let (not_connected_arg, safety_status_arg, pf_status_arg) = fault::gauge_fault_args();
        Snapshot {
            now: Instant::now().as_millis(),
            state: shared.state,
            info: shared.info,
            mfg: shared.mfg,
            mfg_readout_okay: MFG_READOUT_OKAY.load(Ordering::Relaxed),
            faults: safety::faults(),
            not_connected_arg,
            safety_status_arg,
            pf_status_arg,
        }
    }

    /// One `display_tick`.
    async fn tick(&mut self, switch: bool, message_pending: bool) {
        let snapshot = self.snapshot();
        let input = Inputs {
            now: snapshot.now,
            switch,
            battery: snapshot.state,
            safety_fault: snapshot.faults & (1 << faults::BQ40_SAFETY_STATUS) != 0,
        };
        let actions = self.display.tick(input, message_pending);
        self.apply(actions, &snapshot).await;
    }

    async fn apply(&mut self, actions: Actions, snapshot: &Snapshot) {
        if actions.turn_on {
            self.oled.set_display_on(true).await;
        }
        if let Some(contrast) = actions.contrast {
            self.oled.set_contrast(contrast).await;
        }
        if let Some(draw) = actions.draw {
            self.screens.render(&mut self.fb, draw, snapshot);
            self.oled.update(&self.fb).await;
            if draw == Draw::ClearAndOff {
                self.oled.set_display_on(false).await;
            }
        }
    }

    /// Shows a status message for a while (`display_show_msg`).
    async fn show_message(&mut self, switch: bool, msg: &str, submsg: &str) {
        screens::status_message(&mut self.fb, msg, submsg);
        self.oled.update(&self.fb).await;
        self.tick(switch, true).await;
    }
}

/// Shows a message on a freshly initialized display; used before the background task runs.
pub async fn show_early_message(i2c: BusDevice, msg: &str, submsg: &str) {
    let mut oled = Ssd1306::new(i2c);
    oled.init().await;
    let mut fb = Framebuffer::new();
    screens::status_message(&mut fb, msg, submsg);
    oled.update(&fb).await;
}

/// Runs the display and the background checks forever.
#[embassy_executor::task]
pub async fn background_task(i2c: BusDevice, switch: Input<'static>, mut power: Output<'static>) -> ! {
    let mut oled = Ssd1306::new(i2c);
    oled.init().await;
    // The switch state at startup, so holding it to power up isn't a tap
    let mut ui = Ui { oled, fb: Framebuffer::new(), display: Display::new(switch.is_high()), screens: Screens::new() };

    let mut was_connected = false;
    let mut ticker = Ticker::every(TICK);
    loop {
        let switch_level = switch.is_high();
        let state = gauge::state();

        ui.tick(switch_level, false).await;

        // Read out the manufacturer info once the pack is ready; start over whenever it isn't
        if state.pack_ready() {
            if !MFG_READOUT_OKAY.load(Ordering::Relaxed)
                && let Some(info) = gauge::mfg_info()
            {
                MFG_INFO.lock(|m| m.set(Some(info)));
                MFG_READOUT_OKAY.store(true, Ordering::Relaxed);
            }
        } else {
            MFG_READOUT_OKAY.store(false, Ordering::Relaxed);
        }

        // Only stay on while the pack is in use or someone is looking at the display
        if !state.in_use() && !ui.display.is_on() {
            if power.is_set_high() {
                defmt::info!("Both display is off and CAN Bus is offline - shutting off MCU to save power");
            }
            power.set_low();
        }

        let connected = ros_connected();
        if connected != was_connected {
            was_connected = connected;
            let msg = if connected { "ROS Connect" } else { "ROS Lost" };
            defmt::info!("{}", msg);
            ui.show_message(switch_level, msg, "").await;
        }

        ticker.next().await;
    }
}
