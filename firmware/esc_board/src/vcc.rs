//! ESC rail voltage monitoring, from `vcc_meas_cb` in `dshot.c`.
//!
//! The ESCs must see neutral while they power up, or they lock out. The rail is sampled at 1 kHz (the C code ran the
//! ADC free-running at the same rate) and the control loop keeps the thrusters off while it is low, plus
//! [`esc_core::ESC_POWERUP_DELAY_MS`] after it comes back.

use embassy_rp::adc::{self, Adc};
use embassy_time::{Duration, Ticker};
use portable_atomic::{AtomicBool, AtomicU32, Ordering};

use esc_core::{ESC_POWER_THRESHOLD_MV, vcc_mv_from_adc};

const SAMPLE_PERIOD: Duration = Duration::from_millis(1);

static VCC_MV: AtomicU32 = AtomicU32::new(0);
static WAS_OFF: AtomicBool = AtomicBool::new(false);

/// Latest ESC rail voltage in millivolts.
pub fn vcc_mv() -> u32 {
    VCC_MV.load(Ordering::Relaxed)
}

/// True if the ESC rail is powered now.
pub fn esc_board_on() -> bool {
    esc_core::esc_board_on(vcc_mv())
}

/// True if any sample since the last call saw the rail off, so a short dropout between two control loop ticks isn't
/// missed (`esc_board_was_off`).
pub fn take_was_off() -> bool {
    WAS_OFF.swap(false, Ordering::Relaxed)
}

/// Samples the divided VCC on GPIO 28 (ADC 2) every millisecond.
#[embassy_executor::task]
pub async fn vcc_task(mut adc: Adc<'static, adc::Async>, mut vcc: adc::Channel<'static>) -> ! {
    let mut ticker = Ticker::every(SAMPLE_PERIOD);
    loop {
        ticker.next().await;
        match adc.read(&mut vcc).await {
            Ok(raw) => {
                let mv = vcc_mv_from_adc(raw);
                VCC_MV.store(mv, Ordering::Relaxed);
                if mv <= ESC_POWER_THRESHOLD_MV {
                    WAS_OFF.store(true, Ordering::Relaxed);
                }
            }
            Err(err) => defmt::warn!("VCC measurement failed: {}", err),
        }
    }
}
