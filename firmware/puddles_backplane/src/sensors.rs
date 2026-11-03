//! The backplane's I2C devices.
//!
//! * Sensor bus (I2C0, GPIO 8/9): the MS5837-30BA depth sensor outside the hull.
//! * Board bus (I2C1, GPIO 6/7): the MCP3426 rail voltage ADC at 0x68, and an ADC128D818 at 0x2F that the C
//!   firmware initialized at boot but never read (`D818_read` was disabled, and the ESC currents it would report
//!   are commented out of `adc_update`). It is initialized the same way here and then left alone.

use embassy_rp::i2c::{Async, I2c};
use embassy_rp::peripherals::{I2C0, I2C1};
use embassy_time::{Duration, Timer, with_timeout};
use embedded_hal_async::i2c::I2c as _;

use titan_rp2040::drivers::depth::{DepthState, Ms5837, SurfaceStore};
use titan_rp2040::drivers::mcp3426::{self, Mcp3426, Voltages};
use titan_rp2040::safety;

use crate::safety_interface::{FAULT_ADC_ERROR, FAULT_DEPTH_ERROR, FAULT_DEPTH_INIT_ERROR};

/// Depth readings, published on `state/depth/raw` and `state/depth/temp`.
pub static DEPTH: DepthState = DepthState::new();
/// Surface pressure kept across watchdog resets, so a reset underwater doesn't zero the sensor there.
#[unsafe(link_section = ".uninit.depth")]
static SURFACE: SurfaceStore = SurfaceStore::new();
/// Rail voltages, published on `state/electrical`.
pub static VOLTAGES: Voltages = Voltages::new();

/// Runs the depth sensor. Recoverable errors raise `FAULT_DEPTH_ERROR`; if initialization or zeroing fails it
/// raises `FAULT_DEPTH_INIT_ERROR` and stays down, as the C driver did.
#[embassy_executor::task]
pub async fn depth_task(mut sensor: Ms5837<I2c<'static, I2C0, Async>>) -> ! {
    let err = sensor.run(&DEPTH, &SURFACE, |err| safety::raise_fault(FAULT_DEPTH_ERROR, err.code())).await;
    safety::raise_fault(FAULT_DEPTH_INIT_ERROR, err.code());
    core::future::pending().await
}

const D818_ADDRESS: u8 = 0x2F;
const D818_REG_CONFIG: u8 = 0x00;
const D818_REG_IRQ_MASK: u8 = 0x03;
const D818_REG_ADV_CONFIG: u8 = 0x0B;
/// Per-write timeout of `D818_init` (`make_timeout_time_ms(5)`).
const D818_WRITE_TIMEOUT: Duration = Duration::from_millis(5);

/// `D818_init`: the same register writes as C (reset, advanced configuration, mask all interrupts, start). Errors
/// are ignored, as in C.
async fn d818_init(i2c: &mut I2c<'static, I2C1, Async>) {
    let writes =
        [[D818_REG_CONFIG, 0x80], [D818_REG_ADV_CONFIG, 0x40], [D818_REG_IRQ_MASK, 0xFF], [D818_REG_CONFIG, 0x01]];
    for (i, write) in writes.iter().enumerate() {
        if !matches!(with_timeout(D818_WRITE_TIMEOUT, i2c.write(D818_ADDRESS, write)).await, Ok(Ok(()))) {
            defmt::debug!("ADC128D818 write {} failed", i);
        }
        if i == 0 {
            // The chip needs time after the reset bit
            Timer::after_millis(50).await;
        }
    }
}

/// Initializes the ADC128D818, then reads the MCP3426 forever. Failures raise `FAULT_ADC_ERROR`.
#[embassy_executor::task]
pub async fn adc_task(mut i2c: I2c<'static, I2C1, Async>) -> ! {
    d818_init(&mut i2c).await;
    let mut adc = Mcp3426::new(i2c, mcp3426::DEFAULT_ADDRESS);
    adc.run(&VOLTAGES, |err| safety::raise_fault(FAULT_ADC_ERROR, err.code())).await
}
