//! Sensirion SHT41 temperature/humidity sensor, ported from `driver_sht41`.
//!
//! Every [`POLL_PERIOD`] the driver sends the high precision measurement command, waits [`SAMPLING_TIME`] and reads
//! back the 6 byte result (two words, each followed by a CRC-8). Readings are published to a [`Latest`] and count as
//! valid for [`VALID_FOR`], one poll period plus 100 ms of slack.
//!
//! # Errors
//!
//! [`Sht41::run`] calls `on_error` once [`ERROR_THRESHOLD`] consecutive measurements have failed, and again for
//! every further failure, until a measurement succeeds. That's where the C driver called the board's error
//! callback. The board decides which fault to raise.
//!
//! # Differences from C
//!
//! * A measurement with a bad CRC is dropped and counts as a failure. The C driver flagged the error but then
//!   stored the corrupt words anyway, marked them valid and reset the error count.
//! * A failed write or read is retried on the next poll, as in C. The C "timer queue full" errors can't happen.
//!
//! ```ignore
//! static SHT41: Latest<sht41::Reading> = Latest::new(sht41::VALID_FOR);
//!
//! #[embassy_executor::task]
//! async fn sht41_task(mut sensor: Sht41<I2c<'static, I2C1, Async>>) -> ! {
//!     sensor.run(&SHT41, |err| safety::raise_fault(FAULT_SHT41_ERROR, err.code())).await
//! }
//! ```

use embassy_time::{Duration, Ticker, Timer};
use embedded_hal_async::i2c::I2c;
use titan_sensors::sht41;

use super::{I2cError, Latest, i2c_read, i2c_write};

/// 7-bit I2C address of the SHT41-A.
pub const ADDRESS: u8 = 0x44;
/// Time between measurements (`SHT41_POLL_TIME_MS`).
pub const POLL_PERIOD: Duration = Duration::from_millis(500);
/// How long a reading stays valid (`SHT41_INVALID_TIMEOUT_MS`).
pub const VALID_FOR: Duration = Duration::from_millis(POLL_PERIOD.as_millis() + 100);
/// Time the sensor needs for a high precision measurement.
pub const SAMPLING_TIME: Duration = Duration::from_millis(15);
/// Consecutive failures before `on_error` is called (`SHT41_ACCEPTABLE_ERR_COUNT`).
pub const ERROR_THRESHOLD: u8 = 3;

/// Measure temperature and humidity with high precision (high repeatability).
const CMD_MEASURE_HIGH_PRECISION: u8 = 0xFD;

/// A temperature and humidity reading.
#[derive(Clone, Copy, PartialEq, Debug, defmt::Format)]
pub struct Reading {
    /// Temperature in degrees Celsius.
    pub temperature_c: f32,
    /// Relative humidity in %RH.
    pub humidity_rh: f32,
}

/// Why a measurement failed.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub enum Error {
    /// The measurement command or the read failed (`SHT41_ERROR_I2C_COMPLAINT`).
    I2c(I2cError),
    /// A CRC in the result didn't match (`SHT41_ERROR_CRC_INVALID`).
    Crc,
}

impl Error {
    /// Value for a fault's `extra` argument: the C `sht41_error_code`.
    pub fn code(self) -> u32 {
        match self {
            Error::Crc => 3,
            Error::I2c(_) => 4,
        }
    }
}

impl From<I2cError> for Error {
    fn from(err: I2cError) -> Self {
        Error::I2c(err)
    }
}

/// An SHT41 on an I2C bus.
pub struct Sht41<I> {
    i2c: I,
    address: u8,
}

impl<I: I2c> Sht41<I> {
    /// An SHT41 at the standard [`ADDRESS`].
    pub fn new(i2c: I) -> Self {
        Self::with_address(i2c, ADDRESS)
    }

    /// An SHT41 at another address (the -B variant answers on 0x45).
    pub fn with_address(i2c: I, address: u8) -> Self {
        Sht41 { i2c, address }
    }

    /// Takes one high precision measurement. Takes about [`SAMPLING_TIME`].
    pub async fn measure(&mut self) -> Result<Reading, Error> {
        i2c_write(&mut self.i2c, self.address, &[CMD_MEASURE_HIGH_PRECISION]).await?;
        Timer::after(SAMPLING_TIME).await;
        let mut buf = [0u8; 6];
        i2c_read(&mut self.i2c, self.address, &mut buf).await?;
        let raw = sht41::parse_measurement(&buf).map_err(|_| Error::Crc)?;
        Ok(Reading {
            temperature_c: sht41::temperature_c(raw.temperature),
            humidity_rh: sht41::humidity_rh(raw.humidity),
        })
    }

    /// Measures every [`POLL_PERIOD`] forever, publishing each reading to `readings`. See the module docs for when
    /// `on_error` is called.
    pub async fn run(&mut self, readings: &Latest<Reading>, mut on_error: impl FnMut(Error)) -> ! {
        let mut ticker = Ticker::every(POLL_PERIOD);
        let mut consecutive_errors: u8 = 0;
        loop {
            ticker.next().await;
            match self.measure().await {
                Ok(reading) => {
                    consecutive_errors = 0;
                    readings.publish(reading);
                }
                Err(err) => {
                    defmt::warn!("SHT41 measurement failed: {}", err);
                    consecutive_errors = consecutive_errors.saturating_add(1);
                    if consecutive_errors >= ERROR_THRESHOLD {
                        on_error(err);
                    }
                }
            }
        }
    }
}
