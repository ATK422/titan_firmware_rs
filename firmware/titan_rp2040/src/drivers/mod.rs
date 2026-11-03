//! Device drivers shared by several boards, ported from the C `driver_*` libraries.
//!
//! * [`sht41`]: SHT41 temperature/humidity sensor (`driver_sht41`)
//! * [`mcp3426`]: MCP3426 ADC (`driver_mcp3426`)
//! * [`depth`]: MS5837-02BA/30BA depth sensor (`driver_depth`)
//! * [`cd74hc4051`]: CD74HC4051 analog multiplexer (`driver_cd74hc4051`)
//! * [`status_strip`]: WS2812 status strip on the Mk2 electronics cages (`driver_status_strip`)
//!
//! # I2C
//!
//! The C drivers queued callback chains on `driver_async_i2c`. Here each I2C driver owns something implementing
//! [`embedded_hal_async::i2c::I2c`] and simply awaits its transfers. That can be an `embassy_rp::i2c::I2c` in async
//! mode, or an `embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice` when several drivers share one bus. Every
//! transaction is bounded by [`I2C_TIMEOUT`], as `async_i2c_init`'s bus timeout bounded them in C.
//!
//! The conversions and CRCs live in the host-tested `titan_sensors` crate.
//!
//! # Errors and faults
//!
//! The C drivers called a board supplied error callback, which raised a board specific fault. The Rust drivers
//! don't know about board faults. The single-shot methods return a `Result`, and each driver's long running `run`
//! loop takes an `on_error` closure. The loop calls it exactly where the C driver called the board's callback,
//! after the same number of consecutive failures, and keeps running. The board picks the fault:
//!
//! ```ignore
//! sht41.run(&SHT41, |err| safety::raise_fault(FAULT_SHT41_ERROR, err.code())).await
//! ```
//!
//! Each error type has a `code()` for the fault's `extra` argument, using the C error enum values where there was
//! one.
//!
//! # Readings
//!
//! The C drivers kept their last reading in globals plus a `*_set_on_read` flag. The Rust drivers publish into a
//! [`Latest`] (or a driver specific state struct built on it) that the board declares as a `static`, so the
//! publishing tasks and any task reading the values can share it.

use core::cell::Cell;
use core::future::Future;

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, with_timeout};
use embedded_hal_async::i2c::{ErrorKind, I2c};

pub mod cd74hc4051;
pub mod depth;
pub mod mcp3426;
pub mod sht41;
pub mod status_strip;

/// Longest a single I2C transaction may take, the bus timeout the C boards passed to `async_i2c_init`.
///
/// On a shared bus this includes waiting for the bus lock. A transaction that times out is abandoned mid-transfer,
/// so the next one on that bus may fail too; the drivers count that as one more failure and carry on.
pub const I2C_TIMEOUT: Duration = Duration::from_millis(10);

/// Why an I2C transaction failed. A bus-agnostic copy of [`ErrorKind`] plus timeouts.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub enum I2cError {
    /// The device didn't acknowledge its address or a data byte. Usually means it's missing or unpowered.
    NoAcknowledge,
    /// Another controller won arbitration.
    ArbitrationLoss,
    /// Misplaced start or stop condition.
    Bus,
    /// Receive overrun.
    Overrun,
    /// Anything else the bus driver reported.
    Other,
    /// The transaction didn't finish within [`I2C_TIMEOUT`].
    Timeout,
}

impl I2cError {
    /// Value for a fault's `extra` argument.
    pub fn code(self) -> u32 {
        self as u32
    }

    fn from_kind(kind: ErrorKind) -> Self {
        match kind {
            ErrorKind::NoAcknowledge(_) => I2cError::NoAcknowledge,
            ErrorKind::ArbitrationLoss => I2cError::ArbitrationLoss,
            ErrorKind::Bus => I2cError::Bus,
            ErrorKind::Overrun => I2cError::Overrun,
            _ => I2cError::Other,
        }
    }
}

/// Runs one I2C transaction with [`I2C_TIMEOUT`].
async fn transaction<E: embedded_hal_async::i2c::Error>(
    fut: impl Future<Output = Result<(), E>>,
) -> Result<(), I2cError> {
    match with_timeout(I2C_TIMEOUT, fut).await {
        Ok(Ok(())) => Ok(()),
        Ok(Err(e)) => Err(I2cError::from_kind(e.kind())),
        Err(_) => Err(I2cError::Timeout),
    }
}

/// Writes `bytes` to the device at `address`, ending with a stop.
async fn i2c_write<I: I2c>(i2c: &mut I, address: u8, bytes: &[u8]) -> Result<(), I2cError> {
    transaction(i2c.write(address, bytes)).await
}

/// Reads `buf.len()` bytes from the device at `address`, ending with a stop.
async fn i2c_read<I: I2c>(i2c: &mut I, address: u8, buf: &mut [u8]) -> Result<(), I2cError> {
    transaction(i2c.read(address, buf)).await
}

#[derive(Clone, Copy)]
struct Slot<T> {
    value: Option<(T, Instant)>,
    unread: bool,
}

/// The most recent reading from a sensor, with its age, shared between the driver task and its readers.
///
/// Declare it as a `static` next to the driver: `static SHT41: Latest<sht41::Reading> = Latest::new(sht41::VALID_FOR);`
pub struct Latest<T: Copy> {
    slot: Mutex<CriticalSectionRawMutex, Cell<Slot<T>>>,
    valid_for: Duration,
}

impl<T: Copy> Latest<T> {
    /// A reading counts as valid for `valid_for` after it's published.
    pub const fn new(valid_for: Duration) -> Self {
        Latest { slot: Mutex::new(Cell::new(Slot { value: None, unread: false })), valid_for }
    }

    /// Stores a new reading, stamped with the current time.
    pub fn publish(&self, value: T) {
        self.slot.lock(|slot| slot.set(Slot { value: Some((value, Instant::now())), unread: true }));
    }

    /// Forgets the current reading, so [`Latest::get`] returns `None` until the next one.
    pub fn invalidate(&self) {
        self.slot.lock(|slot| slot.set(Slot { value: None, unread: false }));
    }

    /// The current reading, or `None` if there is none or it's older than the validity window.
    pub fn get(&self) -> Option<T> {
        let (value, at) = self.last()?;
        (Instant::now() < at + self.valid_for).then_some(value)
    }

    /// The last reading and when it was published, however old.
    pub fn last(&self) -> Option<(T, Instant)> {
        self.slot.lock(|slot| slot.get().value)
    }

    /// Returns each published reading once, like polling and clearing the C `*_set_on_read` flags.
    pub fn take_new(&self) -> Option<T> {
        self.slot.lock(|slot| {
            let s = slot.get();
            if !s.unread {
                return None;
            }
            slot.set(Slot { unread: false, ..s });
            s.value.map(|(value, _)| value)
        })
    }
}
