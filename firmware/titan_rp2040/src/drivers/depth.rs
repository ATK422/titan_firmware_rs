//! MS5837-02BA / MS5837-30BA depth (pressure) sensor, ported from `driver_depth` (`depth.c` and `ms5837.c`).
//!
//! [`Ms5837`] is the sensor itself: [`Ms5837::reset`] resets it and loads and CRC-checks its calibration PROM,
//! [`Ms5837::convert`] runs a D1 (pressure) and a D2 (temperature) conversion at oversampling 8192 and compensates
//! them. The math lives in [`titan_sensors::ms5837`].
//!
//! [`Ms5837::run`] is the C driver's state machine on top of that:
//! 1. reset and PROM check
//! 2. zeroing: unless a surface pressure survived in a [`SurfaceStore`] from before a watchdog reset, take
//!    [`Zeroing::READINGS`] back-to-back conversions (about 2 s) and average the surface pressure from them
//! 3. poll every [`POLL_PERIOD`], publishing each [`Reading`] to a [`DepthState`]. A reading stays valid for
//!    [`VALID_FOR`], the time [`BAD_READS_FAULT_COUNT`] polls take. [`DepthState::request_recalibration`] makes the
//!    next poll zero again instead.
//!
//! # Errors
//!
//! The C driver reported every error through one callback with a `recoverable` flag. Here:
//! * **Fatal** errors (reset command failed, bad PROM CRC, any failure during PROM read or zeroing) end
//!   [`Ms5837::run`], which returns the error. The state is marked uninitialized. The board raises its "depth init"
//!   fault and may call `run` again to retry, which the C board could only do by calling `depth_init` again.
//! * **Recoverable** errors go to `on_error` and polling continues: a failed conversion once
//!   [`BAD_READS_FAULT_COUNT`] have failed in a row (and every failure after that, until one succeeds), and every
//!   missed poll ([`Error::Overrun`]).
//!
//! ```ignore
//! static DEPTH: DepthState = DepthState::new();
//! #[unsafe(link_section = ".uninit.depth")]
//! static SURFACE: SurfaceStore = SurfaceStore::new();
//!
//! #[embassy_executor::task]
//! async fn depth_task(mut sensor: Ms5837<I2c<'static, I2C0, Async>>) -> ! {
//!     let err = sensor.run(&DEPTH, &SURFACE, |err| safety::raise_fault(FAULT_DEPTH_ERROR, err.code())).await;
//!     safety::raise_fault(FAULT_DEPTH_INIT_ERROR, err.code());
//!     core::future::pending().await
//! }
//! ```
//!
//! # Differences from C
//!
//! * The C "I2C queue full" and "alarm queue full" errors can't happen and are gone.
//! * A poll that would have found the previous conversion still running is reported as [`Error::Overrun`] once
//!   the conversion finishes, rather than from the timer; the following poll keeps to the original schedule.
//! * The 30BA high temperature compensation term no longer overflows; see [`titan_sensors::ms5837::compensate`].

use core::cell::UnsafeCell;
use core::mem::MaybeUninit;

use embassy_time::{Duration, Instant, Timer};
use embedded_hal_async::i2c::I2c;
use portable_atomic::{AtomicBool, Ordering};
use titan_sensors::ms5837::{self, Measurement, PROM_WORDS, Zeroing};

pub use titan_sensors::ms5837::Model;

use super::{I2cError, Latest, i2c_read, i2c_write};

/// 7-bit I2C address of every MS5837.
pub const ADDRESS: u8 = 0x76;
/// Time between conversions once zeroed (`DEPTH_POLLING_RATE_MS`).
pub const POLL_PERIOD: Duration = Duration::from_millis(50);
/// Consecutive failed conversions before `on_error` is called (`DEPTH_BAD_READS_FAULT_COUNT`).
pub const BAD_READS_FAULT_COUNT: u32 = 5;
/// How long a reading stays valid.
pub const VALID_FOR: Duration = Duration::from_millis(POLL_PERIOD.as_millis() * BAD_READS_FAULT_COUNT as u64);

const CMD_RESET: u8 = 0x1E;
const CMD_ADC_READ: u8 = 0x00;
/// OSR 8192, the highest resolution.
const OVERSAMPLING: u8 = 5;
const CMD_CONVERT_D1: u8 = 0x40 + (OVERSAMPLING << 1);
const CMD_CONVERT_D2: u8 = 0x50 + (OVERSAMPLING << 1);
const fn cmd_prom_read(word: u8) -> u8 {
    0xA0 + (word << 1)
}

/// Time the sensor needs to reload its PROM after a reset.
const RESET_DELAY: Duration = Duration::from_millis(10);
/// Conversion time at [`OVERSAMPLING`], with the C driver's 2 ms margin: 2.5 us * 2^(8 + OSR) + 2 ms = 22.48 ms.
const CONVERT_DELAY: Duration = Duration::from_micros((5 * (1 << (8 + OVERSAMPLING as u64))) / 2 + 2000);

/// Why an operation failed. `code()` gives the C `depth_error_event` value.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub enum Error {
    /// The reset command failed (`DEPTH_ERROR_RESET_CMD`). Fatal.
    ResetCmd(I2cError),
    /// The PROM CRC didn't match (`DEPTH_ERROR_BAD_CRC`). Fatal.
    BadCrc { stored: u8, computed: u8 },
    /// A poll came due while the previous conversion was still running (`DEPTH_ERROR_OVERRUN`). Recoverable.
    Overrun,
    /// Any other transfer failed (`DEPTH_ERROR_COMM_FAIL`). Fatal during initialization and zeroing, recoverable
    /// while polling.
    Comm(I2cError),
}

impl Error {
    /// Value for a fault's `extra` argument: the C `depth_error_event`.
    pub fn code(self) -> u32 {
        match self {
            Error::ResetCmd(_) => 0,
            Error::BadCrc { .. } => 1,
            Error::Overrun => 2,
            Error::Comm(_) => 5,
        }
    }
}

impl From<I2cError> for Error {
    fn from(err: I2cError) -> Self {
        Error::Comm(err)
    }
}

/// A depth reading.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub struct Reading {
    /// Absolute pressure in pascals.
    pub pressure_pa: i32,
    /// Surface pressure from the last zeroing, in pascals.
    pub surface_pressure_pa: i32,
    /// Water temperature in hundredths of a degree Celsius.
    pub temperature_centi_c: i32,
}

impl Reading {
    /// Depth below the surface in metres (`depth_read`). Positive is down.
    pub fn depth_m(&self) -> f64 {
        ms5837::depth_m(self.pressure_pa, self.surface_pressure_pa)
    }

    /// Water temperature in degrees Celsius (`depth_get_temperature`).
    pub fn temperature_c(&self) -> f32 {
        self.temperature_centi_c as f32 / 100.0
    }
}

/// State shared between [`Ms5837::run`] and the rest of the board. Declare it as a `static`.
pub struct DepthState {
    readings: Latest<Reading>,
    initialized: AtomicBool,
    recalibrate: AtomicBool,
}

impl DepthState {
    pub const fn new() -> Self {
        DepthState {
            readings: Latest::new(VALID_FOR),
            initialized: AtomicBool::new(false),
            recalibrate: AtomicBool::new(false),
        }
    }

    /// Whether the sensor is initialized and zeroed (`depth_initialized`). Never true if initialization failed.
    pub fn is_initialized(&self) -> bool {
        self.initialized.load(Ordering::Relaxed)
    }

    /// The current reading, if the sensor is initialized and the reading is fresh (`depth_reading_valid`, then
    /// `depth_read` / `depth_get_temperature`).
    pub fn reading(&self) -> Option<Reading> {
        self.readings.get().filter(|_| self.is_initialized())
    }

    /// Returns each new reading once (`depth_set_on_read`), for publishing as soon as it arrives.
    pub fn take_new(&self) -> Option<Reading> {
        self.readings.take_new()
    }

    /// Zeroes the sensor again at the next poll (`depth_recalibrate`). Readings are invalid until it's done.
    pub fn request_recalibration(&self) {
        self.recalibrate.store(true, Ordering::Relaxed);
    }
}

impl Default for DepthState {
    fn default() -> Self {
        Self::new()
    }
}

/// Magic value marking a valid stored surface pressure (`SURFACE_PRESSURE_VALID_MAGIC`).
const SURFACE_VALID_MAGIC: u32 = 0x55AA_6996;

/// A surface pressure that survives a watchdog reset, so a sensor that's already underwater isn't zeroed there.
///
/// It must live in RAM the runtime doesn't initialize, so declare it in the `.uninit` section:
///
/// ```ignore
/// #[unsafe(link_section = ".uninit.depth")]
/// static SURFACE: SurfaceStore = SurfaceStore::new();
/// ```
///
/// Without the attribute it lands in `.bss`, gets zeroed at every boot and the sensor is always zeroed. Use one
/// store per sensor, and only from that sensor's [`Ms5837::run`].
pub struct SurfaceStore {
    /// `[surface pressure, surface pressure ^ magic, magic]`, like the three C variables.
    words: UnsafeCell<MaybeUninit<[u32; 3]>>,
}

// SAFETY: only the one `Ms5837::run` future given the store touches it, through volatile accesses. Futures don't
// run concurrently with themselves, and the store is documented as single-user.
unsafe impl Sync for SurfaceStore {}

impl SurfaceStore {
    pub const fn new() -> Self {
        SurfaceStore { words: UnsafeCell::new(MaybeUninit::uninit()) }
    }

    fn word(&self, i: usize) -> *mut u32 {
        // In bounds: callers pass 0..3
        unsafe { (self.words.get() as *mut u32).add(i) }
    }

    /// The stored surface pressure, if the magic and check word are intact.
    fn load(&self) -> Option<i32> {
        // SAFETY: in-bounds, aligned, and volatile so the compiler makes no assumption about the (possibly never
        // written) contents
        let [pressure, check, magic] = [0, 1, 2].map(|i| unsafe { self.word(i).read_volatile() });
        (magic == SURFACE_VALID_MAGIC && pressure ^ SURFACE_VALID_MAGIC == check).then_some(pressure as i32)
    }

    fn store(&self, pressure_pa: i32) {
        let pressure = pressure_pa as u32;
        // SAFETY: as in `load`
        unsafe {
            self.word(0).write_volatile(pressure);
            self.word(1).write_volatile(pressure ^ SURFACE_VALID_MAGIC);
            self.word(2).write_volatile(SURFACE_VALID_MAGIC);
        }
    }

    fn invalidate(&self) {
        // SAFETY: as in `load`
        unsafe { self.word(2).write_volatile(0) };
    }
}

impl Default for SurfaceStore {
    fn default() -> Self {
        Self::new()
    }
}

/// An MS5837 on an I2C bus.
pub struct Ms5837<I> {
    i2c: I,
    model: Model,
    /// Calibration PROM, loaded by [`Ms5837::reset`].
    prom: Option<[u16; PROM_WORDS]>,
}

impl<I: I2c> Ms5837<I> {
    /// A sensor of the given model. Nothing is sent until [`Ms5837::reset`] or [`Ms5837::run`].
    pub fn new(i2c: I, model: Model) -> Self {
        Ms5837 { i2c, model, prom: None }
    }

    /// Resets the sensor and loads its calibration PROM, checking the CRC (`ms5837_init`).
    pub async fn reset(&mut self) -> Result<(), Error> {
        self.prom = None;
        i2c_write(&mut self.i2c, ADDRESS, &[CMD_RESET]).await.map_err(Error::ResetCmd)?;
        Timer::after(RESET_DELAY).await;

        let mut prom = [0u16; PROM_WORDS];
        for (word, value) in prom.iter_mut().enumerate() {
            i2c_write(&mut self.i2c, ADDRESS, &[cmd_prom_read(word as u8)]).await?;
            let mut buf = [0u8; 2];
            i2c_read(&mut self.i2c, ADDRESS, &mut buf).await?;
            *value = u16::from_be_bytes(buf);
        }
        ms5837::check_prom(&prom).map_err(|(stored, computed)| Error::BadCrc { stored, computed })?;

        self.prom = Some(prom);
        Ok(())
    }

    /// Runs one D1 and D2 conversion and compensates them (`ms5837_do_conversion`). Takes about 45 ms.
    ///
    /// # Panics
    ///
    /// If [`Ms5837::reset`] hasn't succeeded, like the C driver's assertion.
    pub async fn convert(&mut self) -> Result<Measurement, Error> {
        let prom = self.prom.expect("MS5837 converted before a successful reset");
        let d1 = self.adc_conversion(CMD_CONVERT_D1).await?;
        let d2 = self.adc_conversion(CMD_CONVERT_D2).await?;
        Ok(ms5837::compensate(self.model, &prom, d1, d2))
    }

    async fn adc_conversion(&mut self, cmd: u8) -> Result<u32, Error> {
        i2c_write(&mut self.i2c, ADDRESS, &[cmd]).await?;
        Timer::after(CONVERT_DELAY).await;
        i2c_write(&mut self.i2c, ADDRESS, &[CMD_ADC_READ]).await?;
        let mut buf = [0u8; 3];
        i2c_read(&mut self.i2c, ADDRESS, &mut buf).await?;
        Ok(u32::from_be_bytes([0, buf[0], buf[1], buf[2]]))
    }

    /// Averages the surface pressure over back-to-back conversions. Any failure is fatal, as in C.
    async fn zero(&mut self) -> Result<i32, Error> {
        let mut zeroing = Zeroing::new();
        loop {
            let m = self.convert().await?;
            if let Some(surface) = zeroing.feed(m.pressure_pa) {
                defmt::info!("Depth zeroed, surface pressure {} Pa", surface);
                return Ok(surface);
            }
        }
    }

    /// Initializes, zeroes and polls the sensor. Only returns on a fatal error, which it returns. See the module
    /// docs for the error handling.
    pub async fn run(&mut self, state: &DepthState, surface: &SurfaceStore, mut on_error: impl FnMut(Error)) -> Error {
        let result = self.run_inner(state, surface, &mut on_error).await;
        state.initialized.store(false, Ordering::Relaxed);
        state.readings.invalidate();
        let Err(err) = result;
        defmt::error!("Depth sensor failed: {}", err);
        err
    }

    async fn run_inner(
        &mut self,
        state: &DepthState,
        surface: &SurfaceStore,
        on_error: &mut impl FnMut(Error),
    ) -> Result<core::convert::Infallible, Error> {
        state.initialized.store(false, Ordering::Relaxed);
        state.readings.invalidate();
        self.reset().await?;

        let mut surface_pa = match surface.load() {
            Some(pressure) => {
                defmt::info!("Depth surface pressure found ({} Pa), skipping zeroing", pressure);
                pressure
            }
            None => {
                let pressure = self.zero().await?;
                surface.store(pressure);
                pressure
            }
        };
        state.initialized.store(true, Ordering::Relaxed);

        let mut bad_reads: u32 = 0;
        let mut next_poll = Instant::now() + POLL_PERIOD;
        loop {
            Timer::at(next_poll).await;

            if state.recalibrate.swap(false, Ordering::Relaxed) {
                state.readings.invalidate();
                surface.invalidate();
                surface_pa = self.zero().await?;
                surface.store(surface_pa);
                next_poll = Instant::now() + POLL_PERIOD;
                continue;
            }

            match self.convert().await {
                Ok(m) => {
                    bad_reads = 0;
                    state.readings.publish(Reading {
                        pressure_pa: m.pressure_pa,
                        surface_pressure_pa: surface_pa,
                        temperature_centi_c: m.temperature_centi_c,
                    });
                }
                Err(err) => {
                    defmt::warn!("Depth conversion failed: {}", err);
                    bad_reads = bad_reads.saturating_add(1);
                    if bad_reads >= BAD_READS_FAULT_COUNT {
                        on_error(err);
                    }
                }
            }

            // The C timer fired every POLL_PERIOD regardless and reported each tick that found a conversion running
            next_poll += POLL_PERIOD;
            let now = Instant::now();
            while next_poll <= now {
                on_error(Error::Overrun);
                next_poll += POLL_PERIOD;
            }
        }
    }
}
