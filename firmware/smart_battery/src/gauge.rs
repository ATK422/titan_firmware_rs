//! The gauge task on core 1, ported from `core1.c`: polls the BQ40Z80, drives the SOC LEDs and the wake pin, runs
//! the emergency FET shutdown sequence and serves requests from core 0.
//!
//! Like C, the gauge runs alone on core 1 so the SMBus timing (the controller holds SCL low between bytes while
//! software prepares the next one) never waits on the rest of the firmware. Core 1 runs its own embassy executor.
//! The C SIO FIFO plus spin lock protected shared memory become [`REQUESTS`] and the reply signals, all
//! `CriticalSectionRawMutex` based (the RP2040 critical section is a hardware spin lock, so it works across cores).
//!
//! Core 1 checks in every loop; [`core1_watchdog_task`] on core 0 panics (resetting the chip) if it stops for
//! [`CORE1_CHECKIN_INTERVAL`], replacing `safety_core1_checkin` with `SAFETY_CORE1_CHECKIN_INTERVAL_MS=1000`.

use core::cell::Cell;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_futures::select::{Either, select};
use embassy_rp::gpio::Output;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Ticker, Timer};

use smart_battery_core::faults;
use smart_battery_core::monitor::{self, BattState, EmshutAction, EmshutCommand, EmshutRequest, ErrorAction, Monitor};
use smart_battery_core::remote_cmd::{BLOCK_LEN, DebugOp, DebugResult, StatusRegister};
use smart_battery_core::sbs::{BatteryInfo, MfgInfo, cmd};
use smart_battery_core::smbus::BqError;

use crate::bq40z80::Bq40z80;
use crate::fault;

/// Longest core 1 may go without checking in.
pub const CORE1_CHECKIN_INTERVAL: Duration = Duration::from_millis(1000);

/// Requests from core 0 (the C `sio_fifo_req_t` commands).
#[derive(Clone, Copy, defmt::Format)]
pub enum Request {
    Emshut(EmshutCommand),
    /// Read the side detect pin; answered on [`SIDE_DETECT`].
    SideDetect,
    /// Read a status register; answered on [`STATUS_REPLY`].
    Status(StatusRegister),
    /// A debug command; answered on [`DEBUG_REPLY`].
    Debug(DebugOp),
}

pub static REQUESTS: Channel<CriticalSectionRawMutex, Request, 4> = Channel::new();
/// `Some(true)` for port, `None` if the side detect couldn't be read.
pub static SIDE_DETECT: Signal<CriticalSectionRawMutex, Option<bool>> = Signal::new();
pub static STATUS_REPLY: Signal<CriticalSectionRawMutex, Option<u32>> = Signal::new();
pub static DEBUG_REPLY: Signal<CriticalSectionRawMutex, DebugResult> = Signal::new();

/// What core 1 shares with core 0 (`core1_status_shared_mem`).
#[derive(Clone, Copy, Default)]
pub struct Shared {
    pub state: BattState,
    pub info: BatteryInfo,
    /// Valid while `state.is_connected()`.
    pub mfg: MfgInfo,
}

static SHARED: Mutex<CriticalSectionRawMutex, Cell<Shared>> =
    Mutex::new(Cell::new(Shared { state: BattState::Uninitialized, info: zeroed_info(), mfg: zeroed_mfg() }));

const fn zeroed_info() -> BatteryInfo {
    // SAFETY: plain integers and arrays of integers; all zeroes is the Default value
    unsafe { core::mem::zeroed() }
}

const fn zeroed_mfg() -> MfgInfo {
    // SAFETY: as above
    unsafe { core::mem::zeroed() }
}

/// The latest pack state and readings.
pub fn shared() -> Shared {
    SHARED.lock(Cell::get)
}

/// The pack state (`core1_get_batt_state`).
pub fn state() -> BattState {
    shared().state
}

/// The manufacturer info, if the gauge is connected (`core1_get_pack_mfg_info`).
pub fn mfg_info() -> Option<MfgInfo> {
    let s = shared();
    s.state.is_connected().then_some(s.mfg)
}

static CORE1_CHECKIN: AtomicU32 = AtomicU32::new(0);

fn checkin() {
    CORE1_CHECKIN.store(Instant::now().as_millis() as u32, Ordering::Relaxed);
}

/// Panics if core 1 stops checking in (`safety_internal_multicore_tick`).
#[embassy_executor::task]
pub async fn core1_watchdog_task() -> ! {
    // Core 1 checks in as soon as it starts; give it the interval to get there
    checkin();
    let mut ticker = Ticker::every(Duration::from_millis(100));
    loop {
        ticker.next().await;
        let last = CORE1_CHECKIN.load(Ordering::Relaxed);
        let now = Instant::now().as_millis() as u32;
        if now.wrapping_sub(last) > CORE1_CHECKIN_INTERVAL.as_millis() as u32 {
            panic!("Core 1 did not check in to safety within required interval");
        }
    }
}

/// The SOC LEDs (red, yellow, green; `BQ_LEDS`) and the wake pin.
pub struct Pins {
    pub leds: [Output<'static>; 3],
    pub wake: Output<'static>,
}

impl Pins {
    fn clear_soc_leds(&mut self) {
        for led in &mut self.leds {
            led.set_low();
        }
    }

    fn update_soc_leds(&mut self, soc: u8) {
        let lit = monitor::soc_led(soc);
        for (i, led) in self.leds.iter_mut().enumerate() {
            led.set_level((i == lit).into());
        }
    }

    /// Wakes the gauge from shutdown (`bq_pulse_wake`).
    async fn pulse_wake(&mut self) {
        self.wake.set_high();
        Timer::after_millis(monitor::WAKE_PULSE_DURATION_MS).await;
        self.wake.set_low();
    }
}

struct Gauge {
    bq: Bq40z80,
    pins: Pins,
    monitor: Monitor,
    /// The serial number burned into this board, which must match the gauge.
    board_serial: u8,
    mfg: MfgInfo,
    info: BatteryInfo,
}

fn ms(t: Instant) -> u64 {
    t.as_millis()
}

impl Gauge {
    fn publish(&self) {
        let shared = Shared { state: self.monitor.state(), info: self.info, mfg: self.mfg };
        SHARED.lock(|s| s.set(shared));
    }

    /// One poll of the gauge (the refresh half of `core1_main`).
    async fn refresh(&mut self) {
        let result = self.poll().await;
        match result {
            Ok(()) => self.poll_succeeded().await,
            Err(err) => match self.monitor.poll_failed() {
                ErrorAction::Retry => {}
                ErrorAction::Disconnected { wake } => {
                    fault::raise(faults::BQ40_NOT_CONNECTED, err.raw());
                    self.pins.clear_soc_leds();
                    self.publish();
                    if wake {
                        defmt::warn!("Lost the gauge ({}), waking it", err);
                        self.pins.pulse_wake().await;
                    }
                }
            },
        }
    }

    async fn poll(&mut self) -> Result<(), BqError> {
        if self.monitor.needs_mfg_info() {
            // Not connected: start with the manufacturer info
            self.mfg = self.bq.read_mfg_info().await?;
            if self.mfg.serial != self.board_serial as u16 {
                fault::raise(faults::BQ40_MISMATCHED_SERIAL, (self.board_serial as u32) << 16 | self.mfg.serial as u32);
            } else {
                fault::lower(faults::BQ40_MISMATCHED_SERIAL);
            }
        }
        self.bq.read_battery_info(&self.mfg, &mut self.info).await
    }

    async fn poll_succeeded(&mut self) {
        if !self.monitor.state().is_connected() {
            defmt::info!("Connected to gauge, serial {}", self.mfg.serial);
            fault::lower(faults::BQ40_NOT_CONNECTED);
        }

        if self.info.safety_status != 0 {
            fault::raise(faults::BQ40_SAFETY_STATUS, self.info.safety_status);
        } else {
            fault::lower(faults::BQ40_SAFETY_STATUS);
        }
        if self.info.pf_status != 0 {
            fault::raise(faults::BQ40_PF_STATUS, self.info.pf_status);
        } else {
            fault::lower(faults::BQ40_PF_STATUS);
        }

        // The SOC is garbage while the gauge initializes
        if self.info.initializing() {
            self.pins.clear_soc_leds();
        } else {
            self.pins.update_soc_leds(self.info.relative_soc);
        }

        match self.monitor.poll_succeeded(ms(Instant::now()), &self.info) {
            EmshutAction::None => {}
            EmshutAction::Exit => {
                let result = self.bq.emshut_exit().await;
                if let Err(err) = result {
                    fault::raise(faults::BQ40_COMMAND_FAIL, err.raw());
                }
                self.monitor.emshut_exit_result(result.is_ok());
            }
            EmshutAction::CommandTimedOut => fault::raise(faults::BQ40_COMMAND_FAIL, 0),
        }

        self.monitor.update_state(&self.info);
        self.publish();
    }

    /// Serves one request from core 0 (the FIFO half of `core1_main`).
    async fn handle(&mut self, request: Request) {
        match request {
            Request::SideDetect => {
                let reply = match self.monitor.check_side_detect() {
                    Ok(()) => match self.bq.read_side_detect().await {
                        Ok(high) => Some(high),
                        Err(err) => {
                            fault::raise(faults::BQ40_COMMAND_FAIL, err.raw());
                            None
                        }
                    },
                    // PACK+ isn't valid in this state
                    Err(arg) => {
                        fault::raise(faults::BQ40_COMMAND_FAIL, arg);
                        None
                    }
                };
                SIDE_DETECT.signal(reply);
            }
            Request::Status(register) => {
                let command = match register {
                    StatusRegister::Gauging => cmd::GAUGING_STATUS,
                    StatusRegister::Charging => cmd::CHARGING_STATUS,
                };
                let reply = match self.bq.read_status3(command).await {
                    Ok(v) => Some(v),
                    Err(err) => {
                        fault::raise(faults::BQ40_COMMAND_FAIL, err.raw());
                        None
                    }
                };
                STATUS_REPLY.signal(reply);
            }
            Request::Emshut(command) => match self.monitor.request_emshut(ms(Instant::now()), command) {
                EmshutRequest::Enter => {
                    defmt::info!("Entering emergency FET shutdown");
                    // C ignored the result too: a failure shows up as the EMSHUT command timeout
                    if let Err(err) = self.bq.emshut_enter().await {
                        defmt::warn!("Emergency shutdown command failed: {}", err);
                    }
                }
                EmshutRequest::Wait => {}
                EmshutRequest::Refused(arg) => fault::raise(faults::BQ40_COMMAND_FAIL, arg),
            },
            Request::Debug(op) => {
                let result = self.debug(op).await;
                DEBUG_REPLY.signal(result);
            }
        }
    }

    async fn debug(&mut self, op: DebugOp) -> DebugResult {
        let mut data = [0u8; BLOCK_LEN];
        let result = match op {
            DebugOp::SbsReadInt { cmd, width } => self.bq.dbg_read_sbs_int(cmd, width).await.map(DebugResult::Int),
            DebugOp::SbsReadBlock { cmd } => {
                self.bq.dbg_read_sbs_block(cmd, &mut data).await.map(|len| DebugResult::Block { data, len: len as u8 })
            }
            DebugOp::MfgReadBlock { cmd } => {
                self.bq.dbg_read_mfg_block(cmd, &mut data).await.map(|len| DebugResult::Block { data, len: len as u8 })
            }
            DebugOp::MfgCmd { cmd } => self.bq.dbg_mfg_cmd(cmd).await.map(|()| DebugResult::Ok),
            DebugOp::DfWrite { addr, data, len } => {
                self.bq.dbg_df_write(addr, &data[..len as usize]).await.map(|()| DebugResult::Ok)
            }
        };
        result.unwrap_or_else(DebugResult::Err)
    }
}

/// Runs the gauge forever on core 1 (`core1_main`).
#[embassy_executor::task]
pub async fn gauge_task(bq: Bq40z80, pins: Pins, board_serial: u8) -> ! {
    let mut gauge = Gauge {
        bq,
        pins,
        monitor: Monitor::new(),
        board_serial,
        mfg: MfgInfo::default(),
        info: BatteryInfo::default(),
    };
    gauge.pins.wake.set_low();
    gauge.pins.clear_soc_leds();

    let period = Duration::from_millis(monitor::REFRESH_PERIOD_MS);
    let mut next_refresh = Instant::now();
    loop {
        checkin();
        match select(Timer::at(next_refresh), REQUESTS.receive()).await {
            Either::First(()) => {
                // Like C, the next poll is scheduled from the start of this one
                next_refresh = Instant::now() + period;
                gauge.refresh().await;
            }
            Either::Second(request) => gauge.handle(request).await,
        }
    }
}
