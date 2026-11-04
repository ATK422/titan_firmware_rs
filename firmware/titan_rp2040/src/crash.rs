//! Crash reporting, ported from `crash_reporting.c` and the handlers in `safety_helper.S`.
//!
//! The protocol and the log itself are [`titan_control::crash_log`]; this module is the hardware around it:
//!
//! * the crash log, the uptime count and a mirror of the fault list live in `.uninit` RAM, which neither the
//!   runtime nor the C bootloader (it only uses the scratch banks and XIP RAM) touches across a reset;
//! * SysTick counts the uptime in 10 ms ticks, so it keeps counting while the executor is stuck;
//! * crashes write their reason to watchdog scratch 0-2 before the chip resets: the panic handler (`PANIC`), the
//!   HardFault handler (`HARD_FAULT`, faulting PC), an NMI that TIMER alarm 3 raises 1 µs before the watchdog fires
//!   (`WATCHDOG_TIMEOUT`, interrupted PC) and [`core1_timeout`] (`CORE1_TIMEOUT`). Only the first one is recorded,
//!   claimed through SIO spinlock 29 like `SAFETY_CRASH_LOGGED_SPINLOCK_ID`;
//! * [`reboot`] and [`enter_bootloader`] mark the reset as clean (`watchdog_reboot`, `safety_enter_bootloader`).

use core::cell::UnsafeCell;
use core::fmt::Write;
use core::mem::MaybeUninit;

use cortex_m::peripheral::SYST;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{ExceptionFrame, exception};
use embassy_rp::pac;
use titan_control::crash_log::{
    self, ChipReset, CrashData, ResetInputs, ResetOutcome, UPTIME_TICK_XOR_MAGIC, UPTIME_TICKS_PER_SECOND,
    WATCHDOG_NON_REBOOT_MAGIC, clean_reset, reason,
};

/// SIO spinlock claimed by the first crash handler (`PICO_SPINLOCK_ID_OS1`).
const CRASH_LOGGED_SPINLOCK: usize = 29;
/// TIMER alarm raising the NMI just before the watchdog fires (`SAFETY_WATCHDOG_ALARM_NUM`). embassy-time uses
/// alarm 0.
const WATCHDOG_ALARM: usize = 3;
/// Scratch 4 value that makes the bootloader stay in bootloader mode (`safety_enter_bootloader`).
const BOOTLOADER_MAGIC: u32 = titan_control::bootloader::BOOTLOADER_MAGIC;
/// Every PSM block but the oscillators (`PSM_WDSEL_BITS & ~(ROSC | XOSC)`).
const WDSEL_ALL_BUT_OSCILLATORS: u32 = 0x1_FFFF & !0b11;

/// RAM that survives a reset, accessed only through volatile reads and writes of whole words.
struct Persistent<T>(UnsafeCell<MaybeUninit<T>>);

// SAFETY: written by `setup` before anything else runs, by the SysTick handler (uptime), and by `mirror_faults`
// inside a critical section; every access is a volatile word access.
unsafe impl<T> Sync for Persistent<T> {}

impl<T> Persistent<T> {
    const fn new() -> Self {
        Persistent(UnsafeCell::new(MaybeUninit::uninit()))
    }

    fn word(&self, i: usize) -> *mut u32 {
        debug_assert!(i < size_of::<T>() / 4);
        // In bounds for the word counts used below
        unsafe { (self.0.get() as *mut u32).add(i) }
    }

    fn read(&self, i: usize) -> u32 {
        // SAFETY: in bounds and aligned; volatile so nothing is assumed about RAM that may never have been written
        unsafe { self.word(i).read_volatile() }
    }

    fn write(&self, i: usize, value: u32) {
        // SAFETY: as in `read`
        unsafe { self.word(i).write_volatile(value) }
    }
}

#[unsafe(link_section = ".uninit.titan_crash_data")]
static CRASH_DATA: Persistent<CrashData> = Persistent::new();
/// `[ticks, ticks ^ UPTIME_TICK_XOR_MAGIC]` (`safety_uptime_ticks`, `safety_uptime_ticks_xor`).
#[unsafe(link_section = ".uninit.titan_uptime")]
static UPTIME: Persistent<[u32; 2]> = Persistent::new();
/// The fault list at the moment of a reset (`fault_list`).
#[unsafe(link_section = ".uninit.titan_fault_list")]
static FAULT_LIST: Persistent<u32> = Persistent::new();

const CRASH_DATA_WORDS: usize = size_of::<CrashData>() / 4;

fn load_crash_data() -> CrashData {
    let mut data = MaybeUninit::<CrashData>::uninit();
    let words = data.as_mut_ptr() as *mut u32;
    for i in 0..CRASH_DATA_WORDS {
        // SAFETY: `CrashData` is plain `u32`s, so any bit pattern is valid
        unsafe { words.add(i).write(CRASH_DATA.read(i)) };
    }
    // SAFETY: every word was written above
    unsafe { data.assume_init() }
}

fn store_crash_data(data: &CrashData) {
    let words = data as *const CrashData as *const u32;
    for i in 0..CRASH_DATA_WORDS {
        // SAFETY: in bounds of `data`
        CRASH_DATA.write(i, unsafe { words.add(i).read() });
    }
}

/// The crash log as recorded at boot.
pub fn crash_data() -> CrashData {
    load_crash_data()
}

fn scratch(i: usize) -> pac::common::Reg<u32, pac::common::RW> {
    match i {
        0 => pac::WATCHDOG.scratch0(),
        1 => pac::WATCHDOG.scratch1(),
        2 => pac::WATCHDOG.scratch2(),
        _ => pac::WATCHDOG.scratch4(),
    }
}

/// Sets what the firmware is doing in scratch 0, reported if it dies (`*reset_reason_reg = ...`).
pub(crate) fn set_reason(value: u32) {
    pac::WATCHDOG.scratch0().write_value(value);
}

/// Keeps the fault list that a crash record will report up to date.
pub(crate) fn mirror_faults(faults: u32) {
    FAULT_LIST.write(0, faults);
}

/// Records the previous reset, starts the uptime count and installs the watchdog NMI. Returns what was found so
/// safety can raise `FAULT_WATCHDOG_RESET` (`safety_internal_crash_reporting_handle_reset`). Call before the
/// watchdog is started.
pub(crate) fn setup() -> ResetOutcome {
    let watchdog_reason = pac::WATCHDOG.reason().read();
    let chip_reset = pac::VREG_AND_CHIP_RESET.chip_reset().read();
    let (ticks, check) = (UPTIME.read(0), UPTIME.read(1));
    let inputs = ResetInputs {
        watchdog_reason: watchdog_reason.timer() || watchdog_reason.force(),
        scratch: [scratch(0).read(), scratch(1).read(), scratch(2).read()],
        scratch_4: scratch(4).read(),
        uptime: (ticks == check ^ UPTIME_TICK_XOR_MAGIC).then_some(ticks),
        faults: FAULT_LIST.read(0),
        chip_reset: ChipReset {
            had_por: chip_reset.had_por(),
            had_run: chip_reset.had_run(),
            had_psm_restart: chip_reset.had_psm_restart(),
        },
    };

    let mut data = load_crash_data();
    let outcome = crash_log::process_reset(&mut data, &inputs);
    store_crash_data(&data);

    // Clear the state of this session
    UPTIME.write(0, 1);
    UPTIME.write(1, 1 ^ UPTIME_TICK_XOR_MAGIC);
    mirror_faults(0);
    if inputs.scratch[0] == reason::CLEAN_BOOT {
        scratch(1).write_value(0);
    }

    if outcome.log_corrupted {
        defmt::error!("Crash log corrupted");
    }
    if outcome.clean_boot_on_crash {
        defmt::error!("Clean boot specified in reset_reason register, but the watchdoog has reported a reset");
    }
    let mut message = heapless::String::<256>::new();
    match crash_log::format_last_reset(&mut message, &data) {
        // A message cut short by the buffer is still worth printing
        Some(_) => defmt::info!("{=str}", message.as_str()),
        None => defmt::warn!("No reset data found"),
    }

    // Free the crash claim (the SIO resets with the chip, but a debugger reset may not)
    pac::SIO.spinlock(CRASH_LOGGED_SPINLOCK).write_value(1);

    // SAFETY: SysTick and the alarm are used by nothing else; the handlers below are installed by the linker
    let mut syst = unsafe { cortex_m::Peripherals::steal() }.SYST;
    start_uptime(&mut syst);
    install_watchdog_nmi();

    set_reason(reason::UNKNOWN_SAFETY_PREINIT);
    outcome
}

fn start_uptime(syst: &mut SYST) {
    // The external reference is the 1 µs watchdog tick
    syst.set_clock_source(SystClkSource::External);
    syst.set_reload(1_000_000 / UPTIME_TICKS_PER_SECOND - 1);
    syst.clear_current();
    syst.enable_interrupt();
    syst.enable_counter();
}

#[exception]
fn SysTick() {
    let ticks = UPTIME.read(0).wrapping_add(1);
    UPTIME.write(0, ticks);
    UPTIME.write(1, ticks ^ UPTIME_TICK_XOR_MAGIC);
}

/// Writes scratch 4 as `watchdog_enable` does, so a watchdog reset reads as a crash on the next boot.
pub(crate) fn mark_watchdog_enabled() {
    pac::WATCHDOG.scratch4().write_value(WATCHDOG_NON_REBOOT_MAGIC);
}

fn install_watchdog_nmi() {
    let bit = 1 << WATCHDOG_ALARM;
    pac::TIMER.armed().write(|w| w.0 = bit);
    pac::TIMER.intr().write(|w| w.set_alarm(WATCHDOG_ALARM, true));
    pac::TIMER.inte().modify(|w| w.set_alarm(WATCHDOG_ALARM, true));
    // TIMER_IRQ_n is IRQ n. It isn't enabled in the NVIC, so it only reaches the core as an NMI
    pac::SYSCFG.proc0_nmi_mask().write_value(bit);
}

/// Arms the NMI one tick before a watchdog fed now with `timeout_us` fires (`safety_tick`). Call right before
/// feeding the watchdog.
pub(crate) fn arm_watchdog_nmi(timeout_us: u32) {
    let now = pac::TIMER.timerawl().read();
    pac::TIMER.alarm(WATCHDOG_ALARM).write_value(now.wrapping_add(timeout_us).wrapping_sub(1));
}

fn disarm_watchdog_nmi() {
    pac::TIMER.armed().write(|w| w.0 = 1 << WATCHDOG_ALARM);
}

/// Claims the right to record a crash. Only the first crash of a run is recorded.
fn claim_crash() -> bool {
    pac::SIO.spinlock(CRASH_LOGGED_SPINLOCK).read() != 0
}

fn record(reason: u32, scratch_1: u32, scratch_2: u32) {
    if claim_crash() {
        set_reason(reason);
        pac::WATCHDOG.scratch1().write_value(scratch_1);
        pac::WATCHDOG.scratch2().write_value(scratch_2);
    }
}

/// Records a panic. Rust has no message address or caller to store as C does, so scratch 1 holds the address of
/// the panicking file's name and scratch 2 the line.
pub(crate) fn record_panic(file: &'static str, line: u32) {
    record(reason::PANIC, file.as_ptr() as u32, line);
}

/// Resets the chip through the watchdog, keeping whatever was recorded. The next boot reads it as a crash.
pub(crate) fn reset_now() -> ! {
    pac::WATCHDOG.ctrl().write(|w| w.set_trigger(true));
    loop {
        cortex_m::asm::nop();
    }
}

/// Core 1 stopped checking in: records `CORE1_TIMEOUT` and resets (`safety_internal_multicore_tick`). C stores
/// the instruction core 1 was at, which takes an NMI on core 1; here scratch 1 holds the milliseconds since its
/// last check-in instead.
pub fn core1_timeout(ms_since_checkin: u32) -> ! {
    cortex_m::interrupt::disable();
    defmt::error!("Core 1 did not check in for {} ms", ms_since_checkin);
    record(reason::CORE1_TIMEOUT, ms_since_checkin, 0);
    reset_now()
}

#[exception]
unsafe fn HardFault(frame: &ExceptionFrame) -> ! {
    // The stacked PC points after the faulting instruction's start; C subtracts 2 the same way
    record(reason::HARD_FAULT, frame.pc().wrapping_sub(2), 0);
    // C stops at a breakpoint and lets the watchdog fire; reset now so there's no lockup without a debugger
    reset_now()
}

// The watchdog NMI (`safety_nmi_handler`). It runs from RAM: it has 1 µs before the watchdog resets the chip, too
// little for an XIP cache miss. The thread executor runs on the main stack, so the interrupted PC is at MSP + 0x18.
core::arch::global_asm!(
    ".section .data.titan_nmi_handler, \"ax\"",
    ".global NonMaskableInt",
    ".type NonMaskableInt,%function",
    ".thumb_func",
    "NonMaskableInt:",
    "    ldr r0, [sp, #0x18]",
    "    ldr r1, ={record}",
    "    bx r1",
    ".ltorg",
    record = sym record_watchdog_timeout,
);

#[unsafe(link_section = ".data.titan_nmi_record")]
#[inline(never)]
extern "C" fn record_watchdog_timeout(pc: u32) {
    // Only raw register writes here: anything else may live in flash
    let spinlock = (pac::SIO.as_ptr() as *mut u8).wrapping_add(0x100 + 4 * CRASH_LOGGED_SPINLOCK) as *mut u32;
    let scratch = pac::WATCHDOG.as_ptr() as *mut u32;
    // SAFETY: SIO spinlock and WATCHDOG scratch 0/1 registers
    unsafe {
        if spinlock.read_volatile() != 0 {
            scratch.add(3).write_volatile(reason::WATCHDOG_TIMEOUT);
            scratch.add(4).write_volatile(pc.wrapping_sub(2));
        }
    }
}

/// Reboots cleanly (`watchdog_reboot(0, 0, 0)` through `__wrap_watchdog_reboot`).
pub fn reboot() -> ! {
    cortex_m::interrupt::disable();
    disarm_watchdog_nmi();
    set_reason(reason::CLEAN_BOOT);
    pac::WATCHDOG.scratch1().write_value(clean_reset::SOFTWARE);
    pac::WATCHDOG.scratch4().write_value(0);
    pac::WATCHDOG.ctrl().modify(|w| w.set_enable(false));
    pac::PSM.wdsel().write(|w| w.0 = WDSEL_ALL_BUT_OSCILLATORS);
    reset_now()
}

/// Reboots into the bootloader (`safety_enter_bootloader`).
pub fn enter_bootloader() -> ! {
    cortex_m::interrupt::disable();
    disarm_watchdog_nmi();
    set_reason(reason::CLEAN_BOOT);
    pac::WATCHDOG.scratch1().write_value(clean_reset::BOOTLOADER);
    pac::WATCHDOG.scratch4().write_value(BOOTLOADER_MAGIC);
    pac::WATCHDOG.ctrl().modify(|w| w.set_enable(false));
    pac::PSM.wdsel().write(|w| w.0 = WDSEL_ALL_BUT_OSCILLATORS);
    reset_now()
}

/// Logs the whole crash log (`safety_print_crash_log`).
pub fn log_crash_log() {
    let data = load_crash_data();
    let c = data.counts();
    let mut line = heapless::String::<256>::new();
    let _ = write!(line, "Total Crashes: {}", c.total);
    if c.panic != 0 {
        let _ = write!(line, " - Panics: {}", c.panic);
    }
    if c.hard_fault != 0 {
        let _ = write!(line, " - Hard Faults: {}", c.hard_fault);
    }
    if c.assert_fail != 0 {
        let _ = write!(line, " - Assert Fails: {}", c.assert_fail);
    }
    defmt::info!("===========Watchdog Crash Log===========");
    defmt::info!("{=str}", line.as_str());
    for i in 0..data.len() {
        let Some(entry) = data.entry(i) else { break };
        line.clear();
        let _ = write!(line, "Reset # {:3}: ", -(i as i32 + 1));
        let _ = crash_log::format_entry(&mut line, &entry);
        defmt::info!("{=str}", line.as_str());
    }
    defmt::info!("========================================");
}
