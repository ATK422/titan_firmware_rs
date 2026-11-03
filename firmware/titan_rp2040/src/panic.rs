//! Panic handler: leave a marker for the crash record, log the panic, and reset through the watchdog so the next
//! boot reports `FAULT_WATCHDOG_RESET` and counts a panic. This plays the role of the C `panic()` + crash log path.
//!
//! The order matters. Interrupts are off, so nothing feeds the watchdog any more, and once ROS has connected its
//! timeout is only 250 ms. The handler therefore marks scratch3 first, so whatever resets the chip from then on
//! (its own forced reset, or the watchdog if the handler stalls) is reported as a panic by
//! [`crate::safety::setup`]. Then it reloads the watchdog with [`PANIC_WATCHDOG_TIMEOUT_MS`], so the running
//! timeout can't expire during the RTT drain delay, while the watchdog still resets the chip if logging hangs.

use core::panic::PanicInfo;

use embassy_rp::pac;

/// How long the handler gives an attached probe to drain the RTT buffer before the reset.
const RTT_DRAIN_MS: u32 = 100;
/// Watchdog timeout while the handler runs. Longer than the drain delay plus logging, so the handler's own reset
/// normally wins; short enough to keep a stalled handler from leaving the board hung.
const PANIC_WATCHDOG_TIMEOUT_MS: u32 = 300;
/// Cycles per iteration of `cortex_m::asm::delay`'s `subs`/`bne` loop on the Cortex-M0+.
const DELAY_LOOP_CYCLES: u32 = 3;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    cortex_m::interrupt::disable();
    pac::WATCHDOG.scratch3().write_value(crate::safety::PANIC_MARKER);
    reload_watchdog(PANIC_WATCHDOG_TIMEOUT_MS);

    defmt::error!("PANIC: {}", defmt::Display2Format(info));

    // `delay` takes loop iterations, not cycles. Before the clocks are set up the frequency reads as 0 and there
    // is no probe output worth waiting for anyway.
    let cycles_per_ms = embassy_rp::clocks::clk_sys_freq() / 1000;
    cortex_m::asm::delay(cycles_per_ms * RTT_DRAIN_MS / DELAY_LOOP_CYCLES);

    pac::WATCHDOG.ctrl().write(|w| w.set_trigger(true));
    loop {
        cortex_m::asm::nop();
    }
}

/// Restarts the watchdog countdown at `timeout_ms` and makes sure it is running and resets the chip.
fn reload_watchdog(timeout_ms: u32) {
    // Select what the watchdog resets (everything but the oscillators, as `Watchdog::start` does), in case the
    // panic came before `safety::setup` started it
    pac::PSM.wdsel().write_value(pac::psm::regs::Wdsel(0x0001_ffff & !0b11));
    // RP2040-E1: the counter decrements twice per tick, so load twice the timeout in microseconds (as
    // `Watchdog::feed` does)
    pac::WATCHDOG.load().write_value(pac::watchdog::regs::Load(timeout_ms * 1000 * 2));
    pac::WATCHDOG.ctrl().modify(|w| w.set_enable(true));
}
