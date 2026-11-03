//! Panic handler: log the panic, leave a marker for the crash record, and reset through the watchdog so the next
//! boot reports `FAULT_WATCHDOG_RESET`. This plays the role of the C `panic()` + crash log path.

use core::panic::PanicInfo;

use embassy_rp::pac;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    cortex_m::interrupt::disable();
    defmt::error!("PANIC: {}", defmt::Display2Format(info));

    pac::WATCHDOG.scratch3().write_value(crate::safety::PANIC_MARKER);

    // Give an attached probe a moment to drain the RTT buffer before the reset
    cortex_m::asm::delay(12_500_000);

    pac::WATCHDOG.ctrl().write(|w| w.set_trigger(true));
    loop {
        cortex_m::asm::nop();
    }
}
