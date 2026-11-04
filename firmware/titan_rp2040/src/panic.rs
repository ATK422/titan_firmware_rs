//! Panic handler: log the panic, record it in the crash log, and reset through the watchdog so the next boot
//! reports `FAULT_WATCHDOG_RESET`. This plays the role of the C `panic()` + crash log path.

use core::panic::PanicInfo;

use crate::crash;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    cortex_m::interrupt::disable();
    defmt::error!("PANIC: {}", defmt::Display2Format(info));

    let (file, line) = info.location().map_or(("", 0), |l| (l.file(), l.line()));
    crash::record_panic(file, line);

    // Give an attached probe a moment to drain the RTT buffer before the reset
    cortex_m::asm::delay(12_500_000);

    crash::reset_now()
}
