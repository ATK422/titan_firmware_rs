//! Board faults: `titan_rp2040::safety` plus a copy of the last argument of the gauge faults.
//!
//! The C display read `safety_fault_data[id].extra_data` for the "Err: 0x..." screens and the pack status screen.
//! `titan_rp2040::safety` keeps those arguments private, so this module remembers the ones the display needs.

use core::sync::atomic::{AtomicU32, Ordering};

use smart_battery_core::faults;
use titan_rp2040::safety;

/// Fault names for `BoardConfig::fault_names`.
pub static FAULT_NAMES: [&str; 11] = [
    safety::STANDARD_FAULT_NAMES[0],
    safety::STANDARD_FAULT_NAMES[1],
    safety::STANDARD_FAULT_NAMES[2],
    safety::STANDARD_FAULT_NAMES[3],
    safety::STANDARD_FAULT_NAMES[4],
    faults::NAMES[0],
    faults::NAMES[1],
    faults::NAMES[2],
    faults::NAMES[3],
    faults::NAMES[4],
    faults::NAMES[5],
];

const _: () = assert!(faults::SHT41_ERROR == safety::FIRST_BOARD_FAULT);

static NOT_CONNECTED_ARG: AtomicU32 = AtomicU32::new(0);
static SAFETY_STATUS_ARG: AtomicU32 = AtomicU32::new(0);
static PF_STATUS_ARG: AtomicU32 = AtomicU32::new(0);

/// Raises a fault (`safety_raise_fault_with_arg`).
#[track_caller]
pub fn raise(id: u8, arg: u32) {
    let slot = match id {
        faults::BQ40_NOT_CONNECTED => Some(&NOT_CONNECTED_ARG),
        faults::BQ40_SAFETY_STATUS => Some(&SAFETY_STATUS_ARG),
        faults::BQ40_PF_STATUS => Some(&PF_STATUS_ARG),
        _ => None,
    };
    if let Some(slot) = slot {
        slot.store(arg, Ordering::Relaxed);
    }
    safety::raise_fault(id, arg);
}

/// Lowers a fault.
pub fn lower(id: u8) {
    safety::lower_fault(id);
}

/// The arguments of the last `FAULT_BQ40_NOT_CONNECTED`, `FAULT_BQ40_SAFETY_STATUS` and `FAULT_BQ40_PF_STATUS`.
pub fn gauge_fault_args() -> (u32, u32, u32) {
    (
        NOT_CONNECTED_ARG.load(Ordering::Relaxed),
        SAFETY_STATUS_ARG.load(Ordering::Relaxed),
        PF_STATUS_ARG.load(Ordering::Relaxed),
    )
}
