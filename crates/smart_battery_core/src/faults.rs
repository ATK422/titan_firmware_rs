//! Board fault IDs (`safety_interface.h`), after the standard faults 0-4 of `titan_rp2040::safety`.

/// The SHT41 temperature/humidity sensor failed repeatedly.
pub const SHT41_ERROR: u8 = 5;
/// The gauge stopped answering. Argument: the packed gauge error.
pub const BQ40_NOT_CONNECTED: u8 = 6;
/// The gauge reports a safety status. Argument: `SafetyStatus()`.
pub const BQ40_SAFETY_STATUS: u8 = 7;
/// The gauge reports a permanent failure. Argument: `PFStatus()`.
pub const BQ40_PF_STATUS: u8 = 8;
/// The gauge's serial number differs from the one programmed into this board. Argument: `board << 16 | gauge`.
pub const BQ40_MISMATCHED_SERIAL: u8 = 9;
/// A gauge command failed or was refused.
pub const BQ40_COMMAND_FAIL: u8 = 10;

/// Names of the board faults, indexed from [`SHT41_ERROR`].
pub const NAMES: [&str; 6] = [
    "FAULT_SHT41_ERROR",
    "FAULT_BQ40_NOT_CONNECTED",
    "FAULT_BQ40_SAFETY_STATUS",
    "FAULT_BQ40_PF_STATUS",
    "FAULT_BQ40_MISMATCHED_SERIAL",
    "FAULT_BQ40_COMMAND_FAIL",
];
