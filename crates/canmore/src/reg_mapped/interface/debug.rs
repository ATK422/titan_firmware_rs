//! Debug interface (`reg_mapped/interface/debug.h`), served by applications in normal mode.
//!
//! ```text
//! 0x00: MCU control       (registers)
//! 0x01: Version string    RO memory
//! 0x03: GDB stub          (registers)
//! 0x04: Remote command    (registers)
//! 0x05: Remote cmd args   RW memory
//! 0x06: Remote cmd resp   RO memory
//! 0x08: Safety status     (registers)
//! 0x09: Crash log         (registers)
//! 0x0A: Fault name        RO memory
//! 0x0B: Fault data        (registers)
//! 0x0C: Fault filename    RO memory
//! ```

pub const MCU_CONTROL_PAGE: u8 = 0x00;
pub const VERSION_STRING_PAGE: u8 = 0x01;
pub const GDB_STUB_PAGE: u8 = 0x03;
pub const REMOTE_CMD_PAGE: u8 = 0x04;
pub const REMOTE_CMD_ARGS_PAGE: u8 = 0x05;
pub const REMOTE_CMD_RESP_PAGE: u8 = 0x06;
pub const SAFETY_STATUS_PAGE: u8 = 0x08;
pub const CRASH_LOG_PAGE: u8 = 0x09;
pub const FAULT_NAME_PAGE: u8 = 0x0A;
pub const FAULT_DATA_PAGE: u8 = 0x0B;
pub const FAULT_FILENAME_PAGE: u8 = 0x0C;

/// MCU control page.
pub mod mcu_control {
    /// RO: always [`MAGIC_VALUE`].
    pub const MAGIC: u8 = 0x00;
    pub const MAGIC_VALUE: u32 = 0x0DBAA1F0;
    /// WO: any write reboots into the bootloader.
    pub const ENTER_BL: u8 = 0x01;
    /// RO: lower 32 bits of the flash unique ID.
    pub const LOWER_FLASH_ID: u8 = 0x02;
    /// RO: upper 32 bits of the flash unique ID.
    pub const UPPER_FLASH_ID: u8 = 0x03;
    /// RO: firmware major version.
    pub const MAJOR_VERSION: u8 = 0x04;
    /// RO: firmware minor version.
    pub const MINOR_VERSION: u8 = 0x05;
    /// RO: firmware release type (see [`super::release_type`]).
    pub const RELEASE_TYPE: u8 = 0x06;
    /// WO: any write reboots the MCU.
    pub const REBOOT_MCU: u8 = 0x07;
}

/// Release type values reported in the version registers (`titan/version.h`).
pub mod release_type {
    pub const DEBUG: u32 = 0;
    pub const DEV: u32 = 1;
    pub const CLEAN: u32 = 2;
    pub const TAGGED: u32 = 3;
}

/// GDB stub page (same layout as the bootloader's).
pub mod gdb_stub {
    pub use crate::reg_mapped::interface::bootloader::gdb_stub::*;
}

/// Remote command page.
///
/// Write `"cmd\0arg1\0arg2\0\0"` to the args page, read [`EXECUTE`](remote_cmd::EXECUTE) to run it (the register
/// returns the command's return code as an `i32`), then read the response string page. The device clears the args
/// page after executing so a repeated read doesn't run the command twice.
pub mod remote_cmd {
    /// RO: executes the command in the args page and returns its exit code.
    pub const EXECUTE: u8 = 0x00;
}

/// Maximum length of the encoded remote command arguments.
pub const REMOTE_CMD_ARGS_MAX_LEN: usize = crate::reg_mapped::PAGE_SIZE;
/// Maximum length of the remote command response.
pub const REMOTE_CMD_RESP_MAX_LEN: usize = crate::reg_mapped::PAGE_SIZE;

/// Safety status page.
pub mod safety_status {
    /// RO: global safety state bits (see the `GLOBAL_STATE_*` bit numbers).
    pub const GLOBAL_STATE: u8 = 0x00;
    pub const GLOBAL_STATE_IS_SETUP_BIT: u32 = 0;
    pub const GLOBAL_STATE_IS_INITIALIZED_BIT: u32 = 1;
    pub const GLOBAL_STATE_KILL_IS_ENABLED_BIT: u32 = 2;
    pub const GLOBAL_STATE_FAULT_PRESENT_BIT: u32 = 3;
    /// RO: bit mask of active faults.
    pub const FAULT_LIST: u8 = 0x01;
    /// RO: uptime in [`UPTIME_TICKS_PER_SECOND`] ticks.
    pub const UPTIME: u8 = 0x02;
    pub const UPTIME_TICKS_PER_SECOND: u32 = 100;
    /// RW: fault selected for the fault name, data and filename pages.
    pub const FAULT_IDX: u8 = 0x03;
    /// WO: raises the written fault ID.
    pub const RAISE_FAULT: u8 = 0x04;
    /// WO: lowers the written fault ID.
    pub const LOWER_FAULT: u8 = 0x05;
}

/// Crash log page.
pub mod crash_log {
    /// RO: crash counters since the last clean boot (total, panic, hard fault, assert fail; one byte each from LSB).
    pub const CRASH_COUNT: u8 = 0x00;
    pub const CRASH_COUNT_TOTAL_SHIFT: u32 = 0;
    pub const CRASH_COUNT_PANIC_SHIFT: u32 = 8;
    pub const CRASH_COUNT_HARD_FAULT_SHIFT: u32 = 16;
    pub const CRASH_COUNT_ASSERT_FAIL_SHIFT: u32 = 24;
    /// RO: 1 if the log wrapped and lost entries.
    pub const HAS_WRAPPED: u8 = 0x01;
    /// RO: number of entries in the log.
    pub const PREV_COUNT: u8 = 0x02;
    /// RW: selects the entry shown in the `PREV_*` / `RESET_REASON` registers (0 = most recent).
    pub const PREV_INDEX: u8 = 0x03;
    /// RO: reset reason of the selected entry (see the `RESET_REASON_*` values).
    pub const RESET_REASON: u8 = 0x04;
    pub const RESET_REASON_CLEAN_BOOT: u32 = 0;
    pub const RESET_REASON_UNKNOWN_PREINIT: u32 = 1;
    pub const RESET_REASON_UNKNOWN_ACTIVE: u32 = 2;
    pub const RESET_REASON_PANIC: u32 = 3;
    pub const RESET_REASON_HARD_FAULT: u32 = 4;
    pub const RESET_REASON_ASSERT_FAIL: u32 = 5;
    pub const RESET_REASON_HARD_ASSERT: u32 = 7;
    pub const RESET_REASON_WATCHDOG_TIMEOUT: u32 = 8;
    pub const RESET_REASON_CORE1_TIMEOUT: u32 = 9;
    /// RO: fault list of the selected entry.
    pub const PREV_FAULT_LIST: u8 = 0x05;
    /// RO: uptime of the selected entry.
    pub const PREV_UPTIME: u8 = 0x06;
    /// RO: scratch 1 of the selected entry (the clean boot type for clean boots, see `CLEAN_RESET_*`).
    pub const PREV_SCRATCH_1: u8 = 0x07;
    pub const CLEAN_RESET_POR: u32 = 1;
    pub const CLEAN_RESET_RUN: u32 = 2;
    pub const CLEAN_RESET_PSM: u32 = 3;
    pub const CLEAN_RESET_SOFTWARE: u32 = 4;
    pub const CLEAN_RESET_UNK_WDG: u32 = 5;
    pub const CLEAN_RESET_BOOTLOADER: u32 = 6;
    pub const CLEAN_RESET_BOOTROM: u32 = 7;
    /// RO: scratch 2 of the selected entry.
    pub const PREV_SCRATCH_2: u8 = 0x08;
}

/// Fault data page, for the fault selected by [`safety_status::FAULT_IDX`].
pub mod fault_data {
    /// RO: non-zero if the fault was ever raised. The other registers are only valid if set.
    pub const STICKY: u8 = 0x00;
    /// RO: lower 32 bits of the raise timestamp (µs).
    pub const TIME_LOWER: u8 = 0x01;
    /// RO: upper 32 bits of the raise timestamp (µs).
    pub const TIME_UPPER: u8 = 0x02;
    /// RW: extra data passed when raising the fault.
    pub const EXTRA_DATA: u8 = 0x03;
    /// RO: source line that raised the fault.
    pub const LINE_NUMBER: u8 = 0x04;
    /// RO: non-zero if the fault was raised more than once.
    pub const MULTIPLE_FIRES: u8 = 0x05;
}
