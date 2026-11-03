//! Client for the debug interface of applications in normal mode, ported from `canmore_dbg_cpp/DebugClient.cpp`.

use std::collections::HashMap;
use std::fmt::Write as _;

use anyhow::{Result, bail};
use canmore::ControlInterfaceMode;
use canmore::reg_mapped::interface::debug::{self as itf, crash_log, fault_data, gdb_stub, mcu_control, safety_status};

use super::gdb::GdbTarget;
use super::{RegClient, flash_id_from_words};

/// The global safety state bits.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SafetyStatus {
    pub safety_setup: bool,
    pub safety_initialized: bool,
    pub kill_is_enabled: bool,
    pub fault_present: bool,
}

impl SafetyStatus {
    pub fn from_register(value: u32) -> Self {
        let bit = |b: u32| value & (1 << b) != 0;
        SafetyStatus {
            safety_setup: bit(safety_status::GLOBAL_STATE_IS_SETUP_BIT),
            safety_initialized: bit(safety_status::GLOBAL_STATE_IS_INITIALIZED_BIT),
            kill_is_enabled: bit(safety_status::GLOBAL_STATE_KILL_IS_ENABLED_BIT),
            fault_present: bit(safety_status::GLOBAL_STATE_FAULT_PRESENT_BIT),
        }
    }

    pub fn global_state(&self) -> &'static str {
        match (self.kill_is_enabled, self.safety_initialized, self.safety_setup) {
            (true, true, true) => "Robot Enabled",
            (true, _, _) => "Invalid? Kill Enabled",
            (false, true, true) => "ROS Connected",
            (false, true, false) => "Invalid? Safety initialized",
            (false, false, _) => "Disconnected - Idle",
        }
    }
}

/// Uptime in centiseconds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Uptime(pub u32);

impl Uptime {
    /// "1 days 2 hours 3 minutes 4 seconds", leaving out leading zero units.
    pub fn format(&self) -> String {
        let mut seconds = self.0 / safety_status::UPTIME_TICKS_PER_SECOND;
        let days = seconds / (3600 * 24);
        seconds %= 3600 * 24;
        let hours = seconds / 3600;
        seconds %= 3600;
        let minutes = seconds / 60;
        seconds %= 60;
        let mut out = String::new();
        let mut shown = false;
        for (value, unit) in [(days, "days"), (hours, "hours"), (minutes, "minutes")] {
            if value > 0 || shown {
                let _ = write!(out, "{value} {unit} ");
                shown = true;
            }
        }
        let _ = write!(out, "{seconds} seconds");
        out
    }

    /// "123.45"
    pub fn format_seconds(&self) -> String {
        format!("{}.{:02}", self.0 / 100, self.0 % 100)
    }
}

/// Crash counters since the last clean boot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CrashCounter {
    pub total: u8,
    pub panic: u8,
    pub hard_fault: u8,
    pub assert_fail: u8,
}

impl CrashCounter {
    pub fn from_register(value: u32) -> Self {
        let byte = |shift: u32| (value >> shift) as u8;
        CrashCounter {
            total: byte(crash_log::CRASH_COUNT_TOTAL_SHIFT),
            panic: byte(crash_log::CRASH_COUNT_PANIC_SHIFT),
            hard_fault: byte(crash_log::CRASH_COUNT_HARD_FAULT_SHIFT),
            assert_fail: byte(crash_log::CRASH_COUNT_ASSERT_FAIL_SHIFT),
        }
    }

    pub fn format(&self) -> String {
        format!(
            "Total Crashes: {} - Panics: {} - Hard Faults: {} - Assert Fails: {}",
            self.total, self.panic, self.hard_fault, self.assert_fail
        )
    }
}

/// One entry of the crash log.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CrashLogEntry {
    pub reset_reason: u32,
    pub faults: u32,
    pub uptime: u32,
    pub scratch1: u32,
    pub scratch2: u32,
}

impl CrashLogEntry {
    pub fn format(&self) -> String {
        let (s1, s2) = (self.scratch1, self.scratch2);
        let mut out = if self.reset_reason == crash_log::RESET_REASON_CLEAN_BOOT {
            let kind = match s1 {
                crash_log::CLEAN_RESET_POR => "Power-on Reset".to_owned(),
                crash_log::CLEAN_RESET_RUN => "RESET Pin".to_owned(),
                crash_log::CLEAN_RESET_PSM => "PSM Debug Request".to_owned(),
                crash_log::CLEAN_RESET_SOFTWARE => "Software Request".to_owned(),
                crash_log::CLEAN_RESET_UNK_WDG => "Unknown Watchdog Request".to_owned(),
                crash_log::CLEAN_RESET_BOOTLOADER => "Bootloader".to_owned(),
                crash_log::CLEAN_RESET_BOOTROM => "Bootrom USB Upload".to_owned(),
                other => format!("Unknown Clean Boot Type (0x{other:08x})"),
            };
            format!("Clean Boot: {kind}")
        } else {
            let reason = match self.reset_reason {
                crash_log::RESET_REASON_UNKNOWN_PREINIT => "Unexpected Reset before Init".to_owned(),
                crash_log::RESET_REASON_UNKNOWN_ACTIVE => "Unexpected Reset while Active".to_owned(),
                crash_log::RESET_REASON_PANIC => format!("Panic (Message: 0x{s1:08x} Call Address: 0x{s2:08x})"),
                crash_log::RESET_REASON_HARD_FAULT => format!("Hard Fault (Fault Address: 0x{s1:08x})"),
                crash_log::RESET_REASON_ASSERT_FAIL => format!("Assertion Fail (File: 0x{s1:08x} Line: 0x{s2:08x})"),
                crash_log::RESET_REASON_HARD_ASSERT => format!("Hard Assert Fail (Fault Address: 0x{s1:08x})"),
                crash_log::RESET_REASON_WATCHDOG_TIMEOUT => format!("Watchdog Timeout (Last Address: 0x{s1:08x})"),
                crash_log::RESET_REASON_CORE1_TIMEOUT => format!("Core 1 Timeout (Last Address: 0x{s1:08x})"),
                other => format!("Unknown Reset Reason (0x{other:08x})"),
            };
            format!("{reason} - Faults: 0x{:08x}", self.faults)
        };
        if self.uptime != 0 {
            let ticks = safety_status::UPTIME_TICKS_PER_SECOND as f32;
            let (value, unit) = if self.uptime > 3600 * 100 {
                (self.uptime as f32 / (3600.0 * ticks), 'h')
            } else if self.uptime > 60 * 100 {
                (self.uptime as f32 / (60.0 * ticks), 'm')
            } else {
                (self.uptime as f32 / ticks, 's')
            };
            let _ = write!(out, " - Prev. Uptime: {value:.2}{unit}");
        }
        out
    }
}

/// Recorded data of one safety fault.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FaultData {
    pub fault_id: u32,
    pub name: String,
    pub sticky: bool,
    pub multiple_fires: bool,
    /// Raise time in microseconds since boot.
    pub timestamp: u64,
    pub extra_data: u32,
    pub filename: String,
    pub line: u16,
}

impl FaultData {
    /// "h:mm:ss.uuuuuu", leaving out zero hours and minutes.
    pub fn format_timestamp(&self) -> String {
        let mut seconds = self.timestamp / 1_000_000;
        let micros = self.timestamp % 1_000_000;
        let mut out = String::new();
        let mut width = 0;
        if seconds > 3600 {
            let _ = write!(out, "{}:", seconds / 3600);
            seconds %= 3600;
            width = 2;
        }
        let minutes = seconds / 60;
        seconds %= 60;
        if minutes > 0 || width > 0 {
            let _ = write!(out, "{minutes:0width$}:");
            width = 2;
        }
        let _ = write!(out, "{seconds:0width$}.{micros:06}");
        out
    }
}

/// Client for the debug interface ([`ControlInterfaceMode::Normal`]).
pub struct DebugClient {
    client: RegClient,
    flash_id: u64,
    fault_names: HashMap<u32, String>,
}

impl DebugClient {
    /// Connects, checking the interface magic and caching the flash ID.
    pub fn new(mut client: RegClient) -> Result<Self> {
        let magic = client.read(itf::MCU_CONTROL_PAGE, mcu_control::MAGIC)?;
        if magic != mcu_control::MAGIC_VALUE {
            bail!("Unexpected application magic 0x{magic:08x}");
        }
        let lower = client.read(itf::MCU_CONTROL_PAGE, mcu_control::LOWER_FLASH_ID)?;
        let upper = client.read(itf::MCU_CONTROL_PAGE, mcu_control::UPPER_FLASH_ID)?;
        Ok(DebugClient { client, flash_id: flash_id_from_words(lower, upper), fault_names: HashMap::new() })
    }

    pub fn open(link: &super::Link, client_id: u8) -> Result<Self> {
        Self::new(RegClient::open(link, client_id, ControlInterfaceMode::Normal)?)
    }

    pub fn flash_id(&self) -> u64 {
        self.flash_id
    }

    pub fn version(&mut self) -> Result<String> {
        self.client.read_string_page(itf::VERSION_STRING_PAGE)
    }

    pub fn enter_bootloader(&mut self) -> Result<()> {
        self.client.write(itf::MCU_CONTROL_PAGE, mcu_control::ENTER_BL, 1)
    }

    pub fn reboot(&mut self) -> Result<()> {
        self.client.write(itf::MCU_CONTROL_PAGE, mcu_control::REBOOT_MCU, 1)
    }

    /// Runs a remote command (`args[0]` is the command), returning its exit code and output.
    pub fn execute_remote_cmd<S: AsRef<str>>(&mut self, args: &[S]) -> Result<(i32, String)> {
        let mut encoded = Vec::new();
        for arg in args {
            encoded.extend_from_slice(arg.as_ref().as_bytes());
            encoded.push(0);
        }
        encoded.push(0);
        // write_string_page adds one more NUL as terminator
        if encoded.len() + 1 > itf::REMOTE_CMD_ARGS_MAX_LEN {
            bail!("Remote command arguments are too large for remote command register on device");
        }
        self.client.write_string_page(itf::REMOTE_CMD_ARGS_PAGE, &encoded)?;
        let rc = self.client.read(itf::REMOTE_CMD_PAGE, itf::remote_cmd::EXECUTE)? as i32;
        let response = self.client.read_string_page(itf::REMOTE_CMD_RESP_PAGE)?;
        Ok((rc, response))
    }

    pub fn safety_status(&mut self) -> Result<SafetyStatus> {
        Ok(SafetyStatus::from_register(self.client.read(itf::SAFETY_STATUS_PAGE, safety_status::GLOBAL_STATE)?))
    }

    pub fn uptime(&mut self) -> Result<Uptime> {
        Ok(Uptime(self.client.read(itf::SAFETY_STATUS_PAGE, safety_status::UPTIME)?))
    }

    pub fn active_faults(&mut self) -> Result<u32> {
        self.client.read(itf::SAFETY_STATUS_PAGE, safety_status::FAULT_LIST)
    }

    pub fn raise_fault(&mut self, fault_id: u32) -> Result<()> {
        self.client.write(itf::SAFETY_STATUS_PAGE, safety_status::RAISE_FAULT, fault_id)
    }

    pub fn lower_fault(&mut self, fault_id: u32) -> Result<()> {
        self.client.write(itf::SAFETY_STATUS_PAGE, safety_status::LOWER_FAULT, fault_id)
    }

    pub fn crash_counter(&mut self) -> Result<CrashCounter> {
        Ok(CrashCounter::from_register(self.client.read(itf::CRASH_LOG_PAGE, crash_log::CRASH_COUNT)?))
    }

    fn crash_entry(&mut self, index: u32) -> Result<CrashLogEntry> {
        self.client.write(itf::CRASH_LOG_PAGE, crash_log::PREV_INDEX, index)?;
        Ok(CrashLogEntry {
            reset_reason: self.client.read(itf::CRASH_LOG_PAGE, crash_log::RESET_REASON)?,
            faults: self.client.read(itf::CRASH_LOG_PAGE, crash_log::PREV_FAULT_LIST)?,
            uptime: self.client.read(itf::CRASH_LOG_PAGE, crash_log::PREV_UPTIME)?,
            scratch1: self.client.read(itf::CRASH_LOG_PAGE, crash_log::PREV_SCRATCH_1)?,
            scratch2: self.client.read(itf::CRASH_LOG_PAGE, crash_log::PREV_SCRATCH_2)?,
        })
    }

    /// The most recent crash log entry (the cause of the last reset).
    pub fn last_reset_entry(&mut self) -> Result<CrashLogEntry> {
        self.crash_entry(0)
    }

    pub fn crash_log(&mut self) -> Result<Vec<CrashLogEntry>> {
        let count = self.client.read(itf::CRASH_LOG_PAGE, crash_log::PREV_COUNT)?;
        (0..count).map(|i| self.crash_entry(i)).collect()
    }

    pub fn fault_name(&mut self, fault_id: u32) -> Result<String> {
        if let Some(name) = self.fault_names.get(&fault_id) {
            return Ok(name.clone());
        }
        self.client.write(itf::SAFETY_STATUS_PAGE, safety_status::FAULT_IDX, fault_id)?;
        let name = self.client.read_string_page(itf::FAULT_NAME_PAGE)?;
        self.fault_names.insert(fault_id, name.clone());
        Ok(name)
    }

    pub fn fault_data(&mut self, fault_id: u32) -> Result<FaultData> {
        self.client.write(itf::SAFETY_STATUS_PAGE, safety_status::FAULT_IDX, fault_id)?;
        let page = itf::FAULT_DATA_PAGE;
        let sticky = self.client.read(page, fault_data::STICKY)? != 0;
        let multiple_fires = self.client.read(page, fault_data::MULTIPLE_FIRES)? != 0;
        let lower = self.client.read(page, fault_data::TIME_LOWER)?;
        let upper = self.client.read(page, fault_data::TIME_UPPER)?;
        let extra_data = self.client.read(page, fault_data::EXTRA_DATA)?;
        let line = self.client.read(page, fault_data::LINE_NUMBER)? as u16;
        let filename = self.client.read_string_page(itf::FAULT_FILENAME_PAGE)?;
        let name = self.fault_name(fault_id)?;
        Ok(FaultData {
            fault_id,
            name,
            sticky,
            multiple_fires,
            timestamp: flash_id_from_words(lower, upper),
            extra_data,
            filename,
            line,
        })
    }
}

impl GdbTarget for DebugClient {
    fn read_memory(&mut self, addr: u32) -> Result<u32> {
        self.client.write(itf::GDB_STUB_PAGE, gdb_stub::READ_WORD_ADDR, addr)?;
        self.client.read(itf::GDB_STUB_PAGE, gdb_stub::MEMORY_DATA)
    }

    fn write_memory(&mut self, addr: u32, data: u32) -> Result<()> {
        self.client.write(itf::GDB_STUB_PAGE, gdb_stub::MEMORY_DATA, data)?;
        self.client.write(itf::GDB_STUB_PAGE, gdb_stub::WRITE_WORD_ADDR, addr)
    }

    fn stub_pc(&mut self) -> Result<u32> {
        self.client.read(itf::GDB_STUB_PAGE, gdb_stub::PC_REGISTER)
    }

    fn stub_sp(&mut self) -> Result<u32> {
        self.client.read(itf::GDB_STUB_PAGE, gdb_stub::SP_REGISTER)
    }

    fn stub_lr(&mut self) -> Result<u32> {
        self.client.read(itf::GDB_STUB_PAGE, gdb_stub::LR_REGISTER)
    }

    fn ping(&mut self) -> Result<()> {
        let magic = self.client.read(itf::MCU_CONTROL_PAGE, mcu_control::MAGIC)?;
        if magic != mcu_control::MAGIC_VALUE {
            bail!("Unexpected application magic during ping");
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn formatting() {
        assert_eq!(Uptime(12_345).format(), "2 minutes 3 seconds");
        assert_eq!(Uptime(100 * 86_400 + 100).format(), "1 days 0 hours 0 minutes 1 seconds");
        assert_eq!(Uptime(12_345).format_seconds(), "123.45");
        assert_eq!(Uptime(5).format_seconds(), "0.05");

        let fault = |timestamp| FaultData {
            fault_id: 0,
            name: String::new(),
            sticky: true,
            multiple_fires: false,
            timestamp,
            extra_data: 0,
            filename: String::new(),
            line: 0,
        };
        assert_eq!(fault(1_500_000).format_timestamp(), "1.500000");
        assert_eq!(fault(61_000_001).format_timestamp(), "1:01.000001");
        assert_eq!(fault(3_661_000_000).format_timestamp(), "1:01:01.000000");

        let crash = CrashLogEntry { reset_reason: 3, faults: 0x10, uptime: 9000, scratch1: 0xAB, scratch2: 0x1000 };
        assert_eq!(
            crash.format(),
            "Panic (Message: 0x000000ab Call Address: 0x00001000) - Faults: 0x00000010 - Prev. Uptime: 1.50m"
        );
        let clean = CrashLogEntry { reset_reason: 0, faults: 0, uptime: 0, scratch1: 1, scratch2: 0 };
        assert_eq!(clean.format(), "Clean Boot: Power-on Reset");

        assert_eq!(SafetyStatus::from_register(0b0111).global_state(), "Robot Enabled");
        assert_eq!(SafetyStatus::from_register(0b0011).global_state(), "ROS Connected");
        assert_eq!(SafetyStatus::from_register(0).global_state(), "Disconnected - Idle");
        assert_eq!(
            CrashCounter::from_register(0x0102_0304).format(),
            "Total Crashes: 4 - Panics: 3 - Hard Faults: 2 - Assert Fails: 1"
        );
    }
}
