//! The debug register map (`titan_debug`), served by applications in
//! [`Normal`](canmore::ControlInterfaceMode::Normal) mode on CANmore channel 14 or UDP port 2202.
//!
//! [`DebugMap`] implements every page of [`canmore::reg_mapped::interface::debug`] on top of a [`DebugBackend`] (the
//! chip: safety state, versions, memory) and a [`DebugState`] (the buffers and selections that persist between
//! requests). Register semantics, permissions and error codes follow the C `bindings/*.h`:
//!
//! * MCU control: magic, flash ID, version, and the enter-bootloader and reboot triggers. The triggers only set
//!   [`DebugState::enter_bootloader`] / [`DebugState::reboot`]; the firmware acts on them once the response is out.
//! * GDB stub: word reads (flash, ROM, RAM, peripherals) and writes (RAM, peripherals) through
//!   [`DebugBackend::read_word`] / [`DebugBackend::write_word`] after the range checks in [`crate::memory`], plus the
//!   stub's PC, SP and LR.
//! * Remote commands: the args and response pages and the execute register (see [`crate::remote_cmd`]). If the
//!   firmware already ran the command (deferred commands), it stores the result in [`DebugState::pending_rc`]
//!   before handing the execute request to the server; otherwise the map runs synchronous commands itself.
//! * Safety status, fault data and crash log, from the backend.

use canmore::reg_mapped::interface::debug::{
    self as itf, REMOTE_CMD_ARGS_MAX_LEN, REMOTE_CMD_RESP_MAX_LEN, crash_log, fault_data, gdb_stub, mcu_control,
    remote_cmd, safety_status,
};
use canmore::reg_mapped::{Access, Flags, MemoryPage, READ_REQUEST_LEN, RegisterMap, ServerError};

use crate::memory;
use crate::remote_cmd::{Outcome, Registry, Response};

/// Largest valid fault ID (`MAX_FAULT_ID`).
pub const MAX_FAULT_ID: u32 = 31;
/// Longest fault name kept for the fault name page, including the terminator.
pub const FAULT_NAME_LEN: usize = 64;
/// Longest source file name kept for the fault filename page, including the terminator.
pub const FAULT_FILE_LEN: usize = 128;

/// Global safety state (`safety_global_state_cb`).
#[derive(Clone, Copy, Default, Debug, PartialEq, Eq)]
pub struct SafetyStatus {
    /// Safety was set up (`safety_setup`).
    pub is_setup: bool,
    /// Safety is initialized: kill switches are honoured (ROS is connected).
    pub initialized: bool,
    /// No kill switch asserts kill: the vehicle is enabled.
    pub kill_enabled: bool,
    /// Bit mask of the raised faults.
    pub faults: u32,
}

/// What was recorded the last time a fault was raised (`struct fault_data`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FaultRecord {
    pub time_us: u64,
    pub extra: u32,
    pub file: &'static str,
    pub line: u32,
    pub multiple_fires: bool,
}

/// Crash counters since the last clean boot (each saturates at 255 when reported).
#[derive(Clone, Copy, Default, Debug, PartialEq, Eq)]
pub struct CrashCounts {
    pub total: u32,
    pub panic: u32,
    pub hard_fault: u32,
    pub assert_fail: u32,
}

/// One crash log entry, with values already in protocol terms (`crash_log::RESET_REASON_*`, `CLEAN_RESET_*`).
#[derive(Clone, Copy, Default, Debug, PartialEq, Eq)]
pub struct CrashEntry {
    pub reset_reason: u32,
    pub faults: u32,
    /// Uptime at the reset, in [`safety_status::UPTIME_TICKS_PER_SECOND`] ticks (C: tens of milliseconds).
    pub uptime: u32,
    pub scratch_1: u32,
    pub scratch_2: u32,
}

/// The chip behind the debug map.
pub trait DebugBackend {
    /// Version string page contents, NUL terminated (`FULL_BUILD_TAG`).
    fn version_string(&self) -> &[u8];
    /// `(major, minor, release type)` (see [`itf::release_type`]).
    fn version(&self) -> (u32, u32, u32);
    /// The flash unique ID (lower word in the lower 32 bits).
    fn flash_id(&self) -> u64;
    /// Flash size in bytes, bounding gdb flash reads.
    fn flash_size(&self) -> u32;
    fn uptime_us(&self) -> u64;
    fn safety_status(&self) -> SafetyStatus;
    fn raise_fault(&mut self, id: u8);
    fn lower_fault(&mut self, id: u8);
    /// Name of fault `id` (`safety_lookup_fault_id`).
    fn fault_name(&self, id: u8) -> &str;
    /// The fault's record, if it was ever raised.
    fn fault_record(&self, id: u8) -> Option<FaultRecord>;
    fn crash_counts(&self) -> CrashCounts;
    /// Number of crash log entries.
    fn crash_log_len(&self) -> u32;
    /// True if older entries were overwritten.
    fn crash_log_wrapped(&self) -> bool;
    /// Crash log entry `index` back from the most recent (0).
    fn crash_entry(&self, index: u32) -> Option<CrashEntry>;
    /// Reads the word at `addr`, already checked with [`memory::readable`].
    fn read_word(&mut self, addr: u32) -> u32;
    /// Writes the word at `addr`, already checked with [`memory::writable`].
    fn write_word(&mut self, addr: u32, value: u32);
    /// `(pc, sp, lr)` of the code serving the request, for gdb to unwind the stub's own frame.
    fn stub_registers(&mut self) -> (u32, u32, u32);
    /// The registered remote commands.
    fn commands(&self) -> Registry<'static>;
}

/// Buffers and selections the debug interface keeps between requests.
pub struct DebugState {
    /// The remote command args page. Two extra NULs end the last argument even if the agent filled the page.
    args: [u8; REMOTE_CMD_ARGS_MAX_LEN + 2],
    resp: [u8; REMOTE_CMD_RESP_MAX_LEN],
    /// Result of a command the firmware already ran, returned by the next execute register read.
    pub pending_rc: Option<i32>,
    /// The agent asked to reboot into the bootloader.
    pub enter_bootloader: bool,
    /// The agent asked to reboot.
    pub reboot: bool,
    gdb_memory_data: u32,
    crash_index: u32,
    crash_entry: CrashEntry,
    fault_idx: u32,
    fault_selected: bool,
    fault: Option<FaultRecord>,
    fault_name: [u8; FAULT_NAME_LEN],
    fault_name_len: usize,
    fault_file: [u8; FAULT_FILE_LEN],
    fault_file_len: usize,
}

impl Default for DebugState {
    fn default() -> Self {
        Self::new()
    }
}

impl DebugState {
    pub const fn new() -> Self {
        DebugState {
            args: [0; REMOTE_CMD_ARGS_MAX_LEN + 2],
            resp: [0; REMOTE_CMD_RESP_MAX_LEN],
            pending_rc: None,
            enter_bootloader: false,
            reboot: false,
            gdb_memory_data: 0,
            crash_index: u32::MAX,
            crash_entry: CrashEntry { reset_reason: 0, faults: 0, uptime: 0, scratch_1: 0, scratch_2: 0 },
            fault_idx: u32::MAX,
            fault_selected: false,
            fault: None,
            fault_name: [0; FAULT_NAME_LEN],
            fault_name_len: 0,
            fault_file: [0; FAULT_FILE_LEN],
            fault_file_len: 0,
        }
    }

    /// The args page with the terminating NULs in place, ready for [`crate::remote_cmd::Args::parse`].
    pub fn args(&mut self) -> &[u8] {
        self.args[REMOTE_CMD_ARGS_MAX_LEN] = 0;
        self.args[REMOTE_CMD_ARGS_MAX_LEN + 1] = 0;
        &self.args
    }

    /// The args page and the response page, for running a command outside the map.
    pub fn command_buffers(&mut self) -> (&[u8], Response<'_>) {
        self.args[REMOTE_CMD_ARGS_MAX_LEN] = 0;
        self.args[REMOTE_CMD_ARGS_MAX_LEN + 1] = 0;
        (&self.args, Response::new(&mut self.resp))
    }

    /// Runs the command in the args page if it is synchronous. Returns the command to run otherwise.
    pub fn execute_sync(&mut self, registry: &Registry<'static>) -> Outcome {
        let (args, mut out) = self.command_buffers();
        crate::remote_cmd::execute(registry, args, &mut out)
    }
}

/// True if `req` is a request the server will answer by reading the remote command execute register: a single
/// read of that register in `mode`, outside a bulk transfer. Firmware with deferred commands checks this to run the
/// command before handing the request to the server.
pub fn is_execute_request(req: &[u8], mode: u8, in_bulk: bool) -> bool {
    req.len() == READ_REQUEST_LEN
        && !in_bulk
        && Flags::from_byte(req[0]) == Flags { mode, ..Flags::default() }
        && req[2] == itf::REMOTE_CMD_PAGE
        && req[3] == remote_cmd::EXECUTE
}

/// The debug register map over a backend and its state. Build one per request (it only borrows).
pub struct DebugMap<'a, B: DebugBackend + ?Sized> {
    pub backend: &'a mut B,
    pub state: &'a mut DebugState,
}

const ADDRESS: ServerError = ServerError::InvalidRegisterAddress;
const MODE: ServerError = ServerError::InvalidRegisterMode;
const DATA: ServerError = ServerError::InvalidData;

fn saturate(count: u32) -> u32 {
    count.min(0xFF)
}

impl<B: DebugBackend + ?Sized> DebugMap<'_, B> {
    pub fn new<'a>(backend: &'a mut B, state: &'a mut DebugState) -> DebugMap<'a, B> {
        DebugMap { backend, state }
    }

    fn memory_page(&mut self, page: u8) -> Result<MemoryPage<'_>, ServerError> {
        Ok(match page {
            itf::VERSION_STRING_PAGE => MemoryPage::Bytes(Access::ReadOnly(self.backend.version_string())),
            // Only the advertised length is exposed, the extra NULs stay private
            itf::REMOTE_CMD_ARGS_PAGE => {
                MemoryPage::Bytes(Access::ReadWrite(&mut self.state.args[..REMOTE_CMD_ARGS_MAX_LEN]))
            }
            itf::REMOTE_CMD_RESP_PAGE => MemoryPage::Bytes(Access::ReadOnly(&self.state.resp)),
            // Unimplemented until a fault is selected, like the C pages filled in by the fault index callback
            itf::FAULT_NAME_PAGE if self.state.fault_selected => {
                MemoryPage::Bytes(Access::ReadOnly(&self.state.fault_name[..self.state.fault_name_len]))
            }
            itf::FAULT_FILENAME_PAGE if self.state.fault_selected => {
                MemoryPage::Bytes(Access::ReadOnly(&self.state.fault_file[..self.state.fault_file_len]))
            }
            _ => return Err(ADDRESS),
        })
    }

    fn select_crash_entry(&mut self, index: u32) -> Result<(), ServerError> {
        if index >= self.backend.crash_log_len() {
            return Err(DATA);
        }
        let entry = self.backend.crash_entry(index).ok_or(DATA)?;
        self.state.crash_index = index;
        self.state.crash_entry = entry;
        Ok(())
    }

    fn select_fault(&mut self, id: u32) -> Result<(), ServerError> {
        if id > MAX_FAULT_ID {
            return Err(DATA);
        }
        let state = &mut *self.state;
        state.fault_idx = id;
        state.fault_selected = true;
        state.fault = self.backend.fault_record(id as u8);
        state.fault_name_len = copy_c_string(&mut state.fault_name, self.backend.fault_name(id as u8));
        // The filename is only valid once the fault fired
        let file = state.fault.map(|f| f.file).unwrap_or("");
        state.fault_file_len = copy_c_string(&mut state.fault_file, file);
        Ok(())
    }

    fn gdb_read(&mut self, addr: u32) -> Result<(), ServerError> {
        if !memory::readable(addr, self.backend.flash_size()) {
            return Err(DATA);
        }
        // Read flash through the no-allocate alias so the cache isn't disturbed
        let addr = if memory::is_flash(addr, self.backend.flash_size()) {
            memory::XIP_NOALLOC_BASE + memory::flash_offset(addr)
        } else {
            addr
        };
        self.state.gdb_memory_data = self.backend.read_word(addr);
        Ok(())
    }

    fn execute(&mut self) -> i32 {
        let rc = match self.state.pending_rc.take() {
            Some(rc) => rc,
            None => match self.state.execute_sync(&self.backend.commands()) {
                Outcome::Done(rc) => rc,
                Outcome::Deferred(cmd) => {
                    use core::fmt::Write;
                    let mut out = Response::new(&mut self.state.resp);
                    let _ = write!(out, "Command '{}' can't run here", cmd.name);
                    -1
                }
            },
        };
        // Don't run the same command again if the register is read twice
        self.state.args[0] = 0;
        rc
    }
}

/// Copies `s` with a terminator into `buf`, truncating. Returns the length including the terminator.
fn copy_c_string(buf: &mut [u8], s: &str) -> usize {
    let n = s.len().min(buf.len() - 1);
    buf[..n].copy_from_slice(&s.as_bytes()[..n]);
    buf[n] = 0;
    n + 1
}

impl<B: DebugBackend + ?Sized> RegisterMap for DebugMap<'_, B> {
    fn read(&mut self, page: u8, offset: u8) -> Result<u32, ServerError> {
        match page {
            itf::MCU_CONTROL_PAGE => match offset {
                mcu_control::MAGIC => Ok(mcu_control::MAGIC_VALUE),
                mcu_control::LOWER_FLASH_ID => Ok(self.backend.flash_id() as u32),
                mcu_control::UPPER_FLASH_ID => Ok((self.backend.flash_id() >> 32) as u32),
                mcu_control::MAJOR_VERSION => Ok(self.backend.version().0),
                mcu_control::MINOR_VERSION => Ok(self.backend.version().1),
                mcu_control::RELEASE_TYPE => Ok(self.backend.version().2),
                mcu_control::ENTER_BL | mcu_control::REBOOT_MCU => Err(MODE),
                _ => Err(ADDRESS),
            },
            itf::GDB_STUB_PAGE => match offset {
                gdb_stub::MEMORY_DATA => Ok(self.state.gdb_memory_data),
                gdb_stub::PC_REGISTER => Ok(self.backend.stub_registers().0),
                gdb_stub::SP_REGISTER => Ok(self.backend.stub_registers().1),
                gdb_stub::LR_REGISTER => Ok(self.backend.stub_registers().2),
                gdb_stub::READ_WORD_ADDR | gdb_stub::WRITE_WORD_ADDR => Err(MODE),
                _ => Err(ADDRESS),
            },
            itf::REMOTE_CMD_PAGE => match offset {
                remote_cmd::EXECUTE => Ok(self.execute() as u32),
                _ => Err(ADDRESS),
            },
            itf::SAFETY_STATUS_PAGE => {
                let status = self.backend.safety_status();
                match offset {
                    safety_status::GLOBAL_STATE => Ok((status.is_setup as u32)
                        << safety_status::GLOBAL_STATE_IS_SETUP_BIT
                        | (status.initialized as u32) << safety_status::GLOBAL_STATE_IS_INITIALIZED_BIT
                        | (status.kill_enabled as u32) << safety_status::GLOBAL_STATE_KILL_IS_ENABLED_BIT
                        | ((status.faults != 0) as u32) << safety_status::GLOBAL_STATE_FAULT_PRESENT_BIT),
                    safety_status::FAULT_LIST => Ok(status.faults),
                    safety_status::UPTIME => {
                        Ok((self.backend.uptime_us() / (1_000_000 / safety_status::UPTIME_TICKS_PER_SECOND as u64))
                            as u32)
                    }
                    safety_status::FAULT_IDX => Ok(self.state.fault_idx),
                    safety_status::RAISE_FAULT | safety_status::LOWER_FAULT => Err(MODE),
                    _ => Err(ADDRESS),
                }
            }
            itf::CRASH_LOG_PAGE => match offset {
                crash_log::CRASH_COUNT => {
                    let c = self.backend.crash_counts();
                    Ok(saturate(c.total) << crash_log::CRASH_COUNT_TOTAL_SHIFT
                        | saturate(c.panic) << crash_log::CRASH_COUNT_PANIC_SHIFT
                        | saturate(c.hard_fault) << crash_log::CRASH_COUNT_HARD_FAULT_SHIFT
                        | saturate(c.assert_fail) << crash_log::CRASH_COUNT_ASSERT_FAIL_SHIFT)
                }
                crash_log::HAS_WRAPPED => Ok(self.backend.crash_log_wrapped() as u32),
                crash_log::PREV_COUNT => Ok(self.backend.crash_log_len()),
                crash_log::PREV_INDEX => Ok(self.state.crash_index),
                crash_log::RESET_REASON => Ok(self.state.crash_entry.reset_reason),
                crash_log::PREV_FAULT_LIST => Ok(self.state.crash_entry.faults),
                crash_log::PREV_UPTIME => Ok(self.state.crash_entry.uptime),
                crash_log::PREV_SCRATCH_1 => Ok(self.state.crash_entry.scratch_1),
                crash_log::PREV_SCRATCH_2 => Ok(self.state.crash_entry.scratch_2),
                _ => Err(ADDRESS),
            },
            itf::FAULT_DATA_PAGE => {
                let fault = self.state.fault;
                let time = fault.map_or(0, |f| f.time_us);
                match offset {
                    fault_data::STICKY => Ok(fault.is_some() as u32),
                    fault_data::TIME_LOWER => Ok(time as u32),
                    fault_data::TIME_UPPER => Ok((time >> 32) as u32),
                    fault_data::EXTRA_DATA => Ok(fault.map_or(0, |f| f.extra)),
                    fault_data::LINE_NUMBER => Ok(fault.map_or(0, |f| f.line)),
                    fault_data::MULTIPLE_FIRES => Ok(fault.is_some_and(|f| f.multiple_fires) as u32),
                    _ => Err(ADDRESS),
                }
            }
            _ => self.memory_page(page)?.read(offset),
        }
    }

    fn write(&mut self, page: u8, offset: u8, data: u32) -> Result<(), ServerError> {
        match page {
            itf::MCU_CONTROL_PAGE => match offset {
                mcu_control::ENTER_BL => {
                    self.state.enter_bootloader = true;
                    Ok(())
                }
                mcu_control::REBOOT_MCU => {
                    self.state.reboot = true;
                    Ok(())
                }
                mcu_control::MAGIC
                | mcu_control::LOWER_FLASH_ID
                | mcu_control::UPPER_FLASH_ID
                | mcu_control::MAJOR_VERSION
                | mcu_control::MINOR_VERSION
                | mcu_control::RELEASE_TYPE => Err(MODE),
                _ => Err(ADDRESS),
            },
            itf::GDB_STUB_PAGE => match offset {
                gdb_stub::READ_WORD_ADDR => self.gdb_read(data),
                gdb_stub::WRITE_WORD_ADDR => {
                    if !memory::writable(data) {
                        return Err(DATA);
                    }
                    self.backend.write_word(data, self.state.gdb_memory_data);
                    Ok(())
                }
                gdb_stub::MEMORY_DATA => {
                    self.state.gdb_memory_data = data;
                    Ok(())
                }
                gdb_stub::PC_REGISTER | gdb_stub::SP_REGISTER | gdb_stub::LR_REGISTER => Err(MODE),
                _ => Err(ADDRESS),
            },
            itf::REMOTE_CMD_PAGE => match offset {
                remote_cmd::EXECUTE => Err(MODE),
                _ => Err(ADDRESS),
            },
            itf::SAFETY_STATUS_PAGE => match offset {
                safety_status::FAULT_IDX => self.select_fault(data),
                safety_status::RAISE_FAULT if data <= MAX_FAULT_ID => {
                    self.backend.raise_fault(data as u8);
                    Ok(())
                }
                safety_status::LOWER_FAULT if data <= MAX_FAULT_ID => {
                    self.backend.lower_fault(data as u8);
                    Ok(())
                }
                safety_status::RAISE_FAULT | safety_status::LOWER_FAULT => Err(DATA),
                safety_status::GLOBAL_STATE | safety_status::FAULT_LIST | safety_status::UPTIME => Err(MODE),
                _ => Err(ADDRESS),
            },
            itf::CRASH_LOG_PAGE => match offset {
                crash_log::PREV_INDEX => self.select_crash_entry(data),
                crash_log::CRASH_COUNT..=crash_log::PREV_SCRATCH_2 => Err(MODE),
                _ => Err(ADDRESS),
            },
            itf::FAULT_DATA_PAGE => match offset {
                fault_data::STICKY..=fault_data::MULTIPLE_FIRES => Err(MODE),
                _ => Err(ADDRESS),
            },
            _ => self.memory_page(page)?.write(offset, data),
        }
    }

    fn read_multiword(&mut self, page: u8, offset: u8, out: &mut [u8]) -> Result<(), ServerError> {
        self.memory_page(page)?.read_multiword(offset, out)
    }

    fn write_multiword(&mut self, page: u8, offset: u8, data: &[u8]) -> Result<(), ServerError> {
        self.memory_page(page)?.write_multiword(offset, data)
    }
}
