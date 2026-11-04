//! The CANmore control interface in Normal mode, ported from `titan_debug`: version, uptime, reboot, enter
//! bootloader, faults, crash log, gdb memory access and remote commands, as `canmore_cli` and `upload_tool` use
//! them.
//!
//! The register map is [`titan_control::debug`]; this module is the chip behind it ([`DebugBackend`]) and the
//! request loop. A transport feeds each request into [`serve`] with its own [`Server`] and sends the response back,
//! then calls [`Action::run`]: reboots requested by the agent wait until the response is out, as C acts on
//! `enter_bootloader_on_return` / `reboot_mcu_on_return` after answering. [`can_control_task`] does this for CAN
//! (utility channel 14, classic frames, no multiword like `debug_init(..., 0)`); Puddles serves UDP port 2202.
//!
//! Remote commands come in three groups besides `help`: the global ones (`memstats`), the transport's (`candbg` on
//! CAN) and the board's ([`init`]). Synchronous commands run inside the register map. A command that needs the
//! board's async drivers is declared with [`Command::deferred`]: the request loop hands it to the board through
//! [`next_deferred`] and waits for [`DeferredRequest::respond`].

use core::cell::RefCell;
use core::fmt::Write;

use canmore::reg_mapped::Server;
use canmore::reg_mapped::interface::debug::{REMOTE_CMD_ARGS_MAX_LEN, REMOTE_CMD_RESP_MAX_LEN};
use canmore::{CHAN_CONTROL_INTERFACE, ControlInterfaceMode};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::once_lock::OnceLock;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use titan_control::crash_log;
use titan_control::debug::{
    CrashCounts, CrashEntry, DebugBackend, DebugMap, DebugState, FaultRecord, SafetyStatus, is_execute_request,
};
pub use titan_control::remote_cmd::{Args, Command, Response, parse_int};
use titan_control::remote_cmd::{BUILTIN, Handler, Registry};

use crate::board::Version;
use crate::{canbus, crash, safety};

/// Flash size of every UWRT RP2040 board.
pub const FLASH_SIZE: u32 = titan_control::bootloader::FLASH_SIZE;
/// How long a deferred command may take before the agent gets an error.
const DEFERRED_TIMEOUT: Duration = Duration::from_millis(1000);
/// Time for a response queued on the bus to leave the controller before a requested reboot.
const RESPONSE_DRAIN: Duration = Duration::from_millis(5);

struct Info {
    /// `FULL_BUILD_TAG`, NUL terminated: `<crate> <major>.<minor>-<board> <git describe>`. C's tag also had the
    /// robot before the board and the builder and build date after the commit.
    version_string: heapless::Vec<u8, 128>,
    version: (u32, u32, u32),
    flash_id: u64,
    /// `help` and the global, transport and board commands.
    groups: [&'static [Command]; 4],
}

static INFO: OnceLock<Info> = OnceLock::new();
static STATE: Mutex<CriticalSectionRawMutex, RefCell<DebugState>> = Mutex::new(RefCell::new(DebugState::new()));

/// What the agent asked for that has to wait until its response is sent.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
#[must_use]
pub enum Action {
    None,
    Reboot,
    EnterBootloader,
}

impl Action {
    /// Carries the action out. Call once the response is sent.
    pub fn run(self) {
        match self {
            Action::None => {}
            Action::Reboot => {
                defmt::info!("Rebooting on request");
                crash::reboot()
            }
            Action::EnterBootloader => {
                defmt::info!("Entering bootloader on request");
                crash::enter_bootloader()
            }
        }
    }
}

/// Sets up the control interface (`debug_init`). `transport_commands` are the transport's own commands (`candbg`
/// for CAN), `board_commands` the board's. Call once, before serving requests.
pub fn init(
    board_name: &str,
    version: Version,
    flash_id: u64,
    transport_commands: &'static [Command],
    board_commands: &'static [Command],
) {
    let mut version_string = heapless::Vec::<u8, 128>::new();
    // A long git description is cut short, like any other overlong tag
    let mut tag = heapless::String::<127>::new();
    let _ = write!(tag, "{} {}.{}-{} {}", version.package, version.major, version.minor, board_name, version.git);
    let _ = version_string.extend_from_slice(tag.as_bytes());
    let _ = version_string.push(0);
    let info = Info {
        version_string,
        version: (version.major as u32, version.minor as u32, version.release_type as u32),
        flash_id,
        groups: [&BUILTIN, &GLOBAL_COMMANDS, transport_commands, board_commands],
    };
    if INFO.init(info).is_err() {
        defmt::warn!("Control interface already initialized");
    }
    if let Some(name) = registry().duplicate() {
        defmt::warn!("Remote command {=str} registered twice", name);
    }
}

fn info() -> &'static Info {
    INFO.try_get().expect("control interface not initialized")
}

fn registry() -> Registry<'static> {
    Registry::new(&info().groups)
}

/// The chip behind the debug register map.
struct Chip;

impl DebugBackend for Chip {
    fn version_string(&self) -> &[u8] {
        &info().version_string
    }

    fn version(&self) -> (u32, u32, u32) {
        info().version
    }

    fn flash_id(&self) -> u64 {
        info().flash_id
    }

    fn flash_size(&self) -> u32 {
        FLASH_SIZE
    }

    fn uptime_us(&self) -> u64 {
        Instant::now().as_micros()
    }

    fn safety_status(&self) -> SafetyStatus {
        SafetyStatus {
            is_setup: safety::is_setup(),
            initialized: safety::is_initialized(),
            kill_enabled: !safety::asserting_kill(),
            faults: safety::faults(),
        }
    }

    fn raise_fault(&mut self, id: u8) {
        safety::raise_fault(id, 0);
    }

    fn lower_fault(&mut self, id: u8) {
        safety::lower_fault(id);
    }

    fn fault_name(&self, id: u8) -> &str {
        safety::fault_name(id)
    }

    fn fault_record(&self, id: u8) -> Option<FaultRecord> {
        safety::fault_record(id)
    }

    fn crash_counts(&self) -> CrashCounts {
        safety::crash_counts()
    }

    fn crash_log_len(&self) -> u32 {
        crash::crash_data().len() as u32
    }

    fn crash_log_wrapped(&self) -> bool {
        crash::crash_data().wrapped()
    }

    fn crash_entry(&self, index: u32) -> Option<CrashEntry> {
        crash::crash_data().entry(index as usize).map(|e| crash_log::to_protocol(&e))
    }

    fn read_word(&mut self, addr: u32) -> u32 {
        // SAFETY: the map checked the address against `titan_control::memory::readable`
        unsafe { (addr as *const u32).read_volatile() }
    }

    fn write_word(&mut self, addr: u32, value: u32) {
        // SAFETY: the map checked the address against `titan_control::memory::writable`; gdb asked for this
        unsafe { (addr as *mut u32).write_volatile(value) }
    }

    fn stub_registers(&mut self) -> (u32, u32, u32) {
        // Where the stub runs, for gdb to show; LR 0 ends its unwind there
        (serve_sync as *const () as u32, cortex_m::register::msp::read(), 0)
    }

    fn commands(&self) -> Registry<'static> {
        registry()
    }
}

/// The remote command in the args page, if it's a deferred one.
fn pending_deferred(state: &mut DebugState) -> Option<DeferredRequest> {
    let args = state.args();
    let parsed = Args::parse(args).ok()?;
    let command = registry().find(parsed.get(0)?)?;
    if !matches!(command.handler, Handler::Deferred) {
        return None;
    }
    Some(DeferredRequest { command, args: heapless::Vec::from_slice(args).ok()? })
}

/// A deferred remote command for the board to run, from [`next_deferred`].
pub struct DeferredRequest {
    pub command: &'static Command,
    args: heapless::Vec<u8, { REMOTE_CMD_ARGS_MAX_LEN + 2 }>,
}

impl DeferredRequest {
    /// The command line; `args.get(0)` is the command name.
    pub fn args(&self) -> Args<'_> {
        // It parsed before it was handed over
        Args::parse(&self.args).unwrap_or(Args::from_slice(&[]))
    }

    /// Answers the request with a return code (0 for success) and the command's output.
    pub fn respond(self, rc: i32, output: &str) {
        let mut text = heapless::String::new();
        // Longer output is truncated like the response page
        for c in output.chars() {
            if text.push(c).is_err() {
                break;
            }
        }
        DEFERRED_DONE.signal((rc, text));
    }
}

static DEFERRED: Channel<CriticalSectionRawMutex, DeferredRequest, 1> = Channel::new();
static DEFERRED_DONE: Signal<CriticalSectionRawMutex, (i32, heapless::String<REMOTE_CMD_RESP_MAX_LEN>)> = Signal::new();

/// Waits for the next deferred remote command. A board with deferred commands runs a task that loops on this.
pub async fn next_deferred() -> DeferredRequest {
    DEFERRED.receive().await
}

async fn run_deferred(request: DeferredRequest) -> (i32, heapless::String<REMOTE_CMD_RESP_MAX_LEN>) {
    let name = request.command.name;
    DEFERRED_DONE.reset();
    if DEFERRED.try_send(request).is_err() {
        let mut text = heapless::String::new();
        let _ = write!(text, "Command '{name}' is busy");
        return (-1, text);
    }
    match with_timeout(DEFERRED_TIMEOUT, DEFERRED_DONE.wait()).await {
        Ok(result) => result,
        Err(_) => {
            // Take it back so a late handler doesn't answer the next one
            let _ = DEFERRED.try_receive();
            let mut text = heapless::String::new();
            let _ = write!(text, "Command '{name}' timed out");
            (-1, text)
        }
    }
}

fn serve_sync(server: &mut Server, req: &[u8], resp: &mut [u8]) -> (usize, Action) {
    STATE.lock(|state| {
        let mut state = state.borrow_mut();
        let mut chip = Chip;
        let len = server.handle_request(&mut DebugMap::new(&mut chip, &mut state), req, resp);
        let action = if state.enter_bootloader {
            Action::EnterBootloader
        } else if state.reboot {
            Action::Reboot
        } else {
            Action::None
        };
        state.enter_bootloader = false;
        state.reboot = false;
        (len, action)
    })
}

/// Handles one control interface request (`debug_process_message`): returns the response length (0: send
/// nothing) and what to do once it is sent.
pub async fn serve(server: &mut Server, req: &[u8], resp: &mut [u8]) -> (usize, Action) {
    if is_execute_request(req, server.mode(), server.in_bulk()) {
        let deferred = STATE.lock(|state| pending_deferred(&mut state.borrow_mut()));
        if let Some(request) = deferred {
            let (rc, text) = run_deferred(request).await;
            STATE.lock(|state| {
                let mut state = state.borrow_mut();
                let (_, mut out) = state.command_buffers();
                out.clear();
                let _ = out.write_str(&text);
                state.pending_rc = Some(rc);
            });
        }
    }
    serve_sync(server, req, resp)
}

/// Serves the control interface on CAN utility channel 14 and reports Normal mode in the heartbeat.
#[embassy_executor::task]
pub async fn can_control_task() -> ! {
    let mut server = Server::new_without_multiword(ControlInterfaceMode::Normal as u8);
    canbus::set_control_interface_mode(ControlInterfaceMode::Normal);
    loop {
        let request = canbus::CONTROL_RX.receive().await;
        let mut resp = [0u8; canmore::MAX_FRAME_SIZE];
        let (len, action) = serve(&mut server, &request, &mut resp).await;
        if len > 0 {
            // `serve` never writes past a classic frame
            let data = heapless::Vec::from_slice(&resp[..len]).unwrap_or_default();
            canbus::UTIL_TX.send(canbus::UtilityFrame { channel: CHAN_CONTROL_INTERFACE, data }).await;
        }
        if action != Action::None {
            while !canbus::UTIL_TX.is_empty() {
                Timer::after_millis(1).await;
            }
            Timer::after(RESPONSE_DRAIN).await;
            action.run();
        }
    }
}

// ========================================
// Global remote commands
// ========================================

static GLOBAL_COMMANDS: [Command; 1] = [Command::new("memstats", "", "Shows memory usage statistics", memstats)];

/// The CAN transport's commands (`candbg`).
pub static CAN_COMMANDS: [Command; 1] = [Command::new(
    "candbg",
    "[action]",
    "Issues a debug action to the can bus (trying to solve weird glitches)\nValid Actions: intr_en, fifo_clear, \
     hw_reset",
    candbg,
)];

fn write_size(out: &mut Response<'_>, label: &str, bytes: u32) {
    let _ = write!(out, "{label}:");
    for _ in label.len() + 1..40 {
        let _ = out.write_char(' ');
    }
    // `%.2f` without floating point
    let hundredths = |unit: u32| (bytes as u64 * 100 + unit as u64 / 2) / unit as u64;
    let _ = if bytes > 1024 * 1024 {
        let h = hundredths(1024 * 1024);
        writeln!(out, "{}.{:02} MB", h / 100, h % 100)
    } else if bytes > 1024 {
        let h = hundredths(1024);
        writeln!(out, "{}.{:02} KB", h / 100, h % 100)
    } else {
        writeln!(out, "{bytes} bytes")
    };
}

/// `memstats`. There is no heap, so this reports the RAM taken by statics and what is left for the stack.
fn memstats(_args: &Args<'_>, out: &mut Response<'_>) -> i32 {
    unsafe extern "C" {
        static __sdata: u8;
        static __sheap: u8;
        static _stack_start: u8;
    }
    // Only the addresses of the linker symbols are taken
    let (start, statics_end, top) =
        (&raw const __sdata as u32, &raw const __sheap as u32, &raw const _stack_start as u32);
    let total = top - start;
    let statics = statics_end - start;
    let used_percent = statics as u64 * 10_000 / total as u64;
    let _ = writeln!(out, "Overall Memory Usage:                   {}.{:02}%", used_percent / 100, used_percent % 100);
    write_size(out, "Total memory on chip", total);
    write_size(out, "Static memory reserved", statics);
    write_size(out, "Stack memory reserved", top - statics_end);
    0
}

fn candbg(args: &Args<'_>, out: &mut Response<'_>) -> i32 {
    if args.len() != 2 {
        let _ = writeln!(out, "Usage: candbg [action]");
        return 1;
    }
    // C printed "FIFO Reset" for hw_reset too; the Rust driver resets the whole controller for both
    let (action, message) = match args.get(1) {
        Some("intr_en") => (canbus::DebugAction::InterruptEnable, "Issuing CAN Interrupt Enable Signal"),
        Some("fifo_clear") => (canbus::DebugAction::Reset, "Issuing CAN FIFO Reset Signal"),
        Some("hw_reset") => (canbus::DebugAction::Reset, "Issuing CAN Hardware Reset Signal"),
        Some(other) => {
            let _ = writeln!(out, "Invalid Action: '{other}'");
            return 1;
        }
        None => return 1,
    };
    let _ = writeln!(out, "{message}");
    canbus::DEBUG_ACTION.signal(action);
    0
}
