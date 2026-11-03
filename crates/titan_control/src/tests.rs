//! The register maps driven through `canmore::reg_mapped::Client`, in memory, with the call sequences of the host
//! tools (`titan_tools::canmore::{DebugClient, BootloaderClient}`) over both CAN (8 byte frames, bulk writes of up
//! to 32 requests, no multiword) and UDP (multiword).

extern crate std;

use core::fmt::Write;
use std::collections::VecDeque;
use std::string::String;
use std::vec;
use std::vec::Vec;

use canmore::ControlInterfaceMode;
use canmore::reg_mapped::interface::bootloader::{self as bl, flash_control};
use canmore::reg_mapped::interface::debug::{self as itf, crash_log, fault_data, gdb_stub, mcu_control, safety_status};
use canmore::reg_mapped::{Client, ClientError, RegisterMap, Server, ServerError, TransferMode, Transport};

use crate::bootloader::{self, BootloaderInfo, BootloaderMap, BootloaderState, FlashBackend};
use crate::debug::{
    CrashCounts, CrashEntry, DebugBackend, DebugMap, DebugState, FaultRecord, SafetyStatus, is_execute_request,
};
use crate::remote_cmd::{self, Args, BUILTIN, Command, Outcome, Registry, Response};

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
struct Timeout;

impl core::fmt::Display for Timeout {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("timeout")
    }
}

/// How a link carries the control interface.
#[derive(Clone, Copy, Debug)]
enum Link {
    /// Classic CAN frames on channel 14, as `CanTransport` uses them (the server has no multiword buffer).
    Can,
    /// UDP datagrams on port 2202.
    Udp,
}

impl Link {
    fn mtu(self) -> usize {
        match self {
            Link::Can => 8,
            Link::Udp => 512,
        }
    }

    fn transfer_mode(self) -> TransferMode {
        match self {
            Link::Can => TransferMode::Bulk { max_in_flight: 32 },
            Link::Udp => TransferMode::Multiword,
        }
    }

    fn server(self, mode: ControlInterfaceMode) -> Server {
        match self {
            Link::Can => Server::new_without_multiword(mode as u8),
            Link::Udp => Server::new(mode as u8),
        }
    }
}

const LINKS: [Link; 2] = [Link::Can, Link::Udp];

/// Something that can answer requests: a server plus the device behind its map.
trait Device {
    fn handle(&mut self, req: &[u8], resp: &mut [u8]) -> usize;
}

/// Delivers requests straight to a device, queueing at most `queue` unanswered requests like the firmware's
/// receive FIFO (requests beyond that are lost).
struct Loopback<D> {
    device: D,
    link: Link,
    responses: VecDeque<Vec<u8>>,
}

impl<D: Device> Transport for Loopback<D> {
    type Error = Timeout;

    fn mtu(&self) -> usize {
        self.link.mtu()
    }

    fn clear_rx(&mut self) -> Result<(), Timeout> {
        self.responses.clear();
        Ok(())
    }

    fn send(&mut self, data: &[u8]) -> Result<(), Timeout> {
        assert!(data.len() <= self.link.mtu());
        let mut resp = vec![0u8; self.link.mtu()];
        let len = self.device.handle(data, &mut resp);
        if len > 0 {
            self.responses.push_back(resp[..len].to_vec());
        }
        Ok(())
    }

    fn recv(&mut self, buf: &mut [u8]) -> Result<usize, Timeout> {
        let resp = self.responses.pop_front().ok_or(Timeout)?;
        buf[..resp.len()].copy_from_slice(&resp);
        Ok(resp.len())
    }
}

fn client<D: Device>(device: D, link: Link, mode: ControlInterfaceMode) -> Client<Loopback<D>> {
    Client::new(Loopback { device, link, responses: VecDeque::new() }, mode as u8, link.transfer_mode())
}

// ================================================================ Debug interface

const FLASH_SIZE: u32 = 2 * 1024 * 1024;
const VERSION: &[u8] = b"test_board 1.2-talos/test_board abc123\0";

/// A fake chip: 4 KiB of RAM at 0x20000000, a flash with a recognisable pattern, and safety state.
struct FakeChip {
    ram: Vec<u32>,
    faults: u32,
    raised: Vec<u8>,
    lowered: Vec<u8>,
    deferred_runs: usize,
}

fn add(args: &Args<'_>, out: &mut Response<'_>) -> i32 {
    match (args.get(1).and_then(remote_cmd::parse_int), args.get(2).and_then(remote_cmd::parse_int)) {
        (Some(a), Some(b)) => {
            let _ = write!(out, "{}", a + b);
            0
        }
        _ => {
            let _ = out.write_str("Usage: add [a] [b]");
            1
        }
    }
}

fn long_output(_: &Args<'_>, out: &mut Response<'_>) -> i32 {
    for i in 0..400 {
        let _ = write!(out, "{i} ");
    }
    7
}

static BOARD_COMMANDS: [Command; 3] = [
    Command::new("add", "[a] [b]", "Adds the two arguments [a] and [b] together", add),
    Command::new("long", "", "Fills the response page", long_output),
    Command::deferred("gauge", "[register]", "Asks another task"),
];
static GROUPS: [&[Command]; 2] = [&BUILTIN, &BOARD_COMMANDS];

impl FakeChip {
    fn new() -> Self {
        FakeChip { ram: vec![0; 1024], faults: 0b100, raised: Vec::new(), lowered: Vec::new(), deferred_runs: 0 }
    }
}

impl DebugBackend for FakeChip {
    fn version_string(&self) -> &[u8] {
        VERSION
    }
    fn version(&self) -> (u32, u32, u32) {
        (1, 2, itf::release_type::DEV)
    }
    fn flash_id(&self) -> u64 {
        0xE660_5838_1B27_9D2F
    }
    fn flash_size(&self) -> u32 {
        FLASH_SIZE
    }
    fn uptime_us(&self) -> u64 {
        12_345_678
    }
    fn safety_status(&self) -> SafetyStatus {
        SafetyStatus { is_setup: true, initialized: false, kill_enabled: false, faults: self.faults }
    }
    fn raise_fault(&mut self, id: u8) {
        self.faults |= 1 << id;
        self.raised.push(id);
    }
    fn lower_fault(&mut self, id: u8) {
        self.faults &= !(1 << id);
        self.lowered.push(id);
    }
    fn fault_name(&self, id: u8) -> &str {
        match id {
            0 => "FAULT_WATCHDOG_RESET",
            2 => "FAULT_CAN_INTERNAL_ERROR",
            _ => "UNKNOWN",
        }
    }
    fn fault_record(&self, id: u8) -> Option<FaultRecord> {
        (id == 2).then_some(FaultRecord {
            time_us: 0x1_0000_0005,
            extra: 0xBEEF,
            file: "firmware/titan_rp2040/src/canbus.rs",
            line: 172,
            multiple_fires: true,
        })
    }
    fn crash_counts(&self) -> CrashCounts {
        CrashCounts { total: 3, panic: 1, hard_fault: 0, assert_fail: 300 }
    }
    fn crash_log_len(&self) -> u32 {
        2
    }
    fn crash_log_wrapped(&self) -> bool {
        false
    }
    fn crash_entry(&self, index: u32) -> Option<CrashEntry> {
        match index {
            0 => Some(CrashEntry {
                reset_reason: crash_log::RESET_REASON_PANIC,
                faults: 0x5,
                uptime: 42,
                scratch_1: 0,
                scratch_2: 0,
            }),
            1 => Some(CrashEntry {
                reset_reason: crash_log::RESET_REASON_CLEAN_BOOT,
                faults: 0,
                uptime: 0,
                scratch_1: crash_log::CLEAN_RESET_POR,
                scratch_2: 0,
            }),
            _ => None,
        }
    }
    fn read_word(&mut self, addr: u32) -> u32 {
        match addr {
            0x2000_0000..0x2000_1000 => self.ram[(addr as usize - 0x2000_0000) / 4],
            // Flash reads must come through the no-allocate alias
            0x1100_0000..0x1120_0000 => addr ^ 0xA5A5_A5A5,
            _ => 0xDEAD_BEEF,
        }
    }
    fn write_word(&mut self, addr: u32, value: u32) {
        if let 0x2000_0000..0x2000_1000 = addr {
            self.ram[(addr as usize - 0x2000_0000) / 4] = value;
        }
    }
    fn stub_registers(&mut self) -> (u32, u32, u32) {
        (0x1000_1234, 0x2003_FF00, 0x1000_5679)
    }
    fn commands(&self) -> Registry<'static> {
        Registry::new(&GROUPS)
    }
}

/// The firmware side of the debug interface: server, map state and the deferred command runner of `titan_rp2040`.
struct DebugDevice {
    server: Server,
    chip: FakeChip,
    state: std::boxed::Box<DebugState>,
}

impl DebugDevice {
    fn new(link: Link) -> Self {
        DebugDevice {
            server: link.server(ControlInterfaceMode::Normal),
            chip: FakeChip::new(),
            state: std::boxed::Box::new(DebugState::new()),
        }
    }
}

impl Device for DebugDevice {
    fn handle(&mut self, req: &[u8], resp: &mut [u8]) -> usize {
        if is_execute_request(req, self.server.mode(), self.server.in_bulk()) {
            // What titan_rp2040 does before handing the request to the server: run deferred commands itself
            let registry = self.chip.commands();
            if let Outcome::Deferred(cmd) = self.state.execute_sync(&registry) {
                let (args, mut out) = self.state.command_buffers();
                let args = Args::parse(args).unwrap();
                let _ = write!(out, "{} ran elsewhere with {:?}", cmd.name, args.as_slice());
                self.chip.deferred_runs += 1;
                self.state.pending_rc = Some(5);
            } else {
                // Sync commands already ran into the response page; hand over the return code
                let (args, mut out) = self.state.command_buffers();
                let _ = (args, &mut out);
            }
        }
        let mut map = DebugMap::new(&mut self.chip, &mut self.state);
        self.server.handle_request(&mut map, req, resp)
    }
}

type DebugClient = Client<Loopback<DebugDevice>>;

fn debug_client(link: Link) -> DebugClient {
    client(DebugDevice::new(link), link, ControlInterfaceMode::Normal)
}

fn device(c: &mut DebugClient) -> &mut DebugDevice {
    &mut c.transport_mut().device
}

fn string_page(c: &mut DebugClient, page: u8) -> String {
    let mut buf = [0u8; canmore::reg_mapped::PAGE_SIZE];
    let len = c.read_string_page(page, &mut buf).unwrap();
    String::from_utf8(buf[..len].to_vec()).unwrap()
}

/// `DebugClient::execute_remote_cmd`.
fn remote_cmd(c: &mut DebugClient, args: &[&str]) -> (i32, String) {
    let mut encoded = Vec::new();
    for a in args {
        encoded.extend_from_slice(a.as_bytes());
        encoded.push(0);
    }
    encoded.push(0);
    c.write_string_page(itf::REMOTE_CMD_ARGS_PAGE, &encoded).unwrap();
    let rc = c.read_register(itf::REMOTE_CMD_PAGE, itf::remote_cmd::EXECUTE).unwrap() as i32;
    (rc, string_page(c, itf::REMOTE_CMD_RESP_PAGE))
}

#[test]
fn debug_identification() {
    for link in LINKS {
        let mut c = debug_client(link);
        let page = itf::MCU_CONTROL_PAGE;
        assert_eq!(c.read_register(page, mcu_control::MAGIC).unwrap(), mcu_control::MAGIC_VALUE);
        assert_eq!(c.read_register(page, mcu_control::LOWER_FLASH_ID).unwrap(), 0x1B27_9D2F);
        assert_eq!(c.read_register(page, mcu_control::UPPER_FLASH_ID).unwrap(), 0xE660_5838);
        assert_eq!(c.read_register(page, mcu_control::MAJOR_VERSION).unwrap(), 1);
        assert_eq!(c.read_register(page, mcu_control::MINOR_VERSION).unwrap(), 2);
        assert_eq!(c.read_register(page, mcu_control::RELEASE_TYPE).unwrap(), itf::release_type::DEV);
        assert_eq!(string_page(&mut c, itf::VERSION_STRING_PAGE), "test_board 1.2-talos/test_board abc123");
        assert_eq!(c.write_register(page, mcu_control::MAGIC, 1), Err(ClientError::Server(ServerError::InvalidRegisterMode)));
        assert_eq!(c.read_register(page, mcu_control::REBOOT_MCU), Err(ClientError::Server(ServerError::InvalidRegisterMode)));
        assert_eq!(c.read_register(page, 0x20), Err(ClientError::Server(ServerError::InvalidRegisterAddress)));
        assert_eq!(c.read_register(0x40, 0), Err(ClientError::Server(ServerError::InvalidRegisterAddress)));

        assert!(!device(&mut c).state.enter_bootloader && !device(&mut c).state.reboot);
        c.write_register(page, mcu_control::ENTER_BL, 1).unwrap();
        assert!(device(&mut c).state.enter_bootloader);
        c.write_register(page, mcu_control::REBOOT_MCU, 1).unwrap();
        assert!(device(&mut c).state.reboot);

        // A boot delay request sent to an application is rejected, not acted upon
        c.send_raw(&canmore::reg_mapped::BOOTLOADER_REQUEST).unwrap();
        let mut resp = [0u8; 8];
        let len = c.transport_mut().recv(&mut resp).unwrap();
        assert_eq!(resp[0], ServerError::InvalidMode as u8, "{len}");
    }
}

#[test]
fn debug_remote_commands() {
    for link in LINKS {
        let mut c = debug_client(link);
        assert_eq!(remote_cmd(&mut c, &["add", "2", "0x10"]), (0, "18".into()));
        assert_eq!(remote_cmd(&mut c, &["add"]), (1, "Usage: add [a] [b]".into()));
        assert_eq!(remote_cmd(&mut c, &["nope"]), (127, "Unknown Remote Command: 'nope'".into()));

        // Reading the execute register again doesn't run the command twice
        let rc = c.read_register(itf::REMOTE_CMD_PAGE, itf::remote_cmd::EXECUTE).unwrap();
        assert_eq!(rc, 127);
        assert_eq!(string_page(&mut c, itf::REMOTE_CMD_RESP_PAGE), "Cannot execute null command");

        // Deferred commands run outside the map
        let (rc, out) = remote_cmd(&mut c, &["gauge", "charging"]);
        assert_eq!((rc, out.as_str()), (5, "gauge ran elsewhere with [\"gauge\", \"charging\"]"));
        assert_eq!(device(&mut c).chip.deferred_runs, 1);

        // Output is truncated to the page, terminator included
        let (rc, out) = remote_cmd(&mut c, &["long"]);
        assert_eq!(rc, 7);
        assert_eq!(out.len(), itf::REMOTE_CMD_RESP_MAX_LEN - 1);

        // The canmore_cli help discovery
        let (rc, count) = remote_cmd(&mut c, &["help", "-n"]);
        assert_eq!((rc, count.as_str()), (0, "4"));
        let names: Vec<String> = (0..4)
            .map(|i| {
                let (rc, info) = remote_cmd(&mut c, &["help", "-i", &std::format!("{i}")]);
                assert_eq!(rc, 0);
                let mut lines = info.split('\n');
                let name = lines.next().unwrap().into();
                assert!(lines.next().is_some(), "usage line missing");
                name
            })
            .collect();
        assert_eq!(names, ["add", "gauge", "help", "long"]);
        assert_eq!(remote_cmd(&mut c, &["help", "-i", "4"]).0, 1);

        // Arguments are rejected once they no longer fit the page
        let big = "x".repeat(itf::REMOTE_CMD_ARGS_MAX_LEN);
        let mut encoded = big.into_bytes();
        encoded.push(0);
        assert!(c.write_string_page(itf::REMOTE_CMD_ARGS_PAGE, &encoded).is_err());
    }
}

#[test]
fn debug_safety_and_faults() {
    for link in LINKS {
        let mut c = debug_client(link);
        let page = itf::SAFETY_STATUS_PAGE;
        let state = c.read_register(page, safety_status::GLOBAL_STATE).unwrap();
        assert_eq!(state, 1 << safety_status::GLOBAL_STATE_IS_SETUP_BIT | 1 << safety_status::GLOBAL_STATE_FAULT_PRESENT_BIT);
        assert_eq!(c.read_register(page, safety_status::FAULT_LIST).unwrap(), 0b100);
        assert_eq!(c.read_register(page, safety_status::UPTIME).unwrap(), 1234);

        c.write_register(page, safety_status::RAISE_FAULT, 7).unwrap();
        c.write_register(page, safety_status::LOWER_FAULT, 2).unwrap();
        assert_eq!(c.read_register(page, safety_status::FAULT_LIST).unwrap(), 0b1000_0000);
        assert_eq!(c.write_register(page, safety_status::RAISE_FAULT, 32), Err(ClientError::Server(ServerError::InvalidData)));
        assert_eq!(device(&mut c).chip.raised, [7]);
        assert_eq!(device(&mut c).chip.lowered, [2]);

        // The fault name and filename pages only exist once a fault is selected
        assert!(c.read_register(itf::FAULT_NAME_PAGE, 0).is_err());
        assert_eq!(c.read_register(page, safety_status::FAULT_IDX).unwrap(), u32::MAX);
        c.write_register(page, safety_status::FAULT_IDX, 2).unwrap();
        assert_eq!(c.read_register(page, safety_status::FAULT_IDX).unwrap(), 2);
        assert_eq!(string_page(&mut c, itf::FAULT_NAME_PAGE), "FAULT_CAN_INTERNAL_ERROR");
        assert_eq!(string_page(&mut c, itf::FAULT_FILENAME_PAGE), "firmware/titan_rp2040/src/canbus.rs");
        let data = itf::FAULT_DATA_PAGE;
        assert_eq!(c.read_register(data, fault_data::STICKY).unwrap(), 1);
        assert_eq!(c.read_register(data, fault_data::TIME_LOWER).unwrap(), 5);
        assert_eq!(c.read_register(data, fault_data::TIME_UPPER).unwrap(), 1);
        assert_eq!(c.read_register(data, fault_data::EXTRA_DATA).unwrap(), 0xBEEF);
        assert_eq!(c.read_register(data, fault_data::LINE_NUMBER).unwrap(), 172);
        assert_eq!(c.read_register(data, fault_data::MULTIPLE_FIRES).unwrap(), 1);

        // "FAULT_WATCHDOG_RESET" is 20 bytes: the terminator must still be there
        c.write_register(page, safety_status::FAULT_IDX, 0).unwrap();
        assert_eq!(string_page(&mut c, itf::FAULT_NAME_PAGE), "FAULT_WATCHDOG_RESET");
        assert_eq!(string_page(&mut c, itf::FAULT_FILENAME_PAGE), "");
        assert_eq!(c.read_register(data, fault_data::STICKY).unwrap(), 0);
        assert_eq!(c.write_register(page, safety_status::FAULT_IDX, 32), Err(ClientError::Server(ServerError::InvalidData)));
    }
}

#[test]
fn debug_crash_log() {
    for link in LINKS {
        let mut c = debug_client(link);
        let page = itf::CRASH_LOG_PAGE;
        let counts = c.read_register(page, crash_log::CRASH_COUNT).unwrap();
        assert_eq!(counts, 3 | 1 << 8 | 0xFF << 24);
        assert_eq!(c.read_register(page, crash_log::HAS_WRAPPED).unwrap(), 0);
        assert_eq!(c.read_register(page, crash_log::PREV_COUNT).unwrap(), 2);
        c.write_register(page, crash_log::PREV_INDEX, 0).unwrap();
        assert_eq!(c.read_register(page, crash_log::RESET_REASON).unwrap(), crash_log::RESET_REASON_PANIC);
        assert_eq!(c.read_register(page, crash_log::PREV_FAULT_LIST).unwrap(), 5);
        assert_eq!(c.read_register(page, crash_log::PREV_UPTIME).unwrap(), 42);
        c.write_register(page, crash_log::PREV_INDEX, 1).unwrap();
        assert_eq!(c.read_register(page, crash_log::PREV_INDEX).unwrap(), 1);
        assert_eq!(c.read_register(page, crash_log::PREV_SCRATCH_1).unwrap(), crash_log::CLEAN_RESET_POR);
        assert_eq!(c.write_register(page, crash_log::PREV_INDEX, 2), Err(ClientError::Server(ServerError::InvalidData)));
        // A failed selection keeps the previous one
        assert_eq!(c.read_register(page, crash_log::PREV_INDEX).unwrap(), 1);
    }
}

#[test]
fn debug_gdb_stub() {
    for link in LINKS {
        let mut c = debug_client(link);
        let page = itf::GDB_STUB_PAGE;
        // GdbTarget::write_memory / read_memory
        c.write_register(page, gdb_stub::MEMORY_DATA, 0x1234_5678).unwrap();
        c.write_register(page, gdb_stub::WRITE_WORD_ADDR, 0x2000_0010).unwrap();
        c.write_register(page, gdb_stub::MEMORY_DATA, 0).unwrap();
        c.write_register(page, gdb_stub::READ_WORD_ADDR, 0x2000_0010).unwrap();
        assert_eq!(c.read_register(page, gdb_stub::MEMORY_DATA).unwrap(), 0x1234_5678);

        // Flash reads are redirected to the no-allocate alias
        c.write_register(page, gdb_stub::READ_WORD_ADDR, 0x1000_0100).unwrap();
        assert_eq!(c.read_register(page, gdb_stub::MEMORY_DATA).unwrap(), 0x1100_0100 ^ 0xA5A5_A5A5);

        let invalid = Err(ClientError::Server(ServerError::InvalidData));
        assert_eq!(c.write_register(page, gdb_stub::WRITE_WORD_ADDR, 0x1000_0100), invalid);
        assert_eq!(c.write_register(page, gdb_stub::WRITE_WORD_ADDR, 0x2000_0002), invalid);
        assert_eq!(c.write_register(page, gdb_stub::READ_WORD_ADDR, 0x3000_0000), invalid);
        assert_eq!(c.write_register(page, gdb_stub::READ_WORD_ADDR, 0x1020_0000), invalid);
        assert_eq!(c.read_register(page, gdb_stub::READ_WORD_ADDR), Err(ClientError::Server(ServerError::InvalidRegisterMode)));

        assert_eq!(c.read_register(page, gdb_stub::PC_REGISTER).unwrap(), 0x1000_1234);
        assert_eq!(c.read_register(page, gdb_stub::SP_REGISTER).unwrap(), 0x2003_FF00);
        assert_eq!(c.read_register(page, gdb_stub::LR_REGISTER).unwrap(), 0x1000_5679);
    }
}

#[test]
fn execute_request_detection() {
    let mode = ControlInterfaceMode::Normal as u8;
    let req = [mode << 5, 0, itf::REMOTE_CMD_PAGE, itf::remote_cmd::EXECUTE];
    assert!(is_execute_request(&req, mode, false));
    assert!(!is_execute_request(&req, mode, true));
    assert!(!is_execute_request(&req, ControlInterfaceMode::Bootloader as u8, false));
    assert!(!is_execute_request(&[mode << 5 | 1, 0, itf::REMOTE_CMD_PAGE, 0, 0, 0, 0, 0], mode, false));
    assert!(!is_execute_request(&[mode << 5, 0, itf::REMOTE_CMD_PAGE, 1], mode, false));
}

// ================================================================ Bootloader interface

struct FakeFlash {
    mem: Vec<u8>,
    erases: Vec<u32>,
    programs: Vec<u32>,
}

impl FakeFlash {
    fn new() -> Self {
        FakeFlash { mem: vec![0x5A; FLASH_SIZE as usize], erases: Vec::new(), programs: Vec::new() }
    }
}

impl FlashBackend for FakeFlash {
    fn read(&mut self, offset: u32, out: &mut [u8; 256]) {
        out.copy_from_slice(&self.mem[offset as usize..offset as usize + 256]);
    }
    fn program(&mut self, offset: u32, data: &[u8; 256]) {
        self.programs.push(offset);
        for (m, d) in self.mem[offset as usize..offset as usize + 256].iter_mut().zip(data) {
            // NOR flash only clears bits
            *m &= d;
        }
    }
    fn erase(&mut self, offset: u32) {
        self.erases.push(offset);
        self.mem[offset as usize..offset as usize + 4096].fill(0xFF);
    }
    fn read_word(&mut self, addr: u32) -> u32 {
        let o = crate::memory::flash_offset(addr) as usize;
        u32::from_le_bytes(self.mem[o..o + 4].try_into().unwrap())
    }
    fn write_word(&mut self, _addr: u32, _value: u32) {}
    fn stub_registers(&mut self) -> (u32, u32, u32) {
        (1, 2, 3)
    }
}

const BL_INFO: BootloaderInfo = BootloaderInfo {
    version_string: b"titan_bootloader 1.0-talos/ivc_board\0",
    major: bootloader::VERSION_MAJOR,
    minor: bootloader::VERSION_MINOR,
    release_type: itf::release_type::CLEAN,
    flash_id: 0x0102_0304_0506_0708,
    flash_size: FLASH_SIZE,
    app_base: bootloader::APP_BASE,
};

struct BlDevice {
    server: Server,
    flash: FakeFlash,
    state: BootloaderState,
}

impl Device for BlDevice {
    fn handle(&mut self, req: &[u8], resp: &mut [u8]) -> usize {
        let mut map = BootloaderMap::new(&BL_INFO, &mut self.flash, &mut self.state);
        self.server.handle_request(&mut map, req, resp)
    }
}

type BlClient = Client<Loopback<BlDevice>>;

fn bl_client(link: Link) -> BlClient {
    let device = BlDevice {
        server: link.server(ControlInterfaceMode::Bootloader),
        flash: FakeFlash::new(),
        state: BootloaderState::new(),
    };
    client(device, link, ControlInterfaceMode::Bootloader)
}

/// `BootloaderClient::{erase_sector, write_page, verify_page, read_page}`.
fn bl_erase(c: &mut BlClient, addr: u32) -> Result<(), ClientError<Timeout>> {
    c.write_register(bl::FLASH_CONTROL_PAGE, flash_control::TARGET_ADDR, addr)?;
    c.write_register(bl::FLASH_CONTROL_PAGE, flash_control::COMMAND, flash_control::COMMAND_ERASE)
}

fn bl_write(c: &mut BlClient, addr: u32, data: &[u8; 256]) -> Result<(), ClientError<Timeout>> {
    c.write_bytes(bl::FLASH_BUFFER_PAGE, 0, data)?;
    c.write_register(bl::FLASH_CONTROL_PAGE, flash_control::TARGET_ADDR, addr)?;
    c.write_register(bl::FLASH_CONTROL_PAGE, flash_control::COMMAND, flash_control::COMMAND_WRITE)
}

fn bl_verify(c: &mut BlClient, addr: u32, data: &[u8; 256]) -> bool {
    c.write_register(bl::FLASH_CONTROL_PAGE, flash_control::TARGET_ADDR, addr).unwrap();
    c.write_register(bl::FLASH_CONTROL_PAGE, flash_control::COMMAND, flash_control::COMMAND_READ).unwrap();
    c.write_register(bl::FLASH_CONTROL_PAGE, flash_control::COMMAND, flash_control::COMMAND_CRC).unwrap();
    c.read_register(bl::FLASH_CONTROL_PAGE, flash_control::CRC).unwrap() == crate::crc32::compute(data)
}

fn bl_read(c: &mut BlClient, addr: u32) -> [u8; 256] {
    c.write_register(bl::FLASH_CONTROL_PAGE, flash_control::TARGET_ADDR, addr).unwrap();
    c.write_register(bl::FLASH_CONTROL_PAGE, flash_control::COMMAND, flash_control::COMMAND_READ).unwrap();
    let mut page = [0u8; 256];
    c.read_bytes(bl::FLASH_BUFFER_PAGE, 0, &mut page).unwrap();
    page
}

/// An application image: a boot2 with a valid checksum followed by `pages - 1` pages of pattern.
fn app_image(pages: usize) -> Vec<[u8; 256]> {
    let mut image: Vec<[u8; 256]> =
        (0..pages).map(|p| core::array::from_fn(|i| (p * 7 + i * 13) as u8)).collect();
    let crc = crate::crc32::compute(&image[0][..252]);
    image[0][252..].copy_from_slice(&crc.to_le_bytes());
    image
}

#[test]
fn bootloader_connect() {
    for link in LINKS {
        let mut c = bl_client(link);
        let page = bl::MCU_CONTROL_PAGE;
        use bl::mcu_control;
        assert_eq!(c.read_register(page, mcu_control::MAGIC).unwrap(), mcu_control::MAGIC_VALUE);
        assert_eq!(c.read_register(page, mcu_control::MAJOR_VERSION).unwrap(), bl::EXPECTED_VERSION_MAJOR);
        assert_eq!(c.read_register(page, mcu_control::MINOR_VERSION).unwrap(), bl::EXPECTED_VERSION_MINOR);
        assert_eq!(c.read_register(page, mcu_control::RELEASE_TYPE).unwrap(), itf::release_type::CLEAN);
        assert_eq!(c.read_register(page, mcu_control::LOWER_FLASH_ID).unwrap(), 0x0506_0708);
        assert_eq!(c.read_register(page, mcu_control::UPPER_FLASH_ID).unwrap(), 0x0102_0304);
        assert_eq!(c.read_register(bl::FLASH_CONTROL_PAGE, flash_control::FLASH_SIZE).unwrap(), FLASH_SIZE);
        assert_eq!(c.read_register(bl::FLASH_CONTROL_PAGE, flash_control::APP_BASE).unwrap(), 0x1000_4000);
        c.write_register(bl::FLASH_CONTROL_PAGE, flash_control::BL_WRITE_KEY, 0).unwrap();
        let mut buf = [0u8; 256];
        let len = c.read_string_page(bl::VERSION_STRING_PAGE, &mut buf).unwrap();
        assert_eq!(&buf[..len], b"titan_bootloader 1.0-talos/ivc_board");

        // A request for the debug interface mode is refused
        c.set_mode(ControlInterfaceMode::Normal as u8);
        assert_eq!(c.read_register(page, 0), Err(ClientError::Server(ServerError::InvalidMode)));
        c.set_mode(ControlInterfaceMode::Bootloader as u8);

        c.write_register(page, mcu_control::REBOOT_MCU, 1).unwrap();
        assert!(c.transport_mut().device.state.reboot);
    }
}

#[test]
fn bootloader_flash_app() {
    for link in LINKS {
        let mut c = bl_client(link);
        let image = app_image(20);
        // flash_ui::flash_image order: everything but the first page, then the first page (boot2) last
        let mut erased = std::collections::HashSet::new();
        let order: Vec<usize> = (1..image.len()).chain([0]).collect();
        for &p in &order {
            let addr = bootloader::APP_BASE + (p * 256) as u32;
            let sector = addr & !0xFFF;
            if erased.insert(sector) {
                bl_erase(&mut c, sector).unwrap();
            }
            bl_write(&mut c, addr, &image[p]).unwrap();
        }
        for (p, data) in image.iter().enumerate() {
            let addr = bootloader::APP_BASE + (p * 256) as u32;
            assert!(bl_verify(&mut c, addr, data), "page {p} on {link:?}");
            assert_eq!(&bl_read(&mut c, addr), data);
        }
        let flash = &c.transport_mut().device.flash;
        assert_eq!(flash.erases, [0x4000, 0x5000]);
        let first: [u8; 256] = flash.mem[0x4000..0x4100].try_into().unwrap();
        assert!(bootloader::app_boot2_valid(&first));

        // A corrupted page fails verification
        assert!(!bl_verify(&mut c, bootloader::APP_BASE + 256, &image[2]));
    }
}

#[test]
fn bootloader_protection() {
    for link in LINKS {
        let mut c = bl_client(link);
        let invalid = Err(ClientError::Server(ServerError::InvalidData));
        // The bootloader region is locked until the key is written
        assert_eq!(bl_erase(&mut c, 0x1000_0000), invalid);
        assert_eq!(bl_write(&mut c, 0x1000_3F00, &[0; 256]), invalid);
        // Alignment and bounds
        assert_eq!(bl_erase(&mut c, 0x1000_4100), invalid);
        assert_eq!(bl_erase(&mut c, 0x1020_0000), invalid);
        assert_eq!(bl_write(&mut c, 0x1000_4080, &[0; 256]), invalid);
        assert_eq!(bl_write(&mut c, 0x101F_FF00 + 0x100, &[0; 256]), invalid);
        assert_eq!(
            c.write_register(bl::FLASH_CONTROL_PAGE, flash_control::COMMAND, 9),
            invalid,
            "unknown command"
        );
        assert_eq!(c.read_register(bl::FLASH_CONTROL_PAGE, flash_control::COMMAND), Err(ClientError::Server(ServerError::InvalidRegisterMode)));
        // Reading the bootloader itself is allowed
        assert_eq!(bl_read(&mut c, 0x1000_0000), [0x5A; 256]);
        assert!(c.transport_mut().device.flash.erases.is_empty());

        c.write_register(bl::FLASH_CONTROL_PAGE, flash_control::BL_WRITE_KEY, flash_control::BL_WRITE_KEY_VALUE).unwrap();
        bl_erase(&mut c, 0x1000_0000).unwrap();
        bl_write(&mut c, 0x1000_0000, &[0x11; 256]).unwrap();
        assert_eq!(bl_read(&mut c, 0x1000_0000), [0x11; 256]);
        assert!(c.transport_mut().device.state.flash_activity);

        // gdb stub reads flash through the backend
        c.write_register(bl::GDB_STUB_PAGE, bl::gdb_stub::READ_WORD_ADDR, 0x1000_0000).unwrap();
        assert_eq!(c.read_register(bl::GDB_STUB_PAGE, bl::gdb_stub::MEMORY_DATA).unwrap(), 0x1111_1111);
    }
}

#[test]
fn bootloader_request_and_boot2() {
    assert!(bootloader::is_bootloader_request(&[0xB0, 0x07, 0x10, 0xAD]));
    assert!(!bootloader::is_bootloader_request(&[0xB0, 0x07, 0x10]));
    let image = app_image(1);
    assert!(bootloader::app_boot2_valid(&image[0]));
    let mut bad = image[0];
    bad[10] ^= 1;
    assert!(!bootloader::app_boot2_valid(&bad));
    assert!(!bootloader::app_boot2_valid(&[0xFF; 256]));
}

/// A map can be served by reference (what the firmware does, building a map per request).
#[test]
fn map_by_reference() {
    let mut chip = FakeChip::new();
    let mut state = DebugState::new();
    let mut map = DebugMap::new(&mut chip, &mut state);
    let map_ref: &mut dyn RegisterMap = &mut map;
    assert_eq!(map_ref.read(itf::MCU_CONTROL_PAGE, 0), Ok(mcu_control::MAGIC_VALUE));
}
