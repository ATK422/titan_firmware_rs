//! The bootloader, debug and Linux clients against simulated devices served with `canmore::reg_mapped::Server`
//! over UDP on localhost (the CANmore Ethernet transport).

use std::collections::HashMap;
use std::net::{Ipv4Addr, UdpSocket};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::JoinHandle;
use std::time::Duration;

use canmore::ControlInterfaceMode;
use canmore::reg_mapped::interface::{bootloader as bl, debug as dbg, linux as lx};
use canmore::reg_mapped::{Access, MemoryPage, RegisterMap, Server, ServerError};
use titan_tools::canmore::udp::{UDP_MAX_LEN, UdpTransport};
use titan_tools::canmore::{BootloaderClient, DebugClient, LinkTransport, LinuxClient, RegClient};
use titan_tools::crc32;
use titan_tools::flash::{EraseWrapper, FLASH_BASE, FlashInterface, PAGE_SIZE};

/// Serves `map` on a localhost UDP port until dropped.
struct UdpDevice {
    port: u16,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl UdpDevice {
    fn spawn<M: RegisterMap + Send + 'static>(mode: ControlInterfaceMode, mut map: M) -> Self {
        let socket = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
        socket.set_read_timeout(Some(Duration::from_millis(20))).unwrap();
        let port = socket.local_addr().unwrap().port();
        let stop = Arc::new(AtomicBool::new(false));
        let thread_stop = stop.clone();
        let thread = std::thread::spawn(move || {
            let mut server = Server::new(mode as u8);
            let mut req = [0u8; UDP_MAX_LEN];
            let mut resp = [0u8; UDP_MAX_LEN];
            while !thread_stop.load(Ordering::Relaxed) {
                let Ok((len, from)) = socket.recv_from(&mut req) else { continue };
                let resp_len = server.handle_request(&mut map, &req[..len], &mut resp);
                if resp_len > 0 {
                    socket.send_to(&resp[..resp_len], from).unwrap();
                }
            }
        });
        UdpDevice { port, stop, thread: Some(thread) }
    }

    fn client(&self, mode: ControlInterfaceMode) -> RegClient {
        RegClient::new(LinkTransport::Udp(UdpTransport::open(Ipv4Addr::LOCALHOST, self.port).unwrap()), mode as u8)
    }
}

impl Drop for UdpDevice {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        let _ = self.thread.take().unwrap().join();
    }
}

const FLASH_ID: u64 = 0x322A_0D63_B014_62E4;
const FLASH_SIZE: usize = 64 * 1024;
const APP_BASE: u32 = FLASH_BASE + 0x4000;

/// A bootloader with 64 KiB of flash, the first 16 KiB of which hold the (write protected) bootloader.
struct FakeBootloader {
    flash: Vec<u8>,
    buffer: [u8; bl::FLASH_BUFFER_SIZE],
    target: u32,
    key: u32,
    crc: u32,
    rebooted: bool,
    version: &'static [u8],
}

impl FakeBootloader {
    fn new() -> Self {
        FakeBootloader {
            flash: vec![0xFF; FLASH_SIZE],
            buffer: [0; bl::FLASH_BUFFER_SIZE],
            target: 0,
            key: 0,
            crc: 0,
            rebooted: false,
            version: b"titan_bootloader 1.0.0",
        }
    }

    fn flash_offset(&self, align: u32, len: usize) -> Result<usize, ServerError> {
        if self.target < FLASH_BASE || self.target & align != 0 {
            return Err(ServerError::InvalidData);
        }
        let off = (self.target - FLASH_BASE) as usize;
        if off + len > self.flash.len() {
            return Err(ServerError::InvalidData);
        }
        Ok(off)
    }

    fn check_writable(&self, off: usize) -> Result<(), ServerError> {
        if off < (APP_BASE - FLASH_BASE) as usize && self.key != bl::flash_control::BL_WRITE_KEY_VALUE {
            return Err(ServerError::InvalidRegisterAddress);
        }
        Ok(())
    }

    fn command(&mut self, cmd: u32) -> Result<(), ServerError> {
        use bl::flash_control::*;
        match cmd {
            COMMAND_READ => {
                let off = self.flash_offset(bl::FLASH_READ_ADDR_ALIGN_MASK, PAGE_SIZE)?;
                self.buffer.copy_from_slice(&self.flash[off..off + PAGE_SIZE]);
            }
            COMMAND_WRITE => {
                let off = self.flash_offset(bl::FLASH_WRITE_ADDR_ALIGN_MASK, PAGE_SIZE)?;
                self.check_writable(off)?;
                // NOR flash can only clear bits
                for (f, b) in self.flash[off..off + PAGE_SIZE].iter_mut().zip(&self.buffer) {
                    *f &= *b;
                }
            }
            COMMAND_ERASE => {
                let off = self.flash_offset(bl::FLASH_ERASE_ADDR_ALIGN_MASK, bl::FLASH_ERASE_SIZE)?;
                self.check_writable(off)?;
                self.flash[off..off + bl::FLASH_ERASE_SIZE].fill(0xFF);
            }
            COMMAND_CRC => self.crc = crc32::compute(&self.buffer),
            _ => return Err(ServerError::InvalidData),
        }
        Ok(())
    }
}

impl RegisterMap for FakeBootloader {
    fn read(&mut self, page: u8, offset: u8) -> Result<u32, ServerError> {
        use bl::{flash_control as fc, mcu_control as mc};
        match (page, offset) {
            (bl::MCU_CONTROL_PAGE, mc::MAGIC) => Ok(mc::MAGIC_VALUE),
            (bl::MCU_CONTROL_PAGE, mc::MAJOR_VERSION) => Ok(1),
            (bl::MCU_CONTROL_PAGE, mc::MINOR_VERSION) => Ok(0),
            (bl::MCU_CONTROL_PAGE, mc::RELEASE_TYPE) => Ok(dbg::release_type::TAGGED),
            (bl::MCU_CONTROL_PAGE, mc::LOWER_FLASH_ID) => Ok(FLASH_ID as u32),
            (bl::MCU_CONTROL_PAGE, mc::UPPER_FLASH_ID) => Ok((FLASH_ID >> 32) as u32),
            (bl::VERSION_STRING_PAGE, _) => MemoryPage::Bytes(Access::ReadOnly(self.version)).read(offset),
            (bl::FLASH_CONTROL_PAGE, fc::TARGET_ADDR) => Ok(self.target),
            (bl::FLASH_CONTROL_PAGE, fc::BL_WRITE_KEY) => Ok(self.key),
            (bl::FLASH_CONTROL_PAGE, fc::CRC) => Ok(self.crc),
            (bl::FLASH_CONTROL_PAGE, fc::FLASH_SIZE) => Ok(FLASH_SIZE as u32),
            (bl::FLASH_CONTROL_PAGE, fc::APP_BASE) => Ok(APP_BASE),
            (bl::FLASH_BUFFER_PAGE, _) => MemoryPage::Bytes(Access::ReadWrite(&mut self.buffer)).read(offset),
            (bl::MCU_CONTROL_PAGE, mc::REBOOT_MCU) | (bl::FLASH_CONTROL_PAGE, fc::COMMAND) => {
                Err(ServerError::InvalidRegisterMode)
            }
            _ => Err(ServerError::InvalidRegisterAddress),
        }
    }

    fn write(&mut self, page: u8, offset: u8, data: u32) -> Result<(), ServerError> {
        use bl::{flash_control as fc, mcu_control as mc};
        match (page, offset) {
            (bl::MCU_CONTROL_PAGE, mc::REBOOT_MCU) => self.rebooted = true,
            (bl::FLASH_CONTROL_PAGE, fc::COMMAND) => self.command(data)?,
            (bl::FLASH_CONTROL_PAGE, fc::TARGET_ADDR) => self.target = data,
            (bl::FLASH_CONTROL_PAGE, fc::BL_WRITE_KEY) => self.key = data,
            (bl::FLASH_BUFFER_PAGE, _) => MemoryPage::Bytes(Access::ReadWrite(&mut self.buffer)).write(offset, data)?,
            (bl::MCU_CONTROL_PAGE, _) | (bl::VERSION_STRING_PAGE, _) | (bl::FLASH_CONTROL_PAGE, 3..=5) => {
                return Err(ServerError::InvalidRegisterMode);
            }
            _ => return Err(ServerError::InvalidRegisterAddress),
        }
        Ok(())
    }

    fn read_multiword(&mut self, page: u8, offset: u8, out: &mut [u8]) -> Result<(), ServerError> {
        match page {
            bl::FLASH_BUFFER_PAGE => MemoryPage::Bytes(Access::ReadWrite(&mut self.buffer)).read_multiword(offset, out),
            bl::VERSION_STRING_PAGE => MemoryPage::Bytes(Access::ReadOnly(self.version)).read_multiword(offset, out),
            _ => Err(ServerError::InvalidRegisterAddress),
        }
    }

    fn write_multiword(&mut self, page: u8, offset: u8, data: &[u8]) -> Result<(), ServerError> {
        match page {
            bl::FLASH_BUFFER_PAGE => {
                MemoryPage::Bytes(Access::ReadWrite(&mut self.buffer)).write_multiword(offset, data)
            }
            _ => Err(ServerError::InvalidRegisterAddress),
        }
    }
}

#[test]
fn bootloader_flash_round_trip() {
    let device = UdpDevice::spawn(ControlInterfaceMode::Bootloader, FakeBootloader::new());
    let mut client = BootloaderClient::new(device.client(ControlInterfaceMode::Bootloader)).unwrap();
    assert!(client.warnings.is_empty());
    assert_eq!(client.flash_id(), FLASH_ID);
    assert_eq!(client.flash_size(), FLASH_SIZE as u32);
    assert_eq!(client.bootloader_size(), 0x4000);
    assert_eq!(client.version().unwrap(), "titan_bootloader 1.0.0");

    // Write 5 pages across a sector boundary after the bootloader, like the upload tool does
    let pages: Vec<[u8; PAGE_SIZE]> = (0..5u8).map(|i| [i.wrapping_mul(37) ^ 0x5A; PAGE_SIZE]).collect();
    let base = APP_BASE + 0xE00;
    {
        let mut wrapper = EraseWrapper::new(&mut client);
        for (i, page) in pages.iter().enumerate() {
            wrapper.write_page(base + (i * PAGE_SIZE) as u32, page).unwrap();
        }
    }
    for (i, page) in pages.iter().enumerate() {
        let addr = base + (i * PAGE_SIZE) as u32;
        assert!(client.verify_page(addr, page).unwrap());
        assert_eq!(&client.read_page(addr).unwrap(), page);
    }
    assert!(!client.verify_page(base, &[0u8; PAGE_SIZE]).unwrap());

    // The bootloader region is locked until the write key is set
    let err = client.erase_sector(FLASH_BASE).unwrap_err();
    assert!(format!("{err:#}").contains("INVALID_REGISTER_ADDRESS"), "{err:#}");
    client.enable_bootloader_overwrite().unwrap();
    client.erase_sector(FLASH_BASE).unwrap();

    // Out of range addresses are rejected locally
    assert!(client.read_page(FLASH_BASE + FLASH_SIZE as u32).is_err());
    assert!(client.write_page(APP_BASE + 1, &pages[0]).is_err());

    client.reboot().unwrap();
}

/// An application exposing the debug interface's identification and safety registers.
struct FakeApp {
    version: &'static [u8],
    args: [u8; canmore::reg_mapped::PAGE_SIZE],
    resp: Vec<u8>,
    gdb_addr_data: (u32, u32),
    memory: HashMap<u32, u32>,
}

impl RegisterMap for FakeApp {
    fn read(&mut self, page: u8, offset: u8) -> Result<u32, ServerError> {
        use dbg::{gdb_stub as gs, mcu_control as mc, safety_status as ss};
        match (page, offset) {
            (dbg::MCU_CONTROL_PAGE, mc::MAGIC) => Ok(mc::MAGIC_VALUE),
            (dbg::MCU_CONTROL_PAGE, mc::LOWER_FLASH_ID) => Ok(FLASH_ID as u32),
            (dbg::MCU_CONTROL_PAGE, mc::UPPER_FLASH_ID) => Ok((FLASH_ID >> 32) as u32),
            (dbg::VERSION_STRING_PAGE, _) => MemoryPage::Bytes(Access::ReadOnly(self.version)).read(offset),
            (dbg::SAFETY_STATUS_PAGE, ss::GLOBAL_STATE) => Ok(0b0011),
            (dbg::SAFETY_STATUS_PAGE, ss::UPTIME) => Ok(12_345),
            (dbg::REMOTE_CMD_PAGE, dbg::remote_cmd::EXECUTE) => {
                // Echo the arguments back, space separated, and return the argument count
                let args: Vec<&[u8]> = self.args.split(|&b| b == 0).take_while(|a| !a.is_empty()).collect();
                let count = args.len() as u32;
                self.resp = args.join(&b' ');
                self.args[0] = 0;
                Ok(count)
            }
            (dbg::REMOTE_CMD_RESP_PAGE, _) => MemoryPage::Bytes(Access::ReadOnly(&self.resp)).read(offset),
            (dbg::GDB_STUB_PAGE, gs::MEMORY_DATA) => Ok(self.gdb_addr_data.1),
            _ => Err(ServerError::InvalidRegisterAddress),
        }
    }

    fn write(&mut self, page: u8, offset: u8, data: u32) -> Result<(), ServerError> {
        use dbg::gdb_stub as gs;
        match (page, offset) {
            (dbg::REMOTE_CMD_ARGS_PAGE, _) => MemoryPage::Bytes(Access::ReadWrite(&mut self.args)).write(offset, data),
            (dbg::GDB_STUB_PAGE, gs::READ_WORD_ADDR) => {
                self.gdb_addr_data.1 = *self.memory.get(&data).ok_or(ServerError::InvalidData)?;
                Ok(())
            }
            (dbg::GDB_STUB_PAGE, gs::MEMORY_DATA) => {
                self.gdb_addr_data.1 = data;
                Ok(())
            }
            (dbg::GDB_STUB_PAGE, gs::WRITE_WORD_ADDR) => {
                self.memory.insert(data, self.gdb_addr_data.1);
                Ok(())
            }
            _ => Err(ServerError::InvalidRegisterAddress),
        }
    }

    fn write_multiword(&mut self, page: u8, offset: u8, data: &[u8]) -> Result<(), ServerError> {
        match page {
            dbg::REMOTE_CMD_ARGS_PAGE => {
                MemoryPage::Bytes(Access::ReadWrite(&mut self.args)).write_multiword(offset, data)
            }
            _ => Err(ServerError::InvalidRegisterAddress),
        }
    }
}

#[test]
fn debug_client() {
    use titan_tools::canmore::gdb::GdbTarget;
    let app = FakeApp {
        version: b"power_board 1.2.3",
        args: [0; canmore::reg_mapped::PAGE_SIZE],
        resp: Vec::new(),
        gdb_addr_data: (0, 0),
        memory: HashMap::from([(0x2000_0000, 0xDEAD_BEEF)]),
    };
    let device = UdpDevice::spawn(ControlInterfaceMode::Normal, app);
    let mut client = DebugClient::new(device.client(ControlInterfaceMode::Normal)).unwrap();
    assert_eq!(client.flash_id(), FLASH_ID);
    assert_eq!(client.version().unwrap(), "power_board 1.2.3");
    assert_eq!(client.safety_status().unwrap().global_state(), "ROS Connected");
    assert_eq!(client.uptime().unwrap().format_seconds(), "123.45");
    assert_eq!(client.execute_remote_cmd(&["echo", "hello", "world"]).unwrap(), (3, "echo hello world".into()));
    assert_eq!(client.read_memory(0x2000_0000).unwrap(), 0xDEAD_BEEF);
    client.write_memory(0x2000_0004, 7).unwrap();
    assert_eq!(client.read_memory(0x2000_0004).unwrap(), 7);
    assert!(client.read_memory(0x3000_0000).is_err());

    // The wrong mode is rejected by the device
    assert!(BootloaderClient::new(device.client(ControlInterfaceMode::Bootloader)).is_err());
}

/// The file transfer half of the Linux debug daemon, with an in-memory file system.
struct FakeLinux {
    files: HashMap<String, (Vec<u8>, u32)>,
    buffer: [u8; 1024],
    regs: [u32; 8],
    status: u32,
}

impl FakeLinux {
    fn filename(&self) -> String {
        let len = self.regs[lx::file_control::FILENAME_LENGTH as usize] as usize;
        String::from_utf8_lossy(&self.buffer[..len]).trim_end_matches('\0').to_owned()
    }

    fn operation(&mut self, op: u32) -> u32 {
        use lx::file_control::*;
        let name = self.filename();
        let regs = &mut self.regs;
        match op {
            OPERATION_NOP => return STATUS_READY,
            OPERATION_WRITE => {
                let len = regs[DATA_LENGTH as usize] as usize;
                if crc32::compute(&self.buffer[..len]) != regs[CRC as usize] {
                    return STATUS_FAIL_BAD_CRC;
                }
                let data_start = name.len().next_multiple_of(4);
                let file = self.files.entry(name).or_default();
                if regs[CLEAR as usize] != 0 {
                    file.0.clear();
                }
                file.0.extend_from_slice(&self.buffer[data_start..len]);
            }
            OPERATION_READ => {
                let Some((data, _)) = self.files.get(&name) else { return self.fail("no such file") };
                let start = (regs[READ_OFFSET as usize] as usize).min(data.len());
                let chunk = &data[start..(start + 1024).min(data.len())];
                self.buffer[..chunk.len()].copy_from_slice(chunk);
                regs[DATA_LENGTH as usize] = chunk.len() as u32;
                regs[CRC as usize] = crc32::compute(chunk);
            }
            OPERATION_GET_FILE_LEN => match self.files.get(&name) {
                Some((data, _)) => regs[DATA_LENGTH as usize] = data.len() as u32,
                None => return self.fail("no such file"),
            },
            OPERATION_CHECK_CRC => {
                let crc = self.files.get(&name).map(|f| crc32::compute(&f.0));
                if crc != Some(regs[CRC as usize]) {
                    return STATUS_FAIL_BAD_CRC;
                }
            }
            OPERATION_SET_MODE => self.files.get_mut(&name).unwrap().1 = regs[FILE_MODE as usize],
            OPERATION_GET_MODE => regs[FILE_MODE as usize] = self.files[&name].1,
            _ => return self.fail("unsupported operation"),
        }
        STATUS_SUCCESS
    }

    fn fail(&mut self, msg: &str) -> u32 {
        self.buffer[..msg.len()].copy_from_slice(msg.as_bytes());
        self.regs[lx::file_control::DATA_LENGTH as usize] = msg.len() as u32;
        lx::file_control::STATUS_FAIL_DEVICE_ERROR
    }
}

impl RegisterMap for FakeLinux {
    fn read(&mut self, page: u8, offset: u8) -> Result<u32, ServerError> {
        use lx::{file_control as fc, gen_control as gc};
        match (page, offset) {
            (lx::GEN_CONTROL_PAGE, gc::MAGIC) => Ok(gc::MAGIC_VALUE),
            (lx::GEN_CONTROL_PAGE, gc::LOWER_FLASH_ID) => Ok(1),
            (lx::GEN_CONTROL_PAGE, gc::UPPER_FLASH_ID) => Ok(2),
            (lx::FILE_CONTROL_PAGE, fc::STATUS) => Ok(self.status),
            (lx::FILE_CONTROL_PAGE, o) if o < 8 => Ok(self.regs[o as usize]),
            (lx::FILE_BUFFER_PAGE, _) => MemoryPage::Bytes(Access::ReadWrite(&mut self.buffer)).read(offset),
            _ => Err(ServerError::InvalidRegisterAddress),
        }
    }

    fn write(&mut self, page: u8, offset: u8, data: u32) -> Result<(), ServerError> {
        match (page, offset) {
            (lx::FILE_CONTROL_PAGE, lx::file_control::OPERATION) => {
                self.status = self.operation(data);
                Ok(())
            }
            (lx::FILE_CONTROL_PAGE, o) if o < 7 => {
                self.regs[o as usize] = data;
                Ok(())
            }
            (lx::FILE_BUFFER_PAGE, _) => MemoryPage::Bytes(Access::ReadWrite(&mut self.buffer)).write(offset, data),
            _ => Err(ServerError::InvalidRegisterAddress),
        }
    }

    fn read_multiword(&mut self, page: u8, offset: u8, out: &mut [u8]) -> Result<(), ServerError> {
        match page {
            lx::FILE_BUFFER_PAGE => MemoryPage::Bytes(Access::ReadWrite(&mut self.buffer)).read_multiword(offset, out),
            _ => Err(ServerError::InvalidRegisterAddress),
        }
    }

    fn write_multiword(&mut self, page: u8, offset: u8, data: &[u8]) -> Result<(), ServerError> {
        match page {
            lx::FILE_BUFFER_PAGE => {
                MemoryPage::Bytes(Access::ReadWrite(&mut self.buffer)).write_multiword(offset, data)
            }
            _ => Err(ServerError::InvalidRegisterAddress),
        }
    }
}

#[test]
fn linux_file_transfer() {
    let linux = FakeLinux { files: HashMap::new(), buffer: [0; 1024], regs: [0; 8], status: 0 };
    let device = UdpDevice::spawn(ControlInterfaceMode::Linux, linux);
    let mut client = LinuxClient::new(device.client(ControlInterfaceMode::Linux)).unwrap();
    assert_eq!(client.flash_id(), 0x2_0000_0001);

    // Several chunks, ending with a partial one; and the exact chunk size edge case
    for len in [0usize, 5, 3000, 1024 - 12] {
        let data: Vec<u8> = (0..len).map(|i| (i * 7 + len) as u8).collect();
        let mut last_progress = 0.0;
        client.upload(&data, "dir/file.bin", 0o100755, &mut |p| last_progress = p).unwrap();
        assert_eq!(last_progress, 1.0);
        let (downloaded, mode) = client.download("dir/file.bin", &mut |_| {}).unwrap();
        assert_eq!(downloaded, data, "len {len}");
        assert_eq!(mode, 0o100755);
    }

    let err = client.download("missing", &mut |_| {}).unwrap_err();
    assert!(format!("{err:#}").contains("no such file"), "{err:#}");
}
