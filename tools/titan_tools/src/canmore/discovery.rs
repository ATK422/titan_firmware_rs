//! Heartbeat based device discovery, ported from `canmore_dbg_cpp/{Discovery,CANDiscovery,EthernetDiscovery}.cpp`.
//!
//! Every CANmore client sends a heartbeat every 500 ms reporting its control interface mode. A [`Discovery`]
//! listens to the heartbeats of one CAN interface (or of the network) in a background thread and keeps the devices
//! heard within the last 1.2 s.

use std::net::IpAddr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, anyhow, bail};
use canmore::id::{self, Direction, Fields, FrameType};
use canmore::reg_mapped::interface::bootloader as bl_itf;
use canmore::{ControlInterfaceMode, Heartbeat};

use super::can::{CAN_EFF_FLAG, CAN_RTR_FLAG, CanBus};
use super::{BootloaderClient, DebugClient, Link, LinuxClient, RegClient, flash_id_from_words, udp};
use crate::binary_info;
use crate::flash::{self, FlashInterface};

/// Devices not heard from for this long are dropped.
const STALE_DISCOVERY: Duration = Duration::from_millis(1200);
/// How long to wait for a device to come back after switching mode.
pub const CAN_MODE_SWITCH_TIMEOUT: Duration = Duration::from_millis(1500);
pub const ETH_MODE_SWITCH_TIMEOUT: Duration = Duration::from_millis(5000);

/// The control interface mode a device reports in its heartbeat.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DeviceMode {
    None,
    Normal,
    Bootloader,
    Linux,
    BootDelay,
    Unknown(u8),
}

impl DeviceMode {
    pub fn from_heartbeat(mode: u8) -> Self {
        match mode {
            m if m == ControlInterfaceMode::None as u8 => DeviceMode::None,
            m if m == ControlInterfaceMode::Normal as u8 => DeviceMode::Normal,
            m if m == ControlInterfaceMode::Bootloader as u8 => DeviceMode::Bootloader,
            m if m == ControlInterfaceMode::Linux as u8 => DeviceMode::Linux,
            m if m == ControlInterfaceMode::BootDelay as u8 => DeviceMode::BootDelay,
            m => DeviceMode::Unknown(m),
        }
    }

    /// Display name, as in the C++ tools.
    pub fn name(&self) -> String {
        match self {
            DeviceMode::Normal => "Application".into(),
            DeviceMode::Bootloader => "Bootloader".into(),
            DeviceMode::Linux => "Linux Machine".into(),
            DeviceMode::BootDelay => "Boot Delay".into(),
            DeviceMode::None => "No Control Interface".into(),
            DeviceMode::Unknown(m) => format!("Unknown ({m})"),
        }
    }

    /// True if the mode has a control interface that can report the flash ID.
    pub fn supports_flash_id(&self) -> bool {
        matches!(self, DeviceMode::Normal | DeviceMode::Bootloader | DeviceMode::Linux)
    }
}

#[derive(Clone)]
struct Record {
    link: Link,
    client_id: u8,
    heartbeat: Heartbeat,
    seen: Instant,
}

#[derive(Default)]
struct Shared {
    records: Mutex<Vec<Record>>,
    notify: Condvar,
    stop: AtomicBool,
}

impl Shared {
    fn report(&self, record: Record) {
        let mut records = self.records.lock().unwrap();
        records.retain(|r| !(r.link == record.link && r.client_id == record.client_id));
        records.push(record);
        records.retain(|r| r.seen.elapsed() < STALE_DISCOVERY);
        self.notify.notify_all();
    }
}

/// Listens for heartbeats on one CAN interface or on UDP.
pub struct Discovery {
    name: String,
    mode_switch_timeout: Duration,
    shared: Arc<Shared>,
    thread: Option<JoinHandle<()>>,
}

impl Discovery {
    /// Listens on the CAN interface `interface`.
    pub fn can(interface: &str) -> Result<Arc<Self>> {
        let bus = CanBus::open(interface)?;
        // All heartbeats: match type, direction and channel but not the client ID
        let filter_id = id::util_id(0, Direction::ClientToAgent, canmore::CHAN_HEARTBEAT);
        let filter_mask = CAN_EFF_FLAG | CAN_RTR_FLAG | (1 << 5) | (1 << 4) | 0xF;
        bus.set_filters(&[(filter_id, filter_mask)])?;

        let shared = Arc::new(Shared::default());
        let thread_shared = shared.clone();
        let link = Link::Can(interface.to_owned());
        let thread = std::thread::Builder::new().name(format!("discovery-{interface}")).spawn(move || {
            while !thread_shared.stop.load(Ordering::Relaxed) {
                let frame = match bus.read(Duration::from_millis(200)) {
                    Ok(Some(frame)) => frame,
                    Ok(None) => continue,
                    Err(e) => {
                        eprintln!("CAN discovery on {} stopped: {e}", bus.name());
                        return;
                    }
                };
                let fields = Fields::decode(frame.id, frame.extended);
                if frame.extended
                    || fields.frame_type != FrameType::Util
                    || frame.data.len() != 1
                    || fields.client_id == 0
                {
                    continue;
                }
                thread_shared.report(Record {
                    link: link.clone(),
                    client_id: fields.client_id,
                    heartbeat: Heartbeat::from_byte(frame.data[0]),
                    seen: Instant::now(),
                });
            }
        })?;
        Ok(Arc::new(Discovery {
            name: interface.to_owned(),
            mode_switch_timeout: CAN_MODE_SWITCH_TIMEOUT,
            shared,
            thread: Some(thread),
        }))
    }

    /// Listens for heartbeats broadcast on the network.
    pub fn udp() -> Result<Arc<Self>> {
        let socket = udp::heartbeat_socket(Duration::from_millis(200))?;
        let shared = Arc::new(Shared::default());
        let thread_shared = shared.clone();
        let thread = std::thread::Builder::new().name("discovery-udp".into()).spawn(move || {
            let mut buf = [0u8; 16];
            while !thread_shared.stop.load(Ordering::Relaxed) {
                let (len, from) = match socket.recv_from(&mut buf) {
                    Ok(r) => r,
                    Err(e) if matches!(e.kind(), std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut) => {
                        continue;
                    }
                    Err(e) => {
                        eprintln!("UDP discovery stopped: {e}");
                        return;
                    }
                };
                let IpAddr::V4(addr) = from.ip() else { continue };
                if len != 1 {
                    continue;
                }
                // Ethernet devices use the last byte of their address as the client ID
                thread_shared.report(Record {
                    link: Link::Udp(addr),
                    client_id: addr.octets()[3],
                    heartbeat: Heartbeat::from_byte(buf[0]),
                    seen: Instant::now(),
                });
            }
        })?;
        Ok(Arc::new(Discovery {
            name: "UDP".into(),
            mode_switch_timeout: ETH_MODE_SWITCH_TIMEOUT,
            shared,
            thread: Some(thread),
        }))
    }

    /// Opens discovery on UDP and every CAN interface present, reporting (and skipping) the ones that fail.
    pub fn all() -> Vec<Arc<Self>> {
        let mut sources = Vec::new();
        match Discovery::udp() {
            Ok(d) => sources.push(d),
            Err(e) => eprintln!("[WARNING] UDP discovery unavailable: {e:#}"),
        }
        for interface in super::can::list_interfaces() {
            match Discovery::can(&interface) {
                Ok(d) => sources.push(d),
                Err(e) => eprintln!("[WARNING] CAN discovery on {interface} unavailable: {e:#}"),
            }
        }
        sources
    }

    pub fn interface_name(&self) -> &str {
        &self.name
    }

    fn device(self: &Arc<Self>, record: &Record) -> CanmoreDevice {
        CanmoreDevice {
            link: record.link.clone(),
            client_id: record.client_id,
            heartbeat: record.heartbeat,
            mode: DeviceMode::from_heartbeat(record.heartbeat.mode),
            discovery: self.clone(),
            flash_id: None,
        }
    }

    /// The devices currently on the bus, ordered by client ID.
    pub fn devices(self: &Arc<Self>) -> Vec<CanmoreDevice> {
        let mut records = self.shared.records.lock().unwrap();
        records.retain(|r| r.seen.elapsed() < STALE_DISCOVERY);
        let mut devices: Vec<CanmoreDevice> = records.iter().map(|r| self.device(r)).collect();
        devices.sort_by_key(|d| d.client_id);
        devices
    }

    /// Waits for `client_id` to send a heartbeat in a mode accepted by `accept`. `None` waits forever.
    pub fn wait_for(
        self: &Arc<Self>,
        client_id: u8,
        accept: impl Fn(DeviceMode) -> bool,
        timeout: Option<Duration>,
    ) -> Option<CanmoreDevice> {
        let deadline = timeout.map(|t| Instant::now() + t);
        let mut records = self.shared.records.lock().unwrap();
        loop {
            let found = records.iter().find(|r| {
                r.client_id == client_id
                    && r.seen.elapsed() < STALE_DISCOVERY
                    && accept(DeviceMode::from_heartbeat(r.heartbeat.mode))
            });
            if let Some(record) = found {
                return Some(self.device(record));
            }
            records = match deadline {
                Some(deadline) => {
                    let remaining = deadline.saturating_duration_since(Instant::now());
                    if remaining.is_zero() {
                        return None;
                    }
                    self.shared.notify.wait_timeout(records, remaining).unwrap().0
                }
                None => self.shared.notify.wait(records).unwrap(),
            };
        }
    }

    /// Waits (forever) for `client_id` to boot, stops it in the bootloader and connects to it.
    pub fn catch_in_boot_delay(self: &Arc<Self>, client_id: u8) -> Result<BootloaderClient> {
        let device = self
            .wait_for(client_id, |m| m == DeviceMode::BootDelay, None)
            .ok_or_else(|| anyhow!("Failed to catch device in boot delay!"))?;
        device.enter_bootloader_from_boot_delay()
    }
}

impl Drop for Discovery {
    fn drop(&mut self) {
        self.shared.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

/// A device found by a [`Discovery`].
#[derive(Clone)]
pub struct CanmoreDevice {
    pub link: Link,
    pub client_id: u8,
    pub heartbeat: Heartbeat,
    pub mode: DeviceMode,
    discovery: Arc<Discovery>,
    flash_id: Option<u64>,
}

impl CanmoreDevice {
    /// "can0 - Client 8"
    pub fn interface_description(&self) -> String {
        match &self.link {
            Link::Udp(addr) => format!("UDP {addr} - Client {}", self.client_id),
            link => format!("{link} - Client {}", self.client_id),
        }
    }

    pub fn discovery(&self) -> &Arc<Discovery> {
        &self.discovery
    }

    pub fn reg_client(&self, mode: ControlInterfaceMode) -> Result<RegClient> {
        RegClient::open(&self.link, self.client_id, mode)
    }

    pub fn debug_client(&self) -> Result<DebugClient> {
        DebugClient::open(&self.link, self.client_id)
    }

    pub fn bootloader_client(&self) -> Result<BootloaderClient> {
        BootloaderClient::open(&self.link, self.client_id)
    }

    pub fn linux_client(&self) -> Result<LinuxClient> {
        LinuxClient::open(&self.link, self.client_id)
    }

    /// Reads the flash ID over the control interface (cached after the first success).
    pub fn read_flash_id(&mut self) -> Result<u64> {
        if let Some(id) = self.flash_id {
            return Ok(id);
        }
        let id = match self.mode {
            DeviceMode::Normal => self.debug_client()?.flash_id(),
            DeviceMode::Linux => self.linux_client()?.flash_id(),
            DeviceMode::Bootloader => {
                // Read the ID registers directly: connecting a BootloaderClient also resets the write key
                let mut client = self.reg_client(ControlInterfaceMode::Bootloader)?;
                let page = bl_itf::MCU_CONTROL_PAGE;
                let magic = client.read(page, bl_itf::mcu_control::MAGIC)?;
                if magic != bl_itf::mcu_control::MAGIC_VALUE {
                    bail!("Unexpected bootloader magic 0x{magic:08x}");
                }
                let lower = client.read(page, bl_itf::mcu_control::LOWER_FLASH_ID)?;
                let upper = client.read(page, bl_itf::mcu_control::UPPER_FLASH_ID)?;
                flash_id_from_words(lower, upper)
            }
            mode => bail!("Flash ID retrieval not supported in mode {}", mode.name()),
        };
        self.flash_id = Some(id);
        Ok(id)
    }

    /// The application or bootloader version string.
    pub fn version(&self) -> Result<String> {
        match self.mode {
            DeviceMode::Normal => self.debug_client()?.version(),
            DeviceMode::Linux => self.linux_client()?.version(),
            DeviceMode::Bootloader => {
                self.reg_client(ControlInterfaceMode::Bootloader)?.read_string_page(bl_itf::VERSION_STRING_PAGE)
            }
            mode => bail!("Version retrieval not supported in mode {}", mode.name()),
        }
    }

    fn wait_for_bootloader(&self) -> Result<BootloaderClient> {
        let device = self
            .discovery
            .wait_for(self.client_id, |m| m == DeviceMode::Bootloader, Some(self.discovery.mode_switch_timeout))
            .ok_or_else(|| anyhow!("Failed to reconnect to device in bootloader mode"))?;
        device.bootloader_client()
    }

    /// Reboots an application into the bootloader and connects to it.
    pub fn switch_to_bootloader(&self) -> Result<BootloaderClient> {
        match self.mode {
            DeviceMode::Bootloader => self.bootloader_client(),
            DeviceMode::Normal => {
                self.debug_client()?.enter_bootloader()?;
                self.wait_for_bootloader()
            }
            DeviceMode::BootDelay => self.enter_bootloader_from_boot_delay(),
            mode => bail!("Device in mode {} can not enter the bootloader", mode.name()),
        }
    }

    /// Sends the bootloader request to a device in boot delay and connects to the bootloader.
    pub fn enter_bootloader_from_boot_delay(&self) -> Result<BootloaderClient> {
        let mut client = self.reg_client(ControlInterfaceMode::BootDelay)?;
        client.send_raw(&canmore::reg_mapped::BOOTLOADER_REQUEST)?;
        self.wait_for_bootloader()
    }

    /// Waits for a device seen in boot delay to boot again (after a reset), then stops it in the bootloader.
    pub fn wait_for_boot_delay_and_enter_bootloader(&self, timeout: Duration) -> Result<BootloaderClient> {
        let device = self
            .discovery
            .wait_for(self.client_id, |m| m == DeviceMode::BootDelay, Some(timeout))
            .context("Failed to discover boot delay device again within timeout")?;
        device.enter_bootloader_from_boot_delay()
    }
}

impl flash::Device for CanmoreDevice {
    fn interface(&self) -> String {
        self.interface_description()
    }

    fn flash_id(&mut self) -> u64 {
        self.read_flash_id().unwrap_or(0)
    }

    fn supports_flash_interface(&self) -> bool {
        // Boot delay devices could be stopped, but the delay is too short to select them interactively; the upload
        // tool's wait for boot option handles them
        matches!(self.mode, DeviceMode::Normal | DeviceMode::Bootloader)
    }

    fn additional_info(&mut self) -> Vec<(String, String)> {
        let mut info = vec![("Mode".to_owned(), self.mode.name())];
        match self.mode {
            DeviceMode::Normal => info.push(("App Version".into(), self.version().unwrap_or_default())),
            DeviceMode::Bootloader => info.push(("BL Version".into(), self.version().unwrap_or_default())),
            _ => {}
        }
        info
    }

    fn flash_interface(&mut self) -> Result<Box<dyn FlashInterface>> {
        Ok(Box::new(self.switch_to_bootloader()?))
    }
}

impl flash::Discovery for Arc<Discovery> {
    fn discover_devices(&self) -> Vec<Box<dyn flash::Device>> {
        self.devices().into_iter().map(|d| Box::new(d) as Box<dyn flash::Device>).collect()
    }
}

/// Reads the binary info of the application behind a bootloader (for the bootloader CLI's `binary_info`).
pub fn bootloader_app_info(client: &mut BootloaderClient) -> binary_info::AppInfo {
    let base = flash::FLASH_BASE + client.bootloader_size();
    let mut mem = binary_info::DeviceMemory::new(client);
    binary_info::extract_app_info(&mut mem, base)
}
