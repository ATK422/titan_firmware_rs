//! Firmware side of `titan_link`: the Rust replacement for the micro-ROS boilerplate in each board's `ros.c` and the
//! ROS state machine in `main.c`.
//!
//! [`link_task`] decodes messages from the bridge and keeps track of the connection. When the first ping arrives ROS
//! counts as connected: safety is initialized and the heartbeat and firmware status start flowing. When pings stop
//! for [`titan_link::HOST_TIMEOUT_MS`] the connection is dropped and safety de-initialized, which kills the vehicle.
//! Board specific messages are handed to [`LinkConfig::on_board_message`].
//!
//! The link does not care how messages travel. A transport task (CAN in [`crate::canbus`], or Ethernet on boards
//! without a CAN block) moves encoded messages between [`OUTBOX`]/[`INBOX`] and the wire, and reports with
//! [`mark_transport_online`] while it can reach the bus.

use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};
use portable_atomic::{AtomicBool, AtomicU64, Ordering};

use titan_link::{DeviceToHost, FirmwareStatus, HostToDevice, ReleaseType};

use crate::safety;

/// One encoded `titan_link` message.
pub type Payload = heapless::Vec<u8, { titan_link::MAX_ENCODED_LEN }>;

const QUEUE_DEPTH: usize = 8;

/// Encoded messages waiting for the transport to send them to the bridge.
pub static OUTBOX: Channel<CriticalSectionRawMutex, Payload, QUEUE_DEPTH> = Channel::new();
/// Encoded messages the transport received from the bridge.
pub static INBOX: Channel<CriticalSectionRawMutex, Payload, QUEUE_DEPTH> = Channel::new();

static TRANSPORT_ONLINE_UNTIL: AtomicU64 = AtomicU64::new(0);

/// Called by the transport whenever it confirms it can reach the bus; the transport counts as online until `until`.
pub fn mark_transport_online(until: Instant) {
    TRANSPORT_ONLINE_UNTIL.store(until.as_ticks(), Ordering::Relaxed);
}

/// True while the transport reports it can reach the bus (CAN heartbeats going out, Ethernet link up).
pub fn transport_online() -> bool {
    Instant::now().as_ticks() < TRANSPORT_ONLINE_UNTIL.load(Ordering::Relaxed)
}

/// Board identity and hooks.
pub struct LinkConfig {
    /// Reported as `board_name`, e.g. `"talos/ivc_board"`.
    pub board_name: &'static str,
    /// Bus ID reported in the firmware status.
    pub bus_id: u8,
    /// CANmore client ID (or 1 on Ethernet), reported in the heartbeat and firmware status.
    pub client_id: u8,
    pub version_major: u8,
    pub version_minor: u8,
    pub release_type: ReleaseType,
    /// Handles messages other than the ping and kill switch. Runs in the link task, so keep it short: queue the
    /// work for a board task, like the executor callbacks in the C firmware.
    pub on_board_message: fn(HostToDevice),
}

static CONNECTED: AtomicBool = AtomicBool::new(false);

/// True while the bridge is pinging us.
pub fn is_connected() -> bool {
    CONNECTED.load(Ordering::Relaxed)
}

/// Queues a message for the bridge. Returns `false` if it was dropped because the transport queue is full.
pub fn send(msg: &DeviceToHost) -> bool {
    let mut buf = [0u8; titan_link::MAX_ENCODED_LEN];
    let Ok(encoded) = titan_link::encode(msg, &mut buf) else {
        defmt::error!("titan_link message too large to encode: {}", msg);
        return false;
    };
    OUTBOX.try_send(Payload::from_slice(encoded).unwrap()).is_ok()
}

/// A periodic timer that notices missed runs, like `timer_ready` in the C `main.c`.
struct Interval {
    next: Instant,
    period: Duration,
}

impl Interval {
    fn starting_in(period: Duration) -> Self {
        Interval { next: Instant::now() + period, period }
    }

    /// Returns true if the timer fired, advancing it. Raises `FAULT_TIMER_MISSED` if whole periods were skipped.
    fn fire(&mut self, now: Instant) -> bool {
        if now < self.next {
            return false;
        }
        let mut next = self.next + self.period;
        let mut missed = 0u32;
        while next <= now {
            next += self.period;
            missed += 1;
        }
        if missed > 0 {
            safety::raise_fault(safety::FAULT_TIMER_MISSED, missed);
        }
        self.next = next;
        true
    }
}

struct Connection {
    last_ping: Instant,
    heartbeat: Interval,
    status: Interval,
}

/// Runs the link forever.
#[embassy_executor::task]
pub async fn link_task(config: LinkConfig) -> ! {
    let mut connection: Option<Connection> = None;
    let mut warned_version = false;

    loop {
        let wake_at = match &connection {
            Some(c) => (c.last_ping + Duration::from_millis(titan_link::HOST_TIMEOUT_MS))
                .min(c.heartbeat.next)
                .min(c.status.next),
            None => Instant::MAX,
        };

        if let Either::First(msg) = select(INBOX.receive(), Timer::at(wake_at)).await {
            match titan_link::decode::<HostToDevice>(&msg) {
                Ok(HostToDevice::Ping { protocol_version }) if protocol_version != titan_link::PROTOCOL_VERSION => {
                    if !warned_version {
                        defmt::error!(
                            "Bridge speaks titan_link v{}, firmware speaks v{}",
                            protocol_version,
                            titan_link::PROTOCOL_VERSION
                        );
                        warned_version = true;
                    }
                    safety::raise_fault(safety::FAULT_ROS_ERROR, protocol_version as u32);
                }
                Ok(HostToDevice::Ping { .. }) => match &mut connection {
                    Some(c) => c.last_ping = Instant::now(),
                    None => {
                        defmt::info!("ROS connected");
                        // Fresh connection, so earlier ROS errors no longer apply
                        safety::lower_fault(safety::FAULT_ROS_ERROR);
                        safety::init();
                        CONNECTED.store(true, Ordering::Relaxed);
                        warned_version = false;
                        // Start timers now so they are staggered from bring-up and don't report misses
                        connection = Some(Connection {
                            last_ping: Instant::now(),
                            heartbeat: Interval::starting_in(Duration::from_millis(titan_link::HEARTBEAT_INTERVAL_MS)),
                            status: Interval::starting_in(Duration::from_millis(
                                titan_link::FIRMWARE_STATUS_INTERVAL_MS,
                            )),
                        });
                    }
                },
                Ok(HostToDevice::KillSwitch { asserting_kill }) => {
                    safety::kill_switch_update(safety::KILL_SWITCH_ROS, asserting_kill, true);
                }
                Ok(other) => (config.on_board_message)(other),
                Err(_) => defmt::warn!("Undecodable titan_link message ({} bytes)", msg.len()),
            }
        }

        let now = Instant::now();
        let Some(c) = &mut connection else { continue };

        if now - c.last_ping >= Duration::from_millis(titan_link::HOST_TIMEOUT_MS) {
            defmt::info!("Lost connection to ROS");
            connection = None;
            CONNECTED.store(false, Ordering::Relaxed);
            safety::deinit();
            continue;
        }

        // Only one message per wake-up, mirroring the one-ROS-call-per-tick rule of the C main loop
        if c.heartbeat.fire(now) {
            send(&DeviceToHost::Heartbeat);
        } else if c.status.fire(now) {
            send(&DeviceToHost::FirmwareStatus(firmware_status(&config)));
        }
    }
}

fn firmware_status(config: &LinkConfig) -> FirmwareStatus {
    let kill = safety::kill_switch_report();
    let mut board_name = heapless::String::new();
    // Names are checked against BOARD_NAME_LEN when the board is written; truncate rather than panic
    for ch in config.board_name.chars() {
        if board_name.push(ch).is_err() {
            break;
        }
    }
    FirmwareStatus {
        board_name,
        bus_id: config.bus_id,
        client_id: config.client_id,
        version_major: config.version_major,
        version_minor: config.version_minor,
        version_release_type: config.release_type as u8,
        uptime_ms: Instant::now().as_millis() as u32,
        faults: safety::faults(),
        kill_switches_enabled: kill.enabled,
        kill_switches_asserting_kill: kill.asserting_kill,
        kill_switches_needs_update: kill.needs_update,
        kill_switches_timed_out: kill.timed_out,
    }
}
