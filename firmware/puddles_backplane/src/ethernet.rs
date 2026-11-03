//! Ethernet transport for the link, replacing `transport_eth` (micro-ROS over UDP) and `ethernet_tick`.
//!
//! The backplane's Ethernet controller is a WIZnet W5500 on SPI0 (MISO GPIO 0, CS 1, SCK 2, MOSI 3, INT 4,
//! RST 5). The C `driver_wiznet` (a port of the Arduino Ethernet library) used the chip's hardware UDP sockets and
//! probed for a W5100, W5200 or W5500. Here `embassy-net-wiznet` runs the W5500 in MACRAW mode under the
//! `embassy-net` (smoltcp) stack, which then does ARP and UDP.
//!
//! [`transport_task`] serves three things:
//! * the `titan_link` socket on UDP port [`wire::PORT`]: datagrams from the robot computer go into
//!   [`link::INBOX`], messages from [`link::OUTBOX`] go out as datagrams (format in [`puddles_core::wire`])
//! * the link status: [`link::mark_transport_online`] while the PHY reports link, which drives the yellow status
//!   LED like `led_network_online_set(ethernet_check_online())`
//! * the CANmore heartbeat, broadcast every 500 ms to port 2201 from port 2202 like `ethernet_tick`. It reports
//!   control interface mode "none": the register mapped control interface on port 2202 isn't ported, so requests
//!   arriving there are dropped (the CAN boards don't serve it either).

use core::cell::Cell;
use core::convert::Infallible;

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_futures::join::join3;
use embassy_futures::select::{Either, select};
use embassy_net::udp::{PacketMetadata, RecvError, UdpSocket};
use embassy_net::{IpAddress, IpEndpoint, Ipv4Address, Ipv4Cidr, StackResources, StaticConfigV4};
use embassy_net_wiznet::chip::W5500;
use embassy_rp::gpio::{Input, Output};
use embassy_rp::peripherals::SPI0;
use embassy_rp::spi::{self, Spi};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};
use static_cell::StaticCell;

use canmore::{ControlInterfaceMode, Heartbeat};
use puddles_core::wire::{self, MAX_DATAGRAM_LEN};
use titan_rp2040::puddles::{ETHERNET_GATEWAY, ETHERNET_MASK, ROBOT_COMPUTER_IP};
use titan_rp2040::{link, safety};

/// SPI clock of the W5500 (`spi_init(..., 30000000)` in `transport_eth_init`).
pub const SPI_FREQUENCY: u32 = 30_000_000;
/// MAC prefix (locally administered); the rest comes from the flash unique ID (`transport_eth_init`).
const MAC_PREFIX: [u8; 2] = [0x2A, 0xCD];
/// CANmore heartbeat broadcast (`CANMORE_ETH_HEARTBEAT_BROADCAST_PORT`) and control interface
/// (`CANMORE_ETH_CONTROL_INTERFACE_PORT`) ports from `canmore/ethernet_defs.h`.
const HEARTBEAT_PORT: u16 = 2201;
const CONTROL_INTERFACE_PORT: u16 = 2202;
/// `ETH_HEARTBEAT_INTERVAL_MS`.
const HEARTBEAT_INTERVAL: Duration = Duration::from_millis(canmore::HEARTBEAT_INTERVAL_MS);
/// How often the link state is checked (`LED_UPTIME_INTERVAL_MS`).
const LINK_CHECK_INTERVAL: Duration = Duration::from_millis(250);
/// The transport stays online this long after a check saw the link up, so a stalled task drops it.
const LINK_ONLINE_HOLD: Duration = Duration::from_millis(600);

type EthSpi = Spi<'static, SPI0, spi::Async>;
type EthDevice = SpiDevice<'static, NoopRawMutex, EthSpi, Output<'static>>;
type EthRunner = embassy_net_wiznet::Runner<'static, W5500, EthDevice, Input<'static>, Output<'static>>;

/// The MAC address: the prefix plus bytes 4-7 of the flash unique ID, as in C.
pub fn mac_address(unique_id: [u8; 8]) -> [u8; 6] {
    let [a, b] = MAC_PREFIX;
    [a, b, unique_id[4], unique_id[5], unique_id[6], unique_id[7]]
}

fn ipv4(octets: [u8; 4]) -> Ipv4Address {
    Ipv4Address::new(octets[0], octets[1], octets[2], octets[3])
}

/// Resets and configures the W5500 and brings up the network stack with the board's static address. Panics if
/// the chip doesn't respond, like `transport_eth_init` failing in C (the watchdog reset then retries).
pub async fn init(
    spawner: Spawner,
    spi: EthSpi,
    cs: Output<'static>,
    int: Input<'static>,
    reset: Output<'static>,
    mac: [u8; 6],
    seed: u64,
) -> embassy_net::Stack<'static> {
    static SPI_BUS: StaticCell<Mutex<NoopRawMutex, EthSpi>> = StaticCell::new();
    static STATE: StaticCell<embassy_net_wiznet::State<2, 2>> = StaticCell::new();
    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();

    let device = SpiDevice::new(SPI_BUS.init(Mutex::new(spi)), cs);
    let (net_device, runner) = match embassy_net_wiznet::new::<2, 2, W5500, _, _, _>(
        mac,
        STATE.init(embassy_net_wiznet::State::new()),
        device,
        int,
        reset,
    )
    .await
    {
        Ok(driver) => driver,
        Err(embassy_net_wiznet::InitError::InvalidChipVersion { expected, actual }) => {
            panic!("Failed to initialize Ethernet hardware: chip version {} (expected {})", actual, expected)
        }
        Err(embassy_net_wiznet::InitError::SpiError(_)) => panic!("Failed to initialize Ethernet hardware: SPI error"),
    };
    spawner.spawn(wiznet_task(runner).unwrap());

    let config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(ipv4(wire::BOARD_IP), u32::from_be_bytes(ETHERNET_MASK).count_ones() as u8),
        gateway: Some(ipv4(ETHERNET_GATEWAY)),
        dns_servers: heapless::Vec::new(),
    });
    let (stack, net_runner) = embassy_net::new(net_device, config, RESOURCES.init(StackResources::new()), seed);
    spawner.spawn(net_task(net_runner).unwrap());
    defmt::info!("Ethernet up as {} (MAC {=[u8]:02x})", ipv4(wire::BOARD_IP), mac);
    stack
}

#[embassy_executor::task]
async fn wiznet_task(runner: EthRunner) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, embassy_net_wiznet::Device<'static>>) -> ! {
    runner.run().await
}

/// Moves `titan_link` messages between the link and UDP, and keeps the link status and heartbeat going.
#[embassy_executor::task]
pub async fn transport_task(stack: embassy_net::Stack<'static>) -> ! {
    let mut link_rx_meta = [PacketMetadata::EMPTY; 8];
    let mut link_rx_buf = [0u8; 4 * MAX_DATAGRAM_LEN];
    let mut link_tx_meta = [PacketMetadata::EMPTY; 8];
    let mut link_tx_buf = [0u8; 4 * MAX_DATAGRAM_LEN];
    let mut link_socket =
        UdpSocket::new(stack, &mut link_rx_meta, &mut link_rx_buf, &mut link_tx_meta, &mut link_tx_buf);
    // Binding a fresh socket to a free port can't fail
    link_socket.bind(wire::PORT).unwrap();

    let mut control_rx_meta = [PacketMetadata::EMPTY; 2];
    let mut control_rx_buf = [0u8; 128];
    let mut control_tx_meta = [PacketMetadata::EMPTY; 2];
    let mut control_tx_buf = [0u8; 16];
    let mut control_socket =
        UdpSocket::new(stack, &mut control_rx_meta, &mut control_rx_buf, &mut control_tx_meta, &mut control_tx_buf);
    control_socket.bind(CONTROL_INTERFACE_PORT).unwrap();

    // Where the bridge listens: its last datagram's source port, on the robot computer
    let bridge = Cell::new(IpEndpoint::new(IpAddress::Ipv4(ipv4(ROBOT_COMPUTER_IP)), wire::PORT));
    let (never, _, _) =
        join3(receive(&link_socket, &bridge), send(&link_socket, &bridge, stack), status(stack, &control_socket)).await;
    match never {}
}

/// Datagrams from the bridge to [`link::INBOX`].
async fn receive(socket: &UdpSocket<'_>, bridge: &Cell<IpEndpoint>) -> Infallible {
    let robot_computer = IpAddress::Ipv4(ipv4(ROBOT_COMPUTER_IP));
    let mut buf = [0u8; MAX_DATAGRAM_LEN];
    loop {
        let (len, meta) = match socket.recv_from(&mut buf).await {
            Ok(received) => received,
            Err(RecvError::Truncated) => {
                defmt::warn!("Dropping oversized datagram");
                continue;
            }
        };
        if meta.endpoint.addr != robot_computer {
            defmt::debug!("Ignoring datagram from {}", meta.endpoint);
            continue;
        }
        let payload = match wire::unframe(&buf[..len]) {
            Ok(payload) => payload,
            Err(err) => {
                defmt::debug!("Ignoring datagram from {}: {}", meta.endpoint, err);
                continue;
            }
        };
        // The bridge's state/kill subscription only ever echoes our own state/kill; see `is_ros_kill_switch`
        if wire::is_ros_kill_switch(payload) {
            continue;
        }
        bridge.set(meta.endpoint);
        // unframe bounds the payload to MAX_ENCODED_LEN, the payload capacity
        let Ok(payload) = link::Payload::from_slice(payload) else { continue };
        if link::INBOX.try_send(payload).is_err() {
            defmt::warn!("Link receive queue full, dropping message");
        }
    }
}

/// Messages from [`link::OUTBOX`] to the bridge.
async fn send(socket: &UdpSocket<'_>, bridge: &Cell<IpEndpoint>, stack: embassy_net::Stack<'static>) -> Infallible {
    let mut buf = [0u8; MAX_DATAGRAM_LEN];
    loop {
        let payload = link::OUTBOX.receive().await;
        // Like a failed micro-ROS write, a message sent while the cable is out is lost
        if !stack.is_link_up() {
            continue;
        }
        let Ok(datagram) = wire::frame(&payload, &mut buf) else { continue };
        if let Err(err) = socket.send_to(datagram, bridge.get()).await {
            defmt::debug!("Sending to the bridge failed: {}", err);
        }
    }
}

/// Link status, CANmore heartbeat broadcasts, and dropping control interface requests.
async fn status(stack: embassy_net::Stack<'static>, control: &UdpSocket<'_>) -> Infallible {
    let broadcast = IpEndpoint::new(IpAddress::Ipv4(Ipv4Address::BROADCAST), HEARTBEAT_PORT);
    let mut heartbeat = Heartbeat { mode: ControlInterfaceMode::None as u8, ..Heartbeat::default() };
    let mut next_link_check = Instant::now();
    let mut next_heartbeat = Instant::now();
    let mut link_up = false;
    let mut warned_control = false;
    let mut request = [0u8; 128];
    loop {
        if let Either::First(Ok((_, meta))) =
            select(control.recv_from(&mut request), Timer::at(next_link_check.min(next_heartbeat))).await
            && !warned_control
        {
            defmt::warn!("Control interface request from {} dropped: not implemented", meta.endpoint);
            warned_control = true;
        }

        let now = Instant::now();
        if now >= next_link_check {
            let up = stack.is_link_up();
            if up != link_up {
                defmt::info!("Ethernet link {}", if up { "up" } else { "down" });
                link_up = up;
            }
            if up {
                link::mark_transport_online(now + LINK_ONLINE_HOLD);
            }
            next_link_check = now + LINK_CHECK_INTERVAL;
        }
        if now >= next_heartbeat {
            heartbeat.cnt = heartbeat.cnt.wrapping_add(1);
            heartbeat.error = safety::faults() != 0;
            if link_up && let Err(err) = control.send_to(&[heartbeat.to_byte()], broadcast).await {
                defmt::debug!("Heartbeat broadcast failed: {}", err);
            }
            next_heartbeat = now + HEARTBEAT_INTERVAL;
        }
    }
}
