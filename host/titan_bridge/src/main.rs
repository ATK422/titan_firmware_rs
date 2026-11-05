//! `titan_bridge`: the ROS 2 side of a Rust titan firmware board.
//!
//! The C firmware runs micro-ROS on the microcontroller and needs a micro-ROS agent. Rust boards instead speak
//! `titan_link` over CANmore, and this node owns their ROS interface: one bridge process per board, publishing and
//! subscribing under the robot namespace exactly like the C board's micro-ROS node did.
//!
//! ```text
//! titan_bridge --board ivc [--interface can0] [--namespace talos] [--client-id 8] [--ros-args ...]
//! titan_bridge --board esc0 --interface cannelloni:127.0.0.1:20000    # CAN over UDP, no SocketCAN
//! ```

mod boards;
mod can;
mod cannelloni;
mod common;
mod link;
mod ros;
mod udp;

use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context as _, Result, bail};
use rclrs::{Context, CreateBasicExecutor, IntoNodeOptions, SpinOptions};
use titan_link::{DeviceToHost, HostToDevice};

use crate::boards::{BOARDS, BoardProfile, Transport};
use crate::common::CommonTopics;
use crate::link::{Event, Link};

/// The board counts as off the bus after this long without a CANmore heartbeat.
const HEARTBEAT_TIMEOUT: Duration = Duration::from_millis(2000);

struct Args {
    board: &'static BoardProfile,
    interface: String,
    namespace: String,
    client_id: u8,
}

fn usage() -> String {
    let boards: Vec<_> = BOARDS.iter().map(|b| b.name).collect();
    format!(
        "usage: titan_bridge --board <{}> [--interface can0 | cannelloni:<host>:<port> | <bind ip[:port]>[,<board ip[:port]>]] \
         [--namespace <robot>] [--client-id <id>] [--ros-args ...]",
        boards.join("|")
    )
}

fn parse_args() -> Result<Args> {
    let mut board = None;
    let mut interface = "can0".to_string();
    let mut namespace = None;
    let mut client_id = None;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--board" => board = args.next(),
            "--interface" => interface = args.next().context("--interface needs a value")?,
            "--namespace" => namespace = Some(args.next().context("--namespace needs a value")?),
            "--client-id" => {
                client_id = Some(args.next().context("--client-id needs a value")?.parse().context("--client-id")?)
            }
            // Everything from here on belongs to rcl
            "--ros-args" => break,
            "-h" | "--help" => {
                println!("{}", usage());
                std::process::exit(0);
            }
            other => bail!("unexpected argument {other}\n{}", usage()),
        }
    }
    let name = board.with_context(usage)?;
    let board = BOARDS.iter().find(|b| b.name == name).with_context(|| format!("unknown board {name}\n{}", usage()))?;
    Ok(Args {
        board,
        interface,
        namespace: namespace.unwrap_or_else(|| board.namespace.to_string()),
        client_id: client_id.unwrap_or(board.client_id),
    })
}

fn main() -> Result<()> {
    let args = parse_args()?;
    let board = args.board;

    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node_name = board.node_name_for(args.client_id);
    let node = executor.create_node(node_name.as_str().namespace(&args.namespace))?;
    let logger = format!("{}.{}", args.namespace, node_name);

    let link: Arc<dyn Link> = match board.transport {
        // A SocketCAN interface, or `cannelloni:HOST:PORT` for CAN over UDP to a cannelloni hub
        Transport::Can { fd } => can::open(&args.interface, args.client_id, fd)?,
        // `--interface` is `[BIND][,BOARD]` here (see `puddles_core::wire::LinkSpec`); the CAN default means the
        // default addresses
        Transport::Udp => {
            Arc::new(udp::UdpLink::open(if args.interface == "can0" { "" } else { args.interface.as_str() })?)
        }
    };
    let common =
        CommonTopics::new(&node, link.clone(), logger.clone(), board.heartbeat_topic(), board.subscribes_kill())?;
    let board_topics = (board.create)(&node, link.clone(), logger.clone())?;
    rclrs::log_info!(
        &logger,
        "Bridging {} (client {}) on {} as /{}/{}",
        board.name,
        args.client_id,
        args.interface,
        args.namespace,
        node_name
    );

    // Pings keep the firmware's ROS connection alive, like the agent answering micro-ROS pings
    let ping_link = link.clone();
    let ping_logger = logger.clone();
    thread::spawn(move || {
        let ping = HostToDevice::Ping { protocol_version: titan_link::PROTOCOL_VERSION };
        let mut failing = false;
        loop {
            match ping_link.send(&ping) {
                Ok(()) => failing = false,
                Err(err) if !failing => {
                    rclrs::log_warn!(&ping_logger, "Ping failed: {err:#}");
                    failing = true;
                }
                Err(_) => {}
            }
            thread::sleep(Duration::from_millis(titan_link::HOST_PING_INTERVAL_MS));
        }
    });

    let client_id = args.client_id;
    let rx_logger = logger.clone();
    thread::spawn(move || {
        let mut last_heartbeat: Option<Instant> = None;
        loop {
            let event = match link.receive() {
                Ok(event) => event,
                Err(err) => {
                    rclrs::log_warn!(&rx_logger, "{err:#}");
                    continue;
                }
            };
            let result = match event {
                Event::Message(DeviceToHost::Heartbeat) => common.publish_heartbeat(client_id),
                Event::Message(DeviceToHost::FirmwareStatus(status)) => common.publish_firmware_status(&status),
                Event::Message(other) => match board_topics.handle(&other) {
                    Ok(true) => Ok(()),
                    Ok(false) => Err(anyhow::anyhow!("unhandled message {other:?}")),
                    Err(err) => Err(err),
                },
                Event::Heartbeat(heartbeat) => {
                    if last_heartbeat.is_none() {
                        rclrs::log_info!(&rx_logger, "Board online (error flag: {})", heartbeat.error);
                    }
                    last_heartbeat = Some(Instant::now());
                    Ok(())
                }
                Event::Utility { channel, data } => match board_topics.handle_utility(channel, &data) {
                    Ok(true) => Ok(()),
                    Ok(false) => Err(anyhow::anyhow!("unhandled utility frame on channel {channel}")),
                    Err(err) => Err(err),
                },
                Event::Idle => Ok(()),
            };
            if let Err(err) = result {
                rclrs::log_warn!(&rx_logger, "{err:#}");
            }
            if last_heartbeat.is_some_and(|t| t.elapsed() > HEARTBEAT_TIMEOUT) {
                rclrs::log_warn!(&rx_logger, "Board went silent (no CANmore heartbeat)");
                last_heartbeat = None;
            }
        }
    });

    for err in executor.spin(SpinOptions::default()) {
        rclrs::log_error!(&logger, "Executor error: {err}");
    }
    Ok(())
}
