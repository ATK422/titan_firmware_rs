//! CANmore client over the MCP251xFD, ported from `driver_canbus`.
//!
//! [`can_task`] owns the controller. It
//! * sends the CANmore heartbeat every 500 ms
//! * carries `titan_link` messages between [`crate::link`] and the bus as CANmore messages of subtype
//!   [`canmore::MSG_SUBTYPE_TITAN_LINK`]
//! * carries other CANmore messages through [`TX`]/[`RX`] and utility frames through [`UTIL_TX`]/[`UTIL_RX`]
//!
//! The bus counts as online while heartbeats keep making it onto the bus; if the controller stays offline for 10 s
//! it is reset, which recovers the chip from the rare states where it silently stops transmitting.

use embassy_futures::select::{Either3, select3};
use embassy_rp::clocks::Gpout;
use embassy_rp::peripherals::PIN_21;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};
use portable_atomic::{AtomicU8, AtomicU64, Ordering};

use canmore::id::{self, Direction, FrameType};
use canmore::{ControlInterfaceMode, Heartbeat, MsgDecoder, MsgEncoder};

use crate::mcp251xfd::{BitRates, Error, Mcp251xfd};
use crate::{link, safety};

/// Largest message payload passed through [`TX`] and [`RX`].
pub const MAX_PAYLOAD: usize = titan_link::MAX_ENCODED_LEN;
const QUEUE_DEPTH: usize = 8;
const UTIL_QUEUE_DEPTH: usize = 16;

/// A CANmore message other than `titan_link` traffic.
#[derive(Clone, Debug)]
pub struct CanMessage {
    pub subtype: u8,
    pub data: heapless::Vec<u8, MAX_PAYLOAD>,
}

/// A CANmore utility frame (always a classic CAN frame).
#[derive(Clone, Debug)]
pub struct UtilityFrame {
    /// Utility channel, see the `canmore::CHAN_*` constants.
    pub channel: u8,
    pub data: heapless::Vec<u8, { canmore::MAX_FRAME_SIZE }>,
}

/// Messages to send to the agent. Senders should `try_send` and drop the message if the queue is full.
pub static TX: Channel<CriticalSectionRawMutex, CanMessage, QUEUE_DEPTH> = Channel::new();
/// Messages received from the agent, other than `titan_link` messages (those go to [`link::INBOX`]).
pub static RX: Channel<CriticalSectionRawMutex, CanMessage, QUEUE_DEPTH> = Channel::new();
/// Utility frames to send to the agent. Sent ahead of messages.
pub static UTIL_TX: Channel<CriticalSectionRawMutex, UtilityFrame, UTIL_QUEUE_DEPTH> = Channel::new();
/// Utility frames received from the agent (for example thruster commands on [`canmore::CHAN_THRUSTER_CMDS`]).
pub static UTIL_RX: Channel<CriticalSectionRawMutex, UtilityFrame, UTIL_QUEUE_DEPTH> = Channel::new();

static CONTROL_INTERFACE_MODE: AtomicU8 = AtomicU8::new(ControlInterfaceMode::None as u8);

/// Sets the control interface mode reported in the heartbeat, for firmware that serves the control interface.
pub fn set_control_interface_mode(mode: ControlInterfaceMode) {
    CONTROL_INTERFACE_MODE.store(mode as u8, Ordering::Relaxed);
}

/// The bus is offline once no heartbeat has been confirmed sent for this long.
const HEARTBEAT_TIMEOUT: Duration = Duration::from_millis(2000);
/// The controller is reset after being offline this long.
const OFFLINE_RESET_TIMEOUT: Duration = Duration::from_millis(10_000);
const CONNECT_ATTEMPTS: usize = 3;
/// Transmit event sequence numbers telling heartbeats apart from message frames.
const SEQ_MESSAGE: u8 = 1;
const SEQ_HEARTBEAT: u8 = 2;

static ONLINE_UNTIL: AtomicU64 = AtomicU64::new(0);

/// True while heartbeats are making it onto the bus.
pub fn is_online() -> bool {
    Instant::now().as_ticks() < ONLINE_UNTIL.load(Ordering::Relaxed)
}

/// Bus and client settings.
#[derive(Clone, Copy, Debug)]
pub struct CanConfig {
    /// CANmore client ID of this board.
    pub client_id: u8,
    pub rates: BitRates,
}

impl CanConfig {
    fn fd(&self) -> bool {
        self.rates.data.is_some()
    }

    /// Accept message frames (standard and extended) and utility frames addressed to this client by the agent.
    fn filters(&self) -> [(u32, u32, bool); 3] {
        let a2c = Direction::AgentToClient;
        [
            (id::std_id(self.client_id, FrameType::Msg, a2c, 0), id::STD_CLIENT_TYPE_DIRECTION_MASK, false),
            (id::ext_id(self.client_id, FrameType::Msg, a2c, 0, 0), id::EXT_CLIENT_TYPE_DIRECTION_MASK, true),
            (id::std_id(self.client_id, FrameType::Util, a2c, 0), id::STD_CLIENT_TYPE_DIRECTION_MASK, false),
        ]
    }
}

/// Generates the 4 MHz CLKIN reference for the MCP251xFD on GPIO 21 from the 48 MHz USB clock.
pub fn start_reference_clock(pin: embassy_rp::Peri<'static, PIN_21>) -> Gpout<'static, PIN_21> {
    let clock = Gpout::new(pin);
    clock.set_src(embassy_rp::clocks::GpoutSrc::Usb);
    clock.set_div(embassy_rp::clocks::clk_usb_freq() / crate::mcp251xfd::CLKIN_HZ, 0);
    clock.enable();
    clock
}

/// Runs the CAN bus forever. `_clock` is the controller's reference clock, kept alive here.
#[embassy_executor::task]
pub async fn can_task(mut dev: Mcp251xfd, _clock: Gpout<'static, PIN_21>, config: CanConfig) -> ! {
    let mut decoder = MsgDecoder::new(config.fd());
    loop {
        let mut attempt = 0;
        while let Err(err) = dev.init(config.rates, &config.filters()).await {
            attempt += 1;
            defmt::warn!("CAN controller init failed ({}), attempt {}", err, attempt);
            if attempt >= CONNECT_ATTEMPTS {
                // No point in continuing without the bus; reboot and try from scratch like the C firmware
                panic!("Failed to initialize CAN bus hardware: {:?}", err);
            }
            Timer::after_millis(10).await;
        }
        defmt::info!("CAN controller online as client {}", config.client_id);
        decoder.reset();

        let err = run(&mut dev, &config, &mut decoder).await;
        defmt::error!("CAN controller error: {}, resetting", err);
        safety::raise_fault(safety::FAULT_CAN_INTERNAL_ERROR, err as u32);
    }
}

async fn run(dev: &mut Mcp251xfd, config: &CanConfig, decoder: &mut MsgDecoder) -> Error {
    match serve(dev, config, decoder).await {
        Err(err) => err,
        Ok(never) => match never {},
    }
}

async fn serve(
    dev: &mut Mcp251xfd,
    config: &CanConfig,
    decoder: &mut MsgDecoder,
) -> Result<core::convert::Infallible, Error> {
    let mut heartbeat = Heartbeat::default();
    let mut next_heartbeat = Instant::now();
    let mut last_online = Instant::now();
    // Message being sent, and how many of its frames are already queued
    let mut outgoing: Option<(CanMessage, usize)> = None;
    // Utility frame that did not fit in the transmit queue yet
    let mut utility_pending: Option<UtilityFrame> = None;

    loop {
        let events = dev.service_interrupts().await?;
        if events.rx_overflow {
            safety::raise_fault(safety::FAULT_CAN_INTERNAL_ERROR, events.flags);
        }
        if events.tx_attempts_exhausted {
            defmt::warn!("CAN frame dropped after retransmission attempts ({})", dev.error_state().await);
        }

        while let Some(frame) = dev.receive().await? {
            let fields = id::Fields::decode(frame.id, frame.extended);
            if fields.frame_type == FrameType::Util {
                let Ok(data) = heapless::Vec::from_slice(frame.data()) else { continue };
                if UTIL_RX.try_send(UtilityFrame { channel: fields.noc, data }).is_err() {
                    defmt::warn!("CAN utility queue full, dropping frame on channel {}", fields.noc);
                }
                continue;
            }
            match decoder.decode_frame(frame.id, frame.extended, frame.data()) {
                Ok(Some(msg)) => {
                    let queued = if msg.subtype == canmore::MSG_SUBTYPE_TITAN_LINK {
                        link::Payload::from_slice(msg.data).map(|data| link::INBOX.try_send(data).is_ok())
                    } else {
                        heapless::Vec::from_slice(msg.data)
                            .map(|data| RX.try_send(CanMessage { subtype: msg.subtype, data }).is_ok())
                    };
                    match queued {
                        Ok(true) => {}
                        Ok(false) => defmt::warn!("CAN receive queue full, dropping message"),
                        Err(_) => defmt::warn!("Dropping {} byte CANmore message", msg.data.len()),
                    }
                }
                Ok(None) => {}
                Err(err) => defmt::debug!("CANmore decode error: {}", err),
            }
        }

        while let Some(seq) = dev.read_tef().await? {
            if seq == SEQ_HEARTBEAT {
                last_online = Instant::now();
                ONLINE_UNTIL.store((last_online + HEARTBEAT_TIMEOUT).as_ticks(), Ordering::Relaxed);
                link::mark_transport_online(last_online + HEARTBEAT_TIMEOUT);
            }
        }

        let now = Instant::now();
        if now >= next_heartbeat {
            heartbeat.error = safety::faults() != 0;
            heartbeat.mode = CONTROL_INTERFACE_MODE.load(Ordering::Relaxed);
            heartbeat.term_valid = true;
            heartbeat.term_enabled = dev.termination_enabled().await;
            if dev.transmit(&heartbeat.frame(config.client_id), false, SEQ_HEARTBEAT).await? {
                heartbeat.cnt = heartbeat.cnt.wrapping_add(1);
                next_heartbeat = now + Duration::from_millis(canmore::HEARTBEAT_INTERVAL_MS);
            }
        }

        // Utility frames first: they carry latency sensitive traffic such as thruster commands
        loop {
            if utility_pending.is_none() {
                utility_pending = UTIL_TX.try_receive().ok();
            }
            let Some(util) = &utility_pending else { break };
            let id = id::util_id(config.client_id, Direction::ClientToAgent, util.channel);
            if dev.transmit(&canmore::Frame::new(id, false, &util.data).unwrap(), false, SEQ_MESSAGE).await? {
                utility_pending = None;
            } else {
                break;
            }
        }

        // Then as many message frames as the transmit queue takes
        loop {
            if outgoing.is_none() {
                outgoing = link::OUTBOX
                    .try_receive()
                    .ok()
                    .map(|data| CanMessage { subtype: canmore::MSG_SUBTYPE_TITAN_LINK, data })
                    .or_else(|| TX.try_receive().ok())
                    .map(|msg| (msg, 0));
            }
            let Some((msg, sent)) = &mut outgoing else { break };
            let mut frames =
                MsgEncoder::new(config.client_id, Direction::ClientToAgent, config.fd(), msg.subtype, &msg.data);
            match frames.nth(*sent) {
                None => outgoing = None,
                Some(frame) => {
                    if dev.transmit(&frame, config.fd(), SEQ_MESSAGE).await? {
                        *sent += 1;
                    } else {
                        break;
                    }
                }
            }
        }

        if now - last_online > OFFLINE_RESET_TIMEOUT {
            defmt::warn!("CAN offline for {} ms ({})", (now - last_online).as_millis(), dev.error_state().await);
            last_online = now;
            dev.init(config.rates, &config.filters()).await?;
        }

        if dev.interrupt_pending() {
            // Something is still flagged; give the chip a moment instead of spinning on the line
            Timer::after_micros(200).await;
            continue;
        }
        let backlog = outgoing.is_some() || utility_pending.is_some();
        match select3(dev.wait_for_interrupt(), Timer::at(next_heartbeat), async {
            if backlog {
                // Transmit queue is full; check back shortly
                Timer::after_millis(1).await
            } else {
                select3(link::OUTBOX.ready_to_receive(), TX.ready_to_receive(), UTIL_TX.ready_to_receive()).await;
            }
        })
        .await
        {
            Either3::First(()) | Either3::Second(()) | Either3::Third(()) => {}
        }
    }
}
