//! The claw servos, ported from `hiwonder_driver.c`, `ros_actuators.c` and the actuator timers in `main.c`.
//!
//! * [`uart_task`] is the C `uart_scheduler` repeating timer: every [`SERVO_TRANSMIT_PERIOD`] it sends the next
//!   queued packet and, for read commands, waits for the answer. A packet at a time, as with `packet_in_flight`.
//! * [`actuator_task`] owns the servo state ([`hbridge_core::actuators`]). It runs the commands from ROS, the
//!   answers from the UART, the stop of timed moves, and while ROS is connected the status (500 ms), command
//!   feedback (100 ms) and position (30 ms) timers.
//!
//! The C debug console commands `hwsetid`, `hwfind` and `hwping` aren't ported: the CANmore debug interface they
//! ran on doesn't exist in the Rust firmware yet.

use embassy_futures::select::{Either3, select3};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};
use heapless::{Deque, Vec};

use hbridge_core::actuators::{Actuators, CLAW_GRIP, CommandError, MOVE_TIME_MS, Outgoing};
use hbridge_core::hiwonder::{self, MAX_PACKET_SIZE};
use hbridge_core::timing::Interval;
use titan_link::DeviceToHost;
use titan_link::hbridge::{self, Feedback};
use titan_rp2040::{link, safety};

use crate::multidrop::{self, MultidropUart};

/// Time between packets (`SERVO_TRANSMIT_PERIOD_MS`).
const SERVO_TRANSMIT_PERIOD: Duration = Duration::from_millis(10);
/// `ACTUATOR_STATUS_TIME_MS`; a missed run raises `FAULT_TIMER_MISSED` as in C.
const STATUS_PERIOD_MS: u64 = 500;
/// `SERVO_UPDATE_CMD_STATUS_PERIOD_MS`
const CMD_FEEDBACK_PERIOD_MS: u64 = 100;
/// `SERVO_UPDATE_DEGREES_PERIOD_MS`
const DEGREES_PERIOD_MS: u64 = 30;
/// How often the actuator task checks the ROS connection when nothing else wakes it.
const CONNECTION_POLL: Duration = Duration::from_millis(10);
/// Timed moves that may be running at once; each command schedules its own stop, as each C call added an alarm.
const MAX_PENDING_STOPS: usize = 8;

/// A command from ROS.
#[derive(Clone, Copy, defmt::Format)]
pub enum Command {
    Arm(bool),
    MoveSpeedForTime(i32),
}

type Response = (Outgoing, Result<Vec<u8, MAX_PACKET_SIZE>, multidrop::Error>);

pub static COMMANDS: Channel<CriticalSectionRawMutex, Command, 4> = Channel::new();
/// The C `tx_queue`: 10 packets, new packets dropped when full.
static TX_QUEUE: Channel<CriticalSectionRawMutex, Outgoing, 10> = Channel::new();
static RESPONSES: Channel<CriticalSectionRawMutex, Response, 2> = Channel::new();

/// Queues a command, dropping it if the queue is full.
pub fn queue_command(command: Command) {
    if COMMANDS.try_send(command).is_err() {
        defmt::warn!("Actuator command queue full, dropping {}", command);
    }
}

fn enqueue(packet: Outgoing) {
    // enqueue_packet ignored a full queue too; with no servo answering, position reads fill it quickly
    if TX_QUEUE.try_send(packet).is_err() {
        defmt::debug!("Servo transmit queue full, dropping command {}", packet.packet.command);
    }
}

/// Sends queued packets and collects their answers.
#[embassy_executor::task]
pub async fn uart_task(mut uart: MultidropUart<'static>) -> ! {
    let mut next_tick = Instant::now() + SERVO_TRANSMIT_PERIOD;
    loop {
        // Ticks that pass during a transaction are skipped, like the C timer finding a packet in flight
        Timer::at(next_tick).await;
        let now = Instant::now();
        while next_tick <= now {
            next_tick += SERVO_TRANSMIT_PERIOD;
        }
        let Ok(outgoing) = TX_QUEUE.try_receive() else { continue };

        let mut buf = [0u8; MAX_PACKET_SIZE];
        let wire = outgoing.packet.encode(&mut buf);
        if let Err(err) = uart.write(wire).await {
            defmt::error!("Servo UART transmit failed: {}", err);
            if outgoing.callback.is_some() {
                RESPONSES.send((outgoing, Err(err))).await;
            }
            continue;
        }
        let (Some(_), Some(len)) = (outgoing.callback, outgoing.packet.response_wire_len()) else { continue };

        let mut response = Vec::new();
        response.resize(len, 0).unwrap();
        let result = uart.read(&mut response, Duration::from_millis(hiwonder::RESPONSE_TIMEOUT_MS)).await;
        if let Err(err) = result {
            defmt::error!("Async UART reported RX error: {}", err);
        }
        RESPONSES.send((outgoing, result.map(|()| response))).await;
    }
}

fn feedback(result: Result<(), CommandError>) -> (bool, Feedback) {
    match result {
        Ok(()) => (true, Feedback::None),
        Err(CommandError::KillSwitchRemoved) => (false, Feedback::KillSwitchRemoved),
        Err(CommandError::AlreadyArmed) => (false, Feedback::AlreadyArmed),
        Err(CommandError::SpeedOutOfRange) => (false, Feedback::SpeedOutOfRange),
    }
}

/// Timers that run while ROS is connected (`start_ros_timers`).
struct RosTimers {
    status: Interval,
    cmd_feedback: Interval,
    degrees: Interval,
}

impl RosTimers {
    fn start(now: u64) -> Self {
        RosTimers {
            status: Interval::starting_at(now, STATUS_PERIOD_MS),
            cmd_feedback: Interval::starting_at(now, CMD_FEEDBACK_PERIOD_MS),
            degrees: Interval::starting_at(now, DEGREES_PERIOD_MS),
        }
    }

    fn next(&self) -> u64 {
        self.status.next().min(self.cmd_feedback.next()).min(self.degrees.next())
    }
}

fn send(msg: hbridge::DeviceToHost) {
    link::send(&DeviceToHost::HBridge(msg));
}

#[embassy_executor::task]
pub async fn actuator_task() -> ! {
    let mut actuators = Actuators::new();
    let mut enqueue = enqueue;
    let mut stops: Deque<Instant, MAX_PENDING_STOPS> = Deque::new();
    let mut timers: Option<RosTimers> = None;
    // Result of the last command, waiting for the feedback timer (`new_cmd`)
    let mut new_cmd: Option<(bool, Feedback)> = None;

    loop {
        let poll = Instant::now() + CONNECTION_POLL;
        let timer_due = timers.as_ref().map_or(Instant::MAX, |t| Instant::from_millis(t.next()));
        let stop_due = stops.front().copied().unwrap_or(Instant::MAX);
        let wake_at = poll.min(timer_due).min(stop_due);

        match select3(COMMANDS.receive(), RESPONSES.receive(), Timer::at(wake_at)).await {
            Either3::First(Command::Arm(true)) => {
                new_cmd = Some(feedback(actuators.arm(safety::asserting_kill(), &mut enqueue)));
            }
            Either3::First(Command::Arm(false)) => {
                actuators.disarm(&mut enqueue);
                new_cmd = Some((true, Feedback::None));
            }
            Either3::First(Command::MoveSpeedForTime(speed)) => {
                let result = actuators.grip_move_time(speed, &mut enqueue);
                if result.is_ok() && stops.push_back(Instant::now() + Duration::from_millis(MOVE_TIME_MS)).is_err() {
                    // Can only happen with more than MAX_PENDING_STOPS moves a second; stop now rather than never
                    defmt::warn!("Too many timed moves pending, stopping the claw");
                    actuators.continuous_stop(CLAW_GRIP, &mut enqueue);
                }
                new_cmd = Some(feedback(result));
            }
            Either3::Second((sent, response)) => {
                let raw = response.as_deref().map_err(|_| ());
                if let Err(err) = actuators.handle_response(&sent, raw, &mut enqueue) {
                    defmt::debug!("Servo {} answer to command {} rejected: {}", sent.servo, sent.packet.command, err);
                }
            }
            Either3::Third(()) => {}
        }

        let now = Instant::now();
        while stops.front().is_some_and(|&at| at <= now) {
            stops.pop_front();
            actuators.continuous_stop(CLAW_GRIP, &mut enqueue);
        }

        match (&mut timers, link::is_connected()) {
            (None, true) => timers = Some(RosTimers::start(now.as_millis())),
            (Some(_), false) => timers = None,
            (Some(t), true) => {
                let ms = now.as_millis();
                if let Some(missed) = t.status.fire(ms) {
                    if missed > 0 {
                        safety::raise_fault(safety::FAULT_TIMER_MISSED, missed);
                    }
                    send(hbridge::DeviceToHost::ActuatorStatus { claw_state: actuators.claw_state() });
                }
                if t.cmd_feedback.fire(ms).is_some()
                    && let Some((success, message)) = new_cmd.take()
                {
                    send(hbridge::DeviceToHost::CmdFeedback { success, message });
                }
                if t.degrees.fire(ms).is_some() {
                    actuators.read_deg(CLAW_GRIP, &mut enqueue);
                    // Almost certainly not the answer to the read just queued, as in C
                    send(hbridge::DeviceToHost::ActuatorDegrees(actuators.servos[CLAW_GRIP].curr_deg as i32));
                }
            }
            (None, false) => {}
        }
    }
}
