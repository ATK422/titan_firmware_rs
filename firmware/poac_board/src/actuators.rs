//! Torpedo/dropper servo: the hardware side of `actuator.c`, `hiwonder_driver.c` and `ros_torp.c`. The logic is
//! `power_core::actuator`.
//!
//! * [`servo_task`] owns the bus. Every 10 ms it sends the next queued packet when the bus is idle and reads the
//!   answer (`uart_scheduler`), pings the servo every second, and runs the end-of-move alarms.
//! * [`handle_command`] runs ROS commands straight away, like the C subscription callbacks, and stores the outcome
//!   for `state/actuator/cmd_feedback` and `cmd_status`.
//! * The `hwsetid` and `hwfind` remote commands ([`crate::remote_cmd`]) reach [`servo_task`] as deferred
//!   requests and run on the bus between its scheduled packets.

use core::cell::{Cell, RefCell};

use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, Ticker};
use power_core::actuator::{self, Actuators, Command};
use power_core::hiwonder::{self, MAX_PACKET_SIZE, ReadError};
use titan_link::poac::{self, ActuatorStatus};
use titan_rp2040::{control, safety};

use crate::uart::MultidropUart;

/// Servo bus baud rate (`UART_BAUD`).
pub const BAUD: u32 = 115_200;
/// Longest wait for an answer (`UART_TIMEOUT_MS`).
pub const RX_TIMEOUT: Duration = Duration::from_millis(50);
/// Bus scheduler period (`SERVO_TRANSMIT_PERIOD_MS`).
const SCHEDULER_PERIOD: Duration = Duration::from_millis(10);
/// Servo ping period (`SERVO_PING_PERIOD_MS`).
const PING_PERIOD: Duration = Duration::from_millis(1000);

static ACTUATORS: Mutex<CriticalSectionRawMutex, RefCell<Actuators>> = Mutex::new(RefCell::new(Actuators::new()));
/// Outcome of the last command not yet published (`new_cmd`, `cmd_status`, `cmd_feedback`).
static FEEDBACK: Mutex<CriticalSectionRawMutex, Cell<Option<Option<poac::CommandError>>>> = Mutex::new(Cell::new(None));

fn to_link_error(err: actuator::CommandError) -> poac::CommandError {
    use actuator::CommandError as E;
    match err {
        E::KillSwitchRemoved => poac::CommandError::KillSwitchRemoved,
        E::NotArmed => poac::CommandError::NotArmed,
        E::AllTorpedosFired => poac::CommandError::AllTorpedosFired,
        E::AlreadyFiring => poac::CommandError::AlreadyFiring,
        E::Busy => poac::CommandError::Busy,
        E::AllDropped => poac::CommandError::AllDropped,
        E::AlreadyArmed => poac::CommandError::AlreadyArmed,
        E::NotConnected => poac::CommandError::NotConnected,
        E::MustBeDisarmed => poac::CommandError::MustBeDisarmed,
    }
}

/// Runs an actuator command from ROS and records its outcome for the feedback topics.
pub fn handle_command(command: Command) {
    let killed = safety::asserting_kill();
    let result = ACTUATORS.lock(|a| a.borrow_mut().command(command, killed));
    match result {
        Ok(()) => defmt::info!("Actuator command {}", command),
        Err(err) => defmt::info!("Actuator command {} refused: {}", command, err),
    }
    FEEDBACK.lock(|f| f.set(Some(result.err().map(to_link_error))));
}

/// The outcome of the last command, once (`ros_actuators_update_cmd_feedback`).
pub fn take_feedback() -> Option<Option<poac::CommandError>> {
    FEEDBACK.lock(|f| f.take())
}

/// `state/actuator/busy` and `state/actuator/status` contents.
pub fn status() -> (bool, ActuatorStatus) {
    let s = ACTUATORS.lock(|a| a.borrow().status());
    let status = ActuatorStatus {
        actuators_armed: s.armed,
        claw_state: 0,
        torpedo_state: s.torpedo_state as u8,
        torpedo_available_count: s.torpedo_available_count,
        dropper_state: s.dropper_state as u8,
        dropper_available_count: s.dropper_available_count,
    };
    (s.busy, status)
}

fn now_ms() -> u64 {
    Instant::now().as_millis()
}

/// Drives the servo bus forever, and runs the deferred remote commands on it.
#[embassy_executor::task]
pub async fn servo_task(mut uart: MultidropUart<'static>) -> ! {
    // init_servo: move home
    ACTUATORS.lock(|a| a.borrow_mut().init());

    let mut scheduler = Ticker::every(SCHEDULER_PERIOD);
    // The C ping timer started at nil time, so the first ping goes out straight away
    let mut next_ping = Instant::now();
    loop {
        if let Either::Second(request) = select(scheduler.next(), control::next_deferred()).await {
            crate::remote_cmd::run(request, &mut uart).await;
            scheduler.reset();
            continue;
        }

        let now = Instant::now();
        if now >= next_ping {
            // A background timer in C: catch up without reporting misses
            while next_ping <= now {
                next_ping += PING_PERIOD;
            }
            ACTUATORS.lock(|a| a.borrow_mut().ping());
        }

        let Some(request) = ACTUATORS.lock(|a| {
            let mut a = a.borrow_mut();
            a.poll(now.as_millis());
            a.next_request()
        }) else {
            continue;
        };

        let mut tx = [0u8; MAX_PACKET_SIZE];
        uart.write(request.packet.encode(&mut tx)).await;

        if let Some(on_read) = request.on_read {
            let length = hiwonder::response_length(request.packet.command).unwrap_or(3);
            let mut rx = [0u8; MAX_PACKET_SIZE];
            let rx = &mut rx[..hiwonder::wire_size(length).min(MAX_PACKET_SIZE)];
            let result = match uart.read(rx).await {
                Ok(()) => hiwonder::parse_response(&request.packet, rx),
                Err(err) => {
                    defmt::warn!("Servo bus receive failed: {}", err);
                    Err(ReadError::Uart)
                }
            };
            if let Err(err) = result {
                defmt::debug!("Servo command {} failed: {}", request.packet.command, err);
            }
            ACTUATORS.lock(|a| a.borrow_mut().on_response(on_read, result, now_ms()));
        }

        // The scheduler only sends while the bus is idle; start counting the period again from here
        scheduler.reset();
    }
}
