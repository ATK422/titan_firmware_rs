//! Dynamixel actuators (`actuators_v2/actuators.c`): wires the Dynamixel driver to the actuator logic in
//! `actuator_core::v2`.
//!
//! The driver task polls both servos; its events update [`ACTUATORS`], whose commands queue servo writes back to
//! the driver. Everything that touches [`ACTUATORS`] does so inside its critical section mutex, which also makes
//! arming atomic with respect to a kill, as `save_and_disable_interrupts` did in C.

use core::cell::RefCell;
use core::fmt::Write;

use embassy_executor::Spawner;
use embassy_rp::Peri;
use embassy_rp::dma;
use embassy_rp::gpio::{Flex, Pull};
use embassy_rp::peripherals::{PIN_9, PIN_10, PIO0};
use embassy_rp::pio::{Common, Irq, IrqFlags, StateMachine};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Instant;

use actuator_core::v2::{Actuators, CLAW_ID, MARKER_TORPEDO_ID, RamReport, SERVO_IDS, ServoBus};
use canmore::reg_mapped::interface::debug::REMOTE_CMD_RESP_MAX_LEN;
use dynamixel::uart::{HalfDuplexUart, MultidropProgram};
use dynamixel::{Driver, Event, Id, Servos};
use titan_link::DeviceToHost;
use titan_link::actuator::{
    self, ActuatorError, ClawState, Command, DropperState, DynamixelStatus, Status, TorpedoState, Variant,
};
use titan_rp2040::control::{self, Args, Command as RemoteCommand, Response};
use titan_rp2040::{link, safety};

use crate::dynamixel_cmds::{self, parse_decimal};
use crate::{FAULT_ACTUATOR_FAILURE, FAULT_ACTUATOR_HW_FAULT, FAULT_ACTUATOR_UNPLUGGED};

/// The servos on the bus.
pub(crate) static SERVOS: Servos<2> = Servos::new(SERVO_IDS);
/// The actuator state.
static ACTUATORS: Mutex<CriticalSectionRawMutex, RefCell<Actuators>> = Mutex::new(RefCell::new(Actuators::new()));

const _: () = assert!(SERVO_IDS[0] == MARKER_TORPEDO_ID && SERVO_IDS[1] == CLAW_ID);

/// PIO0 state machine 1 (as in C) raises IRQ 1 when a transmission ends and IRQ 2 on a framing error.
type Bus = Driver<'static, PIO0, 1, 1, 2, 2>;

/// The peripherals the V2 actuators use.
pub struct Hardware<'a> {
    pub common: &'a mut Common<'static, PIO0>,
    pub sm: StateMachine<'static, PIO0, 1>,
    pub irq_flags: IrqFlags<'static, PIO0>,
    pub tx_irq: Irq<'static, PIO0, 1>,
    pub err_irq: Irq<'static, PIO0, 2>,
    /// `CLAW_CHECK_PIN`, the Dynamixel TTL signal.
    pub pin: Peri<'static, PIN_10>,
    /// `CLAW_PWM_PIN`, wired to the same signal; left as an input without pulls.
    pub pwm_pin: Peri<'static, PIN_9>,
    pub dma: dma::Channel<'static>,
}

/// The driver's view for the actuator logic.
struct ServoAccess;

impl ServoBus for ServoAccess {
    fn set_target_position(&mut self, id: u8, position: i32) {
        SERVOS.set_target_position(id, position);
    }
    fn enable_torque(&mut self, id: u8, enabled: bool) {
        SERVOS.enable_torque(id, enabled);
    }
    fn set_homing_offset(&mut self, id: u8, offset: i32) {
        SERVOS.set_homing_offset(id, offset);
    }
    fn request_eeprom_rescan(&mut self, id: u8) {
        SERVOS.request_eeprom_rescan(id);
    }
    fn homing_offset(&self, id: u8) -> Option<i32> {
        SERVOS.eeprom(id).map(|e| e.homing_offset)
    }
    fn present_position(&self, id: u8) -> Option<i32> {
        SERVOS.position(id)
    }
    fn raise_actuator_failure(&mut self, extra: u32) {
        safety::raise_fault(FAULT_ACTUATOR_FAILURE, extra);
    }
}

fn with_actuators<R>(f: impl FnOnce(&mut Actuators, &mut ServoAccess) -> R) -> R {
    ACTUATORS.lock(|a| f(&mut a.borrow_mut(), &mut ServoAccess))
}

fn now_ms() -> u64 {
    Instant::now().as_millis()
}

/// Brings up the Dynamixel bus (`actuators_initialize`).
pub fn init(spawner: Spawner, hw: Hardware<'_>) {
    // The PWM pin shares the signal: make sure it neither drives nor pulls it. Keep it configured for good.
    let mut pwm_pin = Flex::new(hw.pwm_pin);
    pwm_pin.set_as_input();
    pwm_pin.set_pull(Pull::None);
    core::mem::forget(pwm_pin);

    let program = MultidropProgram::new(hw.common);
    let uart = HalfDuplexUart::new(
        hw.common,
        hw.sm,
        hw.irq_flags,
        hw.tx_irq,
        hw.err_irq,
        hw.pin,
        hw.dma,
        &program,
        dynamixel::BAUD_RATE,
    );
    spawner.spawn(dynamixel_task(Driver::new(uart, &SERVOS)).unwrap());
    spawner.spawn(remote_command_task().unwrap());
}

#[embassy_executor::task]
async fn dynamixel_task(bus: Bus) -> ! {
    bus.run(on_dynamixel_event, |error| {
        defmt::error!("Dynamixel driver error: {}", error);
        safety::raise_fault(FAULT_ACTUATOR_FAILURE, error.code());
    })
    .await
}

/// `actuators_dynamixel_event_cb`.
fn on_dynamixel_event(event: Event, id: Id) {
    match event {
        Event::Connected => {
            if SERVO_IDS.iter().all(|&id| SERVOS.is_connected(id)) {
                safety::lower_fault(FAULT_ACTUATOR_UNPLUGGED);
            }
            with_actuators(|a, _| a.on_connected(id));
        }
        Event::Disconnected => {
            safety::raise_fault(FAULT_ACTUATOR_UNPLUGGED, id as u32);
            with_actuators(|a, bus| a.on_disconnected(id, bus));
        }
        Event::RamRead => {
            if let Some(ram) = SERVOS.ram(id) {
                let report = RamReport {
                    torque_enabled: ram.torque_enable,
                    moving: ram.moving,
                    goal_position: ram.goal_position,
                    present_position: ram.present_position,
                    hardware_error_status: ram.hardware_error_status,
                };
                with_actuators(|a, bus| a.on_ram_read(id, report, now_ms(), bus));
            }
        }
        Event::Alert => {
            let status = SERVOS.ram(id).map_or(0, |ram| ram.hardware_error_status);
            safety::raise_fault(FAULT_ACTUATOR_HW_FAULT, (id as u32) << 24 | status as u32);
            defmt::warn!("Dynamixel (ID {}) reporting hardware error {=u8:#04x}", id, status);
        }
        // The status refresh covers this
        Event::EepromRead => {}
    }
}

/// Stops everything (`actuators_disarm`). Safe in `on_kill`.
pub fn disarm() {
    with_actuators(|a, bus| a.disarm(bus));
}

/// Carries out a service call.
pub fn execute(command: Command) -> Result<(), ActuatorError> {
    let now = now_ms();
    with_actuators(|a, bus| match command {
        Command::Arm => {
            let result = a.arm(safety::asserting_kill(), bus);
            if result.is_ok() {
                defmt::info!("Arming actuators");
            }
            result
        }
        Command::Disarm => {
            a.disarm(bus);
            Ok(())
        }
        Command::FireTorpedo => a.fire_torpedo(now, bus),
        Command::DropMarker => a.drop_marker(now, bus),
        Command::NotifyReload => a.torpedo_notify_reload().and_then(|()| a.dropper_notify_reload()),
        Command::Claw { open: true } => a.claw_open(now, bus),
        Command::Claw { open: false } => a.claw_close(now, bus),
        Command::TorpedoMarkerMoveHome => a.torpedo_marker_move_home(bus),
        Command::TorpedoMarkerSetHome => a.torpedo_marker_set_home(bus),
        Command::ClawSetClosedPosition => a.claw_set_closed_position(bus),
        Command::SetClawTiming { .. } | Command::SetDropperTiming { .. } | Command::SetTorpedoTiming { .. } => {
            Err(ActuatorError::Unsupported)
        }
    })
}

/// `ros_actuators_update_status`.
pub fn status() -> Status {
    ACTUATORS.lock(|a| {
        let a = a.borrow();
        Status {
            variant: Variant::V2,
            armed: a.armed(),
            busy: a.busy(),
            claw: a.claw_state(),
            torpedo: a.torpedo_state(),
            torpedo_available: a.torpedo_available(),
            dropper: a.dropper_state(),
            dropper_available: a.dropper_available(),
        }
    })
}

/// Publishes the status of each connected servo (`actuator_v2_dynamixel_update_status`).
pub fn send_extra_status() {
    for id in SERVO_IDS {
        let (Some(eeprom), Some(ram)) = (SERVOS.eeprom(id), SERVOS.ram(id)) else { continue };
        let status = DynamixelStatus {
            model: eeprom.model_num,
            fw_version: eeprom.firmware_version,
            id: eeprom.id,
            hw_err_status: ram.hardware_error_status,
            torque_enable: ram.torque_enable,
            moving: ram.moving,
            voltage: ram.present_input_voltage,
            temperature: ram.present_temperature,
            position: ram.present_position,
            load: ram.present_load,
        };
        if !link::send(&DeviceToHost::Actuator(actuator::DeviceToHost::Dynamixel(status))) {
            defmt::debug!("Link queue full, skipping Dynamixel {} status", id);
            return;
        }
    }
}

/// Nothing to do on connect: V2 has no parameters.
pub fn on_ros_connected() {}

// ========================================
// CANmore remote commands
// ========================================

/// The board's remote commands: `actuator` (`actuators_initialize`) and the Dynamixel driver's
/// (`dynamixel_canmore_cmds_register`).
pub static REMOTE_COMMANDS: [RemoteCommand; 7] = {
    let dyn_cmds = dynamixel_cmds::commands();
    [
        RemoteCommand::new(
            "actuator",
            "[cmd]",
            "Issues the requested actuator command\n===Command Options===\nGlobal Commands:\n  arm:\t\tArms the \
             actuator system\n  disarm:\tDisarms the actuator system\n  status:\tReports global actuator \
             state\nTorpedo/Marker Commands:\n  drop:\t\tDrops a marker dropper\n  fire:\t\tFires a torpedo\n  \
             notifyreload:\tNotifies that the dropper/torpedo has been reloaded\n  gohome:\tSends the torpedo/marker \
             to the home position\n  sethome:\tSets the torpedo/marker to the home position to the current \
             position\nClaw Commands:\n  clawopen:\tOpen the claw\n  clawclose:\tClose the claw\n  setclosedpos\tSets \
             the claw closed position to the current position\n  clawmove [d]:\tMoves the claw by [d] ticks",
            actuator_cmd,
        ),
        dyn_cmds[0],
        dyn_cmds[1],
        dyn_cmds[2],
        dyn_cmds[3],
        dyn_cmds[4],
        dyn_cmds[5],
    ]
};

/// Runs the deferred remote commands (the ones that talk to a servo and wait for its reply).
#[embassy_executor::task]
async fn remote_command_task() -> ! {
    let mut buf = [0u8; REMOTE_CMD_RESP_MAX_LEN];
    loop {
        let request = control::next_deferred().await;
        let mut out = Response::new(&mut buf);
        let rc = {
            let args = request.args();
            match dynamixel_cmds::run_deferred(&args, &mut out).await {
                Some(rc) => rc,
                None => {
                    // Only the commands above are declared deferred
                    let _ = write!(out, "Unknown Remote Command: '{}'", request.command.name);
                    127
                }
            }
        };
        request.respond(rc, out.as_str());
    }
}

fn claw_state_name(state: ClawState) -> &'static str {
    match state {
        ClawState::Error => "ERROR",
        ClawState::Disarmed => "DISARMED",
        ClawState::Unknown => "UNKNOWN_POSITION",
        ClawState::Opened => "OPENED",
        ClawState::Closed => "CLOSED",
        ClawState::Opening => "OPENING",
        ClawState::Closing => "CLOSING",
    }
}

fn dropper_state_name(state: DropperState) -> &'static str {
    match state {
        DropperState::Error => "ERROR",
        DropperState::Disarmed => "DISARMED",
        DropperState::Ready => "READY",
        DropperState::Dropping => "DROPPING",
        DropperState::Dropped => "DROPPED",
    }
}

fn torpedo_state_name(state: TorpedoState) -> &'static str {
    match state {
        TorpedoState::Error => "ERROR",
        TorpedoState::Disarmed => "DISARMED",
        TorpedoState::Charging => "CHARGING",
        TorpedoState::Charged => "CHARGED",
        TorpedoState::Firing => "FIRING",
        TorpedoState::Fired => "FIRED",
    }
}

/// `actuators_cmd_cb`. Every sub-command only updates the actuator state and queues servo writes, so it runs
/// inside the request handler like in C.
fn actuator_cmd(args: &Args<'_>, out: &mut Response<'_>) -> i32 {
    let Some(cmd) = args.get(1) else {
        // C printed this and then went on with a null command
        let _ = writeln!(out, "Usage: @actuator [cmd]");
        return 2;
    };

    let result = match cmd {
        "arm" => execute(Command::Arm),
        "disarm" => execute(Command::Disarm),
        "status" => {
            let s = status();
            let yes_no = if s.busy { "Yes" } else { "No" };
            let _ = writeln!(out, "Actuators Busy:\t{yes_no}");
            let _ = writeln!(out, "Claw State:\t{}", claw_state_name(s.claw));
            let _ = writeln!(out, "Dropper State:\t\t{}", dropper_state_name(s.dropper));
            let _ = writeln!(out, "# Droppers Left:\t{}", s.dropper_available);
            let _ = writeln!(out, "Torpedo State:\t\t{}", torpedo_state_name(s.torpedo));
            let _ = writeln!(out, "# Torpedoes Left:\t{}", s.torpedo_available);
            return 0;
        }
        "drop" => execute(Command::DropMarker),
        "fire" => execute(Command::FireTorpedo),
        // Only the torpedoes, as in C (the ROS service reloads the droppers too)
        "notifyreload" => with_actuators(|a, _| a.torpedo_notify_reload()),
        "gohome" => execute(Command::TorpedoMarkerMoveHome),
        "sethome" => execute(Command::TorpedoMarkerSetHome),
        "clawopen" => execute(Command::Claw { open: true }),
        "clawclose" => execute(Command::Claw { open: false }),
        "setclosedpos" => execute(Command::ClawSetClosedPosition),
        "clawmove" => {
            let Some(delta) = args.get(2) else {
                let _ = writeln!(out, "Usage: @actuator clawmove [delta]");
                return 2;
            };
            let Some(delta) = parse_decimal(delta) else {
                let _ = writeln!(out, "Invalid number provided for delta");
                return 2;
            };
            let Ok(delta) = i32::try_from(delta) else {
                let _ = writeln!(out, "Number provided for delta does not fit into int32");
                return 2;
            };
            with_actuators(|a, bus| a.claw_creep(delta, bus))
        }
        other => {
            let _ = writeln!(out, "Unknown Command: '{other}'");
            return 2;
        }
    };

    match result {
        Ok(()) => 0,
        Err(err) => {
            let _ = writeln!(out, "Command '{cmd}' Failed with Error: {}", err.message());
            1
        }
    }
}
