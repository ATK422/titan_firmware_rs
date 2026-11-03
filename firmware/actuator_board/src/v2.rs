//! Dynamixel actuators (`actuators_v2/actuators.c`): wires the Dynamixel driver to the actuator logic in
//! `actuator_core::v2`.
//!
//! The driver task polls both servos; its events update [`ACTUATORS`], whose commands queue servo writes back to
//! the driver. Everything that touches [`ACTUATORS`] does so inside its critical section mutex, which also makes
//! arming atomic with respect to a kill, as `save_and_disable_interrupts` did in C.

use core::cell::RefCell;

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
use dynamixel::uart::{HalfDuplexUart, MultidropProgram};
use dynamixel::{Driver, Event, Id, Servos};
use titan_link::DeviceToHost;
use titan_link::actuator::{self, ActuatorError, Command, DynamixelStatus, Status, Variant};
use titan_rp2040::{link, safety};

use crate::{FAULT_ACTUATOR_FAILURE, FAULT_ACTUATOR_HW_FAULT, FAULT_ACTUATOR_UNPLUGGED};

/// The servos on the bus.
static SERVOS: Servos<2> = Servos::new(SERVO_IDS);
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
