//! Dynamixel actuators (`actuators_v2`): one servo drives the torpedo/marker cam, another the claw.
//!
//! The cam sits at its home position and swings to one of four positions to drop a marker or fire a torpedo, then
//! returns home. The claw servo turns several revolutions between its open and closed positions. Both build on the
//! shared actuator state machine of `dxl_actuator_base.c`, fed by the driver's periodic RAM reads.
//!
//! [`Actuators`] holds the whole state; the firmware keeps it behind a critical section mutex and hands it a
//! [`ServoBus`] for the servo commands and fault reports.

use titan_link::actuator::{ActuatorError, ClawState, DropperState, TorpedoState};

/// Dynamixel ID of the torpedo/marker servo.
pub const MARKER_TORPEDO_ID: u8 = 2;
/// Dynamixel ID of the claw servo.
pub const CLAW_ID: u8 = 3;
/// The servos the driver polls, in order.
pub const SERVO_IDS: [u8; 2] = [MARKER_TORPEDO_ID, CLAW_ID];

/// How close (in each direction, in ticks) a servo must get to its target for the move to count as done.
pub const TARGET_POSITION_MARGIN: i32 = 96;

/// Rest position of the torpedo/marker cam.
pub const POSITION_HOME: i32 = 2048;
pub const POSITION_DROPPER1_FIRE: i32 = 3072;
pub const POSITION_DROPPER2_FIRE: i32 = 4076;
pub const POSITION_TORPEDO1_FIRE: i32 = 1024;
pub const POSITION_TORPEDO2_FIRE: i32 = 5;
/// Longest a torpedo/marker move may take before it counts as failed.
pub const MAX_MOVEMENT_TIME_MS: u64 = 5000;

pub const CLAW_OPEN_POSITION: i32 = 12288;
pub const CLAW_CLOSE_POSITION: i32 = 6144;
/// Below this the claw has lost its position (it can't physically get there).
pub const CLAW_MIN_POSITION: i32 = 4608;
const _: () = assert!(CLAW_MIN_POSITION >= 4096, "claw min position must be above one revolution");
pub const MAX_CLAW_MOVE_TIME_MS: u64 = 10000;

/// Number of torpedoes and of markers per load.
const LOAD_COUNT: u8 = 2;

/// What the actuators need from the Dynamixel driver and the safety core.
pub trait ServoBus {
    /// Queues a goal position write.
    fn set_target_position(&mut self, id: u8, position: i32);
    /// Queues a torque enable write.
    fn enable_torque(&mut self, id: u8, enabled: bool);
    /// Queues a homing offset write.
    fn set_homing_offset(&mut self, id: u8, offset: i32);
    /// Queues an EEPROM read so the new homing offset shows up.
    fn request_eeprom_rescan(&mut self, id: u8);
    /// The homing offset from the servo's EEPROM, if it's connected.
    fn homing_offset(&self, id: u8) -> Option<i32>;
    /// The present position from the servo's RAM, if it's connected.
    fn present_position(&self, id: u8) -> Option<i32>;
    /// Raises `FAULT_ACTUATOR_FAILURE` with `extra`.
    fn raise_actuator_failure(&mut self, extra: u32);
}

/// The fields of a RAM read the actuators look at.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RamReport {
    pub torque_enabled: bool,
    pub moving: bool,
    pub goal_position: i32,
    pub present_position: i32,
    pub hardware_error_status: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Role {
    TorpedoMarker,
    Claw,
}

/// State of one servo driven actuator (`struct dxl_actuator_base_state`).
#[derive(Clone, Copy, Debug)]
struct DxlActuator {
    id: u8,
    role: Role,
    max_move_time_ms: u64,
    connected: bool,
    move_active: bool,
    /// Torque is enabled on the servo, as last read (or commanded).
    enabled: bool,
    /// The servo is somewhere a move may start from.
    homed: bool,
    hardware_err: bool,
    /// Only valid while `move_active`.
    target_position: i32,
    move_timeout_ms: u64,
}

impl DxlActuator {
    const fn new(id: u8, role: Role, max_move_time_ms: u64) -> Self {
        DxlActuator {
            id,
            role,
            max_move_time_ms,
            connected: false,
            move_active: false,
            enabled: false,
            homed: false,
            hardware_err: false,
            target_position: 0,
            move_timeout_ms: 0,
        }
    }

    /// Starts a move (`dxlact_base_set_target`).
    fn set_target(
        &mut self,
        armed: bool,
        target: i32,
        now_ms: u64,
        bus: &mut impl ServoBus,
    ) -> Result<(), ActuatorError> {
        if !self.connected {
            return Err(ActuatorError::NotConnected);
        }
        // `enabled` is the servo's actual state (an error on the servo clears it); `armed` is what safety allows,
        // which a failed disable might not have reached the servo yet
        if !self.enabled || !armed {
            return Err(ActuatorError::NotArmed);
        }
        if self.move_active {
            return Err(ActuatorError::Busy);
        }
        if self.hardware_err {
            return Err(ActuatorError::HardwareError);
        }
        if !self.homed {
            return Err(ActuatorError::NotHomed);
        }
        self.target_position = target;
        self.move_timeout_ms = now_ms + self.max_move_time_ms;
        self.move_active = true;
        bus.set_target_position(self.id, target);
        Ok(())
    }

    /// Arms the servo (`dxlact_base_arm`).
    fn arm(&mut self, bus: &mut impl ServoBus) -> Result<(), ActuatorError> {
        if !self.connected {
            return Err(ActuatorError::ActuatorNotConnected);
        }
        bus.enable_torque(self.id, true);
        self.enabled = true;
        Ok(())
    }

    /// Disables torque if it's on (`dxlact_base_safety_disable`). A failed write is retried on the next kill
    /// switch update, as the RAM reads keep `enabled` current.
    fn safety_disable(&mut self, bus: &mut impl ServoBus) {
        if self.enabled {
            bus.enable_torque(self.id, false);
            self.enabled = false;
        }
    }

    /// Checks shared by the manual (calibration) commands.
    fn check_connected_healthy(&self) -> Result<(), ActuatorError> {
        if !self.connected {
            return Err(ActuatorError::NotConnected);
        }
        if self.hardware_err {
            return Err(ActuatorError::HardwareError);
        }
        Ok(())
    }
}

/// Where the claw was found when idle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ClawPosition {
    Opened,
    Closed,
    Middle,
}

fn within(position: i32, target: i32) -> bool {
    (target - TARGET_POSITION_MARGIN..=target + TARGET_POSITION_MARGIN).contains(&position)
}

/// The V2 actuator system.
#[derive(Debug)]
pub struct Actuators {
    armed: bool,
    torpedo_marker: DxlActuator,
    claw: DxlActuator,
    torpedo_next: u8,
    dropper_next: u8,
    /// Only valid while the claw is homed.
    claw_position: ClawPosition,
}

impl Default for Actuators {
    fn default() -> Self {
        Self::new()
    }
}

impl Actuators {
    pub const fn new() -> Self {
        Actuators {
            armed: false,
            torpedo_marker: DxlActuator::new(MARKER_TORPEDO_ID, Role::TorpedoMarker, MAX_MOVEMENT_TIME_MS),
            claw: DxlActuator::new(CLAW_ID, Role::Claw, MAX_CLAW_MOVE_TIME_MS),
            torpedo_next: 0,
            dropper_next: 0,
            claw_position: ClawPosition::Middle,
        }
    }

    fn actuator(&mut self, id: u8) -> Option<&mut DxlActuator> {
        match id {
            MARKER_TORPEDO_ID => Some(&mut self.torpedo_marker),
            CLAW_ID => Some(&mut self.claw),
            _ => None,
        }
    }

    /// True while safety allows the actuators to move.
    pub fn armed(&self) -> bool {
        self.armed
    }

    // ========================================
    // Global
    // ========================================

    /// Arms the actuators (`actuators_arm`). `killed` is the kill switch state, read in the same critical section.
    pub fn arm(&mut self, killed: bool, bus: &mut impl ServoBus) -> Result<(), ActuatorError> {
        if self.armed {
            return Err(ActuatorError::AlreadyArmed);
        }
        if killed {
            return Err(ActuatorError::KillSwitchRemoved);
        }
        self.armed = true;
        for actuator in [&mut self.torpedo_marker, &mut self.claw] {
            // As in C, a servo that fails to arm doesn't fail the whole arm, so the torpedo/marker stays usable
            // without a working claw
            let _ = actuator.arm(bus);
        }
        // Re-arming starts a fresh load
        self.torpedo_next = 0;
        self.dropper_next = 0;
        Ok(())
    }

    /// Brings everything to a safe state (`actuators_disarm`). Only queues writes, so it's fine in `on_kill`.
    pub fn disarm(&mut self, bus: &mut impl ServoBus) {
        self.armed = false;
        self.torpedo_marker.safety_disable(bus);
        self.claw.safety_disable(bus);
    }

    /// An actuator is carrying out a command (`actuators_get_busy`).
    pub fn busy(&self) -> bool {
        self.torpedo_marker.move_active || self.claw.move_active
    }

    // ========================================
    // Driver events
    // ========================================

    /// A servo connected (`dxlact_base_report_connect`).
    pub fn on_connected(&mut self, id: u8) {
        if let Some(a) = self.actuator(id) {
            a.connected = true;
        }
    }

    /// A servo disconnected (`dxlact_base_report_disconnect`). A disconnect while active is a failure: the
    /// servo comes back disarmed and in an unknown state.
    pub fn on_disconnected(&mut self, id: u8, bus: &mut impl ServoBus) {
        if let Some(a) = self.actuator(id) {
            if a.enabled || a.move_active {
                bus.raise_actuator_failure(a.id as u32);
            }
            a.move_active = false;
            a.enabled = false;
            a.connected = false;
        }
    }

    /// A RAM read completed (`dxlact_base_report_state`): tracks moves to completion and idle positions.
    pub fn on_ram_read(&mut self, id: u8, ram: RamReport, now_ms: u64, bus: &mut impl ServoBus) {
        let claw_position = &mut self.claw_position;
        let a = match id {
            MARKER_TORPEDO_ID => &mut self.torpedo_marker,
            CLAW_ID => &mut self.claw,
            _ => return,
        };
        a.enabled = ram.torque_enabled;
        a.hardware_err = ram.hardware_error_status != 0;

        if !a.move_active {
            idle(a, claw_position, ram.present_position);
            return;
        }

        if now_ms >= a.move_timeout_ms {
            a.move_active = false;
            let id = a.id;
            self.disarm(bus);
            bus.raise_actuator_failure(id as u32);
            return;
        }

        // Only trust the reading if the servo is still enabled and heading where we sent it
        if ram.torque_enabled && ram.goal_position == a.target_position {
            let arrived = within(ram.present_position, a.target_position) && !ram.moving;
            if arrived {
                match next_target(a) {
                    Some(next) => {
                        a.target_position = next;
                        bus.set_target_position(a.id, next);
                    }
                    None => a.move_active = false,
                }
            }
        }
    }

    // ========================================
    // Torpedo/marker
    // ========================================

    /// `dropper_get_state`.
    pub fn dropper_state(&self) -> DropperState {
        let a = &self.torpedo_marker;
        if !a.connected || !a.homed || a.hardware_err {
            DropperState::Error
        } else if !a.enabled || !self.armed {
            DropperState::Disarmed
        } else if a.move_active {
            DropperState::Dropping
        } else if self.dropper_next == LOAD_COUNT {
            DropperState::Dropped
        } else {
            DropperState::Ready
        }
    }

    pub fn dropper_available(&self) -> u8 {
        LOAD_COUNT.saturating_sub(self.dropper_next)
    }

    /// `dropper_drop_marker`.
    pub fn drop_marker(&mut self, now_ms: u64, bus: &mut impl ServoBus) -> Result<(), ActuatorError> {
        let target = match self.dropper_next {
            0 => POSITION_DROPPER1_FIRE,
            1 => POSITION_DROPPER2_FIRE,
            _ => return Err(ActuatorError::AllDropped),
        };
        self.torpedo_marker.set_target(self.armed, target, now_ms, bus)?;
        self.dropper_next += 1;
        Ok(())
    }

    /// `dropper_notify_reload`.
    pub fn dropper_notify_reload(&mut self) -> Result<(), ActuatorError> {
        if self.torpedo_marker.move_active {
            return Err(ActuatorError::Busy);
        }
        self.dropper_next = 0;
        Ok(())
    }

    /// `torpedo_get_state`. Unlike the dropper it only looks at the servo's torque, not the arm state, as in C.
    pub fn torpedo_state(&self) -> TorpedoState {
        let a = &self.torpedo_marker;
        if !a.connected || !a.homed || a.hardware_err {
            TorpedoState::Error
        } else if !a.enabled {
            TorpedoState::Disarmed
        } else if a.move_active {
            TorpedoState::Firing
        } else if self.torpedo_next == LOAD_COUNT {
            TorpedoState::Fired
        } else {
            TorpedoState::Charged
        }
    }

    pub fn torpedo_available(&self) -> u8 {
        LOAD_COUNT.saturating_sub(self.torpedo_next)
    }

    /// `torpedo_fire`.
    pub fn fire_torpedo(&mut self, now_ms: u64, bus: &mut impl ServoBus) -> Result<(), ActuatorError> {
        let target = match self.torpedo_next {
            0 => POSITION_TORPEDO1_FIRE,
            1 => POSITION_TORPEDO2_FIRE,
            _ => return Err(ActuatorError::AllFired),
        };
        self.torpedo_marker.set_target(self.armed, target, now_ms, bus)?;
        self.torpedo_next += 1;
        Ok(())
    }

    /// `torpedo_notify_reload`.
    pub fn torpedo_notify_reload(&mut self) -> Result<(), ActuatorError> {
        if self.torpedo_marker.move_active {
            return Err(ActuatorError::Busy);
        }
        self.torpedo_next = 0;
        Ok(())
    }

    /// Makes the cam's present position its home position by adjusting the homing offset, which the servo only
    /// allows within ±1024 ticks (`torpedo_marker_set_home`).
    pub fn torpedo_marker_set_home(&mut self, bus: &mut impl ServoBus) -> Result<(), ActuatorError> {
        let a = &self.torpedo_marker;
        a.check_connected_healthy()?;
        if a.enabled || self.armed {
            return Err(ActuatorError::MustBeDisarmed);
        }
        let (Some(homing_offset), Some(present)) = (bus.homing_offset(a.id), bus.present_position(a.id)) else {
            return Err(ActuatorError::StateReadError);
        };
        let compensation = if (-1024..=1024).contains(&homing_offset) { homing_offset } else { 0 };
        let offset = (POSITION_HOME - present.wrapping_sub(compensation)) % 4096;
        if !(-1024..=1024).contains(&offset) {
            return Err(ActuatorError::HomingOffsetTooLarge);
        }
        bus.set_homing_offset(a.id, offset);
        bus.request_eeprom_rescan(a.id);
        Ok(())
    }

    /// Sends the cam home, for recovering after it moved while unpowered (`torpedo_marker_move_home`).
    pub fn torpedo_marker_move_home(&mut self, bus: &mut impl ServoBus) -> Result<(), ActuatorError> {
        let a = &self.torpedo_marker;
        a.check_connected_healthy()?;
        if !a.enabled || !self.armed {
            return Err(ActuatorError::NotArmed);
        }
        if a.move_active {
            return Err(ActuatorError::Busy);
        }
        bus.set_target_position(a.id, POSITION_HOME);
        Ok(())
    }

    // ========================================
    // Claw
    // ========================================

    /// `claw_get_state`.
    pub fn claw_state(&self) -> ClawState {
        let a = &self.claw;
        if !a.connected || !a.homed || a.hardware_err {
            ClawState::Error
        } else if !a.enabled || !self.armed {
            ClawState::Disarmed
        } else if a.move_active {
            match a.target_position {
                CLAW_OPEN_POSITION => ClawState::Opening,
                CLAW_CLOSE_POSITION => ClawState::Closing,
                _ => ClawState::Unknown,
            }
        } else {
            match self.claw_position {
                ClawPosition::Opened => ClawState::Opened,
                ClawPosition::Closed => ClawState::Closed,
                ClawPosition::Middle => ClawState::Unknown,
            }
        }
    }

    /// `claw_open`.
    pub fn claw_open(&mut self, now_ms: u64, bus: &mut impl ServoBus) -> Result<(), ActuatorError> {
        self.claw.set_target(self.armed, CLAW_OPEN_POSITION, now_ms, bus)
    }

    /// `claw_close`.
    pub fn claw_close(&mut self, now_ms: u64, bus: &mut impl ServoBus) -> Result<(), ActuatorError> {
        self.claw.set_target(self.armed, CLAW_CLOSE_POSITION, now_ms, bus)
    }

    /// Makes the claw's present position its closed position (`claw_set_closed_position`), for when it loses its
    /// multi-turn offset.
    pub fn claw_set_closed_position(&mut self, bus: &mut impl ServoBus) -> Result<(), ActuatorError> {
        let a = &self.claw;
        a.check_connected_healthy()?;
        if a.enabled || self.armed {
            return Err(ActuatorError::MustBeDisarmed);
        }
        let (Some(homing_offset), Some(present)) = (bus.homing_offset(a.id), bus.present_position(a.id)) else {
            return Err(ActuatorError::StateReadError);
        };
        let offset = CLAW_CLOSE_POSITION.wrapping_sub(present.wrapping_sub(homing_offset));
        bus.set_homing_offset(a.id, offset);
        bus.request_eeprom_rescan(a.id);
        Ok(())
    }

    /// Moves the claw by `delta` ticks from where it is (`claw_creep_delta`).
    pub fn claw_creep(&mut self, delta: i32, bus: &mut impl ServoBus) -> Result<(), ActuatorError> {
        let a = &self.claw;
        a.check_connected_healthy()?;
        if !a.enabled || !self.armed {
            return Err(ActuatorError::NotArmed);
        }
        if a.move_active {
            return Err(ActuatorError::Busy);
        }
        let present = bus.present_position(a.id).ok_or(ActuatorError::StateReadError)?;
        bus.set_target_position(a.id, present.wrapping_add(delta));
        Ok(())
    }
}

/// The idle position handlers (`torpedo_marker_idle_handler`, `claw_idle_handler`).
fn idle(a: &mut DxlActuator, claw_position: &mut ClawPosition, present: i32) {
    match a.role {
        Role::TorpedoMarker => a.homed = within(present, POSITION_HOME),
        Role::Claw => {
            a.homed = present >= CLAW_MIN_POSITION;
            if a.homed {
                // The C window was `>= target - margin && <= target - margin` (one position only); a C bug, so the
                // claw nearly always read as unknown. Use the same ±margin window as everywhere else.
                *claw_position = if within(present, CLAW_CLOSE_POSITION) {
                    ClawPosition::Closed
                } else if within(present, CLAW_OPEN_POSITION) {
                    ClawPosition::Opened
                } else {
                    ClawPosition::Middle
                };
            }
        }
    }
}

/// The move done handlers: the cam returns home after reaching a fire position; the claw just stops.
fn next_target(a: &DxlActuator) -> Option<i32> {
    match a.role {
        Role::TorpedoMarker if a.target_position != POSITION_HOME => Some(POSITION_HOME),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    #[derive(Default)]
    struct Bus {
        targets: Vec<(u8, i32)>,
        torque: Vec<(u8, bool)>,
        homing: Vec<(u8, i32)>,
        rescans: Vec<u8>,
        failures: Vec<u32>,
        homing_offset: Option<i32>,
        present: Option<i32>,
    }

    impl ServoBus for Bus {
        fn set_target_position(&mut self, id: u8, position: i32) {
            self.targets.push((id, position));
        }
        fn enable_torque(&mut self, id: u8, enabled: bool) {
            self.torque.push((id, enabled));
        }
        fn set_homing_offset(&mut self, id: u8, offset: i32) {
            self.homing.push((id, offset));
        }
        fn request_eeprom_rescan(&mut self, id: u8) {
            self.rescans.push(id);
        }
        fn homing_offset(&self, _id: u8) -> Option<i32> {
            self.homing_offset
        }
        fn present_position(&self, _id: u8) -> Option<i32> {
            self.present
        }
        fn raise_actuator_failure(&mut self, extra: u32) {
            self.failures.push(extra);
        }
    }

    fn ram(torque: bool, moving: bool, goal: i32, present: i32) -> RamReport {
        RamReport {
            torque_enabled: torque,
            moving,
            goal_position: goal,
            present_position: present,
            hardware_error_status: 0,
        }
    }

    /// Both servos connected, idle at home/closed, disarmed.
    fn connected(bus: &mut Bus) -> Actuators {
        let mut a = Actuators::new();
        a.on_connected(MARKER_TORPEDO_ID);
        a.on_connected(CLAW_ID);
        a.on_ram_read(MARKER_TORPEDO_ID, ram(false, false, 0, POSITION_HOME + 10), 0, bus);
        a.on_ram_read(CLAW_ID, ram(false, false, 0, CLAW_CLOSE_POSITION - 50), 0, bus);
        a
    }

    fn armed(bus: &mut Bus) -> Actuators {
        let mut a = connected(bus);
        a.arm(false, bus).unwrap();
        a
    }

    #[test]
    fn states_before_connect_are_errors() {
        let a = Actuators::new();
        assert_eq!(a.torpedo_state(), TorpedoState::Error);
        assert_eq!(a.dropper_state(), DropperState::Error);
        assert_eq!(a.claw_state(), ClawState::Error);
        assert_eq!((a.torpedo_available(), a.dropper_available()), (2, 2));
        assert!(!a.busy());
    }

    #[test]
    fn disarmed_states_and_claw_position() {
        let mut bus = Bus::default();
        let a = connected(&mut bus);
        assert_eq!(a.torpedo_state(), TorpedoState::Disarmed);
        assert_eq!(a.dropper_state(), DropperState::Disarmed);
        assert_eq!(a.claw_state(), ClawState::Disarmed);
        assert_eq!(a.claw_position, ClawPosition::Closed);
    }

    #[test]
    fn arm_checks() {
        let mut bus = Bus::default();
        let mut a = connected(&mut bus);
        assert_eq!(a.arm(true, &mut bus), Err(ActuatorError::KillSwitchRemoved));
        assert!(!a.armed());
        a.arm(false, &mut bus).unwrap();
        assert_eq!(bus.torque, [(MARKER_TORPEDO_ID, true), (CLAW_ID, true)]);
        assert_eq!(a.arm(false, &mut bus), Err(ActuatorError::AlreadyArmed));
        assert_eq!(a.torpedo_state(), TorpedoState::Charged);
        assert_eq!(a.dropper_state(), DropperState::Ready);
        assert_eq!(a.claw_state(), ClawState::Closed);
    }

    #[test]
    fn arm_succeeds_without_claw() {
        let mut bus = Bus::default();
        let mut a = Actuators::new();
        a.on_connected(MARKER_TORPEDO_ID);
        a.arm(false, &mut bus).unwrap();
        assert_eq!(bus.torque, [(MARKER_TORPEDO_ID, true)]);
    }

    #[test]
    fn disarm_disables_enabled_servos_once() {
        let mut bus = Bus::default();
        let mut a = armed(&mut bus);
        bus.torque.clear();
        a.disarm(&mut bus);
        assert_eq!(bus.torque, [(MARKER_TORPEDO_ID, false), (CLAW_ID, false)]);
        a.disarm(&mut bus);
        assert_eq!(bus.torque.len(), 2);
        assert!(!a.armed());
    }

    #[test]
    fn torpedo_fire_sequence() {
        let mut bus = Bus::default();
        let mut a = armed(&mut bus);
        a.fire_torpedo(100, &mut bus).unwrap();
        assert_eq!(bus.targets, [(MARKER_TORPEDO_ID, POSITION_TORPEDO1_FIRE)]);
        assert_eq!(a.torpedo_state(), TorpedoState::Firing);
        assert!(a.busy());
        assert_eq!(a.fire_torpedo(110, &mut bus), Err(ActuatorError::Busy));
        assert_eq!(a.torpedo_notify_reload(), Err(ActuatorError::Busy));

        // Still moving: nothing happens
        a.on_ram_read(MARKER_TORPEDO_ID, ram(true, true, POSITION_TORPEDO1_FIRE, 1500), 300, &mut bus);
        assert_eq!(bus.targets.len(), 1);
        // Arrived: head home
        a.on_ram_read(MARKER_TORPEDO_ID, ram(true, false, POSITION_TORPEDO1_FIRE, 1000), 500, &mut bus);
        assert_eq!(bus.targets[1], (MARKER_TORPEDO_ID, POSITION_HOME));
        assert!(a.busy());
        // The stale goal from the previous leg is ignored
        a.on_ram_read(MARKER_TORPEDO_ID, ram(true, false, POSITION_TORPEDO1_FIRE, 1000), 700, &mut bus);
        assert!(a.busy());
        a.on_ram_read(MARKER_TORPEDO_ID, ram(true, false, POSITION_HOME, 2040), 900, &mut bus);
        assert!(!a.busy());
        assert_eq!(a.torpedo_available(), 1);

        a.fire_torpedo(1000, &mut bus).unwrap();
        assert_eq!(bus.targets[2], (MARKER_TORPEDO_ID, POSITION_TORPEDO2_FIRE));
        a.on_ram_read(MARKER_TORPEDO_ID, ram(true, false, POSITION_TORPEDO2_FIRE, 50), 1200, &mut bus);
        a.on_ram_read(MARKER_TORPEDO_ID, ram(true, false, POSITION_HOME, 2048), 1400, &mut bus);
        a.on_ram_read(MARKER_TORPEDO_ID, ram(true, false, POSITION_HOME, 2048), 1600, &mut bus);
        assert_eq!(a.torpedo_state(), TorpedoState::Fired);
        assert_eq!(a.fire_torpedo(1700, &mut bus), Err(ActuatorError::AllFired));
        a.torpedo_notify_reload().unwrap();
        assert_eq!(a.torpedo_available(), 2);
    }

    #[test]
    fn dropper_sequence_and_rearm_resets_counts() {
        let mut bus = Bus::default();
        let mut a = armed(&mut bus);
        for (i, target) in [POSITION_DROPPER1_FIRE, POSITION_DROPPER2_FIRE].into_iter().enumerate() {
            let t = i as u64 * 1000;
            a.drop_marker(t, &mut bus).unwrap();
            assert_eq!(a.dropper_state(), DropperState::Dropping);
            a.on_ram_read(MARKER_TORPEDO_ID, ram(true, false, target, target + 20), t + 200, &mut bus);
            a.on_ram_read(MARKER_TORPEDO_ID, ram(true, false, POSITION_HOME, POSITION_HOME), t + 400, &mut bus);
            a.on_ram_read(MARKER_TORPEDO_ID, ram(true, false, POSITION_HOME, POSITION_HOME), t + 600, &mut bus);
        }
        assert_eq!(a.dropper_state(), DropperState::Dropped);
        assert_eq!(a.drop_marker(5000, &mut bus), Err(ActuatorError::AllDropped));
        a.disarm(&mut bus);
        a.arm(false, &mut bus).unwrap();
        assert_eq!(a.dropper_available(), 2);
    }

    #[test]
    fn move_checks() {
        let mut bus = Bus::default();
        let mut a = connected(&mut bus);
        assert_eq!(a.fire_torpedo(0, &mut bus), Err(ActuatorError::NotArmed));
        a.arm(false, &mut bus).unwrap();
        // Off home: not homed
        a.on_ram_read(MARKER_TORPEDO_ID, ram(true, false, 0, POSITION_HOME + 200), 0, &mut bus);
        assert_eq!(a.torpedo_state(), TorpedoState::Error);
        assert_eq!(a.drop_marker(0, &mut bus), Err(ActuatorError::NotHomed));
        // Hardware error
        let mut r = ram(true, false, 0, POSITION_HOME);
        r.hardware_error_status = 0x20;
        a.on_ram_read(MARKER_TORPEDO_ID, r, 0, &mut bus);
        assert_eq!(a.drop_marker(0, &mut bus), Err(ActuatorError::HardwareError));
        // Torque dropped on the servo: not armed
        a.on_ram_read(MARKER_TORPEDO_ID, ram(false, false, 0, POSITION_HOME), 0, &mut bus);
        assert_eq!(a.drop_marker(0, &mut bus), Err(ActuatorError::NotArmed));
        a.on_disconnected(MARKER_TORPEDO_ID, &mut bus);
        assert_eq!(a.drop_marker(0, &mut bus), Err(ActuatorError::NotConnected));
        assert!(bus.targets.is_empty());
    }

    #[test]
    fn move_timeout_disarms_and_faults() {
        let mut bus = Bus::default();
        let mut a = armed(&mut bus);
        a.claw_open(1000, &mut bus).unwrap();
        assert_eq!(a.claw_state(), ClawState::Opening);
        a.on_ram_read(CLAW_ID, ram(true, true, CLAW_OPEN_POSITION, 9000), 1000 + MAX_CLAW_MOVE_TIME_MS - 1, &mut bus);
        assert!(a.busy());
        bus.torque.clear();
        a.on_ram_read(CLAW_ID, ram(true, true, CLAW_OPEN_POSITION, 9000), 1000 + MAX_CLAW_MOVE_TIME_MS, &mut bus);
        assert!(!a.busy() && !a.armed());
        assert_eq!(bus.failures, [CLAW_ID as u32]);
        assert_eq!(bus.torque, [(MARKER_TORPEDO_ID, false), (CLAW_ID, false)]);
    }

    #[test]
    fn disconnect_while_active_faults() {
        let mut bus = Bus::default();
        let mut a = connected(&mut bus);
        a.on_disconnected(CLAW_ID, &mut bus);
        assert!(bus.failures.is_empty());
        let mut a2 = armed(&mut bus);
        a2.on_disconnected(CLAW_ID, &mut bus);
        assert_eq!(bus.failures, [CLAW_ID as u32]);
        assert_eq!(a2.claw_state(), ClawState::Error);
        a.on_disconnected(99, &mut bus);
    }

    #[test]
    fn claw_open_close() {
        let mut bus = Bus::default();
        let mut a = armed(&mut bus);
        a.claw_open(0, &mut bus).unwrap();
        a.on_ram_read(CLAW_ID, ram(true, false, CLAW_OPEN_POSITION, CLAW_OPEN_POSITION + 90), 100, &mut bus);
        assert!(!a.busy());
        // Next idle read places it
        a.on_ram_read(CLAW_ID, ram(true, false, CLAW_OPEN_POSITION, CLAW_OPEN_POSITION + 90), 300, &mut bus);
        assert_eq!(a.claw_state(), ClawState::Opened);
        a.claw_close(400, &mut bus).unwrap();
        assert_eq!(a.claw_state(), ClawState::Closing);
        a.on_ram_read(CLAW_ID, ram(true, false, CLAW_CLOSE_POSITION, CLAW_CLOSE_POSITION), 600, &mut bus);
        a.on_ram_read(CLAW_ID, ram(true, false, CLAW_CLOSE_POSITION, CLAW_CLOSE_POSITION), 800, &mut bus);
        assert_eq!(a.claw_state(), ClawState::Closed);
        a.on_ram_read(CLAW_ID, ram(true, false, 0, 9000), 1000, &mut bus);
        assert_eq!(a.claw_state(), ClawState::Unknown);
        // Lost its turns offset
        a.on_ram_read(CLAW_ID, ram(true, false, 0, 100), 1200, &mut bus);
        assert_eq!(a.claw_state(), ClawState::Error);
    }

    #[test]
    fn set_home_offsets() {
        let mut bus = Bus::default();
        let mut a = connected(&mut bus);
        bus.homing_offset = Some(100);
        bus.present = Some(2448);
        // (2448 - 100) - 2048 = 300 → offset -300
        a.torpedo_marker_set_home(&mut bus).unwrap();
        assert_eq!(bus.homing, [(MARKER_TORPEDO_ID, -300)]);
        assert_eq!(bus.rescans, [MARKER_TORPEDO_ID]);

        // An out of range offset isn't compensated for
        bus.homing_offset = Some(2000);
        bus.present = Some(2048 + 4096 + 500);
        a.torpedo_marker_set_home(&mut bus).unwrap();
        assert_eq!(bus.homing[1], (MARKER_TORPEDO_ID, -500));

        bus.homing_offset = Some(0);
        bus.present = Some(2048 + 1500);
        assert_eq!(a.torpedo_marker_set_home(&mut bus), Err(ActuatorError::HomingOffsetTooLarge));

        bus.present = None;
        assert_eq!(a.torpedo_marker_set_home(&mut bus), Err(ActuatorError::StateReadError));

        a.arm(false, &mut bus).unwrap();
        assert_eq!(a.torpedo_marker_set_home(&mut bus), Err(ActuatorError::MustBeDisarmed));
        assert_eq!(a.claw_set_closed_position(&mut bus), Err(ActuatorError::MustBeDisarmed));
    }

    #[test]
    fn claw_closed_position_and_creep() {
        let mut bus = Bus::default();
        let mut a = connected(&mut bus);
        bus.homing_offset = Some(-4096);
        bus.present = Some(1000);
        // -((1000 - -4096) - 6144) = 1048
        a.claw_set_closed_position(&mut bus).unwrap();
        assert_eq!(bus.homing, [(CLAW_ID, 1048)]);
        assert_eq!(a.claw_creep(100, &mut bus), Err(ActuatorError::NotArmed));
        a.arm(false, &mut bus).unwrap();
        a.claw_creep(-200, &mut bus).unwrap();
        assert_eq!(bus.targets, [(CLAW_ID, 800)]);
    }

    #[test]
    fn move_home_checks() {
        let mut bus = Bus::default();
        let mut a = connected(&mut bus);
        assert_eq!(a.torpedo_marker_move_home(&mut bus), Err(ActuatorError::NotArmed));
        a.arm(false, &mut bus).unwrap();
        // Works off home too: that's what it's for
        a.on_ram_read(MARKER_TORPEDO_ID, ram(true, false, 0, 3500), 0, &mut bus);
        a.torpedo_marker_move_home(&mut bus).unwrap();
        assert_eq!(bus.targets, [(MARKER_TORPEDO_ID, POSITION_HOME)]);
    }
}
