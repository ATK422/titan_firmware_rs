//! Poac torpedo/dropper actuator: one Hiwonder servo that swings to a position to fire a torpedo or drop a marker
//! and then returns home. Ported from `talos/PoacBoard/src/actuators/actuator.c` (servo state machine) and
//! `ros_torp.c` (torpedo and dropper logic).
//!
//! The C driver queued servo packets and handled each answer in a callback, which could queue more packets
//! (retries, read-backs) or arm an alarm. [`Actuators`] keeps exactly that structure without the hardware:
//! * [`Actuators::next_request`] hands out the next queued packet. The firmware sends it every 10 ms when the bus
//!   is idle (`uart_scheduler`) and, if [`Request::on_read`] is set, reads the answer.
//! * [`Actuators::on_response`] runs the answer's callback.
//! * [`Actuators::poll`] runs due alarms (the end of a move).
//!
//! Every C quirk that changes behaviour is kept, among them:
//! * One error counter is shared by all operations and only some reset it; once it reaches
//!   [`MAX_NUM_ERRORS`] retries stop until a ping succeeds.
//! * Dropping a marker doesn't check the kill switch, arming or a running move, unlike firing a torpedo.
//! * Killing the vehicle doesn't disarm the servo; it only blocks firing and arming.

use heapless::{Deque, Vec};

use crate::hiwonder::{self, Packet, ReadError, cmd};

/// ID of the torpedo/dropper servo.
pub const SERVO_ID: u8 = 1;
/// Speed used to compute move times (`SERVO_MAX_DPS`).
pub const SERVO_MAX_DPS: u16 = 300;
/// Extra wait after the computed move time before a move counts as done (`SERVO_POSTMOVE_DELAY_MS`).
pub const POSTMOVE_DELAY_MS: u64 = 250;
/// Error count at which retries stop (`MAX_NUM_ERRORS`).
pub const MAX_NUM_ERRORS: u8 = 10;
/// Largest difference between the commanded and read back target position (`SERVO_MAX_TARGET_ERROR`).
pub const MAX_TARGET_ERROR: i32 = 5;
/// Packets that can wait for the bus (the C `tx_queue`).
pub const QUEUE_DEPTH: usize = 10;
/// Home position at boot.
pub const DEFAULT_HOME_DEG: u16 = 120;

/// Torpedoes loaded (`TORP_NUMBER`).
pub const TORPEDO_COUNT: u8 = 2;
/// Markers loaded (`DROPPER_NUMBER`).
pub const MARKER_COUNT: u8 = 2;
/// Servo angle that fires the first torpedo (`TORP_2_DEG`, used while two are left).
pub const TORPEDO_2_DEG: f32 = 200.0;
/// Servo angle that fires the second torpedo (`TORP_1_DEG`).
pub const TORPEDO_1_DEG: f32 = 240.0;
/// Servo angle that drops the first marker (`MARKER_2_DEG`).
pub const MARKER_2_DEG: f32 = 50.0;
/// Servo angle that drops the second marker (`MARKER_1_DEG`).
pub const MARKER_1_DEG: f32 = 0.0;

/// `riptide_msgs2/ActuatorStatus` torpedo states.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum TorpedoState {
    Error = 0,
    Disarmed = 1,
    Charging = 2,
    Charged = 3,
    Firing = 4,
    Fired = 5,
}

/// `riptide_msgs2/ActuatorStatus` dropper states.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum DropperState {
    Error = 0,
    Disarmed = 1,
    Ready = 2,
    Dropping = 3,
    Dropped = 4,
}

/// A ROS command for the actuators.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Command {
    /// `command/actuator/torpedo`
    FireTorpedo,
    /// `command/actuator/dropper`
    DropMarker,
    /// `command/actuator/notify_reload`
    NotifyReload,
    /// `command/actuator/arm`
    Arm(bool),
    /// `command/actuator/torpedo_marker/go_home`
    GoHome,
    /// `command/actuator/torpedo_marker/set_home`
    SetHome,
}

/// Why a command was refused; published on `state/actuator/cmd_feedback` with the C message text.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CommandError {
    KillSwitchRemoved,
    NotArmed,
    AllTorpedosFired,
    AlreadyFiring,
    Busy,
    AllDropped,
    AlreadyArmed,
    NotConnected,
    MustBeDisarmed,
}

/// Which callback handles the answer to a request (the C `on_read` function pointer).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OnRead {
    Ping,
    IsArmed,
    ReadTarget,
    ReadDeg,
}

/// A packet waiting for the bus.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Request {
    pub packet: Packet,
    /// Set if the servo answers; the firmware reads the answer and passes it to [`Actuators::on_response`].
    pub on_read: Option<OnRead>,
}

/// Snapshot for `state/actuator/status` and `state/actuator/busy`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Status {
    pub armed: bool,
    pub busy: bool,
    pub torpedo_state: TorpedoState,
    pub torpedo_available_count: u8,
    pub dropper_state: DropperState,
    pub dropper_available_count: u8,
}

/// Servo and torpedo/dropper state.
pub struct Actuators {
    queue: Deque<Request, QUEUE_DEPTH>,
    /// Pending end-of-move alarms (milliseconds). The C code could have more than one outstanding.
    alarms: Vec<u64, 4>,

    home_deg: u16,
    curr_deg: u16,
    is_sethome_req: bool,
    return_home_after_move: bool,
    desired_armed_state: bool,

    connected: bool,
    move_active: bool,
    enabled: bool,
    homed: bool,
    num_errors: u8,

    target_pos: u16,
    curr_move_time_ms: u16,

    num_torp: u8,
    num_marker: u8,
}

impl Actuators {
    pub const fn new() -> Self {
        Actuators {
            queue: Deque::new(),
            alarms: Vec::new(),
            home_deg: DEFAULT_HOME_DEG,
            curr_deg: 0,
            is_sethome_req: false,
            return_home_after_move: false,
            desired_armed_state: false,
            connected: false,
            move_active: false,
            enabled: false,
            homed: false,
            num_errors: 0,
            target_pos: 0,
            curr_move_time_ms: 0,
            num_torp: TORPEDO_COUNT,
            num_marker: MARKER_COUNT,
        }
    }

    /// Starts up the servo: moves it home (`init_servo`).
    pub fn init(&mut self) {
        self.go_home();
    }

    // ========================================
    // Servo (actuator.c)
    // ========================================

    fn enqueue(&mut self, command: u8, params: &[u8], on_read: Option<OnRead>) {
        // Like enqueue_packet, a full queue drops the packet
        let _ = self.queue.push_back(Request { packet: Packet::request(SERVO_ID, command, params), on_read });
    }

    /// The next packet to send, if any (`dequeue_packet`).
    pub fn next_request(&mut self) -> Option<Request> {
        self.queue.pop_front()
    }

    /// Checks that the servo answers (`servo_ping`), run every second.
    pub fn ping(&mut self) {
        self.enqueue(cmd::TEMP_READ, &[], Some(OnRead::Ping));
    }

    /// Loads or unloads the servo motor (`servo_set_armed`) and reads the state back.
    pub fn set_armed(&mut self, armed: bool) {
        self.enqueue(cmd::LOAD_OR_UNLOAD_WRITE, &[armed as u8], None);
        self.desired_armed_state = armed;
        self.enqueue(cmd::LOAD_OR_UNLOAD_READ, &[], Some(OnRead::IsArmed));
    }

    /// Starts a move to `deg` (`servo_set_deg`), returning the move time.
    fn set_deg(&mut self, deg: f32) -> u16 {
        let target = hiwonder::deg_to_position(deg);
        let move_time_ms = hiwonder::move_time_ms(deg, self.curr_deg, SERVO_MAX_DPS);
        let [t0, t1] = target.to_le_bytes();
        let [m0, m1] = move_time_ms.to_le_bytes();
        self.enqueue(cmd::MOVE_TIME_WRITE, &[t0, t1, m0, m1], None);
        self.target_pos = target;
        self.move_active = true;
        self.curr_move_time_ms = move_time_ms;
        self.enqueue(cmd::MOVE_TIME_READ, &[], Some(OnRead::ReadTarget));
        move_time_ms
    }

    fn read_deg(&mut self) {
        self.enqueue(cmd::POS_READ, &[], Some(OnRead::ReadDeg));
    }

    fn go_home(&mut self) {
        self.homed = true;
        self.set_deg(self.home_deg as f32);
    }

    fn set_home(&mut self) {
        self.is_sethome_req = true;
        self.homed = true;
        self.read_deg();
    }

    fn set_deg_then_home(&mut self, deg: f32) {
        self.set_deg(deg);
        self.return_home_after_move = true;
    }

    /// Handles the answer (or failure) for a request with [`Request::on_read`] set.
    pub fn on_response(&mut self, on_read: OnRead, result: Result<Packet, ReadError>, now_ms: u64) {
        match on_read {
            OnRead::Ping => match result {
                Err(_) if self.num_errors > MAX_NUM_ERRORS => self.connected = false,
                Err(_) => self.num_errors += 1,
                Ok(_) => {
                    self.num_errors = 0;
                    self.connected = true;
                }
            },
            OnRead::IsArmed => match result {
                Ok(rx) if rx.params[0] == self.desired_armed_state as u8 => {
                    self.enabled = rx.params[0] != 0;
                }
                _ => {
                    if self.num_errors < MAX_NUM_ERRORS {
                        self.num_errors += 1;
                        self.set_armed(self.desired_armed_state);
                    }
                }
            },
            OnRead::ReadTarget => match result {
                Ok(rx) if (rx.param_u16() as i32 - self.target_pos as i32).abs() <= MAX_TARGET_ERROR => {
                    let at = now_ms + self.curr_move_time_ms as u64 + POSTMOVE_DELAY_MS;
                    // The C alarm pool was bigger than anything this could reach; drop if somehow full
                    let _ = self.alarms.push(at);
                }
                _ => {
                    if self.num_errors < MAX_NUM_ERRORS {
                        self.num_errors += 1;
                        self.set_deg(hiwonder::position_to_deg_f32(self.target_pos));
                    } else {
                        self.move_active = false;
                    }
                }
            },
            OnRead::ReadDeg => match result {
                Ok(rx) => {
                    self.curr_deg = hiwonder::position_to_deg(rx.param_u16());
                    if self.is_sethome_req {
                        self.home_deg = self.curr_deg;
                        self.is_sethome_req = false;
                    }
                }
                Err(_) => {
                    if self.num_errors < MAX_NUM_ERRORS {
                        self.num_errors += 1;
                        self.read_deg();
                    }
                }
            },
        }
    }

    /// When the next alarm is due, if any.
    pub fn next_alarm(&self) -> Option<u64> {
        self.alarms.iter().copied().min()
    }

    /// Runs the alarms due at `now_ms` (`servo_move_complete_cb`).
    pub fn poll(&mut self, now_ms: u64) {
        while let Some(i) = self.alarms.iter().position(|at| *at <= now_ms) {
            self.alarms.swap_remove(i);
            self.move_active = false;
            self.read_deg();
            if self.return_home_after_move {
                self.go_home();
                self.return_home_after_move = false;
            }
        }
    }

    // ========================================
    // Torpedo and dropper (ros_torp.c)
    // ========================================

    fn torpedo_fire(&mut self, vehicle_killed: bool) -> Result<(), CommandError> {
        if vehicle_killed {
            return Err(CommandError::KillSwitchRemoved);
        }
        if !self.enabled {
            return Err(CommandError::NotArmed);
        }
        if self.num_torp == 0 {
            return Err(CommandError::AllTorpedosFired);
        }
        if self.move_active {
            return Err(CommandError::AlreadyFiring);
        }
        match self.num_torp {
            2 => self.set_deg_then_home(TORPEDO_2_DEG),
            1 => self.set_deg_then_home(TORPEDO_1_DEG),
            _ => {}
        }
        self.num_torp -= 1;
        Ok(())
    }

    fn torpedo_notify_reload(&mut self) -> Result<(), CommandError> {
        if self.move_active {
            return Err(CommandError::Busy);
        }
        self.num_torp = TORPEDO_COUNT;
        Ok(())
    }

    fn dropper_drop_marker(&mut self) -> Result<(), CommandError> {
        match self.num_marker {
            2 => self.set_deg_then_home(MARKER_2_DEG),
            1 => self.set_deg_then_home(MARKER_1_DEG),
            _ => return Err(CommandError::AllDropped),
        }
        self.num_marker -= 1;
        Ok(())
    }

    fn dropper_notify_reload(&mut self) -> Result<(), CommandError> {
        if self.move_active {
            return Err(CommandError::Busy);
        }
        self.num_marker = MARKER_COUNT;
        Ok(())
    }

    fn arm(&mut self, vehicle_killed: bool) -> Result<(), CommandError> {
        if self.enabled {
            return Err(CommandError::AlreadyArmed);
        }
        if vehicle_killed {
            return Err(CommandError::KillSwitchRemoved);
        }
        self.set_armed(true);
        Ok(())
    }

    fn torpedo_marker_set_home(&mut self) -> Result<(), CommandError> {
        if !self.connected {
            return Err(CommandError::NotConnected);
        }
        if self.enabled {
            return Err(CommandError::MustBeDisarmed);
        }
        self.set_home();
        Ok(())
    }

    fn torpedo_marker_move_home(&mut self) -> Result<(), CommandError> {
        if !self.connected {
            return Err(CommandError::NotConnected);
        }
        if !self.enabled {
            return Err(CommandError::NotArmed);
        }
        if self.move_active {
            return Err(CommandError::Busy);
        }
        self.go_home();
        Ok(())
    }

    /// Runs a ROS command (the subscription callbacks). The result is the command feedback.
    pub fn command(&mut self, command: Command, vehicle_killed: bool) -> Result<(), CommandError> {
        match command {
            Command::FireTorpedo => self.torpedo_fire(vehicle_killed),
            Command::DropMarker => self.dropper_drop_marker(),
            Command::NotifyReload => self.torpedo_notify_reload().and_then(|()| self.dropper_notify_reload()),
            Command::Arm(true) => self.arm(vehicle_killed),
            Command::Arm(false) => {
                self.set_armed(false);
                Ok(())
            }
            Command::GoHome => self.torpedo_marker_move_home(),
            Command::SetHome => self.torpedo_marker_set_home(),
        }
    }

    fn torpedo_state(&self) -> TorpedoState {
        if !self.connected || !self.homed {
            TorpedoState::Error
        } else if !self.enabled {
            TorpedoState::Disarmed
        } else if self.move_active {
            TorpedoState::Firing
        } else if self.num_torp > 0 {
            TorpedoState::Charged
        } else {
            TorpedoState::Fired
        }
    }

    fn dropper_state(&self) -> DropperState {
        if !self.connected || !self.homed {
            DropperState::Error
        } else if !self.enabled {
            DropperState::Disarmed
        } else if self.move_active {
            DropperState::Dropping
        } else if self.num_marker > 0 {
            DropperState::Ready
        } else {
            DropperState::Dropped
        }
    }

    /// The status published every 500 ms (`ros_actuators_update_status`).
    pub fn status(&self) -> Status {
        Status {
            armed: self.enabled,
            busy: self.move_active,
            torpedo_state: self.torpedo_state(),
            torpedo_available_count: self.num_torp,
            dropper_state: self.dropper_state(),
            dropper_available_count: self.num_marker,
        }
    }
}

impl Default for Actuators {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use std::vec::Vec;

    use super::*;

    /// A simulated servo answering every request, tracking its position and load state.
    struct Servo {
        position: u16,
        loaded: bool,
        online: bool,
        /// Report this instead of the commanded target on MOVE_TIME_READ, once
        bad_target_once: Option<u16>,
    }

    impl Servo {
        fn new() -> Self {
            Servo { position: 0, loaded: false, online: true, bad_target_once: None }
        }

        fn answer(&mut self, request: &Packet) -> Result<Packet, ReadError> {
            if !self.online {
                return Err(ReadError::Uart);
            }
            let reply = |params: &[u8]| {
                let length = params.len() as u8 + 3;
                let mut p = Packet { id: request.id, command: request.command, length, params: [0; 4] };
                p.params[..params.len()].copy_from_slice(params);
                let mut buf = [0u8; hiwonder::MAX_PACKET_SIZE];
                let raw = p.encode(&mut buf);
                hiwonder::parse_response(request, raw)
            };
            match request.command {
                cmd::MOVE_TIME_WRITE => {
                    self.position = request.param_u16();
                    Err(ReadError::Uart)
                }
                cmd::LOAD_OR_UNLOAD_WRITE => {
                    self.loaded = request.params[0] != 0;
                    Err(ReadError::Uart)
                }
                cmd::MOVE_TIME_READ => {
                    let pos = self.bad_target_once.take().unwrap_or(self.position);
                    let [a, b] = pos.to_le_bytes();
                    reply(&[a, b, 0, 0])
                }
                cmd::POS_READ => reply(&self.position.to_le_bytes()),
                cmd::LOAD_OR_UNLOAD_READ => reply(&[self.loaded as u8]),
                cmd::TEMP_READ => reply(&[35]),
                other => panic!("unexpected command {other}"),
            }
        }
    }

    /// Drives the bus like the firmware: one request per 10 ms, alarms in between. Returns sent commands.
    fn run(a: &mut Actuators, servo: &mut Servo, now: &mut u64, until: u64) -> Vec<u8> {
        let mut sent = Vec::new();
        while *now < until {
            a.poll(*now);
            if let Some(req) = a.next_request() {
                sent.push(req.packet.command);
                let answer = servo.answer(&req.packet);
                if let Some(on_read) = req.on_read {
                    a.on_response(on_read, answer, *now);
                }
            }
            *now += 10;
        }
        sent
    }

    fn run_for(a: &mut Actuators, servo: &mut Servo, now: &mut u64, duration: u64) -> Vec<u8> {
        let until = *now + duration;
        run(a, servo, now, until)
    }

    fn ready() -> (Actuators, Servo, u64) {
        let mut a = Actuators::new();
        let mut servo = Servo::new();
        let mut now = 0;
        a.init();
        a.ping();
        run(&mut a, &mut servo, &mut now, 2000);
        (a, servo, now)
    }

    #[test]
    fn boot_moves_home_and_connects() {
        let (a, servo, _) = ready();
        assert_eq!(servo.position, 500);
        let status = a.status();
        assert!(!status.armed && !status.busy);
        assert_eq!(status.torpedo_state, TorpedoState::Disarmed);
        assert_eq!(status.dropper_state, DropperState::Disarmed);
        assert_eq!(a.curr_deg, 120);
    }

    #[test]
    fn errors_until_connected() {
        let a = Actuators::new();
        assert_eq!(a.status().torpedo_state, TorpedoState::Error);
        assert_eq!(a.status().dropper_state, DropperState::Error);
    }

    #[test]
    fn arming_rules() {
        let (mut a, mut servo, mut now) = ready();
        assert_eq!(a.command(Command::Arm(true), true), Err(CommandError::KillSwitchRemoved));
        assert_eq!(a.command(Command::FireTorpedo, false), Err(CommandError::NotArmed));
        assert_eq!(a.command(Command::Arm(true), false), Ok(()));
        run_for(&mut a, &mut servo, &mut now, 100);
        assert!(servo.loaded && a.status().armed);
        assert_eq!(a.command(Command::Arm(true), false), Err(CommandError::AlreadyArmed));
        assert_eq!(a.status().torpedo_state, TorpedoState::Charged);
        assert_eq!(a.command(Command::Arm(false), true), Ok(()));
        run_for(&mut a, &mut servo, &mut now, 100);
        assert!(!servo.loaded && !a.status().armed);
    }

    #[test]
    fn fires_both_torpedoes_then_refuses() {
        let (mut a, mut servo, mut now) = ready();
        a.command(Command::Arm(true), false).unwrap();
        run_for(&mut a, &mut servo, &mut now, 100);

        assert_eq!(a.command(Command::FireTorpedo, true), Err(CommandError::KillSwitchRemoved));
        assert_eq!(a.command(Command::FireTorpedo, false), Ok(()));
        assert_eq!(a.status().torpedo_state, TorpedoState::Firing);
        assert_eq!(a.status().torpedo_available_count, 1);
        assert_eq!(a.command(Command::FireTorpedo, false), Err(CommandError::AlreadyFiring));
        assert_eq!(a.command(Command::NotifyReload, false), Err(CommandError::Busy));

        // Swing to 200 deg (833) and back home
        run_for(&mut a, &mut servo, &mut now, 50);
        assert_eq!(servo.position, 833);
        run_for(&mut a, &mut servo, &mut now, 2000);
        assert_eq!(servo.position, 500);
        assert!(!a.status().busy);

        assert_eq!(a.command(Command::FireTorpedo, false), Ok(()));
        run_for(&mut a, &mut servo, &mut now, 50);
        assert_eq!(servo.position, 1000);
        run_for(&mut a, &mut servo, &mut now, 2000);
        assert_eq!(a.status().torpedo_state, TorpedoState::Fired);
        assert_eq!(a.command(Command::FireTorpedo, false), Err(CommandError::AllTorpedosFired));

        assert_eq!(a.command(Command::NotifyReload, false), Ok(()));
        assert_eq!(a.status().torpedo_available_count, TORPEDO_COUNT);
        assert_eq!(a.status().torpedo_state, TorpedoState::Charged);
    }

    #[test]
    fn move_completes_after_move_time_plus_delay() {
        let (mut a, mut servo, mut now) = ready();
        a.command(Command::Arm(true), false).unwrap();
        run_for(&mut a, &mut servo, &mut now, 100);
        a.command(Command::FireTorpedo, false).unwrap();
        let start = now;
        // MOVE_TIME_WRITE at start, MOVE_TIME_READ at start + 10 arms the alarm: 266 ms move + 250 ms
        run(&mut a, &mut servo, &mut now, start + 20);
        assert_eq!(a.next_alarm(), Some(start + 10 + 266 + 250));
        run(&mut a, &mut servo, &mut now, start + 10 + 266 + 250);
        assert!(a.status().busy);
        // The alarm reads the position back, then starts the move home
        run(&mut a, &mut servo, &mut now, start + 10 + 266 + 280);
        assert!(a.status().busy);
        assert_eq!(servo.position, 500);
    }

    #[test]
    fn dropper_skips_safety_checks() {
        let (mut a, mut servo, mut now) = ready();
        // Not armed and killed, as in C
        assert_eq!(a.command(Command::DropMarker, true), Ok(()));
        run_for(&mut a, &mut servo, &mut now, 30);
        assert_eq!(servo.position, 208);
        assert_eq!(a.status().dropper_available_count, 1);
        run_for(&mut a, &mut servo, &mut now, 2000);
        assert_eq!(a.command(Command::DropMarker, false), Ok(()));
        run_for(&mut a, &mut servo, &mut now, 30);
        assert_eq!(servo.position, 0);
        assert_eq!(a.command(Command::DropMarker, false), Err(CommandError::AllDropped));
    }

    #[test]
    fn set_and_go_home() {
        let (mut a, mut servo, mut now) = ready();
        servo.position = 300; // Moved by hand
        assert_eq!(a.command(Command::SetHome, false), Ok(()));
        run_for(&mut a, &mut servo, &mut now, 50);
        assert_eq!(a.home_deg, 72);

        assert_eq!(a.command(Command::GoHome, false), Err(CommandError::NotArmed));
        a.command(Command::Arm(true), false).unwrap();
        run_for(&mut a, &mut servo, &mut now, 100);
        assert_eq!(a.command(Command::SetHome, false), Err(CommandError::MustBeDisarmed));
        assert_eq!(a.command(Command::GoHome, false), Ok(()));
        assert_eq!(a.command(Command::GoHome, false), Err(CommandError::Busy));
        run_for(&mut a, &mut servo, &mut now, 1000);
        assert_eq!(servo.position, 300);
    }

    #[test]
    fn home_commands_need_connection() {
        let mut a = Actuators::new();
        assert_eq!(a.command(Command::SetHome, false), Err(CommandError::NotConnected));
        assert_eq!(a.command(Command::GoHome, false), Err(CommandError::NotConnected));
    }

    #[test]
    fn bad_target_is_resent() {
        let (mut a, mut servo, mut now) = ready();
        a.command(Command::Arm(true), false).unwrap();
        run_for(&mut a, &mut servo, &mut now, 100);
        servo.bad_target_once = Some(0);
        a.command(Command::FireTorpedo, false).unwrap();
        let sent = run_for(&mut a, &mut servo, &mut now, 40);
        assert_eq!(sent, [cmd::MOVE_TIME_WRITE, cmd::MOVE_TIME_READ, cmd::MOVE_TIME_WRITE, cmd::MOVE_TIME_READ]);
        assert!(a.next_alarm().is_some());
    }

    #[test]
    fn ping_tracks_connection_with_shared_error_count() {
        let (mut a, mut servo, mut now) = ready();
        servo.online = false;
        // 11 failures are absorbed, the 12th marks the servo disconnected
        for _ in 0..=MAX_NUM_ERRORS {
            a.ping();
            run_for(&mut a, &mut servo, &mut now, 10);
            assert_eq!(a.status().torpedo_state, TorpedoState::Disarmed);
        }
        a.ping();
        run_for(&mut a, &mut servo, &mut now, 10);
        assert_eq!(a.status().torpedo_state, TorpedoState::Error);

        // Retries of other operations stop once the count is high
        assert_eq!(a.command(Command::DropMarker, false), Ok(()));
        let sent = run_for(&mut a, &mut servo, &mut now, 100);
        assert_eq!(sent, [cmd::MOVE_TIME_WRITE, cmd::MOVE_TIME_READ]);
        assert!(!a.status().busy);

        servo.online = true;
        a.ping();
        run_for(&mut a, &mut servo, &mut now, 10);
        assert_eq!(a.status().torpedo_state, TorpedoState::Disarmed);
    }

    #[test]
    fn full_queue_drops_packets() {
        let mut a = Actuators::new();
        for _ in 0..QUEUE_DEPTH + 3 {
            a.ping();
        }
        let mut n = 0;
        while a.next_request().is_some() {
            n += 1;
        }
        assert_eq!(n, QUEUE_DEPTH);
    }
}
