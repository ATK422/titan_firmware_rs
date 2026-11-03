//! The claw servos and the ROS actuator commands, ported from `actuator.c` and `ros_actuators.c`.
//!
//! The C driver queued packets with a callback per packet and ran the callbacks from the UART interrupt. Here
//! [`Actuators`] holds the servo state; every method that talks to a servo hands [`Outgoing`] packets to an
//! `enqueue` function (the firmware's bounded transmit queue, which drops packets when full like the C queue),
//! and the firmware passes each answer (or UART error) back to [`Actuators::handle_response`], which runs the
//! matching callback logic.
//!
//! Only what the HBridge firmware reaches is ported: arming and disarming with read-back, timed continuous moves,
//! position reads and the status query. The C positional moves (`servo_set_deg`, homing) and ID tools were never
//! called on this board (the latter only from CANmore debug commands).

use crate::hiwonder::{self, Packet, ReadError, cmd};

/// Servos on the board.
pub const NUM_SERVOS: usize = 2;
/// Index of the claw grip servo (ID 3).
pub const CLAW_GRIP: usize = 0;
/// Index of the claw rack servo (ID 4).
pub const CLAW_RACK: usize = 1;
/// Bus IDs, in index order (`init_servos`).
pub const SERVO_IDS: [u8; NUM_SERVOS] = [3, 4];
/// Retries after errors before giving up (`MAX_NUM_ERRORS`).
pub const MAX_NUM_ERRORS: u8 = 10;
/// Largest speed accepted by a timed move (`SERVO_MAX_SPEED`).
pub const SERVO_MAX_SPEED: i32 = 1000;
/// How long a timed move runs before the stop command (`DEBUG_MOVE_TIME_MS`).
pub const MOVE_TIME_MS: u64 = 1000;

/// `riptide_msgs2/ActuatorStatus` claw states used by the board.
pub mod claw_state {
    pub const ERROR: u8 = 0;
    pub const DISARMED: u8 = 1;
    pub const SPOON_CLOSED: u8 = 5;
}

/// What to do with the answer to a packet: the C `on_read` callback.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Callback {
    /// `servo_ping_cb`
    Ping,
    /// `servo_is_armed_cb`
    IsArmed,
    /// `servo_read_deg_cb`
    ReadDeg,
}

/// A packet for the transmit queue.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Outgoing {
    /// Index of the servo the packet is for.
    pub servo: usize,
    pub packet: Packet,
    /// `None` for commands the servo doesn't answer.
    pub callback: Option<Callback>,
}

/// Why a command was refused. The messages are the C feedback strings.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CommandError {
    /// "Kill Switch Removed"
    KillSwitchRemoved,
    /// "At least one actuator already armed"
    AlreadyArmed,
    /// "Requested speed out of range"
    SpeedOutOfRange,
}

/// One servo's state (`servo_t`, the fields this board uses).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Servo {
    pub id: u8,
    /// Answered a ping; only pings set it.
    pub connected: bool,
    /// Torque on, as read back after arming.
    pub enabled: bool,
    pub desired_armed: bool,
    /// Errors since the last successful ping; retries stop at [`MAX_NUM_ERRORS`].
    pub num_errors: u8,
    /// Last position read, in degrees.
    pub curr_deg: u16,
}

impl Servo {
    /// `make_servo`
    pub const fn new(id: u8) -> Self {
        Servo { id, connected: false, enabled: false, desired_armed: false, num_errors: 0, curr_deg: 0 }
    }
}

/// The board's servos.
#[derive(Debug)]
pub struct Actuators {
    pub servos: [Servo; NUM_SERVOS],
}

impl Default for Actuators {
    fn default() -> Self {
        Self::new()
    }
}

impl Actuators {
    pub const fn new() -> Self {
        Actuators { servos: [Servo::new(SERVO_IDS[0]), Servo::new(SERVO_IDS[1])] }
    }

    fn send(
        &self,
        servo: usize,
        command: u8,
        params: &[u8],
        callback: Option<Callback>,
        enqueue: &mut impl FnMut(Outgoing),
    ) {
        enqueue(Outgoing { servo, packet: Packet::command(self.servos[servo].id, command, params), callback });
    }

    /// Asks a servo for its temperature to check it's there (`servo_ping`).
    pub fn ping(&mut self, servo: usize, enqueue: &mut impl FnMut(Outgoing)) {
        self.send(servo, cmd::TEMP_READ, &[], Some(Callback::Ping), enqueue);
    }

    /// Turns a servo's torque on or off and reads it back (`servo_set_armed`).
    pub fn set_armed(&mut self, servo: usize, armed: bool, enqueue: &mut impl FnMut(Outgoing)) {
        self.send(servo, cmd::LOAD_OR_UNLOAD_WRITE, &[armed as u8], None, enqueue);
        self.servos[servo].desired_armed = armed;
        self.send(servo, cmd::LOAD_OR_UNLOAD_READ, &[], Some(Callback::IsArmed), enqueue);
    }

    /// Requests a position read (`servo_read_deg`). The answer updates [`Servo::curr_deg`].
    pub fn read_deg(&mut self, servo: usize, enqueue: &mut impl FnMut(Outgoing)) {
        self.send(servo, cmd::POS_READ, &[], Some(Callback::ReadDeg), enqueue);
    }

    /// Switches a servo to motor mode at `speed` (`servo_continuous_move_ms`). The caller sends
    /// [`Actuators::continuous_stop`] after the move time.
    pub fn continuous_move(&mut self, servo: usize, speed: i16, enqueue: &mut impl FnMut(Outgoing)) {
        let [lsb, msb] = (speed as u16).to_le_bytes();
        // The second parameter is unused by the servo; C sent an uninitialized byte
        self.send(servo, cmd::OR_MOTOR_MODE_WRITE, &[1, 0, lsb, msb], None, enqueue);
    }

    /// Stops a continuous move (`servo_continuous_stop_cb`).
    pub fn continuous_stop(&mut self, servo: usize, enqueue: &mut impl FnMut(Outgoing)) {
        self.continuous_move(servo, 0, enqueue);
    }

    /// Arms every servo unless the vehicle is killed (`actuators_arm`).
    ///
    /// As in C, a servo that's already armed makes the command fail but the others are still armed.
    pub fn arm(&mut self, killed: bool, enqueue: &mut impl FnMut(Outgoing)) -> Result<(), CommandError> {
        if killed {
            return Err(CommandError::KillSwitchRemoved);
        }
        let mut result = Ok(());
        for servo in 0..NUM_SERVOS {
            if self.servos[servo].enabled {
                result = Err(CommandError::AlreadyArmed);
            } else {
                self.set_armed(servo, true, enqueue);
            }
        }
        result
    }

    /// Disarms every servo (the `false` branch of `arm_subscription_callback`).
    pub fn disarm(&mut self, enqueue: &mut impl FnMut(Outgoing)) {
        for servo in 0..NUM_SERVOS {
            self.set_armed(servo, false, enqueue);
        }
    }

    /// Starts a timed move of the claw grip (`grip_move_time`). On success the caller sends
    /// [`Actuators::continuous_stop`] for [`CLAW_GRIP`] after [`MOVE_TIME_MS`].
    pub fn grip_move_time(&mut self, speed: i32, enqueue: &mut impl FnMut(Outgoing)) -> Result<(), CommandError> {
        if speed.unsigned_abs() > SERVO_MAX_SPEED as u32 {
            return Err(CommandError::SpeedOutOfRange);
        }
        self.continuous_move(CLAW_GRIP, speed as i16, enqueue);
        Ok(())
    }

    /// The claw state for `state/actuator/status` (`claw_grip_get_state`).
    pub fn claw_state(&self) -> u8 {
        let grip = &self.servos[CLAW_GRIP];
        if !grip.connected {
            claw_state::ERROR
        } else if !grip.enabled {
            claw_state::DISARMED
        } else {
            claw_state::SPOON_CLOSED
        }
    }

    /// Runs the callback of `sent` with its answer. `response` is the raw bytes received, or `Err` if the UART
    /// failed.
    ///
    /// The C driver passed an uninitialized servo pointer to the callback on UART errors; here the callback gets
    /// the servo the packet was sent to.
    pub fn handle_response(
        &mut self,
        sent: &Outgoing,
        response: Result<&[u8], ()>,
        enqueue: &mut impl FnMut(Outgoing),
    ) -> Result<(), ReadError> {
        let Some(callback) = sent.callback else { return Ok(()) };
        let parsed = match response {
            Ok(raw) => hiwonder::parse_response(&sent.packet, raw).map_err(|(err, _)| err),
            Err(()) => Err(ReadError::Uart),
        };
        let idx = sent.servo;
        let servo = &mut self.servos[idx];
        match callback {
            Callback::Ping => match parsed {
                Ok(_) => {
                    servo.num_errors = 0;
                    servo.connected = true;
                }
                Err(_) if servo.num_errors > MAX_NUM_ERRORS => servo.connected = false,
                Err(_) => servo.num_errors += 1,
            },
            Callback::IsArmed => match parsed {
                Ok(p) if p.params[0] == servo.desired_armed as u8 => servo.enabled = p.params[0] != 0,
                _ => {
                    if servo.num_errors < MAX_NUM_ERRORS {
                        servo.num_errors += 1;
                        let desired = servo.desired_armed;
                        self.set_armed(idx, desired, enqueue);
                    }
                }
            },
            Callback::ReadDeg => match parsed {
                Ok(p) => servo.curr_deg = hiwonder::pos_to_deg(p.param_u16()),
                Err(_) => {
                    if servo.num_errors < MAX_NUM_ERRORS {
                        servo.num_errors += 1;
                        self.read_deg(idx, enqueue);
                    }
                }
            },
        }
        parsed.map(|_| ())
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use std::vec::Vec;

    use super::*;
    use crate::hiwonder::MAX_PACKET_SIZE;

    fn answer(sent: &Outgoing, params: &[u8]) -> ([u8; MAX_PACKET_SIZE], usize) {
        let p = Packet {
            id: sent.packet.id,
            command: sent.packet.command,
            length: hiwonder::response_length(sent.packet.command).unwrap(),
            params: core::array::from_fn(|i| params.get(i).copied().unwrap_or(0)),
        };
        let mut buf = [0; MAX_PACKET_SIZE];
        let len = p.encode(&mut buf).len();
        (buf, len)
    }

    #[test]
    fn arming_writes_then_reads_back() {
        let mut a = Actuators::new();
        let mut q = Vec::new();
        assert_eq!(a.arm(false, &mut |o| q.push(o)), Ok(()));
        let commands: Vec<_> = q.iter().map(|o| (o.packet.id, o.packet.command, o.callback)).collect();
        assert_eq!(
            commands,
            [
                (3, cmd::LOAD_OR_UNLOAD_WRITE, None),
                (3, cmd::LOAD_OR_UNLOAD_READ, Some(Callback::IsArmed)),
                (4, cmd::LOAD_OR_UNLOAD_WRITE, None),
                (4, cmd::LOAD_OR_UNLOAD_READ, Some(Callback::IsArmed)),
            ]
        );
        assert_eq!(q[0].packet.params(), &[1]);

        // Read-back confirms the grip
        let read = q[1];
        let (buf, len) = answer(&read, &[1]);
        let mut more = Vec::new();
        assert_eq!(a.handle_response(&read, Ok(&buf[..len]), &mut |o| more.push(o)), Ok(()));
        assert!(a.servos[CLAW_GRIP].enabled);
        assert!(more.is_empty());

        // Arming again fails for the grip but still arms the rack
        q.clear();
        assert_eq!(a.arm(false, &mut |o| q.push(o)), Err(CommandError::AlreadyArmed));
        assert_eq!(q.len(), 2);
        assert!(q.iter().all(|o| o.servo == CLAW_RACK));
    }

    #[test]
    fn arming_refused_while_killed() {
        let mut a = Actuators::new();
        let mut q = Vec::new();
        assert_eq!(a.arm(true, &mut |o| q.push(o)), Err(CommandError::KillSwitchRemoved));
        assert!(q.is_empty());
    }

    #[test]
    fn failed_read_back_retries_up_to_the_limit() {
        let mut a = Actuators::new();
        let mut q = Vec::new();
        a.set_armed(CLAW_RACK, true, &mut |o| q.push(o));
        let read = q[1];
        let mut retries = 0;
        loop {
            let mut more = Vec::new();
            assert_eq!(a.handle_response(&read, Err(()), &mut |o| more.push(o)), Err(ReadError::Uart));
            if more.is_empty() {
                break;
            }
            assert_eq!(more.len(), 2);
            retries += 1;
        }
        assert_eq!(retries, MAX_NUM_ERRORS);
        assert!(!a.servos[CLAW_RACK].enabled);

        // A read-back showing the wrong state counts as an error too
        let mut b = Actuators::new();
        q.clear();
        b.set_armed(CLAW_GRIP, true, &mut |o| q.push(o));
        let (buf, len) = answer(&q[1], &[0]);
        let mut more = Vec::new();
        assert_eq!(b.handle_response(&q[1], Ok(&buf[..len]), &mut |o| more.push(o)), Ok(()));
        assert_eq!(more.len(), 2);
        assert_eq!(b.servos[CLAW_GRIP].num_errors, 1);
    }

    #[test]
    fn position_reads_update_degrees() {
        let mut a = Actuators::new();
        let mut q = Vec::new();
        a.read_deg(CLAW_GRIP, &mut |o| q.push(o));
        let (buf, len) = answer(&q[0], &[0xF4, 0x01]);
        a.handle_response(&q[0], Ok(&buf[..len]), &mut |_| panic!("no retry")).unwrap();
        assert_eq!(a.servos[CLAW_GRIP].curr_deg, 120);

        // Errors retry the read
        let mut more = Vec::new();
        let (mut buf, len) = answer(&q[0], &[0, 0]);
        buf[len - 1] ^= 1;
        assert_eq!(a.handle_response(&q[0], Ok(&buf[..len]), &mut |o| more.push(o)), Err(ReadError::BadChecksum));
        assert_eq!(more.len(), 1);
        assert_eq!(more[0].packet.command, cmd::POS_READ);
        assert_eq!(a.servos[CLAW_GRIP].curr_deg, 120);
    }

    #[test]
    fn timed_moves_check_the_speed() {
        let mut a = Actuators::new();
        let mut q = Vec::new();
        assert_eq!(a.grip_move_time(1001, &mut |o| q.push(o)), Err(CommandError::SpeedOutOfRange));
        assert_eq!(a.grip_move_time(i32::MIN, &mut |o| q.push(o)), Err(CommandError::SpeedOutOfRange));
        assert!(q.is_empty());
        assert_eq!(a.grip_move_time(-1000, &mut |o| q.push(o)), Ok(()));
        a.continuous_stop(CLAW_GRIP, &mut |o| q.push(o));
        assert_eq!(q[0].packet.command, cmd::OR_MOTOR_MODE_WRITE);
        assert_eq!(q[0].packet.params(), &[1, 0, 0x18, 0xFC]);
        assert_eq!(q[1].packet.params(), &[1, 0, 0, 0]);
        assert!(q.iter().all(|o| o.servo == CLAW_GRIP && o.callback.is_none()));
    }

    #[test]
    fn status_follows_ping_and_arming() {
        let mut a = Actuators::new();
        assert_eq!(a.claw_state(), claw_state::ERROR);
        let mut q = Vec::new();
        a.ping(CLAW_GRIP, &mut |o| q.push(o));
        let (buf, len) = answer(&q[0], &[35]);
        a.handle_response(&q[0], Ok(&buf[..len]), &mut |_| {}).unwrap();
        assert_eq!(a.claw_state(), claw_state::DISARMED);
        a.servos[CLAW_GRIP].enabled = true;
        assert_eq!(a.claw_state(), claw_state::SPOON_CLOSED);

        // Losing the servo takes more than MAX_NUM_ERRORS failed pings
        for _ in 0..=MAX_NUM_ERRORS {
            a.handle_response(&q[0], Err(()), &mut |_| {}).unwrap_err();
            assert!(a.servos[CLAW_GRIP].connected);
        }
        a.handle_response(&q[0], Err(()), &mut |_| {}).unwrap_err();
        assert!(!a.servos[CLAW_GRIP].connected);
    }
}
