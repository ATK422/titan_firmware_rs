//! Hiwonder bus servo protocol, ported from `hiwonder_driver.c` and `hiwonder_driver.h`.
//!
//! A packet on the half duplex 115200 baud bus is `0x55 0x55 id length command params.. checksum`. `length`
//! counts itself, the command, the parameters and the checksum (so it is the parameter count plus 3), and the
//! checksum is the inverted low byte of the sum of `id`, `length`, `command` and the parameters. Read commands are
//! answered by the servo with a packet of the same command and a fixed length.

/// Packet header byte, sent twice (`HEADER_CODE`).
pub const HEADER: u8 = 0x55;
/// Header bytes (`HEADER_SIZE`).
pub const HEADER_SIZE: usize = 2;
/// Checksum bytes (`CHECKSUM_SIZE`).
pub const CHECKSUM_SIZE: usize = 1;
/// Most parameter bytes in any packet (`PARAMETER_MTU`).
pub const PARAMETER_MTU: usize = 4;
/// Largest packet on the wire (`MAX_PACKET_SIZE`).
pub const MAX_PACKET_SIZE: usize = 10;
/// Bus baud rate (`UART_BAUD`).
pub const BAUD: u32 = 115_200;
/// Longest wait for a response (`UART_TIMEOUT_MS`).
pub const RESPONSE_TIMEOUT_MS: u64 = 50;
/// Servo IDs a servo can be given (`SERVO_ID_MIN`, `SERVO_ID_MAX`).
pub const ID_MIN: u8 = 1;
pub const ID_MAX: u8 = 252;
/// Broadcast ID, answered by any servo.
pub const ID_BROADCAST: u8 = 0xFE;

/// Command numbers from the Hiwonder datasheet.
pub mod cmd {
    pub const MOVE_TIME_WRITE: u8 = 1;
    pub const MOVE_TIME_READ: u8 = 2;
    pub const MOVE_TIME_WAIT_WRITE: u8 = 7;
    pub const MOVE_TIME_WAIT_READ: u8 = 8;
    pub const MOVE_START: u8 = 11;
    pub const MOVE_STOP: u8 = 12;
    pub const ID_WRITE: u8 = 13;
    pub const ID_READ: u8 = 14;
    pub const ANGLE_OFFSET_ADJUST: u8 = 17;
    pub const ANGLE_OFFSET_WRITE: u8 = 18;
    pub const ANGLE_OFFSET_READ: u8 = 19;
    pub const ANGLE_LIMIT_WRITE: u8 = 20;
    pub const ANGLE_LIMIT_READ: u8 = 21;
    pub const VIN_LIMIT_WRITE: u8 = 22;
    pub const VIN_LIMIT_READ: u8 = 23;
    pub const TEMP_MAX_LIMIT_WRITE: u8 = 24;
    pub const TEMP_MAX_LIMIT_READ: u8 = 25;
    pub const TEMP_READ: u8 = 26;
    pub const VIN_READ: u8 = 27;
    pub const POS_READ: u8 = 28;
    pub const OR_MOTOR_MODE_WRITE: u8 = 29;
    pub const OR_MOTOR_MODE_READ: u8 = 30;
    pub const LOAD_OR_UNLOAD_WRITE: u8 = 31;
    pub const LOAD_OR_UNLOAD_READ: u8 = 32;
    pub const LED_CTRL_WRITE: u8 = 33;
    pub const LED_CTRL_READ: u8 = 34;
    pub const LED_ERROR_WRITE: u8 = 35;
    pub const LED_ERROR_READ: u8 = 36;
}

/// The `length` field of a command sent to a servo (`SERVO_*_LEN`), or `None` for an unknown command.
pub const fn command_length(command: u8) -> Option<u8> {
    use cmd::*;
    Some(match command {
        MOVE_TIME_WRITE | MOVE_TIME_WAIT_WRITE | ANGLE_LIMIT_WRITE | VIN_LIMIT_WRITE | OR_MOTOR_MODE_WRITE => 7,
        ID_WRITE | ANGLE_OFFSET_ADJUST | TEMP_MAX_LIMIT_WRITE | LOAD_OR_UNLOAD_WRITE | LED_CTRL_WRITE
        | LED_ERROR_WRITE => 4,
        MOVE_TIME_READ | MOVE_TIME_WAIT_READ | MOVE_START | MOVE_STOP | ID_READ | ANGLE_OFFSET_WRITE
        | ANGLE_OFFSET_READ | ANGLE_LIMIT_READ | VIN_LIMIT_READ | TEMP_MAX_LIMIT_READ | TEMP_READ | VIN_READ
        | POS_READ | OR_MOTOR_MODE_READ | LOAD_OR_UNLOAD_READ | LED_CTRL_READ | LED_ERROR_READ => 3,
        _ => return None,
    })
}

/// The `length` field of a servo's answer to a read command (`SERVO_*_RESPONSE_LEN`), or `None` if the command
/// isn't answered.
pub const fn response_length(command: u8) -> Option<u8> {
    use cmd::*;
    Some(match command {
        MOVE_TIME_READ | MOVE_TIME_WAIT_READ | ANGLE_LIMIT_READ | VIN_LIMIT_READ | OR_MOTOR_MODE_READ => 7,
        VIN_READ | POS_READ => 5,
        ID_READ | ANGLE_OFFSET_READ | TEMP_MAX_LIMIT_READ | TEMP_READ | LOAD_OR_UNLOAD_READ | LED_CTRL_READ
        | LED_ERROR_READ => 4,
        _ => return None,
    })
}

/// One packet, in either direction.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Packet {
    /// Servo ID (the target of a command, the sender of a response).
    pub id: u8,
    pub command: u8,
    /// The length field: parameter count plus 3.
    pub length: u8,
    /// Parameters; only the first `length - 3` are used.
    pub params: [u8; PARAMETER_MTU],
}

impl Packet {
    /// A command to servo `id`. The length comes from [`command_length`]; missing parameters are zero.
    ///
    /// # Panics
    ///
    /// If the command is unknown or `params` is longer than the command takes.
    pub fn command(id: u8, command: u8, params: &[u8]) -> Self {
        let length = command_length(command).expect("unknown Hiwonder command");
        assert!(params.len() <= (length - 3) as usize, "too many parameters");
        let mut p = [0; PARAMETER_MTU];
        p[..params.len()].copy_from_slice(params);
        Packet { id, command, length, params: p }
    }

    /// The parameters actually carried.
    pub fn params(&self) -> &[u8] {
        &self.params[..(self.length as usize).saturating_sub(3).min(PARAMETER_MTU)]
    }

    /// Checksum over id, length, command and parameters (`calculate_checksum`).
    pub fn checksum(&self) -> u8 {
        let sum = self
            .params()
            .iter()
            .fold(self.id as u32 + self.length as u32 + self.command as u32, |sum, &b| sum + b as u32);
        !(sum as u8)
    }

    /// Bytes on the wire: `length + 3`.
    pub fn wire_len(&self) -> usize {
        self.length as usize + HEADER_SIZE + CHECKSUM_SIZE
    }

    /// Serializes the packet into `buf` and returns the used part (`send_packet`).
    pub fn encode<'a>(&self, buf: &'a mut [u8; MAX_PACKET_SIZE]) -> &'a [u8] {
        let len = self.wire_len();
        buf[0] = HEADER;
        buf[1] = HEADER;
        buf[2] = self.id;
        buf[3] = self.length;
        buf[4] = self.command;
        let params = self.params();
        buf[5..5 + params.len()].copy_from_slice(params);
        buf[len - 1] = self.checksum();
        &buf[..len]
    }

    /// Bytes to read for the answer to this command, or `None` if the servo doesn't answer it.
    pub fn response_wire_len(&self) -> Option<usize> {
        response_length(self.command).map(|len| len as usize + HEADER_SIZE + CHECKSUM_SIZE)
    }

    /// The first two parameters as a little endian `u16`.
    pub fn param_u16(&self) -> u16 {
        u16::from_le_bytes([self.params[0], self.params[1]])
    }
}

/// Why a response was rejected (`enum servo_read_err`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ReadError {
    /// The UART reported a timeout, framing error or overflow (`SERVO_INTERNAL_UART_ERROR`).
    Uart,
    /// The response carried another command (`SERVO_BAD_RESPONSE_TYPE`).
    BadResponseType,
    /// The response length didn't match the command (`SERVO_BAD_RESPONSE_LENGTH`).
    BadResponseLength,
    /// Another servo answered (`SERVO_INCORRECT_RESPONDER`).
    IncorrectResponder,
    /// The checksum didn't match (`SERVO_BAD_CHECKSUM`).
    BadChecksum,
}

/// Parses and checks the answer to `sent` (`on_packet_received`).
///
/// The checks run in the C order: command, length, checksum, then responder. A packet from another servo is
/// returned in the error for commands where that is expected (reading the ID over the broadcast address). The
/// header bytes aren't checked, as in C. Unlike C, a corrupt length can't index past the received bytes.
pub fn parse_response(sent: &Packet, raw: &[u8]) -> Result<Packet, (ReadError, Option<Packet>)> {
    if raw.len() < 5 {
        return Err((ReadError::BadResponseLength, None));
    }
    let (id, length, command) = (raw[2], raw[3], raw[4]);
    if command != sent.command {
        return Err((ReadError::BadResponseType, None));
    }
    if Some(length) != response_length(sent.command) || raw.len() < length as usize + HEADER_SIZE + CHECKSUM_SIZE {
        return Err((ReadError::BadResponseLength, None));
    }
    let mut packet = Packet { id, command, length, params: [0; PARAMETER_MTU] };
    let n = length as usize - 3;
    packet.params[..n].copy_from_slice(&raw[5..5 + n]);
    if raw[length as usize + HEADER_SIZE + CHECKSUM_SIZE - 1] != packet.checksum() {
        return Err((ReadError::BadChecksum, None));
    }
    if id != sent.id {
        return Err((ReadError::IncorrectResponder, Some(packet)));
    }
    Ok(packet)
}

/// Servo position units (0-1000) per degree: the servo covers 240 degrees.
const POS_PER_DEG: f64 = 1000.0 / 240.0;

/// Position command for an angle in degrees (`servo_set_deg`). Truncates like the C conversion.
pub fn deg_to_pos(deg: f32) -> u16 {
    (deg as f64 * POS_PER_DEG) as u16
}

/// Angle in whole degrees of a reported position, clamped to 1000 (`servo_read_deg_cb`).
pub fn pos_to_deg(pos: u16) -> u16 {
    (pos.min(1000) as f64 * (240.0 / 1000.0)) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encodes_a_position_read() {
        let mut buf = [0; MAX_PACKET_SIZE];
        let p = Packet::command(3, cmd::POS_READ, &[]);
        // checksum = ~(3 + 3 + 28) = ~34 = 0xDD
        assert_eq!(p.encode(&mut buf), &[0x55, 0x55, 3, 3, 28, 0xDD]);
        assert_eq!(p.response_wire_len(), Some(8));
    }

    #[test]
    fn encodes_a_motor_mode_command_with_negative_speed() {
        let mut buf = [0; MAX_PACKET_SIZE];
        let [lsb, msb] = (-500i16 as u16).to_le_bytes();
        let p = Packet::command(3, cmd::OR_MOTOR_MODE_WRITE, &[1, 0, lsb, msb]);
        let wire = p.encode(&mut buf);
        assert_eq!(wire.len(), 10);
        assert_eq!(&wire[..9], &[0x55, 0x55, 3, 7, 29, 1, 0, 0x0C, 0xFE]);
        let sum: u32 = [3u32, 7, 29, 1, 0, 0x0C, 0xFE].iter().sum();
        assert_eq!(wire[9], !(sum as u8));
        assert_eq!(p.response_wire_len(), None);
    }

    #[test]
    fn all_lengths_known() {
        for c in 1..=36u8 {
            if let Some(r) = response_length(c) {
                assert_eq!(command_length(c), Some(3), "read command {c} has no parameters");
                assert!(r as usize + 3 <= MAX_PACKET_SIZE);
            }
            if let Some(l) = command_length(c) {
                assert!(l as usize + 3 <= MAX_PACKET_SIZE);
            }
        }
    }

    fn response(id: u8, command: u8, params: &[u8]) -> ([u8; MAX_PACKET_SIZE], usize) {
        let p = Packet {
            id,
            command,
            length: params.len() as u8 + 3,
            params: core::array::from_fn(|i| params.get(i).copied().unwrap_or(0)),
        };
        let mut buf = [0; MAX_PACKET_SIZE];
        let len = p.encode(&mut buf).len();
        (buf, len)
    }

    #[test]
    fn parses_a_good_response() {
        let sent = Packet::command(3, cmd::POS_READ, &[]);
        let (buf, len) = response(3, cmd::POS_READ, &[0xF4, 0x01]);
        let got = parse_response(&sent, &buf[..len]).unwrap();
        assert_eq!(got.param_u16(), 500);
        assert_eq!(pos_to_deg(got.param_u16()), 120);
    }

    #[test]
    fn rejects_bad_responses_in_c_order() {
        let sent = Packet::command(3, cmd::POS_READ, &[]);
        let (buf, len) = response(3, cmd::TEMP_READ, &[40]);
        assert_eq!(parse_response(&sent, &buf[..len]).unwrap_err().0, ReadError::BadResponseType);

        let (buf, len) = response(3, cmd::POS_READ, &[1]);
        assert_eq!(parse_response(&sent, &buf[..len]).unwrap_err().0, ReadError::BadResponseLength);

        let (mut buf, len) = response(3, cmd::POS_READ, &[1, 2]);
        buf[len - 1] ^= 0xFF;
        assert_eq!(parse_response(&sent, &buf[..len]).unwrap_err().0, ReadError::BadChecksum);

        let (buf, len) = response(4, cmd::POS_READ, &[1, 2]);
        let (err, packet) = parse_response(&sent, &buf[..len]).unwrap_err();
        assert_eq!(err, ReadError::IncorrectResponder);
        assert_eq!(packet.unwrap().id, 4);

        // A corrupt length can't run past the buffer
        let mut buf = [0x55, 0x55, 3, 0xFF, cmd::POS_READ, 0, 0, 0];
        assert_eq!(parse_response(&sent, &buf).unwrap_err().0, ReadError::BadResponseLength);
        buf[3] = 5;
        assert_eq!(parse_response(&sent, &buf[..6]).unwrap_err().0, ReadError::BadResponseLength);
    }

    #[test]
    fn angle_conversions_truncate() {
        assert_eq!(deg_to_pos(240.0), 1000);
        assert_eq!(deg_to_pos(1.0), 4);
        assert_eq!(pos_to_deg(1000), 240);
        assert_eq!(pos_to_deg(1500), 240);
        assert_eq!(pos_to_deg(3), 0);
        assert_eq!(pos_to_deg(5), 1);
    }
}
