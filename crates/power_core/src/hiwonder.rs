//! Hiwonder bus servo protocol (LX/HTS series), ported from `talos/PoacBoard/src/actuators/hiwonder_driver.c`.
//!
//! The servos share one half duplex UART line. Every packet, in both directions, is:
//!
//! ```text
//! 0x55 0x55 | id | length | command | params (length - 3 bytes) | checksum
//! ```
//!
//! `length` counts itself, the command, the parameters and the checksum. The checksum is the inverted low byte of
//! the sum of id, length, command and parameters. A servo answers read commands with a packet carrying the same
//! command.

/// Header byte, sent twice (`HEADER_CODE`).
pub const HEADER: u8 = 0x55;
/// Bytes before the ID (`HEADER_SIZE`).
pub const HEADER_SIZE: usize = 2;
/// Bytes after the parameters (`CHECKSUM_SIZE`).
pub const CHECKSUM_SIZE: usize = 1;
/// Most parameter bytes in any packet this driver uses (`PARAMETER_MTU`).
pub const PARAMETER_MTU: usize = 4;
/// Largest packet on the wire (`MAX_PACKET_SIZE`).
pub const MAX_PACKET_SIZE: usize = 10;
/// ID every servo answers to (used to discover a single servo's ID).
pub const BROADCAST_ID: u8 = 0xFE;

/// Command codes from the Hiwonder datasheet.
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

/// `length` field of each request (`SERVO_*_LEN`), or `None` for an unknown command.
pub fn request_length(command: u8) -> Option<u8> {
    use cmd::*;
    Some(match command {
        MOVE_TIME_WRITE | MOVE_TIME_WAIT_WRITE | ANGLE_LIMIT_WRITE | VIN_LIMIT_WRITE | OR_MOTOR_MODE_WRITE => 7,
        ID_WRITE | ANGLE_OFFSET_ADJUST | TEMP_MAX_LIMIT_WRITE | LOAD_OR_UNLOAD_WRITE | LED_CTRL_WRITE
        | LED_ERROR_WRITE => 4,
        // The datasheet gives 4 for ANGLE_OFFSET_WRITE but the C header said 3; it was never used
        MOVE_TIME_READ | MOVE_TIME_WAIT_READ | MOVE_START | MOVE_STOP | ID_READ | ANGLE_OFFSET_WRITE
        | ANGLE_OFFSET_READ | ANGLE_LIMIT_READ | VIN_LIMIT_READ | TEMP_MAX_LIMIT_READ | TEMP_READ | VIN_READ
        | POS_READ | OR_MOTOR_MODE_READ | LOAD_OR_UNLOAD_READ | LED_CTRL_READ | LED_ERROR_READ => 3,
        _ => return None,
    })
}

/// `length` field of the servo's answer to a read command (`response_len`), or `None` if it doesn't answer.
pub fn response_length(command: u8) -> Option<u8> {
    use cmd::*;
    Some(match command {
        MOVE_TIME_READ | MOVE_TIME_WAIT_READ | ANGLE_LIMIT_READ | VIN_LIMIT_READ | OR_MOTOR_MODE_READ => 7,
        VIN_READ | POS_READ => 5,
        ID_READ | ANGLE_OFFSET_READ | TEMP_MAX_LIMIT_READ | TEMP_READ | LOAD_OR_UNLOAD_READ | LED_CTRL_READ
        | LED_ERROR_READ => 4,
        _ => return None,
    })
}

/// Bytes on the wire for a packet with this `length` field.
pub const fn wire_size(length: u8) -> usize {
    length as usize + HEADER_SIZE + CHECKSUM_SIZE
}

/// One packet, either direction (`ServoPacket`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Packet {
    pub id: u8,
    pub command: u8,
    /// The `length` field: parameter count + 3.
    pub length: u8,
    /// Parameters; only the first `length - 3` are meaningful.
    pub params: [u8; PARAMETER_MTU],
}

impl Packet {
    /// A request to servo `id`, with the command's standard length.
    ///
    /// # Panics
    ///
    /// On an unknown command, or more parameters than the command takes.
    pub fn request(id: u8, command: u8, params: &[u8]) -> Self {
        let length = request_length(command).expect("unknown Hiwonder command");
        assert!(params.len() <= length as usize - 3, "too many parameters");
        let mut p = [0u8; PARAMETER_MTU];
        p[..params.len()].copy_from_slice(params);
        Packet { id, command, length, params: p }
    }

    /// Meaningful parameters.
    pub fn params(&self) -> &[u8] {
        &self.params[..(self.length as usize).saturating_sub(3).min(PARAMETER_MTU)]
    }

    /// The first two parameters as a little endian word, as positions and times are sent.
    pub fn param_u16(&self) -> u16 {
        u16::from_le_bytes([self.params[0], self.params[1]])
    }

    /// The packet's checksum (`calculate_checksum`).
    pub fn checksum(&self) -> u8 {
        let sum = self
            .params()
            .iter()
            .fold(self.id as u32 + self.length as u32 + self.command as u32, |sum, p| sum + *p as u32);
        !(sum as u8)
    }

    /// Serializes the packet into `buf`, returning the bytes to send (`send_packet`).
    pub fn encode<'a>(&self, buf: &'a mut [u8; MAX_PACKET_SIZE]) -> &'a [u8] {
        let size = wire_size(self.length).min(MAX_PACKET_SIZE);
        buf[0] = HEADER;
        buf[1] = HEADER;
        buf[2] = self.id;
        buf[3] = self.length;
        buf[4] = self.command;
        let params = self.params();
        buf[5..5 + params.len()].copy_from_slice(params);
        buf[size - 1] = self.checksum();
        &buf[..size]
    }
}

/// Why a response was refused (`enum servo_read_err`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ReadError {
    /// The UART reported a timeout, short read, framing error or overrun (`SERVO_INTERNAL_UART_ERROR`).
    Uart,
    /// The answer carries another command (`SERVO_BAD_RESPONSE_TYPE`).
    BadResponseType,
    /// The answer's length field is wrong for the command (`SERVO_BAD_RESPONSE_LENGTH`).
    BadResponseLength,
    /// Another servo answered (`SERVO_INCORRECT_RESPONDER`).
    IncorrectResponder,
    /// The checksum didn't match (`SERVO_BAD_CHECKSUM`).
    BadChecksum,
}

/// Parses the answer to `request` (`on_packet_received`). `raw` holds the bytes read, normally
/// `wire_size(response_length(request.command))`. The checks run in the C order: command, length, checksum, ID.
pub fn parse_response(request: &Packet, raw: &[u8]) -> Result<Packet, ReadError> {
    if raw.len() < HEADER_SIZE + 3 {
        return Err(ReadError::Uart);
    }
    let (id, length, command) = (raw[2], raw[3], raw[4]);
    if command != request.command {
        return Err(ReadError::BadResponseType);
    }
    if Some(length) != response_length(request.command) || raw.len() < wire_size(length) {
        return Err(ReadError::BadResponseLength);
    }
    let mut packet = Packet { id, command, length, params: [0; PARAMETER_MTU] };
    let param_count = length as usize - 3;
    packet.params[..param_count].copy_from_slice(&raw[5..5 + param_count]);
    if packet.checksum() != raw[wire_size(length) - 1] {
        return Err(ReadError::BadChecksum);
    }
    if id != request.id {
        return Err(ReadError::IncorrectResponder);
    }
    Ok(packet)
}

/// Servo position units per full travel; positions run 0 to 1000 over [`TRAVEL_DEG`].
pub const POSITION_RANGE: u16 = 1000;
/// Travel of the servo in degrees (`SERVO_MAX_DEG`).
pub const TRAVEL_DEG: f32 = 240.0;

/// Servo position for an angle, truncated like the C `uint16_t` conversion (negative angles give 0).
pub fn deg_to_position(deg: f32) -> u16 {
    (deg as f64 * (POSITION_RANGE as f64 / TRAVEL_DEG as f64)) as u16
}

/// Angle for a position read back from the servo, clamped to the travel and truncated to whole degrees.
pub fn position_to_deg(position: u16) -> u16 {
    (position.min(POSITION_RANGE) as f64 * (TRAVEL_DEG as f64 / POSITION_RANGE as f64)) as u16
}

/// Angle for a commanded position, for re-sending a move (not truncated).
pub fn position_to_deg_f32(position: u16) -> f32 {
    (position as f64 * (TRAVEL_DEG as f64 / POSITION_RANGE as f64)) as f32
}

/// Move time for travelling from `from_deg` to `to_deg` at `max_dps` degrees per second.
pub fn move_time_ms(to_deg: f32, from_deg: u16, max_dps: u16) -> u16 {
    ((to_deg - from_deg as f32).abs() as f64 * (1.0 / max_dps as f64) * 1000.0) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encodes_move_command() {
        // Move servo 1 to position 500 in 1000 ms; checksum computed by hand:
        // 1 + 7 + 1 + 0xF4 + 0x01 + 0xE8 + 0x03 = 0x1E9, !0xE9 = 0x16
        let p = Packet::request(1, cmd::MOVE_TIME_WRITE, &[0xF4, 0x01, 0xE8, 0x03]);
        let mut buf = [0u8; MAX_PACKET_SIZE];
        assert_eq!(p.encode(&mut buf), &[0x55, 0x55, 0x01, 0x07, 0x01, 0xF4, 0x01, 0xE8, 0x03, 0x16]);
    }

    #[test]
    fn encodes_read_commands_without_params() {
        let p = Packet::request(1, cmd::POS_READ, &[]);
        let mut buf = [0u8; MAX_PACKET_SIZE];
        // 1 + 3 + 28 = 32, !32 = 0xDF
        assert_eq!(p.encode(&mut buf), &[0x55, 0x55, 0x01, 0x03, 0x1C, 0xDF]);
        let p = Packet::request(1, cmd::LOAD_OR_UNLOAD_WRITE, &[1]);
        assert_eq!(p.encode(&mut buf), &[0x55, 0x55, 0x01, 0x04, 0x1F, 0x01, !(1u8 + 4 + 31 + 1)]);
    }

    fn answer(id: u8, command: u8, params: &[u8]) -> ([u8; MAX_PACKET_SIZE], usize) {
        let length = params.len() as u8 + 3;
        let mut p = Packet { id, command, length, params: [0; PARAMETER_MTU] };
        p.params[..params.len()].copy_from_slice(params);
        let mut buf = [0u8; MAX_PACKET_SIZE];
        let len = p.encode(&mut buf).len();
        (buf, len)
    }

    #[test]
    fn parses_position_answer() {
        let request = Packet::request(1, cmd::POS_READ, &[]);
        let (raw, len) = answer(1, cmd::POS_READ, &[0x20, 0x03]);
        assert_eq!(len, wire_size(response_length(cmd::POS_READ).unwrap()));
        let rx = parse_response(&request, &raw[..len]).unwrap();
        assert_eq!(rx.param_u16(), 800);
        assert_eq!(rx.params(), &[0x20, 0x03]);
    }

    #[test]
    fn response_checks_in_c_order() {
        let request = Packet::request(1, cmd::POS_READ, &[]);

        let (raw, len) = answer(1, cmd::TEMP_READ, &[40]);
        assert_eq!(parse_response(&request, &raw[..len]), Err(ReadError::BadResponseType));

        let (raw, len) = answer(1, cmd::POS_READ, &[1, 2, 3]);
        assert_eq!(parse_response(&request, &raw[..len]), Err(ReadError::BadResponseLength));

        let (mut raw, len) = answer(1, cmd::POS_READ, &[1, 2]);
        raw[len - 1] ^= 0xFF;
        assert_eq!(parse_response(&request, &raw[..len]), Err(ReadError::BadChecksum));

        let (raw, len) = answer(2, cmd::POS_READ, &[1, 2]);
        assert_eq!(parse_response(&request, &raw[..len]), Err(ReadError::IncorrectResponder));

        assert_eq!(parse_response(&request, &raw[..3]), Err(ReadError::Uart));
        assert_eq!(parse_response(&request, &raw[..len - 1]), Err(ReadError::BadResponseLength));
    }

    #[test]
    fn every_read_command_has_both_lengths() {
        for command in 0..=40u8 {
            if let Some(len) = response_length(command) {
                assert_eq!(request_length(command), Some(3), "command {command}");
                assert!(wire_size(len) <= MAX_PACKET_SIZE);
            }
        }
    }

    #[test]
    fn angle_conversions() {
        assert_eq!(deg_to_position(120.0), 500);
        assert_eq!(deg_to_position(240.0), 1000);
        assert_eq!(deg_to_position(50.0), 208);
        assert_eq!(deg_to_position(-5.0), 0);
        assert_eq!(position_to_deg(500), 120);
        assert_eq!(position_to_deg(1023), 240);
        assert_eq!(position_to_deg(208), 49);
        assert!((position_to_deg_f32(208) - 49.92).abs() < 1e-4);
    }

    #[test]
    fn move_times() {
        // 300 deg/s: 120 deg takes 400 ms
        assert_eq!(move_time_ms(120.0, 0, 300), 400);
        assert_eq!(move_time_ms(0.0, 120, 300), 400);
        assert_eq!(move_time_ms(200.0, 120, 300), 266);
        assert_eq!(move_time_ms(120.0, 120, 300), 0);
    }
}
