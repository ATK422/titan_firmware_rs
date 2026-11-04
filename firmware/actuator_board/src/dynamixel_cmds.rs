//! The Dynamixel driver's CANmore remote commands (`dynamixel_canmore_cmds.c`, registered by `dynamixel_init`):
//! `dynwrite`, `dynping`, `dynreboot`, `dynram`, `dyneeprom` and `dynstatus`.
//!
//! `dynram`, `dyneeprom` and `dynstatus` only read the tracked state, so they run inside the request handler. The
//! others send a packet and wait for the reply, so they are deferred: [`run_deferred`] carries them out from the
//! board's remote command task, through [`dynamixel::Servos::raw_request`].

use core::fmt::Write;
use core::num::IntErrorKind;

use dynamixel::{Error, ErrorKind, RawError, RawReply, RawRequest, Source};
use embassy_time::Duration;
use titan_rp2040::control::{Args, Command, Response};

use crate::v2::SERVOS;

/// How long a deferred command waits for the driver to run its packet. C waited 60 ms
/// (`DYNAMIXEL_CANMORE_RX_TIMEOUT_MS`) because it blocked the CANmore handler; here the request may first wait for
/// the driver to finish a polling step, so allow longer while staying under the 1 s deferred command limit.
const RX_TIMEOUT: Duration = Duration::from_millis(500);

/// The commands, in C registration order.
pub const fn commands() -> [Command; 6] {
    [
        Command::deferred(
            "dynwrite",
            "[id] [addr] [reg type] [value]",
            "Writes the data to the requested address\nThe reg type field can be one of [u8/s8/u16/s16/u32/s32]\n\tThe \
             u/s stands for unsigned/sign (if the variable accepts negative numbers)\n\tThe 8/16/32 determines the \
             number of bits that the integer takes up (multiply the size in bytes by 8)\nRefer to the Servo Register \
             Map for more information:\n\thttps://emanual.robotis.com/docs/en/dxl/x/xl430-w250/\nNote: All numbers \
             are in decimal\nExample: dynwrite 4 7 u8 5\n\tWrites ID register (address 7, 1 byte unsigned) of \
             dynamixel 4, changing its ID to 5",
        ),
        Command::deferred("dynping", "[id]", "Attempts to ping the requested dynamixel"),
        Command::deferred("dynreboot", "[id]", "Reboots the requested dynamixel"),
        Command::new(
            "dynram",
            "[id]",
            "Dumps the current RAM state of the requested dynamixel\nNote this can only read dynamixels that are \
             registered for monitoring the application",
            dynram,
        ),
        Command::new(
            "dyneeprom",
            "[id]",
            "Dumps the EEPROM of the requested dynamixel\nNote this can only read dynamixels that are registered for \
             monitoring the application",
            dyneeprom,
        ),
        Command::new("dynstatus", "", "Lists the IDs of all monitored dynamixel servos.", dynstatus),
    ]
}

// ========================================
// Parsing and formatting helpers
// ========================================

/// `strtoll(str, &end, 10)` with the whole string consumed. Out of range values saturate like strtoll.
pub fn parse_decimal(s: &str) -> Option<i64> {
    match s.parse::<i64>() {
        Ok(v) => Some(v),
        Err(e) => match e.kind() {
            IntErrorKind::PosOverflow => Some(i64::MAX),
            IntErrorKind::NegOverflow => Some(i64::MIN),
            _ => None,
        },
    }
}

/// `parse_int_with_bounds`: prints the C error and gives `Err(1)` on failure.
fn parse_bounded(out: &mut impl Write, s: &str, min: i64, max: i64) -> Result<i64, i32> {
    let Some(value) = parse_decimal(s) else {
        let _ = writeln!(out, "Invalid Decimal Value Specified: '{s}'");
        return Err(1);
    };
    if value > max || value < min {
        let _ = writeln!(out, "Provided number '{s}' out of bounds! Must be between {min} - {max}");
        return Err(1);
    }
    Ok(value)
}

fn parse_id(out: &mut impl Write, s: &str) -> Result<u8, i32> {
    parse_bounded(out, s, 1, 0xFE).map(|v| v as u8)
}

fn yes_no(value: bool) -> &'static str {
    if value { "Yes" } else { "No" }
}

fn lookup<'a>(table: &[&'a str], index: usize) -> &'a str {
    table.get(index).copied().unwrap_or("Unknown")
}

/// `value * factor / 1000` printed like `%.3f`; exact, as the C factors (0.113, 0.229) have three decimals.
struct Milli(i64);

impl core::fmt::Display for Milli {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let sign = if self.0 < 0 { "-" } else { "" };
        let abs = self.0.unsigned_abs();
        write!(f, "{sign}{}.{:03}", abs / 1000, abs % 1000)
    }
}

fn pwm_percent(raw: i64) -> Milli {
    Milli(raw * 113)
}

fn rpm(raw: i64) -> Milli {
    Milli(raw * 229)
}

// ========================================
// Error reporting
// ========================================

static ASYNC_UART_ERRORS: [&str; 7] = ["OK", "BUSY", "TIMEOUT", "INCOMPLETE", "FRAME_ERROR", "DATA_LOST", "ABORTED"];

static DXL_LIB_ERRORS: [&str; 21] = [
    "OK",
    "PROCEEDING",
    "ERROR_NOT_SUPPORTED",
    "ERROR_TIMEOUT",
    "ERROR_INVAILD_ID",
    "ERROR_NOT_SUPPORT_BROADCAST",
    "ERROR_NULLPTR",
    "ERROR_LENGTH",
    "ERROR_INVAILD_ADDR",
    "ERROR_ADDR_LENGTH",
    "ERROR_BUFFER_OVERFLOW",
    "ERROR_PORT_NOT_OPEN",
    "ERROR_WRONG_PACKET",
    "ERROR_CHECK_SUM",
    "ERROR_CRC",
    "ERROR_INVAILD_DATA_LENGTH",
    "ERROR_MEMORY_ALLOCATION_FAIL",
    "ERROR_INVAILD_PROTOCOL_VERSION",
    "ERROR_NOT_INITIALIZED",
    "ERROR_NOT_ENOUGH_BUFFER_SIZE",
    "ERROR_PORT_WRITE",
];

static RESPONSE_ERRORS: [&str; 8] = [
    "OK",
    "Result Fail",
    "Instruction Error",
    "CRC Error",
    "Data Range Error",
    "Data Length Error",
    "Data Limit Error",
    "Access Error",
];

/// `decode_dynamixel_error`. The Rust driver doesn't track source lines, so the source is printed with line 0.
fn decode_error(out: &mut impl Write, error: Error) {
    let wrapped = error.wrapped;
    let mut uses_wrapped = false;
    let _ = out.write_str(" - Error:\t");
    let lib_error = |out: &mut dyn Write, uses_wrapped: &mut bool| {
        if wrapped != 0 {
            let _ = writeln!(out, " - DXL Lib Err:\t{}", lookup(&DXL_LIB_ERRORS, wrapped as usize));
            *uses_wrapped = true;
        }
    };
    match error.kind {
        ErrorKind::Driver => {
            let _ = writeln!(out, "Internal Driver Error");
            lib_error(out, &mut uses_wrapped);
        }
        ErrorKind::Ttl => {
            let _ = writeln!(out, "Low-level Async UART Error");
            let _ = writeln!(out, " - Wrapped Err:\t{}", lookup(&ASYNC_UART_ERRORS, wrapped as usize));
            uses_wrapped = true;
        }
        ErrorKind::Request => {
            let _ = writeln!(out, "Request Create");
            lib_error(out, &mut uses_wrapped);
        }
        ErrorKind::Packet => {
            let _ = writeln!(out, "Response Packet Parse Error");
            lib_error(out, &mut uses_wrapped);
        }
        ErrorKind::Response => {
            let _ = writeln!(out, "Error Set in Packet Response");
            let _ = writeln!(out, " - Resp Err:\t{}", lookup(&RESPONSE_ERRORS, wrapped as usize));
            uses_wrapped = true;
        }
        ErrorKind::CommandQueueFull => {
            let _ = writeln!(out, "Command Queue Full");
        }
        ErrorKind::InvalidId => {
            let _ = writeln!(out, "Packet received not from intended target");
            let _ = writeln!(out, " - ID Found:\t{wrapped}");
            uses_wrapped = true;
        }
    }
    if !uses_wrapped && wrapped != 0 {
        let _ = writeln!(out, " - Wrapped Err:\t{wrapped}");
    }
    let source = match error.source {
        Source::Schedule => "dynamixel_schedule.c",
        Source::Comms => "dynamixel_comms.c",
    };
    let _ = writeln!(out, " - Source:\t{source}:0");
}

/// `queue_and_wait_for_resp` plus the reply length check of the packet callback. Prints the C errors on failure.
async fn send_and_wait(out: &mut impl Write, id: u8, request: RawRequest, expected_params: usize) -> Option<RawReply> {
    match SERVOS.raw_request(id, request, RX_TIMEOUT).await {
        Ok(reply) if reply.param_len == expected_params => Some(reply),
        Ok(_) => {
            let message = if expected_params == 0 {
                "Received params in response, when none were expected"
            } else {
                "Invalid Packet Length Received (Expected 3 bytes)"
            };
            let _ = write!(out, "Error while processing response:\n\t{message}\n");
            None
        }
        Err(RawError::Outstanding) => {
            let _ = writeln!(
                out,
                "Error: There is already an outstanding packet sent from CANmore that previously timed out"
            );
            let _ = writeln!(out, "If it has been over a second since the last request, it is likely a driver bug");
            None
        }
        Err(RawError::QueueFull) => {
            let _ = writeln!(out, "Failed to schedule write packet - Transmit Queue Full");
            None
        }
        Err(RawError::Timeout) => {
            let _ = writeln!(out, "Error: Did not receive response from driver within timeout");
            None
        }
        Err(RawError::Driver(error)) => {
            let _ = writeln!(out, "Error Received from Driver: 0x{:08X}", error.code());
            decode_error(out, error);
            None
        }
    }
}

// ========================================
// Deferred commands (send a packet)
// ========================================

/// Runs a deferred Dynamixel command, if `args` names one, writing its output to `out`.
pub async fn run_deferred(args: &Args<'_>, out: &mut impl Write) -> Option<i32> {
    let rc = match args.get(0)? {
        "dynwrite" => dynwrite(args, out).await,
        "dynping" => dynping(args, out).await,
        "dynreboot" => dynreboot(args, out).await,
        _ => return None,
    };
    Some(rc.unwrap_or_else(|rc| rc))
}

async fn dynwrite(args: &Args<'_>, out: &mut impl Write) -> Result<i32, i32> {
    let (5, Some(id), Some(address), Some(mode), Some(value)) =
        (args.len(), args.get(1), args.get(2), args.get(3), args.get(4))
    else {
        let _ = writeln!(out, "Usage: dynwrite [id] [addr] [u8/s8/u16/s16/u32/s32/bytes] [data...]");
        return Err(1);
    };
    let id = parse_id(out, id)?;
    let address = parse_bounded(out, address, 0, u16::MAX as i64)? as u16;

    let (min, max, len) = match mode {
        "u8" => (0, u8::MAX as i64, 1),
        "s8" => (i8::MIN as i64, i8::MAX as i64, 1),
        "u16" => (0, u16::MAX as i64, 2),
        "s16" => (i16::MIN as i64, i16::MAX as i64, 2),
        "u32" => (0, u32::MAX as i64, 4),
        "s32" => (i32::MIN as i64, i32::MAX as i64, 4),
        other => {
            let _ = writeln!(out, "Invalid Write Mode: '{other}'");
            return Err(1);
        }
    };
    let value = parse_bounded(out, value, min, max)?;
    // Little endian, like the C union of the integer types; 4 bytes always fit
    let request = RawRequest::write(address, &value.to_le_bytes()[..len]).ok_or(1)?;

    send_and_wait(out, id, request, 0).await.ok_or(1)?;
    let _ = writeln!(out, "Wrote {len} bytes starting at {address} to dynamixel {id}");
    Ok(0)
}

async fn dynping(args: &Args<'_>, out: &mut impl Write) -> Result<i32, i32> {
    let (2, Some(id)) = (args.len(), args.get(1)) else {
        let _ = writeln!(out, "Usage: dynping [id]");
        return Err(1);
    };
    let id = parse_id(out, id)?;
    let reply = send_and_wait(out, id, RawRequest::Ping, 3).await.ok_or(1)?;
    let model = u16::from_le_bytes([reply.params[0], reply.params[1]]);
    let firmware = reply.params[2];
    let _ = writeln!(out, "Ping Received from Device {id} (Model: {model}; Firmware: v{firmware})");
    Ok(0)
}

async fn dynreboot(args: &Args<'_>, out: &mut impl Write) -> Result<i32, i32> {
    let (2, Some(id)) = (args.len(), args.get(1)) else {
        let _ = writeln!(out, "Usage: dynreboot [id]");
        return Err(1);
    };
    let id = parse_id(out, id)?;
    send_and_wait(out, id, RawRequest::Reboot, 0).await.ok_or(1)?;
    let _ = writeln!(out, "Reboot Successful");
    Ok(0)
}

// ========================================
// Synchronous commands (tracked state)
// ========================================

fn parse_id_arg(name: &str, args: &Args<'_>, out: &mut Response<'_>) -> Result<u8, i32> {
    let (2, Some(id)) = (args.len(), args.get(1)) else {
        let _ = writeln!(out, "Usage: {name} [id]");
        return Err(1);
    };
    parse_id(out, id)
}

static VELOCITY_PROFILES: [&str; 4] = ["Not Used", "Rectangular", "Triangular", "Trapezoidal"];

/// The hardware error status bits `dynram` lists.
static HARDWARE_ERROR_BITS: [&str; 8] = [
    "    - Bit 0: Input Voltage Error",
    "    - Bit 1: Unknown???",
    "    - Bit 2: Overheating Error",
    "    - Bit 3: Motor Encoder Error",
    "    - Bit 4: Electrical Shock Error",
    "    - Bit 5: Overload Error",
    "    - Bit 6: Unknown??",
    "    - Bit 7: Unknown??",
];

fn dynram(args: &Args<'_>, out: &mut Response<'_>) -> i32 {
    let id = match parse_id_arg("dynram", args, out) {
        Ok(id) => id,
        Err(rc) => return rc,
    };
    let Some(ram) = SERVOS.ram(id) else {
        let _ = writeln!(out, "Dynamixel ID {id} is not monitored by firmware or is not currently connected");
        return 1;
    };

    let status = ram.moving_status;
    let _ = writeln!(out, "--- Servo {id} RAM ---");
    let _ = writeln!(out, "  Torque Enable (64):\t\t{}", yes_no(ram.torque_enable));
    let _ = writeln!(out, "  Moving (122):\t\t\t{}", yes_no(ram.moving));
    let _ = writeln!(out, "  Moving Status (123):\t\t{status}");
    let _ = writeln!(out, "   - Bit 0: In-Position:\t{}", yes_no(status & (1 << 0) != 0));
    let _ = writeln!(out, "   - Bit 1: Profile Ongoing:\t{}", yes_no(status & (1 << 1) != 0));
    let _ = writeln!(out, "   - Bit 3: Following Error:\t{}", yes_no(status & (1 << 3) != 0));
    let _ = writeln!(
        out,
        "   - Bit 4-5: Velocity Profile:\t{}",
        lookup(&VELOCITY_PROFILES, ((status >> 4) & 0x3) as usize)
    );
    let _ = writeln!(out, "  Realtime Tick (120):\t\t{} ms", ram.realtime_tick);

    let hw = ram.hardware_error_status;
    let _ = writeln!(out, "  Hardware Error Status (70):\t{hw}");
    for (bit, line) in HARDWARE_ERROR_BITS.iter().enumerate() {
        if hw & (1 << bit) != 0 {
            let _ = writeln!(out, "{line}");
        }
    }

    let _ = writeln!(out, "  Goal PWM (100):\t\t{}% ({})", pwm_percent(ram.goal_pwm as i64), ram.goal_pwm);
    let _ = writeln!(out, "  Goal Velocity (104):\t\t{} rpm ({})", rpm(ram.goal_velocity as i64), ram.goal_velocity);
    let _ = writeln!(out, "  Goal Position (116):\t\t{}", ram.goal_position);
    let _ = writeln!(out, "  Present PWM (124):\t\t{}% ({})", pwm_percent(ram.present_pwm as i64), ram.present_pwm);
    // Same truncating division as C, so negative loads print like it did
    let _ = writeln!(out, "  Present Load (126):\t\t{}.{}%", ram.present_load / 10, ram.present_load % 10);
    let _ =
        writeln!(out, "  Present Velocity (128):\t{} rpm ({})", rpm(ram.present_velocity as i64), ram.present_velocity);
    let _ = writeln!(out, "  Present Position (132):\t{}", ram.present_position);
    let voltage = ram.present_input_voltage;
    let _ = writeln!(out, "  Present Voltage (144):\t{}.{} V", voltage / 10, voltage % 10);
    let _ = writeln!(out, "  Present Temperature (146):\t{} C", ram.present_temperature);
    let _ = writeln!(out, "-------------------");
    0
}

static BAUD_RATES: [&str; 8] = ["9,600b", "57,600b", "115,200b", "1Mb", "2Mb", "3Mb", "4Mb", "4.5Mb"];

fn operating_mode(mode: u8) -> &'static str {
    match mode {
        1 => "Velocity Control",
        3 => "Position Control",
        4 => "Extended Position Control",
        16 => "PWM Control",
        _ => "Unknown",
    }
}

fn dyneeprom(args: &Args<'_>, out: &mut Response<'_>) -> i32 {
    let id = match parse_id_arg("dyneeprom", args, out) {
        Ok(id) => id,
        Err(rc) => return rc,
    };
    let Some(eeprom) = SERVOS.eeprom(id) else {
        // C's wording, missing "not" included
        let _ = writeln!(out, "Dynamixel ID {id} is monitored by firmware or is not currently connected");
        return 1;
    };

    let drive_mode = eeprom.drive_mode;
    let _ = writeln!(out, "--- Servo {id} EEPROM ---");
    let _ = writeln!(out, "  Model Number (0):\t\t{}", eeprom.model_num);
    let _ = writeln!(out, "  Model Info (2):\t\t{}", eeprom.model_info);
    let _ = writeln!(out, "  Firmware Version (6):\t\tv{}", eeprom.firmware_version);
    let _ = writeln!(out, "  ID (7):\t\t\t{}", eeprom.id);
    let _ =
        writeln!(out, "  Baud Rate (8):\t\t{} ({})", lookup(&BAUD_RATES, eeprom.baud_rate as usize), eeprom.baud_rate);
    let _ = writeln!(out, "  Return Delay Time (9):\t{} us", eeprom.return_delay_time);
    let _ = writeln!(out, "  Drive Mode (10):\t\t{drive_mode}");
    let direction = if drive_mode & (1 << 0) != 0 { "Reverse (CW+, CCW-)" } else { "Normal (CCW+, CW-)" };
    let _ = writeln!(out, "   - Bit 0: Servo Direction:\t{direction}");
    let profile = if drive_mode & (1 << 2) != 0 { "Time-based" } else { "Velocity-based" };
    let _ = writeln!(out, "   - Bit 2: Profile Config:\t{profile}");
    let _ = writeln!(out, "   - Bit 3: Torque Auto-Enable:\t{}", yes_no(drive_mode & (1 << 3) != 0));
    let _ = writeln!(
        out,
        "  Operating Mode (11):\t\t{} ({})",
        operating_mode(eeprom.operating_mode),
        eeprom.operating_mode
    );
    let _ = writeln!(out, "  Secondary ID (12):\t\t{}", eeprom.secondary_id);
    let _ = writeln!(out, "  Protocol (13):\t\tv{}", eeprom.protocol_type);
    let _ = writeln!(out, "  Homing Offset (20):\t\t{}", eeprom.homing_offset);
    let _ = writeln!(
        out,
        "  Moving Threshold (24):\t{} rpm ({})",
        rpm(eeprom.moving_threshold as i64),
        eeprom.moving_threshold
    );
    let _ = writeln!(out, "  Temperature Limit (31):\t{} C", eeprom.temperature_limit);
    let (min_v, max_v) = (eeprom.min_voltage_limit, eeprom.max_voltage_limit);
    let _ = writeln!(out, "  Min Voltage Limit (32):\t{}.{} V", min_v / 10, min_v % 10);
    let _ = writeln!(out, "  Max Voltage Limit (34):\t{}.{} V", max_v / 10, max_v % 10);
    let _ = writeln!(out, "  PWM Limit (36):\t\t{}% ({})", pwm_percent(eeprom.pwm_limit as i64), eeprom.pwm_limit);
    let _ = writeln!(
        out,
        "  Velocity Limit (44):\t\t{} rpm ({})",
        rpm(eeprom.velocity_limit as i64),
        eeprom.velocity_limit
    );
    // C's labels (the max limit is really at 52)
    let _ = writeln!(out, "  Min Position Limit (48):\t{}", eeprom.min_position_limit);
    let _ = writeln!(out, "  Max Position Limit (49):\t{}", eeprom.max_position_limit);
    let startup = eeprom.startup_config;
    let _ = writeln!(out, "  Startup Config (60):\t\t{startup}");
    let _ = writeln!(out, "   - Bit 0: Startup Torque On:\t{}", yes_no(startup & (1 << 0) != 0));
    let _ = writeln!(out, "   - Bit 1: RAM Restore:\t{}", yes_no(startup & (1 << 1) != 0));
    let _ = writeln!(out, "-----------------------");
    0
}

fn dynstatus(_args: &Args<'_>, out: &mut Response<'_>) -> i32 {
    let states = SERVOS.link_states();
    if states.is_empty() {
        let _ = writeln!(out, "No Dynamixels Configured");
    }
    for state in states {
        let text = if !state.first_connect_attempted {
            "Not Polled Yet..."
        } else if !state.connected {
            "Not Connected"
        } else if !state.alert_notified {
            "Connected"
        } else {
            "Connected, Alert Present"
        };
        let _ = writeln!(out, "Dynamixel {}: {text}", state.id);
    }
    0
}
