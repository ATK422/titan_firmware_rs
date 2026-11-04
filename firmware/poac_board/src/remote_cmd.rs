//! The servo's CANmore remote commands from `actuator.c` (`init_servo`): `hwsetid` and `hwfind`, for programming
//! the servo ID with `canmore_cli`.
//!
//! They need the servo bus, so they are deferred: [`crate::actuators::servo_task`] takes them between its scheduled
//! packets and runs them with [`run`]. In C the callbacks queued a packet for the bus scheduler; here the bus owner
//! sends it straight away, which keeps it in order with the other packets the same way.
//!
//! Differences from C:
//! * `hwfind` answers as soon as the servo replies instead of always sleeping 100 ms. It retries a failed read while
//!   the 100 ms window lasts, where C retried until the shared servo error counter reached `MAX_NUM_ERRORS`; in
//!   practice both get two tries in that window with the 50 ms receive timeout.
//! * `hwsetid` answers after the packet is out, not when it is queued. Like C it doesn't check that the servo took
//!   the new ID (the ID write isn't answered).

use core::fmt::Write;

use canmore::reg_mapped::interface::debug::REMOTE_CMD_RESP_MAX_LEN;
use embassy_time::{Duration, Instant};
use power_core::hiwonder::{self, BROADCAST_ID, MAX_PACKET_SIZE, Packet, ReadError, cmd};
use titan_rp2040::control::{Args, Command, DeferredRequest, Response, parse_int};

use crate::uart::MultidropUart;

/// The board's remote commands, for `BoardConfig::remote_commands`.
pub static COMMANDS: [Command; 2] = [
    Command::deferred("hwsetid", "[old_id] [new_id]", "Sets a servo of ID [old_id] to the desired ID [1, 252]"),
    Command::deferred(
        "hwfind",
        "",
        "Discover the ID of a servo on the bus. Only one servo can be connected for this command to function.",
    ),
];

/// IDs `hwsetid` accepts.
const ID_MIN: i64 = 1;
const ID_MAX: i64 = 252;
/// How long `hwfind` waits for the servo to answer (the C `packet_wait_time`).
const FIND_WINDOW: Duration = Duration::from_millis(100);

/// Runs a deferred remote command on the servo bus and answers it.
pub async fn run(request: DeferredRequest, uart: &mut MultidropUart<'_>) {
    let mut buf = [0u8; REMOTE_CMD_RESP_MAX_LEN];
    let mut out = Response::new(&mut buf);
    let args = request.args();
    let rc = match request.command.name {
        "hwsetid" => set_id(&args, &mut out, uart).await,
        "hwfind" => discover(&mut out, uart).await,
        other => {
            let _ = write!(out, "Unknown Remote Command: '{other}'");
            -1
        }
    };
    request.respond(rc, out.as_str());
}

/// `parse_int_with_bounds`: the value, or the C error message and `None`.
fn parse_with_bounds(arg: &str, min: i64, max: i64, out: &mut Response<'_>) -> Option<u8> {
    let Some(value) = parse_int(arg) else {
        let _ = writeln!(out, "Invalid Decimal Value Specified: '{arg}'");
        return None;
    };
    if value > max || value < min {
        let _ = writeln!(out, "Provided number '{arg}' out of bounds! Must be between {min} - {max}");
        return None;
    }
    // Bounded to the servo ID range above
    Some(value as u8)
}

/// `debug_set_id_cb`
async fn set_id(args: &Args<'_>, out: &mut Response<'_>, uart: &mut MultidropUart<'_>) -> i32 {
    if args.len() != 3 {
        // C printed argc, which counts the command name
        let _ = write!(out, "Incorrect number of arguments ({})! Needs old and new ID", args.len());
        return 1;
    }
    let Some(old_id) = parse_with_bounds(args.get(1).unwrap_or(""), ID_MIN, ID_MAX, out) else { return 1 };
    let Some(new_id) = parse_with_bounds(args.get(2).unwrap_or(""), ID_MIN, ID_MAX, out) else { return 1 };

    // servo_set_id
    defmt::info!("Setting servo ID {} to {}", old_id, new_id);
    let mut tx = [0u8; MAX_PACKET_SIZE];
    uart.write(Packet::request(old_id, cmd::ID_WRITE, &[new_id]).encode(&mut tx)).await;
    0
}

/// One `servo_read_id`: the ID the servo on the bus reports.
async fn read_id(uart: &mut MultidropUart<'_>) -> Result<u8, ReadError> {
    let request = Packet::request(BROADCAST_ID, cmd::ID_READ, &[]);
    let mut tx = [0u8; MAX_PACKET_SIZE];
    uart.write(request.encode(&mut tx)).await;

    let length = hiwonder::response_length(cmd::ID_READ).unwrap_or(4);
    let mut rx = [0u8; MAX_PACKET_SIZE];
    let rx = &mut rx[..hiwonder::wire_size(length).min(MAX_PACKET_SIZE)];
    if let Err(err) = uart.read(rx).await {
        defmt::debug!("Servo ID read failed: {}", err);
        return Err(ReadError::Uart);
    }
    // The servo answers the broadcast address with its own ID: C took the answer despite SERVO_INCORRECT_RESPONDER,
    // so check it as if sent to the responder
    let responder = Packet { id: rx[2], ..request };
    hiwonder::parse_response(&responder, rx).map(|packet| packet.params[0])
}

/// `debug_discover_servo_cb`
async fn discover(out: &mut Response<'_>, uart: &mut MultidropUart<'_>) -> i32 {
    let deadline = Instant::now() + FIND_WINDOW;
    let mut discovered_id = 0;
    while Instant::now() < deadline {
        match read_id(uart).await {
            Ok(id) => {
                discovered_id = id;
                break;
            }
            Err(err) => defmt::debug!("Servo ID read refused: {}", err),
        }
    }

    if discovered_id == 0 {
        let _ = write!(out, "Servo not found");
        return 1;
    }
    let _ = write!(out, "Discovered servo id {discovered_id}");
    0
}
