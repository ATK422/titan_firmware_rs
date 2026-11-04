//! The servo CANmore remote commands from `actuator.c` (`servo_init_internal`): `hwsetid`, `hwfind` and `hwping`,
//! for programming and checking servo IDs with `canmore_cli`.
//!
//! They need the servo bus, so they are deferred: [`crate::actuators::uart_task`] takes them between its scheduled
//! packets and runs them with [`run`]. In C the callbacks queued a packet for the bus scheduler; here the bus owner
//! sends it straight away, which keeps it in order with the other packets the same way.
//!
//! Differences from C:
//! * `hwfind` and `hwping` answer as soon as the servo replies or the 50 ms receive timeout passes, instead of
//!   always sleeping 100 ms.
//! * `hwsetid` answers after the packet is out, not when it is queued. Like C it doesn't check that the servo took
//!   the new ID (the ID write isn't answered), and a transmit failure is only logged.
//! * The servos' ping state (`connected`) isn't touched by `hwping`, as C pinged a temporary servo.

use core::fmt::Write;

use canmore::reg_mapped::interface::debug::REMOTE_CMD_RESP_MAX_LEN;
use embassy_time::Duration;
use hbridge_core::hiwonder::{self, ID_BROADCAST, ID_MAX, ID_MIN, MAX_PACKET_SIZE, Packet, cmd};
use titan_rp2040::control::{Args, Command, DeferredRequest, Response, parse_int};

use crate::multidrop::MultidropUart;

/// The board's remote commands, for `BoardConfig::remote_commands`.
pub static COMMANDS: [Command; 3] = [
    Command::deferred("hwsetid", "[old_id] [new_id]", "Sets a servo of ID [old_id] to the desired ID [1, 252]"),
    Command::deferred(
        "hwfind",
        "",
        "Discover the ID of a servo on the bus. Only one servo can be connected for this command to function.",
    ),
    Command::deferred("hwping", "[id]", "Check if a servo with [id] is connected"),
];

/// Runs a deferred remote command on the servo bus and answers it.
pub async fn run(request: DeferredRequest, uart: &mut MultidropUart<'_>) {
    let mut buf = [0u8; REMOTE_CMD_RESP_MAX_LEN];
    let mut out = Response::new(&mut buf);
    let args = request.args();
    let rc = match request.command.name {
        "hwsetid" => set_id(&args, &mut out, uart).await,
        "hwfind" => discover(&mut out, uart).await,
        "hwping" => ping(&args, &mut out, uart).await,
        other => {
            let _ = write!(out, "Unknown Remote Command: '{other}'");
            -1
        }
    };
    request.respond(rc, out.as_str());
}

/// An ID argument; garbage reads as 0 (out of range), as C ignored `parse_int` failing.
fn id_arg(args: &Args<'_>, i: usize) -> i64 {
    args.get(i).and_then(parse_int).unwrap_or(0)
}

fn valid_id(id: i64) -> bool {
    (ID_MIN as i64..=ID_MAX as i64).contains(&id)
}

/// Sends `packet` and, if the servo answers it, reads and checks the answer.
async fn transfer(uart: &mut MultidropUart<'_>, packet: &Packet) -> Option<Result<Packet, hiwonder::ReadError>> {
    let mut tx = [0u8; MAX_PACKET_SIZE];
    if let Err(err) = uart.write(packet.encode(&mut tx)).await {
        defmt::error!("Servo UART transmit failed: {}", err);
        return Some(Err(hiwonder::ReadError::Uart));
    }
    let len = packet.response_wire_len()?.min(MAX_PACKET_SIZE);
    let mut rx = [0u8; MAX_PACKET_SIZE];
    let rx = &mut rx[..len];
    if let Err(err) = uart.read(rx, Duration::from_millis(hiwonder::RESPONSE_TIMEOUT_MS)).await {
        defmt::debug!("Servo answer to command {} not received: {}", packet.command, err);
        return Some(Err(hiwonder::ReadError::Uart));
    }
    Some(match hiwonder::parse_response(packet, rx) {
        Ok(answer) => Ok(answer),
        // The broadcast ID read is answered by the servo's own ID
        Err((hiwonder::ReadError::IncorrectResponder, Some(answer))) if packet.id == ID_BROADCAST => Ok(answer),
        Err((err, _)) => Err(err),
    })
}

/// `debug_set_id_cb`
async fn set_id(args: &Args<'_>, out: &mut Response<'_>, uart: &mut MultidropUart<'_>) -> i32 {
    if args.len() != 3 {
        let _ = write!(out, "Incorrect number of arguments ({})! Needs old and new ID", args.len().saturating_sub(1));
        return 1;
    }
    let (old_id, new_id) = (id_arg(args, 1), id_arg(args, 2));
    if !valid_id(old_id) || !valid_id(new_id) {
        let _ = write!(out, "ID is invalid! It must be between {ID_MIN} and {ID_MAX}");
        return 1;
    }

    // servo_set_id; both IDs are in the u8 range checked above
    defmt::info!("Setting servo ID {} to {}", old_id, new_id);
    let _ = transfer(uart, &Packet::command(old_id as u8, cmd::ID_WRITE, &[new_id as u8])).await;
    0
}

/// `debug_discover_servo_cb`
async fn discover(out: &mut Response<'_>, uart: &mut MultidropUart<'_>) -> i32 {
    // servo_read_id: no retry, the C retry was commented out on this board
    let discovered_id = match transfer(uart, &Packet::command(ID_BROADCAST, cmd::ID_READ, &[])).await {
        Some(Ok(answer)) => answer.params[0],
        Some(Err(err)) => {
            defmt::debug!("Servo ID read refused: {}", err);
            0
        }
        None => 0,
    };

    if discovered_id == 0 {
        let _ = write!(out, "Servo not found");
        return 1;
    }
    let _ = write!(out, "Discovered servo id {discovered_id}");
    0
}

/// `debug_ping_servo`
async fn ping(args: &Args<'_>, out: &mut Response<'_>, uart: &mut MultidropUart<'_>) -> i32 {
    if args.len() != 2 {
        let _ =
            write!(out, "Incorrect number of arguments ({})! Needs an ID to search for", args.len().saturating_sub(1));
        return 1;
    }
    let id = id_arg(args, 1);
    if !valid_id(id) {
        let _ = write!(out, "ID is invalid! It must be between {ID_MIN} and {ID_MAX}");
        return 1;
    }

    // servo_ping: a temperature read, connected only on a clean answer
    let connected = matches!(transfer(uart, &Packet::command(id as u8, cmd::TEMP_READ, &[])).await, Some(Ok(_)));
    if connected {
        let _ = write!(out, "Servo with ID {id} found");
    } else {
        let _ = write!(out, "Servo not found");
    }
    0
}
