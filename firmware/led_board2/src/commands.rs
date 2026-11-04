//! CANmore remote commands, ported from `register_canmore_commands` in `ledc_canmore.c`: `leds`, `set_active`,
//! `set_peak_current` and `temp`. The C `ledc` command was commented out and is not ported.
//!
//! Every command needs the controller SPI or the thermistor ADC, both owned by [`ledc::ledc_task`], so they are all
//! deferred: the task picks them up between watchdog feeds and answers through [`DeferredRequest::respond`].
//!
//! The C callbacks of `set_active` and `set_peak_current` returned `void` and the others fell off the end of an
//! `int32_t` function, so their return codes were undefined; here they return 0 on success.

use core::fmt::{self, Write};

use canmore::reg_mapped::interface::debug::REMOTE_CMD_RESP_MAX_LEN;
use ledc_core::l99ld20::{CR1, CR2, Chip, Cr1, Cr2, Response as Frame};
use ledc_core::thermal;
use titan_rp2040::control::{Command, DeferredRequest};

use crate::ledc::{self, Hardware};

/// The board's remote commands, for `BoardConfig::remote_commands`.
pub static COMMANDS: [Command; 4] = [
    Command::deferred("leds", "", "Read and decode status of led controllers.\n"),
    Command::deferred("set_active", "", "Set both controllers into active mode.\n"),
    Command::deferred("set_peak_current", "", "Set both controllers peak current.\n"),
    Command::deferred("temp", "", "Read Temperature on LEDS.\n"),
];

/// Command output, truncated at the response page size like the C `fmemopen` buffer.
struct Output {
    buf: [u8; REMOTE_CMD_RESP_MAX_LEN],
    len: usize,
}

impl Output {
    fn new() -> Self {
        Output { buf: [0; REMOTE_CMD_RESP_MAX_LEN], len: 0 }
    }

    fn as_str(&self) -> &str {
        // Only whole `str`s are copied in
        core::str::from_utf8(&self.buf[..self.len]).unwrap_or("")
    }
}

impl Write for Output {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let room = self.buf.len() - self.len;
        let mut n = s.len().min(room);
        while !s.is_char_boundary(n) {
            n -= 1;
        }
        self.buf[self.len..self.len + n].copy_from_slice(&s.as_bytes()[..n]);
        self.len += n;
        Ok(())
    }
}

/// Runs a deferred command and answers it.
pub async fn run(hw: &mut Hardware, request: DeferredRequest) {
    let mut out = Output::new();
    let rc = match request.command.name {
        "leds" => leds(hw, &mut out),
        // Both rerun the start-up sequence with the read-modify-write fix described in `ledc`: in C, `set_active`
        // also cleared buck 1's duty and both peak currents and `set_peak_current` cleared EN again, leaving the controllers
        // disabled; here each only changes the fields it names, as C intended.
        "set_active" => {
            ledc::set_active_mode(&mut hw.ledc).await;
            0
        }
        "set_peak_current" => {
            ledc::set_peak_current(&mut hw.ledc).await;
            0
        }
        "temp" => temp(hw, &mut out),
        other => {
            let _ = writeln!(out, "Unknown command '{other}'");
            -1
        }
    };
    request.respond(rc, out.as_str());
}

/// `get_ledc_status_cb`.
///
/// C decoded UNLOCK, GOSTBY and EN with `decode_cr1`/`decode_cr2`, which shift each field down and then apply its
/// in-place mask, so it actually tested CR1 bit 2 and CR2 bits 6 and 4 (unrelated duty and current bits). This port
/// reports the real UNLOCK (CR1 bit 1), GOSTBY (CR2 bit 3) and EN (CR2 bit 2). C also read each register up to three
/// times; here each is read once and the CR1 read doubles as the "frame" line (global status byte and data).
fn leds(hw: &mut Hardware, out: &mut Output) -> i32 {
    let cr1: [Frame; 2] = Chip::ALL.map(|chip| defmt::unwrap!(hw.ledc.read(chip, CR1)));
    let cr2: [Frame; 2] = Chip::ALL.map(|chip| defmt::unwrap!(hw.ledc.read(chip, CR2)));

    for (i, r) in cr1.iter().enumerate() {
        let _ = writeln!(out, "controller {} cr1: [{:X}]", i + 1, r.data);
    }
    for (i, r) in cr2.iter().enumerate() {
        let _ = writeln!(out, "controller {} cr2: [{:X}]", i + 1, r.data);
    }
    for (i, r) in cr1.iter().enumerate() {
        let frame = (r.global_status.0 as u32) << 24 | r.data;
        let _ = writeln!(out, "controller {} cr1 frame: [{:X}]", i + 1, frame);
    }
    for i in 0..2 {
        let (cr1, cr2) = (Cr1(cr1[i].data), Cr2(cr2[i].data));
        let _ = writeln!(out, "CONTROLLER {} STATUS", i + 1);
        let _ = out.write_str(if cr1.unlock() { "Unlocked - UNLOCK: 1 \n" } else { "Locked - UNLOCK: 0 \n" });
        let _ = out.write_str(if cr2.go_standby() {
            "In Standby - GOSTBY: 1 \n"
        } else {
            "Woke Up, Not in Standby  -  GOSTBY: 0 \n"
        });
        let _ = out.write_str(if cr2.enable() { "Enabled - EN: 1 \n" } else { "Not Enabled / Limp Home -  EN: 0 \n" });
    }
    0
}

/// `read_temp_cb`. C had no ADC error path; a failed read is reported with return code 1.
fn temp(hw: &mut Hardware, out: &mut Output) -> i32 {
    match hw.adc.blocking_read(&mut hw.thermistor) {
        Ok(raw) => {
            let _ = writeln!(out, "Aluminum Board Temperature: [{:.4}]", thermal::thermistor_celsius(raw));
            0
        }
        Err(_) => {
            let _ = writeln!(out, "Thermistor read failed");
            1
        }
    }
}
