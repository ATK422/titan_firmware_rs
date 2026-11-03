//! `smart_battery_id`: writes the UF2 image that programs a smart battery board's serial number, which is also its
//! CANmore client ID and must match the serial number of the BQ40Z80 in its pack.
//!
//! Replaces `talos/SmartBattery/id_gen/id.uf2`, which had to be hex edited. Needed after fully erasing the flash or
//! on a new SBH_MCU board.
//!
//! ```text
//! smart_battery_id <serial 1-254> [output.uf2]
//! picotool load -v output.uf2      (or copy it onto the BOOTSEL drive)
//! ```

use std::process::ExitCode;

use smart_battery_core::uf2;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (id, path) = match args.as_slice() {
        [id] => (id, format!("smart_battery_id_{id}.uf2")),
        [id, path] => (id, path.clone()),
        _ => {
            eprintln!("usage: smart_battery_id <serial 1-254> [output.uf2]");
            return ExitCode::FAILURE;
        }
    };
    let id = match id.parse::<u8>() {
        Ok(id) if uf2::valid_id(id) => id,
        _ => {
            eprintln!("serial must be 1 to 254 (0 and 255 read as unprogrammed)");
            return ExitCode::FAILURE;
        }
    };
    if let Err(err) = std::fs::write(&path, uf2::id_block(id)) {
        eprintln!("writing {path}: {err}");
        return ExitCode::FAILURE;
    }
    println!("Wrote {path}: serial {id} at {:#010X}", uf2::ID_ADDRESS);
    ExitCode::SUCCESS
}
