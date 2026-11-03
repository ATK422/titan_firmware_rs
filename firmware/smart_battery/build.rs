use std::env;
use std::fs;
use std::path::PathBuf;

/// The standard map minus the last 4 KiB flash sector, which holds the board serial number (`id_gen/id.uf2`).
/// Keeping it out of FLASH means the firmware image can never be placed over it.
const MEMORY_X: &str = "\
MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100 - 4K
    RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}
";

fn main() {
    titan_build::configure();
    // Replace the memory map titan_build just wrote to the same place
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::write(out.join("memory.x"), MEMORY_X).unwrap();
}
