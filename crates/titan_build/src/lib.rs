//! Build script helper shared by every firmware board. A board's `build.rs` is just:
//!
//! ```ignore
//! fn main() {
//!     titan_build::configure();
//! }
//! ```
//!
//! It provides the RP2040 memory map, passes the embassy/defmt linker scripts, and derives the release type reported
//! in the firmware status, following `titan/version.h`: debug builds are DEBUG, a dirty tree is DEV, a clean tree is
//! CLEAN, a tagged commit is TAGGED. The results are exposed to the board as `TITAN_RELEASE_TYPE` and
//! `TITAN_GIT_DESCRIBE`, which `titan_rp2040::version!()` reads.
//!
//! Like the C applications (`memmap_app.ld`), images are linked behind the 16 KiB Titan bootloader: the
//! application's boot2 at 0x10004000 and its vector table at 0x10004100. The ELF is then an OTA image that
//! `upload_tool` sends over CAN, Ethernet or USB, and the board needs the bootloader in flash. Build with
//! `TITAN_BOOTLOADER=0` to link at the start of flash instead, for a board without the bootloader.

use std::env;
use std::fs;
use std::path::PathBuf;
use std::process::Command;

/// RP2040 with the 2 MiB W25Q080 flash used on every UWRT RP2040 board, behind the bootloader.
const MEMORY_X: &str = "\
MEMORY {
    BOOT2 : ORIGIN = 0x10004000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10004100, LENGTH = 2048K - 16K - 0x100
    RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}
";

/// The same without the bootloader (`TITAN_BOOTLOADER=0`).
const MEMORY_X_NO_BOOTLOADER: &str = "\
MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
    RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}
";

/// The memory map for a `TITAN_BOOTLOADER` setting: behind the bootloader unless it is `0`.
pub fn memory_x(titan_bootloader: Option<&str>) -> &'static str {
    if titan_bootloader == Some("0") { MEMORY_X_NO_BOOTLOADER } else { MEMORY_X }
}

fn git(args: &[&str]) -> Option<String> {
    let out = Command::new("git").args(args).output().ok()?;
    out.status.success().then(|| String::from_utf8_lossy(&out.stdout).trim().to_string())
}

/// Release type number as defined by `enum version_release_type`.
pub fn release_type(debug_profile: bool, dirty: bool, tagged: bool) -> u8 {
    if debug_profile {
        0
    } else if dirty {
        1
    } else if tagged {
        3
    } else {
        2
    }
}

/// Configures linking and version information for a firmware board. Call from `build.rs`.
pub fn configure() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let bootloader = env::var("TITAN_BOOTLOADER").ok();
    fs::write(out.join("memory.x"), memory_x(bootloader.as_deref())).unwrap();
    println!("cargo:rerun-if-env-changed=TITAN_BOOTLOADER");
    println!("cargo:rustc-link-search={}", out.display());

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tlink-rp.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");

    let describe = git(&["describe", "--always", "--dirty", "--tags"]).unwrap_or_else(|| "unknown".into());
    let dirty = git(&["status", "--porcelain"]).is_none_or(|s| !s.is_empty());
    let tagged = git(&["describe", "--tags", "--exact-match"]).is_some();
    let debug = env::var("PROFILE").as_deref() == Ok("debug");
    println!("cargo:rustc-env=TITAN_GIT_DESCRIBE={describe}");
    println!("cargo:rustc-env=TITAN_RELEASE_TYPE={}", release_type(debug, dirty, tagged));
    if let Some(top) = git(&["rev-parse", "--show-toplevel"]) {
        println!("cargo:rerun-if-changed={top}/.git/HEAD");
        println!("cargo:rerun-if-changed={top}/.git/index");
    }
    println!("cargo:rerun-if-changed=build.rs");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn layouts() {
        assert!(memory_x(None).contains("BOOT2 : ORIGIN = 0x10004000"));
        assert!(memory_x(Some("1")).contains("FLASH : ORIGIN = 0x10004100"));
        assert!(memory_x(Some("0")).contains("BOOT2 : ORIGIN = 0x10000000"));
    }

    #[test]
    fn release_types_match_version_h() {
        assert_eq!(release_type(true, false, true), 0);
        assert_eq!(release_type(false, true, true), 1);
        assert_eq!(release_type(false, false, false), 2);
        assert_eq!(release_type(false, false, true), 3);
    }
}
