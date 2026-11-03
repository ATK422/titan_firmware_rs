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

use std::env;
use std::fs;
use std::path::PathBuf;
use std::process::Command;

/// RP2040 with the 2 MiB W25Q080 flash used on every UWRT RP2040 board.
const MEMORY_X: &str = "\
MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
    RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}
";

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
    fs::write(out.join("memory.x"), MEMORY_X).unwrap();
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
    fn release_types_match_version_h() {
        assert_eq!(release_type(true, false, true), 0);
        assert_eq!(release_type(false, true, true), 1);
        assert_eq!(release_type(false, false, false), 2);
        assert_eq!(release_type(false, false, true), 3);
    }
}
