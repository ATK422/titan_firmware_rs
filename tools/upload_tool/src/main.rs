//! Flashes RP2040 boards, ported from `titan_firmware/tools/upload_tool`.
//!
//! Finds boards on USB (BOOTSEL mode, or pico-sdk applications with the USB reset interface), on every CAN interface
//! and on the network (CANmore heartbeats), picks the one matching the image's board type (or asks), and writes the
//! image through the bootrom or the CANmore bootloader. Accepts UF2 files and ELF files (such as the Rust firmware
//! built by `cargo build`).

mod flash_ui;

use std::path::PathBuf;
use std::time::Duration;

use anyhow::{Context, Result, bail};
use clap::Parser;
use titan_tools::canmore::Discovery;
use titan_tools::device_map::DeviceMap;
use titan_tools::flash::{self, Device};
use titan_tools::image::FlashImage;
use titan_tools::usb::UsbDiscovery;

#[derive(Parser)]
#[command(version, about, after_help = AFTER_HELP)]
struct Args {
    /// Full image flash. Allows flashing images containing a bootloader rather than restricting to OTA images
    /// (which start after the bootloader).
    #[arg(short = 'f')]
    full_image: bool,

    /// Print the binary info of the image and quit.
    #[arg(short = 'i')]
    info: bool,

    /// Force uploading via OpenOCD (not supported by this port, use probe-rs).
    #[arg(short = 'o')]
    force_openocd: bool,

    /// Always prompt for the device instead of selecting the only matching board automatically.
    #[arg(short = 'p')]
    always_prompt: bool,

    /// Wait for a CANmore device to boot and stop it in the bootloader during its boot delay.
    #[arg(short = 'w')]
    wait_in_boot_delay: bool,

    /// Board type to match devices against, overriding the image's binary info. Rust firmware doesn't embed a
    /// board type, so pass e.g. `--board talos/ivc_board` to keep automatic selection.
    #[arg(short = 'b', long = "board")]
    board: Option<String>,

    /// The UF2 or ELF file to flash.
    image: PathBuf,

    /// The name of a board (from DeviceList.jsonc) to select.
    device_name: Option<String>,
}

const AFTER_HELP: &str = "\
Devices are discovered on USB, on every CAN interface (canN/vcanN) and on UDP port 2201.
USB access needs permission to the RP2040 devices (see titan_firmware/tools/rp2040.rules).

Examples:
  upload_tool build/talos/power_board/power_board_ota.uf2
  upload_tool -b talos/ivc_board firmware/target/thumbv6m-none-eabi/release/ivc_board
  upload_tool -w power_board.uf2              # catch the board in its boot delay";

fn main() {
    let args = Args::parse();
    match run(args) {
        Ok(true) => {}
        Ok(false) => std::process::exit(1),
        Err(e) => {
            eprintln!("\n[ERROR] {e:#}");
            std::process::exit(255);
        }
    }
}

fn run(args: Args) -> Result<bool> {
    let device_map = DeviceMap::get();
    let mut image = FlashImage::load(&args.image)?;
    if let Some(board) = &args.board {
        image.board_type = board.clone();
    }

    if args.info {
        flash_ui::dump_image(&image);
        return Ok(true);
    }

    if args.force_openocd {
        bail!(
            "OpenOCD uploads (-o) are not supported by the Rust upload tool. Flash over SWD with probe-rs instead \
             (`cargo run --release` in firmware/, or `probe-rs download --chip RP2040 <elf>`)"
        );
    }

    let canmore_sources = Discovery::all();
    let mut interface = if args.wait_in_boot_delay {
        flash_ui::catch_in_boot_delay(&canmore_sources, device_map, &image)?
    } else {
        let usb = match UsbDiscovery::new() {
            Ok(usb) => Some(usb),
            Err(e) => {
                eprintln!("[WARNING] USB discovery unavailable: {e:#}");
                None
            }
        };

        println!("Waiting for devices...");
        std::thread::sleep(Duration::from_millis(1000));

        let mut discovered: Vec<Box<dyn Device>> = Vec::new();
        for source in &canmore_sources {
            discovered.extend(flash::Discovery::discover_devices(source));
        }
        if let Some(usb) = &usb {
            discovered.extend(usb.discover());
        }
        if discovered.is_empty() {
            println!("No devices to select");
            return Ok(false);
        }

        let device = match &args.device_name {
            None => flash_ui::select_device(&mut discovered, device_map, &image.board_type, !args.always_prompt),
            Some(name) => {
                let serial = device_map.lookup_serial_by_name(name)?;
                Some(flash_ui::select_device_by_serial(&mut discovered, device_map, serial)?)
            }
        };
        let Some(device) = device else { return Ok(false) };
        discovered[device].flash_interface().context("entering flashing mode")?
    };

    flash_ui::flash_image(interface.as_mut(), &image, !args.full_image)
}
