//! CANmore CLI, ported from `titan_firmware/tools/canmore_cli`.
//!
//! Discovers CANmore devices from their heartbeats on every CAN interface and on the network, and opens a debug
//! CLI matching the device's mode: application (normal mode), bootloader, or Linux machine.
//!
//! ```text
//! canmore_cli                                  interactive device selection
//! canmore_cli <device name> <command> [args]   run one command on a board from DeviceList.jsonc
//! ```

mod app_cli;
mod bl_cli;
mod camera;
mod cli;
mod gdbserver;
mod linux_cli;
mod select;

use std::time::Duration;

use anyhow::{Result, bail};
use titan_tools::canmore::{CanmoreDevice, DeviceMode, Discovery};
use titan_tools::device_map::DeviceMap;
use titan_tools::ui::{COLOR_BODY, COLOR_ERROR, COLOR_HEADER, COLOR_NAME, COLOR_RESET, render_header};

/// A CLI of any device type.
trait Runnable {
    fn run(&mut self) -> Result<()>;
    fn run_once(&mut self, cmd: &str, args: &[String]);
}

impl<T: Send + 'static> Runnable for cli::Cli<T> {
    fn run(&mut self) -> Result<()> {
        cli::Cli::run(self)
    }
    fn run_once(&mut self, cmd: &str, args: &[String]) {
        cli::Cli::run_once(self, cmd, args)
    }
}

fn create_cli(device: &CanmoreDevice) -> Result<Box<dyn Runnable>> {
    Ok(match device.mode {
        DeviceMode::Bootloader => Box::new(bl_cli::create(device.bootloader_client()?)?),
        DeviceMode::Normal => Box::new(app_cli::create(device.debug_client()?)?),
        DeviceMode::BootDelay => {
            println!("Waiting for 15 seconds for boot delay device to reappear...");
            Box::new(bl_cli::create(device.wait_for_boot_delay_and_enter_bootloader(Duration::from_secs(15))?)?)
        }
        DeviceMode::Linux => Box::new(linux_cli::create(device.linux_client()?)?),
        mode => bail!("Failed to start CLI!\nNo compatible CLI backends for device mode '{}'", mode.name()),
    })
}

fn report_error(msg: &str) {
    println!("\n{COLOR_ERROR}{msg}{COLOR_RESET}");
    let _ = titan_tools::ui::read_line(&format!("{COLOR_HEADER}Press enter to continue...{COLOR_RESET}"));
    println!();
}

fn print_usage(prog: &str) {
    render_header("Usage");
    println!("{COLOR_NAME}{prog}{COLOR_RESET}");
    println!("\tRun CANmore CLI in interactive mode");
    println!();
    println!("{COLOR_NAME}{prog}{COLOR_BODY} [Target Device] [Command ...]{COLOR_RESET}");
    println!("\tRun the requested CLI command on the specified target");
    println!(
        "\tSetting the environment variable {COLOR_BODY}CANMORECLI_QUIET=1{COLOR_RESET} will silence connect messages"
    );
    println!();
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let prog = args.first().map(String::as_str).unwrap_or("canmore_cli");
    if args.len() == 2 {
        print_usage(prog);
        std::process::exit(1);
    }
    if let Err(e) = run(&args) {
        eprintln!("\n[ERROR] {e:#}");
        std::process::exit(1);
    }
}

fn run(args: &[String]) -> Result<()> {
    let sources = Discovery::all();
    if sources.is_empty() {
        bail!("No discovery sources available (no CAN interfaces, and UDP port 2201 could not be bound)");
    }

    if args.len() < 3 {
        let mut cache = select::FlashIdCache::default();
        while let Some(device) = select::select_device(&sources, &mut cache) {
            match create_cli(&device).and_then(|mut cli| cli.run()) {
                Ok(()) => {}
                Err(e) => report_error(&format!("Exception while executing CLI!\n  what(): {e:#}")),
            }
        }
        return Ok(());
    }

    let target = &args[1];
    let serial = DeviceMap::get().lookup_serial_by_name(target)?;
    let device = select::wait_for_target(&sources, serial);
    let mut cli = create_cli(&device)?;
    cli.run_once(&args[2], &args[3..]);
    Ok(())
}
