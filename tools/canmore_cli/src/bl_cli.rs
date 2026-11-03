//! CLI for devices in bootloader mode, ported from `canmore_cli/src/bootloader_cli.cpp`.

use std::time::Duration;

use anyhow::Result;
use titan_tools::canmore::BootloaderClient;
use titan_tools::canmore::discovery::bootloader_app_info;
use titan_tools::canmore::gdb::GdbTarget;
use titan_tools::device_map::DeviceMap;
use titan_tools::flash::{FLASH_BASE, FlashInterface, MAX_FLASH_SIZE, PAGE_SIZE};
use titan_tools::ui::{decode_u32, format_app_info, hex_dump, render_field, render_header, render_name};

use crate::cli::{Cli, Command, usage};

pub fn create(client: BootloaderClient) -> Result<Cli<BootloaderClient>> {
    let mut cli = Cli::new("Bootloader", client);
    cli.register(Command {
        name: "version",
        args: "",
        help: "Returns the current bootloader version.",
        run: |s, _| {
            render_field("BL Version", &s.handle().version()?);
            Ok(())
        },
    });
    cli.register(Command {
        name: "reboot",
        args: "",
        help: "Reboots the device into application",
        run: |s, _| {
            println!("Sending reboot command...");
            s.handle().reboot()?;
            std::thread::sleep(Duration::from_millis(1000));
            s.close("Closed due to Reboot Request");
            Ok(())
        },
    });
    cli.register(Command {
        name: "binary_info",
        args: "",
        help: "Returns the current binary info for the application.",
        run: |s, _| {
            let info = bootloader_app_info(&mut s.handle());
            render_header("Application Info");
            print!("{}", format_app_info(&info));
            Ok(())
        },
    });
    cli.register(Command {
        name: "read",
        args: "[address]",
        help: "Reads a page at the specified address\nAddress must be a flash address in hex",
        run: |s, args| {
            let [address] = args else { return usage("Expected 1 argument") };
            let Some(address) = decode_u32(address, u32::MAX) else {
                println!("Invalid 32-bit integer provided for address");
                return Ok(());
            };
            if !(FLASH_BASE..FLASH_BASE + MAX_FLASH_SIZE).contains(&address) || address % PAGE_SIZE as u32 != 0 {
                println!("Invalid address outside of flash memory");
                return Ok(());
            }
            render_header("Reading Flash Page");
            let page = s.handle().read_page(address)?;
            print!("{}", hex_dump(address, &page));
            Ok(())
        },
    });
    cli.register(Command {
        name: "gdbserver",
        args: "[port: Default 3333]",
        help: "Starts gdb server over Canmore",
        run: |s, args| {
            let port = crate::gdbserver::parse_port(args)?;
            crate::gdbserver::run(port, &mut *s.handle());
            Ok(())
        },
    });
    cli.set_background(Box::new(|client: &mut BootloaderClient| client.ping().map(|_| Vec::new())));

    if !cli.quiet() {
        let mut client = cli.handle();
        for warning in &client.warnings {
            println!("{warning}");
        }
        let flash_id = client.flash_id();
        let descr = DeviceMap::get().lookup_serial(flash_id);
        println!();
        render_header("Connecting to Bootloader");
        render_name(&descr.name);
        if !descr.is_unknown {
            render_field("Board Type", &descr.board_type);
        } else if flash_id != 0 {
            render_field("Unique ID", &descr.hex_serial());
        }
        render_field("Flash Size", &format!("{:.1} MB", client.flash_size() as f64 / 1024.0 / 1024.0));
        render_field("Version", &client.version()?);
        println!();
    }
    Ok(cli)
}
