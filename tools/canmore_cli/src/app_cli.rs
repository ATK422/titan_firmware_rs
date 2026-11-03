//! CLI for applications in normal mode, ported from `canmore_cli/src/application_cli.cpp`.

use std::fmt::Write as _;
use std::time::Duration;

use anyhow::{Result, bail};
use canmore::reg_mapped::interface::debug::REMOTE_CMD_RESP_MAX_LEN;
use titan_tools::canmore::DebugClient;
use titan_tools::canmore::debug::FaultData;
use titan_tools::canmore::gdb::GdbTarget;
use titan_tools::device_map::DeviceMap;
use titan_tools::ui::{
    CLEAR_LINE_AFTER, COLOR_BODY, COLOR_ERROR, COLOR_HEADER, COLOR_NAME, COLOR_NOTICE, COLOR_RESET, COLOR_TITLE,
    decode_u32, header, hex_dump, render_field, render_header, render_name,
};

use crate::cli::{Cli, Command, PrefixHandler, Session, usage, write_multiline};

pub fn create(client: DebugClient) -> Result<Cli<DebugClient>> {
    let mut cli = Cli::new("Application", client);
    cli.register(Command {
        name: "uptime",
        args: "",
        help: "Returns the current uptime.",
        run: |s, _| {
            render_field("Uptime", &s.handle().uptime()?.format());
            Ok(())
        },
    });
    cli.register(Command {
        name: "version",
        args: "",
        help: "Returns the current application version.",
        run: |s, _| {
            render_field("App Version", &s.handle().version()?);
            Ok(())
        },
    });
    cli.register(Command {
        name: "reboot",
        args: "",
        help: "Reboots the device.",
        run: |s, _| {
            println!("Sending reboot command...");
            s.handle().reboot()?;
            std::thread::sleep(Duration::from_millis(1000));
            s.close("Closed due to Reboot Request");
            Ok(())
        },
    });
    cli.register(Command {
        name: "enterbl",
        args: "",
        help: "Reboots the device into bootloader mode.",
        run: |s, _| {
            println!("Requesting reboot into bootloader mode...");
            s.handle().enter_bootloader()?;
            std::thread::sleep(Duration::from_millis(1000));
            s.close("Closed due to Reboot Request");
            Ok(())
        },
    });
    cli.register(Command {
        name: "crashlog",
        args: "",
        help: "Returns the full device crash log.",
        run: |s, _| {
            let mut handle = s.handle();
            let counter = handle.crash_counter()?;
            let log = handle.crash_log()?;
            render_header("Crash Log");
            render_name(&counter.format());
            for (i, entry) in log.iter().enumerate() {
                render_field(&format!("Reset -{}", i + 1), &entry.format());
            }
            Ok(())
        },
    });
    cli.register(Command {
        name: "safetystatus",
        args: "",
        help: "Returns the current safety status.",
        run: |s, _| {
            let status = s.handle().safety_status()?;
            let yes_no = |b: bool| if b { "Yes" } else { "No" };
            render_field("Safety Setup", yes_no(status.safety_setup));
            render_field("Safety Inited", yes_no(status.safety_initialized));
            render_field("Kill State", if status.kill_is_enabled { "Enabled" } else { "Killed" });
            let error =
                if status.fault_present { format!("{COLOR_ERROR}Fault Present{COLOR_RESET}") } else { "Normal".into() };
            render_field("Error State", &error);
            Ok(())
        },
    });
    cli.register(Command {
        name: "fault",
        args: "[action] [fault num (for raise/lower)]",
        help: "Interfaces with the Titan Safety Fault System\nValid Actions:\n  list: Lists active faults (Default)\n  \
               listall: Lists all faults, including inactive ones\n  raise: Raises the specified fault number\n  \
               lower: Lowers the specified fault number",
        run: fault_command,
    });
    cli.register(Command {
        name: "mem",
        args: "[action] [...]",
        help: "Performs various low-level actions to device memory:\nActions:\n  read [address]\n    Reads the word at \
               the specified address\n  write [address] [data]\n    Writes the provided data word to the specified \
               word-aligned address\n  readstr [address] [optional max len]\n    Reads the null-terminated string at \
               the specified address\n    The max len determines how far to search for the null termination \
               (Defaults to 256)\n  dump [address] [length]\n    Shows a hex dump starting at the provided address \
               (must be word-aligned)",
        run: mem_command,
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
    cli.add_prefix(Box::new(RemoteCmdPrefix { help: None }));
    cli.set_background(keepalive_task());

    if !cli.quiet() {
        print_banner(&mut cli.handle())?;
    }
    Ok(cli)
}

fn print_banner(client: &mut DebugClient) -> Result<()> {
    let flash_id = client.flash_id();
    let descr = DeviceMap::get().lookup_serial(flash_id);
    println!();
    render_header("Connecting to Application");
    render_name(&descr.name);
    if !descr.is_unknown {
        render_field("Board Type", &descr.board_type);
    } else if flash_id != 0 {
        render_field("Unique ID", &descr.hex_serial());
    }
    render_field("Version", &client.version()?);
    render_field("Reset Cause", &client.last_reset_entry()?.format());
    render_field("Uptime", &client.uptime()?.format());
    let crashes = client.crash_counter()?;
    if crashes.total > 0 {
        render_field("Crash Count", &crashes.format());
    }
    let status = client.safety_status()?;
    render_field("System State", status.global_state());
    let error = if status.fault_present { format!("{COLOR_ERROR}Fault Present") } else { "Normal".into() };
    render_field("Error State", &error);
    println!();
    Ok(())
}

/// Reports safety state changes and raised/lowered faults while the prompt is idle.
fn keepalive_task() -> crate::cli::BackgroundTask<DebugClient> {
    let mut last: Option<(bool, bool, u32)> = None;
    Box::new(move |client: &mut DebugClient| {
        let status = client.safety_status()?;
        let faults = client.active_faults()?;
        let mut events = Vec::new();
        if let Some((initialized, kill_enabled, last_faults)) = last {
            if initialized != status.safety_initialized {
                events.push(if status.safety_initialized {
                    "Safety Initialized (ROS Connected)".to_owned()
                } else {
                    "Safety Deinitialized (ROS Disconnected)".to_owned()
                });
            }
            if kill_enabled != status.kill_is_enabled {
                events.push(if status.kill_is_enabled { "Kill Switch Inserted" } else { "Kill Switch Removed" }.into());
            }
            let changed = faults ^ last_faults;
            for i in (0..32).filter(|i| changed & (1 << i) != 0) {
                let state = if faults & (1 << i) != 0 { "Raised" } else { "Lowered" };
                events.push(format!("Fault {} ({i}) {state}", client.fault_name(i)?));
            }
        }
        last = Some((status.safety_initialized, status.kill_is_enabled, faults));
        if events.is_empty() {
            return Ok(Vec::new());
        }
        let uptime = client.uptime()?.format_seconds();
        Ok(events.into_iter().map(|e| format!("{COLOR_NOTICE}[{uptime}] {e}{COLOR_RESET}")).collect())
    })
}

fn fault_command(s: &mut Session<DebugClient>, args: &[String]) -> Result<()> {
    if args.len() > 2 {
        return usage("Invalid Syntax!");
    }
    let action = args.first().map(String::as_str).unwrap_or("list");
    match action {
        "raise" | "lower" => {
            let Some(id_arg) = args.get(1) else { return usage("Invalid Syntax: Fault number argument required") };
            let Some(fault_id) = decode_u32(id_arg, 31) else {
                println!("Invalid fault id specified, expected integer <32.");
                println!("Use 'listall' action to view ID to name mapping");
                return Ok(());
            };
            let mut handle = s.handle();
            let name = handle.fault_name(fault_id)?;
            if action == "raise" {
                println!("Raising fault {name}");
                handle.raise_fault(fault_id)
            } else {
                println!("Lowering fault {name}");
                handle.lower_fault(fault_id)
            }
        }
        "list" | "listall" => {
            if args.len() > 1 {
                return usage("Invalid Syntax!");
            }
            let list_all = action == "listall";
            let mut handle = s.handle();
            let active = handle.active_faults()?;
            let faults = (0..32)
                .filter(|i| list_all || active & (1 << i) != 0)
                .map(|i| handle.fault_data(i))
                .collect::<Result<Vec<_>>>()?;
            if faults.is_empty() {
                println!("{COLOR_HEADER}    No faults{COLOR_RESET}{CLEAR_LINE_AFTER}");
                return Ok(());
            }
            let (header_row, rows) = render_fault_table(&faults, list_all);
            println!("{}", header(if list_all { "All Faults" } else { "Active Faults" }, visible_len(&header_row)));
            println!("{header_row}");
            for row in rows {
                println!("{row}");
            }
            Ok(())
        }
        other => {
            println!("Invalid Action: {other}");
            Ok(())
        }
    }
}

/// Length of a string without its ANSI escape sequences.
fn visible_len(s: &str) -> usize {
    let mut len = 0;
    let mut in_escape = false;
    for c in s.chars() {
        match (in_escape, c) {
            (false, '\x1b') => in_escape = true,
            (false, _) => len += 1,
            (true, c) if c.is_ascii_alphabetic() => in_escape = false,
            (true, _) => {}
        }
    }
    len
}

fn render_fault_table(faults: &[FaultData], show_sticky: bool) -> (String, Vec<String>) {
    let name_w = faults.iter().map(|f| f.name.len()).max().unwrap_or(0).max(4);
    let file_w = faults.iter().map(|f| f.filename.len()).max().unwrap_or(0).max(4);
    let ts_w = faults.iter().map(|f| f.format_timestamp().len()).max().unwrap_or(0).max(9);
    let (id_w, sticky_w, multiple_w, line_w, extra_w) = (2, 6, 8, 6, 10);

    let mut header = format!("{COLOR_HEADER} | {:<id_w$} | {:<name_w$}", "Id", "Name");
    if show_sticky {
        let _ = write!(header, " | {:<sticky_w$}", "Sticky");
    }
    let _ = write!(
        header,
        " | {:<multiple_w$} | {:<ts_w$} | {:<file_w$} | {:<line_w$} | {:<extra_w$} |{COLOR_RESET}{CLEAR_LINE_AFTER}",
        "Multiple", "Timestamp", "File", "Line", "Extra Data"
    );

    let yes_no = |b: bool| if b { "Yes" } else { "No" };
    let rows = faults
        .iter()
        .map(|f| {
            let mut row = format!("{COLOR_BODY} | {:>id_w$} | {:>name_w$}", f.fault_id, f.name);
            if show_sticky {
                let _ = write!(row, " | {:>sticky_w$}", yes_no(f.sticky));
            }
            let (ts, file, line, extra) = if f.sticky {
                let extra = if f.extra_data != 0 {
                    format!("0x{:0w$X}", f.extra_data, w = extra_w - 2)
                } else {
                    format!("{:>extra_w$}", "0")
                };
                (f.format_timestamp(), f.filename.clone(), f.line.to_string(), extra)
            } else {
                ("-".into(), "-".into(), "-".into(), format!("{:>extra_w$}", "-"))
            };
            let _ = write!(
                row,
                " | {:>multiple_w$} | {ts:>ts_w$} | {file:>file_w$} | {line:>line_w$} | {extra} |{COLOR_RESET}{CLEAR_LINE_AFTER}",
                yes_no(f.multiple_fires)
            );
            row
        })
        .collect();
    (header, rows)
}

fn mem_command(s: &mut Session<DebugClient>, args: &[String]) -> Result<()> {
    if !(2..=3).contains(&args.len()) {
        return usage("Invalid Syntax: Expected 2-3 arguments");
    }
    let action = args[0].as_str();
    if !matches!(action, "read" | "write" | "readstr" | "dump") {
        return usage(format!("Invalid Action: {action}"));
    }
    let Some(address) = decode_u32(&args[1], u32::MAX) else {
        println!("Invalid 32-bit integer provided for address");
        return Ok(());
    };
    if address % 4 != 0 && action != "readstr" {
        println!("Invalid address: must be 32-bit aligned");
        return Ok(());
    }
    let third = match args.get(2) {
        Some(arg) => match decode_u32(arg, u32::MAX) {
            Some(v) => Some(v),
            None => {
                println!("Invalid integer provided in third argument (must be either hex or decimal)");
                return Ok(());
            }
        },
        None => None,
    };

    let mut handle = s.handle();
    match action {
        "write" => {
            let Some(data) = third else { return usage("Invalid Syntax: Write requires data argument to be provided") };
            handle.write_memory(address, data)
        }
        "readstr" => {
            let max_len = third.unwrap_or(256);
            let mut out = Vec::new();
            let mut cached: Option<(u32, u32)> = None;
            for i in 0..max_len {
                let addr = address.wrapping_add(i);
                let word_addr = addr & !3;
                let word = match cached {
                    Some((a, w)) if a == word_addr => w,
                    _ => {
                        let w = handle.read_memory(word_addr)?;
                        cached = Some((word_addr, w));
                        w
                    }
                };
                let byte = (word >> ((addr % 4) * 8)) as u8;
                if byte == 0 {
                    println!(
                        "{COLOR_HEADER}String @0x{address:08x}: \"{COLOR_RESET}{}{COLOR_HEADER}\"{COLOR_RESET}",
                        String::from_utf8_lossy(&out)
                    );
                    return Ok(());
                }
                out.push(byte);
            }
            println!("{COLOR_HEADER}String @0x{address:08x}: \"{COLOR_RESET}{}", String::from_utf8_lossy(&out));
            println!("{COLOR_NOTICE}[WARN] Reached maximum length of {max_len}{COLOR_RESET}");
            Ok(())
        }
        "dump" => {
            let Some(length) = third else {
                return usage("Invalid Syntax: Hex dump requires length argument to be provided");
            };
            if length % 4 != 0 || length == 0 {
                println!("Invalid Length: Must be divisible by word size");
                return Ok(());
            }
            render_header("Memory Hex Dump");
            let mut data = Vec::with_capacity(length as usize);
            for offset in (0..length).step_by(4) {
                data.extend_from_slice(&handle.read_memory(address.wrapping_add(offset))?.to_le_bytes());
            }
            print!("{}", hex_dump(address, &data));
            Ok(())
        }
        _ => {
            if third.is_some() {
                return usage("Invalid Syntax: Expected 2 arguments");
            }
            println!("0x{:08X}", handle.read_memory(address)?);
            Ok(())
        }
    }
}

/// `@command args...` runs a command registered in the firmware.
struct RemoteCmdPrefix {
    help: Option<String>,
}

impl RemoteCmdPrefix {
    fn fetch_help(client: &mut DebugClient) -> Result<String> {
        let (rc, response) = client.execute_remote_cmd(&["help", "-n"])?;
        if rc != 0 {
            bail!("Non-zero status code when retrieving command count");
        }
        let Some(count) = decode_u32(response.trim(), u32::MAX) else {
            bail!("Invalid integer response from command count: {response}");
        };
        let mut help = String::new();
        if count > 0 {
            let _ = writeln!(help, "\n{COLOR_TITLE}====Remote Commands (Prefixed with @)===={COLOR_RESET}");
        }
        for i in 0..count {
            let (rc, response) = client.execute_remote_cmd(&["help", "-i", &i.to_string()])?;
            if rc != 0 {
                bail!("Non-zero status code when retrieving remote command help");
            }
            let mut lines = response.lines();
            let name = lines.next().ok_or_else(|| anyhow::anyhow!("Malformed response - no command name in help"))?;
            let usage = lines.next().ok_or_else(|| anyhow::anyhow!("Malformed response - no command usage in help"))?;
            let _ = writeln!(help, "{COLOR_NAME}@{name} {COLOR_BODY}{usage}{COLOR_RESET}");
            let rest: Vec<&str> = lines.collect();
            if rest.is_empty() {
                let _ = writeln!(help, "\t<No Help Message Provided>");
            }
            for line in rest {
                if line.is_empty() {
                    help.push('\n');
                } else {
                    let _ = writeln!(help, "\t{line}");
                }
            }
        }
        Ok(help)
    }
}

impl PrefixHandler<DebugClient> for RemoteCmdPrefix {
    fn prefix(&self) -> char {
        '@'
    }

    fn run(&mut self, s: &mut Session<DebugClient>, args: &[String]) -> Result<()> {
        if args.is_empty() {
            println!("Cannot send empty remote command");
            return Ok(());
        }
        let (rc, response) = s.handle().execute_remote_cmd(args)?;
        write_multiline(&response, "");
        if response.len() == REMOTE_CMD_RESP_MAX_LEN - 1 {
            println!("{COLOR_NOTICE}... Notice: Output may be truncated{COLOR_RESET}");
        }
        if rc != 0 {
            println!("{COLOR_NOTICE}Client responded with non-zero status code: {rc}{COLOR_RESET}");
        }
        Ok(())
    }

    fn help(&mut self, s: &mut Session<DebugClient>, usage_only: bool) {
        if usage_only {
            println!("Usage: @[Remote Command] [Remote Command Args]");
            return;
        }
        if self.help.is_none() {
            match Self::fetch_help(&mut s.handle()) {
                Ok(help) => self.help = Some(help),
                Err(e) => {
                    println!("{COLOR_ERROR}Exception caught while fetching remote help:{COLOR_RESET}");
                    println!("{COLOR_ERROR}  what(): {e:#}{COLOR_RESET}");
                    return;
                }
            }
        }
        print!("{}", self.help.as_deref().unwrap_or_default());
    }
}
