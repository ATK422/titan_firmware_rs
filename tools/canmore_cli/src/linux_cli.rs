//! CLI for Linux machines on the bus, ported from `canmore_cli/src/linux_cli.cpp` and `remote_tty_client_task.cpp`.

use std::io::Write;
use std::os::fd::AsRawFd;
use std::os::unix::fs::PermissionsExt;
use std::time::Duration;

use anyhow::{Context, Result, bail};
use titan_tools::canmore::remote_tty::{RemoteTtyClient, TtyEvent};
use titan_tools::canmore::{Link, LinuxClient};
use titan_tools::device_map::DeviceMap;
use titan_tools::ui::{
    COLOR_ERROR, COLOR_NAME, COLOR_NOTICE, COLOR_RESET, decode_u32, render_field, render_header, render_name,
};

use crate::cli::{Cli, Command, PrefixHandler, Session, usage};

pub fn create(client: LinuxClient) -> Result<Cli<LinuxClient>> {
    let mut cli = Cli::new("Linux", client);
    cli.register(Command {
        name: "version",
        args: "",
        help: "Returns the current version.",
        run: |s, _| {
            render_field("Version", &s.handle().version()?);
            Ok(())
        },
    });
    cli.register(Command {
        name: "reboot",
        args: "",
        help: "Performs a full system reboot",
        run: |s, _| {
            println!("Sending reboot command...");
            s.handle().reboot()?;
            std::thread::sleep(Duration::from_millis(1000));
            s.close("Closed due to Reboot Request");
            Ok(())
        },
    });
    cli.register(Command {
        name: "daemon_restart",
        args: "",
        help: "Restarts the daemon service (if bash manages to break)",
        run: |s, _| {
            println!("Resetting CanmoreCLI Daemon...");
            s.handle().restart_daemon()?;
            std::thread::sleep(Duration::from_millis(2000));
            s.close("Closed due to daemon stopping");
            Ok(())
        },
    });
    cli.register(Command {
        name: "upload",
        args: "[src_file] [optional dst_file]",
        help: "Uploads a file to the remote client.",
        run: upload_command,
    });
    cli.register(Command {
        name: "download",
        args: "[src_file] [optional dst_file]",
        help: "Downloads a file from the remote client.",
        run: download_command,
    });
    cli.register(Command {
        name: "cd",
        args: "[directory]",
        help: "Changes working directory on the remote device",
        run: |s, args| {
            if args.len() > 1 {
                return usage(format!("Expected 0-1 arguments but got {}", args.len()));
            }
            s.handle().cd(args.first().map(String::as_str).unwrap_or(""))
        },
    });
    cli.register(Command {
        name: "ls",
        args: "[optional directory]",
        help: "Lists the contents of a directory on the remote system",
        run: |s, args| {
            if args.len() > 1 {
                return usage(format!("Expected 0-1 arguments but got {}", args.len()));
            }
            print!("{}", s.handle().ls(args.first().map(String::as_str).unwrap_or(""))?);
            Ok(())
        },
    });
    cli.register(Command {
        name: "pwd",
        args: "",
        help: "Reports the current working directory of the remote device",
        run: |s, _| {
            println!("{}", s.handle().pwd()?);
            Ok(())
        },
    });
    cli.register(Command {
        name: "remotesh",
        args: "[optional command]",
        help: "Launches a remote shell on the host. If the optional command is provided, that is executed instead of the \
               login shell. The command is ran from the user's home directory",
        run: |s, args| run_remote_tty(&mut s.handle(), &args.join(" "), false),
    });
    cli.register(Command {
        name: "camera_server",
        args: "[optional port (defaults to 3005)]",
        help: "Launches a camera server to receive images over CANmore",
        run: |s, args| {
            let port = match args.first() {
                None => crate::camera::DEFAULT_PORT,
                Some(arg) => match decode_u32(arg, u16::MAX as u32 - 1) {
                    Some(port) => port as u16,
                    None => return usage("Invalid port number"),
                },
            };
            let (interface, client_id) = can_link(&s.handle())?;
            crate::camera::run(&interface, client_id, port)
        },
    });
    cli.add_prefix(Box::new(LocalShellPrefix));
    cli.add_prefix(Box::new(RemoteShellPrefix));
    cli.set_background(Box::new(|client: &mut LinuxClient| client.ping().map(|_| Vec::new())));

    if !cli.quiet() {
        let mut client = cli.handle();
        let flash_id = client.flash_id();
        let descr = DeviceMap::get().lookup_serial(flash_id);
        println!();
        render_header("Connecting to Linux Device");
        render_name(&descr.name);
        if !descr.is_unknown {
            render_field("Board Type", &descr.board_type);
        } else if flash_id != 0 {
            render_field("Unique ID", &descr.hex_serial());
        }
        render_field("Version", &client.version()?);
        println!();
    }
    Ok(cli)
}

/// The CAN interface and client ID of the device: the remote TTY and camera feed only exist on CAN.
fn can_link(client: &LinuxClient) -> Result<(String, u8)> {
    match client.link() {
        Some((Link::Can(interface), client_id)) => Ok((interface.clone(), client_id)),
        _ => bail!("Current interface is not CAN bus, this feature is only available over CAN"),
    }
}

fn file_name(path: &str) -> &str {
    path.rsplit('/').next().unwrap_or(path)
}

fn progress_bar(fraction: f64) {
    const LENGTH: usize = 32;
    let filled = ((fraction.clamp(0.0, 1.0)) * LENGTH as f64) as usize;
    let bar: String = (0..LENGTH)
        .map(|i| {
            if i + 1 == filled {
                '>'
            } else if i < filled {
                '='
            } else {
                ' '
            }
        })
        .collect();
    print!("\r\x1b[K[{bar}] {}%", (fraction * 100.0) as u32);
    let _ = std::io::stdout().flush();
}

fn upload_command(s: &mut Session<LinuxClient>, args: &[String]) -> Result<()> {
    if !(1..=2).contains(&args.len()) {
        println!("{COLOR_ERROR}Expected 1-2 arguments but got {}{COLOR_RESET}", args.len());
        return Ok(());
    }
    let src = &args[0];
    let dst = args.get(1).map(String::as_str).unwrap_or_else(|| file_name(src));
    let data = std::fs::read(src).with_context(|| format!("Failed to open file {src}"))?;
    let mode =
        std::fs::metadata(src).map(|m| m.permissions().mode()).with_context(|| format!("Failed to stat {src}"))?;
    println!("Uploading file");
    let result = s.handle().upload(&data, dst, mode, &mut progress_bar);
    println!();
    result?;
    println!("Upload complete.");
    Ok(())
}

fn download_command(s: &mut Session<LinuxClient>, args: &[String]) -> Result<()> {
    if !(1..=2).contains(&args.len()) {
        println!("{COLOR_ERROR}Expected 1-2 arguments but got {}{COLOR_RESET}", args.len());
        return Ok(());
    }
    let src = &args[0];
    let dst = args.get(1).map(String::as_str).unwrap_or_else(|| file_name(src));
    println!("Downloading file");
    let result = s.handle().download(src, &mut progress_bar);
    println!();
    let (data, mode) = result?;
    std::fs::write(dst, &data).with_context(|| format!("Error writing file on local system: {dst}"))?;
    std::fs::set_permissions(dst, std::fs::Permissions::from_mode(mode & 0o7777))
        .context("Failed to set local file permissions")?;
    println!("Download complete.");
    Ok(())
}

/// `!cmd` runs a command in the local shell, `!` alone starts the shell.
struct LocalShellPrefix;

impl PrefixHandler<LinuxClient> for LocalShellPrefix {
    fn prefix(&self) -> char {
        '!'
    }

    fn run(&mut self, _: &mut Session<LinuxClient>, args: &[String]) -> Result<()> {
        let cmd = if args.is_empty() {
            match std::env::var("SHELL") {
                Ok(shell) => shell,
                Err(_) => {
                    println!("Cannot launch system shell!\nSHELL environment variable not set");
                    return Ok(());
                }
            }
        } else {
            args.join(" ")
        };
        let status = std::process::Command::new("sh").arg("-c").arg(&cmd).status()?;
        if !status.success() {
            println!(
                "{COLOR_NOTICE}Command exited with non-zero status code: {}{COLOR_RESET}",
                status.code().unwrap_or(-1)
            );
        }
        Ok(())
    }

    fn help(&mut self, _: &mut Session<LinuxClient>, usage_only: bool) {
        if usage_only {
            println!("Usage: ![Command to Execute]");
            return;
        }
        println!("{COLOR_NAME}!command{COLOR_RESET}\n\tExecute 'command' in the local shell");
        println!("{COLOR_NAME}!{COLOR_RESET}\n\tEscape to local shell");
    }
}

/// `@cmd` runs a command in a remote shell in the file transfer directory, `@` alone opens the shell there.
struct RemoteShellPrefix;

impl PrefixHandler<LinuxClient> for RemoteShellPrefix {
    fn prefix(&self) -> char {
        '@'
    }

    fn run(&mut self, s: &mut Session<LinuxClient>, args: &[String]) -> Result<()> {
        run_remote_tty(&mut s.handle(), &args.join(" "), true)
    }

    fn help(&mut self, _: &mut Session<LinuxClient>, usage_only: bool) {
        if usage_only {
            println!("Usage: @[Command to Execute]");
            return;
        }
        println!(
            "{COLOR_NAME}@command{COLOR_RESET}\n\tExecute 'command' in the remote shell from the current working directory"
        );
        println!("{COLOR_NAME}@{COLOR_RESET}\n\tSpawn a remote shell in the current working directory");
    }
}

/// Puts the terminal in raw mode until dropped.
struct RawTerminal(libc::termios);

impl RawTerminal {
    fn enable() -> Result<Self> {
        // SAFETY: termios is plain data, filled by tcgetattr before use
        let mut old: libc::termios = unsafe { std::mem::zeroed() };
        // SAFETY: valid fd and pointer to a termios
        if unsafe { libc::tcgetattr(libc::STDIN_FILENO, &mut old) } < 0 {
            return Err(std::io::Error::last_os_error()).context("tcgetattr");
        }
        let mut raw = old;
        // SAFETY: valid pointer to a termios
        unsafe { libc::cfmakeraw(&mut raw) };
        // SAFETY: valid fd and pointer to a termios
        if unsafe { libc::tcsetattr(libc::STDIN_FILENO, libc::TCSANOW, &raw) } < 0 {
            return Err(std::io::Error::last_os_error()).context("tcsetattr");
        }
        print!("\x1b[0m");
        let _ = std::io::stdout().flush();
        Ok(RawTerminal(old))
    }
}

impl Drop for RawTerminal {
    fn drop(&mut self) {
        print!("\x1b[0m");
        let _ = std::io::stdout().flush();
        // SAFETY: restoring the attributes read in enable()
        unsafe { libc::tcsetattr(libc::STDIN_FILENO, libc::TCSANOW, &self.0) };
    }
}

fn window_size() -> Option<(u16, u16)> {
    // SAFETY: winsize is plain data, filled by the ioctl
    let mut ws: libc::winsize = unsafe { std::mem::zeroed() };
    // SAFETY: TIOCGWINSZ writes a winsize through the valid pointer
    let ok = unsafe { libc::ioctl(libc::STDOUT_FILENO, libc::TIOCGWINSZ, &mut ws) } == 0;
    ok.then_some((ws.ws_row, ws.ws_col))
}

/// Bridges the local terminal to a remote shell until the remote side disconnects (exit the shell to leave).
fn run_remote_tty(client: &mut LinuxClient, cmd: &str, use_upload_dir: bool) -> Result<()> {
    let (interface, client_id) = can_link(client)?;
    let mut tty = RemoteTtyClient::open(&interface, client_id)?;

    let mut size = window_size().unwrap_or((24, 80));
    if client.remote_tty_enabled()? {
        println!("{COLOR_NOTICE}Resuming existing remote tty connection...{COLOR_RESET}");
    } else {
        let term = std::env::var("TERM").unwrap_or_else(|_| "xterm-256color".into());
        client.enable_remote_tty(&term, size.0, size.1, cmd, use_upload_dir)?;
    }

    let _raw = RawTerminal::enable()?;
    let can_fd = tty.bus().as_raw_fd();
    let mut stdout = std::io::stdout();
    let mut stderr = std::io::stderr();
    while tty.disconnected().is_none() {
        let next_tick = tty.tick()?;
        let mut fds = [
            libc::pollfd { fd: can_fd, events: libc::POLLIN, revents: 0 },
            // Only read stdin while there is room to send it
            libc::pollfd {
                fd: if tty.stdin_can_write() { libc::STDIN_FILENO } else { -1 },
                events: libc::POLLIN,
                revents: 0,
            },
        ];
        let timeout_ms = next_tick.as_millis().clamp(1, 200) as i32;
        // SAFETY: fds is a valid array of two pollfds
        if unsafe { libc::poll(fds.as_mut_ptr(), 2, timeout_ms) } < 0 {
            let err = std::io::Error::last_os_error();
            if err.kind() != std::io::ErrorKind::Interrupted {
                return Err(err).context("poll");
            }
        }

        if fds[0].revents & libc::POLLIN != 0 {
            match tty.poll(Duration::from_micros(1))? {
                TtyEvent::Stdout(data) => {
                    stdout.write_all(&data)?;
                    stdout.flush()?;
                }
                TtyEvent::Stderr(data) => {
                    stderr.write_all(&data)?;
                    stderr.flush()?;
                }
                _ => {}
            }
        }

        if fds[1].revents & libc::POLLIN != 0 {
            let mut buf = vec![0u8; tty.max_frame_size()];
            // SAFETY: reading into a valid buffer of buf.len() bytes
            let n = unsafe { libc::read(libc::STDIN_FILENO, buf.as_mut_ptr().cast(), buf.len()) };
            if n <= 0 {
                break;
            }
            tty.stdin_write(&buf[..n as usize])?;
        }

        // Polling the window size is cheaper than wiring up SIGWINCH
        if let Some(new_size) = window_size()
            && new_size != size
        {
            tty.send_window_size(new_size.0, new_size.1)?;
            size = new_size;
        }
    }
    if tty.disconnected().is_none() {
        tty.disconnect(false)?;
    }
    Ok(())
}
