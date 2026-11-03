//! The command line framework shared by the device CLIs, ported from `CLIInterface.hpp` and `cli_core.cpp`.
//!
//! A CLI owns a handle to the device (a debug, bootloader or Linux client) and a table of commands. Optionally it
//! has a prefix handler (`@cmd` runs `cmd` on the device) and a background task that runs every 1.5 s while the
//! prompt waits for input, to keep the connection alive and report events (fault raised, kill switch removed, ...).
//! A failing background task means the connection is lost and closes the CLI.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use anyhow::Result;
use rustyline::error::ReadlineError;
use rustyline::{DefaultEditor, ExternalPrinter};
use titan_tools::ui::{
    CLEAR_TERMINAL, COLOR_BODY, COLOR_ERROR, COLOR_NAME, COLOR_PROMPT, COLOR_RESET, CURSOR_GOTO_START,
};

const BACKGROUND_INTERVAL: Duration = Duration::from_millis(1500);

/// Per-CLI state passed to commands.
pub struct Session<T> {
    handle: Arc<Mutex<T>>,
    exit_message: Option<String>,
    /// `CANMORECLI_QUIET=1` silences the connection banner.
    pub quiet: bool,
    /// True when running a single command from the command line (no interactive prompt).
    pub one_shot: bool,
}

impl<T> Session<T> {
    /// Locks the device handle for the duration of a command.
    pub fn handle(&self) -> MutexGuard<'_, T> {
        self.handle.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Closes the CLI after the current command, printing `message`.
    pub fn close(&mut self, message: &str) {
        self.exit_message = Some(message.to_owned());
    }
}

/// A command: name, argument summary, help text and handler.
pub struct Command<T> {
    pub name: &'static str,
    pub args: &'static str,
    pub help: &'static str,
    pub run: fn(&mut Session<T>, &[String]) -> Result<()>,
}

/// Handles commands starting with a special character (`@` for remote commands).
pub trait PrefixHandler<T> {
    fn prefix(&self) -> char;
    fn run(&mut self, session: &mut Session<T>, args: &[String]) -> Result<()>;
    fn help(&mut self, session: &mut Session<T>, usage_only: bool);
}

/// Runs in the background while the prompt waits. Returns lines to print; an error closes the CLI.
pub type BackgroundTask<T> = Box<dyn FnMut(&mut T) -> Result<Vec<String>> + Send>;

pub struct Cli<T> {
    name: &'static str,
    commands: Vec<Command<T>>,
    prefixes: Vec<Box<dyn PrefixHandler<T>>>,
    background: Option<BackgroundTask<T>>,
    session: Session<T>,
}

/// Reads a `0`/`1` environment variable.
pub fn env_bool(name: &str) -> bool {
    match std::env::var(name).as_deref() {
        Ok("1") => true,
        Ok("0") | Err(_) => false,
        Ok(value) => {
            eprintln!(
                "[WARNING] Invalid boolean value '{value}' for environment variable '{name}'. Expected '0' or '1'."
            );
            false
        }
    }
}

fn builtin<T>() -> Vec<Command<T>> {
    vec![
        Command {
            name: "clear",
            args: "",
            help: "Clears the terminal.",
            run: |_, _| {
                print!("{CLEAR_TERMINAL}{CURSOR_GOTO_START}");
                Ok(())
            },
        },
        Command {
            name: "exit",
            args: "",
            help: "Closes the CLI and returns to device selection screen.",
            run: |s, _| {
                s.close("");
                Ok(())
            },
        },
        // Handled by the CLI itself, listed for help
        Command {
            name: "help",
            args: "[COMMAND]",
            help: "Shows this help message.\nIf COMMAND is provided, only help for that command is printed.",
            run: |_, _| Ok(()),
        },
    ]
}

pub fn write_multiline(text: &str, indent: &str) {
    for line in text.lines() {
        if line.is_empty() {
            println!();
        } else {
            println!("{indent}{line}");
        }
    }
}

impl<T: Send + 'static> Cli<T> {
    pub fn new(name: &'static str, handle: T) -> Self {
        Cli {
            name,
            commands: builtin(),
            prefixes: Vec::new(),
            background: None,
            session: Session {
                handle: Arc::new(Mutex::new(handle)),
                exit_message: None,
                quiet: env_bool("CANMORECLI_QUIET"),
                one_shot: false,
            },
        }
    }

    pub fn quiet(&self) -> bool {
        self.session.quiet
    }

    pub fn handle(&self) -> MutexGuard<'_, T> {
        self.session.handle()
    }

    pub fn register(&mut self, command: Command<T>) {
        assert!(!self.commands.iter().any(|c| c.name == command.name), "duplicate command {}", command.name);
        self.commands.push(command);
        self.commands.sort_by_key(|c| c.name);
    }

    pub fn add_prefix(&mut self, prefix: Box<dyn PrefixHandler<T>>) {
        assert!(!self.prefixes.iter().any(|p| p.prefix() == prefix.prefix()), "duplicate prefix");
        self.prefixes.push(prefix);
    }

    pub fn set_background(&mut self, task: BackgroundTask<T>) {
        self.background = Some(task);
    }

    /// Prints help for every command, or usage/help of one.
    pub fn show_help(&mut self, command: &str, usage_only: bool) {
        if command.is_empty() {
            for c in &self.commands {
                println!("{COLOR_NAME}{} {COLOR_BODY}{}{COLOR_RESET}", c.name, c.args);
                write_multiline(c.help, "\t");
            }
            for prefix in &mut self.prefixes {
                prefix.help(&mut self.session, false);
            }
            return;
        }
        if command.chars().count() == 1
            && let Some(prefix) = self.prefixes.iter_mut().find(|p| command.starts_with(p.prefix()))
        {
            prefix.help(&mut self.session, usage_only);
            return;
        }
        match self.commands.iter().find(|c| c.name == command) {
            None => println!("No such command: {command}"),
            Some(c) if usage_only => println!("Usage: {} {}", c.name, c.args),
            Some(c) => {
                println!("{} {}", c.name, c.args);
                write_multiline(c.help, "\t");
            }
        }
    }

    /// Runs one command line. Errors are reported and don't close the CLI.
    pub fn run_command(&mut self, cmd: &str, args: &[String]) {
        if cmd.is_empty() {
            return;
        }
        let result = if let Some(prefix) = self.prefixes.iter_mut().find(|p| cmd.starts_with(p.prefix())) {
            // "@cmd a b" and "@ cmd a b" are the same
            let mut prefixed = Vec::with_capacity(args.len() + 1);
            let rest = &cmd[prefix.prefix().len_utf8()..];
            if !rest.is_empty() {
                prefixed.push(rest.to_owned());
            }
            prefixed.extend_from_slice(args);
            prefix.run(&mut self.session, &prefixed)
        } else if cmd == "help" {
            let target = args.first().cloned().unwrap_or_default();
            self.show_help(&target, false);
            Ok(())
        } else if let Some(c) = self.commands.iter().find(|c| c.name == cmd) {
            let run = c.run;
            match run(&mut self.session, args) {
                Err(e) if e.downcast_ref::<UsageError>().is_some() => {
                    println!("{e}");
                    self.show_help(cmd, true);
                    Ok(())
                }
                other => other,
            }
        } else {
            println!("{cmd}: command not found");
            Ok(())
        };
        if let Err(e) = result {
            println!("{COLOR_ERROR}Exception caught while running command:{COLOR_RESET}");
            println!("{COLOR_ERROR}  what(): {e:#}{COLOR_RESET}");
        }
    }

    /// Runs a single command non-interactively.
    pub fn run_once(&mut self, cmd: &str, args: &[String]) {
        self.session.one_shot = true;
        self.run_command(cmd, args);
    }

    /// Runs the interactive prompt until `exit`, Ctrl+D or a lost connection.
    pub fn run(&mut self) -> Result<()> {
        println!("Connected to {} CLI. Type 'help' for a list of commands.", self.name);

        // Run the background task once up front, so a dead connection is noticed right away
        if let Some(task) = &mut self.background {
            for line in task(&mut self.session.handle())? {
                println!("{line}");
            }
        }

        let mut editor = DefaultEditor::new()?;
        let stop = Arc::new(AtomicBool::new(false));
        let lost = Arc::new(Mutex::new(None::<String>));
        let thread = match (self.background.take(), editor.create_external_printer()) {
            (Some(mut task), Ok(mut printer)) => {
                let handle = self.session.handle.clone();
                let (stop, lost) = (stop.clone(), lost.clone());
                Some(std::thread::spawn(move || {
                    'outer: while !stop.load(Ordering::Relaxed) {
                        // Sleep in small steps so closing the CLI doesn't wait for a whole interval
                        let start = std::time::Instant::now();
                        while start.elapsed() < BACKGROUND_INTERVAL {
                            std::thread::sleep(Duration::from_millis(50));
                            if stop.load(Ordering::Relaxed) {
                                break 'outer;
                            }
                        }
                        let result = task(&mut handle.lock().unwrap_or_else(|e| e.into_inner()));
                        match result {
                            Ok(lines) => {
                                for line in lines {
                                    let _ = printer.print(format!("{line}\n"));
                                }
                            }
                            Err(e) => {
                                let msg = format!("Connection lost: {e:#}");
                                let _ = printer.print(format!("{COLOR_ERROR}{msg} (press Enter){COLOR_RESET}\n"));
                                *lost.lock().unwrap() = Some(msg);
                                break;
                            }
                        }
                    }
                }))
            }
            _ => None,
        };

        let prompt = format!("{COLOR_PROMPT}{}> {COLOR_RESET}", self.name);
        self.session.exit_message = None;
        let exit_message = loop {
            let line = match editor.readline(&prompt) {
                Ok(line) => line,
                Err(ReadlineError::Interrupted) => continue,
                Err(ReadlineError::Eof) => break "Connection Closed".to_owned(),
                Err(e) => break format!("Terminal error: {e}"),
            };
            if let Some(msg) = lost.lock().unwrap().take() {
                break msg;
            }
            let mut words = line.split_whitespace().map(str::to_owned);
            let Some(cmd) = words.next() else { continue };
            let _ = editor.add_history_entry(line.trim());
            let args: Vec<String> = words.collect();
            self.run_command(&cmd, &args);
            if let Some(msg) = self.session.exit_message.take() {
                break msg;
            }
        };

        stop.store(true, Ordering::Relaxed);
        if let Some(thread) = thread {
            let _ = thread.join();
        }
        if !exit_message.is_empty() {
            println!("{exit_message}");
        }
        Ok(())
    }
}

/// A command was used wrongly: the message is printed followed by the command's usage.
#[derive(Debug)]
pub struct UsageError(pub String);

impl std::fmt::Display for UsageError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for UsageError {}

/// Returns a [`UsageError`].
pub fn usage<T>(msg: impl Into<String>) -> Result<T> {
    Err(UsageError(msg.into()).into())
}
