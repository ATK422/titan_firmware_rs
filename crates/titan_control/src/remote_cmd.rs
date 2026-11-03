//! Remote commands (`remote_cmd_handler.c`): the `@cmd` mechanism of `canmore_cli`.
//!
//! The agent writes `"cmd\0arg1\0arg2\0\0"` to the remote command args page and reads the execute register, which
//! runs the command and returns its exit code. The command's output goes to the response page as a NUL terminated
//! string.
//!
//! Commands are grouped in `static` slices of [`Command`] and looked up through a [`Registry`] (a list of groups),
//! so firmware can add commands from several places without allocating. Names must be unique across the registry
//! ([`Registry::duplicate`] finds clashes, which the C firmware turned into a panic at registration).
//!
//! The built-in `help` command lists the commands in name order, like the C binary search tree, and supports the
//! `-n` (count) and `-i <index>` (name, usage and help of one command) flags `canmore_cli` uses to discover them.

use core::fmt::{self, Write};

/// Most arguments (including the command name) a command line may have.
pub const MAX_ARGS: usize = 32;

/// A command that runs to completion inside the request handler.
///
/// `args.get(0)` is the command name. The return code is sent back to the agent; zero means success. Output written
/// to `out` beyond the response page is truncated, like the C `fmemopen` buffer.
pub type SyncHandler = fn(args: &Args<'_>, out: &mut Response<'_>) -> i32;

/// How a command runs.
#[derive(Clone, Copy)]
pub enum Handler {
    /// Runs inside the request handler.
    Sync(SyncHandler),
    /// Runs asynchronously outside the register map: [`execute`] returns [`Outcome::Deferred`] and the firmware
    /// runs the command (for example in a board task) before answering the execute register.
    Deferred,
    /// The built-in `help` command.
    #[doc(hidden)]
    Help,
}

/// A remote command (`debug_remote_cmd_register`).
#[derive(Clone, Copy)]
pub struct Command {
    pub name: &'static str,
    /// Argument synopsis, for example `"[a] [b]"`.
    pub usage: &'static str,
    pub help: &'static str,
    pub handler: Handler,
}

impl Command {
    /// A command run inside the request handler.
    pub const fn new(name: &'static str, usage: &'static str, help: &'static str, handler: SyncHandler) -> Self {
        Command { name, usage, help, handler: Handler::Sync(handler) }
    }

    /// A command run by the firmware outside the request handler (see [`Handler::Deferred`]).
    pub const fn deferred(name: &'static str, usage: &'static str, help: &'static str) -> Self {
        Command { name, usage, help, handler: Handler::Deferred }
    }
}

/// The commands every device has (`debug_remote_cmd_init`).
pub static BUILTIN: [Command; 1] = [Command {
    name: "help",
    usage: "[optional command]",
    help: "Shows remote command help\nIf command is provided, then it shows the help for that command.\nIf no \
           command is provided, a list of commands is given.",
    handler: Handler::Help,
}];

/// Why an argument buffer could not be parsed.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ParseError {
    /// More than [`MAX_ARGS`] arguments.
    TooManyArgs,
    /// An argument is not valid UTF-8.
    InvalidUtf8,
}

/// Parsed command line: the NUL separated strings of the args page up to the first empty one.
#[derive(Clone, Copy, Debug)]
pub struct Args<'a> {
    argv: [&'a str; MAX_ARGS],
    argc: usize,
}

impl<'a> Args<'a> {
    /// Parses `"cmd\0arg1\0\0"`. A buffer that ends without an empty string ends the list at the buffer end.
    pub fn parse(buf: &'a [u8]) -> Result<Self, ParseError> {
        let mut args = Args { argv: [""; MAX_ARGS], argc: 0 };
        let mut rest = buf;
        loop {
            let len = rest.iter().position(|&b| b == 0).unwrap_or(rest.len());
            if len == 0 {
                return Ok(args);
            }
            if args.argc == MAX_ARGS {
                return Err(ParseError::TooManyArgs);
            }
            args.argv[args.argc] = core::str::from_utf8(&rest[..len]).map_err(|_| ParseError::InvalidUtf8)?;
            args.argc += 1;
            rest = rest.get(len + 1..).unwrap_or(&[]);
        }
    }

    /// Builds arguments from strings (tests, local invocation).
    pub fn from_slice(argv: &[&'a str]) -> Self {
        assert!(argv.len() <= MAX_ARGS, "too many arguments");
        let mut args = Args { argv: [""; MAX_ARGS], argc: argv.len() };
        args.argv[..argv.len()].copy_from_slice(argv);
        args
    }

    /// Number of arguments, including the command name.
    pub fn len(&self) -> usize {
        self.argc
    }

    pub fn is_empty(&self) -> bool {
        self.argc == 0
    }

    /// Argument `i` (0 is the command name).
    pub fn get(&self, i: usize) -> Option<&'a str> {
        self.as_slice().get(i).copied()
    }

    /// All arguments, starting with the command name.
    pub fn as_slice(&self) -> &[&'a str] {
        &self.argv[..self.argc]
    }
}

/// Command output, written into the response page. Always NUL terminated; output that doesn't fit is dropped.
pub struct Response<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl<'a> Response<'a> {
    /// Starts an empty response in `buf` (at least 1 byte, for the terminator).
    pub fn new(buf: &'a mut [u8]) -> Self {
        assert!(!buf.is_empty(), "response buffer must hold the terminator");
        buf[0] = 0;
        Response { buf, len: 0 }
    }

    /// The text written so far.
    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }

    /// The text written so far, if it is valid UTF-8 (it is unless truncation split a character).
    pub fn as_str(&self) -> &str {
        core::str::from_utf8(self.as_bytes()).unwrap_or("")
    }

    /// Discards the output written so far.
    pub fn clear(&mut self) {
        self.len = 0;
        self.buf[0] = 0;
    }

    /// Room left, not counting the terminator.
    pub fn remaining(&self) -> usize {
        self.buf.len() - 1 - self.len
    }
}

impl Write for Response<'_> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let n = s.len().min(self.remaining());
        self.buf[self.len..self.len + n].copy_from_slice(&s.as_bytes()[..n]);
        self.len += n;
        self.buf[self.len] = 0;
        Ok(())
    }
}

/// The commands of a device: groups of commands, normally [`BUILTIN`] plus whatever the firmware registered.
#[derive(Clone, Copy)]
pub struct Registry<'a> {
    groups: &'a [&'a [Command]],
}

impl<'a> Registry<'a> {
    pub const fn new(groups: &'a [&'a [Command]]) -> Self {
        Registry { groups }
    }

    /// Every command, in registration order.
    pub fn iter(&self) -> impl Iterator<Item = &'a Command> + 'a {
        self.groups.iter().flat_map(|g| g.iter())
    }

    pub fn len(&self) -> usize {
        self.groups.iter().map(|g| g.len()).sum()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn find(&self, name: &str) -> Option<&'a Command> {
        self.iter().find(|c| c.name == name)
    }

    /// The `index`th command in name order (`strcmp` order, like the C binary search tree).
    pub fn by_sorted_index(&self, index: usize) -> Option<&'a Command> {
        // A handful of commands: counting the smaller names beats keeping a sorted copy
        self.iter().find(|c| self.iter().filter(|o| o.name.as_bytes() < c.name.as_bytes()).count() == index)
    }

    /// A name registered more than once, if any.
    pub fn duplicate(&self) -> Option<&'a str> {
        self.iter().enumerate().find_map(|(i, c)| self.iter().skip(i + 1).any(|o| o.name == c.name).then_some(c.name))
    }

    /// True if `group` can be added without clashing with a registered name.
    pub fn can_add(&self, group: &[Command]) -> bool {
        group.iter().all(|c| self.find(c.name).is_none())
            && group.iter().enumerate().all(|(i, c)| !group[i + 1..].iter().any(|o| o.name == c.name))
    }
}

/// What [`execute`] did.
#[derive(Clone, Copy)]
pub enum Outcome {
    /// The command ran (or failed to start); this is its return code.
    Done(i32),
    /// The command is [`Handler::Deferred`]: the caller runs it with the parsed arguments.
    Deferred(&'static Command),
}

/// Return code for a missing or unknown command (as a shell would say).
pub const RC_NOT_FOUND: i32 = 127;

/// Runs the command line in `args_buf` (`debug_remote_cmd_handle`). `out` receives the output.
pub fn execute(registry: &Registry<'static>, args_buf: &[u8], out: &mut Response<'_>) -> Outcome {
    out.clear();
    let args = match Args::parse(args_buf) {
        Ok(args) => args,
        Err(err) => {
            let _ = match err {
                ParseError::TooManyArgs => write!(out, "Too many arguments (at most {MAX_ARGS})"),
                ParseError::InvalidUtf8 => out.write_str("Arguments are not valid UTF-8"),
            };
            return Outcome::Done(1);
        }
    };
    let Some(name) = args.get(0) else {
        let _ = out.write_str("Cannot execute null command");
        return Outcome::Done(RC_NOT_FOUND);
    };
    let Some(command) = registry.find(name) else {
        let _ = write!(out, "Unknown Remote Command: '{name}'");
        return Outcome::Done(RC_NOT_FOUND);
    };
    match command.handler {
        Handler::Sync(handler) => Outcome::Done(handler(&args, out)),
        Handler::Help => Outcome::Done(help(registry, &args, out)),
        Handler::Deferred => Outcome::Deferred(command),
    }
}

/// The `help` command (`debug_remote_cmd_help_handler`), with the exact output format `canmore_cli` parses.
fn help(registry: &Registry<'_>, args: &Args<'_>, out: &mut Response<'_>) -> i32 {
    match args.as_slice() {
        [_] => {
            let _ = out.write_str("=====List of Commands=====\n");
            for i in 0..registry.len() {
                let Some(c) = registry.by_sorted_index(i) else { return -1 };
                let _ = writeln!(out, " {}", c.name);
            }
            0
        }
        [_, "-n"] => {
            let _ = write!(out, "{}", registry.len());
            0
        }
        [_, "-i", index] => {
            // atoi: garbage parses as 0
            let index = index.parse::<usize>().unwrap_or(0);
            match registry.by_sorted_index(index) {
                Some(c) => {
                    let _ = write!(out, "{}\n{}\n{}", c.name, c.usage, c.help);
                    0
                }
                None => 1,
            }
        }
        [_, name] => match registry.find(name) {
            Some(c) => {
                let _ = write!(out, "{} {}\n\t{}", c.name, c.usage, c.help);
                0
            }
            None => {
                let _ = out.write_str("No command found by name");
                1
            }
        },
        _ => {
            let _ = out.write_str("Unexpected arguments passed\nUsage: help [optional command name]");
            1
        }
    }
}

/// Parses an integer argument like `strtol(.., 0)`: decimal, `0x` hexadecimal or leading-zero octal, with an
/// optional sign. Handy for command handlers.
pub fn parse_int(s: &str) -> Option<i64> {
    let (negative, digits) = match s.as_bytes().first() {
        Some(b'-') => (true, &s[1..]),
        Some(b'+') => (false, &s[1..]),
        _ => (false, s),
    };
    let value = if let Some(hex) = digits.strip_prefix("0x").or_else(|| digits.strip_prefix("0X")) {
        i64::from_str_radix(hex, 16).ok()?
    } else if digits.len() > 1 && digits.starts_with('0') {
        i64::from_str_radix(&digits[1..], 8).ok()?
    } else {
        digits.parse::<i64>().ok()?
    };
    Some(if negative { -value } else { value })
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::string::String;

    fn add(args: &Args<'_>, out: &mut Response<'_>) -> i32 {
        let (Some(a), Some(b)) = (args.get(1).and_then(parse_int), args.get(2).and_then(parse_int)) else {
            let _ = out.write_str("Usage: add [a] [b]");
            return 1;
        };
        let _ = write!(out, "{}", a + b);
        0
    }

    static BOARD: [Command; 3] = [
        Command::new("add", "[a] [b]", "Adds [a] and [b]", add),
        Command::deferred("slow", "", "Runs elsewhere"),
        Command::new("echo", "[...]", "", |args, out| {
            for a in &args.as_slice()[1..] {
                let _ = write!(out, "{a} ");
            }
            0
        }),
    ];
    static GROUPS: [&[Command]; 2] = [&BUILTIN, &BOARD];
    const REGISTRY: Registry<'static> = Registry::new(&GROUPS);

    fn run(line: &[u8]) -> (Option<i32>, String) {
        let mut buf = [0xAAu8; 64];
        let mut out = Response::new(&mut buf);
        let rc = match execute(&REGISTRY, line, &mut out) {
            Outcome::Done(rc) => Some(rc),
            Outcome::Deferred(_) => None,
        };
        (rc, String::from(out.as_str()))
    }

    #[test]
    fn parsing() {
        let args = Args::parse(b"cmd\0a\0bc\0\0garbage").unwrap();
        assert_eq!(args.as_slice(), ["cmd", "a", "bc"]);
        assert!(Args::parse(b"\0cmd").unwrap().is_empty());
        assert_eq!(Args::parse(b"x").unwrap().as_slice(), ["x"]);
        assert_eq!(Args::parse(b"\xFF\0\0").unwrap_err(), ParseError::InvalidUtf8);
        assert_eq!(Args::parse(&[b'a', 0].repeat(33)).unwrap_err(), ParseError::TooManyArgs);
    }

    #[test]
    fn commands() {
        assert_eq!(run(b"add\x002\x000x10\0\0"), (Some(0), "18".into()));
        assert_eq!(run(b"add\x002\0\0"), (Some(1), "Usage: add [a] [b]".into()));
        assert_eq!(run(b"\0"), (Some(127), "Cannot execute null command".into()));
        assert_eq!(run(b"nope\0\0"), (Some(127), "Unknown Remote Command: 'nope'".into()));
        assert_eq!(run(b"slow\0\0").0, None);
    }

    #[test]
    fn help_discovery() {
        assert_eq!(run(b"help\0-n\0\0"), (Some(0), "4".into()));
        // Name order: add, echo, help, slow
        assert_eq!(run(b"help\0-i\x000\0\0"), (Some(0), "add\n[a] [b]\nAdds [a] and [b]".into()));
        assert_eq!(run(b"help\0-i\x003\0\0"), (Some(0), "slow\n\nRuns elsewhere".into()));
        assert_eq!(run(b"help\0-i\x004\0\0").0, Some(1));
        assert_eq!(run(b"help\0add\0\0"), (Some(0), "add [a] [b]\n\tAdds [a] and [b]".into()));
        assert_eq!(run(b"help\0zzz\0\0"), (Some(1), "No command found by name".into()));
        let (rc, list) = run(b"help\0\0");
        assert_eq!(rc, Some(0));
        assert_eq!(list, "=====List of Commands=====\n add\n echo\n help\n slow\n");
    }

    #[test]
    fn truncation() {
        let mut buf = [0u8; 8];
        let mut out = Response::new(&mut buf);
        let _ = out.write_str("0123456789");
        assert_eq!(out.as_str(), "0123456");
        assert_eq!(buf[7], 0);
    }

    #[test]
    fn duplicates() {
        assert!(REGISTRY.duplicate().is_none());
        static DUP: [&[Command]; 2] = [&BUILTIN, &BUILTIN];
        assert_eq!(Registry::new(&DUP).duplicate(), Some("help"));
        assert!(!REGISTRY.can_add(&BUILTIN));
        assert!(REGISTRY.can_add(&[Command::new("other", "", "", add)]));
    }

    #[test]
    fn ints() {
        assert_eq!(parse_int("42"), Some(42));
        assert_eq!(parse_int("-0x10"), Some(-16));
        assert_eq!(parse_int("010"), Some(8));
        assert_eq!(parse_int("0"), Some(0));
        assert_eq!(parse_int("x"), None);
    }
}
