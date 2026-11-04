//! The crash log kept in RAM across resets (`crash_reporting.c`), and the watchdog scratch register protocol that
//! feeds it.
//!
//! While the firmware runs, watchdog scratch 0 holds what the chip is doing ([`reason`]): `UNKNOWN_SAFETY_PREINIT`
//! after safety setup, `UNKNOWN_SAFETY_ACTIVE` once ROS is up, or the crash being taken (`PANIC`, `HARD_FAULT`,
//! `WATCHDOG_TIMEOUT`, ...) with its details in scratch 1 and 2. A planned reset writes `CLEAN_BOOT` and the
//! [`clean_reset`] type to scratch 1 instead. Scratch 3 is unused, and 4-7 belong to the boot ROM and the bootloader.
//!
//! On the next boot [`process_reset`] turns those registers, the uptime and the fault list into a new
//! [`Entry`] of [`CrashData`], a ring of [`NUM_ENTRIES`] entries with a CRC that lives in RAM the runtime doesn't
//! initialize. A watchdog reset is told apart from a planned one the pico-sdk way: the watchdog reason register is
//! set and scratch 4 still holds [`WATCHDOG_NON_REBOOT_MAGIC`], which enabling the watchdog writes and every planned
//! reboot overwrites.
//!
//! The layout, magic values, CRC and messages are those of the C firmware, so the log reads the same in
//! `canmore_cli` and the messages match the C ones.

use core::fmt::{self, Write};

use canmore::reg_mapped::interface::debug::crash_log as itf;

use crate::debug::{CrashCounts, CrashEntry};

/// Values of watchdog scratch 0 (`safety_magic_values.h`).
pub mod reason {
    pub const CLEAN_BOOT: u32 = 0x0103_5000;
    pub const UNKNOWN_SAFETY_PREINIT: u32 = 0x0103_5001;
    pub const UNKNOWN_SAFETY_ACTIVE: u32 = 0x0103_5002;
    pub const PANIC: u32 = 0x0103_5003;
    pub const HARD_FAULT: u32 = 0x0103_5004;
    pub const ASSERT_FAIL: u32 = 0x0103_5005;
    pub const HARD_ASSERT: u32 = 0x0103_5007;
    pub const WATCHDOG_TIMEOUT: u32 = 0x0103_5008;
    pub const CORE1_TIMEOUT: u32 = 0x0103_5009;
}

/// Values of watchdog scratch 1 with a [`reason::CLEAN_BOOT`] (`CLEAN_RESET_TYPE_*`).
pub mod clean_reset {
    pub const POR: u32 = 0x0719_3001;
    pub const RUN: u32 = 0x0719_3002;
    pub const PSM: u32 = 0x0719_3003;
    pub const SOFTWARE: u32 = 0x0719_3004;
    pub const UNK_WDG: u32 = 0x0719_3005;
    pub const BOOTLOADER: u32 = 0x0719_3006;
    pub const BOOTROM: u32 = 0x0719_3007;
}

/// Number of entries in the crash log (`SAFETY_NUM_CRASH_LOG_ENTRIES`).
pub const NUM_ENTRIES: usize = 24;
/// The uptime is stored with its XOR against this, to tell a valid count from random RAM.
pub const UPTIME_TICK_XOR_MAGIC: u32 = 0xA713_9207;
/// Uptime ticks per second (the C SysTick rate).
pub const UPTIME_TICKS_PER_SECOND: u32 = 100;
/// Scratch 4 value written when the watchdog is enabled (pico-sdk `WATCHDOG_NON_REBOOT_MAGIC`).
pub const WATCHDOG_NON_REBOOT_MAGIC: u32 = 0x6AB7_3121;
/// `flags.valid` of an initialized crash log (`VALID_MAGIC_VALUE`).
const VALID_MAGIC: u8 = 0x5;

/// One reset (`struct crash_log_entry`).
#[repr(C)]
#[derive(Clone, Copy, Default, Debug, PartialEq, Eq)]
pub struct Entry {
    pub reset_reason: u32,
    pub scratch_1: u32,
    pub scratch_2: u32,
    /// Uptime at the reset in [`UPTIME_TICKS_PER_SECOND`] ticks, 0 if unknown.
    pub uptime: u32,
    pub faults: u32,
}

/// The crash log (`struct crash_data`), with the C layout: a header word (CRC in the low half, then the flags byte
/// with `valid` in bits 0-3 and `log_wrapped` in bit 4, then the next entry index), a word of four crash counters
/// (total, panics, hard faults, assert failures) and the entries.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CrashData {
    header: u32,
    counter: u32,
    log: [Entry; NUM_ENTRIES],
}

/// CRC-16/CCITT (polynomial 0x1021, MSB first) seeded with 0x1541, as `calc_crc16`.
pub fn crc16(data: impl IntoIterator<Item = u8>) -> u16 {
    let mut crc: u16 = 0x1541;
    for byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x1021 } else { crc << 1 };
        }
    }
    crc
}

impl CrashData {
    /// An empty, valid crash log.
    pub fn new() -> Self {
        let mut data =
            CrashData { header: (VALID_MAGIC as u32) << 16, counter: 0, log: [Entry::default(); NUM_ENTRIES] };
        data.update_crc();
        data
    }

    fn stored_crc(&self) -> u16 {
        self.header as u16
    }

    fn flags(&self) -> u8 {
        (self.header >> 16) as u8
    }

    /// Index of the entry the next reset goes into.
    pub fn next_entry(&self) -> usize {
        (self.header >> 24) as usize
    }

    /// True if older entries were overwritten.
    pub fn wrapped(&self) -> bool {
        self.flags() & 0x10 != 0
    }

    /// Number of entries recorded.
    pub fn len(&self) -> usize {
        if self.wrapped() { NUM_ENTRIES } else { self.next_entry().min(NUM_ENTRIES) }
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// The CRC of everything after the CRC itself (`calc_crash_data_crc`).
    pub fn compute_crc(&self) -> u16 {
        let header = self.header.to_le_bytes();
        let words = [self.counter]
            .into_iter()
            .chain(self.log.iter().flat_map(|e| [e.reset_reason, e.scratch_1, e.scratch_2, e.uptime, e.faults]));
        crc16(header[2..].iter().copied().chain(words.flat_map(u32::to_le_bytes)))
    }

    fn update_crc(&mut self) {
        self.header = (self.header & !0xFFFF) | self.compute_crc() as u32;
    }

    /// True if the flags mark the log valid and the CRC matches.
    pub fn is_valid(&self) -> bool {
        self.flags() & 0x0F == VALID_MAGIC && self.compute_crc() == self.stored_crc()
    }

    /// Crash counters since the last clean boot.
    pub fn counts(&self) -> CrashCounts {
        let [total, panic, hard_fault, assert_fail] = self.counter.to_le_bytes().map(u32::from);
        CrashCounts { total, panic, hard_fault, assert_fail }
    }

    /// Entry `index` back from the most recent (0).
    pub fn entry(&self, index: usize) -> Option<Entry> {
        if index >= self.len() {
            return None;
        }
        let i = (self.next_entry() + NUM_ENTRIES - 1 - index) % NUM_ENTRIES;
        Some(self.log[i])
    }

    fn bump(&mut self, counter: usize) {
        let mut counts = self.counter.to_le_bytes();
        counts[counter] = counts[counter].saturating_add(1);
        self.counter = u32::from_le_bytes(counts);
    }
}

impl Default for CrashData {
    fn default() -> Self {
        Self::new()
    }
}

/// Which reset flags the chip reported (`VREG_AND_CHIP_RESET.CHIP_RESET`).
#[derive(Clone, Copy, Default, Debug)]
pub struct ChipReset {
    pub had_por: bool,
    pub had_run: bool,
    pub had_psm_restart: bool,
}

/// What the hardware held at boot, before anything overwrote it.
#[derive(Clone, Copy, Default, Debug)]
pub struct ResetInputs {
    /// The watchdog reason register is non-zero (`watchdog_caused_reboot`).
    pub watchdog_reason: bool,
    /// Watchdog scratch 0-2.
    pub scratch: [u32; 3],
    /// Watchdog scratch 4.
    pub scratch_4: u32,
    /// The uptime count, if its check word was intact.
    pub uptime: Option<u32>,
    /// The fault list of the previous run.
    pub faults: u32,
    pub chip_reset: ChipReset,
}

impl ResetInputs {
    /// The watchdog fired while enabled (`watchdog_enable_caused_reboot`): a crash rather than a planned reset.
    pub fn crashed(&self) -> bool {
        self.watchdog_reason && self.scratch_4 == WATCHDOG_NON_REBOOT_MAGIC
    }
}

/// What [`process_reset`] found.
#[derive(Clone, Copy, Default, Debug, PartialEq, Eq)]
pub struct ResetOutcome {
    /// Raise `FAULT_WATCHDOG_RESET`.
    pub raise_watchdog_reset: bool,
    /// A crash reset found the log corrupted (it was cleared).
    pub log_corrupted: bool,
    /// Scratch 0 said clean boot although the watchdog reported a crash.
    pub clean_boot_on_crash: bool,
}

/// Records the previous reset in `data` (`safety_process_last_reset_cause`). Call once at boot; the caller then
/// clears the uptime and fault list and sets scratch 0 to [`reason::UNKNOWN_SAFETY_PREINIT`].
pub fn process_reset(data: &mut CrashData, inputs: &ResetInputs) -> ResetOutcome {
    let mut outcome = ResetOutcome::default();
    let crashed = inputs.crashed();
    if !data.is_valid() {
        outcome.log_corrupted = crashed;
        outcome.raise_watchdog_reset = crashed;
        *data = CrashData::new();
    }

    let mut entry = Entry { uptime: inputs.uptime.unwrap_or(0), ..Entry::default() };
    if crashed {
        entry.reset_reason = inputs.scratch[0];
        entry.faults = inputs.faults;
        entry.scratch_1 = inputs.scratch[1];
        entry.scratch_2 = inputs.scratch[2];
        data.bump(0);
        match entry.reset_reason {
            reason::PANIC => data.bump(1),
            reason::HARD_FAULT => data.bump(2),
            reason::ASSERT_FAIL | reason::HARD_ASSERT => data.bump(3),
            _ => {}
        }
        outcome.clean_boot_on_crash = entry.reset_reason == reason::CLEAN_BOOT;
        outcome.raise_watchdog_reset = true;
    } else {
        // A clean boot clears the counters, and with them the sticky fault
        data.counter = 0;
        entry.reset_reason = reason::CLEAN_BOOT;
        entry.scratch_1 = if inputs.scratch[0] == reason::CLEAN_BOOT {
            inputs.scratch[1]
        } else if inputs.watchdog_reason {
            clean_reset::UNK_WDG
        } else if inputs.chip_reset.had_psm_restart {
            clean_reset::PSM
        } else if inputs.chip_reset.had_run {
            clean_reset::RUN
        } else if inputs.chip_reset.had_por {
            clean_reset::POR
        } else {
            0
        };
    }

    let next = data.next_entry();
    data.log[next] = entry;
    let (next, wrapped) = if next == NUM_ENTRIES - 1 { (0, true) } else { (next + 1, data.wrapped()) };
    let flags = (data.flags() & 0x0F) | if wrapped { 0x10 } else { 0 };
    data.header = (next as u32) << 24 | (flags as u32) << 16;
    data.update_crc();
    outcome
}

/// Writes `uptime` ticks like the C log (`%.2f` seconds, minutes or hours), without floating point.
fn write_uptime(out: &mut impl Write, ticks: u32) -> fmt::Result {
    let per_second = UPTIME_TICKS_PER_SECOND;
    let (unit, scale) = if ticks > 3600 * per_second {
        ('h', 3600 * per_second)
    } else if ticks > 60 * per_second {
        ('m', 60 * per_second)
    } else {
        ('s', per_second)
    };
    // Hundredths, rounded like printf
    let hundredths = (ticks as u64 * 100 + scale as u64 / 2) / scale as u64;
    write!(out, " - Uptime: {}.{:02} {}", hundredths / 100, hundredths % 100, unit)
}

/// Formats one entry like `safety_format_reset_cause_entry`.
pub fn format_entry(out: &mut impl Write, entry: &Entry) -> fmt::Result {
    if entry.reset_reason == reason::CLEAN_BOOT {
        let reset_type = match entry.scratch_1 {
            clean_reset::POR => "Power-on Reset",
            clean_reset::RUN => "RESET Pin",
            clean_reset::PSM => "PSM Debug Request",
            clean_reset::SOFTWARE => "Software Request",
            clean_reset::UNK_WDG => "Unknown Watchdog Request",
            clean_reset::BOOTLOADER => "Bootloader",
            clean_reset::BOOTROM => "Bootrom USB Upload",
            _ => "Invalid",
        };
        write!(out, "Clean Boot: {reset_type}")?;
    } else {
        let (s1, s2) = (entry.scratch_1, entry.scratch_2);
        match entry.reset_reason {
            reason::UNKNOWN_SAFETY_PREINIT => write!(out, "UNKNOWN_SAFETY_PREINIT")?,
            reason::UNKNOWN_SAFETY_ACTIVE => write!(out, "UNKNOWN_SAFETY_ACTIVE")?,
            reason::PANIC => write!(out, "PANIC (Message: 0x{s1:08X}, Call Address: 0x{s2:08X})")?,
            reason::HARD_FAULT => write!(out, "HARD_FAULT (Fault Address: 0x{s1:08X})")?,
            reason::ASSERT_FAIL => write!(out, "ASSERT_FAIL (File: 0x{s1:08X} Line: {})", s2 as i32)?,
            reason::HARD_ASSERT => write!(out, "HARD_ASSERT (Call Address: 0x{s1:08X})")?,
            reason::WATCHDOG_TIMEOUT => write!(out, "WATCHDOG_TIMEOUT (Last Address: 0x{s1:08X})")?,
            reason::CORE1_TIMEOUT => write!(out, "CORE1_TIMEOUT (Last Address: 0x{s1:08X})")?,
            _ => write!(out, "Invalid Data in Reason Register")?,
        }
        write!(out, " - Faults: 0x{:08X}", entry.faults)?;
    }
    if entry.uptime != 0 {
        write_uptime(out, entry.uptime)?;
    }
    Ok(())
}

/// Formats the most recent reset like `safety_print_last_reset_cause`, or returns `None` if there is none.
pub fn format_last_reset(out: &mut impl Write, data: &CrashData) -> Option<fmt::Result> {
    let last = data.entry(0)?;
    Some((|| {
        if last.reset_reason != reason::CLEAN_BOOT {
            let c = data.counts();
            write!(out, "Watchdog Reset (Total Crashes: {}", c.total)?;
            if c.panic != 0 {
                write!(out, " - Panics: {}", c.panic)?;
            }
            if c.hard_fault != 0 {
                write!(out, " - Hard Faults: {}", c.hard_fault)?;
            }
            if c.assert_fail != 0 {
                write!(out, " - Assert Fails: {}", c.assert_fail)?;
            }
            write!(out, ") - ")?;
        }
        format_entry(out, &last)
    })())
}

/// The entry in protocol terms for the crash log page (`crash_log.h`): reasons and clean reset types become the
/// small `RESET_REASON_*` / `CLEAN_RESET_*` values, unknown ones stay raw.
pub fn to_protocol(entry: &Entry) -> CrashEntry {
    let reset_reason = match entry.reset_reason {
        reason::CLEAN_BOOT => itf::RESET_REASON_CLEAN_BOOT,
        reason::UNKNOWN_SAFETY_PREINIT => itf::RESET_REASON_UNKNOWN_PREINIT,
        reason::UNKNOWN_SAFETY_ACTIVE => itf::RESET_REASON_UNKNOWN_ACTIVE,
        reason::PANIC => itf::RESET_REASON_PANIC,
        reason::HARD_FAULT => itf::RESET_REASON_HARD_FAULT,
        reason::ASSERT_FAIL => itf::RESET_REASON_ASSERT_FAIL,
        reason::HARD_ASSERT => itf::RESET_REASON_HARD_ASSERT,
        reason::WATCHDOG_TIMEOUT => itf::RESET_REASON_WATCHDOG_TIMEOUT,
        reason::CORE1_TIMEOUT => itf::RESET_REASON_CORE1_TIMEOUT,
        raw => raw,
    };
    let scratch_1 = if entry.reset_reason == reason::CLEAN_BOOT {
        match entry.scratch_1 {
            clean_reset::POR => itf::CLEAN_RESET_POR,
            clean_reset::RUN => itf::CLEAN_RESET_RUN,
            clean_reset::PSM => itf::CLEAN_RESET_PSM,
            clean_reset::SOFTWARE => itf::CLEAN_RESET_SOFTWARE,
            clean_reset::UNK_WDG => itf::CLEAN_RESET_UNK_WDG,
            clean_reset::BOOTLOADER => itf::CLEAN_RESET_BOOTLOADER,
            clean_reset::BOOTROM => itf::CLEAN_RESET_BOOTROM,
            raw => raw,
        }
    } else {
        entry.scratch_1
    };
    CrashEntry { reset_reason, faults: entry.faults, uptime: entry.uptime, scratch_1, scratch_2: entry.scratch_2 }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use std::string::String;

    use super::*;

    fn boot(data: &mut CrashData, inputs: ResetInputs) -> ResetOutcome {
        process_reset(data, &inputs)
    }

    fn por() -> ResetInputs {
        ResetInputs { chip_reset: ChipReset { had_por: true, ..ChipReset::default() }, ..ResetInputs::default() }
    }

    fn crash(reason: u32, s1: u32, s2: u32, faults: u32, uptime: u32) -> ResetInputs {
        ResetInputs {
            watchdog_reason: true,
            scratch: [reason, s1, s2],
            scratch_4: WATCHDOG_NON_REBOOT_MAGIC,
            uptime: Some(uptime),
            faults,
            ..ResetInputs::default()
        }
    }

    fn last(data: &CrashData) -> String {
        let mut s = String::new();
        format_last_reset(&mut s, data).unwrap().unwrap();
        s
    }

    #[test]
    fn crc_matches_the_c_table() {
        // crc16_poly_table[i] is the CRC register after shifting byte i through a zero register
        let table_entry = |i: u8| {
            let mut crc = (i as u16) << 8;
            for _ in 0..8 {
                crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x1021 } else { crc << 1 };
            }
            crc
        };
        for (i, expected) in [(1, 0x1021), (16, 0x1231), (32, 0x2462), (100, 0x2C22), (173, 0x6447), (255, 0x1EF0)] {
            assert_eq!(table_entry(i), expected, "table entry {i}");
        }
        // And the table-driven update equals the bitwise one
        let table: std::vec::Vec<u16> = (0..=255).map(table_entry).collect();
        let mut crc: u16 = 0x1541;
        for b in b"titan" {
            crc = (crc << 8) ^ table[(((crc >> 8) as u8) ^ b) as usize];
        }
        assert_eq!(crc16(*b"titan"), crc);
    }

    #[test]
    fn power_on_is_a_clean_boot() {
        let mut data = CrashData::new();
        let outcome = boot(&mut data, por());
        assert_eq!(outcome, ResetOutcome::default());
        assert_eq!(last(&data), "Clean Boot: Power-on Reset");
        assert!(data.is_valid());
        assert_eq!(data.len(), 1);
    }

    #[test]
    fn a_hang_matches_the_c_message() {
        // What the C power board printed after a hang in the emulator
        let mut data = CrashData::new();
        boot(&mut data, por());
        let outcome = boot(&mut data, crash(reason::UNKNOWN_SAFETY_PREINIT, 0, 0, 0x80, 1042));
        assert!(outcome.raise_watchdog_reset);
        assert_eq!(
            last(&data),
            "Watchdog Reset (Total Crashes: 1) - UNKNOWN_SAFETY_PREINIT - Faults: 0x00000080 - Uptime: 10.42 s"
        );
    }

    #[test]
    fn counters_count_until_a_clean_boot() {
        let mut data = CrashData::new();
        boot(&mut data, crash(reason::PANIC, 0x1000_1234, 0x1000_5678, 0, 6000));
        boot(&mut data, crash(reason::HARD_FAULT, 0x2000_0000, 0, 1, 100));
        assert_eq!(data.counts(), CrashCounts { total: 2, panic: 1, hard_fault: 1, assert_fail: 0 });
        assert_eq!(
            last(&data),
            "Watchdog Reset (Total Crashes: 2 - Panics: 1 - Hard Faults: 1) - HARD_FAULT (Fault Address: \
             0x20000000) - Faults: 0x00000001 - Uptime: 1.00 s"
        );
        let mut s = String::new();
        format_entry(&mut s, &data.entry(1).unwrap()).unwrap();
        assert_eq!(s, "PANIC (Message: 0x10001234, Call Address: 0x10005678) - Faults: 0x00000000 - Uptime: 60.00 s");

        boot(&mut data, por());
        assert_eq!(data.counts(), CrashCounts::default());
        assert_eq!(data.len(), 3);
    }

    #[test]
    fn planned_reboots_keep_their_type() {
        let mut data = CrashData::new();
        // Software reboot: scratch 0 says clean boot, scratch 4 was cleared by the reboot
        let inputs = ResetInputs {
            watchdog_reason: true,
            scratch: [reason::CLEAN_BOOT, clean_reset::SOFTWARE, 0],
            ..ResetInputs::default()
        };
        assert!(!boot(&mut data, inputs).raise_watchdog_reset);
        assert_eq!(last(&data), "Clean Boot: Software Request");
        assert_eq!(to_protocol(&data.entry(0).unwrap()).scratch_1, itf::CLEAN_RESET_SOFTWARE);

        // A watchdog reset without our magic and without a clean boot marker
        let inputs = ResetInputs { watchdog_reason: true, scratch: [0, 0, 0], ..ResetInputs::default() };
        boot(&mut data, inputs);
        assert_eq!(last(&data), "Clean Boot: Unknown Watchdog Request");
    }

    #[test]
    fn the_log_wraps() {
        let mut data = CrashData::new();
        for i in 0..NUM_ENTRIES as u32 + 3 {
            boot(&mut data, crash(reason::WATCHDOG_TIMEOUT, 0x1000_0000 + i, 0, 0, 0));
        }
        assert!(data.wrapped());
        assert_eq!(data.len(), NUM_ENTRIES);
        assert_eq!(data.entry(0).unwrap().scratch_1, 0x1000_0000 + NUM_ENTRIES as u32 + 2);
        assert_eq!(data.entry(NUM_ENTRIES - 1).unwrap().scratch_1, 0x1000_0003);
        assert_eq!(data.entry(NUM_ENTRIES), None);
        assert_eq!(data.counts().total, 0xFF.min(NUM_ENTRIES as u32 + 3));
    }

    #[test]
    fn corruption_is_reported_on_a_crash_only() {
        let mut data = CrashData::new();
        boot(&mut data, por());
        data.log[0].faults ^= 1;
        let outcome = boot(&mut data, crash(reason::PANIC, 0, 0, 0, 0));
        assert!(outcome.log_corrupted && outcome.raise_watchdog_reset);
        assert_eq!(data.len(), 1);

        data.log[0].faults ^= 1;
        assert!(!boot(&mut data, por()).log_corrupted);
    }

    #[test]
    fn protocol_values() {
        let e = Entry { reset_reason: reason::WATCHDOG_TIMEOUT, scratch_1: 0x1000_0100, ..Entry::default() };
        assert_eq!(to_protocol(&e).reset_reason, itf::RESET_REASON_WATCHDOG_TIMEOUT);
        assert_eq!(to_protocol(&e).scratch_1, 0x1000_0100);
        let raw = Entry { reset_reason: 0x1234, ..Entry::default() };
        assert_eq!(to_protocol(&raw).reset_reason, 0x1234);
    }

    #[test]
    fn uptime_units() {
        let fmt = |ticks| {
            let mut s = String::new();
            write_uptime(&mut s, ticks).unwrap();
            s
        };
        assert_eq!(fmt(1), " - Uptime: 0.01 s");
        assert_eq!(fmt(6001), " - Uptime: 1.00 m");
        assert_eq!(fmt(360_001), " - Uptime: 1.00 h");
    }
}
