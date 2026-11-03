//! The flash recorder (`storage.c`, `storage_tick`, `ivc/storage/configure`).
//!
//! The top 512 KiB of flash is a scratch area for capturing data on the vehicle and dumping it later. This is what
//! the C boards used `hardware_flash` and `hardware_sync` for; there is no persisted calibration. Configured to
//! write, the board erases the area and then fills it one capture sized chunk at a time. The C code was still being
//! brought up: instead of ADC captures (that line is commented out) it writes a test pattern, every byte of chunk
//! `n` holding `n + 1`, and the port keeps that. Configured to read, the board dumps the area to its log.
//! Comms pause while the recorder is enabled.

use crate::NUM_SAMPLES;

/// Size of the scratch area (`STORAGE_SIZE`).
pub const STORAGE_SIZE: u32 = 512 * 1024;
/// Bytes written per step: one capture (`NSAMP`).
pub const CHUNK_SIZE: usize = NUM_SAMPLES;
/// Flash erase granularity.
pub const SECTOR_SIZE: u32 = 4096;

/// Offset of the scratch area from the start of a flash of `flash_size` bytes: the top [`STORAGE_SIZE`]
/// (`STORAGE_OFFSET`).
pub const fn storage_offset(flash_size: u32) -> u32 {
    flash_size - STORAGE_SIZE
}

/// Recorder mode selected on `ivc/storage/configure`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StorageMode {
    /// 0: recorder off, comms running.
    Off,
    /// 1: erase the area and write it.
    Write,
    /// 2: dump the area to the log.
    Read,
}

impl StorageMode {
    /// Parses the `ivc/storage/configure` value; other values are ignored, like `configure_storage_callback`.
    pub const fn from_config(value: i8) -> Option<StorageMode> {
        match value {
            0 => Some(StorageMode::Off),
            1 => Some(StorageMode::Write),
            2 => Some(StorageMode::Read),
            _ => None,
        }
    }
}

/// One step of filling the scratch area.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WriteStep {
    /// Erase the sector at this offset within the area.
    Erase(u32),
    /// Write a chunk filled with `fill` at this offset within the area.
    Write { offset: u32, fill: u8 },
    /// The area is full (reported once).
    Done,
}

/// Progress through erasing and then writing the scratch area. The C code erased all 512 KiB in one call with
/// interrupts off, which takes seconds and outlasts the watchdog; this erases a sector per step so the firmware can
/// keep feeding it.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct WriteSession {
    erased: u32,
    written: u32,
    done_reported: bool,
}

impl WriteSession {
    pub const fn new() -> Self {
        WriteSession { erased: 0, written: 0, done_reported: false }
    }

    /// The next thing to do, or `None` once the area is full and [`WriteStep::Done`] has been returned.
    pub fn next_step(&mut self) -> Option<WriteStep> {
        if self.erased < STORAGE_SIZE {
            let offset = self.erased;
            self.erased += SECTOR_SIZE;
            Some(WriteStep::Erase(offset))
        } else if self.written < STORAGE_SIZE {
            let offset = self.written;
            self.written += CHUNK_SIZE as u32;
            // `memset(test_write_buffer, data++, NSAMP)` with `data` starting at 1
            let fill = (offset / CHUNK_SIZE as u32 + 1) as u8;
            Some(WriteStep::Write { offset, fill })
        } else if !self.done_reported {
            self.done_reported = true;
            Some(WriteStep::Done)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::vec::Vec;

    #[test]
    fn modes() {
        assert_eq!(StorageMode::from_config(0), Some(StorageMode::Off));
        assert_eq!(StorageMode::from_config(1), Some(StorageMode::Write));
        assert_eq!(StorageMode::from_config(2), Some(StorageMode::Read));
        assert_eq!(StorageMode::from_config(3), None);
        assert_eq!(StorageMode::from_config(-1), None);
    }

    #[test]
    fn area_is_the_top_of_flash() {
        assert_eq!(storage_offset(2 * 1024 * 1024), 1536 * 1024);
        assert_eq!(storage_offset(2 * 1024 * 1024) % SECTOR_SIZE, 0);
    }

    #[test]
    fn session_erases_then_writes_the_pattern() {
        let mut session = WriteSession::new();
        let steps: Vec<WriteStep> = core::iter::from_fn(|| session.next_step()).collect();
        let sectors = (STORAGE_SIZE / SECTOR_SIZE) as usize;
        let chunks = STORAGE_SIZE as usize / CHUNK_SIZE;
        assert_eq!(steps.len(), sectors + chunks + 1);
        assert!(steps[..sectors].iter().enumerate().all(|(i, s)| *s == WriteStep::Erase(i as u32 * SECTOR_SIZE)));
        assert_eq!(steps[sectors], WriteStep::Write { offset: 0, fill: 1 });
        assert_eq!(steps[sectors + chunks - 1], WriteStep::Write { offset: STORAGE_SIZE - 4096, fill: 128 });
        assert_eq!(steps.last(), Some(&WriteStep::Done));
        assert_eq!(session.next_step(), None);
    }
}
