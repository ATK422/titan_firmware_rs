//! The flash recorder (`storage.c`, `storage_tick`), on embassy-rp's flash driver.
//!
//! See `ivc2_core::storage` for what it records. The C firmware dumped the area one byte per main loop pass with
//! `printf` on the UART; this logs 32 bytes per line over defmt, one line per millisecond. Each switch into write or
//! read mode starts over, where the C counters kept going (so a second write wrote nothing and a second dump printed
//! nothing).

use embassy_futures::yield_now;
use embassy_rp::flash::{Blocking, Flash};
use embassy_rp::peripherals::FLASH;
use embassy_time::Timer;

use ivc2_core::NUM_SAMPLES;
use ivc2_core::storage::{self, SECTOR_SIZE, STORAGE_SIZE, StorageMode, WriteSession, WriteStep};

/// Size of the board's flash (W25Q080 class, 2 MiB on every UWRT RP2040 board).
pub const FLASH_SIZE: usize = 2 * 1024 * 1024;
const STORAGE_OFFSET: u32 = storage::storage_offset(FLASH_SIZE as u32);
const LINE_LEN: usize = 32;

pub type BoardFlash = Flash<'static, FLASH, Blocking, FLASH_SIZE>;

enum Session {
    Idle,
    Write(WriteSession),
    Read { offset: u32 },
}

pub struct Recorder {
    flash: BoardFlash,
    session: Session,
}

impl Recorder {
    pub fn new(flash: BoardFlash) -> Self {
        Recorder { flash, session: Session::Idle }
    }

    /// Ends the current session (`ivc/storage/configure` 0).
    pub fn stop(&mut self) {
        self.session = Session::Idle;
    }

    /// Does one step of `mode`, using `buffer` as scratch space.
    pub async fn step(&mut self, mode: StorageMode, buffer: &mut [u8; NUM_SAMPLES]) {
        match (mode, &self.session) {
            (StorageMode::Write, Session::Write(_)) | (StorageMode::Read, Session::Read { .. }) => {}
            (StorageMode::Write, _) => {
                if program_overlaps_storage() {
                    defmt::error!("Firmware image reaches into the flash storage area, not writing it");
                    let mut finished = WriteSession::new();
                    while finished.next_step().is_some() {}
                    self.session = Session::Write(finished);
                } else {
                    defmt::info!("Erasing and writing flash storage");
                    self.session = Session::Write(WriteSession::new());
                }
            }
            (StorageMode::Read, _) => {
                defmt::info!("START");
                self.session = Session::Read { offset: 0 };
            }
            (StorageMode::Off, _) => {
                self.stop();
                return;
            }
        }

        match &mut self.session {
            Session::Write(session) => match session.next_step() {
                // One sector or one chunk per step, so the watchdog keeps getting fed in between
                Some(WriteStep::Erase(offset)) => {
                    let from = STORAGE_OFFSET + offset;
                    if let Err(err) = self.flash.blocking_erase(from, from + SECTOR_SIZE) {
                        defmt::error!("Flash erase at {=u32:#x} failed: {}", from, err);
                    }
                }
                Some(WriteStep::Write { offset, fill }) => {
                    buffer.fill(fill);
                    if let Err(err) = self.flash.blocking_write(STORAGE_OFFSET + offset, buffer) {
                        defmt::error!("Flash write at {=u32:#x} failed: {}", STORAGE_OFFSET + offset, err);
                    }
                }
                Some(WriteStep::Done) => defmt::info!("DONE WRITING"),
                None => Timer::after_millis(10).await,
            },
            Session::Read { offset } if *offset < STORAGE_SIZE => {
                let line = &mut buffer[..LINE_LEN];
                match self.flash.blocking_read(STORAGE_OFFSET + *offset, line) {
                    Ok(()) => defmt::info!("{=u32:#07x}: {=[u8]:02x}", *offset, line),
                    Err(err) => defmt::error!("Flash read at {=u32:#x} failed: {}", *offset, err),
                }
                *offset += LINE_LEN as u32;
                if *offset >= STORAGE_SIZE {
                    defmt::info!("DONE READING");
                }
                Timer::after_millis(1).await;
            }
            Session::Read { .. } | Session::Idle => Timer::after_millis(10).await,
        }
        yield_now().await;
    }
}

/// `check_flash_safe`: whether the firmware image (code, read-only data and the initial values of `.data`) ends past
/// the start of the storage area.
fn program_overlaps_storage() -> bool {
    unsafe extern "C" {
        static __sidata: u8;
        static __sdata: u8;
        static __edata: u8;
    }
    let data_len = (&raw const __edata) as u32 - (&raw const __sdata) as u32;
    let image_end = (&raw const __sidata) as u32 + data_len - embassy_rp::flash::FLASH_BASE as u32;
    image_end > STORAGE_OFFSET
}
