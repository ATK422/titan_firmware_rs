//! Dynamixel servo driver (protocol 2.0 over a half duplex TTL line), ported from `lib/driver_dynamixel`.
//!
//! * [`uart`]: the PIO half duplex UART (`async_uart.c`, `uart_multidrop.pio`)
//! * [`Driver`]: the bus owner. It polls every servo in the list every [`REFRESH_INTERVAL`] (ping until it answers,
//!   then read its EEPROM and RAM, disable torque and report it connected; after that read its RAM), and sends
//!   queued commands in between, ahead of the polling (`dynamixel_schedule.c`, `dynamixel_comms.c`).
//! * [`Servos`]: the state shared with the rest of the firmware. Declare it as a `static`. Commands such as
//!   [`Servos::set_target_position`] only queue the request, so they are safe from any context, including
//!   `on_kill` (`dynamixel.c`).
//!
//! The packet format, CRC and register map live in the host-tested `dynamixel_protocol` crate.
//!
//! ```ignore
//! static SERVOS: Servos<2> = Servos::new([2, 3]);
//!
//! #[embassy_executor::task]
//! async fn dynamixel_task(driver: Driver<'static, PIO0, 1, 1, 2, 2>) -> ! {
//!     driver.run(|event, id| { /* like the C event callback */ }, |err| { /* raise a fault */ }).await
//! }
//!
//! // in main
//! let Pio { mut common, sm1, irq_flags, irq1, irq2, .. } = Pio::new(p.PIO0, Irqs);
//! let program = MultidropProgram::new(&mut common);
//! let dma = dma::Channel::new(p.DMA_CH3, Irqs);
//! let uart = HalfDuplexUart::new(&mut common, sm1, irq_flags, irq1, irq2, p.PIN_10, dma, &program, BAUD_RATE);
//! spawner.spawn(dynamixel_task(Driver::new(uart, &SERVOS)).unwrap());
//! ```
//!
//! # Differences from the C driver
//!
//! * The event and error callbacks run in the driver's task instead of in interrupt context.
//! * A full command queue is reported through the error callback from the driver task, and the command methods
//!   return `false`, rather than calling the error callback from the caller's context.
//! * The CANmore debug commands (`@dynping`, `@dynread`, `@dynwrite`, ...) are not ported: the CANmore remote
//!   command interface doesn't exist in the Rust firmware yet.

#![no_std]

mod driver;
pub mod uart;

pub use driver::{Driver, Servos};
pub use dynamixel_protocol as protocol;
pub use dynamixel_protocol::{Eeprom, Ram};

use embassy_time::Duration;

/// A servo ID.
pub type Id = u8;

/// Baud rate of the servo bus (`DYNAMIXEL_BAUD_RATE`).
pub const BAUD_RATE: u32 = 57_600;
/// Longest wait for each part of a reply (`DYNAMIXEL_TIMEOUT_INTERVAL_MS`).
pub const REPLY_TIMEOUT: Duration = Duration::from_millis(50);
/// Time between the starts of two polling rounds (`DYNAMIXEL_REFRESH_INTERVAL_MS`).
pub const REFRESH_INTERVAL: Duration = Duration::from_millis(200);
/// Failed RAM reads in a row before a servo counts as disconnected (`DYNAMIXEL_MAX_MISSED_PINGS`).
pub const MAX_MISSED_PINGS: u8 = 3;
/// Depth of the command queue (`DYNAMIXEL_MAX_CMDS`).
pub const MAX_COMMANDS: usize = 8;
/// Largest parameter block in a packet (`DYNAMIXEL_PACKET_BUFFER_SIZE`).
pub const PACKET_BUFFER_SIZE: usize = 128;
/// Largest data block a queued write can carry. The C queue allowed up to the packet buffer, but nothing wrote more
/// than 4 bytes.
pub const MAX_WRITE_LEN: usize = 16;

/// Something the driver noticed while polling (`enum dynamixel_event`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum Event {
    /// The servo answered, its EEPROM and RAM were read and its torque disabled.
    Connected,
    /// The servo stopped answering, or didn't answer the first attempt to find it.
    Disconnected,
    /// A periodic RAM read completed.
    RamRead,
    /// A requested EEPROM read ([`Servos::request_eeprom_rescan`]) completed.
    EepromRead,
    /// The servo set the alert bit (a hardware error); the RAM's `hardware_error_status` tells which.
    Alert,
}

/// What went wrong (`enum dynamixel_error_code`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum ErrorKind {
    /// A bug in the driver.
    Driver = 1,
    /// The UART failed; `wrapped` is an [`uart::RxError`] or [`uart::TxError`] code.
    Ttl = 2,
    /// A request could not be built; `wrapped` is a packet build error code.
    Request = 3,
    /// A reply could not be parsed; `wrapped` is a `DXLLibErrorCode`.
    Packet = 4,
    /// The servo reported an error; `wrapped` is the error byte without the alert bit.
    Response = 5,
    /// The command queue was full and a command was dropped.
    CommandQueueFull = 6,
    /// A reply came from another servo; `wrapped` is its ID.
    InvalidId = 7,
}

/// Which part of the driver reported the error (`enum dynamixel_error_source`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum Source {
    Schedule = 0,
    Comms = 1,
}

/// A driver error (`dynamixel_error_t`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub struct Error {
    pub kind: ErrorKind,
    pub source: Source,
    pub wrapped: u8,
}

impl Error {
    pub(crate) const fn new(kind: ErrorKind, source: Source, wrapped: u8) -> Self {
        Error { kind, source, wrapped }
    }

    /// The error packed like the C `dynamixel_error_t` (code in bits 0-7, source in bit 8, wrapped code in bits
    /// 24-31), for a fault's `extra` argument. The C source line (bits 9-23) is not tracked and reads as 0.
    pub fn code(&self) -> u32 {
        self.kind as u32 | (self.source as u32) << 8 | (self.wrapped as u32) << 24
    }
}
