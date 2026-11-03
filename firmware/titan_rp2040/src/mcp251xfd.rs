//! Async driver for the Microchip MCP2517FD/MCP2518FD SPI CAN FD controller.
//!
//! Configured the same way `driver_canbus` configures it through the MCP251XFD library:
//! * CLKIN is a 4 MHz reference from the RP2040, multiplied by the PLL to a 40 MHz SYSCLK
//! * bit timing with one time quantum per SYSCLK cycle and an ~80 % nominal sample point
//! * a transmit queue, a transmit event FIFO (used to confirm heartbeats actually went out) and one receive FIFO fed
//!   by acceptance filters
//! * RAM ECC enabled; GPIO0 used as an input sensing the termination resistor jumper
//!
//! One deliberate difference: `driver_canbus` restricts every frame to three transmission attempts. When they run
//! out (no other node acknowledging, after the cable came out, or an error burst), the controller clears TXREQ and
//! keeps the frame at the head of the queue, and nothing requests it again until a new frame is queued, which can't
//! happen once the queue is full. The board then stayed silent on a working bus until the 10 s offline reset in
//! [`crate::canbus`]. Here the transmit queue retries without limit, as CAN controllers normally do, so it starts
//! sending again as soon as the bus works.

use embassy_rp::gpio::{Input, Output};
use embassy_rp::peripherals::SPI0;
use embassy_rp::spi::{Async, Spi};
use embassy_time::{Duration, Instant, Timer};

use canmore::Frame;

/// The SPI bus the CAN block wires the controller to.
pub type CanSpi = Spi<'static, SPI0, Async>;

/// SYSCLK after the PLL.
pub const SYSCLK_HZ: u32 = 40_000_000;
/// Required reference clock on CLKIN.
pub const CLKIN_HZ: u32 = 4_000_000;
/// SPI clock before the PLL locks. Must stay below CLKIN / 2.
const SPI_SLOW_HZ: u32 = 1_000_000;
/// SPI clock once SYSCLK runs at 40 MHz (datasheet limit is 0.85 * SYSCLK / 2).
const SPI_FAST_HZ: u32 = 17_000_000;

const CMD_RESET: u8 = 0x0;
const CMD_WRITE: u8 = 0x2;
const CMD_READ: u8 = 0x3;

const RAM_START: u16 = 0x400;
const RAM_SIZE: u16 = 2048;
/// Message object header (ID word + flags word).
const OBJ_HEADER: usize = 8;
/// Largest message object: header plus a 64 byte payload.
const OBJ_MAX: usize = OBJ_HEADER + 64;

/// Depth of the transmit event FIFO.
const TEF_DEPTH: u32 = 8;
/// Depth of the transmit queue.
const TXQ_DEPTH: u32 = 8;
/// Depth of the receive FIFO (FIFO 1).
const RX_DEPTH: u32 = 16;
// TEF: 8 * 8 B, TXQ: 8 * 72 B, RX: 16 * 72 B = 1792 B of the 2048 B RAM
const _: () = assert!(TEF_DEPTH * 8 + (TXQ_DEPTH + RX_DEPTH) * OBJ_MAX as u32 <= RAM_SIZE as u32);

mod reg {
    pub const C1CON: u16 = 0x000;
    pub const C1NBTCFG: u16 = 0x004;
    pub const C1DBTCFG: u16 = 0x008;
    pub const C1TDC: u16 = 0x00C;
    pub const C1INT: u16 = 0x01C;
    pub const C1TREC: u16 = 0x034;
    pub const C1TEFCON: u16 = 0x040;
    pub const C1TEFSTA: u16 = 0x044;
    pub const C1TEFUA: u16 = 0x048;
    pub const C1TXQCON: u16 = 0x050;
    pub const C1TXQSTA: u16 = 0x054;
    pub const C1TXQUA: u16 = 0x058;
    pub const C1FIFOCON1: u16 = 0x05C;
    pub const C1FIFOSTA1: u16 = 0x060;
    pub const C1FIFOUA1: u16 = 0x064;
    pub const C1FLTCON0: u16 = 0x1D0;
    pub const C1FLTOBJ0: u16 = 0x1F0;
    pub const C1MASK0: u16 = 0x1F4;
    pub const OSC: u16 = 0xE00;
    pub const IOCON: u16 = 0xE04;
    pub const ECCCON: u16 = 0xE0C;
    pub const ECCSTAT: u16 = 0xE10;
}

// OSC
const OSC_PLLEN: u32 = 1 << 0;
const OSC_PLLRDY: u32 = 1 << 8;
const OSC_OSCRDY: u32 = 1 << 10;

// C1CON
const CON_ISOCRCEN: u32 = 1 << 5;
const CON_PXEDIS: u32 = 1 << 6;
const CON_WAKFIL: u32 = 1 << 8;
const CON_WFT_T11: u32 = 0b11 << 9;
const CON_STEF: u32 = 1 << 19;
const CON_TXQEN: u32 = 1 << 20;
const CON_REQOP_SHIFT: u32 = 24;
const OPMOD_SHIFT: u32 = 21;
const MODE_CANFD: u32 = 0b000;
const MODE_CONFIG: u32 = 0b100;
const MODE_CAN20: u32 = 0b110;

// C1INT flags (low half) and enables (high half)
pub const INT_RXIF: u32 = 1 << 1;
pub const INT_TEFIF: u32 = 1 << 4;
pub const INT_ECCIF: u32 = 1 << 8;
pub const INT_RXOVIF: u32 = 1 << 11;
pub const INT_SERRIF: u32 = 1 << 12;
const INT_ENABLES: u32 = (INT_RXIF | INT_TEFIF | INT_ECCIF | INT_RXOVIF | INT_SERRIF) << 16;
/// Flags cleared by writing zero to C1INT. The rest mirror FIFO/ECC status and clear at their source.
const INT_CLEARABLE: u32 = INT_SERRIF;

// FIFO control/status
const FIFO_TFNRFNIE: u32 = 1 << 0;
const FIFO_RXOVIE: u32 = 1 << 3;
const FIFO_UINC: u8 = 1 << 0; // bit 8, in byte 1
const FIFO_TXREQ: u8 = 1 << 1; // bit 9, in byte 1
const FIFO_FSIZE_SHIFT: u32 = 24;
const FIFO_PLSIZE_64: u32 = 0b111 << 29;
/// Unlimited retransmission attempts (also what C1CON.RTXAT = 0 means, which makes the controller ignore TXAT).
const FIFO_TXAT_UNLIMITED: u32 = 0b11 << 21;
const STA_NOT_EMPTY_OR_FULL: u8 = 1 << 0;
const TEF_TEFNEIE: u32 = 1 << 0;

// Message object flags word
const OBJ_IDE: u32 = 1 << 4;
const OBJ_BRS: u32 = 1 << 6;
const OBJ_FDF: u32 = 1 << 7;
const OBJ_SEQ_SHIFT: u32 = 9;

// Filters / masks
const FLT_EXIDE: u32 = 1 << 30;
const MASK_MIDE: u32 = 1 << 30;
const FLTCON_ENABLE: u8 = 1 << 7;

// IOCON: GPIO0/GPIO1 as GPIO inputs
const IOCON_TRIS0: u32 = 1 << 0;
const IOCON_TRIS1: u32 = 1 << 1;
const IOCON_GPIO0: u32 = 1 << 16;
const IOCON_PM0: u32 = 1 << 24;
const IOCON_PM1: u32 = 1 << 25;

// ECCEN and DEDIE: single bit errors are corrected silently, double bit errors interrupt
const ECC_ENABLE: u32 = 0b101;

/// Driver errors. All of them leave the chip in an unknown state; re-run [`Mcp251xfd::init`].
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub enum Error {
    /// The oscillator or PLL never reported ready, usually because the chip is missing or CLKIN is not running.
    OscillatorTimeout,
    /// The chip did not enter the requested operating mode.
    ModeTimeout,
    /// A register read back differently from what was written.
    Readback,
    /// The requested bit rate cannot be generated from the 40 MHz SYSCLK.
    BitRate,
    /// The chip reported a system error.
    SystemError,
    /// The chip detected an uncorrectable RAM error.
    RamEcc,
}

/// Bit rates of the bus. `data` is `None` for a classic CAN bus.
#[derive(Clone, Copy, Debug)]
pub struct BitRates {
    pub nominal: u32,
    pub data: Option<u32>,
}

/// Nominal or data phase segment lengths, in time quanta, as the register fields expect (value - 1).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct Segments {
    tseg1: u32,
    tseg2: u32,
    sjw: u32,
}

/// Splits a bit of `tq` time quanta like the MCP251XFD library: phase segment 2 is about 20 % of the bit (sample
/// point near 80 %), SJW as large as possible.
const fn segments(tq: u32) -> Segments {
    let mut tseg2 = tq / 5;
    if tq % 5 > 2 {
        tseg2 += 1;
    }
    let tseg1 = tq - tseg2 - 1;
    let sjw = if tseg1 < tseg2 { tseg1 } else { tseg2 };
    Segments { tseg1: tseg1 - 1, tseg2: tseg2 - 1, sjw: sjw - 1 }
}

// The Talos internal bus must come out exactly as the C driver configures it.
const _: () = {
    let n = segments(SYSCLK_HZ / 1_000_000);
    assert!(n.tseg1 == 30 && n.tseg2 == 7 && n.sjw == 7);
    let d = segments(SYSCLK_HZ / 5_000_000);
    assert!(d.tseg1 == 4 && d.tseg2 == 1 && d.sjw == 1);
};

/// What [`Mcp251xfd::service_interrupts`] found.
#[derive(Clone, Copy, Default, Debug, defmt::Format)]
pub struct Events {
    /// Raw C1INT flags.
    pub flags: u32,
    /// The receive FIFO overflowed and frames were lost because they were not read in time.
    pub rx_overflow: bool,
}

/// Transmit/receive error counters and bus state (C1TREC).
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct ErrorState {
    pub rx_errors: u8,
    pub tx_errors: u8,
    pub bus_off: bool,
    pub error_passive: bool,
}

pub struct Mcp251xfd {
    spi: CanSpi,
    cs: Output<'static>,
    int: Input<'static>,
}

impl Mcp251xfd {
    /// `cs` must start high. `int` is the active low interrupt line (push-pull as configured here).
    pub fn new(spi: CanSpi, cs: Output<'static>, int: Input<'static>) -> Self {
        Mcp251xfd { spi, cs, int }
    }

    // ---------------------------------------------------------------- SPI access

    async fn transfer(&mut self, buf: &mut [u8]) {
        self.cs.set_low();
        // The embassy-rp SPI error type is uninhabited
        let _ = self.spi.transfer_in_place(buf).await;
        self.cs.set_high();
    }

    async fn write(&mut self, addr: u16, data: &[u8]) {
        let mut buf = [0u8; 2 + OBJ_MAX];
        buf[0] = (CMD_WRITE << 4) | ((addr >> 8) as u8 & 0x0F);
        buf[1] = addr as u8;
        buf[2..2 + data.len()].copy_from_slice(data);
        self.transfer(&mut buf[..2 + data.len()]).await;
    }

    async fn read(&mut self, addr: u16, out: &mut [u8]) {
        let mut buf = [0u8; 2 + OBJ_MAX];
        buf[0] = (CMD_READ << 4) | ((addr >> 8) as u8 & 0x0F);
        buf[1] = addr as u8;
        self.transfer(&mut buf[..2 + out.len()]).await;
        out.copy_from_slice(&buf[2..2 + out.len()]);
    }

    async fn read_u32(&mut self, addr: u16) -> u32 {
        let mut b = [0u8; 4];
        self.read(addr, &mut b).await;
        u32::from_le_bytes(b)
    }

    async fn write_u32(&mut self, addr: u16, value: u32) {
        self.write(addr, &value.to_le_bytes()).await;
    }

    async fn read_u8(&mut self, addr: u16) -> u8 {
        let mut b = [0u8; 1];
        self.read(addr, &mut b).await;
        b[0]
    }

    async fn write_u8(&mut self, addr: u16, value: u8) {
        self.write(addr, &[value]).await;
    }

    async fn poll_until(
        &mut self,
        addr: u16,
        mask: u32,
        value: u32,
        timeout: Duration,
        err: Error,
    ) -> Result<(), Error> {
        let deadline = Instant::now() + timeout;
        loop {
            if self.read_u32(addr).await & mask == value {
                return Ok(());
            }
            if Instant::now() > deadline {
                return Err(err);
            }
            Timer::after_micros(100).await;
        }
    }

    // ---------------------------------------------------------------- Setup

    /// Resets and fully configures the controller, then joins the bus.
    ///
    /// `filters` lists `(id, mask, extended)` acceptance filters routed to the receive FIFO; frames matching none
    /// of them are ignored.
    pub async fn init(&mut self, rates: BitRates, filters: &[(u32, u32, bool)]) -> Result<(), Error> {
        assert!(filters.len() <= 32);
        self.spi.set_frequency(SPI_SLOW_HZ);
        self.transfer(&mut [CMD_RESET << 4, 0]).await;
        Timer::after_millis(1).await;

        // Oscillator: wait for CLKIN, then lock the x10 PLL
        self.poll_until(reg::OSC, OSC_OSCRDY, OSC_OSCRDY, Duration::from_millis(10), Error::OscillatorTimeout).await?;
        self.write_u32(reg::OSC, OSC_PLLEN).await;
        let ready = OSC_PLLRDY | OSC_OSCRDY;
        self.poll_until(reg::OSC, ready, ready, Duration::from_millis(10), Error::OscillatorTimeout).await?;
        self.spi.set_frequency(SPI_FAST_HZ);

        // GPIO0 senses the termination jumper; also proves the bus works at full speed
        let iocon = IOCON_TRIS0 | IOCON_TRIS1 | IOCON_PM0 | IOCON_PM1;
        self.write_u32(reg::IOCON, iocon).await;
        if self.read_u32(reg::IOCON).await & 0xFF00_00FF != iocon & 0xFF00_00FF {
            return Err(Error::Readback);
        }

        // Clear RAM before enabling ECC, otherwise reading uninitialized RAM raises ECC errors
        let zeros = [0u8; 64];
        for offset in (0..RAM_SIZE).step_by(zeros.len()) {
            self.write(RAM_START + offset, &zeros).await;
        }
        self.write_u32(reg::ECCCON, ECC_ENABLE).await;

        // Still in configuration mode after reset
        let con = CON_ISOCRCEN
            | CON_PXEDIS
            | CON_WAKFIL
            | CON_WFT_T11
            | CON_STEF
            | CON_TXQEN
            | MODE_CONFIG << CON_REQOP_SHIFT;
        self.write_u32(reg::C1CON, con).await;

        // Bit timing, BRP = 1 so one time quantum per SYSCLK cycle
        let tq_of = |rate: u32| {
            if rate == 0 || !SYSCLK_HZ.is_multiple_of(rate) { Err(Error::BitRate) } else { Ok(SYSCLK_HZ / rate) }
        };
        let ntq = tq_of(rates.nominal)?;
        if !(4..=385).contains(&ntq) {
            return Err(Error::BitRate);
        }
        let n = segments(ntq);
        self.write_u32(reg::C1NBTCFG, n.tseg1 << 16 | n.tseg2 << 8 | n.sjw).await;
        if let Some(data) = rates.data {
            let dtq = tq_of(data)?;
            if !(4..=49).contains(&dtq) {
                return Err(Error::BitRate);
            }
            let d = segments(dtq);
            self.write_u32(reg::C1DBTCFG, d.tseg1 << 16 | d.tseg2 << 8 | d.sjw).await;
            // Automatic transmitter delay compensation with the offset at the data sample point
            let tdco = (d.tseg1 + 1).min(63);
            self.write_u32(reg::C1TDC, 0b10 << 16 | tdco << 8).await;
        }

        self.write_u32(reg::C1TEFCON, (TEF_DEPTH - 1) << FIFO_FSIZE_SHIFT | TEF_TEFNEIE).await;
        self.write_u32(reg::C1TXQCON, FIFO_PLSIZE_64 | (TXQ_DEPTH - 1) << FIFO_FSIZE_SHIFT | FIFO_TXAT_UNLIMITED).await;
        self.write_u32(
            reg::C1FIFOCON1,
            FIFO_PLSIZE_64 | (RX_DEPTH - 1) << FIFO_FSIZE_SHIFT | FIFO_RXOVIE | FIFO_TFNRFNIE,
        )
        .await;

        for (i, &(id, mask, extended)) in filters.iter().enumerate() {
            let i = i as u16;
            let (obj, msk) = if extended {
                (id_field(id, true) | FLT_EXIDE, id_field(mask, true) | MASK_MIDE)
            } else {
                (id_field(id, false), id_field(mask, false) | MASK_MIDE)
            };
            self.write_u32(reg::C1FLTOBJ0 + 8 * i, obj).await;
            self.write_u32(reg::C1MASK0 + 8 * i, msk).await;
            self.write_u8(reg::C1FLTCON0 + i, FLTCON_ENABLE | 1).await;
        }

        self.write_u32(reg::C1INT, INT_ENABLES).await;

        let mode = if rates.data.is_some() { MODE_CANFD } else { MODE_CAN20 };
        self.write_u8(reg::C1CON + 3, mode as u8).await;
        self.poll_until(
            reg::C1CON,
            0b111 << OPMOD_SHIFT,
            mode << OPMOD_SHIFT,
            Duration::from_millis(10),
            Error::ModeTimeout,
        )
        .await
    }

    // ---------------------------------------------------------------- Runtime

    /// Waits until the controller signals an interrupt.
    pub async fn wait_for_interrupt(&mut self) {
        self.int.wait_for_low().await;
    }

    /// True while the interrupt line is asserted.
    pub fn interrupt_pending(&self) -> bool {
        self.int.is_low()
    }

    /// Reads and clears error interrupts. Receive and TEF events clear themselves as their FIFOs are drained.
    ///
    /// Errors that leave the controller untrustworthy (system error, uncorrectable RAM error) are returned as `Err`.
    pub async fn service_interrupts(&mut self) -> Result<Events, Error> {
        let flags = self.read_u32(reg::C1INT).await & 0xFFFF;
        let mut events = Events { flags, ..Default::default() };

        if flags & INT_CLEARABLE != 0 {
            // Writing 1 leaves a flag alone, writing 0 clears it; the enables in the upper half are preserved
            self.write(reg::C1INT, &(!(flags & INT_CLEARABLE) as u16).to_le_bytes()).await;
        }
        if flags & INT_RXOVIF != 0 {
            self.write_u8(reg::C1FIFOSTA1, 0).await;
            events.rx_overflow = true;
        }
        if flags & INT_ECCIF != 0 {
            self.write_u32(reg::ECCSTAT, 0).await;
            return Err(Error::RamEcc);
        }
        if flags & INT_SERRIF != 0 {
            return Err(Error::SystemError);
        }
        Ok(events)
    }

    /// Queues a frame. Returns `false` without sending if the transmit queue is full.
    ///
    /// `fd` sends a CAN FD frame with bit rate switching; the payload is zero padded to a valid FD length. `seq`
    /// comes back in the transmit event FIFO once the frame is on the bus (see [`Self::read_tef`]).
    pub async fn transmit(&mut self, frame: &Frame, fd: bool, seq: u8) -> Result<bool, Error> {
        if self.read_u8(reg::C1TXQSTA).await & STA_NOT_EMPTY_OR_FULL == 0 {
            return Ok(false);
        }
        let ua = (self.read_u32(reg::C1TXQUA).await & 0xFFF) as u16;

        let data = frame.data();
        let (dlc, len) = if fd {
            let dlc = canmore::fd_len_to_dlc(data.len());
            (dlc, canmore::fd_dlc_to_len(dlc))
        } else {
            assert!(data.len() <= 8, "classic CAN frame too long");
            (data.len() as u8, data.len())
        };

        let mut obj = [0u8; OBJ_MAX];
        let mut flags = dlc as u32 | (seq as u32) << OBJ_SEQ_SHIFT;
        if frame.extended {
            flags |= OBJ_IDE;
        }
        if fd {
            flags |= OBJ_FDF | OBJ_BRS;
        }
        obj[0..4].copy_from_slice(&id_field(frame.id, frame.extended).to_le_bytes());
        obj[4..8].copy_from_slice(&flags.to_le_bytes());
        obj[OBJ_HEADER..OBJ_HEADER + data.len()].copy_from_slice(data);

        // RAM is written in whole words
        let total = (OBJ_HEADER + len).next_multiple_of(4);
        self.write(RAM_START + ua, &obj[..total]).await;
        self.write_u8(reg::C1TXQCON + 1, FIFO_UINC | FIFO_TXREQ).await;
        Ok(true)
    }

    /// Takes the next frame from the receive FIFO.
    pub async fn receive(&mut self) -> Result<Option<Frame>, Error> {
        if self.read_u8(reg::C1FIFOSTA1).await & STA_NOT_EMPTY_OR_FULL == 0 {
            return Ok(None);
        }
        let ua = (self.read_u32(reg::C1FIFOUA1).await & 0xFFF) as u16;
        let mut obj = [0u8; OBJ_MAX];
        self.read(RAM_START + ua, &mut obj).await;
        self.write_u8(reg::C1FIFOCON1 + 1, FIFO_UINC).await;

        let raw_id = u32::from_le_bytes(obj[0..4].try_into().unwrap());
        let flags = u32::from_le_bytes(obj[4..8].try_into().unwrap());
        let extended = flags & OBJ_IDE != 0;
        let dlc = (flags & 0xF) as u8;
        let len = if flags & OBJ_FDF != 0 { canmore::fd_dlc_to_len(dlc) } else { (dlc as usize).min(8) };
        let id = if extended { (raw_id & 0x7FF) << 18 | (raw_id >> 11) & 0x3FFFF } else { raw_id & 0x7FF };
        Ok(Frame::new(id, extended, &obj[OBJ_HEADER..OBJ_HEADER + len]))
    }

    /// Takes the next transmit event, returning the `seq` of a frame that made it onto the bus.
    pub async fn read_tef(&mut self) -> Result<Option<u8>, Error> {
        if self.read_u8(reg::C1TEFSTA).await & STA_NOT_EMPTY_OR_FULL == 0 {
            return Ok(None);
        }
        let ua = (self.read_u32(reg::C1TEFUA).await & 0xFFF) as u16;
        let mut obj = [0u8; OBJ_HEADER];
        self.read(RAM_START + ua, &mut obj).await;
        self.write_u8(reg::C1TEFCON + 1, FIFO_UINC).await;
        let flags = u32::from_le_bytes(obj[4..8].try_into().unwrap());
        Ok(Some(((flags >> OBJ_SEQ_SHIFT) & 0x7F) as u8))
    }

    /// Reads the termination sense input on GPIO0. `true` means the termination resistor is enabled.
    pub async fn termination_enabled(&mut self) -> bool {
        self.read_u32(reg::IOCON).await & IOCON_GPIO0 != 0
    }

    /// Reads the error counters.
    pub async fn error_state(&mut self) -> ErrorState {
        let trec = self.read_u32(reg::C1TREC).await;
        ErrorState {
            rx_errors: trec as u8,
            tx_errors: (trec >> 8) as u8,
            error_passive: trec & (0b11 << 19) != 0,
            bus_off: trec & (1 << 21) != 0,
        }
    }
}

/// Packs an identifier into the SID/EID layout used by message objects and filters.
fn id_field(id: u32, extended: bool) -> u32 {
    if extended { (id >> 18) & 0x7FF | (id & 0x3FFFF) << 11 } else { id & 0x7FF }
}
