//! Half duplex ("multidrop") UART on one PIO state machine, ported from `uart_multidrop.pio` and `async_uart.c`.
//!
//! The servo bus is a single wire. The state machine normally receives; [`MultidropUart::write`] makes it drive
//! the line for the given bytes, after which the program falls straight back to receiving so the servo's answer
//! isn't missed. The PIO program is the C one, instruction for instruction.
//!
//! Differences from C:
//! * The C driver moved bytes with two DMA channels and finished transfers in DMA, PIO and alarm interrupts. Here
//!   the transfers await embassy's interrupt driven PIO FIFO and IRQ futures; packets are at most 10 bytes, so
//!   DMA buys nothing and two DMA channels stay free.
//! * The receive timeout and frame error end the read through `select` instead of racing interrupt handlers.

use embassy_futures::select::{Either, select};
use embassy_rp::Peri;
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::gpio::{Level, Pull};
use embassy_rp::peripherals::{PIN_27, PIO0};
use embassy_rp::pio::program::{InstructionOperands, MovDestination, MovOperation, MovSource};
use embassy_rp::pio::{Config, Direction, Irq, IrqFlags, Pio, ShiftDirection, StateMachine};
use embassy_time::{Duration, with_timeout};
use fixed::FixedU32;
use fixed::types::extra::U8;

/// Why a receive failed (`enum async_uart_rx_err`).
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub enum RxError {
    /// Nothing arrived within the timeout.
    Timeout,
    /// Only part of the requested bytes arrived within the timeout.
    Incomplete,
    /// A byte had a bad stop bit (or the line was held low).
    FrameError,
    /// The receive FIFO overflowed, so bytes were lost.
    DataLost,
}

/// PIO IRQ flag raised when a transmission has completed (`irq_tx_compl_rel`, relative to state machine 0).
const IRQ_TX_COMPLETE: usize = 0;
/// PIO IRQ flag raised on a framing error (`irq_frame_error_rel`).
const IRQ_FRAME_ERROR: usize = 1;

/// The UART on PIO0 state machine 0.
pub struct MultidropUart<'d> {
    sm: StateMachine<'d, PIO0, 0>,
    tx_complete: Irq<'d, PIO0, 0>,
    frame_error: Irq<'d, PIO0, 1>,
    irq_flags: IrqFlags<'d, PIO0>,
    /// Absolute address of the `txdata` entry point.
    txdata: u8,
    timeout: Duration,
}

impl<'d> MultidropUart<'d> {
    /// Loads the program and starts receiving on `pin` (`async_uart_init`). `timeout` bounds each read.
    pub fn new(pio: Pio<'d, PIO0>, pin: Peri<'d, PIN_27>, baud: u32, timeout: Duration) -> Self {
        let Pio { mut common, irq_flags, irq0, irq1, mut sm0, .. } = pio;

        let program = pio::pio_asm!(
            ".side_set 1 opt",
            // OUT pin 0 and side-set pin 0 are both mapped to the line, as are IN pin 0 and the JMP pin
            "public rxdata:",
            "    set pindirs, 0",
            ".wrap_target",
            "rxcontinue:",
            "    wait 0 pin 0  [3]", // Stall until the start bit
            "    set x, 7      [7]", // Preload the bit counter, delay to the middle of the first data bit
            "rxloop:",
            "    in pins, 1",         // Shift a data bit into the ISR
            "    jmp x-- rxloop [6]", // 8 bits, 8 cycles each
            "    jmp pin good_stop",  // Check the stop bit
            "    irq 1 rel",          // Framing error or break: flag it,
            "    wait 1 pin 0",       // wait for the line to idle again
            "    jmp rxcontinue",     // and drop the byte
            "good_stop:",
            "    push noblock",
            ".wrap",
            // Entry point for transmitting: x holds the byte count - 1. Falls back to receiving when done.
            "public txdata:",
            "    pull           side 1 [1]", // Stop bit / idle while waiting for data
            "    set pindirs, 1 side 0 [7]", // Start bit, and drive the line
            "txloop:",
            "    out pins, 1",
            "    jmp !osre txloop      [6]",
            "    jmp x-- txdata side 1 [5]", // Stop bit, next byte
            "    irq 0 rel",                 // Transmission complete
            "    jmp rxdata     side 1",
        );
        let loaded = common.load_program(&program.program);

        let mut line = common.make_pio_pin(pin);
        line.set_pull(Pull::Up);
        sm0.set_pins(Level::High, &[&line]);
        sm0.set_pin_dirs(Direction::In, &[&line]);

        let mut cfg = Config::default();
        cfg.use_program(&loaded, &[&line]);
        cfg.set_in_pins(&[&line]);
        cfg.set_jmp_pin(&line);
        cfg.set_out_pins(&[&line]);
        cfg.set_set_pins(&[&line]);
        cfg.shift_out.direction = ShiftDirection::Right;
        cfg.shift_out.auto_fill = false;
        cfg.shift_out.threshold = 8;
        cfg.shift_in.direction = ShiftDirection::Right;
        cfg.shift_in.auto_fill = false;
        cfg.shift_in.threshold = 32;
        // 8 cycles per bit, with the fractional divider the C code got from its float division
        let divider_bits = (clk_sys_freq() as u64 * 256) / (8 * baud as u64);
        cfg.clock_divider = FixedU32::<U8>::from_bits(divider_bits as u32);
        // The program starts at rxdata, its first instruction
        sm0.set_config(&cfg);
        sm0.set_enable(true);

        MultidropUart {
            sm: sm0,
            tx_complete: irq0,
            frame_error: irq1,
            irq_flags,
            txdata: loaded.origin + program.public_defines.txdata as u8,
            timeout,
        }
    }

    /// Transmits `data`, returning once the last stop bit is out (`async_uart_write` with `preserve_read` false).
    /// Discards anything received before.
    pub async fn write(&mut self, data: &[u8]) {
        if data.is_empty() {
            return;
        }
        self.sm.set_enable(false);
        self.sm.restart();
        // Clear the receive state: FIFOs, overflow flag and a pending frame error
        self.sm.clear_fifos();
        let _ = self.sm.rx().stalled();
        self.irq_flags.clear(IRQ_FRAME_ERROR);

        // Load x with the byte count - 1 and jump to txdata
        const PULL: u16 = InstructionOperands::PULL { if_empty: false, block: true }.encode();
        const MOV_X_OSR: u16 =
            InstructionOperands::MOV { destination: MovDestination::X, op: MovOperation::None, source: MovSource::OSR }
                .encode();
        self.sm.tx().push(data.len() as u32 - 1);
        // SAFETY: the state machine is stopped and the instructions only touch OSR, x and the program counter
        unsafe {
            self.sm.exec_instr(PULL);
            self.sm.exec_instr(MOV_X_OSR);
            self.sm.exec_jmp(self.txdata);
        }
        self.irq_flags.clear(IRQ_TX_COMPLETE);
        self.sm.set_enable(true);

        for byte in data {
            self.sm.tx().wait_push(*byte as u32).await;
        }
        self.tx_complete.wait().await;
    }

    /// Receives exactly `buf.len()` bytes within the timeout (`async_uart_read`).
    pub async fn read(&mut self, buf: &mut [u8]) -> Result<(), RxError> {
        let mut received = 0;
        let rx = self.sm.rx();
        let receive = async {
            for byte in buf.iter_mut() {
                // Bytes arrive left justified in the 32-bit FIFO word
                *byte = (rx.wait_pull().await >> 24) as u8;
                received += 1;
            }
        };
        let result = with_timeout(self.timeout, select(receive, self.frame_error.wait())).await;
        let overflowed = self.sm.rx().stalled();
        match result {
            Ok(Either::Second(())) => Err(RxError::FrameError),
            _ if overflowed => Err(RxError::DataLost),
            Ok(Either::First(())) => Ok(()),
            Err(_) if received == 0 => Err(RxError::Timeout),
            Err(_) => Err(RxError::Incomplete),
        }
    }
}
