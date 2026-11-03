//! Half duplex single wire UART on a PIO state machine, ported from `async_uart.c` and `uart_multidrop.pio`.
//!
//! The PIO program idles in receive mode on the pin. A write restarts the state machine in its transmit routine
//! with the byte count in X; after the last stop bit it raises the "transmit complete" IRQ flag and falls back to
//! receiving, so a servo's answer is caught right after the command. Received bytes land in the top byte of each
//! RX FIFO word. Bytes move by DMA in both directions as in C; the C completion callbacks and timeout alarm become
//! awaits on the DMA transfer, the PIO IRQ flags and a timer.
//!
//! # Differences from C
//!
//! * The receive DMA moves whole FIFO words and keeps their top bytes, instead of reading byte 3 of the FIFO
//!   register, because embassy-rp's FIFO DMA reads from the register's base address.
//! * A transmit that doesn't complete within [`TX_TIMEOUT`] is reported; the C driver would have waited forever
//!   with its packet marked in flight.
//! * Busy errors can't happen: the one owner of the UART performs one transaction at a time.

use embassy_futures::select::{Either3, select3};
use embassy_rp::Peri;
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::dma;
use embassy_rp::gpio::{Level, Pull};
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::program::{InstructionOperands, JmpCondition, MovDestination, MovOperation, MovSource, pio_asm};
use embassy_rp::pio::{Common, Config, Direction, Irq, IrqFlags, PioPin, ShiftConfig, ShiftDirection, StateMachine};
use embassy_time::{Duration, Timer, with_timeout};
use fixed::FixedU32;
use fixed::types::extra::U8;

/// IRQ flag raised by the program when a transmit finishes (`irq_tx_compl_rel`, relative to SM 0).
const TX_DONE_IRQ: usize = 0;
/// IRQ flag raised on a framing error or break (`irq_frame_error_rel`, relative to SM 0).
const FRAME_ERROR_IRQ: usize = 1;
/// Longest a transmit may take before it counts as stuck: 10 bytes at 115200 baud take under 1 ms.
pub const TX_TIMEOUT: Duration = Duration::from_millis(10);
/// Longest receive supported.
pub const MAX_READ: usize = 16;

/// Why a transfer failed (`enum async_uart_rx_err`, plus the transmit timeout).
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub enum Error {
    /// Nothing was received in time (`ASYNC_UART_RX_TIMEOUT`).
    Timeout,
    /// Only part of the data arrived in time (`ASYNC_UART_RX_INCOMPLETE`).
    Incomplete,
    /// A byte had a bad stop bit, or the line was held low (`ASYNC_UART_RX_FRAME_ERROR`).
    Framing,
    /// The RX FIFO overflowed (`ASYNC_UART_RX_DATA_LOST`).
    DataLost,
    /// The transmit didn't finish within [`TX_TIMEOUT`].
    TxTimeout,
}

/// The UART on state machine 0 of PIO0.
pub struct MultidropUart<'d> {
    sm: StateMachine<'d, PIO0, 0>,
    tx_done: Irq<'d, PIO0, 0>,
    frame_error: Irq<'d, PIO0, 1>,
    irq_flags: IrqFlags<'d, PIO0>,
    tx_dma: dma::Channel<'d>,
    rx_dma: dma::Channel<'d>,
    /// Absolute addresses of the program's entry points.
    txdata: u8,
    rxdata: u8,
}

impl<'d> MultidropUart<'d> {
    /// Loads the program and starts receiving on `pin` (`async_uart_init` and `uart_multidrop_program_init`).
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        common: &mut Common<'d, PIO0>,
        mut sm: StateMachine<'d, PIO0, 0>,
        tx_done: Irq<'d, PIO0, 0>,
        frame_error: Irq<'d, PIO0, 1>,
        irq_flags: IrqFlags<'d, PIO0>,
        pin: Peri<'d, impl PioPin>,
        tx_dma: dma::Channel<'d>,
        rx_dma: dma::Channel<'d>,
        baud: u32,
    ) -> Self {
        let program = pio_asm!(
            ".side_set 1 opt",
            // OUT pin 0 and side-set pin 0 are both mapped to UART TX pin.
            // IN pin 0 and JMP pin are both mapped to the GPIO used as UART RX.
            "public rxdata:",
            "    set pindirs, 0",
            ".wrap_target",
            "rxcontinue:",
            "    wait 0 pin 0  [3]",     // Stall until start bit is asserted
            "    set x, 7   [7]",        // Preload bit counter, then delay until halfway through
            "rxloop:",                   // the first data bit (12 cycles incl wait, set).
            "    in pins, 1",            // Shift data bit into ISR
            "    jmp x-- rxloop    [6]", // Loop 8 times, each loop iteration is 8 cycles
            "    jmp pin good_stop",     // Check stop bit (should be high)
            "    irq 1 rel",             // Either a framing error or a break. Set a sticky flag,
            "    wait 1 pin 0",          // and wait for line to return to idle state.
            "    jmp rxcontinue",        // Don't push data if we didn't see good framing.
            "good_stop:",                // No delay before returning to start; a little slack is
            "    push noblock",          // important in case the TX clock is slightly too fast.
            ".wrap",
            // Entry point to begin data transmission. X must hold the number of bytes to transmit minus one.
            // After transmission is complete, it will begin receiving data.
            "public txdata:",
            "    pull             side 1 [1]", // Assert stop bit/stall with line idle waiting for data
            "    set pindirs, 1   side 0 [7]", // Assert start bit, and set to output if not set already
            "txloop:",
            "    out pins, 1",
            "    jmp !osre txloop        [6]", // Loop for each bit to transmit
            "    jmp x-- txdata   side 1 [5]", // Loop until all bytes have been sent, asserting the stop bit
            "    irq 0 rel",                   // Notify that transmit has been completed
            "    jmp rxdata       side 1",     // Transmit done, begin receiving data (also 8th tick of stop bit)
        );
        let loaded = common.load_program(&program.program);
        let txdata = loaded.origin + program.public_defines.txdata as u8;
        let rxdata = loaded.origin + program.public_defines.rxdata as u8;

        // Idle high, input, mapped to PIO and pulled up
        let mut pin = common.make_pio_pin(pin);
        pin.set_pull(Pull::Up);
        sm.set_pins(Level::High, &[&pin]);
        sm.set_pin_dirs(Direction::In, &[&pin]);

        let mut config = Config::default();
        config.use_program(&loaded, &[&pin]);
        config.set_in_pins(&[&pin]);
        config.set_jmp_pin(&pin);
        config.set_out_pins(&[&pin]);
        config.set_set_pins(&[&pin]);
        config.shift_out = ShiftConfig { threshold: 8, direction: ShiftDirection::Right, auto_fill: false };
        config.shift_in = ShiftConfig { threshold: 32, direction: ShiftDirection::Right, auto_fill: false };
        // 8 cycles per bit; the divider is truncated to 16.8 fixed point like sm_config_set_clkdiv
        let divider = clk_sys_freq() as f32 / (8 * baud) as f32;
        config.clock_divider = FixedU32::<U8>::from_bits((divider * 256.0) as u32);
        // Starts at the program origin, which is `rxdata`
        sm.set_config(&config);
        sm.set_enable(true);

        MultidropUart { sm, tx_done, frame_error, irq_flags, tx_dma, rx_dma, txdata, rxdata }
    }

    fn exec(&mut self, operands: InstructionOperands) {
        // SAFETY: only issued while the state machine is stopped and being redirected to an entry point
        unsafe { self.sm.exec_instr(operands.encode()) };
    }

    /// Transmits `data`, clearing any receive state, and returns once the last stop bit is out
    /// (`async_uart_write` with `preserve_read` false). The line is in receive mode afterwards.
    pub async fn write(&mut self, data: &[u8]) -> Result<(), Error> {
        self.sm.set_enable(false);
        self.sm.restart();
        // Clear the receive state: overflow flag, FIFOs, frame error
        let _ = self.sm.rx().stalled();
        self.sm.clear_fifos();
        self.irq_flags.clear(FRAME_ERROR_IRQ);

        let Some(last) = data.len().checked_sub(1) else {
            self.exec(InstructionOperands::JMP { condition: JmpCondition::Always, address: self.rxdata });
            self.sm.set_enable(true);
            return Ok(());
        };

        // Load X with the number of bytes to transmit, then start at txdata
        self.sm.tx().push(last as u32);
        self.exec(InstructionOperands::PULL { if_empty: false, block: true });
        self.exec(InstructionOperands::MOV {
            destination: MovDestination::X,
            op: MovOperation::None,
            source: MovSource::OSR,
        });
        self.exec(InstructionOperands::JMP { condition: JmpCondition::Always, address: self.txdata });
        self.irq_flags.clear(TX_DONE_IRQ);
        self.sm.set_enable(true);

        let Self { sm, tx_done, tx_dma, .. } = self;
        let transmit = async {
            sm.tx().dma_push(tx_dma, data, false).await;
            tx_done.wait().await;
        };
        with_timeout(TX_TIMEOUT, transmit).await.map_err(|_| Error::TxTimeout)
    }

    /// Receives exactly `buf.len()` bytes (at most [`MAX_READ`]) within `timeout` of the call
    /// (`async_uart_read`).
    pub async fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<(), Error> {
        assert!(buf.len() <= MAX_READ, "read too long");
        // A received word only has its top byte set, so untouched slots are recognisable
        const EMPTY: u32 = u32::MAX;
        let mut words = [EMPTY; MAX_READ];
        let words = &mut words[..buf.len()];

        let Self { sm, frame_error, rx_dma, .. } = self;
        let result = select3(sm.rx().dma_pull(rx_dma, words, false), frame_error.wait(), Timer::after(timeout)).await;
        match result {
            // The C driver didn't check for overflow after a frame error
            Either3::Second(()) => return Err(Error::Framing),
            Either3::First(()) | Either3::Third(()) if sm.rx().stalled() => return Err(Error::DataLost),
            Either3::Third(()) if words[0] == EMPTY => return Err(Error::Timeout),
            Either3::Third(()) => return Err(Error::Incomplete),
            Either3::First(()) => {}
        }
        for (byte, word) in buf.iter_mut().zip(words.iter()) {
            *byte = (word >> 24) as u8;
        }
        Ok(())
    }
}
