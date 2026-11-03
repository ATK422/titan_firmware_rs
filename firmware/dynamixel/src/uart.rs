//! Half duplex ("multidrop") UART on one pin, ported from `async_uart.c` and `uart_multidrop.pio`.
//!
//! One PIO state machine both transmits and receives on the Dynamixel TTL data line. After the last stop bit of a
//! transmission it releases the line and starts receiving straight away, so the servo's reply (which follows after
//! its return delay time) is never missed. Data moves through a DMA channel in both directions, as in C, so
//! executor latency only matters in the short gap between reading a reply's header and its body.

use embassy_futures::select::{Either3, select3};
use embassy_rp::Peri;
use embassy_rp::dma;
use embassy_rp::gpio::{Level, Pull};
use embassy_rp::pac;
use embassy_rp::peripherals::{PIO0, PIO1};
use embassy_rp::pio::{
    Common, Config, Direction, FifoJoin, Instance, Irq, IrqFlags, LoadedProgram, PioPin, ShiftDirection, StateMachine,
};
use embassy_time::{Duration, Timer};
use fixed::FixedU32;
use pio::{InstructionOperands, JmpCondition, MovDestination, MovOperation, MovSource};

/// Gives the driver the PIO block's registers for the FIFO debug flags, which embassy-rp doesn't expose.
pub trait PioRegs: Instance {
    fn regs() -> pac::pio::Pio;
}

impl PioRegs for PIO0 {
    fn regs() -> pac::pio::Pio {
        pac::PIO0
    }
}

impl PioRegs for PIO1 {
    fn regs() -> pac::pio::Pio {
        pac::PIO1
    }
}

/// The `uart_multidrop` program loaded into a PIO block.
pub struct MultidropProgram<'d, PIO: Instance> {
    program: LoadedProgram<'d, PIO>,
    /// Address of the `rxdata` entry point.
    rxdata: u8,
    /// Address of the `txdata` entry point.
    txdata: u8,
    /// IRQ flag offsets, relative to the state machine number.
    irq_tx_complete_rel: u8,
    irq_frame_error_rel: u8,
}

impl<'d, PIO: Instance> MultidropProgram<'d, PIO> {
    pub fn new(common: &mut Common<'d, PIO>) -> Self {
        // `uart_multidrop.pio` with one change: `in null 24` before the push moves the received byte from the top
        // to the bottom of the FIFO word, so 8-bit DMA reads pick it up (C read byte 3 of the FIFO register instead)
        let prg = pio::pio_asm!(
            r#"
            .side_set 1 opt

            .define public irq_tx_compl_rel 0
            .define public irq_frame_error_rel 1

            ; OUT pin 0 and side-set pin 0 are both mapped to UART TX pin.
            ; IN pin 0 and JMP pin are both mapped to the GPIO used as UART RX.

            public rxdata:
                set pindirs, 0
            .wrap_target
            rxcontinue:
                wait 0 pin 0  [3]            ; Stall until start bit is asserted
                set x, 7   [7]               ; Preload bit counter, then delay until halfway through
            rxloop:                          ; the first data bit (12 cycles incl wait, set).
                in pins, 1                   ; Shift data bit into ISR
                jmp x-- rxloop    [6]        ; Loop 8 times, each loop iteration is 8 cycles
                jmp pin good_stop            ; Check stop bit (should be high)

                irq irq_frame_error_rel rel  ; Either a framing error or a break. Set a sticky flag,
                wait 1 pin 0                 ; and wait for line to return to idle state.
                jmp rxcontinue               ; Don't push data if we didn't see good framing.

            good_stop:                       ; No delay before returning to start; a little slack is
                in null, 24                  ; Right-justify the byte for 8-bit reads
                push noblock                 ; important in case the TX clock is slightly too fast.
            .wrap

            ; Entry point to begin data transmission
            ; x must be loaded with the number of bytes to transmit
            ; After transmission is complete, it will begin receiving data
            public txdata:
                pull             side 1 [1]  ; Assert stop bit/stall with line in idle state waiting for data
                set pindirs, 1   side 0 [7]  ; Assert start bit, and set to output if not set already
            txloop:
                out pins, 1
                jmp !osre txloop        [6]  ; Loop for each bit to transmit
                jmp x-- txdata   side 1 [5]  ; Loop until all bytes have been sent, and assert stop bit
                irq irq_tx_compl_rel rel     ; Notify that transmit has been completed
                jmp rxdata       side 1      ; Transmit done, begin receiving data (also 8th tick of stop bit)
            "#
        );
        let program = common.load_program(&prg.program);
        let rxdata = program.origin + prg.public_defines.rxdata as u8;
        let txdata = program.origin + prg.public_defines.txdata as u8;
        MultidropProgram {
            program,
            rxdata,
            txdata,
            irq_tx_complete_rel: prg.public_defines.irq_tx_compl_rel as u8,
            irq_frame_error_rel: prg.public_defines.irq_frame_error_rel as u8,
        }
    }
}

/// Transmit failures (`enum async_uart_tx_err`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum TxError {
    /// The state machine never signalled the end of the transmission. The C driver had no such timeout.
    Timeout = 2,
}

/// Receive failures, numbered like `enum async_uart_rx_err`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum RxError {
    /// Nothing arrived before the timeout.
    Timeout = 2,
    /// Some, but not all, of the requested bytes arrived before the timeout.
    Incomplete = 3,
    /// A byte had a bad stop bit (or the line was held low).
    FrameError = 4,
    /// The receive FIFO overflowed, so bytes were dropped.
    DataLost = 5,
}

/// The half duplex UART. `TX_IRQ` and `ERR_IRQ` must be the PIO IRQ flags the program raises from state machine
/// `SM`: `(SM + 0) % 4` and `(SM + 1) % 4`.
pub struct HalfDuplexUart<'d, PIO: PioRegs, const SM: usize, const TX_IRQ: usize, const ERR_IRQ: usize> {
    sm: StateMachine<'d, PIO, SM>,
    irq_flags: IrqFlags<'d, PIO>,
    tx_irq: Irq<'d, PIO, TX_IRQ>,
    err_irq: Irq<'d, PIO, ERR_IRQ>,
    dma: dma::Channel<'d>,
    rxdata: u8,
    txdata: u8,
}

/// Longest a transmission may take before [`TxError::Timeout`]: the largest packet takes about 23 ms at 57600 baud.
const TX_TIMEOUT: Duration = Duration::from_millis(50);

impl<'d, PIO: PioRegs, const SM: usize, const TX_IRQ: usize, const ERR_IRQ: usize>
    HalfDuplexUart<'d, PIO, SM, TX_IRQ, ERR_IRQ>
{
    /// Sets up `sm` on `pin` at `baud` (`async_uart_init`). The line idles high with the pad's pull-up enabled.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        common: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        irq_flags: IrqFlags<'d, PIO>,
        tx_irq: Irq<'d, PIO, TX_IRQ>,
        err_irq: Irq<'d, PIO, ERR_IRQ>,
        pin: Peri<'d, impl PioPin>,
        dma: dma::Channel<'d>,
        program: &MultidropProgram<'d, PIO>,
        baud: u32,
    ) -> Self {
        assert_eq!(TX_IRQ, (SM + program.irq_tx_complete_rel as usize) % 4, "TX_IRQ does not match the state machine");
        assert_eq!(
            ERR_IRQ,
            (SM + program.irq_frame_error_rel as usize) % 4,
            "ERR_IRQ does not match the state machine"
        );

        let mut pin = common.make_pio_pin(pin);
        pin.set_pull(Pull::Up);
        sm.set_pins(Level::High, &[&pin]);
        sm.set_pin_dirs(Direction::In, &[&pin]);

        let mut cfg = Config::default();
        cfg.use_program(&program.program, &[&pin]);
        cfg.set_in_pins(&[&pin]);
        cfg.set_jmp_pin(&pin);
        cfg.set_out_pins(&[&pin]);
        cfg.set_set_pins(&[&pin]);
        cfg.shift_out.direction = ShiftDirection::Right;
        cfg.shift_out.auto_fill = false;
        cfg.shift_out.threshold = 8;
        cfg.shift_in.direction = ShiftDirection::Right;
        cfg.shift_in.auto_fill = false;
        cfg.shift_in.threshold = 32;
        cfg.fifo_join = FifoJoin::Duplex;
        // 8 cycles per bit; 24.8 fixed point divider, fraction kept like the C float divider
        let sysclk = embassy_rp::clocks::clk_sys_freq() as u64;
        cfg.clock_divider = FixedU32::from_bits(((sysclk << 8) / (8 * baud as u64)) as u32);
        // Starts at `rxdata` (the program origin)
        sm.set_config(&cfg);
        sm.set_enable(true);

        HalfDuplexUart { sm, irq_flags, tx_irq, err_irq, dma, rxdata: program.rxdata, txdata: program.txdata }
    }

    fn rx_stalled() -> bool {
        PIO::regs().fdebug().read().rxstall() & (1 << SM) != 0
    }

    fn clear_rx_stall() {
        PIO::regs().fdebug().write(|w| w.set_rxstall(1 << SM));
    }

    /// Transmits `data`, discarding anything received so far, and then listens for a reply (`async_uart_write`
    /// with `preserve_read` false). An empty `data` just resets the receiver.
    pub async fn write(&mut self, data: &[u8]) -> Result<(), TxError> {
        self.sm.set_enable(false);
        self.sm.restart();

        // Clear the receive state
        Self::clear_rx_stall();
        self.sm.clear_fifos();
        self.irq_flags.clear(ERR_IRQ);

        if data.is_empty() {
            let rxdata = InstructionOperands::JMP { condition: JmpCondition::Always, address: self.rxdata };
            // SAFETY: the state machine is stopped; jumping to the receive entry point is always valid
            unsafe { self.sm.exec_instr(rxdata.encode()) };
            self.sm.set_enable(true);
            return Ok(());
        }

        // Load x with the number of bytes to transmit, then enter `txdata`
        self.sm.tx().push(data.len() as u32 - 1);
        let pull = InstructionOperands::PULL { if_empty: false, block: true };
        let mov =
            InstructionOperands::MOV { destination: MovDestination::X, op: MovOperation::None, source: MovSource::OSR };
        let jmp = InstructionOperands::JMP { condition: JmpCondition::Always, address: self.txdata };
        // SAFETY: the state machine is stopped and the FIFO holds the word `pull` takes
        unsafe {
            self.sm.exec_instr(pull.encode());
            self.sm.exec_instr(mov.encode());
            self.sm.exec_instr(jmp.encode());
        }
        self.irq_flags.clear(TX_IRQ);
        self.sm.set_enable(true);

        let Self { sm, tx_irq, dma, .. } = self;
        let sent = async {
            sm.tx().dma_push(dma, data, false).await;
            tx_irq.wait().await;
        };
        embassy_time::with_timeout(TX_TIMEOUT, sent).await.map_err(|_| TxError::Timeout)
    }

    /// Receives exactly `buf.len()` bytes, giving up after `timeout` (`async_uart_read`). Call it promptly after
    /// [`HalfDuplexUart::write`]: the FIFO holds only 4 bytes until the DMA starts.
    pub async fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<(), RxError> {
        if buf.is_empty() {
            return Ok(());
        }
        let len = buf.len() as u32;
        let regs = self.dma.regs();
        let Self { sm, err_irq, dma, .. } = self;
        let transfer = sm.rx().dma_pull(dma, buf, false);
        let mut transfer = core::pin::pin!(transfer);
        let result = match select3(transfer.as_mut(), err_irq.wait(), Timer::after(timeout)).await {
            Either3::First(()) => Ok(()),
            Either3::Second(()) => Err(RxError::FrameError),
            Either3::Third(()) => {
                // Remaining count before the transfer is dropped (which aborts it)
                if regs.trans_count().read() == len { Err(RxError::Timeout) } else { Err(RxError::Incomplete) }
            }
        };
        if result != Err(RxError::FrameError) && Self::rx_stalled() {
            Self::clear_rx_stall();
            return Err(RxError::DataLost);
        }
        result
    }
}
