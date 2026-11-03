//! SMBus controller on a PIO state machine, ported from `pio_smbus.c` and `i2c.pio`.
//!
//! The BMS bus has SDA on GPIO 7 and SCL on GPIO 6, the reverse of what the RP2040 I2C pin mux offers, so it runs on
//! PIO like in C. The PIO program is the pico-examples I2C program: each 10 bit TX FIFO word clocks one byte out
//! (or in, when sending 0xFF) and handles the ACK, stretching the clock while the gauge holds SCL low. START, STOP
//! and repeated START are made by executing `set pindirs` instructions directly on the state machine.
//!
//! The C version busy-waited on the FIFOs and the NAK flag. Here a transfer awaits the RX FIFO, the NAK IRQ flag or
//! the SMBus timeout, whichever comes first. Only the short START/STOP hold times are busy waits.
//!
//! Only the packet error checked transfers the gauge driver uses are ported (word read, block read, block write);
//! the C file also had non-PEC variants and a PEC word write that nothing called.

use embassy_futures::select::{Either3, select3};
use embassy_futures::yield_now;
use embassy_rp::Peri;
use embassy_rp::gpio::{Level, Pull};
use embassy_rp::pac;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::program::{
    InstructionOperands, JmpCondition, MovDestination, MovOperation, MovSource, WaitSource, pio_asm,
};
use embassy_rp::pio::{
    Common, Config, Direction, Irq, LoadedProgram, Pin, PioPin, ShiftConfig, ShiftDirection, StateMachine,
};
use embassy_time::{Duration, Instant, Timer, block_for};
use fixed::FixedU32;
use fixed::types::extra::U8;

use smart_battery_core::smbus::{SmbusError, pec_update};

/// Cumulative clock low extension a slave may use (`SMBUS_SEXT_TIME_MS`).
const SEXT_TIME: Duration = Duration::from_millis(25);
/// Longest the bus may be held low before every device resets (`SMBUS_TIMEOUT_MS`).
const TIMEOUT_MS: u64 = 35;
/// Grace period after a timeout before giving up on the STOP (`SMBUS_TIMEOUT_BUFFER_MS`).
const TIMEOUT_BUFFER_MS: u64 = 5;
/// START/STOP hold time; SMBus asks for 4.7 us (`SMBUS_START_STOP_SEQ_DURATION_US`).
const START_STOP_HOLD: Duration = Duration::from_micros(5);
/// SCL frequency: the program takes 32 PIO cycles per bit.
const SCL_HZ: u32 = 100_000;

// TX word layout: | 9: final | 8:1 data | 0: NAK |, shifted to the top of the FIFO word
const CMD_WIDTH: u32 = 10;
const FINAL_LSB: u32 = 9;
const DATA_LSB: u32 = 1;
const NAK_LSB: u32 = 0;
/// Receive and ACK.
const XFER_TX_ACK: u32 = 0;
/// Transmit; a NAK halts the state machine and raises its IRQ flag.
const XFER_REQUIRE_ACK: u32 = 1 << NAK_LSB;
/// Transmit or receive without acknowledging; a NAK is expected.
const XFER_ALLOW_NAK: u32 = (1 << NAK_LSB) | (1 << FINAL_LSB);

/// Offset of SCL from SDA in the `wait` instructions: 31 means SCL is the pin before SDA.
const SCL_OFFSET: u8 = 31;

/// The `set_scl_sda` instruction table.
#[derive(Clone, Copy)]
enum PinSet {
    Scl0Sda0 = 0,
    Scl0Sda1 = 1,
    Scl1Sda0 = 2,
    Scl1Sda1 = 3,
}

/// The loaded I2C program.
pub struct SmbusProgram<'d> {
    prg: LoadedProgram<'d, PIO0>,
    entry_point: u8,
    pin_sets: [u16; 4],
}

impl<'d> SmbusProgram<'d> {
    pub fn new(common: &mut Common<'d, PIO0>) -> Self {
        let i2c = pio_asm!(
            r#"
            .side_set 1 opt pindirs

            do_nack:
                jmp y-- push_byte          ; Continue if NAK was expected
                irq wait 0 rel             ; Otherwise stop, ask for help
                mov isr, null              ; Purge the ISR before resuming

            public entry_point:
            .wrap_target
                out y, 1                   ; Unpack the NAK ignore bit
            do_byte:
                set x, 7                   ; Loop 8 times
            bitloop:
                out pindirs, 1         [7] ; Serialise write data (all-ones if reading)
                nop             side 1 [2] ; SCL rising edge
                wait 1 pin, 31         [4] ; Allow clock to be stretched
                in pins, 1             [7] ; Sample read data in middle of SCL pulse
                jmp x-- bitloop side 0 [7] ; SCL falling edge

                ; Handle ACK pulse
                out pindirs, 1         [7] ; On reads, we provide the ACK
                nop             side 1 [7] ; SCL rising edge
                wait 1 pin, 31         [7] ; Allow clock to be stretched
                jmp pin do_nack side 0 [2] ; Test SDA for ACK/NAK, fall through if ACK
            push_byte:
                push                       ; Push the received byte once it was acknowledged
            .wrap
            "#
        );
        // Not loaded: a table of instructions executed directly to make START/STOP/repeated START
        let set_scl_sda = pio_asm!(
            r#"
            .side_set 1 opt
                set pindirs, 0 side 0 [7] ; SCL = 0, SDA = 0
                set pindirs, 1 side 0 [7] ; SCL = 0, SDA = 1
                set pindirs, 0 side 1 [7] ; SCL = 1, SDA = 0
                set pindirs, 1 side 1 [7] ; SCL = 1, SDA = 1
            "#
        );
        let code = &set_scl_sda.program.code;
        SmbusProgram {
            prg: common.load_program(&i2c.program),
            entry_point: i2c.public_defines.entry_point as u8,
            pin_sets: [code[0], code[1], code[2], code[3]],
        }
    }
}

/// State machine running the controller (`BQ_PIO_SM`). Its NAK flag is IRQ 0 (`irq wait 0 rel`).
const SM: usize = 0;

/// An SMBus controller on PIO0 state machine 0 (`BQ_PIO_INST`, `BQ_PIO_SM`).
pub struct PioSmbus<'d> {
    sm: StateMachine<'d, PIO0, SM>,
    irq: Irq<'d, PIO0, SM>,
    sda: u8,
    scl: u8,
    /// Absolute address of the program's `entry_point` (the wrap target).
    entry: u8,
    pin_sets: [u16; 4],
}

impl<'d> PioSmbus<'d> {
    /// Sets up the pins and starts the state machine (`i2c_program_init`). SCL must be the pin just below SDA.
    pub fn new(
        common: &mut Common<'d, PIO0>,
        mut sm: StateMachine<'d, PIO0, SM>,
        irq: Irq<'d, PIO0, SM>,
        sda: Peri<'d, impl PioPin>,
        scl: Peri<'d, impl PioPin>,
        program: &SmbusProgram<'d>,
    ) -> Self {
        let mut sda: Pin<'d, PIO0> = common.make_pio_pin(sda);
        let mut scl: Pin<'d, PIO0> = common.make_pio_pin(scl);
        assert_eq!(sda.pin(), scl.pin() + 1, "SCL must be the pin below SDA");
        sda.set_pull(Pull::Up);
        scl.set_pull(Pull::Up);

        let mut cfg = Config::default();
        cfg.use_program(&program.prg, &[&scl]);
        cfg.set_out_pins(&[&sda]);
        cfg.set_set_pins(&[&sda]);
        cfg.set_in_pins(&[&sda]);
        cfg.set_jmp_pin(&sda);
        cfg.shift_out = ShiftConfig { auto_fill: true, direction: ShiftDirection::Left, threshold: CMD_WIDTH as u8 };
        cfg.shift_in = ShiftConfig { auto_fill: false, direction: ShiftDirection::Left, threshold: 8 };
        let sys = embassy_rp::clocks::clk_sys_freq() as u64;
        cfg.clock_divider = FixedU32::<U8>::from_bits((sys * 256 / (32 * SCL_HZ as u64)) as u32);
        sm.set_config(&cfg);

        // Avoid glitching the bus while connecting: the pins only ever drive low, when PIO asserts its (inverted)
        // output enable, and are pulled up otherwise
        sm.set_pins(Level::High, &[&sda, &scl]);
        sm.set_pin_dirs(Direction::Out, &[&sda, &scl]);
        sda.set_output_enable_inversion(true);
        scl.set_output_enable_inversion(true);
        sm.set_pins(Level::Low, &[&sda, &scl]);

        let entry = program.prg.origin + program.entry_point;
        let mut smbus = PioSmbus { sda: sda.pin(), scl: scl.pin(), sm, irq, entry, pin_sets: program.pin_sets };
        smbus.irq_flag_clear();
        smbus.jump_to_entry();
        smbus.sm.set_enable(true);
        smbus
    }

    fn irq_flag_clear(&mut self) {
        pac::PIO0.irq().write(|w| w.set_irq(1 << SM));
    }

    fn jump_to_entry(&mut self) {
        let jmp = InstructionOperands::JMP { condition: JmpCondition::Always, address: self.entry }.encode();
        // SAFETY: the state machine is idle or being reset
        unsafe { self.sm.exec_instr(jmp) };
    }

    fn exec_stalled() -> bool {
        pac::PIO0.sm(SM).execctrl().read().exec_stalled()
    }

    // ========================================
    // Low level control
    // ========================================

    /// Both lines must be high before a START (`pio_smbus_check_bus_inactive`).
    fn check_bus_inactive(&self) -> Result<(), SmbusError> {
        let high = |pin: u8| pac::IO_BANK0.gpio(pin as usize).status().read().infrompad();
        if high(self.sda) && high(self.scl) { Ok(()) } else { Err(SmbusError::BusStuckLow) }
    }

    /// Puts the state machine back at its entry point after a timeout (`pio_smbus_fix_after_timeout`).
    fn fix_after_timeout(&mut self) {
        self.sm.set_enable(false);
        self.sm.restart();
        self.sm.clear_fifos();
        self.jump_to_entry();
        self.sm.set_enable(true);
    }

    /// Executes one of the non-stalling pin set instructions and holds it (`pio_smbus_exec_pinset`).
    fn pinset(&mut self, set: PinSet) {
        // SAFETY: the state machine is idle, waiting for its next TX word; `set` never stalls
        unsafe { self.sm.exec_instr(self.pin_sets[set as usize]) };
        block_for(START_STOP_HOLD);
    }

    /// Waits for a clock stretching slave to release SCL (`pio_smbus_wait_for_clock_high_timeout`).
    async fn wait_for_clock_high(&mut self, timeout: Duration) -> Result<(), SmbusError> {
        let wait =
            InstructionOperands::WAIT { polarity: 1, source: WaitSource::PIN, index: SCL_OFFSET, relative: false }
                .encode();
        // SAFETY: the state machine is idle; the wait is cancelled below if it doesn't finish
        unsafe { self.sm.exec_instr(wait) };
        let deadline = Instant::now() + timeout;
        while Self::exec_stalled() {
            if Instant::now() >= deadline {
                let nop = InstructionOperands::MOV {
                    destination: MovDestination::Y,
                    op: MovOperation::None,
                    source: MovSource::Y,
                }
                .encode();
                // SAFETY: replaces the stalled wait
                unsafe { self.sm.exec_instr(nop) };
                return Err(SmbusError::Timeout);
            }
            yield_now().await;
        }
        Ok(())
    }

    /// Clocks one byte through the program and returns what was on SDA (`pio_smbus_do_xfer`).
    async fn xfer(&mut self, data: u8, mode: u32) -> Result<u8, SmbusError> {
        let word = (mode | (data as u32) << DATA_LSB) << (32 - CMD_WIDTH);
        if !self.sm.tx().try_push(word) {
            // The state machine consumes every word before the previous transfer returns
            return Err(SmbusError::Timeout);
        }
        let (rx, _) = self.sm.rx_tx();
        match select3(rx.wait_pull(), self.irq.wait(), Timer::after(SEXT_TIME)).await {
            Either3::First(rx) => Ok(rx as u8),
            // The flag is cleared by the IRQ future, which releases the state machine
            Either3::Second(()) => Err(SmbusError::Nak),
            Either3::Third(()) => {
                self.fix_after_timeout();
                Err(SmbusError::Timeout)
            }
        }
    }

    /// Transmits a byte and checks that it made it onto the bus (`pio_smbus_tx_byte`).
    async fn tx_byte(&mut self, data: u8) -> Result<(), SmbusError> {
        let echoed = self.xfer(data, XFER_REQUIRE_ACK).await?;
        // A stuck SDA still looks like an ACK, but the data won't match
        if echoed != data { Err(SmbusError::ArbitrationLost) } else { Ok(()) }
    }

    /// Transmits an address byte, reporting a NAK as `nak_err`.
    async fn tx_addr(&mut self, data: u8, nak_err: SmbusError) -> Result<(), SmbusError> {
        self.tx_byte(data).await.map_err(|e| if e == SmbusError::Nak { nak_err } else { e })
    }

    /// Receives a byte, acknowledging it unless it's the last (`pio_smbus_rx_byte`).
    async fn rx_byte(&mut self, ack: bool) -> Result<u8, SmbusError> {
        self.xfer(0xFF, if ack { XFER_TX_ACK } else { XFER_ALLOW_NAK }).await
    }

    async fn start(&mut self) -> Result<(), SmbusError> {
        self.check_bus_inactive()?;
        self.pinset(PinSet::Scl1Sda0);
        self.pinset(PinSet::Scl0Sda0);
        Ok(())
    }

    async fn repeated_start(&mut self) -> Result<(), SmbusError> {
        self.pinset(PinSet::Scl0Sda1);
        self.pinset(PinSet::Scl1Sda1);
        self.wait_for_clock_high(SEXT_TIME).await?;
        self.pinset(PinSet::Scl1Sda0);
        self.pinset(PinSet::Scl0Sda0);
        Ok(())
    }

    async fn stop(&mut self) -> Result<(), SmbusError> {
        self.pinset(PinSet::Scl0Sda0);
        self.pinset(PinSet::Scl1Sda0);
        self.wait_for_clock_high(SEXT_TIME).await?;
        self.pinset(PinSet::Scl1Sda1);
        Ok(())
    }

    /// Tries to leave the bus idle after a failure (`pio_smbus_cleanup_failed_operation`).
    async fn cleanup(&mut self, err: SmbusError) {
        let mut timeout = TIMEOUT_MS + TIMEOUT_BUFFER_MS;
        if err == SmbusError::Timeout {
            // Part of the bus timeout already passed
            timeout -= SEXT_TIME.as_millis();
        }
        self.pinset(PinSet::Scl0Sda0);
        self.pinset(PinSet::Scl1Sda0);
        // Errors are ignored: if the bus is still stuck the next START reports it
        let _ = self.wait_for_clock_high(Duration::from_millis(timeout)).await;
        self.pinset(PinSet::Scl1Sda1);
    }

    /// Runs `op`, cleaning the bus up if it fails.
    async fn with_cleanup<T>(&mut self, result: Result<T, SmbusError>) -> Result<T, SmbusError> {
        if let Err(e) = result {
            self.cleanup(e).await;
        }
        result
    }

    // ========================================
    // SMBus transfers with packet error checking
    // ========================================

    /// Read Word with PEC (`pio_smbus_word_read_pec`).
    pub async fn word_read_pec(&mut self, addr: u8, cmd: u8) -> Result<u16, SmbusError> {
        let result = self.word_read_pec_inner(addr, cmd).await;
        match result {
            // The STOP was sent; a bad checksum needs no cleanup
            Err(SmbusError::BadChecksum) => result,
            _ => self.with_cleanup(result).await,
        }
    }

    async fn word_read_pec_inner(&mut self, addr: u8, cmd: u8) -> Result<u16, SmbusError> {
        let mut crc = 0;
        self.start().await?;
        self.tx_addr(addr << 1, SmbusError::AddrNak).await?;
        crc = pec_update(crc, addr << 1);
        self.tx_byte(cmd).await?;
        crc = pec_update(crc, cmd);

        self.repeated_start().await?;
        self.tx_addr(addr << 1 | 1, SmbusError::AddrRestartNak).await?;
        crc = pec_update(crc, addr << 1 | 1);
        let lo = self.rx_byte(true).await?;
        crc = pec_update(crc, lo);
        let hi = self.rx_byte(true).await?;
        crc = pec_update(crc, hi);
        // The PEC byte is always NAKed by the controller
        let crc_rx = self.rx_byte(false).await?;
        self.stop().await?;

        if crc_rx != crc {
            return Err(SmbusError::BadChecksum);
        }
        Ok(u16::from_le_bytes([lo, hi]))
    }

    /// Block Read with PEC into `buf`, returning the number of bytes (`pio_smbus_block_read_pec`).
    pub async fn block_read_pec(&mut self, addr: u8, cmd: u8, buf: &mut [u8]) -> Result<usize, SmbusError> {
        let result = self.block_read_pec_inner(addr, cmd, buf).await;
        match result {
            Err(SmbusError::BadChecksum) => result,
            _ => self.with_cleanup(result).await,
        }
    }

    async fn block_read_pec_inner(&mut self, addr: u8, cmd: u8, buf: &mut [u8]) -> Result<usize, SmbusError> {
        let mut crc = 0;
        self.start().await?;
        self.tx_addr(addr << 1, SmbusError::AddrNak).await?;
        crc = pec_update(crc, addr << 1);
        self.tx_byte(cmd).await?;
        crc = pec_update(crc, cmd);

        self.repeated_start().await?;
        self.tx_addr(addr << 1 | 1, SmbusError::AddrRestartNak).await?;
        crc = pec_update(crc, addr << 1 | 1);

        let len = self.rx_byte(true).await?;
        crc = pec_update(crc, len);
        let len = len as usize;
        if len > buf.len() {
            // NAK the next byte so the gauge stops; the transfer fails either way
            let _ = self.rx_byte(false).await;
            return Err(SmbusError::BufTooSmall);
        }
        for byte in &mut buf[..len] {
            *byte = self.rx_byte(true).await?;
            crc = pec_update(crc, *byte);
        }
        let crc_rx = self.rx_byte(false).await?;
        self.stop().await?;

        if crc_rx != crc {
            return Err(SmbusError::BadChecksum);
        }
        Ok(len)
    }

    /// Block Write with PEC (`pio_smbus_block_write_pec`).
    pub async fn block_write_pec(&mut self, addr: u8, cmd: u8, data: &[u8]) -> Result<(), SmbusError> {
        if data.len() > u8::MAX as usize {
            return Err(SmbusError::BufTooLarge);
        }
        let result = self.block_write_pec_inner(addr, cmd, data).await;
        self.with_cleanup(result).await
    }

    async fn block_write_pec_inner(&mut self, addr: u8, cmd: u8, data: &[u8]) -> Result<(), SmbusError> {
        let mut crc = 0;
        self.start().await?;
        self.tx_addr(addr << 1, SmbusError::AddrNak).await?;
        crc = pec_update(crc, addr << 1);
        for &byte in [cmd, data.len() as u8].iter().chain(data) {
            self.tx_byte(byte).await?;
            crc = pec_update(crc, byte);
        }
        // The gauge NAKs a bad PEC byte
        self.tx_addr(crc, SmbusError::BadChecksum).await?;
        self.stop().await
    }
}
