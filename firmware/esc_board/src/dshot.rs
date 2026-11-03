//! DShot transmit, eRPM receive and UART telemetry receive over PIO, ported from `dshot.c`.
//!
//! * PIO0 runs `bidir_dshot.pio` on all four state machines, one per ESC. [`Dshot`] belongs to the control loop on
//!   core 1, which polls it (no interrupts), like the C code did.
//! * PIO1 runs `uart_rx.pio` on all four state machines, one per ESC telemetry wire. [`uart_telemetry_task`] runs
//!   on core 0 and feeds the bytes into [`UART_TELEMETRY`]. In C this was the PIO1 interrupt handler.
//!
//! The `.pio` files are the C ones, unchanged, so the timing is identical: DShot600 with each bit 40 PIO cycles,
//! and the UART at 8 PIO cycles per bit.

use core::cell::RefCell;

use embassy_futures::select::{Either4, select4};
use embassy_rp::Peri;
use embassy_rp::gpio::Pull;
use embassy_rp::peripherals::{PIO0, PIO1};
use embassy_rp::pio::{
    Common, Config, Direction, FifoJoin, Instance, LoadedProgram, Pio, PioPin, ShiftConfig, ShiftDirection,
    StateMachine, StatusSource,
};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use fixed::FixedU32;
use fixed::types::extra::U8;

use esc_core::dshot::{TelemetrySchedule, encode_frame, pio_word, throttle_command};
use esc_core::telemetry::UartTelemetry;
use esc_core::{DSHOT_RATE_KHZ, NUM_THRUSTERS, TELEM_BAUD, erpm};

/// UART telemetry state, shared between the control loop (which requests telemetry) and the receive task.
pub static UART_TELEMETRY: Mutex<CriticalSectionRawMutex, RefCell<UartTelemetry>> =
    Mutex::new(RefCell::new(UartTelemetry::new()));

/// PIO clock divider for `cycles_per_second`, truncated to 16.8 fixed point like `sm_config_set_clkdiv`.
fn clock_divider(cycles_per_second: u32) -> FixedU32<U8> {
    let sys = embassy_rp::clocks::clk_sys_freq() as u64;
    FixedU32::from_bits((sys * 256 / cycles_per_second as u64) as u32)
}

/// Runs `$body` with `$sm` bound to state machine `$i` of `$self`.
macro_rules! with_sm {
    ($self:ident, $i:expr, |$sm:ident| $body:expr) => {
        match $i {
            0 => {
                let $sm = &mut $self.sm0;
                $body
            }
            1 => {
                let $sm = &mut $self.sm1;
                $body
            }
            2 => {
                let $sm = &mut $self.sm2;
                $body
            }
            _ => {
                let $sm = &mut $self.sm3;
                $body
            }
        }
    };
}

/// The four bidirectional DShot outputs (`DSHOT_PIO_BLOCK`).
pub struct Dshot {
    // Keeps the program memory and pin registrations alive
    _common: Common<'static, PIO0>,
    sm0: StateMachine<'static, PIO0, 0>,
    sm1: StateMachine<'static, PIO0, 1>,
    sm2: StateMachine<'static, PIO0, 2>,
    sm3: StateMachine<'static, PIO0, 3>,
    /// Address of the program's `start` label.
    start: u8,
    /// Direction each thruster was last commanded in, to sign the RPM feedback.
    rpm_reversed: [bool; NUM_THRUSTERS],
    telemetry_schedule: TelemetrySchedule,
}

impl Dshot {
    /// Loads `bidir_dshot` and starts the state machines. ESC1-4 are on GPIO 6, 4, 2 and 0 (state machines 0-3).
    pub fn new(
        pio: Pio<'static, PIO0>,
        esc1: Peri<'static, impl PioPin>,
        esc2: Peri<'static, impl PioPin>,
        esc3: Peri<'static, impl PioPin>,
        esc4: Peri<'static, impl PioPin>,
    ) -> Self {
        let Pio { mut common, mut sm0, mut sm1, mut sm2, mut sm3, .. } = pio;
        let program = embassy_rp::pio::program::pio_file!("src/bidir_dshot.pio", select_program("bidir_dshot"));
        let loaded = common.load_program(&program.program);
        let start = loaded.origin + program.public_defines.start as u8;
        init_dshot_sm(&mut common, &mut sm0, &loaded, start, esc1);
        init_dshot_sm(&mut common, &mut sm1, &loaded, start, esc2);
        init_dshot_sm(&mut common, &mut sm2, &loaded, start, esc3);
        init_dshot_sm(&mut common, &mut sm3, &loaded, start, esc4);
        Dshot {
            _common: common,
            sm0,
            sm1,
            sm2,
            sm3,
            start,
            rpm_reversed: [false; NUM_THRUSTERS],
            telemetry_schedule: TelemetrySchedule::new(),
        }
    }

    /// Restarts a state machine at `start` with empty FIFOs (`bidir_dshot_reset` without a delayed start). This
    /// avoids desynchronisation if an ESC is still answering when the next frame goes out.
    fn reset(&mut self, thruster: usize) {
        let start = self.start;
        with_sm!(self, thruster, |sm| {
            sm.set_enable(false);
            sm.clear_fifos();
            sm.restart();
            sm.clkdiv_restart();
            // SAFETY: the state machine is stopped, and `start` is the beginning of the loaded program
            unsafe { sm.exec_jmp(start) };
            sm.set_enable(true);
        })
    }

    /// Sends one frame to every ESC (`dshot_update_thrusters`). `throttle` is -999..=999 per thruster; anything
    /// else is sent as neutral. Every [`esc_core::TELEM_PACKET_DELAY`] frames one ESC is asked for UART telemetry.
    ///
    /// Frames must be at least [`esc_core::DSHOT_MIN_FRAME_TIME_US`] apart.
    pub fn send(&mut self, throttle: &[i16; NUM_THRUSTERS]) {
        let mut values = [0u16; NUM_THRUSTERS];
        for (i, &t) in throttle.iter().enumerate() {
            let command = throttle_command(t);
            values[i] = command.value;
            if let Some(reversed) = command.reversed {
                self.rpm_reversed[i] = reversed;
            }
        }
        let telemetry_request = self.telemetry_schedule.next_frame();

        for i in 0..NUM_THRUSTERS {
            self.reset(i);
        }
        for (i, &value) in values.iter().enumerate() {
            let request = telemetry_request == Some(i);
            if request {
                let reversed = self.rpm_reversed[i];
                UART_TELEMETRY.lock(|t| t.borrow_mut().prepare_for_rx(i, reversed));
            }
            let word = pio_word(encode_frame(value, request));
            // The FIFO was just cleared, so there is room
            with_sm!(self, i, |sm| sm.tx().push(word));
        }
    }

    /// True if an eRPM reply from `thruster` is waiting (`dshot_rpm_available`).
    pub fn rpm_available(&mut self, thruster: usize) -> bool {
        with_sm!(self, thruster, |sm| !sm.rx().empty())
    }

    /// Takes the waiting eRPM reply of `thruster` and decodes it into mechanical RPM (`dshot_get_rpm`). Returns
    /// `None` if nothing was waiting or the reply was corrupted.
    pub fn read_rpm(&mut self, thruster: usize) -> Option<i32> {
        let packet = with_sm!(self, thruster, |sm| sm.rx().try_pull())?;
        erpm::decode_rpm(packet, self.rpm_reversed[thruster])
    }
}

fn init_dshot_sm<const SM: usize>(
    common: &mut Common<'static, PIO0>,
    sm: &mut StateMachine<'static, PIO0, SM>,
    program: &LoadedProgram<'static, PIO0>,
    start: u8,
    pin: Peri<'static, impl PioPin>,
) {
    sm.set_enable(false);
    // Input with a pull-up, so the line idles high whenever the program isn't driving it
    let mut pin = common.make_pio_pin(pin);
    pin.set_pull(Pull::Up);
    sm.set_pin_dirs(Direction::In, &[&pin]);

    let mut config = Config::default();
    config.use_program(program, &[]);
    config.set_out_pins(&[&pin]);
    config.set_set_pins(&[&pin]);
    config.set_in_pins(&[&pin]);
    config.set_jmp_pin(&pin);
    // Each DShot bit is 40 PIO cycles
    config.clock_divider = clock_divider(DSHOT_RATE_KHZ * 1000 * 40);
    // TX: 16 bits, manual pull, MSB first. RX: 21 bits, manual push, shifted in from the right
    config.shift_out = ShiftConfig { threshold: 16, direction: ShiftDirection::Left, auto_fill: false };
    config.shift_in = ShiftConfig { threshold: 21, direction: ShiftDirection::Left, auto_fill: false };
    // `MOV Y, STATUS` reads all ones once the next frame is queued, so the receiver gives up waiting for a reply
    config.status_sel = StatusSource::TxFifoLevel;
    config.status_n = 1;
    sm.set_config(&config);
    // SAFETY: the state machine is stopped, and `start` is the program's normal entry point
    unsafe { sm.exec_jmp(start) };
    sm.set_enable(true);
}

/// The four ESC telemetry receivers (`DSHOT_TELEM_PIO_BLOCK`).
pub struct UartTelemetryRx {
    _common: Common<'static, PIO1>,
    sm0: StateMachine<'static, PIO1, 0>,
    sm1: StateMachine<'static, PIO1, 1>,
    sm2: StateMachine<'static, PIO1, 2>,
    sm3: StateMachine<'static, PIO1, 3>,
}

impl UartTelemetryRx {
    /// Loads `uart_rx` and starts the receivers. ESC1-4 telemetry is on GPIO 7, 5, 3 and 1 (state machines 0-3).
    pub fn new(
        pio: Pio<'static, PIO1>,
        esc1: Peri<'static, impl PioPin>,
        esc2: Peri<'static, impl PioPin>,
        esc3: Peri<'static, impl PioPin>,
        esc4: Peri<'static, impl PioPin>,
    ) -> Self {
        let Pio { mut common, mut sm0, mut sm1, mut sm2, mut sm3, .. } = pio;
        let program = embassy_rp::pio::program::pio_file!("src/uart_rx.pio", select_program("uart_rx"));
        let loaded = common.load_program(&program.program);
        init_uart_sm(&mut common, &mut sm0, &loaded, esc1);
        init_uart_sm(&mut common, &mut sm1, &loaded, esc2);
        init_uart_sm(&mut common, &mut sm2, &loaded, esc3);
        init_uart_sm(&mut common, &mut sm3, &loaded, esc4);
        UartTelemetryRx { _common: common, sm0, sm1, sm2, sm3 }
    }
}

fn init_uart_sm<PIO: Instance, const SM: usize>(
    common: &mut Common<'static, PIO>,
    sm: &mut StateMachine<'static, PIO, SM>,
    program: &LoadedProgram<'static, PIO>,
    pin: Peri<'static, impl PioPin>,
) {
    sm.set_enable(false);
    // No pull-up (make_pio_pin leaves the pulls off): the ESC pulls the line down while unpowered, and fighting it
    // leaves the line at an intermediate level that decodes as garbage
    let pin = common.make_pio_pin(pin);
    sm.set_pin_dirs(Direction::In, &[&pin]);

    let mut config = Config::default();
    config.use_program(program, &[]);
    config.set_in_pins(&[&pin]);
    config.set_jmp_pin(&pin);
    // Shift right, manual push: each byte ends up in the top 8 bits of the FIFO word
    config.shift_in = ShiftConfig { threshold: 32, direction: ShiftDirection::Right, auto_fill: false };
    config.fifo_join = FifoJoin::RxOnly;
    config.clock_divider = clock_divider(8 * TELEM_BAUD);
    sm.set_config(&config);
    sm.set_enable(true);
}

/// Receives ESC telemetry bytes and hands them to [`UART_TELEMETRY`] (`dshot_uart_telem_cb`).
#[embassy_executor::task]
pub async fn uart_telemetry_task(rx: UartTelemetryRx) -> ! {
    let UartTelemetryRx { _common, mut sm0, mut sm1, mut sm2, mut sm3 } = rx;
    loop {
        let (thruster, word) =
            match select4(sm0.rx().wait_pull(), sm1.rx().wait_pull(), sm2.rx().wait_pull(), sm3.rx().wait_pull()).await
            {
                Either4::First(w) => (0, w),
                Either4::Second(w) => (1, w),
                Either4::Third(w) => (2, w),
                Either4::Fourth(w) => (3, w),
            };
        UART_TELEMETRY.lock(|t| t.borrow_mut().receive(thruster, (word >> 24) as u8));
    }
}
