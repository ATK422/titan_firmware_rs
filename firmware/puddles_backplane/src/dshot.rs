//! The eight thrusters over bidirectional DShot300, ported from `dshot.c` and `bidir_dshot.pio`.
//!
//! Each ESC has a PIO state machine running `bidir_dshot`: thrusters 1-4 on PIO0 state machines 0-3 (GPIO 25-22),
//! thrusters 5-8 on PIO1 (GPIO 21-18). The program sends one 16-bit frame per word pushed, then listens for the
//! ESC's 21-bit eRPM reply and pushes it back.
//!
//! [`dshot_task`] replaces the C hardware alarm: every [`TX_PERIOD_US`] it collects the replies to the previous
//! frames and sends the next ones. A command from `command/dshot` ([`command`]) restarts the state machines and
//! sends straight away, then publishes `state/thrusters/rpm`. The command state (timeouts, holdoffs, kill) is
//! [`puddles_core::dshot::Thrusters`], shared with the kill hook through [`stop_thrusters`].
//!
//! The C code read the replies in the PIO interrupt; here they wait in the RX FIFO until the next frame (or a
//! restart) collects them. Each ESC answers once per frame, so the 4-entry FIFO never overflows, and the missed
//! reply counting is the same.

use core::cell::RefCell;

use embassy_futures::select::{Either, select};
use embassy_rp::Peri;
use embassy_rp::gpio::Pull;
use embassy_rp::peripherals::{PIN_18, PIN_19, PIN_20, PIN_21, PIN_22, PIN_23, PIN_24, PIN_25, PIO0, PIO1};
use embassy_rp::pio::{
    Common, Config, Direction, Instance, LoadedProgram, Pio, PioPin, ShiftConfig, ShiftDirection, StateMachine,
    StatusSource,
};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use fixed::FixedU32;
use fixed::types::extra::U8;

use puddles_core::dshot::{
    self, NUM_THRUSTERS, PIO_CYCLES_PER_BIT, RATE_KHZ, TX_PERIOD_US, Telemetry, Thrusters, Update,
};
use titan_link::{DeviceToHost, puddles_backplane};
use titan_rp2040::{link, safety};

use crate::safety_interface::{FAULT_ROS_BAD_COMMAND, FAULT_THRUSTER_TIMEOUT};

static THRUSTERS: Mutex<CriticalSectionRawMutex, RefCell<Thrusters>> = Mutex::new(RefCell::new(Thrusters::new()));
static COMMANDS: Signal<CriticalSectionRawMutex, [i16; NUM_THRUSTERS]> = Signal::new();

const TX_PERIOD: Duration = Duration::from_micros(TX_PERIOD_US);

fn now_us() -> u64 {
    Instant::now().as_micros()
}

/// Queues a `command/dshot` command for [`dshot_task`]. A newer command replaces one not yet handled.
pub fn command(throttles: [i16; NUM_THRUSTERS]) {
    COMMANDS.signal(throttles);
}

/// Stops every thruster at the next frame (`dshot_stop_thrusters`). Called from the kill hook.
pub fn stop_thrusters() {
    THRUSTERS.lock(|t| t.borrow_mut().stop());
}

/// Reports the physical kill switch, for the 5 s wake-up holdoff (`dshot_notify_physical_kill_switch_change`).
pub fn notify_physical_kill_switch(asserting_kill: bool) {
    THRUSTERS.lock(|t| t.borrow_mut().notify_physical_kill_switch(now_us(), asserting_kill));
}

/// Entry points of `bidir_dshot` in one PIO block's instruction memory.
#[derive(Clone, Copy)]
struct Entry {
    start: u8,
    delayed_start: u8,
}

/// One ESC's state machine.
trait Output {
    /// Aborts whatever the state machine is doing and restarts it, holding off the first frame by `delay` loops
    /// of `delayed_start` if given (`bidir_dshot_reset`).
    fn reset(&mut self, entry: Entry, delay: Option<u32>);
    /// Queues a frame.
    fn send(&mut self, word: u32);
    /// Takes a received reply.
    fn reply(&mut self) -> Option<u32>;
}

impl<P: Instance, const SM: usize> Output for StateMachine<'static, P, SM> {
    fn reset(&mut self, entry: Entry, delay: Option<u32>) {
        self.set_enable(false);
        self.clear_fifos();
        self.restart();
        self.clkdiv_restart();
        // SAFETY: both addresses are entry points of the program this state machine runs
        match delay {
            Some(loops) => {
                unsafe { self.exec_jmp(entry.delayed_start) };
                // The FIFO was just cleared, so there is room
                self.tx().try_push(loops);
            }
            None => unsafe { self.exec_jmp(entry.start) },
        }
        self.set_enable(true);
    }

    fn send(&mut self, word: u32) {
        // The previous frame left the FIFO long ago (a frame takes 53 us, frames are 1.5 ms apart)
        if !self.tx().try_push(word) {
            defmt::warn!("DShot transmit FIFO full");
        }
    }

    fn reply(&mut self) -> Option<u32> {
        self.rx().try_pull()
    }
}

/// Builds `bidir_dshot`. It fills all 32 instructions of a PIO block, which is why each block runs its own copy.
macro_rules! bidir_dshot_program {
    () => {
        pio::pio_asm!(
            // Setting the PC here holds off the first frame for X+1 bit times, X being the first word pushed. Used
            // after an abrupt restart so a frame isn't sent while the ESC may still be answering the last one.
            "public delayed_start:",
            "    pull block",
            "    mov x, osr",
            "delay_loop:",
            "    nop [31]",
            "    jmp x-- delay_loop [7]",
            "public start:",
            // Transmit: the line idles high through the pull-up; a bit is asserted by driving it low
            "    set pins, 1",
            "    set pindirs, 1",
            // Block for data here rather than autopull, so the line isn't asserted while waiting
            "    pull block",
            // 40 cycles per bit: 15 asserted, 15 at the bit value, 10 released
            "tx_loop:",
            "    set pins, 0 [14]",
            "    out pins, 1 [14]",
            "    set pins, 1 [8]",
            "    jmp !osre tx_loop",
            // Receive the 21-bit reply, sent at 5/4 the bit rate
            "    set pindirs, 0",
            "    set x, 20",
            // Wait for the start of the reply, or give up when the next frame is queued
            "wait_rx:",
            "    mov y, status",
            "    jmp !y start",
            "    jmp pin wait_rx",
            "rx_from_edge:",
            "    nop [13]",
            "rx_immediate:",
            "    in pins, 1",
            "    jmp x-- clock_recovery",
            "    push noblock",
            "    jmp start",
            // Resynchronise to the ESC's clock on every level change; keep the timing when there's none
            "clock_recovery:",
            "    jmp pin clkrec_pin_high",
            "    set y, 11",
            "clkrec_low_loop:",
            "    jmp pin pin_low_edge_delay",
            "    jmp y-- clkrec_low_loop",
            "    jmp rx_immediate [3]",
            "pin_low_edge_delay:",
            "    jmp rx_from_edge",
            "clkrec_pin_high:",
            "    set y, 11",
            "clkrec_high_loop:",
            "    jmp pin clkrec_y_cmp",
            "    jmp rx_from_edge",
            "clkrec_y_cmp:",
            "    jmp y-- clkrec_high_loop",
            "    jmp rx_immediate [3]",
        )
    };
}

/// Loads `bidir_dshot` into a PIO block.
fn load_program<P: Instance>(common: &mut Common<'static, P>) -> (LoadedProgram<'static, P>, Entry) {
    let program = bidir_dshot_program!();
    let loaded = common.load_program(&program.program);
    let entry = Entry {
        start: loaded.origin + program.public_defines.start as u8,
        delayed_start: loaded.origin + program.public_defines.delayed_start as u8,
    };
    (loaded, entry)
}

/// Configures one state machine for the ESC on `pin` and starts it (`bidir_dshot_program_init`).
fn start_output<P: Instance, const SM: usize>(
    common: &mut Common<'static, P>,
    sm: &mut StateMachine<'static, P, SM>,
    program: &LoadedProgram<'static, P>,
    entry: Entry,
    pin: Peri<'static, impl PioPin>,
) {
    sm.set_enable(false);
    let mut pin = common.make_pio_pin(pin);
    // The line is pulled high whenever the state machine releases it
    pin.set_pull(Pull::Up);
    sm.set_pin_dirs(Direction::In, &[&pin]);

    let mut config = Config::default();
    config.use_program(program, &[]);
    config.set_out_pins(&[&pin]);
    config.set_set_pins(&[&pin]);
    config.set_in_pins(&[&pin]);
    config.set_jmp_pin(&pin);
    let divider = embassy_rp::clocks::clk_sys_freq() as f32 / (RATE_KHZ * 1000 * PIO_CYCLES_PER_BIT) as f32;
    config.clock_divider = FixedU32::<U8>::from_num(divider);
    config.shift_out = ShiftConfig { threshold: 16, direction: ShiftDirection::Left, auto_fill: false };
    config.shift_in = ShiftConfig { threshold: 21, direction: ShiftDirection::Left, auto_fill: false };
    // `mov y, status` reads all ones while the TX FIFO is empty
    config.status_sel = StatusSource::TxFifoLevel;
    config.status_n = 1;
    sm.set_config(&config);
    // SAFETY: `start` is an entry point of the loaded program
    unsafe { sm.exec_jmp(entry.start) };
    sm.set_enable(true);
    // The pin stays assigned to the PIO block when the handle goes
}

/// The thruster pins, in thruster order.
pub struct ThrusterPins {
    pub t1: Peri<'static, PIN_25>,
    pub t2: Peri<'static, PIN_24>,
    pub t3: Peri<'static, PIN_23>,
    pub t4: Peri<'static, PIN_22>,
    pub t5: Peri<'static, PIN_21>,
    pub t6: Peri<'static, PIN_20>,
    pub t7: Peri<'static, PIN_19>,
    pub t8: Peri<'static, PIN_18>,
}

/// The ESC state machines and their telemetry.
pub struct Dshot {
    pio0: Pio<'static, PIO0>,
    pio1: Pio<'static, PIO1>,
    entry0: Entry,
    entry1: Entry,
    telemetry: [Telemetry; NUM_THRUSTERS],
}

impl Dshot {
    /// Loads the program and starts every state machine (`dshot_init`). The ESCs see an idle line until
    /// [`dshot_task`] sends the first frame.
    pub fn new(mut pio0: Pio<'static, PIO0>, mut pio1: Pio<'static, PIO1>, pins: ThrusterPins) -> Self {
        let (program0, entry0) = load_program(&mut pio0.common);
        let (program1, entry1) = load_program(&mut pio1.common);
        let c0 = &mut pio0.common;
        start_output(c0, &mut pio0.sm0, &program0, entry0, pins.t1);
        start_output(c0, &mut pio0.sm1, &program0, entry0, pins.t2);
        start_output(c0, &mut pio0.sm2, &program0, entry0, pins.t3);
        start_output(c0, &mut pio0.sm3, &program0, entry0, pins.t4);
        let c1 = &mut pio1.common;
        start_output(c1, &mut pio1.sm0, &program1, entry1, pins.t5);
        start_output(c1, &mut pio1.sm1, &program1, entry1, pins.t6);
        start_output(c1, &mut pio1.sm2, &program1, entry1, pins.t7);
        start_output(c1, &mut pio1.sm3, &program1, entry1, pins.t8);
        // Dropping the program handles leaves the programs loaded
        Dshot { pio0, pio1, entry0, entry1, telemetry: [Telemetry::new(); NUM_THRUSTERS] }
    }

    /// The state machines in thruster order, with the program entry points of their block.
    fn outputs(&mut self) -> [(&mut dyn Output, Entry); NUM_THRUSTERS] {
        let (p0, p1) = (&mut self.pio0, &mut self.pio1);
        [
            (&mut p0.sm0, self.entry0),
            (&mut p0.sm1, self.entry0),
            (&mut p0.sm2, self.entry0),
            (&mut p0.sm3, self.entry0),
            (&mut p1.sm0, self.entry1),
            (&mut p1.sm1, self.entry1),
            (&mut p1.sm2, self.entry1),
            (&mut p1.sm3, self.entry1),
        ]
    }

    /// Collects the ESCs' replies to the last frames (`dshot_telem_cb`).
    fn collect_replies(&mut self) {
        let mut telemetry = self.telemetry;
        for ((output, _), t) in self.outputs().into_iter().zip(telemetry.iter_mut()) {
            while let Some(packet) = output.reply() {
                t.on_reply(packet);
            }
        }
        self.telemetry = telemetry;
    }

    /// Restarts every state machine so new values go out immediately, delaying the first frame if the last ones
    /// may still be on the line. `update` refuses commands until the line is free, so the delay is normally zero,
    /// as in C.
    fn restart(&mut self, next_frame_at: u64) {
        self.collect_replies();
        for (output, entry) in self.outputs() {
            let delay = next_frame_at.saturating_sub(now_us());
            output.reset(entry, (delay > 0).then(|| dshot::delayed_start_count(delay)));
        }
    }

    /// Sends a round of frames (`dshot_transmit_timer_cb`).
    fn transmit(&mut self) {
        self.collect_replies();
        let killed = safety::asserting_kill();
        let frames = THRUSTERS.lock(|t| t.borrow_mut().transmit(now_us(), killed));
        if frames.timed_out {
            defmt::warn!("Thruster commands timed out");
            safety::raise_fault(FAULT_THRUSTER_TIMEOUT, 0);
        }
        let mut telemetry = self.telemetry;
        for (((output, _), t), value) in self.outputs().into_iter().zip(telemetry.iter_mut()).zip(frames.values) {
            t.on_transmit();
            output.send(dshot::pio_word(dshot::encode_frame(value, false)));
        }
        self.telemetry = telemetry;
    }

    /// `state/thrusters/rpm` from the latest telemetry (`ros_send_rpm`).
    fn rpm_feedback(&self) -> puddles_backplane::RpmFeedback {
        let reversed = THRUSTERS.lock(|t| t.borrow().reversed());
        dshot::rpm_feedback(&self.telemetry, &reversed)
    }
}

/// Sends frames every [`TX_PERIOD_US`] and applies thruster commands.
#[embassy_executor::task]
pub async fn dshot_task(mut dshot: Dshot) -> ! {
    let mut next_frame = Instant::now();
    loop {
        match select(COMMANDS.wait(), Timer::at(next_frame)).await {
            Either::First(throttles) => {
                let killed = safety::asserting_kill();
                let now = now_us();
                let (update, next_frame_at) = THRUSTERS.lock(|t| {
                    let mut t = t.borrow_mut();
                    (t.update(now, killed, &throttles), t.next_frame_at())
                });
                match update {
                    Update::Ignored(why) => defmt::trace!("Thruster command ignored: {}", why),
                    Update::Applied | Update::Invalid(_) => {
                        if let Update::Invalid(value) = update {
                            defmt::warn!("Invalid thruster command: {}", value);
                            safety::raise_fault(FAULT_ROS_BAD_COMMAND, value as u32);
                        }
                        dshot.restart(next_frame_at);
                        dshot.transmit();
                        next_frame = Instant::now() + TX_PERIOD;
                    }
                }
                // C answers every received command with the RPM telemetry, whatever became of the command
                if link::is_connected() {
                    link::send(&DeviceToHost::PuddlesBackplane(puddles_backplane::DeviceToHost::Rpm(
                        dshot.rpm_feedback(),
                    )));
                }
            }
            Either::Second(()) => {
                dshot.transmit();
                // Like the C alarm, the next round is timed from this one
                next_frame = Instant::now() + TX_PERIOD;
            }
        }
    }
}
