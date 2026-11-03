//! Original actuators (`actuators_v1`): coil gun torpedoes, solenoid marker droppers and a timed DC motor claw.
//!
//! * **Torpedoes**: a capacitor bank (charge level on GPIO 28, ADC 2) fires three coils in sequence through the
//!   selected torpedo's switch. A PIO state machine times the coils to the microsecond (`torpedo.pio`) and raises
//!   IRQ 0 when done. The arm line enables charging and the drain line discharges the bank when disarmed.
//! * **Droppers**: a pin per dropper, held high for `dropper_active_timing_ms`.
//! * **Claw**: an H-bridge driven open or closed for `claw_timing_ms`.
//!
//! Timings come from the bridge's ROS parameters ([`Command::SetTorpedoTiming`] and friends). The hardware lives
//! in [`STATE`] behind a critical section mutex, so a kill ([`disarm`]) can stop everything at any time. The C
//! alarm and interrupt callbacks are small tasks here.

use core::cell::{Cell, RefCell};

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_rp::Peri;
use embassy_rp::adc::{self, Adc};
use embassy_rp::gpio::{Level, Output, Pull};
use embassy_rp::interrupt::typelevel::Binding;
use embassy_rp::pac;
use embassy_rp::peripherals::{
    ADC, PIN_0, PIN_2, PIN_5, PIN_6, PIN_7, PIN_8, PIN_11, PIN_23, PIN_24, PIN_25, PIN_26, PIN_27, PIN_28, PIO0,
};
use embassy_rp::pio::{Common, Config, Direction, FifoJoin, Irq, Pin, StateMachine};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Ticker, Timer};
use fixed::FixedU32;
use pio::{InstructionOperands, JmpCondition, MovDestination, MovOperation, MovSource, SideSet};

use actuator_core::v1::{self as core_v1, ChargeMonitor, NUM_DROPPERS, NUM_TIMINGS, NUM_TORPEDOES, TorpedoTimings};
use titan_link::DeviceToHost;
use titan_link::actuator::{self, ActuatorError, Command, Status, Variant};
use titan_rp2040::{link, safety};

use crate::FAULT_ACTUATOR_FAILURE;

// Pin levels
const ARM_LEVEL_ARMED: Level = Level::Low;
const ARM_LEVEL_DISARMED: Level = Level::High;
const DRAIN_LEVEL_NODRAIN: Level = Level::High;
const DRAIN_LEVEL_DRAIN: Level = Level::Low;
const TORP_SEL_LEVEL_ON: Level = Level::High;
const TORP_SEL_LEVEL_OFF: Level = Level::Low;
const DROPPER_LEVEL_ON: Level = Level::High;
const DROPPER_LEVEL_OFF: Level = Level::Low;

/// Charge level samples per second (the C ADC ran free at 48 MHz / 48000).
const ADC_PERIOD: Duration = Duration::from_millis(1);

/// The peripherals the V1 actuators use.
pub struct Hardware<'a, I> {
    pub common: &'a mut Common<'static, PIO0>,
    pub sm: StateMachine<'static, PIO0, 1>,
    /// PIO IRQ 0, raised by the coil sequencer when it finishes.
    pub fired_irq: Irq<'static, PIO0, 0>,
    /// The coils must be consecutive pins with coil 3 lowest, for side-set.
    pub coil3: Peri<'static, PIN_25>,
    pub coil2: Peri<'static, PIN_26>,
    pub coil1: Peri<'static, PIN_27>,
    pub select1: Peri<'static, PIN_7>,
    pub select2: Peri<'static, PIN_6>,
    pub arm: Peri<'static, PIN_11>,
    pub drain: Peri<'static, PIN_8>,
    pub adc: Peri<'static, ADC>,
    pub charge_level: Peri<'static, PIN_28>,
    pub irqs: I,
    pub dropper1: Peri<'static, PIN_24>,
    pub dropper2: Peri<'static, PIN_23>,
    pub claw_enable: Peri<'static, PIN_0>,
    pub claw_phase: Peri<'static, PIN_2>,
    pub claw_mode2: Peri<'static, PIN_5>,
}

/// The coil sequencer state machine (`torpedo.pio`).
struct Sequencer {
    sm: StateMachine<'static, PIO0, 1>,
    origin: u8,
    coils: [Pin<'static, PIO0>; 3],
}

const SIDE_SET: SideSet = SideSet::new(false, 3, false);

impl Sequencer {
    /// Executes `operands` with `side 0b111`, so the coils stay idle (side-set isn't optional in this program).
    fn exec_idle(&mut self, operands: InstructionOperands) {
        let instr = pio::Instruction { operands, delay: 0, side_set: Some(0b111) };
        // SAFETY: only used with the state machine stopped, for a nop or a jump to the program start
        unsafe { self.sm.exec_instr(instr.encode(SIDE_SET)) };
    }

    /// `nop side 0b111`: all coil outputs to their idle level, then make them outputs.
    fn idle_coils(&mut self) {
        let nop =
            InstructionOperands::MOV { destination: MovDestination::Y, op: MovOperation::None, source: MovSource::Y };
        self.exec_idle(nop);
        let [c3, c2, c1] = &self.coils;
        self.sm.set_pin_dirs(Direction::Out, &[c3, c2, c1]);
    }

    /// Loads the five timings and starts the sequence (`torpedo_fire_sequence`).
    fn fire(&mut self, timings: &[u16; NUM_TIMINGS]) -> bool {
        if self.sm.is_enabled() || self.sm.tx().level() != 0 {
            return false;
        }
        for &t in timings {
            self.sm.tx().push(t as u32);
        }
        self.sm.set_enable(true);
        true
    }

    /// Stops and rewinds the state machine (`torpedo_reset`).
    ///
    /// The C code handed the coil pins to SIO (at their initial low level) while it reset the state machine. Here
    /// they stay with the PIO, which holds their last side-set value until the `nop side 0b111` puts them back to
    /// idle; the torpedo select lines are already off whenever this runs.
    fn reset(&mut self) {
        self.sm.set_enable(false);
        pac::PIO0.irq().write(|w| w.set_irq(1 << TORPEDO_IRQ));
        self.sm.clear_fifos();
        pac::PIO0.fdebug().write(|w| {
            w.set_txover(1 << 1);
            w.set_rxunder(1 << 1);
            w.set_txstall(1 << 1);
            w.set_rxstall(1 << 1);
        });
        self.sm.restart();
        self.sm.clkdiv_restart();
        self.exec_idle(InstructionOperands::JMP { condition: JmpCondition::Always, address: self.origin });
        self.idle_coils();
    }
}

/// IRQ flag the sequencer raises when done (`torpedo_irq_num`).
const TORPEDO_IRQ: u8 = 0;

struct Torpedoes {
    sequencer: Sequencer,
    select: [Output<'static>; NUM_TORPEDOES],
    arm: Output<'static>,
    drain: Output<'static>,
    timings: TorpedoTimings,
    next: u8,
    firing: bool,
    charge: ChargeMonitor,
}

impl Torpedoes {
    fn clear_select(&mut self) {
        for pin in &mut self.select {
            pin.set_level(TORP_SEL_LEVEL_OFF);
        }
    }

    /// `torpedo_arm`: start charging.
    fn arm(&mut self, armed: bool) -> Result<(), ActuatorError> {
        if !armed {
            return Err(ActuatorError::Killed);
        }
        self.next = 0;
        self.drain.set_level(DRAIN_LEVEL_NODRAIN);
        self.arm.set_level(ARM_LEVEL_ARMED);
        defmt::info!("Arming torpedos");
        Ok(())
    }

    /// `torpedo_safety_disable`: stop any firing, disarm and drain.
    fn safety_disable(&mut self) {
        self.clear_select();
        self.arm.set_level(ARM_LEVEL_DISARMED);
        self.drain.set_level(DRAIN_LEVEL_DRAIN);
        if self.firing {
            self.firing = false;
            self.sequencer.reset();
            // We told ROS it fired, so report that it was cut short
            safety::raise_fault(FAULT_ACTUATOR_FAILURE, 0);
        }
    }

    /// `torpedo_fired_callback`.
    fn fired(&mut self) {
        self.clear_select();
        if self.firing {
            self.firing = false;
            self.next += 1;
            self.sequencer.reset();
        }
    }

    fn charged(&mut self) -> bool {
        self.charge.check(CHARGE.lock(Cell::get))
    }

    /// `torpedo_fire`.
    fn fire(&mut self, killed: bool, armed: bool) -> Result<(), ActuatorError> {
        if killed {
            return Err(ActuatorError::KillSwitchRemoved);
        }
        if !armed {
            return Err(ActuatorError::NotArmed);
        }
        if self.next as usize >= NUM_TORPEDOES {
            return Err(ActuatorError::AllTorpedosFired);
        }
        if self.firing {
            return Err(ActuatorError::AlreadyFiring);
        }
        if !self.charged() {
            return Err(ActuatorError::NotCharged);
        }
        let index = self.next as usize;
        let Some(timings) = self.timings.sequence(index) else {
            return Err(ActuatorError::MissingTimings);
        };

        self.firing = true;
        self.select[index].set_level(TORP_SEL_LEVEL_ON);
        if !self.sequencer.fire(&timings) {
            self.firing = false;
            self.select[index].set_level(TORP_SEL_LEVEL_OFF);
            self.sequencer.reset();
            safety::raise_fault(FAULT_ACTUATOR_FAILURE, 0);
            return Err(ActuatorError::PioError);
        }
        defmt::info!("Firing torpedo {}", index + 1);
        Ok(())
    }

    /// `torpedo_notify_reload`.
    fn notify_reload(&mut self) -> Result<(), ActuatorError> {
        if self.firing {
            return Err(ActuatorError::FiringInProgress);
        }
        defmt::info!("Resetting torpedo fire count");
        self.next = 0;
        Ok(())
    }
}

struct Droppers {
    pins: [Output<'static>; NUM_DROPPERS],
    /// 0 until the timing parameter arrives.
    active_time_ms: u16,
    next: u8,
    /// A dropper timer is running. A kill stops the dropper but not the timer, so this stays set until it expires
    /// (see `dropper_alarm_active` in C).
    alarm_active: bool,
}

impl Droppers {
    fn clear(&mut self) {
        for pin in &mut self.pins {
            pin.set_level(DROPPER_LEVEL_OFF);
        }
    }

    /// `dropper_safety_disable`.
    fn safety_disable(&mut self) {
        self.clear();
        if self.alarm_active {
            safety::raise_fault(FAULT_ACTUATOR_FAILURE, 0);
        }
    }

    /// `dropper_drop_marker`: returns how long to hold the dropper.
    fn drop_marker(&mut self, killed: bool, armed: bool) -> Result<Duration, ActuatorError> {
        if self.active_time_ms == 0 {
            return Err(ActuatorError::MissingTimings);
        }
        // The C code set this message but carried on and indexed past the pin table; stop here instead
        if self.next as usize >= NUM_DROPPERS {
            return Err(ActuatorError::AllDroppersFired);
        }
        if killed {
            return Err(ActuatorError::KillSwitchPulled);
        }
        if !armed {
            return Err(ActuatorError::NotArmed);
        }
        let index = self.next as usize;
        self.alarm_active = true;
        self.pins[index].set_level(DROPPER_LEVEL_ON);
        defmt::info!("Dropping marker {}", index + 1);
        Ok(Duration::from_millis(self.active_time_ms as u64))
    }

    /// `dropper_finish_callback`.
    fn finish(&mut self) {
        self.clear();
        self.next += 1;
        self.alarm_active = false;
    }

    /// `dropper_notify_reload`.
    fn notify_reload(&mut self) -> Result<(), ActuatorError> {
        if self.alarm_active {
            return Err(ActuatorError::DropperActive);
        }
        defmt::info!("Resetting dropper fire count");
        self.next = 0;
        Ok(())
    }
}

struct Claw {
    logic: core_v1::Claw,
    enable: Output<'static>,
    direction: Output<'static>,
    _mode2: Output<'static>,
    /// Bumped by every move and stop, so a stale timer does nothing.
    generation: u32,
}

impl Claw {
    /// `claw_driver_stop`.
    fn stop(&mut self) {
        self.direction.set_low();
        self.enable.set_low();
    }

    /// Starts a move if the logic asks for one.
    fn start(&mut self, mv: Option<core_v1::ClawMove>) {
        let Some(mv) = mv else { return };
        defmt::info!("{} claw", if mv.forward == core_v1::OPEN_DIRECTION_IS_FORWARD { "Opening" } else { "Closing" });
        self.generation = self.generation.wrapping_add(1);
        self.direction.set_level(mv.forward.into());
        self.enable.set_high();
        CLAW_TIMER.signal((self.generation, Instant::now() + Duration::from_millis(mv.duration_ms as u64)));
    }

    /// `claw_safety_disable`.
    fn safety_disable(&mut self) {
        if self.logic.busy() {
            self.generation = self.generation.wrapping_add(1);
        }
        self.logic.safety_disable();
        self.stop();
    }
}

struct V1 {
    armed: bool,
    torpedoes: Torpedoes,
    droppers: Droppers,
    claw: Claw,
}

impl V1 {
    /// `actuators_disarm`.
    fn disarm(&mut self) {
        self.armed = false;
        self.droppers.safety_disable();
        self.torpedoes.safety_disable();
        self.claw.safety_disable();
    }
}

static STATE: Mutex<CriticalSectionRawMutex, RefCell<Option<V1>>> = Mutex::new(RefCell::new(None));
/// Latest capacitor bank voltage.
static CHARGE: Mutex<CriticalSectionRawMutex, Cell<f32>> = Mutex::new(Cell::new(0.0));
/// Deadlines of running dropper timers.
static DROPPER_TIMERS: Channel<CriticalSectionRawMutex, Instant, NUM_DROPPERS> = Channel::new();
/// The claw move timer: generation and deadline.
static CLAW_TIMER: Signal<CriticalSectionRawMutex, (u32, Instant)> = Signal::new();

fn with_state<R>(f: impl FnOnce(&mut V1) -> R) -> Option<R> {
    STATE.lock(|s| s.borrow_mut().as_mut().map(f))
}

/// Sets up the pins, the coil sequencer and the charge monitor (`actuators_initialize`).
pub fn init<I>(spawner: Spawner, hw: Hardware<'_, I>)
where
    I: Binding<embassy_rp::interrupt::typelevel::ADC_IRQ_FIFO, adc::InterruptHandler> + Copy + 'static,
{
    // Torpedo pins: disarmed and draining
    let arm = Output::new(hw.arm, ARM_LEVEL_DISARMED);
    let drain = Output::new(hw.drain, DRAIN_LEVEL_DRAIN);
    let select = [Output::new(hw.select1, TORP_SEL_LEVEL_OFF), Output::new(hw.select2, TORP_SEL_LEVEL_OFF)];

    let adc = Adc::new(hw.adc, hw.irqs, adc::Config::default());
    let charge_level = adc::Channel::new_pin(hw.charge_level, Pull::None);
    spawner.spawn(charge_task(adc, charge_level).unwrap());

    let common = hw.common;
    let prg = pio::pio_asm!(
        r#"
        .side_set 3

        ; There are 5 coil timings that must be sent
        .define public num_timings 5
        .define public irq_num     0

        ; Coil 1
            pull                    side 0b111
            mov x, osr              side 0b011   ; Note side set pin 3 is coil 1
        delay_coil1_on:
            jmp x-- delay_coil1_on  side 0b011

            pull                    side 0b111
            mov x, osr              side 0b111
        delay_coil1_off:
            jmp x-- delay_coil1_off side 0b111

        ; Coil 2
            pull                    side 0b111
            mov x, osr              side 0b101
        delay_coil2_on:
            jmp x-- delay_coil2_on  side 0b101

            pull                    side 0b111
            mov x, osr              side 0b111
        delay_coil2_off:
            jmp x-- delay_coil2_off side 0b111

        ; Coil 3
            pull                    side 0b111
            mov x, osr              side 0b110
        delay_coil3_on:
            jmp x-- delay_coil3_on  side 0b110

        ; Done
            irq irq_num             side 0b111
        loop:
            jmp loop                side 0b111
        "#
    );
    const _: () = assert!(NUM_TIMINGS <= 8, "more timings than the joined FIFO holds");
    assert_eq!(prg.public_defines.num_timings as usize, NUM_TIMINGS);
    assert_eq!(prg.public_defines.irq_num as u8, TORPEDO_IRQ);
    let program = common.load_program(&prg.program);

    let coils = [common.make_pio_pin(hw.coil3), common.make_pio_pin(hw.coil2), common.make_pio_pin(hw.coil1)];
    let mut cfg = Config::default();
    {
        let [c3, c2, c1] = &coils;
        cfg.use_program(&program, &[c3, c2, c1]);
    }
    // 1 MHz, integer divider only (a fractional one adds too much jitter)
    let sysclk = embassy_rp::clocks::clk_sys_freq();
    assert!(sysclk.is_multiple_of(1_000_000), "system clock must be a whole number of MHz");
    cfg.clock_divider = FixedU32::from_num(sysclk / 1_000_000);
    cfg.fifo_join = FifoJoin::TxOnly;
    let mut sm = hw.sm;
    sm.set_config(&cfg);
    pac::PIO0.irq().write(|w| w.set_irq(1 << TORPEDO_IRQ));
    let mut sequencer = Sequencer { sm, origin: program.origin, coils };
    sequencer.idle_coils();
    spawner.spawn(torpedo_fired_task(hw.fired_irq).unwrap());

    let torpedoes = Torpedoes {
        sequencer,
        select,
        arm,
        drain,
        timings: TorpedoTimings::new(),
        next: 0,
        firing: false,
        charge: ChargeMonitor::new(),
    };

    let droppers = Droppers {
        pins: [Output::new(hw.dropper1, DROPPER_LEVEL_OFF), Output::new(hw.dropper2, DROPPER_LEVEL_OFF)],
        active_time_ms: 0,
        next: 0,
        alarm_active: false,
    };
    spawner.spawn(dropper_task().unwrap());

    let claw = Claw {
        logic: core_v1::Claw::new(),
        enable: Output::new(hw.claw_enable, Level::Low),
        direction: Output::new(hw.claw_phase, Level::Low),
        _mode2: Output::new(hw.claw_mode2, Level::Low),
        generation: 0,
    };
    spawner.spawn(claw_task().unwrap());

    STATE.lock(|s| *s.borrow_mut() = Some(V1 { armed: false, torpedoes, droppers, claw }));
}

/// Samples the capacitor bank voltage (`torpedo_adc_cb`).
#[embassy_executor::task]
async fn charge_task(mut adc: Adc<'static, adc::Async>, mut channel: adc::Channel<'static>) -> ! {
    let mut ticker = Ticker::every(ADC_PERIOD);
    loop {
        ticker.next().await;
        match adc.read(&mut channel).await {
            Ok(raw) => CHARGE.lock(|c| c.set(core_v1::charge_volts(raw))),
            Err(err) => defmt::warn!("Charge level read failed: {}", err),
        }
    }
}

/// Runs the end of a torpedo shot when the sequencer raises its IRQ.
#[embassy_executor::task]
async fn torpedo_fired_task(mut irq: Irq<'static, PIO0, 0>) -> ! {
    loop {
        irq.wait().await;
        with_state(|s| s.torpedoes.fired());
    }
}

/// Ends each drop when its time is up (`dropper_finish_callback`).
#[embassy_executor::task]
async fn dropper_task() -> ! {
    loop {
        let deadline = DROPPER_TIMERS.receive().await;
        Timer::at(deadline).await;
        with_state(|s| s.droppers.finish());
    }
}

/// Stops the claw when its move time is up (`claw_finish_callback`).
#[embassy_executor::task]
async fn claw_task() -> ! {
    let (mut generation, mut deadline) = CLAW_TIMER.wait().await;
    loop {
        match select(Timer::at(deadline), CLAW_TIMER.wait()).await {
            Either::First(()) => {
                with_state(|s| {
                    if s.claw.generation == generation {
                        s.claw.stop();
                        s.claw.logic.finish();
                    }
                });
                (generation, deadline) = CLAW_TIMER.wait().await;
            }
            Either::Second(next) => (generation, deadline) = next,
        }
    }
}

/// Stops everything (`actuators_disarm`). Safe in `on_kill`; does nothing before [`init`].
pub fn disarm() {
    with_state(V1::disarm);
}

/// A rejected timing parameter: the C parameter callback raised `FAULT_ROS_ERROR` and refused the change.
fn timing_rejected() -> Result<(), ActuatorError> {
    safety::raise_fault(safety::FAULT_ROS_ERROR, 0);
    Err(ActuatorError::TimingRejected)
}

/// Carries out a service call or parameter change.
pub fn execute(command: Command) -> Result<(), ActuatorError> {
    let killed = safety::asserting_kill();
    with_state(|s| match command {
        Command::Arm => {
            if s.armed {
                return Err(ActuatorError::AlreadyArmed);
            }
            if killed {
                return Err(ActuatorError::KillSwitchRemoved);
            }
            s.armed = true;
            defmt::info!("Arming actuators");
            // As in C, a failure here leaves the actuators armed
            s.torpedoes.arm(s.armed)?;
            s.droppers.notify_reload()
        }
        Command::Disarm => {
            s.disarm();
            Ok(())
        }
        Command::FireTorpedo => s.torpedoes.fire(killed, s.armed),
        Command::DropMarker => {
            let hold = s.droppers.drop_marker(killed, s.armed)?;
            if DROPPER_TIMERS.try_send(Instant::now() + hold).is_err() {
                // Only two droppers, so at most two timers; can't happen, but never leave a dropper on
                s.droppers.finish();
            }
            Ok(())
        }
        Command::NotifyReload => s.torpedoes.notify_reload().and_then(|()| s.droppers.notify_reload()),
        Command::Claw { open } => {
            let mv = if open { s.claw.logic.open(killed) } else { s.claw.logic.close(killed) }?;
            s.claw.start(mv);
            Ok(())
        }
        Command::SetClawTiming { ms } => {
            if !s.claw.logic.set_timings(ms, ms) {
                return timing_rejected();
            }
            defmt::info!("Setting claw timing (open {} ms, close {} ms)", ms, ms);
            Ok(())
        }
        Command::SetDropperTiming { ms } => {
            if ms == 0 || s.armed {
                return timing_rejected();
            }
            defmt::info!("Setting dropper timings to {} ms", ms);
            s.droppers.active_time_ms = ms;
            Ok(())
        }
        Command::SetTorpedoTiming { timing, us } => {
            let armed = s.armed;
            let timings = &mut s.torpedoes.timings;
            if !(timings.set(1, timing, us, armed) && timings.set(2, timing, us, armed)) {
                return timing_rejected();
            }
            defmt::info!("Setting torpedo timing {} to {} us", timing, us);
            Ok(())
        }
        Command::TorpedoMarkerMoveHome | Command::TorpedoMarkerSetHome | Command::ClawSetClosedPosition => {
            Err(ActuatorError::Unsupported)
        }
    })
    .unwrap_or(Err(ActuatorError::NotConnected))
}

/// The actuator status (`ros_actuators_update_status`).
pub fn status() -> Status {
    with_state(|s| {
        let t = &mut s.torpedoes;
        let charged = t.charged();
        let d = &s.droppers;
        Status {
            variant: Variant::V1,
            armed: s.armed,
            busy: t.firing || d.alarm_active || s.claw.logic.busy(),
            claw: s.claw.logic.state(),
            torpedo: core_v1::torpedo_state(t.timings.valid(), s.armed, t.firing, charged),
            torpedo_available: (NUM_TORPEDOES as u8).saturating_sub(t.next),
            dropper: core_v1::dropper_state(d.active_time_ms, s.armed, d.alarm_active),
            dropper_available: (NUM_DROPPERS as u8).saturating_sub(d.next),
        }
    })
    .expect("status before init")
}

/// V1 has no Dynamixel status.
pub fn send_extra_status() {}

/// Asks the bridge for the timing parameters, as the C parameter server set them when ROS connected.
pub fn on_ros_connected() {
    if !link::send(&DeviceToHost::Actuator(actuator::DeviceToHost::RequestTimings)) {
        defmt::warn!("Link queue full, couldn't request the actuator timings");
    }
}
