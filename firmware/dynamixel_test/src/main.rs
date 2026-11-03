//! Dynamixel bench test, ported from `examples/dynamixel_test`.
//!
//! Runs on a plain Pico with an XL430 (ID 2) on GPIO 4 behind a 3.3/5 V level shifter. It logs the servo's EEPROM
//! when it connects and its RAM on every refresh, enables torque on connect, and then holds position 2048, or 500
//! while the BOOTSEL button is held. The on-board LED blinks while the main loop runs, and a 5 s watchdog resets the
//! chip if it stalls.
//!
//! Logs go out over defmt/RTT instead of UART stdio.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{self, Pio};
use embassy_rp::watchdog::Watchdog;
use embassy_rp::{bind_interrupts, dma};
use embassy_time::{Duration, Timer};

// Linked for its panic handler (log, then reset through the watchdog)
use titan_rp2040 as _;

use dynamixel::uart::{HalfDuplexUart, MultidropProgram};
use dynamixel::{Driver, Event, Id, Servos};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>;
});

/// The servo under test.
const SERVO_ID: Id = 2;
const WATCHDOG_TIMEOUT: Duration = Duration::from_millis(5000);

static SERVOS: Servos<1> = Servos::new([SERVO_ID]);

/// State machine 0 raises IRQ 0 when a transmission ends and IRQ 1 on a framing error.
type Bus = Driver<'static, PIO0, 0, 0, 1, 1>;

fn on_event(event: Event, id: Id) {
    match event {
        Event::Disconnected => defmt::info!("Servo {} disconnected", id),
        Event::Connected => {
            let Some(e) = SERVOS.eeprom(id) else { return };
            defmt::info!("Servo connected!");
            defmt::info!("--- Servo {} EEPROM ---", id);
            defmt::info!("  Model Number: {}", e.model_num);
            defmt::info!("  Model Info: {}", e.model_info);
            defmt::info!("  Firmware Version: {}", e.firmware_version);
            defmt::info!("  ID: {}", e.id);
            defmt::info!("  Baud Rate: {}", e.baud_rate);
            defmt::info!("  Return Delay Time: {}", e.return_delay_time);
            defmt::info!("  Drive Mode: {}", e.drive_mode);
            defmt::info!("  Operating Mode: {}", e.operating_mode);
            defmt::info!("  Secondary ID: {}", e.secondary_id);
            defmt::info!("  Protocol: {}", e.protocol_type);
            defmt::info!("  Homing Offset: {}", e.homing_offset);
            defmt::info!("  Temperature Limit: {}", e.temperature_limit);
            defmt::info!("  Min Voltage Limit: {}", e.min_voltage_limit);
            defmt::info!("  Max Voltage Limit: {}", e.max_voltage_limit);
            defmt::info!("  PWM Limit: {}", e.pwm_limit);
            defmt::info!("  Velocity Limit: {}", e.velocity_limit);
            defmt::info!("  Min Position Limit: {}", e.min_position_limit);
            defmt::info!("  Max Position Limit: {}", e.max_position_limit);
            defmt::info!("  Startup Config: {}", e.startup_config);
            defmt::info!("-------------------");
            SERVOS.enable_torque(SERVO_ID, true);
        }
        Event::RamRead => {
            let Some(r) = SERVOS.ram(id) else { return };
            defmt::info!("--- Servo {} RAM ---", id);
            defmt::info!("  Torque Enable: {}", r.torque_enable);
            defmt::info!("  Moving: {}", r.moving);
            defmt::info!("  Realtime Tick: {}", r.realtime_tick);
            defmt::info!("  Hardware Error Status: {}", r.hardware_error_status);
            defmt::info!("  Goal Position: {}", r.goal_position);
            defmt::info!("  Present Position: {}", r.present_position);
            defmt::info!("  Present Velocity: {}", r.present_velocity);
            defmt::info!("  Present Voltage: {}.{} V", r.present_input_voltage / 10, r.present_input_voltage % 10);
            defmt::info!("  Present Temperature: {} C", r.present_temperature);
            defmt::info!("-------------------");
        }
        Event::Alert => defmt::info!("Servo {} hardware error status set!", id),
        Event::EepromRead => defmt::info!("Received unexpected dynamixel event {} for ID {}", event, id),
    }
}

#[embassy_executor::task]
async fn dynamixel_task(bus: Bus) -> ! {
    bus.run(on_event, |error| {
        defmt::info!(
            "Dynamixel driver error: {} (arg: {}) - {}",
            error.kind,
            error.wrapped,
            if error.source == dynamixel::Source::Comms { "dynamixel_comms" } else { "dynamixel_schedule" }
        )
    })
    .await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut p = embassy_rp::init(Default::default());
    let mut watchdog = Watchdog::new(p.WATCHDOG);
    watchdog.pause_on_debug(true);
    watchdog.start(WATCHDOG_TIMEOUT);
    let mut led = Output::new(p.PIN_25, Level::Low);
    let version = titan_rp2040::version!();
    defmt::info!("dynamixel_test v{}.{} ({})", version.major, version.minor, version.git);

    Timer::after_millis(2000).await;
    defmt::info!("Starting up dynamixel");
    let Pio { mut common, sm0, irq_flags, irq0, irq1, .. } = Pio::new(p.PIO0, Irqs);
    let program = MultidropProgram::new(&mut common);
    let dma = dma::Channel::new(p.DMA_CH0, Irqs);
    let uart =
        HalfDuplexUart::new(&mut common, sm0, irq_flags, irq0, irq1, p.PIN_4, dma, &program, dynamixel::BAUD_RATE);
    spawner.spawn(dynamixel_task(Driver::new(uart, &SERVOS)).unwrap());

    Timer::after_millis(100).await;
    loop {
        watchdog.feed(WATCHDOG_TIMEOUT);
        led.toggle();
        Timer::after_millis(250).await;

        let target = if embassy_rp::bootsel::is_bootsel_pressed(p.BOOTSEL.reborrow()) { 500 } else { 2048 };
        SERVOS.set_target_position(SERVO_ID, target);
    }
}
