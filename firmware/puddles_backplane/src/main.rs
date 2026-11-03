//! Puddles backplane: Rust port of `puddles/Backplane`.
//!
//! The backplane is Puddles' only board. It talks to `titan_bridge --board puddles_backplane` on the robot
//! computer over Ethernet (UDP, see [`ethernet`] and `puddles_core::wire`) instead of CAN. ROS topics:
//! * `command/dshot` (`riptide_msgs2/DshotCommand`, best effort, subscribed): thruster throttles, -999..=999
//! * `state/thrusters/rpm` (`riptide_msgs2/DshotRPMFeedback`, best effort, published after every command)
//! * `command/software_kill` (`riptide_msgs2/KillSwitchReport`, best effort, subscribed): software kill switches
//! * `state/kill` (`std_msgs/Bool`, best effort, published): whether the vehicle is killed
//! * `state/depth/raw` (`riptide_msgs2/Depth`) and `state/depth/temp` (`std_msgs/Float32`), sensor data QoS
//! * `state/electrical` (`riptide_msgs2/ElectricalReadings`): rail voltages
//! * `command/electrical` (`riptide_msgs2/ElectricalCommand`, subscribed): power cycles and depth zeroing
//! * `state/fw_heartbeat` (`std_msgs/Int8`) and `state/firmware` (`riptide_msgs2/FirmwareStatus`)
//!
//! Pins (`boards/mk2/puddles_backplane.h`): status LED on GPIO 29/28/27 (R/G/B); W5500 on SPI0 (GPIO 0-3, INT 4,
//! RST 5); sensor I2C0 on GPIO 8/9, board I2C1 on GPIO 6/7; kill switch sense GPIO 12 and aux switch sense
//! GPIO 11 (active low); computer power GPIO 26 and acoustics power GPIO 10 (active high); thrusters 1-8 on
//! GPIO 25-18.
//!
//! The board has no CAN block, so it doesn't use `titan_rp2040::board::start`: it sets up safety, the LED and the
//! link itself and runs the Ethernet transport instead of `canbus`.

#![no_std]
#![no_main]

mod dshot;
mod ethernet;
mod ros;
mod safety_interface;
mod sensors;

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::clocks::RoscRng;
use embassy_rp::flash::{Blocking, Flash};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{self, I2c};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, I2C0, I2C1, PIO0, PIO1};
use embassy_rp::pio::{self, Pio};
use embassy_rp::pwm::Pwm;
use embassy_rp::spi::{self, Spi};
use embassy_rp::{bind_interrupts, dma};

use titan_rp2040::drivers::depth::{Model, Ms5837};
use titan_rp2040::led::{self, StatusLed};
use titan_rp2040::link::{self, LinkConfig};
use titan_rp2040::safety::{self, SafetyConfig};

bind_interrupts!(struct Irqs {
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    PIO1_IRQ_0 => pio::InterruptHandler<PIO1>;
    // DMA_CH0/1 feed the W5500's SPI
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>;
});

/// `PICO_BOARD`, reported in the firmware status.
const BOARD_NAME: &str = "mk2/puddles_backplane";
/// Both I2C buses run at 400 kHz with the internal pull-ups on, as `async_i2c_init` set them up.
const I2C_FREQUENCY: u32 = 400_000;
/// The flash size given to the flash driver, which is only used to read the unique ID.
const FLASH_SIZE: usize = 2 * 1024 * 1024;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let version = titan_rp2040::version!();
    defmt::info!("{} v{}.{} (git {})", BOARD_NAME, version.major, version.minor, version.git);

    // Safety first, so the watchdog covers everything after it
    let safety = safety::setup(
        p.WATCHDOG,
        SafetyConfig {
            fault_names: &safety_interface::FAULT_NAMES,
            on_kill: safety_interface::on_kill,
            on_enable: safety_interface::on_enable,
        },
    );
    spawner.spawn(safety::safety_task(safety).unwrap());
    // The physical kill switch, pulled up: inserting it pulls the line low
    spawner.spawn(safety_interface::kill_switch_task(Input::new(p.PIN_12, Pull::Up)).unwrap());

    // Red on GPIO 29 (PWM 6B), green on 28 (PWM 6A), blue on 27 (PWM 5B)
    let (green, red) = Pwm::new_output_ab(p.PWM_SLICE6, p.PIN_28, p.PIN_29, StatusLed::slice_config()).split();
    let (_, blue) = Pwm::new_output_b(p.PWM_SLICE5, p.PIN_27, StatusLed::slice_config()).split();
    spawner.spawn(led::led_task(StatusLed::new(red.unwrap(), green.unwrap(), blue.unwrap())).unwrap());

    // Depth sensor on the sensor bus
    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = I2C_FREQUENCY;
    let sensor_i2c = I2c::new_async(p.I2C0, p.PIN_9, p.PIN_8, Irqs, i2c_config);
    spawner.spawn(sensors::depth_task(Ms5837::new(sensor_i2c, Model::Ms5837_30ba)).unwrap());

    // Thrusters
    let thrusters = dshot::Dshot::new(
        Pio::new(p.PIO0, Irqs),
        Pio::new(p.PIO1, Irqs),
        dshot::ThrusterPins {
            t1: p.PIN_25,
            t2: p.PIN_24,
            t3: p.PIN_23,
            t4: p.PIN_22,
            t5: p.PIN_21,
            t6: p.PIN_20,
            t7: p.PIN_19,
            t8: p.PIN_18,
        },
    );
    spawner.spawn(dshot::dshot_task(thrusters).unwrap());

    // Rail voltage ADC (and the unused ADC128D818) on the board bus
    let board_i2c = I2c::new_async(p.I2C1, p.PIN_7, p.PIN_6, Irqs, i2c_config);
    spawner.spawn(sensors::adc_task(board_i2c).unwrap());

    // Power rails, both on at boot
    let computer_power = Output::new(p.PIN_26, Level::High);
    let acoustics_power = Output::new(p.PIN_10, Level::High);
    spawner.spawn(ros::power_cycle_task(&ros::COMPUTER_POWER, computer_power).unwrap());
    spawner.spawn(ros::power_cycle_task(&ros::ACOUSTICS_POWER, acoustics_power).unwrap());
    // The aux switch is configured as an input like in C, but nothing reads it. The pad keeps its reset pull-down,
    // as `gpio_init` left it.
    let aux_switch = Input::new(p.PIN_11, Pull::Down);
    core::mem::forget(aux_switch);

    spawner.spawn(
        link::link_task(LinkConfig {
            board_name: BOARD_NAME,
            bus_id: titan_rp2040::puddles::ETHERNET_BUS_ID,
            client_id: ros::CLIENT_ID,
            version_major: version.major,
            version_minor: version.minor,
            release_type: version.release_type,
            on_board_message: ros::handle_board_message,
        })
        .unwrap(),
    );
    spawner.spawn(ros::ros_task().unwrap());

    // Ethernet last, as in C
    let mut unique_id = [0u8; 8];
    let mut flash = Flash::<_, Blocking, FLASH_SIZE>::new_blocking(p.FLASH);
    if flash.blocking_unique_id(&mut unique_id).is_err() {
        defmt::warn!("Couldn't read the flash unique ID; using a fixed MAC address");
    }
    let mut spi_config = spi::Config::default();
    spi_config.frequency = ethernet::SPI_FREQUENCY;
    let spi = Spi::new(p.SPI0, p.PIN_2, p.PIN_3, p.PIN_0, p.DMA_CH0, p.DMA_CH1, Irqs, spi_config);
    let stack = ethernet::init(
        spawner,
        spi,
        Output::new(p.PIN_1, Level::High),
        Input::new(p.PIN_4, Pull::Up),
        Output::new(p.PIN_5, Level::High),
        ethernet::mac_address(unique_id),
        RoscRng.next_u64(),
    )
    .await;
    spawner.spawn(ethernet::transport_task(stack).unwrap());
}
