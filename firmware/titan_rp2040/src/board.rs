//! Standard bring-up for boards built on the `rp2040_can_block`, the Rust counterpart of the template `main.c`.
//!
//! ```ignore
//! bind_interrupts!(struct Irqs {
//!     // The CAN block's SPI uses DMA channels 1 and 2
//!     DMA_IRQ_0 => dma::InterruptHandler<DMA_CH1>, dma::InterruptHandler<DMA_CH2>;
//! });
//!
//! #[embassy_executor::main]
//! async fn main(spawner: Spawner) {
//!     let p = embassy_rp::init(Default::default());
//!     titan_rp2040::board::start(spawner, titan_rp2040::can_block!(p, Irqs), BoardConfig {
//!         name: "mk2/power_board",
//!         client_id: canmore::client_ids::CLIENT_ID_POWER_BOARD,
//!         bus: Bus::Internal,
//!         version: titan_rp2040::version!(),
//!         ..BoardConfig::DEFAULT
//!     });
//!     // Board specific hardware and tasks from here on
//! }
//! ```
//!
//! [`start`] sets up safety first (so the watchdog covers everything after it), then spawns the status LED, the CAN
//! bus and the link. The `can_block!` macro moves the block's peripherals out of `p`: WATCHDOG, PWM slices 6 and 7,
//! GPIO 13-15 and 17-22, SPI0 and DMA channels 1 and 2. Everything else stays available to the board.

use embassy_executor::Spawner;
use embassy_rp::Peri;
use embassy_rp::clocks::Gpout;
use embassy_rp::peripherals::{PIN_21, WATCHDOG};
use titan_link::{HostToDevice, ReleaseType};

use crate::canbus::{self, CanConfig};
use crate::led::{self, StatusLed};
use crate::link::{self, LinkConfig};
use crate::mcp251xfd::{BitRates, Mcp251xfd};
use crate::safety::{self, STANDARD_FAULT_NAMES, SafetyConfig};

/// Firmware version, normally produced by [`crate::version!`].
#[derive(Clone, Copy, Debug)]
pub struct Version {
    pub major: u8,
    pub minor: u8,
    pub release_type: ReleaseType,
    /// `git describe` of the build.
    pub git: &'static str,
}

/// Which CAN bus of the robot the board sits on (`CAN_BUS_NAME` in the C board headers).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Bus {
    /// CAN FD, 1 Mbit/s arbitration and 5 Mbit/s data (Talos and Lilboat).
    Internal,
    /// Classic CAN at 250 kbit/s (Talos smart batteries).
    External,
}

impl Bus {
    pub fn rates(self) -> BitRates {
        match self {
            Bus::Internal => {
                BitRates { nominal: crate::talos::INTERNAL_CAN_RATE, data: Some(crate::talos::INTERNAL_CAN_FD_RATE) }
            }
            Bus::External => BitRates { nominal: crate::talos::EXTERNAL_CAN_RATE, data: None },
        }
    }

    pub fn id(self) -> u8 {
        match self {
            Bus::Internal => crate::talos::INTERNAL_CAN_ID,
            Bus::External => crate::talos::EXTERNAL_CAN_ID,
        }
    }
}

/// Board identity and hooks.
pub struct BoardConfig {
    /// `PICO_BOARD` of the C firmware, e.g. `"talos/ivc_board"`. Reported in the firmware status.
    pub name: &'static str,
    /// CANmore client ID. Boards that pick their ID at runtime (ESC boards, smart batteries) read it before `start`.
    pub client_id: u8,
    pub bus: Bus,
    pub version: Version,
    /// See [`SafetyConfig::fault_names`].
    pub fault_names: &'static [&'static str],
    /// See [`SafetyConfig::on_kill`].
    pub on_kill: fn(),
    /// See [`SafetyConfig::on_enable`].
    pub on_enable: fn(),
    /// See [`LinkConfig::on_board_message`].
    pub on_board_message: fn(HostToDevice),
}

fn noop() {}

fn ignore_message(msg: HostToDevice) {
    defmt::warn!("No handler for {}", msg);
}

impl BoardConfig {
    /// Defaults for the hooks and fault names; `name`, `client_id`, `bus` and `version` must be overridden.
    pub const DEFAULT: BoardConfig = BoardConfig {
        name: "unknown",
        client_id: 0,
        bus: Bus::Internal,
        version: Version { major: 0, minor: 0, release_type: ReleaseType::Debug, git: "unknown" },
        fault_names: &STANDARD_FAULT_NAMES,
        on_kill: noop,
        on_enable: noop,
        on_board_message: ignore_message,
    };
}

/// The peripherals of the `rp2040_can_block`, built by [`crate::can_block!`].
pub struct CanBlock {
    pub watchdog: Peri<'static, WATCHDOG>,
    pub led: StatusLed,
    pub can: Mcp251xfd,
    pub can_clock: Gpout<'static, PIN_21>,
}

/// Brings up safety, the status LED, the CAN bus and the link.
pub fn start(spawner: Spawner, block: CanBlock, config: BoardConfig) {
    assert!(config.name.len() <= titan_link::BOARD_NAME_LEN, "board name too long");
    defmt::info!(
        "{} v{}.{} (release type {}, git {})",
        config.name,
        config.version.major,
        config.version.minor,
        config.version.release_type as u8,
        config.version.git
    );

    let safety = safety::setup(
        block.watchdog,
        SafetyConfig { fault_names: config.fault_names, on_kill: config.on_kill, on_enable: config.on_enable },
    );
    spawner.spawn(safety::safety_task(safety).unwrap());
    spawner.spawn(led::led_task(block.led).unwrap());

    let can_config = CanConfig { client_id: config.client_id, rates: config.bus.rates() };
    spawner.spawn(canbus::can_task(block.can, block.can_clock, can_config).unwrap());

    spawner.spawn(
        link::link_task(LinkConfig {
            board_name: config.name,
            bus_id: config.bus.id(),
            client_id: config.client_id,
            version_major: config.version.major,
            version_minor: config.version.minor,
            release_type: config.version.release_type,
            on_board_message: config.on_board_message,
        })
        .unwrap(),
    );
}

#[doc(hidden)]
pub const fn parse_u8(s: &str) -> u8 {
    let bytes = s.as_bytes();
    let mut value: u8 = 0;
    let mut i = 0;
    while i < bytes.len() {
        value = value * 10 + (bytes[i] - b'0');
        i += 1;
    }
    value
}

#[doc(hidden)]
pub const fn release_type(s: &str) -> ReleaseType {
    match parse_u8(s) {
        0 => ReleaseType::Debug,
        1 => ReleaseType::Dev,
        2 => ReleaseType::Clean,
        _ => ReleaseType::Tagged,
    }
}

/// The calling crate's [`Version`]: major and minor from its `Cargo.toml` version, release type and git description
/// from `titan_build::configure()` in its build script.
#[macro_export]
macro_rules! version {
    () => {
        $crate::board::Version {
            major: $crate::board::parse_u8(env!("CARGO_PKG_VERSION_MAJOR")),
            minor: $crate::board::parse_u8(env!("CARGO_PKG_VERSION_MINOR")),
            release_type: $crate::board::release_type(env!("TITAN_RELEASE_TYPE")),
            git: env!("TITAN_GIT_DESCRIBE"),
        }
    };
}

/// Builds a [`board::CanBlock`](crate::board::CanBlock) from the `embassy_rp::init` peripherals `p`. `irqs` must bind
/// the DMA_IRQ_0 handlers for DMA_CH1 and DMA_CH2.
#[macro_export]
macro_rules! can_block {
    ($p:ident, $irqs:expr) => {{
        use $crate::__private::embassy_rp::{gpio, spi};
        let mut spi_config = spi::Config::default();
        spi_config.frequency = 1_000_000;
        let spi = spi::Spi::new($p.SPI0, $p.PIN_18, $p.PIN_19, $p.PIN_20, $p.DMA_CH1, $p.DMA_CH2, $irqs, spi_config);
        $crate::board::CanBlock {
            watchdog: $p.WATCHDOG,
            led: $crate::led::StatusLed::can_block($p.PWM_SLICE6, $p.PIN_13, $p.PWM_SLICE7, $p.PIN_14, $p.PIN_15),
            can: $crate::mcp251xfd::Mcp251xfd::new(
                spi,
                gpio::Output::new($p.PIN_17, gpio::Level::High),
                gpio::Input::new($p.PIN_22, gpio::Pull::Up),
            ),
            can_clock: $crate::canbus::start_reference_clock($p.PIN_21),
        }
    }};
}
