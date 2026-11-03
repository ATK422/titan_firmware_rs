//! The board's ROS traffic, ported from `ros.c` and the ROS half of `main.c`. The bridge
//! (`titan_bridge --board smart_battery --client-id <serial>`) owns the topics:
//!
//! * `state/battery` (`riptide_msgs2/BatteryStatus`): every second, from [`battery_status_task`]
//! * `status/temp/smartbattery`, `status/humidity/smartbattery` (`std_msgs/Float32`, best effort): every SHT41
//!   reading, from [`temp_humidity_task`]
//! * `command/electrical` (`riptide_msgs2/ElectricalCommand`): `CYCLE_ROBOT` and `KILL_ROBOT_POWER` arrive here as
//!   [`smart_battery::HostToDevice`] messages
//! * `command/software_kill` and `state/leak`: handled in the bridge (the C leak response was disabled)
//!
//! The C firmware only brought its ROS node up once the pack's manufacturer info was read, so nothing here is sent
//! before [`background::mfg_readout`] has it.

use embassy_time::{Duration, Ticker, with_timeout};

use smart_battery_core::faults;
use smart_battery_core::monitor::EmshutCommand;
use titan_link::smart_battery::{self, BatteryStatus, Detect, ROBOT_POWER_CYCLE_TIME_MS};
use titan_link::{DeviceToHost, HostToDevice};
use titan_rp2040::drivers::Latest;
use titan_rp2040::drivers::sht41::{self, Sht41};
use titan_rp2040::link;

use crate::background::{self, BusDevice};
use crate::fault;
use crate::gauge::{self, Request};

/// `BATTERY_STATUS_TIME_MS`.
const BATTERY_STATUS_PERIOD: Duration = Duration::from_millis(1000);
/// How long to wait for the side detect read on core 1.
const SIDE_DETECT_TIMEOUT: Duration = Duration::from_millis(200);
/// How often new SHT41 readings are looked for; readings arrive every 500 ms.
const SHT41_CHECK_PERIOD: Duration = Duration::from_millis(50);

/// The latest SHT41 reading.
pub static SHT41: Latest<sht41::Reading> = Latest::new(sht41::VALID_FOR);

fn request(req: Request) {
    if gauge::REQUESTS.try_send(req).is_err() {
        defmt::warn!("Gauge request queue full, dropping {}", req);
    }
}

/// Link hook for the board's messages (`electrical_command_callback`).
pub fn handle_board_message(msg: HostToDevice) {
    match msg {
        HostToDevice::SmartBattery(smart_battery::HostToDevice::PowerCycleRobot) => {
            defmt::info!("ROS Command requested robot power cycle... Issuing temporary Emergency FET Shutdown");
            request(Request::Emshut(EmshutCommand::PowerCycle { duration_ms: ROBOT_POWER_CYCLE_TIME_MS }));
        }
        HostToDevice::SmartBattery(smart_battery::HostToDevice::KillRobotPower) => {
            defmt::info!("ROS Command requested kill robot power... Issuing latched Emergency FET Shutdown");
            request(Request::Emshut(EmshutCommand::KillPower));
        }
        other => defmt::warn!("Unexpected message for the smart battery: {}", other),
    }
}

/// Reads the side detect pin through core 1.
async fn side_detect() -> Detect {
    gauge::SIDE_DETECT.reset();
    gauge::REQUESTS.send(Request::SideDetect).await;
    match with_timeout(SIDE_DETECT_TIMEOUT, gauge::SIDE_DETECT.wait()).await {
        Ok(Some(true)) => Detect::Port,
        Ok(Some(false)) => Detect::Stbd,
        Ok(None) | Err(_) => Detect::None,
    }
}

/// Publishes the battery status every second while ROS is up (`ros_update_battery_status`).
///
/// The C version asked core 1 for the side detect pin and published whatever result was ready, so every other
/// message reported `DETECT_NONE`. This one waits for the read and reports it in every message.
#[embassy_executor::task]
pub async fn battery_status_task() -> ! {
    let mut ticker = Ticker::every(BATTERY_STATUS_PERIOD);
    loop {
        ticker.next().await;
        let Some(mfg) = background::mfg_readout() else { continue };
        if !link::is_connected() {
            continue;
        }
        let detect = side_detect().await;
        let info = gauge::shared().info;
        let (time_to_dischg, _) = info.time_remaining();
        let status = BatteryStatus {
            serial: mfg.serial,
            detect,
            pack_voltage_mv: info.voltage(),
            pack_current_ma: info.current,
            average_current_ma: info.avg_current,
            time_to_dischg,
            soc: info.relative_soc,
        };
        if !link::send(&DeviceToHost::SmartBattery(smart_battery::DeviceToHost::BatteryStatus(status))) {
            defmt::warn!("Dropped battery status, CAN queue full");
        }
    }
}

/// Polls the SHT41 (`sht41_init` with `sht41_sensor_error_cb`).
#[embassy_executor::task]
pub async fn sht41_task(mut sensor: Sht41<BusDevice>) -> ! {
    sensor.run(&SHT41, |err| fault::raise(faults::SHT41_ERROR, err.code())).await
}

/// Forwards each valid SHT41 reading while ROS is up (`ros_update_temp_humidity_publisher`).
#[embassy_executor::task]
pub async fn temp_humidity_task() -> ! {
    let mut ticker = Ticker::every(SHT41_CHECK_PERIOD);
    loop {
        ticker.next().await;
        if background::mfg_readout().is_none() || !link::is_connected() {
            continue;
        }
        let (Some(reading), Some(_)) = (SHT41.take_new(), SHT41.get()) else { continue };
        let msg = smart_battery::DeviceToHost::TempHumidity {
            temperature_c: reading.temperature_c,
            humidity_rh: reading.humidity_rh,
        };
        link::send(&DeviceToHost::SmartBattery(msg));
    }
}
