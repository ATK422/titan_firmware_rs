//! Smart battery housing MCU (`talos/SmartBattery`): `state/battery`, `status/temp/smartbattery`,
//! `status/humidity/smartbattery`, `command/electrical`, `command/software_kill` and `state/leak`.
//!
//! Each pack's board picks its CANmore client ID from its serial number, so run one bridge per pack with
//! `--client-id <serial>`.
//!
//! Two names differ from the C node until the bridge can set them per board (see the port's notes): the C node was
//! called `smart_battery_<serial>` and published its heartbeat on `state/fw_heartbeat`. Until then, run with
//! `--ros-args -r __node:=smart_battery_<serial> -r heartbeat:=state/fw_heartbeat` to match.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use anyhow::Result;
use canmore::client_ids;
use rclrs::{DynamicMessage, DynamicSubscription, IntoPrimitiveOptions, MessageInfo, Node};
use titan_link::smart_battery::{self, BatteryStatus};
use titan_link::{DeviceToHost, HostToDevice};

use super::{BoardProfile, BoardTopics, Transport};
use crate::link::Link;
use crate::ros::{self, Publisher};

pub const PROFILE: BoardProfile = BoardProfile {
    name: "smart_battery",
    node_name: "smart_battery",
    namespace: "talos",
    client_id: client_ids::CLIENT_ID_SMART_BATTERY_PORT,
    transport: Transport::Can { fd: false },
    create: |node, link, logger| Ok(Box::new(SmartBatteryTopics::new(node, link, logger)?)),
};

/// `riptide_msgs2/ElectricalCommand` values the board acts on.
const CYCLE_ROBOT: u8 = 3;
const KILL_ROBOT_POWER: u8 = 4;
/// `riptide_msgs2/KillSwitchReport::KILL_SWITCH_RQT_CONTROLLER`: the operator's controls are open.
const KILL_SWITCH_RQT_CONTROLLER: u8 = 1;

/// Cut robot power on a leak when no operator is around. Disabled in the C firmware ("Put this back in once leak
/// false positives are fixed"), so disabled here too.
const LEAK_KILLS_POWER: bool = false;

pub struct SmartBatteryTopics {
    battery: Publisher,
    temperature: Publisher,
    humidity: Publisher,
    _subscriptions: [DynamicSubscription; 3],
}

/// Copies a [`BatteryStatus`] into a `riptide_msgs2/msg/BatteryStatus` (`ros_update_battery_status`).
pub fn fill_battery_status(msg: &mut DynamicMessage, status: &BatteryStatus) -> Result<()> {
    ros::set_string(msg, "cell_name", "")?;
    ros::set_u16(msg, "serial", status.serial)?;
    ros::set_u8(msg, "detect", status.detect as u8)?;
    ros::set_f32(msg, "pack_voltage", status.pack_voltage_mv as f32 / 1000.0)?;
    ros::set_f32(msg, "pack_current", status.pack_current_ma as f32 / 1000.0)?;
    ros::set_f32(msg, "average_current", status.average_current_ma as f32 / 1000.0)?;
    ros::set_u16(msg, "time_to_dischg", status.time_to_dischg)?;
    ros::set_u8(msg, "soc", status.soc)
}

/// The board command for an `ElectricalCommand`, if it's one the smart battery handles.
fn electrical_command(command: u8) -> Option<smart_battery::HostToDevice> {
    match command {
        CYCLE_ROBOT => Some(smart_battery::HostToDevice::PowerCycleRobot),
        KILL_ROBOT_POWER => Some(smart_battery::HostToDevice::KillRobotPower),
        // Commands for other boards
        _ => None,
    }
}

impl SmartBatteryTopics {
    pub fn new(node: &Node, link: Arc<dyn Link>, logger: String) -> Result<Self> {
        let (cmd_link, cmd_logger) = (link.clone(), logger.clone());
        let electrical = node.create_dynamic_subscription(
            "riptide_msgs2/msg/ElectricalCommand".try_into().unwrap(),
            "command/electrical",
            move |msg: DynamicMessage, _info: MessageInfo| {
                let result = ros::get_u8(&msg, "command").and_then(|command| match electrical_command(command) {
                    Some(cmd) => {
                        rclrs::log_info!(&cmd_logger, "Forwarding electrical command {cmd:?}");
                        cmd_link.send(&HostToDevice::SmartBattery(cmd))
                    }
                    None => Ok(()),
                });
                if let Err(err) = result {
                    rclrs::log_warn!(&cmd_logger, "Forwarding electrical command failed: {err:#}");
                }
            },
        )?;

        // The leak response (`swkill_subscription_callback`, `leak_subscription_callback`) never touched the
        // board's hardware, so it lives here
        let has_rviz = Arc::new(AtomicBool::new(false));
        let swkill_rviz = has_rviz.clone();
        let swkill_logger = logger.clone();
        let software_kill = node.create_dynamic_subscription(
            "riptide_msgs2/msg/KillSwitchReport".try_into().unwrap(),
            "command/software_kill",
            move |msg: DynamicMessage, _info: MessageInfo| match ros::get_u8(&msg, "kill_switch_id") {
                Ok(KILL_SWITCH_RQT_CONTROLLER) => swkill_rviz.store(true, Ordering::Relaxed),
                Ok(_) => {}
                Err(err) => rclrs::log_warn!(&swkill_logger, "Bad software kill message: {err:#}"),
            },
        )?;
        let leak = node.create_dynamic_subscription(
            "std_msgs/msg/Bool".try_into().unwrap(),
            "state/leak",
            move |msg: DynamicMessage, _info: MessageInfo| match ros::get_bool(&msg, "data") {
                Ok(true) if !has_rviz.load(Ordering::Relaxed) => {
                    if LEAK_KILLS_POWER {
                        rclrs::log_info!(
                            &logger,
                            "Leak detected but no operator found to respond... Issuing latched Emergency FET Shutdown"
                        );
                        if let Err(err) =
                            link.send(&HostToDevice::SmartBattery(smart_battery::HostToDevice::KillRobotPower))
                        {
                            rclrs::log_warn!(&logger, "Sending kill robot power failed: {err:#}");
                        }
                    } else {
                        rclrs::log_info!(
                            &logger,
                            "Leak detected but no operator found to respond (power kill disabled)"
                        );
                    }
                }
                Ok(true) => {}
                // Recheck for the operator; not while leaking, since the leak popup blocks the software kill
                Ok(false) => has_rviz.store(false, Ordering::Relaxed),
                Err(err) => rclrs::log_warn!(&logger, "Bad leak message: {err:#}"),
            },
        )?;

        Ok(SmartBatteryTopics {
            battery: Publisher::new(node, "riptide_msgs2/msg/BatteryStatus", "state/battery")?,
            // rclc_publisher_init_best_effort in C
            temperature: Publisher::new(node, "std_msgs/msg/Float32", "status/temp/smartbattery".best_effort())?,
            humidity: Publisher::new(node, "std_msgs/msg/Float32", "status/humidity/smartbattery".best_effort())?,
            _subscriptions: [electrical, software_kill, leak],
        })
    }
}

impl BoardTopics for SmartBatteryTopics {
    fn handle(&self, msg: &DeviceToHost) -> Result<bool> {
        match msg {
            DeviceToHost::SmartBattery(smart_battery::DeviceToHost::BatteryStatus(status)) => {
                self.battery.publish(|m| fill_battery_status(m, status))?;
            }
            DeviceToHost::SmartBattery(smart_battery::DeviceToHost::TempHumidity { temperature_c, humidity_rh }) => {
                self.temperature.publish(|m| ros::set_f32(m, "data", *temperature_c))?;
                self.humidity.publish(|m| ros::set_f32(m, "data", *humidity_rh))?;
            }
            _ => return Ok(false),
        }
        Ok(true)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use anyhow::bail;
    use titan_link::smart_battery::Detect;

    #[test]
    fn electrical_commands() {
        assert_eq!(electrical_command(3), Some(smart_battery::HostToDevice::PowerCycleRobot));
        assert_eq!(electrical_command(4), Some(smart_battery::HostToDevice::KillRobotPower));
        assert_eq!(electrical_command(0), None);
        assert_eq!(electrical_command(10), None);
    }

    /// Runs inside `riptide-shell`: goes through the real `riptide_msgs2` type support.
    #[test]
    fn battery_status_fills_every_field() -> Result<()> {
        let metadata = rclrs::DynamicMessageMetadata::new("riptide_msgs2/msg/BatteryStatus".try_into().unwrap())
            .map_err(|e| anyhow::anyhow!("{e:?}"))?;
        let mut msg = metadata.create().map_err(|e| anyhow::anyhow!("{e:?}"))?;
        let status = BatteryStatus {
            serial: 5,
            detect: Detect::Stbd,
            pack_voltage_mv: 18_400,
            pack_current_ma: -12_500,
            average_current_ma: -12_000,
            time_to_dischg: 95,
            soc: 87,
        };
        fill_battery_status(&mut msg, &status)?;
        assert_eq!(ros::get_u8(&msg, "detect")?, 2);
        assert_eq!(ros::get_u8(&msg, "soc")?, 87);
        assert_eq!(ros::get_f32(&msg, "pack_voltage")?, 18.4);
        assert_eq!(ros::get_f32(&msg, "pack_current")?, -12.5);
        match msg.get("serial") {
            Some(rclrs::Value::Simple(rclrs::SimpleValue::Uint16(v))) => assert_eq!(*v, 5),
            other => bail!("serial is {other:?}"),
        }
        Ok(())
    }
}
