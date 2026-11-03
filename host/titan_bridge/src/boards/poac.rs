//! Poac board (`talos/PoacBoard`): the power stage topics plus the torpedo/dropper actuator.
//!
//! | Topic | Type | Direction |
//! |---|---|---|
//! | power stage topics, see [`super::power`] | | temperature/humidity on `state/{temp,humidity}/poacboard` |
//! | `command/actuator/torpedo` | `std_msgs/Empty` | subscribed |
//! | `command/actuator/dropper` | `std_msgs/Empty` | subscribed |
//! | `command/actuator/notify_reload` | `std_msgs/Empty` | subscribed |
//! | `command/actuator/arm` | `std_msgs/Bool` | subscribed |
//! | `command/actuator/torpedo_marker/go_home` | `std_msgs/Empty` | subscribed |
//! | `command/actuator/torpedo_marker/set_home` | `std_msgs/Empty` | subscribed |
//! | `state/actuator/busy` | `std_msgs/Bool` | published (best effort) |
//! | `state/actuator/status` | `riptide_msgs2/ActuatorStatus` | published (best effort) |
//! | `state/actuator/dynamixel` | `riptide_msgs2/DynamixelStatus` | advertised (best effort), never published as in C |
//! | `state/actuator/cmd_feedback` | `std_msgs/String` | published |
//! | `state/actuator/cmd_status` | `std_msgs/Bool` | published |

use std::sync::Arc;

use anyhow::Result;
use canmore::client_ids;
use rclrs::{DynamicSubscription, IntoPrimitiveOptions, Node};
use titan_link::{DeviceToHost, HostToDevice, poac};

use super::power::{PowerStage, PowerStageTopics, forward};
use super::{BoardProfile, BoardTopics, Transport};
use crate::link::Link;
use crate::ros::{self, Publisher};

pub const PROFILE: BoardProfile = BoardProfile {
    name: "poac",
    node_name: "poac_board",
    namespace: "talos",
    client_id: client_ids::CLIENT_ID_POWER_BOARD,
    transport: Transport::Can { fd: true },
    create: |node, link, logger| Ok(Box::new(PoacTopics::new(node, link, logger)?)),
};

pub struct PoacTopics {
    stage: PowerStageTopics,
    busy: Publisher,
    status: Publisher,
    _dynamixel: Publisher,
    cmd_feedback: Publisher,
    cmd_status: Publisher,
    _subscriptions: Vec<DynamicSubscription>,
}

impl PoacTopics {
    pub fn new(node: &Node, link: Arc<dyn Link>, logger: String) -> Result<Self> {
        let stage = PowerStage {
            temp_topic: "state/temp/poacboard",
            humidity_topic: "state/humidity/poacboard",
            talos_extras: true,
            wrap: |msg| HostToDevice::Poac(poac::HostToDevice::Power(msg)),
        };
        let stage = PowerStageTopics::new(node, link.clone(), logger.clone(), &stage)?;

        let empty_commands = [
            ("command/actuator/torpedo", poac::HostToDevice::FireTorpedo),
            ("command/actuator/dropper", poac::HostToDevice::DropMarker),
            ("command/actuator/notify_reload", poac::HostToDevice::NotifyReload),
            ("command/actuator/torpedo_marker/go_home", poac::HostToDevice::GoHome),
            ("command/actuator/torpedo_marker/set_home", poac::HostToDevice::SetHome),
        ];
        let mut subscriptions = Vec::new();
        for (topic, command) in empty_commands {
            subscriptions.push(forward(node, "std_msgs/msg/Empty", topic, link.clone(), logger.clone(), move |_| {
                Ok(HostToDevice::Poac(command.clone()))
            })?);
        }
        subscriptions.push(forward(node, "std_msgs/msg/Bool", "command/actuator/arm", link, logger, |m| {
            Ok(HostToDevice::Poac(poac::HostToDevice::Arm(ros::get_bool(m, "data")?)))
        })?);

        Ok(PoacTopics {
            stage,
            busy: Publisher::new(node, "std_msgs/msg/Bool", "state/actuator/busy".best_effort())?,
            status: Publisher::new(node, "riptide_msgs2/msg/ActuatorStatus", "state/actuator/status".best_effort())?,
            _dynamixel: Publisher::new(
                node,
                "riptide_msgs2/msg/DynamixelStatus",
                "state/actuator/dynamixel".best_effort(),
            )?,
            cmd_feedback: Publisher::new(node, "std_msgs/msg/String", "state/actuator/cmd_feedback")?,
            cmd_status: Publisher::new(node, "std_msgs/msg/Bool", "state/actuator/cmd_status")?,
            _subscriptions: subscriptions,
        })
    }
}

impl BoardTopics for PoacTopics {
    fn handle(&self, msg: &DeviceToHost) -> Result<bool> {
        let DeviceToHost::Poac(msg) = msg else { return Ok(false) };
        match msg {
            poac::DeviceToHost::Power(msg) => self.stage.handle(msg)?,
            poac::DeviceToHost::Status { busy, status } => {
                self.busy.publish(|m| ros::set_bool(m, "data", *busy))?;
                self.status.publish(|m| {
                    ros::set_bool(m, "actuators_armed", status.actuators_armed)?;
                    ros::set_u8(m, "claw_state", status.claw_state)?;
                    ros::set_u8(m, "torpedo_state", status.torpedo_state)?;
                    ros::set_u8(m, "torpedo_available_count", status.torpedo_available_count)?;
                    ros::set_u8(m, "dropper_state", status.dropper_state)?;
                    ros::set_u8(m, "dropper_available_count", status.dropper_available_count)
                })?;
            }
            poac::DeviceToHost::CommandFeedback(error) => {
                let text = error.map_or("", poac::CommandError::message);
                self.cmd_feedback.publish(|m| ros::set_string(m, "data", text))?;
                self.cmd_status.publish(|m| ros::set_bool(m, "data", error.is_none()))?;
            }
        }
        Ok(true)
    }
}

#[cfg(test)]
mod tests {
    use super::super::power::test_support::{FakeLink, node, spin_until};
    use super::*;
    use titan_link::power;

    #[test]
    fn commands_are_forwarded() -> Result<()> {
        let (mut executor, node) = node("poac")?;
        let link = Arc::new(FakeLink::default());
        let _topics = PoacTopics::new(&node, link.clone(), "test".into())?;
        let torpedo = Publisher::new(&node, "std_msgs/msg/Empty", "command/actuator/torpedo")?;
        let set_home = Publisher::new(&node, "std_msgs/msg/Empty", "command/actuator/torpedo_marker/set_home")?;
        let arm = Publisher::new(&node, "std_msgs/msg/Bool", "command/actuator/arm")?;
        let electrical = Publisher::new(&node, "riptide_msgs2/msg/ElectricalCommand", "command/electrical")?;

        let expected = [
            HostToDevice::Poac(poac::HostToDevice::FireTorpedo),
            HostToDevice::Poac(poac::HostToDevice::SetHome),
            HostToDevice::Poac(poac::HostToDevice::Arm(true)),
            HostToDevice::Poac(poac::HostToDevice::Power(power::HostToDevice::ElectricalCommand(5))),
        ];
        spin_until(
            &mut executor,
            &link,
            || {
                torpedo.publish(|_| Ok(()))?;
                set_home.publish(|_| Ok(()))?;
                arm.publish(|m| ros::set_bool(m, "data", true))?;
                electrical.publish(|m| ros::set_u8(m, "command", 5))
            },
            |sent| expected.iter().all(|e| sent.contains(e)),
        )?;
        Ok(())
    }

    #[test]
    fn every_message_publishes() -> Result<()> {
        let (_executor, node) = node("poac_publish")?;
        let topics = PoacTopics::new(&node, Arc::new(FakeLink::default()), "test".into())?;
        let messages = [
            poac::DeviceToHost::Power(power::DeviceToHost::AuxSwitch(false)),
            poac::DeviceToHost::Status { busy: true, status: poac::ActuatorStatus::default() },
            poac::DeviceToHost::CommandFeedback(None),
            poac::DeviceToHost::CommandFeedback(Some(poac::CommandError::Busy)),
        ];
        for msg in messages {
            assert!(topics.handle(&DeviceToHost::Poac(msg))?);
        }
        assert!(!topics.handle(&DeviceToHost::Power(power::DeviceToHost::KillState(true)))?);
        Ok(())
    }
}
