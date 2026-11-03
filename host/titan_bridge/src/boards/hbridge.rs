//! Lilboat H-bridge board (`lilboat/HBridgeBoard`): motor and thruster bridges, and the claw servos.

use std::sync::Arc;

use anyhow::Result;
use canmore::client_ids;
use rclrs::{DynamicMessage, DynamicSubscription, IntoPrimitiveOptions, MessageInfo, Node};
use titan_link::hbridge;
use titan_link::{DeviceToHost, HostToDevice};

use super::{BoardProfile, BoardTopics, Transport};
use crate::link::Link;
use crate::ros::{self, Publisher};

pub const PROFILE: BoardProfile = BoardProfile {
    name: "hbridge",
    node_name: "hbridge_board",
    namespace: "liltank",
    client_id: client_ids::CLIENT_ID_ESC_BOARD0,
    transport: Transport::Can { fd: true },
    create: |node, link, logger| Ok(Box::new(HBridgeTopics::new(node, link, logger)?)),
};

/// Bridge command topics in C index order (`HBRIDGE_SUBSCRIBER_NAMES`).
const BRIDGE_TOPICS: [&str; hbridge::NUM_BRIDGE_TOPICS] = [
    "command/thruster/port",
    "command/thruster/stbd",
    "command/motor/stbd",
    "command/motor/port",
    "command/bridge2/unused",
];

pub struct HBridgeTopics {
    status: Publisher,
    degrees: Publisher,
    cmd_feedback: Publisher,
    cmd_status: Publisher,
    _subscriptions: Vec<DynamicSubscription>,
}

fn forward(
    link: Arc<dyn Link>,
    logger: String,
    to_msg: impl Fn(&DynamicMessage) -> Result<hbridge::HostToDevice> + Send + Sync + 'static,
) -> impl Fn(DynamicMessage, MessageInfo) + Send + Sync + 'static {
    move |msg, _info| {
        let result = to_msg(&msg).and_then(|m| link.send(&HostToDevice::HBridge(m)));
        if let Err(err) = result {
            rclrs::log_warn!(&logger, "Forwarding H-bridge command failed: {err:#}");
        }
    }
}

impl HBridgeTopics {
    pub fn new(node: &Node, link: Arc<dyn Link>, logger: String) -> Result<Self> {
        let mut subscriptions = Vec::new();
        for (bridge, topic) in BRIDGE_TOPICS.into_iter().enumerate() {
            subscriptions.push(node.create_dynamic_subscription(
                "std_msgs/msg/Float32".try_into().unwrap(),
                topic,
                forward(link.clone(), logger.clone(), move |m| {
                    Ok(hbridge::HostToDevice::SetTarget { bridge: bridge as u8, percent: ros::get_f32(m, "data")? })
                }),
            )?);
        }
        subscriptions.push(node.create_dynamic_subscription(
            "std_msgs/msg/Bool".try_into().unwrap(),
            "command/actuator/arm",
            forward(link.clone(), logger.clone(), |m| {
                Ok(hbridge::HostToDevice::ActuatorArm(ros::get_bool(m, "data")?))
            }),
        )?);
        subscriptions.push(node.create_dynamic_subscription(
            "std_msgs/msg/Int32".try_into().unwrap(),
            "command/actuator/move_speed_for_time",
            forward(link, logger, |m| Ok(hbridge::HostToDevice::MoveSpeedForTime(ros::get_i32(m, "data")?))),
        )?);

        Ok(HBridgeTopics {
            status: Publisher::new(node, "riptide_msgs2/msg/ActuatorStatus", "state/actuator/status".best_effort())?,
            degrees: Publisher::new(node, "std_msgs/msg/Int32", "state/actuator/degrees")?,
            cmd_feedback: Publisher::new(node, "std_msgs/msg/String", "state/actuator/cmd_feedback")?,
            cmd_status: Publisher::new(node, "std_msgs/msg/Bool", "state/actuator/cmd_status")?,
            _subscriptions: subscriptions,
        })
    }
}

impl BoardTopics for HBridgeTopics {
    fn handle(&self, msg: &DeviceToHost) -> Result<bool> {
        let DeviceToHost::HBridge(msg) = msg else { return Ok(false) };
        match msg {
            hbridge::DeviceToHost::ActuatorStatus { claw_state } => {
                // The other fields stay at their defaults (false/0); C left them uninitialized
                self.status.publish(|m| ros::set_u8(m, "claw_state", *claw_state))?;
            }
            hbridge::DeviceToHost::ActuatorDegrees(degrees) => {
                self.degrees.publish(|m| ros::set_i32(m, "data", *degrees))?;
            }
            hbridge::DeviceToHost::CmdFeedback { success, message } => {
                self.cmd_feedback.publish(|m| ros::set_string(m, "data", message.as_str()))?;
                self.cmd_status.publish(|m| ros::set_bool(m, "data", *success))?;
            }
        }
        Ok(true)
    }
}

#[cfg(test)]
mod tests {
    //! Run inside `riptide-shell`: these load real ROS type support.

    use super::*;
    use rclrs::DynamicMessageMetadata;

    fn message(type_name: &str) -> DynamicMessage {
        DynamicMessageMetadata::new(type_name.try_into().unwrap()).unwrap().create().unwrap()
    }

    #[test]
    fn int32_accessors_round_trip() -> Result<()> {
        let mut msg = message("std_msgs/msg/Int32");
        ros::set_i32(&mut msg, "data", -1000)?;
        assert_eq!(ros::get_i32(&msg, "data")?, -1000);
        Ok(())
    }

    #[test]
    fn actuator_status_has_a_claw_state() -> Result<()> {
        let mut msg = message("riptide_msgs2/msg/ActuatorStatus");
        ros::set_u8(&mut msg, "claw_state", 5)?;
        assert_eq!(ros::get_u8(&msg, "claw_state")?, 5);
        assert!(!ros::get_bool(&msg, "actuators_armed")?);
        Ok(())
    }
}
