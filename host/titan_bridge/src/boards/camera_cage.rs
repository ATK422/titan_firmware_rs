//! Camera cage breakout board (`talos/CameraCageBB`): depth, cage temperature/humidity, status strip, fans and the
//! Orin power switch.
//!
//! The C board published its heartbeat on `state/fw_heartbeat` rather than `heartbeat`; the common topics don't
//! let a board choose yet.

use std::sync::Arc;

use anyhow::{Context, Result};
use canmore::client_ids;
use rclrs::{Clock, DynamicMessage, DynamicSubscription, IntoPrimitiveOptions, MessageInfo, Node};
use titan_link::camera_cage::{self, LedCommand};
use titan_link::{DeviceToHost, HostToDevice};

use super::{BoardProfile, BoardTopics, Transport};
use crate::link::Link;
use crate::ros::{self, Publisher};

pub const PROFILE: BoardProfile = BoardProfile {
    name: "camera_cage",
    node_name: "camera_cage_bb",
    namespace: "talos",
    client_id: client_ids::CLIENT_ID_CAMERA_CAGE_BB,
    transport: Transport::Can { fd: true },
    create: |node, link, logger| Ok(Box::new(CameraCageTopics::new(node, link, logger)?)),
};

/// `depth_variance` of the C board.
const DEPTH_VARIANCE: f32 = 0.003;

pub struct CameraCageTopics {
    depth: Publisher,
    water_temp: Publisher,
    temperature: Publisher,
    humidity: Publisher,
    /// Advertised like the C board, which never published it (its leak code was commented out).
    _leak: Publisher,
    clock: Clock,
    /// `<robot>/pressure_link`
    depth_frame: String,
    _subscriptions: [DynamicSubscription; 3],
}

fn forward(
    link: Arc<dyn Link>,
    logger: String,
    to_msg: impl Fn(&DynamicMessage) -> Result<camera_cage::HostToDevice> + Send + Sync + 'static,
) -> impl Fn(DynamicMessage, MessageInfo) + Send + Sync + 'static {
    move |msg, _info| {
        let result = to_msg(&msg).and_then(|m| link.send(&HostToDevice::CameraCage(m)));
        if let Err(err) = result {
            rclrs::log_warn!(&logger, "Forwarding camera cage command failed: {err:#}");
        }
    }
}

fn led_command(msg: &DynamicMessage) -> Result<camera_cage::HostToDevice> {
    Ok(camera_cage::HostToDevice::LedCommand(LedCommand {
        red: ros::get_u8(msg, "red")?,
        green: ros::get_u8(msg, "green")?,
        blue: ros::get_u8(msg, "blue")?,
        mode: ros::get_u8(msg, "mode")?,
        target: ros::get_u8(msg, "target")?,
    }))
}

impl CameraCageTopics {
    pub fn new(node: &Node, link: Arc<dyn Link>, logger: String) -> Result<Self> {
        let led = node.create_dynamic_subscription(
            "riptide_msgs2/msg/LedCommand".try_into().unwrap(),
            "command/led",
            forward(link.clone(), logger.clone(), led_command),
        )?;
        let physkill_notify = node.create_dynamic_subscription(
            "std_msgs/msg/Bool".try_into().unwrap(),
            "state/physkill_notify",
            forward(link.clone(), logger.clone(), |m| {
                Ok(camera_cage::HostToDevice::PhysKillNotify(ros::get_bool(m, "data")?))
            }),
        )?;
        let electrical = node.create_dynamic_subscription(
            "riptide_msgs2/msg/ElectricalCommand".try_into().unwrap(),
            "command/electrical",
            forward(link, logger, |m| Ok(camera_cage::HostToDevice::ElectricalCommand(ros::get_u8(m, "command")?))),
        )?;

        // Like ROBOT_NAMESPACE "/pressure_link" in C, following --namespace
        let robot = node.namespace().trim_start_matches('/').to_string();
        Ok(CameraCageTopics {
            depth: Publisher::new(node, "riptide_msgs2/msg/Depth", "state/depth/raw".sensor_data_qos())?,
            water_temp: Publisher::new(node, "std_msgs/msg/Float32", "state/depth/temp".sensor_data_qos())?,
            temperature: Publisher::new(node, "std_msgs/msg/Float32", "state/temp/cameracage".best_effort())?,
            humidity: Publisher::new(node, "std_msgs/msg/Float32", "state/humidity/cameracage".best_effort())?,
            _leak: Publisher::new(node, "std_msgs/msg/Bool", "state/leak")?,
            clock: node.get_clock(),
            depth_frame: format!("{robot}/pressure_link"),
            _subscriptions: [led, physkill_notify, electrical],
        })
    }
}

impl BoardTopics for CameraCageTopics {
    fn handle(&self, msg: &DeviceToHost) -> Result<bool> {
        let DeviceToHost::CameraCage(msg) = msg else { return Ok(false) };
        match msg {
            camera_cage::DeviceToHost::Depth(depth) => {
                // Stamped on arrival with ROS time; the C board used the agent-synchronized epoch
                let stamp = self.clock.now().nsec;
                self.depth
                    .publish(|m| {
                        ros::set_header(m, stamp, Some(&self.depth_frame))?;
                        ros::set_f32(m, "depth", *depth)?;
                        ros::set_f32(m, "variance", DEPTH_VARIANCE)
                    })
                    .context("state/depth/raw")?;
            }
            camera_cage::DeviceToHost::WaterTemp(temp) => {
                self.water_temp.publish(|m| ros::set_f32(m, "data", *temp))?;
            }
            camera_cage::DeviceToHost::TempHumidity { temperature_c, humidity_rh } => {
                self.temperature.publish(|m| ros::set_f32(m, "data", *temperature_c))?;
                self.humidity.publish(|m| ros::set_f32(m, "data", *humidity_rh))?;
            }
        }
        Ok(true)
    }
}

#[cfg(test)]
mod tests {
    //! Run inside `riptide-shell`: these load real ROS type support.

    use super::*;
    use anyhow::bail;
    use rclrs::{DynamicMessageMetadata, SimpleValue, Value};

    #[test]
    fn depth_header_is_filled() -> Result<()> {
        let mut msg =
            DynamicMessageMetadata::new("riptide_msgs2/msg/Depth".try_into().unwrap()).unwrap().create().unwrap();
        ros::set_header(&mut msg, 1_700_000_000_123_456_789, Some("talos/pressure_link"))?;
        ros::set_f32(&mut msg, "depth", -1.5)?;
        let Some(Value::Simple(SimpleValue::Message(header))) = msg.get("header") else { bail!("no header") };
        let Some(Value::Simple(SimpleValue::Message(stamp))) = header.get("stamp") else { bail!("no stamp") };
        assert_eq!(stamp.get("sec"), Some(Value::Simple(SimpleValue::Int32(&1_700_000_000))));
        assert_eq!(stamp.get("nanosec"), Some(Value::Simple(SimpleValue::Uint32(&123_456_789))));
        let Some(Value::Simple(SimpleValue::String(frame))) = header.get("frame_id") else { bail!("no frame") };
        assert_eq!(frame.to_string(), "talos/pressure_link");
        assert_eq!(ros::get_f32(&msg, "depth")?, -1.5);
        Ok(())
    }
}
