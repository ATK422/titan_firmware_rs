//! Puddles backplane (`puddles/Backplane`), reached over UDP: thrusters, depth, rail voltages, kill switches and
//! power control.
//!
//! | Topic | Type | QoS | Direction |
//! |---|---|---|---|
//! | `command/dshot` | `riptide_msgs2/DshotCommand` | best effort | subscribed |
//! | `state/thrusters/rpm` | `riptide_msgs2/DshotRPMFeedback` | best effort | published |
//! | `command/software_kill` | `riptide_msgs2/KillSwitchReport` | best effort | subscribed |
//! | `state/kill` | `std_msgs/Bool` | best effort | published |
//! | `state/depth/raw` | `riptide_msgs2/Depth` | sensor data | published |
//! | `state/depth/temp` | `std_msgs/Float32` | sensor data | published |
//! | `state/electrical` | `riptide_msgs2/ElectricalReadings` | default | published |
//! | `command/electrical` | `riptide_msgs2/ElectricalCommand` | default | subscribed |
//!
//! `state/fw_heartbeat` and `state/firmware` come from the common topics (`BoardProfile::heartbeat_topic`). The
//! board is the source of `state/kill`, so the common `state/kill` subscription is not created for it
//! (`BoardProfile::subscribes_kill`); the firmware still drops any `KillSwitch` it receives (see
//! `puddles_core::wire`).

use std::sync::Arc;

use anyhow::{Result, bail};
use rclrs::{
    ArrayValue, ArrayValueMut, Clock, DynamicMessage, DynamicSubscription, IntoPrimitiveOptions, MessageInfo, Node,
    Value, ValueMut,
};
use titan_link::puddles_backplane::{
    self, DeviceToHost as Board, ElectricalCommand, ElectricalReadings, NUM_THRUSTERS, RpmFeedback, SoftwareKill,
};
use titan_link::{DeviceToHost, HostToDevice};

use super::{BoardProfile, BoardTopics, Transport};
use crate::link::Link;
use crate::ros::{self, Publisher};

pub const PROFILE: BoardProfile = BoardProfile {
    name: "puddles_backplane",
    node_name: "backplane_firmware",
    namespace: "puddles",
    client_id: 1,
    transport: Transport::Udp,
    create: |node, link, logger| Ok(Box::new(BackplaneTopics::new(node, link, logger)?)),
};

/// `depth_variance` in `ros.c`.
const DEPTH_VARIANCE: f32 = 0.003;

pub struct BackplaneTopics {
    kill: Publisher,
    rpm: Publisher,
    depth: Publisher,
    water_temp: Publisher,
    electrical: Publisher,
    clock: Clock,
    /// `<robot>/pressure_link`, like `ROBOT_NAMESPACE "/pressure_link"`.
    depth_frame: String,
    _subscriptions: [DynamicSubscription; 3],
}

/// Subscription callback that turns a message into a backplane command, logging failures. `None` drops it.
fn forward(
    link: Arc<dyn Link>,
    logger: String,
    what: &'static str,
    to_command: impl Fn(&DynamicMessage) -> Result<Option<puddles_backplane::HostToDevice>> + Send + Sync + 'static,
) -> impl Fn(DynamicMessage, MessageInfo) + Send + Sync + 'static {
    move |msg, _info| {
        let result = to_command(&msg).and_then(|command| match command {
            Some(command) => link.send(&HostToDevice::PuddlesBackplane(command)),
            None => Ok(()),
        });
        if let Err(err) = result {
            rclrs::log_warn!(&logger, "Forwarding {what} failed: {err:#}");
        }
    }
}

impl BackplaneTopics {
    pub fn new(node: &Node, link: Arc<dyn Link>, logger: String) -> Result<Self> {
        let dshot = node.create_dynamic_subscription(
            "riptide_msgs2/msg/DshotCommand".try_into().unwrap(),
            "command/dshot".best_effort(),
            forward(link.clone(), logger.clone(), "thruster command", |m| {
                Ok(Some(puddles_backplane::HostToDevice::Dshot(read_dshot_command(m)?)))
            }),
        )?;
        let software_kill = node.create_dynamic_subscription(
            "riptide_msgs2/msg/KillSwitchReport".try_into().unwrap(),
            "command/software_kill".best_effort(),
            forward(link.clone(), logger.clone(), "software kill", |m| {
                Ok(Some(puddles_backplane::HostToDevice::SoftwareKill(read_software_kill(m)?)))
            }),
        )?;
        let electrical_logger = logger.clone();
        let electrical = node.create_dynamic_subscription(
            "riptide_msgs2/msg/ElectricalCommand".try_into().unwrap(),
            "command/electrical",
            forward(link, logger, "electrical command", move |m| {
                let command = ros::get_u8(m, "command")?;
                let mapped = ElectricalCommand::from_ros(command);
                if mapped.is_none() {
                    // The C firmware silently ignored the commands it doesn't implement
                    rclrs::log_debug!(&electrical_logger, "Ignoring electrical command {command}");
                }
                Ok(mapped.map(puddles_backplane::HostToDevice::Electrical))
            }),
        )?;

        let namespace = node.namespace();
        Ok(BackplaneTopics {
            kill: Publisher::new(node, "std_msgs/msg/Bool", "state/kill".best_effort())?,
            rpm: Publisher::new(node, "riptide_msgs2/msg/DshotRPMFeedback", "state/thrusters/rpm".best_effort())?,
            depth: Publisher::new(node, "riptide_msgs2/msg/Depth", "state/depth/raw".sensor_data_qos())?,
            water_temp: Publisher::new(node, "std_msgs/msg/Float32", "state/depth/temp".sensor_data_qos())?,
            electrical: Publisher::new(node, "riptide_msgs2/msg/ElectricalReadings", "state/electrical")?,
            clock: node.get_clock(),
            depth_frame: format!("{}/pressure_link", namespace.trim_start_matches('/')),
            _subscriptions: [dshot, software_kill, electrical],
        })
    }
}

impl BoardTopics for BackplaneTopics {
    fn handle(&self, msg: &DeviceToHost) -> Result<bool> {
        let DeviceToHost::PuddlesBackplane(msg) = msg else { return Ok(false) };
        match msg {
            Board::Kill(killed) => self.kill.publish(|m| ros::set_bool(m, "data", *killed))?,
            Board::Rpm(feedback) => self.rpm.publish(|m| fill_rpm(m, feedback))?,
            Board::Depth(depth) => {
                let stamp = self.clock.now().nsec;
                self.depth.publish(|m| fill_depth(m, *depth, stamp, &self.depth_frame))?
            }
            Board::WaterTemp(celsius) => self.water_temp.publish(|m| ros::set_f32(m, "data", *celsius))?,
            Board::Electrical(readings) => self.electrical.publish(|m| fill_electrical(m, readings))?,
        }
        Ok(true)
    }
}

/// `DshotCommand.values`.
pub fn read_dshot_command(msg: &DynamicMessage) -> Result<[i16; NUM_THRUSTERS]> {
    match msg.get("values") {
        Some(Value::Array(ArrayValue::Int16Array(values))) => {
            Ok(values.try_into().map_err(|_| anyhow::anyhow!("values has {} elements", values.len()))?)
        }
        other => bail!("values is {other:?}, expected int16[{NUM_THRUSTERS}]"),
    }
}

/// A `KillSwitchReport` as a [`SoftwareKill`]; sender IDs too long for the firmware stay too long.
pub fn read_software_kill(msg: &DynamicMessage) -> Result<SoftwareKill> {
    Ok(puddles_core::kill::software_kill(
        &ros::get_string(msg, "sender_id")?,
        ros::get_u8(msg, "kill_switch_id")?,
        ros::get_bool(msg, "switch_asserting_kill")?,
        ros::get_bool(msg, "switch_needs_update")?,
    ))
}

/// Fills a `DshotRPMFeedback`.
pub fn fill_rpm(msg: &mut DynamicMessage, feedback: &RpmFeedback) -> Result<()> {
    ros::set_u8(msg, "rpm_valid_mask", feedback.valid_mask)?;
    match msg.get_mut("rpm") {
        Some(ValueMut::Array(ArrayValueMut::Int16Array(rpm))) if rpm.len() == NUM_THRUSTERS => {
            rpm.copy_from_slice(&feedback.rpm);
            Ok(())
        }
        other => bail!("rpm is {other:?}, expected int16[{NUM_THRUSTERS}]"),
    }
}

/// Fills a `riptide_msgs2/Depth`: the header stamped with `stamp_ns` (ROS time) and `frame_id`, the depth and the
/// fixed variance. C stamped with `rmw_uros_epoch_nanos`, the agent's clock.
pub fn fill_depth(msg: &mut DynamicMessage, depth: f32, stamp_ns: i64, frame_id: &str) -> Result<()> {
    ros::set_header(msg, stamp_ns, Some(frame_id))?;
    ros::set_f32(msg, "depth", depth)?;
    ros::set_f32(msg, "variance", DEPTH_VARIANCE)
}

/// Fills a `riptide_msgs2/ElectricalReadings`; the fields the backplane doesn't measure stay zero / false.
pub fn fill_electrical(msg: &mut DynamicMessage, readings: &ElectricalReadings) -> Result<()> {
    ros::set_f32(msg, "three_volt_voltage", readings.three_volt_voltage)?;
    ros::set_f32(msg, "five_volt_voltage", readings.five_volt_voltage)?;
    ros::set_f32(msg, "twelve_volt_voltage", readings.twelve_volt_voltage)?;
    ros::set_f32(msg, "balanced_voltage", readings.balanced_voltage)
}

#[cfg(test)]
mod tests {
    //! Run inside `riptide-shell`: the messages go through real `riptide_msgs2` type support and DDS.

    use super::*;
    use std::sync::mpsc;
    use std::time::Duration;

    use rclrs::{Context, CreateBasicExecutor, IntoNodeOptions, SimpleValue, SpinOptions};

    /// Publishes with `fill` until the subscription (which runs `read`) sees a message, then returns what it read.
    fn round_trip<T: Send + 'static>(
        type_name: &str,
        fill: impl Fn(&mut DynamicMessage) -> Result<()>,
        read: impl Fn(&DynamicMessage) -> Result<T> + Send + Sync + 'static,
    ) -> Result<T> {
        let context = Context::default_from_env()?;
        let mut executor = context.create_basic_executor();
        let topic = format!("puddles_round_trip_{}", type_name.replace('/', "_"));
        let node = executor.create_node("puddles_test".namespace("puddles_test"))?;
        let publisher = Publisher::new(&node, type_name, topic.as_str())?;
        let (tx, rx) = mpsc::channel();
        let _sub = node.create_dynamic_subscription(
            type_name.try_into().unwrap(),
            topic.as_str(),
            move |msg: DynamicMessage, _| {
                let _ = tx.send(read(&msg));
            },
        )?;
        for _ in 0..100 {
            publisher.publish(&fill)?;
            executor.spin(SpinOptions::default().timeout(Duration::from_millis(50)));
            if let Ok(got) = rx.try_recv() {
                return got;
            }
        }
        bail!("nothing received on {topic}")
    }

    fn i16_array(msg: &DynamicMessage, field: &str) -> Vec<i16> {
        match msg.get(field) {
            Some(Value::Array(ArrayValue::Int16Array(values))) => values.to_vec(),
            other => panic!("{field} is {other:?}"),
        }
    }

    #[test]
    fn rpm_feedback_reaches_ros() -> Result<()> {
        let feedback = RpmFeedback { valid_mask: 0b1000_0011, rpm: [1500, -1500, 0, 0, 0, 0, 0, i16::MAX] };
        let (mask, rpm) = round_trip(
            "riptide_msgs2/msg/DshotRPMFeedback",
            |m| fill_rpm(m, &feedback),
            |m| Ok((ros::get_u8(m, "rpm_valid_mask")?, i16_array(m, "rpm"))),
        )?;
        assert_eq!(mask, feedback.valid_mask);
        assert_eq!(rpm, feedback.rpm);
        Ok(())
    }

    #[test]
    fn dshot_command_is_read() -> Result<()> {
        let values = [-999, -1, 0, 1, 999, 5, -5, 0];
        let read = round_trip(
            "riptide_msgs2/msg/DshotCommand",
            |m| match m.get_mut("values") {
                Some(ValueMut::Array(ArrayValueMut::Int16Array(slot))) => {
                    slot.copy_from_slice(&values);
                    Ok(())
                }
                other => bail!("{other:?}"),
            },
            read_dshot_command,
        )?;
        assert_eq!(read, values);
        Ok(())
    }

    #[test]
    fn depth_has_header_and_variance() -> Result<()> {
        let stamp_ns = 1_700_000_123_456_789_012;
        let (sec, nanosec, frame, depth, variance) = round_trip(
            "riptide_msgs2/msg/Depth",
            |m| fill_depth(m, -1.5, stamp_ns, "puddles/pressure_link"),
            |m| {
                let Some(Value::Simple(SimpleValue::Message(header))) = m.get("header") else { bail!("header") };
                let Some(Value::Simple(SimpleValue::Message(stamp))) = header.get("stamp") else { bail!("stamp") };
                let sec = match stamp.get("sec") {
                    Some(Value::Simple(SimpleValue::Int32(v))) => *v,
                    _ => bail!("sec"),
                };
                let nanosec = match stamp.get("nanosec") {
                    Some(Value::Simple(SimpleValue::Uint32(v))) => *v,
                    _ => bail!("nanosec"),
                };
                let frame = match header.get("frame_id") {
                    Some(Value::Simple(SimpleValue::String(v))) => v.to_string(),
                    _ => bail!("frame_id"),
                };
                Ok((sec, nanosec, frame, ros::get_f32(m, "depth")?, ros::get_f32(m, "variance")?))
            },
        )?;
        assert_eq!(sec, 1_700_000_123);
        assert_eq!(nanosec, 456_789_012);
        assert_eq!(frame, "puddles/pressure_link");
        assert_eq!(depth, -1.5);
        assert_eq!(variance, DEPTH_VARIANCE);
        Ok(())
    }

    #[test]
    fn software_kill_and_electrical_messages() -> Result<()> {
        let long_sender = "n".repeat(40);
        let kill = round_trip(
            "riptide_msgs2/msg/KillSwitchReport",
            |m| {
                ros::set_string(m, "sender_id", &long_sender)?;
                ros::set_u8(m, "kill_switch_id", 2)?;
                ros::set_bool(m, "switch_asserting_kill", true)?;
                ros::set_bool(m, "switch_needs_update", true)
            },
            read_software_kill,
        )?;
        assert_eq!(kill.kill_switch_id, 2);
        assert!(kill.asserting_kill && kill.needs_update);
        // Cut to the firmware's limit, which it then rejects as too long
        assert_eq!(kill.sender_id.len(), puddles_backplane::SENDER_ID_LEN);

        let readings = ElectricalReadings {
            three_volt_voltage: 3.3,
            five_volt_voltage: f32::NAN,
            twelve_volt_voltage: 12.1,
            balanced_voltage: 16.0,
        };
        let (three, five, twelve, balanced, port) = round_trip(
            "riptide_msgs2/msg/ElectricalReadings",
            |m| fill_electrical(m, &readings),
            |m| {
                Ok((
                    ros::get_f32(m, "three_volt_voltage")?,
                    ros::get_f32(m, "five_volt_voltage")?,
                    ros::get_f32(m, "twelve_volt_voltage")?,
                    ros::get_f32(m, "balanced_voltage")?,
                    ros::get_f32(m, "port_voltage")?,
                ))
            },
        )?;
        assert_eq!((three, twelve, balanced, port), (3.3, 12.1, 16.0, 0.0));
        assert!(five.is_nan());
        Ok(())
    }
}
