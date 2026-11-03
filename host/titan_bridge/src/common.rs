//! Topics every titan board has: `heartbeat`, `state/firmware` and `state/kill`.

use std::sync::Arc;

use anyhow::Result;
use rclrs::{DynamicMessage, DynamicSubscription, IntoPrimitiveOptions, MessageInfo, Node};
use titan_link::{FirmwareStatus, HostToDevice};

use crate::link::Link;
use crate::ros::{self, Publisher};

pub struct CommonTopics {
    heartbeat: Publisher,
    firmware_status: Publisher,
    /// `None` for boards that publish `state/kill` themselves.
    _kill: Option<DynamicSubscription>,
}

impl CommonTopics {
    /// `subscribe_kill` forwards `state/kill` to the board's ROS kill switch. Boards that are the source of
    /// `state/kill` (they own the physical kill switch) must not get their own state echoed back.
    pub fn new(
        node: &Node,
        link: Arc<dyn Link>,
        logger: String,
        heartbeat_topic: &str,
        subscribe_kill: bool,
    ) -> Result<Self> {
        let kill = subscribe_kill.then(|| {
            node.create_dynamic_subscription(
                "std_msgs/msg/Bool".try_into().unwrap(),
                // Matches rclc_subscription_init_best_effort in the C firmware
                "state/kill".best_effort(),
                move |msg: DynamicMessage, _info: MessageInfo| {
                    let result = ros::get_bool(&msg, "data")
                        .and_then(|asserting_kill| link.send(&HostToDevice::KillSwitch { asserting_kill }));
                    if let Err(err) = result {
                        rclrs::log_warn!(&logger, "Forwarding kill switch failed: {err:#}");
                    }
                },
            )
        });
        let kill = kill.transpose()?;

        Ok(CommonTopics {
            heartbeat: Publisher::new(node, "std_msgs/msg/Int8", heartbeat_topic)?,
            firmware_status: Publisher::new(node, "riptide_msgs2/msg/FirmwareStatus", "state/firmware")?,
            _kill: kill,
        })
    }

    /// Publishes the heartbeat as the board's client ID.
    pub fn publish_heartbeat(&self, client_id: u8) -> Result<()> {
        self.heartbeat.publish(|msg| ros::set_i8(msg, "data", client_id as i8))
    }

    pub fn publish_firmware_status(&self, status: &FirmwareStatus) -> Result<()> {
        self.firmware_status.publish(|msg| fill_firmware_status(msg, status))
    }
}

/// Copies a [`FirmwareStatus`] into a `riptide_msgs2/msg/FirmwareStatus`.
pub fn fill_firmware_status(msg: &mut DynamicMessage, status: &FirmwareStatus) -> Result<()> {
    ros::set_string(msg, "board_name", &status.board_name)?;
    ros::set_u8(msg, "bus_id", status.bus_id)?;
    ros::set_u8(msg, "client_id", status.client_id)?;
    ros::set_u8(msg, "version_major", status.version_major)?;
    ros::set_u8(msg, "version_minor", status.version_minor)?;
    ros::set_u8(msg, "version_release_type", status.version_release_type)?;
    ros::set_u32(msg, "uptime_ms", status.uptime_ms)?;
    ros::set_u32(msg, "faults", status.faults)?;
    ros::set_u32(msg, "kill_switches_enabled", status.kill_switches_enabled)?;
    ros::set_u32(msg, "kill_switches_asserting_kill", status.kill_switches_asserting_kill)?;
    ros::set_u32(msg, "kill_switches_needs_update", status.kill_switches_needs_update)?;
    ros::set_u32(msg, "kill_switches_timed_out", status.kill_switches_timed_out)
}

#[cfg(test)]
mod tests {
    //! Run inside `riptide-shell`: these go through real ROS type support and DDS, in process.

    use super::*;
    use std::sync::mpsc;
    use std::time::Duration;

    use anyhow::bail;
    use rclrs::{Context, CreateBasicExecutor, IntoNodeOptions, SpinOptions};

    fn read_status(msg: &DynamicMessage) -> Result<FirmwareStatus> {
        Ok(FirmwareStatus {
            board_name: ros::get_string(msg, "board_name")?.as_str().try_into().map_err(|_| anyhow::anyhow!("name"))?,
            bus_id: ros::get_u8(msg, "bus_id")?,
            client_id: ros::get_u8(msg, "client_id")?,
            version_major: ros::get_u8(msg, "version_major")?,
            version_minor: ros::get_u8(msg, "version_minor")?,
            version_release_type: ros::get_u8(msg, "version_release_type")?,
            uptime_ms: ros::get_u32(msg, "uptime_ms")?,
            faults: ros::get_u32(msg, "faults")?,
            kill_switches_enabled: ros::get_u32(msg, "kill_switches_enabled")?,
            kill_switches_asserting_kill: ros::get_u32(msg, "kill_switches_asserting_kill")?,
            kill_switches_needs_update: ros::get_u32(msg, "kill_switches_needs_update")?,
            kill_switches_timed_out: ros::get_u32(msg, "kill_switches_timed_out")?,
        })
    }

    /// Publishes with `fill` until the subscription (which runs `read`) sees a message, then returns what it read.
    fn round_trip<T: Send + 'static>(
        type_name: &str,
        fill: impl Fn(&mut DynamicMessage) -> Result<()>,
        read: impl Fn(&DynamicMessage) -> Result<T> + Send + Sync + 'static,
    ) -> Result<T> {
        let context = Context::default_from_env()?;
        let mut executor = context.create_basic_executor();
        let topic = format!("round_trip_{}", type_name.replace('/', "_"));
        let node = executor.create_node("titan_bridge_test".namespace("titan_bridge_test"))?;
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

    #[test]
    fn firmware_status_reaches_ros_intact() -> Result<()> {
        let status = FirmwareStatus {
            board_name: "talos/ivc_board".try_into().unwrap(),
            bus_id: 1,
            client_id: 8,
            version_major: 1,
            version_minor: 0,
            version_release_type: 2,
            uptime_ms: 123_456,
            faults: 0b1_0001,
            kill_switches_enabled: 1,
            kill_switches_asserting_kill: 0,
            kill_switches_needs_update: 1,
            kill_switches_timed_out: 0,
        };
        let got = round_trip("riptide_msgs2/msg/FirmwareStatus", |m| fill_firmware_status(m, &status), read_status)?;
        assert_eq!(got, status);
        Ok(())
    }

    #[test]
    fn ivc_byte_survives_int8() -> Result<()> {
        // ivc/tx and ivc/rx carry raw bytes in a signed Int8, as in the C firmware
        let got =
            round_trip("std_msgs/msg/Int8", |m| ros::set_i8(m, "data", 0xA5u8 as i8), |m| ros::get_i8(m, "data"))?;
        assert_eq!(got as u8, 0xA5);
        let got =
            round_trip("std_msgs/msg/Float32", |m| ros::set_f32(m, "data", 18_000.0), |m| ros::get_f32(m, "data"))?;
        assert_eq!(got, 18_000.0);
        let got = round_trip("std_msgs/msg/Bool", |m| ros::set_bool(m, "data", true), |m| ros::get_bool(m, "data"))?;
        assert!(got);
        Ok(())
    }
}
