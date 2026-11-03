//! IVC board with DAC transmitter (`talos/IVCDAC`): tone commands with start/end timestamps and the receive side
//! band magnitude stream, plus the leftover IVCBoard2 topics.
//!
//! The C firmware stamped its messages with the agent-synchronised ROS time (`rmw_uros_epoch_nanos`). The bridge
//! stamps them with its node clock when they arrive, minus the age the firmware reports for capture data; CAN adds
//! about a millisecond.

use std::sync::Arc;

use anyhow::Result;
use canmore::client_ids;
use rclrs::{Clock, DynamicMessage, DynamicSubscription, IntoPrimitiveOptions, MessageInfo, Node, ValueMut};
use titan_link::{DeviceToHost, HostToDevice, ivc_dac};

use super::ivc2::{SILENT_TOPICS, forward};
use super::{BoardProfile, BoardTopics, Transport};
use crate::link::Link;
use crate::ros::{self, Publisher};

pub const PROFILE: BoardProfile = BoardProfile {
    name: "ivc_dac",
    node_name: "ivc_dac",
    namespace: "talos",
    client_id: client_ids::CLIENT_ID_IVC_BOARD,
    transport: Transport::Can { fd: true },
    create: |node, link, logger| Ok(Box::new(IvcDacTopics::new(node, link, logger)?)),
};

/// Type of `ivc_dac/rx_state`. It is not part of the released `riptide_msgs2`; the C firmware filled `stamp` and a
/// three element `freqs` array (low, high, sync).
const RX_STATE_TYPE: &str = "riptide_msgs2/msg/IvcRx";

/// IVCBoard2 publishers that IVCDAC still creates but never publishes on.
const SILENT_IVCBOARD2_TOPICS: [(&str, &str); 3] = [
    ("std_msgs/msg/Int8", "ivc/rx_new"),
    ("std_msgs/msg/Float32", "ivc/pinger/selected_freq_amp_stream"),
    ("std_msgs/msg/Int32", "ivc/pinger/selected_freq_khz"),
];

/// IVCBoard2 subscriptions that IVCDAC still creates but whose state its firmware never reads.
const IGNORED_TOPICS: [(&str, &str); 3] = [
    ("std_msgs/msg/Int8", "ivc/enqueue_data"),
    ("std_msgs/msg/Int8", "ivc/storage/configure"),
    ("std_msgs/msg/Float32", "ivc/set_min_snr"),
];

pub struct IvcDacTopics {
    clock: Clock,
    logger: String,
    tx_start: Publisher,
    tx_end: Publisher,
    /// `None` when the message type is not installed.
    rx_state: Option<Publisher>,
    _silent: Vec<Publisher>,
    _subscriptions: Vec<DynamicSubscription>,
}

impl IvcDacTopics {
    pub fn new(node: &Node, link: Arc<dyn Link>, logger: String) -> Result<Self> {
        let to = |msg| HostToDevice::IvcDac(msg);
        let mut subscriptions = vec![
            // rclc_subscription_init_best_effort in the C firmware
            forward(
                node,
                "std_msgs/msg/Int8",
                "ivc_dac/send_freq".best_effort(),
                &link,
                &logger,
                |m| ros::get_i8(m, "data"),
                move |v| to(ivc_dac::HostToDevice::SendFreq(v as u8)),
            )?,
            forward(
                node,
                "std_msgs/msg/Bool",
                "ivc/pinger/enable",
                &link,
                &logger,
                |m| ros::get_bool(m, "data"),
                move |v| to(ivc_dac::HostToDevice::PingerEnable(v)),
            )?,
            forward(
                node,
                "std_msgs/msg/Int32",
                "ivc/pinger/set_freq_khz",
                &link,
                &logger,
                |m| ros::get_i32(m, "data"),
                move |v| to(ivc_dac::HostToDevice::PingerSetFreqKhz(v)),
            )?,
        ];
        for (type_name, topic) in IGNORED_TOPICS {
            let logger = logger.clone();
            subscriptions.push(node.create_dynamic_subscription(
                type_name.try_into().map_err(|e| anyhow::anyhow!("{type_name}: {e:?}"))?,
                topic,
                move |_: DynamicMessage, _: MessageInfo| {
                    rclrs::log_debug!(&logger, "{topic} has no effect on IVCDAC");
                },
            )?);
        }

        let rx_state = match Publisher::new(node, RX_STATE_TYPE, "ivc_dac/rx_state") {
            Ok(publisher) => Some(publisher),
            Err(err) => {
                rclrs::log_warn!(&logger, "Not publishing ivc_dac/rx_state: {err:#}");
                None
            }
        };

        Ok(IvcDacTopics {
            clock: node.get_clock(),
            tx_start: Publisher::new(node, "builtin_interfaces/msg/Time", "ivc_dac/tx_start")?,
            tx_end: Publisher::new(node, "builtin_interfaces/msg/Time", "ivc_dac/tx_end")?,
            rx_state,
            _silent: SILENT_TOPICS
                .iter()
                .chain(SILENT_IVCBOARD2_TOPICS.iter())
                .map(|&(type_name, topic)| Publisher::new(node, type_name, topic))
                .collect::<Result<_>>()?,
            _subscriptions: subscriptions,
            logger,
        })
    }
}

impl BoardTopics for IvcDacTopics {
    fn handle(&self, msg: &DeviceToHost) -> Result<bool> {
        let DeviceToHost::IvcDac(msg) = msg else { return Ok(false) };
        let now = self.clock.now().nsec;
        match *msg {
            ivc_dac::DeviceToHost::TxStart => self.tx_start.publish(|m| ros::set_time(m, now))?,
            ivc_dac::DeviceToHost::TxEnd => self.tx_end.publish(|m| ros::set_time(m, now))?,
            ivc_dac::DeviceToHost::RxState { low, high, sync, age_us } => match &self.rx_state {
                Some(publisher) => {
                    publisher.publish(|m| fill_rx_state(m, now - age_us as i64 * 1000, [low, high, sync]))?
                }
                None => rclrs::log_debug!(&self.logger, "Dropping rx_state, {RX_STATE_TYPE} is not installed"),
            },
        }
        Ok(true)
    }
}

/// Fills an `IvcRx` like `ros_publish_rx_state`: `freqs` = low, high, sync.
fn fill_rx_state(msg: &mut DynamicMessage, stamp_ns: i64, freqs: [f32; 3]) -> Result<()> {
    ros::set_stamp(msg, "stamp", stamp_ns)?;
    match msg.get_mut("freqs") {
        Some(ValueMut::Array(rclrs::ArrayValueMut::FloatArray(slots))) if slots.len() == freqs.len() => {
            slots.copy_from_slice(&freqs);
            Ok(())
        }
        other => anyhow::bail!("IvcRx freqs is {other:?}, expected float32[3]"),
    }
}

#[cfg(test)]
mod tests {
    //! Run inside `riptide-shell`: these load the real ROS type support.

    use super::*;
    use crate::ros::split_time;
    use rclrs::{DynamicMessageMetadata, SimpleValue, Value};

    fn create(type_name: &str) -> DynamicMessage {
        DynamicMessageMetadata::new(type_name.try_into().unwrap()).unwrap().create().unwrap()
    }

    #[test]
    fn time_fields_reach_builtin_time() -> Result<()> {
        let mut msg = create("builtin_interfaces/msg/Time");
        ros::set_time(&mut msg, 1_700_000_123_456_789_012)?;
        assert!(matches!(msg.get("sec"), Some(Value::Simple(SimpleValue::Int32(1_700_000_123)))));
        assert!(matches!(msg.get("nanosec"), Some(Value::Simple(SimpleValue::Uint32(456_789_012)))));
        Ok(())
    }

    #[test]
    fn stamp_of_stamped_message() -> Result<()> {
        // Header carries a nested builtin_interfaces/Time `stamp`, the same shape IvcRx uses
        let mut msg = create("std_msgs/msg/Header");
        ros::set_stamp(&mut msg, "stamp", 2_500_000_001)?;
        let Some(Value::Simple(SimpleValue::Message(stamp))) = msg.get("stamp") else { panic!("no stamp") };
        assert!(matches!(stamp.get("sec"), Some(Value::Simple(SimpleValue::Int32(2)))));
        assert!(matches!(stamp.get("nanosec"), Some(Value::Simple(SimpleValue::Uint32(500_000_001)))));
        Ok(())
    }

    #[test]
    fn int32_fields() -> Result<()> {
        let mut msg = create("std_msgs/msg/Int32");
        crate::ros::set_i32(&mut msg, "data", -37)?;
        assert_eq!(ros::get_i32(&msg, "data")?, -37);
        Ok(())
    }

    #[test]
    fn negative_times_split_downwards() {
        assert_eq!(split_time(-1), (-1, 999_999_999));
        assert_eq!(split_time(1_000_000_000), (1, 0));
    }
}
