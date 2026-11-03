//! IVC board, second revision firmware (`talos/IVCBoard2`): the comms, pinger, storage and debug topics.
//!
//! Also holds the field helpers both IVCBoard2 and IVCDAC need beyond `crate::ros` (`std_msgs/Int32` and
//! `builtin_interfaces/Time` fields).

use std::sync::Arc;

use anyhow::{Result, anyhow};
use canmore::client_ids;
use rclrs::{DynamicMessage, DynamicSubscription, IntoPrimitiveOptions, MessageInfo, Node, SubscriptionOptions};
use titan_link::{DeviceToHost, HostToDevice, ivc2};

use super::{BoardProfile, BoardTopics, Transport};
use crate::link::Link;
use crate::ros::{self, Publisher};

pub const PROFILE: BoardProfile = BoardProfile {
    name: "ivc2",
    node_name: "IVCBoard2",
    namespace: "talos",
    client_id: client_ids::CLIENT_ID_IVC_BOARD,
    transport: Transport::Can { fd: true },
    create: |node, link, logger| Ok(Box::new(Ivc2Topics::new(node, link, logger)?)),
};

/// Publishers the C firmware creates but never publishes on (their publish calls are commented out). Created so the
/// topic list matches.
pub(super) const SILENT_TOPICS: [(&str, &str); 4] = [
    ("std_msgs/msg/Int8", "ivc/adc_dump"),
    ("std_msgs/msg/Float32", "ivc/new_transducer"),
    ("std_msgs/msg/Int8", "ivc/rx_sample_stream"),
    ("std_msgs/msg/Float32", "ivc/rx_sample_snr_stream"),
];

pub struct Ivc2Topics {
    rx_new: Publisher,
    pinger_amplitude: Publisher,
    pinger_freq_khz: Publisher,
    _silent: Vec<Publisher>,
    _subscriptions: Vec<DynamicSubscription>,
}

/// Subscribes to `topic` and forwards the field read by `field` to the board as `to_msg(value)`.
pub(super) fn forward<'a, T>(
    node: &Node,
    type_name: &str,
    options: impl Into<SubscriptionOptions<'a>>,
    link: &Arc<dyn Link>,
    logger: &str,
    field: impl Fn(&DynamicMessage) -> Result<T> + Send + Sync + 'static,
    to_msg: impl Fn(T) -> HostToDevice + Send + Sync + 'static,
) -> Result<DynamicSubscription> {
    let (link, logger) = (link.clone(), logger.to_owned());
    let message_type = type_name.try_into().map_err(|e| anyhow!("{type_name}: {e:?}"))?;
    Ok(node.create_dynamic_subscription(message_type, options, move |msg: DynamicMessage, _: MessageInfo| {
        if let Err(err) = field(&msg).and_then(|value| link.send(&to_msg(value))) {
            rclrs::log_warn!(&logger, "Forwarding IVC command failed: {err:#}");
        }
    })?)
}

impl Ivc2Topics {
    pub fn new(node: &Node, link: Arc<dyn Link>, logger: String) -> Result<Self> {
        let to = |msg| HostToDevice::Ivc2(msg);
        let subscriptions = vec![
            forward(
                node,
                "std_msgs/msg/Int8",
                "ivc/enqueue_data",
                &link,
                &logger,
                |m| ros::get_i8(m, "data"),
                move |v| to(ivc2::HostToDevice::EnqueueData(v as u8)),
            )?,
            forward(
                node,
                "std_msgs/msg/Int32",
                "ivc/pinger/set_freq_khz",
                &link,
                &logger,
                |m| ros::get_i32(m, "data"),
                move |v| to(ivc2::HostToDevice::PingerSetFreqKhz(v)),
            )?,
            forward(
                node,
                "std_msgs/msg/Int8",
                "ivc/storage/configure",
                &link,
                &logger,
                |m| ros::get_i8(m, "data"),
                move |v| to(ivc2::HostToDevice::StorageConfigure(v)),
            )?,
            forward(
                node,
                "std_msgs/msg/Bool",
                "ivc/pinger/enable",
                &link,
                &logger,
                |m| ros::get_bool(m, "data"),
                move |v| to(ivc2::HostToDevice::PingerEnable(v)),
            )?,
            forward(
                node,
                "std_msgs/msg/Float32",
                "ivc/set_min_snr",
                &link,
                &logger,
                |m| ros::get_f32(m, "data"),
                move |v| to(ivc2::HostToDevice::SetMinSnr(v)),
            )?,
            // rclc_subscription_init_best_effort in the C firmware
            forward(
                node,
                "std_msgs/msg/Int8",
                "ivc/debug/send_freq".best_effort(),
                &link,
                &logger,
                |m| ros::get_i8(m, "data"),
                move |v| to(ivc2::HostToDevice::DebugSendFreq(v as u8)),
            )?,
        ];
        Ok(Ivc2Topics {
            rx_new: Publisher::new(node, "std_msgs/msg/Int8", "ivc/rx_new")?,
            pinger_amplitude: Publisher::new(node, "std_msgs/msg/Float32", "ivc/pinger/selected_freq_amp_stream")?,
            pinger_freq_khz: Publisher::new(node, "std_msgs/msg/Int32", "ivc/pinger/selected_freq_khz")?,
            _silent: SILENT_TOPICS
                .iter()
                .map(|&(type_name, topic)| Publisher::new(node, type_name, topic))
                .collect::<Result<_>>()?,
            _subscriptions: subscriptions,
        })
    }
}

impl BoardTopics for Ivc2Topics {
    fn handle(&self, msg: &DeviceToHost) -> Result<bool> {
        let DeviceToHost::Ivc2(msg) = msg else { return Ok(false) };
        match *msg {
            ivc2::DeviceToHost::RxNew(byte) => self.rx_new.publish(|m| ros::set_i8(m, "data", byte as i8))?,
            ivc2::DeviceToHost::PingerAmplitude(amp) => {
                self.pinger_amplitude.publish(|m| ros::set_f32(m, "data", amp))?
            }
            ivc2::DeviceToHost::PingerFreqKhz(khz) => self.pinger_freq_khz.publish(|m| ros::set_i32(m, "data", khz))?,
        }
        Ok(true)
    }
}

// Field helpers missing from `crate::ros`
