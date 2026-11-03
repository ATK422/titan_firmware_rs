//! IVC board (`talos/IVCBoard`): `ivc/tx`, `ivc/rx`, `ivc/debug_tx_frequency` and `ivc/debug_rx_mag`.

use std::sync::Arc;

use anyhow::Result;
use canmore::client_ids;
use rclrs::Node;
use titan_link::DeviceToHost;

use rclrs::{DynamicMessage, DynamicSubscription, MessageInfo};
use titan_link::{HostToDevice, ivc};

use super::{BoardProfile, BoardTopics, Transport};
use crate::link::Link;
use crate::ros::{self, Publisher};

pub const PROFILE: BoardProfile = BoardProfile {
    name: "ivc",
    node_name: "ivc_board",
    namespace: "talos",
    client_id: client_ids::CLIENT_ID_IVC_BOARD,
    transport: Transport::Can { fd: true },
    create: |node, link, logger| Ok(Box::new(IvcTopics::new(node, link, logger)?)),
};

pub struct IvcTopics {
    rx: Publisher,
    debug_rx_mag: Publisher,
    _subscriptions: [DynamicSubscription; 2],
}

fn forward<T>(
    link: Arc<dyn Link>,
    logger: String,
    field: impl Fn(&DynamicMessage) -> Result<T> + Send + Sync + 'static,
    to_msg: impl Fn(T) -> ivc::HostToDevice + Send + Sync + 'static,
) -> impl Fn(DynamicMessage, MessageInfo) + Send + Sync + 'static {
    move |msg, _info| {
        let result = field(&msg).and_then(|value| link.send(&HostToDevice::Ivc(to_msg(value))));
        if let Err(err) = result {
            rclrs::log_warn!(&logger, "Forwarding IVC command failed: {err:#}");
        }
    }
}

impl IvcTopics {
    pub fn new(node: &Node, link: Arc<dyn Link>, logger: String) -> Result<Self> {
        let tx = node.create_dynamic_subscription(
            "std_msgs/msg/Int8".try_into().unwrap(),
            "ivc/tx",
            forward(link.clone(), logger.clone(), |m| ros::get_i8(m, "data"), |b| ivc::HostToDevice::Tx(b as u8)),
        )?;
        let debug_tx_frequency = node.create_dynamic_subscription(
            "std_msgs/msg/Float32".try_into().unwrap(),
            "ivc/debug_tx_frequency",
            forward(link, logger, |m| ros::get_f32(m, "data"), ivc::HostToDevice::DebugTxFrequency),
        )?;
        Ok(IvcTopics {
            rx: Publisher::new(node, "std_msgs/msg/Int8", "ivc/rx")?,
            debug_rx_mag: Publisher::new(node, "std_msgs/msg/Float32", "ivc/debug_rx_mag")?,
            _subscriptions: [tx, debug_tx_frequency],
        })
    }
}

impl BoardTopics for IvcTopics {
    fn handle(&self, msg: &DeviceToHost) -> Result<bool> {
        match msg {
            DeviceToHost::Ivc(ivc::DeviceToHost::Rx(byte)) => {
                self.rx.publish(|m| ros::set_i8(m, "data", *byte as i8))?;
            }
            DeviceToHost::Ivc(ivc::DeviceToHost::RxDebugMagnitude(mag)) => {
                self.debug_rx_mag.publish(|m| ros::set_f32(m, "data", *mag))?;
            }
            _ => return Ok(false),
        }
        Ok(true)
    }
}
