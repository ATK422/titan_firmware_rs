//! LED board (`talos/LEDBoard`): `command/led`, `state/physkill_notify` and `state/depth/raw`, all subscribed.
//!
//! Messages are forwarded field for field; the firmware does the target filtering and mode mapping, as the C
//! executor callbacks did.

use std::sync::Arc;

use anyhow::Result;
use canmore::client_ids;
use rclrs::{DynamicMessage, DynamicSubscription, IntoPrimitiveOptions, MessageInfo, Node};
use titan_link::{DeviceToHost, HostToDevice, led};

use super::{BoardProfile, BoardTopics, Transport};
use crate::link::Link;
use crate::ros;

pub const PROFILE: BoardProfile = BoardProfile {
    name: "led",
    node_name: "led_board",
    namespace: "talos",
    client_id: client_ids::CLIENT_ID_LED_BOARD,
    transport: Transport::Can { fd: true },
    create: |node, link, logger| Ok(Box::new(LedTopics::new(node, link, logger)?)),
};

pub struct LedTopics {
    _subscriptions: [DynamicSubscription; 3],
}

/// Reads a `riptide_msgs2/msg/LedCommand`.
fn led_command(msg: &DynamicMessage) -> Result<led::LedCommand> {
    Ok(led::LedCommand {
        red: ros::get_u8(msg, "red")?,
        green: ros::get_u8(msg, "green")?,
        blue: ros::get_u8(msg, "blue")?,
        mode: ros::get_u8(msg, "mode")?,
        target: ros::get_u8(msg, "target")?,
    })
}

/// A subscription callback that converts each message with `to_msg` and sends it to the board.
fn forward(
    link: Arc<dyn Link>,
    logger: String,
    to_msg: impl Fn(&DynamicMessage) -> Result<led::HostToDevice> + Send + Sync + 'static,
) -> impl Fn(DynamicMessage, MessageInfo) + Send + Sync + 'static {
    move |msg, _info| {
        let result = to_msg(&msg).and_then(|m| link.send(&HostToDevice::Led(m)));
        if let Err(err) = result {
            rclrs::log_warn!(&logger, "Forwarding LED board command failed: {err:#}");
        }
    }
}

impl LedTopics {
    pub fn new(node: &Node, link: Arc<dyn Link>, logger: String) -> Result<Self> {
        let command = node.create_dynamic_subscription(
            "riptide_msgs2/msg/LedCommand".try_into().unwrap(),
            "command/led",
            forward(link.clone(), logger.clone(), |m| Ok(led::HostToDevice::Command(led_command(m)?))),
        )?;
        let physkill_notify = node.create_dynamic_subscription(
            "std_msgs/msg/Bool".try_into().unwrap(),
            "state/physkill_notify",
            forward(link.clone(), logger.clone(), |m| Ok(led::HostToDevice::PhysKillNotify(ros::get_bool(m, "data")?))),
        )?;
        let depth = node.create_dynamic_subscription(
            "riptide_msgs2/msg/Depth".try_into().unwrap(),
            // Matches rclc_subscription_init_best_effort in the C firmware
            "state/depth/raw".best_effort(),
            forward(link, logger, |m| Ok(led::HostToDevice::Depth(ros::get_f32(m, "depth")?))),
        )?;
        Ok(LedTopics { _subscriptions: [command, physkill_notify, depth] })
    }
}

impl BoardTopics for LedTopics {
    fn handle(&self, _msg: &DeviceToHost) -> Result<bool> {
        // The board publishes only the common topics
        Ok(false)
    }
}

#[cfg(test)]
mod tests {
    //! Run inside `riptide-shell`: these go through real ROS type support and DDS, in process.

    use super::*;
    use std::sync::Mutex;
    use std::time::Duration;

    use anyhow::bail;
    use rclrs::{Context, CreateBasicExecutor, IntoNodeOptions, SpinOptions};

    use crate::link::Event;
    use crate::ros::Publisher;

    /// Records what the topics send to the board.
    #[derive(Default)]
    struct RecordingLink(Mutex<Vec<HostToDevice>>);

    impl Link for RecordingLink {
        fn send(&self, msg: &HostToDevice) -> Result<()> {
            self.0.lock().unwrap().push(msg.clone());
            Ok(())
        }

        fn receive(&self) -> Result<Event> {
            Ok(Event::Idle)
        }
    }

    #[test]
    fn subscriptions_forward_to_the_board() -> Result<()> {
        let context = Context::default_from_env()?;
        let mut executor = context.create_basic_executor();
        let node = executor.create_node("led_board_test".namespace("titan_bridge_led_test"))?;
        let link = Arc::new(RecordingLink::default());
        let _topics = LedTopics::new(&node, link.clone(), "led_test".into())?;

        let command = Publisher::new(&node, "riptide_msgs2/msg/LedCommand", "command/led")?;
        let physkill = Publisher::new(&node, "std_msgs/msg/Bool", "state/physkill_notify")?;
        let depth = Publisher::new(&node, "riptide_msgs2/msg/Depth", "state/depth/raw".best_effort())?;

        let want_command = led::LedCommand { red: 255, green: 100, blue: 7, mode: 3, target: 2 };
        let want = [
            HostToDevice::Led(led::HostToDevice::Command(want_command)),
            HostToDevice::Led(led::HostToDevice::PhysKillNotify(true)),
            HostToDevice::Led(led::HostToDevice::Depth(-1.5)),
        ];
        for _ in 0..100 {
            command.publish(|m| {
                ros::set_u8(m, "red", want_command.red)?;
                ros::set_u8(m, "green", want_command.green)?;
                ros::set_u8(m, "blue", want_command.blue)?;
                ros::set_u8(m, "mode", want_command.mode)?;
                ros::set_u8(m, "target", want_command.target)
            })?;
            physkill.publish(|m| ros::set_bool(m, "data", true))?;
            depth.publish(|m| ros::set_f32(m, "depth", -1.5))?;
            executor.spin(SpinOptions::default().timeout(Duration::from_millis(50)));
            let got = link.0.lock().unwrap();
            if want.iter().all(|w| got.contains(w)) {
                assert!(got.iter().all(|g| want.contains(g)), "unexpected message in {got:?}");
                return Ok(());
            }
        }
        bail!("got only {:?}", link.0.lock().unwrap())
    }
}
