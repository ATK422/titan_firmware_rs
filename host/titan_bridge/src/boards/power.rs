//! Power board (`talos/PowerBoard`), plus the power stage topics it shares with the Poac and PoHalf boards
//! ([`PowerStageTopics`]): the kill state they publish, software kill commands, rail voltages and the SHT41.
//!
//! Topics of the power board, relative to the robot namespace:
//!
//! | Topic | Type | Direction |
//! |---|---|---|
//! | `state/kill` | `std_msgs/Bool` | published |
//! | `state/physkill_notify` | `std_msgs/Bool` | published |
//! | `command/software_kill` | `riptide_msgs2/KillSwitchReport` | subscribed (best effort) |
//! | `state/electrical` | `riptide_msgs2/ElectricalReadings` | published |
//! | `state/batteries_balanced` | `std_msgs/Bool` | published |
//! | `state/aux` | `std_msgs/Bool` | published |
//! | `command/electrical` | `riptide_msgs2/ElectricalCommand` | subscribed |
//! | `state/temp/powerboard` | `std_msgs/Float32` | published (best effort) |
//! | `state/humidity/powerboard` | `std_msgs/Float32` | published (best effort) |
//!
//! The heartbeat goes to `state/fw_heartbeat` like the C power board's (`BoardProfile::heartbeat_topic`). Being the
//! kill switch source, none of the power stage boards subscribed to `state/kill` in C. The common topics in
//! `main.rs` still subscribe for every board, which feeds the board's own kill state back into its kill switch 0
//! (the physical switch on these boards); that subscription must be skipped for them.

use std::sync::Arc;

use anyhow::{Result, bail};
use canmore::client_ids;
use rclrs::{DynamicMessage, DynamicSubscription, IntoPrimitiveOptions, MessageInfo, Node, SubscriptionOptions};
use titan_link::{DeviceToHost, HostToDevice, power};

use super::{BoardProfile, BoardTopics, Transport};
use crate::link::Link;
use crate::ros::{self, Publisher};

pub const PROFILE: BoardProfile = BoardProfile {
    name: "power",
    node_name: "power_board",
    namespace: "talos",
    client_id: client_ids::CLIENT_ID_POWER_BOARD,
    transport: Transport::Can { fd: true },
    create: |node, link, logger| Ok(Box::new(PowerTopics::new(node, link, logger)?)),
};

/// Which power stage topics a board has.
pub(super) struct PowerStage {
    /// `state/temp/<board>`
    pub temp_topic: &'static str,
    /// `state/humidity/<board>`
    pub humidity_topic: &'static str,
    /// `state/aux`, `state/batteries_balanced` and `command/electrical`, which only the Talos boards have.
    pub talos_extras: bool,
    /// Wraps a power stage command into the board's message.
    pub wrap: fn(power::HostToDevice) -> HostToDevice,
}

/// Subscribes to `topic` and forwards each message, converted by `to_msg`, to the board.
pub(super) fn forward<'a>(
    node: &Node,
    type_name: &str,
    topic: impl Into<SubscriptionOptions<'a>>,
    link: Arc<dyn Link>,
    logger: String,
    to_msg: impl Fn(&DynamicMessage) -> Result<HostToDevice> + Send + Sync + 'static,
) -> Result<DynamicSubscription> {
    let type_name = type_name.to_string();
    let what = type_name.clone();
    Ok(node.create_dynamic_subscription(
        type_name.as_str().try_into().map_err(|e| anyhow::anyhow!("{type_name}: {e:?}"))?,
        topic,
        move |msg: DynamicMessage, _info: MessageInfo| {
            if let Err(err) = to_msg(&msg).and_then(|cmd| link.send(&cmd)) {
                rclrs::log_warn!(&logger, "Forwarding {what} failed: {err:#}");
            }
        },
    )?)
}

/// The power stage topics.
pub(super) struct PowerStageTopics {
    kill: Publisher,
    physkill_notify: Publisher,
    electrical: Publisher,
    batteries_balanced: Option<Publisher>,
    aux: Option<Publisher>,
    temp: Publisher,
    humidity: Publisher,
    _subscriptions: Vec<DynamicSubscription>,
}

/// Converts a `riptide_msgs2/KillSwitchReport` into the firmware's software kill command.
fn software_kill(msg: &DynamicMessage) -> Result<power::SoftwareKill> {
    let sender = ros::get_string(msg, "sender_id")?;
    // The firmware refuses IDs of SENDER_ID_LEN bytes or more, so cutting longer ones there keeps the refusal
    let sender = &sender.as_bytes()[..sender.len().min(power::SENDER_ID_LEN)];
    Ok(power::SoftwareKill {
        kill_switch_id: ros::get_u8(msg, "kill_switch_id")?,
        asserting_kill: ros::get_bool(msg, "switch_asserting_kill")?,
        needs_update: ros::get_bool(msg, "switch_needs_update")?,
        sender_id: sender.try_into().expect("cut to capacity"),
    })
}

impl PowerStageTopics {
    pub fn new(node: &Node, link: Arc<dyn Link>, logger: String, stage: &PowerStage) -> Result<Self> {
        let wrap = stage.wrap;
        let mut subscriptions = vec![forward(
            node,
            "riptide_msgs2/msg/KillSwitchReport",
            // Matches rclc_subscription_init_best_effort in the C firmware
            "command/software_kill".best_effort(),
            link.clone(),
            logger.clone(),
            move |m| Ok(wrap(power::HostToDevice::SoftwareKill(software_kill(m)?))),
        )?];
        if stage.talos_extras {
            subscriptions.push(forward(
                node,
                "riptide_msgs2/msg/ElectricalCommand",
                "command/electrical",
                link,
                logger,
                move |m| Ok(wrap(power::HostToDevice::ElectricalCommand(ros::get_u8(m, "command")?))),
            )?);
        }

        let bool_publisher = |topic: &str| Publisher::new(node, "std_msgs/msg/Bool", topic);
        Ok(PowerStageTopics {
            kill: bool_publisher("state/kill")?,
            physkill_notify: bool_publisher("state/physkill_notify")?,
            electrical: Publisher::new(node, "riptide_msgs2/msg/ElectricalReadings", "state/electrical")?,
            batteries_balanced: stage.talos_extras.then(|| bool_publisher("state/batteries_balanced")).transpose()?,
            aux: stage.talos_extras.then(|| bool_publisher("state/aux")).transpose()?,
            temp: Publisher::new(node, "std_msgs/msg/Float32", stage.temp_topic.best_effort())?,
            humidity: Publisher::new(node, "std_msgs/msg/Float32", stage.humidity_topic.best_effort())?,
            _subscriptions: subscriptions,
        })
    }

    pub fn handle(&self, msg: &power::DeviceToHost) -> Result<()> {
        match msg {
            power::DeviceToHost::KillState(killed) => self.kill.publish(|m| ros::set_bool(m, "data", *killed)),
            power::DeviceToHost::PhysicalKillNotify(asserting_kill) => {
                self.physkill_notify.publish(|m| ros::set_bool(m, "data", *asserting_kill))
            }
            power::DeviceToHost::Electrical(r) => {
                self.electrical.publish(|m| {
                    ros::set_f32(m, "port_voltage", r.port_voltage)?;
                    ros::set_f32(m, "stbd_voltage", r.stbd_voltage)?;
                    ros::set_f32(m, "balanced_voltage", r.balanced_voltage)?;
                    ros::set_f32(m, "twelve_volt_voltage", r.twelve_volt_voltage)?;
                    ros::set_f32(m, "five_volt_voltage", r.five_volt_voltage)?;
                    ros::set_f32(m, "three_volt_voltage", r.three_volt_voltage)?;
                    ros::set_bool(m, "stbd_powering", r.stbd_powering)?;
                    ros::set_bool(m, "port_powering", r.port_powering)
                })?;
                // Published right after the readings, true while both batteries are powering
                if let Some(balanced) = &self.batteries_balanced {
                    balanced.publish(|m| ros::set_bool(m, "data", r.port_powering && r.stbd_powering))?;
                }
                Ok(())
            }
            power::DeviceToHost::AuxSwitch(level) => match &self.aux {
                Some(aux) => aux.publish(|m| ros::set_bool(m, "data", *level)),
                None => bail!("this board has no state/aux topic"),
            },
            power::DeviceToHost::TempHumidity { temperature_c, humidity_rh } => {
                self.temp.publish(|m| ros::set_f32(m, "data", *temperature_c))?;
                self.humidity.publish(|m| ros::set_f32(m, "data", *humidity_rh))
            }
        }
    }
}

/// Talos power board topics: just the power stage.
pub struct PowerTopics {
    stage: PowerStageTopics,
}

impl PowerTopics {
    pub fn new(node: &Node, link: Arc<dyn Link>, logger: String) -> Result<Self> {
        let stage = PowerStage {
            temp_topic: "state/temp/powerboard",
            humidity_topic: "state/humidity/powerboard",
            talos_extras: true,
            wrap: HostToDevice::Power,
        };
        Ok(PowerTopics { stage: PowerStageTopics::new(node, link, logger, &stage)? })
    }
}

impl BoardTopics for PowerTopics {
    fn handle(&self, msg: &DeviceToHost) -> Result<bool> {
        match msg {
            DeviceToHost::Power(msg) => self.stage.handle(msg).map(|()| true),
            _ => Ok(false),
        }
    }
}

/// Helpers for the power stage boards' tests. Run inside `riptide-shell`: they use real type support and DDS.
#[cfg(test)]
pub(super) mod test_support {
    use std::sync::Mutex;
    use std::time::Duration;

    use anyhow::Result;
    use rclrs::{Context, CreateBasicExecutor, Executor, IntoNodeOptions, Node, SpinOptions};
    use titan_link::HostToDevice;

    use crate::link::{Event, Link};

    /// Records what the topics send to the board.
    #[derive(Default)]
    pub struct FakeLink {
        pub sent: Mutex<Vec<HostToDevice>>,
    }

    impl Link for FakeLink {
        fn send(&self, msg: &HostToDevice) -> Result<()> {
            self.sent.lock().unwrap().push(msg.clone());
            Ok(())
        }

        fn receive(&self) -> Result<Event> {
            Ok(Event::Idle)
        }
    }

    /// A node in its own namespace, so concurrent tests don't hear each other.
    pub fn node(name: &str) -> Result<(Executor, Node)> {
        let context = Context::default_from_env()?;
        let executor = context.create_basic_executor();
        let node = executor.create_node(name.namespace(format!("titan_bridge_{name}").as_str()))?;
        Ok((executor, node))
    }

    /// Calls `publish` and spins until `done` holds for what the link received (at most about 5 s).
    pub fn spin_until(
        executor: &mut Executor,
        link: &FakeLink,
        publish: impl Fn() -> Result<()>,
        done: impl Fn(&[HostToDevice]) -> bool,
    ) -> Result<Vec<HostToDevice>> {
        for _ in 0..100 {
            publish()?;
            executor.spin(SpinOptions::default().timeout(Duration::from_millis(50)));
            let sent = link.sent.lock().unwrap();
            if done(&sent) {
                return Ok(sent.clone());
            }
        }
        anyhow::bail!("the board never received the expected commands")
    }
}

#[cfg(test)]
mod tests {
    use super::test_support::{FakeLink, node, spin_until};
    use super::*;

    #[test]
    fn commands_are_forwarded() -> Result<()> {
        let (mut executor, node) = node("power")?;
        let link = Arc::new(FakeLink::default());
        let _topics = PowerTopics::new(&node, link.clone(), "test".into())?;
        let kill = Publisher::new(&node, "riptide_msgs2/msg/KillSwitchReport", "command/software_kill".best_effort())?;
        let electrical = Publisher::new(&node, "riptide_msgs2/msg/ElectricalCommand", "command/electrical")?;
        let long_sender = "x".repeat(40);

        let sent = spin_until(
            &mut executor,
            &link,
            || {
                kill.publish(|m| {
                    ros::set_string(m, "sender_id", &long_sender)?;
                    ros::set_u8(m, "kill_switch_id", 2)?;
                    ros::set_bool(m, "switch_asserting_kill", true)?;
                    ros::set_bool(m, "switch_needs_update", true)
                })?;
                electrical.publish(|m| ros::set_u8(m, "command", 6))
            },
            |sent| {
                sent.iter().any(|m| matches!(m, HostToDevice::Power(power::HostToDevice::SoftwareKill(_))))
                    && sent.contains(&HostToDevice::Power(power::HostToDevice::ElectricalCommand(6)))
            },
        )?;
        let kill = sent
            .iter()
            .find_map(|m| match m {
                HostToDevice::Power(power::HostToDevice::SoftwareKill(k)) => Some(k),
                _ => None,
            })
            .unwrap();
        assert_eq!((kill.kill_switch_id, kill.asserting_kill, kill.needs_update), (2, true, true));
        // Cut to the length the firmware refuses, so it still raises its bad command fault
        assert_eq!(kill.sender_id.len(), power::SENDER_ID_LEN);
        Ok(())
    }

    #[test]
    fn every_message_publishes() -> Result<()> {
        let (_executor, node) = node("power_publish")?;
        let topics = PowerTopics::new(&node, Arc::new(FakeLink::default()), "test".into())?;
        let messages = [
            power::DeviceToHost::KillState(true),
            power::DeviceToHost::PhysicalKillNotify(false),
            power::DeviceToHost::Electrical(power::ElectricalReadings {
                balanced_voltage: f32::NAN,
                port_powering: true,
                stbd_powering: true,
                ..Default::default()
            }),
            power::DeviceToHost::AuxSwitch(true),
            power::DeviceToHost::TempHumidity { temperature_c: 30.0, humidity_rh: 45.0 },
        ];
        for msg in messages {
            assert!(topics.handle(&DeviceToHost::Power(msg))?);
        }
        assert!(!topics.handle(&DeviceToHost::Heartbeat)?);
        Ok(())
    }
}
