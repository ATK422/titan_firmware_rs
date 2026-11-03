//! Lilboat power half board (`lilboat/PoHalfBoard`): the power stage topics plus solenoids, pressure and depth.
//!
//! | Topic | Type | Direction |
//! |---|---|---|
//! | `state/kill`, `state/physkill_notify`, `command/software_kill`, `state/electrical` | see [`super::power`] | |
//! | `state/temp/pohalf`, `state/humidity/pohalf` | `std_msgs/Float32` | published (best effort) |
//! | `command/solenoid/{water,pressure,exhaust}`, `command/pump` | `std_msgs/Bool` | subscribed (best effort) |
//! | `state/solenoid/{water,pressure,exhaust}`, `state/pump` | `std_msgs/Bool` | published |
//! | `command/fart_for_ms` | `std_msgs/Int32` | subscribed |
//! | `state/pressure/tank`, `state/pressure/regulated` | `std_msgs/Float32` | published |
//! | `state/depth/raw` | `riptide_msgs2/Depth` | published (sensor data QoS) |
//! | `state/pressure/regulator_housing` | `std_msgs/Float32` | published |
//! | `command/depth/recalibrate` | `std_msgs/Int8` | subscribed |

use std::sync::Arc;

use anyhow::Result;
use canmore::client_ids;
use rclrs::{DynamicSubscription, IntoPrimitiveOptions, Node};
use titan_link::po_half::{self, solenoid};
use titan_link::{DeviceToHost, HostToDevice};

use super::power::{PowerStage, PowerStageTopics, forward};
use super::{BoardProfile, BoardTopics, Transport};
use crate::link::Link;
use crate::ros::{self, Publisher};

pub const PROFILE: BoardProfile = BoardProfile {
    name: "po_half",
    node_name: "pohalf_board",
    namespace: "liltank",
    client_id: client_ids::CLIENT_ID_POWER_BOARD,
    transport: Transport::Can { fd: true },
    create: |node, link, logger| Ok(Box::new(PoHalfTopics::new(node, link, logger)?)),
};

/// Command and state topic names of each output, indexed as in [`solenoid`].
const SOLENOID_TOPICS: [(&str, &str); solenoid::COUNT] = [
    ("command/solenoid/water", "state/solenoid/water"),
    ("command/solenoid/pressure", "state/solenoid/pressure"),
    ("command/solenoid/exhaust", "state/solenoid/exhaust"),
    ("command/pump", "state/pump"),
];

pub struct PoHalfTopics {
    stage: PowerStageTopics,
    solenoid_states: Vec<Publisher>,
    tank_pressure: Publisher,
    regulated_pressure: Publisher,
    depth: Publisher,
    regulator_housing_pressure: Publisher,
    _subscriptions: Vec<DynamicSubscription>,
}

impl PoHalfTopics {
    pub fn new(node: &Node, link: Arc<dyn Link>, logger: String) -> Result<Self> {
        let stage = PowerStage {
            temp_topic: "state/temp/pohalf",
            humidity_topic: "state/humidity/pohalf",
            talos_extras: false,
            wrap: |msg| HostToDevice::PoHalf(po_half::HostToDevice::Power(msg)),
        };
        let stage = PowerStageTopics::new(node, link.clone(), logger.clone(), &stage)?;

        let mut subscriptions = Vec::new();
        let mut solenoid_states = Vec::new();
        for (index, (command, state)) in SOLENOID_TOPICS.into_iter().enumerate() {
            subscriptions.push(forward(
                node,
                "std_msgs/msg/Bool",
                // Matches rclc_subscription_init_best_effort in the C firmware
                command.best_effort(),
                link.clone(),
                logger.clone(),
                move |m| {
                    let open = ros::get_bool(m, "data")?;
                    Ok(HostToDevice::PoHalf(po_half::HostToDevice::Solenoid { index: index as u8, open }))
                },
            )?);
            solenoid_states.push(Publisher::new(node, "std_msgs/msg/Bool", state)?);
        }
        subscriptions.push(forward(
            node,
            "std_msgs/msg/Int8",
            "command/depth/recalibrate",
            link.clone(),
            logger.clone(),
            |m| Ok(HostToDevice::PoHalf(po_half::HostToDevice::DepthRecalibrate(ros::get_i8(m, "data")?))),
        )?);
        subscriptions.push(forward(node, "std_msgs/msg/Int32", "command/fart_for_ms", link, logger, |m| {
            Ok(HostToDevice::PoHalf(po_half::HostToDevice::SolenoidCycle(ros::get_i32(m, "data")?)))
        })?);

        Ok(PoHalfTopics {
            stage,
            solenoid_states,
            tank_pressure: Publisher::new(node, "std_msgs/msg/Float32", "state/pressure/tank")?,
            regulated_pressure: Publisher::new(node, "std_msgs/msg/Float32", "state/pressure/regulated")?,
            // rmw_qos_profile_sensor_data in C
            depth: Publisher::new(node, "riptide_msgs2/msg/Depth", "state/depth/raw".sensor_data_qos())?,
            regulator_housing_pressure: Publisher::new(
                node,
                "std_msgs/msg/Float32",
                "state/pressure/regulator_housing",
            )?,
            _subscriptions: subscriptions,
        })
    }
}

impl BoardTopics for PoHalfTopics {
    fn handle(&self, msg: &DeviceToHost) -> Result<bool> {
        let DeviceToHost::PoHalf(msg) = msg else { return Ok(false) };
        match msg {
            po_half::DeviceToHost::Power(msg) => self.stage.handle(msg)?,
            po_half::DeviceToHost::Depth(depth) => {
                // frame_id and variance stay empty/0, as the C board left them
                self.depth.publish(|m| {
                    ros::set_header(m, ros::now_nanoseconds(), None)?;
                    ros::set_f32(m, "depth", *depth)
                })?;
            }
            po_half::DeviceToHost::RegulatorHousingPressure(pa) => {
                self.regulator_housing_pressure.publish(|m| ros::set_f32(m, "data", *pa))?;
            }
            po_half::DeviceToHost::AdcPressures { tank, regulated } => {
                self.tank_pressure.publish(|m| ros::set_f32(m, "data", *tank))?;
                self.regulated_pressure.publish(|m| ros::set_f32(m, "data", *regulated))?;
            }
            po_half::DeviceToHost::SolenoidStates(states) => {
                for (publisher, open) in self.solenoid_states.iter().zip(states) {
                    publisher.publish(|m| ros::set_bool(m, "data", *open))?;
                }
            }
        }
        Ok(true)
    }
}

#[cfg(test)]
mod tests {
    //! Run inside `riptide-shell`: these go through real ROS type support.

    use super::super::power::test_support::{FakeLink, node, spin_until};
    use super::*;
    use anyhow::bail;
    use rclrs::DynamicMessageMetadata;
    use rclrs::{DynamicMessage, SimpleValue, SimpleValueMut, Value, ValueMut};
    use titan_link::power;

    fn create(type_name: &str) -> DynamicMessage {
        DynamicMessageMetadata::new(type_name.try_into().unwrap()).unwrap().create().unwrap()
    }

    #[test]
    fn depth_stamp_and_value() -> Result<()> {
        let mut msg = create("riptide_msgs2/msg/Depth");
        ros::set_header(&mut msg, 1_700_000_000_123_456_789, None)?;
        ros::set_f32(&mut msg, "depth", -1.5)?;
        let Some(Value::Simple(SimpleValue::Message(header))) = msg.get("header") else { bail!("no header") };
        let Some(Value::Simple(SimpleValue::Message(stamp))) = header.get("stamp") else { bail!("no stamp") };
        assert_eq!(stamp.get("sec"), Some(Value::Simple(SimpleValue::Int32(&1_700_000_000))));
        assert_eq!(stamp.get("nanosec"), Some(Value::Simple(SimpleValue::Uint32(&123_456_789))));
        assert_eq!(ros::get_f32(&msg, "depth")?, -1.5);
        Ok(())
    }

    #[test]
    fn int32_field() -> Result<()> {
        let msg = create("std_msgs/msg/Int32");
        assert_eq!(ros::get_i32(&msg, "data")?, 0);
        Ok(())
    }

    #[test]
    fn commands_are_forwarded() -> Result<()> {
        let (mut executor, node) = node("po_half")?;
        let link = Arc::new(FakeLink::default());
        let _topics = PoHalfTopics::new(&node, link.clone(), "test".into())?;
        let pump = Publisher::new(&node, "std_msgs/msg/Bool", "command/pump".best_effort())?;
        let exhaust = Publisher::new(&node, "std_msgs/msg/Bool", "command/solenoid/exhaust".best_effort())?;
        let cycle = Publisher::new(&node, "std_msgs/msg/Int32", "command/fart_for_ms")?;
        let recalibrate = Publisher::new(&node, "std_msgs/msg/Int8", "command/depth/recalibrate")?;

        let expected = [
            HostToDevice::PoHalf(po_half::HostToDevice::Solenoid { index: solenoid::PUMP, open: true }),
            HostToDevice::PoHalf(po_half::HostToDevice::Solenoid { index: solenoid::EXHAUST, open: false }),
            HostToDevice::PoHalf(po_half::HostToDevice::SolenoidCycle(-1500)),
            HostToDevice::PoHalf(po_half::HostToDevice::DepthRecalibrate(1)),
        ];
        spin_until(
            &mut executor,
            &link,
            || {
                pump.publish(|m| ros::set_bool(m, "data", true))?;
                exhaust.publish(|m| ros::set_bool(m, "data", false))?;
                cycle.publish(|m| match m.get_mut("data") {
                    Some(ValueMut::Simple(SimpleValueMut::Int32(v))) => {
                        *v = -1500;
                        Ok(())
                    }
                    _ => bail!("no data"),
                })?;
                recalibrate.publish(|m| ros::set_i8(m, "data", 1))
            },
            |sent| expected.iter().all(|e| sent.contains(e)),
        )?;
        Ok(())
    }

    #[test]
    fn every_message_publishes() -> Result<()> {
        let (_executor, node) = node("po_half_publish")?;
        let topics = PoHalfTopics::new(&node, Arc::new(FakeLink::default()), "test".into())?;
        let messages = [
            po_half::DeviceToHost::Power(power::DeviceToHost::KillState(false)),
            po_half::DeviceToHost::Power(power::DeviceToHost::Electrical(Default::default())),
            po_half::DeviceToHost::Power(power::DeviceToHost::TempHumidity { temperature_c: 1.0, humidity_rh: 2.0 }),
            po_half::DeviceToHost::Depth(-0.5),
            po_half::DeviceToHost::RegulatorHousingPressure(101_325.0),
            po_half::DeviceToHost::AdcPressures { tank: 1.0, regulated: 0.5 },
            po_half::DeviceToHost::SolenoidStates([true, false, true, false]),
        ];
        for msg in messages {
            assert!(topics.handle(&DeviceToHost::PoHalf(msg))?);
        }
        // No aux switch on this board
        let aux = DeviceToHost::PoHalf(po_half::DeviceToHost::Power(power::DeviceToHost::AuxSwitch(true)));
        assert!(topics.handle(&aux).is_err());
        Ok(())
    }
}
