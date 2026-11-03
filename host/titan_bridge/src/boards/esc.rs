//! ESC boards (`talos/ESCBoard`): thruster RPM commands, RPM feedback, telemetry and controller tuning.
//!
//! Talos has two ESC boards. Board 0 (client ESC_BOARD0) runs as node `esc_board_0` and drives thrusters 0-3;
//! board 1 (ESC_BOARD1) runs as `esc_board_1` with thrusters 4-7. Both serve the same topics, like the C micro-ROS
//! nodes:
//!
//! | Topic | Type | Direction |
//! |---|---|---|
//! | `command/thruster_rpm` | `riptide_msgs2/DshotCommand` (best effort) | subscribed, all 8 thrusters |
//! | `state/thrusters/rpm` | `riptide_msgs2/DshotRPMFeedback` (best effort) | published after each command |
//! | `state/thrusters/cmd` | `riptide_msgs2/DshotRPMFeedback` (best effort) | published after each command |
//! | `state/thrusters/telemetry` | `riptide_msgs2/DshotPartialTelemetry` (best effort) | published every 500 ms |
//! | `dshot_tune/set_raw` | `std_msgs/Bool` | subscribed |
//! | `dshot_tune/{p_gain,i_gain,i_bound,hard_limit,min_command}` | `std_msgs/Int32` | subscribed |
//!
//! Thruster commands from the robot computer's controller normally bypass ROS entirely: it sends them as CANmore
//! utility frames on channel 0, which the firmware handles itself. `command/thruster_rpm` goes through `titan_link`
//! rather than a utility frame so the board knows to answer with the RPM feedback, as the C board did.

use std::sync::Arc;

use anyhow::{Result, anyhow, bail};
use canmore::client_ids;
use rclrs::{
    ArrayValue, ArrayValueMut, DynamicMessage, DynamicMessageViewMut, DynamicSubscription, IntoPrimitiveOptions,
    MessageInfo, Node, SimpleValueMut, Value, ValueMut,
};
use titan_link::esc::{self, THRUSTERS_PER_BOARD, TOTAL_THRUSTERS};
use titan_link::{DeviceToHost, HostToDevice};

use super::{BoardProfile, BoardTopics, Transport};
use crate::link::Link;
use crate::ros::{self, Publisher};

/// ESC board 0 (thrusters 0-3).
pub const PROFILE: BoardProfile = BoardProfile {
    name: "esc0",
    node_name: "esc_board_0",
    namespace: "talos",
    client_id: client_ids::CLIENT_ID_ESC_BOARD0,
    transport: Transport::Can { fd: true },
    create: |node, link, logger| Ok(Box::new(EscTopics::new(node, link, logger)?)),
};

/// ESC board 1 (thrusters 4-7).
pub const PROFILE_BOARD1: BoardProfile =
    BoardProfile { name: "esc1", node_name: "esc_board_1", client_id: client_ids::CLIENT_ID_ESC_BOARD1, ..PROFILE };

pub struct EscTopics {
    rpm: Publisher,
    cmd: Publisher,
    telemetry: Publisher,
    _subscriptions: Vec<DynamicSubscription>,
}

impl EscTopics {
    pub fn new(node: &Node, link: Arc<dyn Link>, logger: String) -> Result<Self> {
        let forward = |read: fn(&DynamicMessage) -> Result<esc::HostToDevice>| {
            let link = link.clone();
            let logger = logger.clone();
            move |msg: DynamicMessage, _info: MessageInfo| {
                if let Err(err) = read(&msg).and_then(|cmd| link.send(&HostToDevice::Esc(cmd))) {
                    rclrs::log_warn!(&logger, "Forwarding ESC command failed: {err:#}");
                }
            }
        };
        let int32 = || "std_msgs/msg/Int32".try_into().unwrap();
        let subscriptions = vec![
            node.create_dynamic_subscription(
                "riptide_msgs2/msg/DshotCommand".try_into().unwrap(),
                // Matches rclc_subscription_init_best_effort in the C firmware
                "command/thruster_rpm".best_effort(),
                forward(|m| Ok(esc::HostToDevice::ThrusterRpm(get_i16_array(m, "values")?))),
            )?,
            node.create_dynamic_subscription(
                "std_msgs/msg/Bool".try_into().unwrap(),
                "dshot_tune/set_raw",
                forward(|m| Ok(esc::HostToDevice::SetRawMode(ros::get_bool(m, "data")?))),
            )?,
            node.create_dynamic_subscription(
                int32(),
                "dshot_tune/p_gain",
                forward(|m| Ok(esc::HostToDevice::SetPGain(ros::get_i32(m, "data")?))),
            )?,
            node.create_dynamic_subscription(
                int32(),
                "dshot_tune/i_gain",
                forward(|m| Ok(esc::HostToDevice::SetIGain(ros::get_i32(m, "data")?))),
            )?,
            node.create_dynamic_subscription(
                int32(),
                "dshot_tune/i_bound",
                forward(|m| Ok(esc::HostToDevice::SetIBound(ros::get_i32(m, "data")?))),
            )?,
            node.create_dynamic_subscription(
                int32(),
                "dshot_tune/hard_limit",
                forward(|m| Ok(esc::HostToDevice::SetHardLimit(ros::get_i32(m, "data")?))),
            )?,
            node.create_dynamic_subscription(
                int32(),
                "dshot_tune/min_command",
                forward(|m| Ok(esc::HostToDevice::SetMinCommand(ros::get_i32(m, "data")?))),
            )?,
        ];
        Ok(EscTopics {
            rpm: Publisher::new(node, "riptide_msgs2/msg/DshotRPMFeedback", "state/thrusters/rpm".best_effort())?,
            cmd: Publisher::new(node, "riptide_msgs2/msg/DshotRPMFeedback", "state/thrusters/cmd".best_effort())?,
            telemetry: Publisher::new(
                node,
                "riptide_msgs2/msg/DshotPartialTelemetry",
                "state/thrusters/telemetry".best_effort(),
            )?,
            _subscriptions: subscriptions,
        })
    }
}

impl BoardTopics for EscTopics {
    fn handle(&self, msg: &DeviceToHost) -> Result<bool> {
        match msg {
            DeviceToHost::Esc(esc::DeviceToHost::Rpm(feedback)) => {
                let (rpm, cmd) = feedback_arrays(feedback)?;
                let mask = (feedback.valid_mask as u32) << feedback.start_thruster;
                self.rpm.publish(|m| fill_rpm_feedback(m, mask as u8, &rpm))?;
                self.cmd.publish(|m| fill_rpm_feedback(m, mask as u8, &cmd))?;
            }
            DeviceToHost::Esc(esc::DeviceToHost::Telemetry(telemetry)) => {
                self.telemetry.publish(|m| fill_partial_telemetry(m, telemetry))?;
            }
            _ => return Ok(false),
        }
        Ok(true)
    }
}

/// Places one board's RPM and command values into the vehicle wide arrays (`ros_send_rpm`). Like the C board,
/// `rpm` is only filled in where valid, and the command message carries the same mask.
fn feedback_arrays(feedback: &esc::RpmFeedback) -> Result<([i16; TOTAL_THRUSTERS], [i16; TOTAL_THRUSTERS])> {
    let start = feedback.start_thruster as usize;
    if start + THRUSTERS_PER_BOARD > TOTAL_THRUSTERS {
        bail!("RPM feedback for thrusters {start}.. is out of range");
    }
    let mut rpm = [0; TOTAL_THRUSTERS];
    let mut cmd = [0; TOTAL_THRUSTERS];
    for i in 0..THRUSTERS_PER_BOARD {
        if feedback.valid_mask & (1 << i) != 0 {
            rpm[start + i] = feedback.rpm[i];
        }
        cmd[start + i] = feedback.cmd[i];
    }
    Ok((rpm, cmd))
}

fn fill_rpm_feedback(msg: &mut DynamicMessage, valid_mask: u8, values: &[i16; TOTAL_THRUSTERS]) -> Result<()> {
    ros::set_u8(msg, "rpm_valid_mask", valid_mask)?;
    match msg.get_mut("rpm") {
        Some(ValueMut::Array(ArrayValueMut::Int16Array(slot))) if slot.len() == TOTAL_THRUSTERS => {
            slot.copy_from_slice(values);
            Ok(())
        }
        other => bail!("field rpm is {other:?}, expected int16[{TOTAL_THRUSTERS}]"),
    }
}

/// Fills a `riptide_msgs2/DshotPartialTelemetry`, converting units like `ros_send_telemetry`.
fn fill_partial_telemetry(msg: &mut DynamicMessage, t: &esc::Telemetry) -> Result<()> {
    ros::set_bool(msg, "escs_powered", t.escs_powered)?;
    ros::set_bool(msg, "thrusters_moving", t.thrusters_moving)?;
    ros::set_u8(msg, "start_thruster_num", t.start_thruster)?;
    ros::set_u8(msg, "disabled_flags", t.disabled_flags)?;
    ros::set_f32(msg, "vcc_voltage", t.vcc_mv as f32 / 1000.0)?;
    ros::set_u32(msg, "packet_delta_us", t.packet_delta_us)?;
    set(msg, "tick_cnt", Scalar::U16(t.tick_cnt))?;
    match msg.get_mut("esc_telemetry") {
        Some(ValueMut::Array(ArrayValueMut::MessageArray(mut views))) if views.len() == THRUSTERS_PER_BOARD => {
            for (view, esc) in views.iter_mut().zip(&t.escs) {
                fill_single_telemetry(view, esc)?;
            }
            Ok(())
        }
        other => bail!("field esc_telemetry is {other:?}, expected DshotSingleTelemetry[{THRUSTERS_PER_BOARD}]"),
    }
}

fn fill_single_telemetry(msg: &mut DynamicMessageViewMut, e: &esc::EscTelemetry) -> Result<()> {
    set(msg, "esc_online", Scalar::Bool(e.esc_online))?;
    set(msg, "thruster_ready", Scalar::Bool(e.thruster_ready))?;
    set(msg, "temperature_c", Scalar::U8(e.temperature_c))?;
    set(msg, "rpm", Scalar::I16(e.rpm))?;
    set(msg, "voltage", Scalar::F32(e.voltage_cv as f32 / 100.0))?;
    set(msg, "current", Scalar::F32(e.current_ca as f32 / 100.0))?;
    set(msg, "consumption_ah", Scalar::F32(e.consumption_mah as f32 / 1000.0))?;
    set(msg, "dshot_missed_cnt", Scalar::U16(e.dshot_missed_cnt))?;
    set(msg, "dshot_offline_cnt", Scalar::U16(e.dshot_offline_cnt))?;
    set(msg, "uart_success_cnt", Scalar::U16(e.uart_success_cnt))?;
    set(msg, "uart_missed_cnt", Scalar::U16(e.uart_missed_cnt))?;
    set(msg, "uart_disabled_cnt", Scalar::U16(e.uart_disabled_cnt))
}

// Field access for the types `crate::ros` has no helpers for, on both messages and nested message views.

trait MessageFields {
    fn field_mut(&mut self, name: &str) -> Option<ValueMut<'_>>;
}

impl MessageFields for DynamicMessage {
    fn field_mut(&mut self, name: &str) -> Option<ValueMut<'_>> {
        self.get_mut(name)
    }
}

impl MessageFields for DynamicMessageViewMut<'_> {
    fn field_mut(&mut self, name: &str) -> Option<ValueMut<'_>> {
        self.get_mut(name)
    }
}

#[derive(Clone, Copy, Debug)]
enum Scalar {
    Bool(bool),
    U8(u8),
    I16(i16),
    U16(u16),
    F32(f32),
}

fn set(msg: &mut impl MessageFields, name: &str, value: Scalar) -> Result<()> {
    match (msg.field_mut(name), value) {
        (Some(ValueMut::Simple(SimpleValueMut::Boolean(slot))), Scalar::Bool(v)) => *slot = v,
        (Some(ValueMut::Simple(SimpleValueMut::Uint8(slot))), Scalar::U8(v)) => *slot = v,
        (Some(ValueMut::Simple(SimpleValueMut::Int16(slot))), Scalar::I16(v)) => *slot = v,
        (Some(ValueMut::Simple(SimpleValueMut::Uint16(slot))), Scalar::U16(v)) => *slot = v,
        (Some(ValueMut::Simple(SimpleValueMut::Float(slot))), Scalar::F32(v)) => *slot = v,
        (other, value) => bail!("field {name} is {other:?}, cannot store {value:?}"),
    }
    Ok(())
}

fn get_i16_array<const N: usize>(msg: &DynamicMessage, name: &str) -> Result<[i16; N]> {
    match msg.get(name) {
        Some(Value::Array(ArrayValue::Int16Array(values))) => {
            (*values).try_into().map_err(|_| anyhow!("field {name} has {} elements, expected {N}", values.len()))
        }
        other => bail!("field {name} is {other:?}, expected int16[{N}]"),
    }
}

#[cfg(test)]
mod tests {
    //! Run inside `riptide-shell`: these load the real `riptide_msgs2` type support.

    use super::*;
    use rclrs::SimpleValue;
    use rclrs::{DynamicMessageMetadata, DynamicMessageView};

    fn new_message(type_name: &str) -> DynamicMessage {
        DynamicMessageMetadata::new(type_name.try_into().unwrap()).unwrap().create().unwrap()
    }

    fn simple<'a>(view: &'a DynamicMessageView, name: &str) -> SimpleValue<'a> {
        match view.get(name) {
            Some(Value::Simple(v)) => v,
            other => panic!("{name}: {other:?}"),
        }
    }

    #[test]
    fn dshot_command_is_read() {
        let mut msg = new_message("riptide_msgs2/msg/DshotCommand");
        match msg.get_mut("values") {
            Some(ValueMut::Array(ArrayValueMut::Int16Array(slot))) => {
                slot.copy_from_slice(&[1, -2, 3, -4, 5, -6, 7, -999])
            }
            other => panic!("{other:?}"),
        }
        assert_eq!(get_i16_array::<8>(&msg, "values").unwrap(), [1, -2, 3, -4, 5, -6, 7, -999]);
        assert!(get_i16_array::<4>(&msg, "values").is_err());

        let mut msg = new_message("std_msgs/msg/Int32");
        match msg.get_mut("data") {
            Some(ValueMut::Simple(SimpleValueMut::Int32(slot))) => *slot = 100_000,
            other => panic!("{other:?}"),
        }
        assert_eq!(ros::get_i32(&msg, "data").unwrap(), 100_000);
    }

    #[test]
    fn rpm_feedback_lands_in_the_board_half() {
        let feedback =
            esc::RpmFeedback { start_thruster: 4, valid_mask: 0b0101, rpm: [10, 20, 30, 40], cmd: [1, 2, 3, 4] };
        let (rpm, cmd) = feedback_arrays(&feedback).unwrap();
        assert_eq!(rpm, [0, 0, 0, 0, 10, 0, 30, 0]);
        assert_eq!(cmd, [0, 0, 0, 0, 1, 2, 3, 4]);
        assert!(feedback_arrays(&esc::RpmFeedback { start_thruster: 5, ..feedback }).is_err());

        let mut msg = new_message("riptide_msgs2/msg/DshotRPMFeedback");
        fill_rpm_feedback(&mut msg, 0b0101 << 4, &rpm).unwrap();
        match msg.get("rpm") {
            Some(Value::Array(ArrayValue::Int16Array(values))) => assert_eq!(values, &rpm),
            other => panic!("{other:?}"),
        }
        assert_eq!(ros::get_u8(&msg, "rpm_valid_mask").unwrap(), 0b0101_0000);
    }

    #[test]
    fn telemetry_is_converted() {
        let mut escs: [esc::EscTelemetry; 4] = Default::default();
        escs[2] = esc::EscTelemetry {
            esc_online: true,
            thruster_ready: true,
            temperature_c: 41,
            rpm: -500,
            voltage_cv: 1612,
            current_ca: 250,
            consumption_mah: 1234,
            dshot_missed_cnt: 3,
            dshot_offline_cnt: 4,
            uart_success_cnt: 5,
            uart_missed_cnt: 6,
            uart_disabled_cnt: 7,
        };
        let t = esc::Telemetry {
            start_thruster: 4,
            escs_powered: true,
            thrusters_moving: true,
            disabled_flags: 0b100,
            vcc_mv: 16_150,
            packet_delta_us: 500_123,
            tick_cnt: 1234,
            escs,
        };
        let mut msg = new_message("riptide_msgs2/msg/DshotPartialTelemetry");
        fill_partial_telemetry(&mut msg, &t).unwrap();

        assert!(ros::get_bool(&msg, "escs_powered").unwrap());
        assert!(ros::get_bool(&msg, "thrusters_moving").unwrap());
        assert_eq!(ros::get_u8(&msg, "start_thruster_num").unwrap(), 4);
        assert_eq!(ros::get_u8(&msg, "disabled_flags").unwrap(), 0b100);
        assert_eq!(ros::get_f32(&msg, "vcc_voltage").unwrap(), 16.15);
        assert_eq!(ros::get_u32(&msg, "packet_delta_us").unwrap(), 500_123);
        let Some(Value::Array(ArrayValue::MessageArray(views))) = msg.get("esc_telemetry") else { panic!() };
        let e = &views[2];
        assert_eq!(simple(e, "esc_online"), SimpleValue::Boolean(&true));
        assert_eq!(simple(e, "temperature_c"), SimpleValue::Uint8(&41));
        assert_eq!(simple(e, "rpm"), SimpleValue::Int16(&-500));
        assert_eq!(simple(e, "voltage"), SimpleValue::Float(&16.12));
        assert_eq!(simple(e, "current"), SimpleValue::Float(&2.5));
        assert_eq!(simple(e, "consumption_ah"), SimpleValue::Float(&1.234));
        assert_eq!(simple(e, "uart_disabled_cnt"), SimpleValue::Uint16(&7));
        assert_eq!(simple(&views[0], "esc_online"), SimpleValue::Boolean(&false));
        match msg.get("tick_cnt") {
            Some(Value::Simple(SimpleValue::Uint16(v))) => assert_eq!(*v, 1234),
            other => panic!("{other:?}"),
        }
    }
}
