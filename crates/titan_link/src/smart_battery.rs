//! Smart battery housing MCU messages, `talos/SmartBattery`.
//!
//! The board sits on the external classic CAN bus, so every frame carries only 8 bytes; the messages keep integer
//! units and let the bridge convert to the float fields of the ROS messages.

use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HostToDevice {
    /// From `command/electrical` `CYCLE_ROBOT`: turn the robot off for [`ROBOT_POWER_CYCLE_TIME_MS`] through an
    /// emergency FET shutdown, then back on.
    PowerCycleRobot,
    /// From `command/electrical` `KILL_ROBOT_POWER`: latch the robot off through an emergency FET shutdown.
    KillRobotPower,
}

/// Length of a robot power cycle (`ROBOT_POWER_CYCLE_TIME_MS`).
pub const ROBOT_POWER_CYCLE_TIME_MS: u32 = 10_000;

/// Which side of the robot the pack is plugged into (`riptide_msgs2/BatteryStatus` `DETECT_*`).
#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Detect {
    /// Unknown: the pack isn't in use or the side detect read failed.
    None = 0,
    Port = 1,
    Stbd = 2,
}

/// For `state/battery` (`riptide_msgs2/BatteryStatus`).
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BatteryStatus {
    /// Gauge serial number.
    pub serial: u16,
    pub detect: Detect,
    /// Battery voltage in mV (`pack_voltage`, V in ROS).
    pub pack_voltage_mv: u16,
    /// Current in mA, negative while discharging (`pack_current`, A in ROS).
    pub pack_current_ma: i32,
    /// Average current in mA (`average_current`, A in ROS).
    pub average_current_ma: i32,
    /// Minutes to empty while discharging, to full while charging; 65535 if unknown (`time_to_dischg`).
    pub time_to_dischg: u16,
    /// Relative state of charge in percent.
    pub soc: u8,
}

#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeviceToHost {
    /// Every second while the pack's manufacturer info is known.
    BatteryStatus(BatteryStatus),
    /// Each new SHT41 reading, for `status/temp/smartbattery` and `status/humidity/smartbattery`.
    TempHumidity { temperature_c: f32, humidity_rh: f32 },
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{MAX_ENCODED_LEN, decode, encode};

    #[test]
    fn round_trip() {
        let device = [
            crate::DeviceToHost::SmartBattery(DeviceToHost::BatteryStatus(BatteryStatus {
                serial: 5,
                detect: Detect::Stbd,
                pack_voltage_mv: 18_400,
                pack_current_ma: -12_345,
                average_current_ma: i32::MIN,
                time_to_dischg: u16::MAX,
                soc: 100,
            })),
            crate::DeviceToHost::SmartBattery(DeviceToHost::TempHumidity { temperature_c: 24.5, humidity_rh: 40.25 }),
        ];
        for msg in device {
            let mut buf = [0u8; MAX_ENCODED_LEN];
            let encoded = encode(&msg, &mut buf).unwrap();
            // Classic CAN: keep the periodic messages to a few frames
            assert!(encoded.len() <= 24, "{} bytes", encoded.len());
            assert_eq!(decode::<crate::DeviceToHost>(encoded).unwrap(), msg);
        }
        for msg in [HostToDevice::PowerCycleRobot, HostToDevice::KillRobotPower] {
            let msg = crate::HostToDevice::SmartBattery(msg);
            let mut buf = [0u8; MAX_ENCODED_LEN];
            let encoded = encode(&msg, &mut buf).unwrap();
            assert_eq!(decode::<crate::HostToDevice>(encoded).unwrap(), msg);
        }
    }
}
