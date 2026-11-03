//! Lilboat power half board (`lilboat/PoHalfBoard`) messages: the power stage messages plus solenoids, pressure
//! transducers and two depth (pressure) sensors.

use serde::{Deserialize, Serialize};

use crate::power;

/// Solenoid indices used by [`HostToDevice::Solenoid`] and [`DeviceToHost::SolenoidStates`], from
/// `boards/lilboat/po_half.h`.
pub mod solenoid {
    /// `command/solenoid/water`, `state/solenoid/water`
    pub const WATER: u8 = 0;
    /// `command/solenoid/pressure`, `state/solenoid/pressure`
    pub const PRESSURE: u8 = 1;
    /// `command/solenoid/exhaust`, `state/solenoid/exhaust`
    pub const EXHAUST: u8 = 2;
    /// `command/pump`, `state/pump`
    pub const PUMP: u8 = 3;
    /// Number of outputs.
    pub const COUNT: usize = 4;
}

#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HostToDevice {
    /// The power stage: `command/software_kill`.
    Power(power::HostToDevice),
    /// From `command/solenoid/{water,pressure,exhaust}` and `command/pump` (`std_msgs/Bool`).
    Solenoid { index: u8, open: bool },
    /// From `command/depth/recalibrate` (`std_msgs/Int8`): zero depth sensor 0 or 1 again. Other values are ignored.
    DepthRecalibrate(i8),
    /// From `command/fart_for_ms` (`std_msgs/Int32`): run a solenoid cycle with the pressure solenoid open this long.
    SolenoidCycle(i32),
}

#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeviceToHost {
    /// The power stage topics.
    Power(power::DeviceToHost),
    /// For `state/depth/raw` (`riptide_msgs2/Depth::depth`): water depth sensor reading in metres, negative below
    /// the surface. The bridge stamps it on arrival.
    Depth(f32),
    /// For `state/pressure/regulator_housing`: absolute pressure in pascals from the regulator housing sensor.
    RegulatorHousingPressure(f32),
    /// For `state/pressure/tank` and `state/pressure/regulated`: the analog transducer readings (volts).
    AdcPressures { tank: f32, regulated: f32 },
    /// For `state/solenoid/*` and `state/pump`, indexed as in [`solenoid`].
    SolenoidStates([bool; solenoid::COUNT]),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{MAX_FD_FRAME_LEN, decode, encode};

    #[test]
    fn round_trip() {
        let host = [
            HostToDevice::Power(power::HostToDevice::SoftwareKill(power::SoftwareKill {
                kill_switch_id: 1,
                asserting_kill: false,
                needs_update: false,
                sender_id: heapless::Vec::from_slice(b"rqt").unwrap(),
            })),
            HostToDevice::Solenoid { index: solenoid::PUMP, open: true },
            HostToDevice::DepthRecalibrate(-3),
            HostToDevice::SolenoidCycle(-1),
        ];
        for msg in host {
            let msg = crate::HostToDevice::PoHalf(msg);
            let mut buf = [0u8; MAX_FD_FRAME_LEN];
            let encoded = encode(&msg, &mut buf).unwrap();
            assert_eq!(decode::<crate::HostToDevice>(encoded).unwrap(), msg);
        }

        let device = [
            DeviceToHost::Power(power::DeviceToHost::PhysicalKillNotify(true)),
            DeviceToHost::Depth(-1.25),
            DeviceToHost::RegulatorHousingPressure(101_325.0),
            DeviceToHost::AdcPressures { tank: 1.5, regulated: 0.75 },
            DeviceToHost::SolenoidStates([true, false, false, true]),
        ];
        for msg in device {
            let msg = crate::DeviceToHost::PoHalf(msg);
            let mut buf = [0u8; MAX_FD_FRAME_LEN];
            let encoded = encode(&msg, &mut buf).unwrap();
            assert_eq!(decode::<crate::DeviceToHost>(encoded).unwrap(), msg);
        }
    }
}
