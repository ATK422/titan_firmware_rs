//! Power board (`talos/PowerBoard`) messages.
//!
//! The power stage messages here are shared with the other boards that act as the vehicle's kill switch source:
//! [`crate::poac`] and [`crate::po_half`] carry them in their `Power` variants.

use serde::{Deserialize, Serialize};

/// Longest `sender_id` forwarded in a [`SoftwareKill`]. The firmware refuses IDs of this length or longer
/// (`SAFETY_SOFTWARE_KILL_FRAME_STR_SIZE`), so the bridge forwards longer ones truncated to exactly this length.
pub const SENDER_ID_LEN: usize = 32;

/// From `command/software_kill` (`riptide_msgs2/KillSwitchReport`).
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SoftwareKill {
    pub kill_switch_id: u8,
    pub asserting_kill: bool,
    pub needs_update: bool,
    /// The sender's ID bytes, truncated to [`SENDER_ID_LEN`].
    pub sender_id: heapless::Vec<u8, SENDER_ID_LEN>,
}

/// For `state/electrical`: field for field copy of `riptide_msgs2/ElectricalReadings`. Rails that haven't been
/// read are NaN; boards without battery monitoring report 0 V and not powering.
#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ElectricalReadings {
    pub port_voltage: f32,
    pub stbd_voltage: f32,
    pub balanced_voltage: f32,
    pub twelve_volt_voltage: f32,
    pub five_volt_voltage: f32,
    pub three_volt_voltage: f32,
    pub stbd_powering: bool,
    pub port_powering: bool,
}

#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HostToDevice {
    /// From `command/software_kill`.
    SoftwareKill(SoftwareKill),
    /// From `command/electrical` (`riptide_msgs2/ElectricalCommand::command`). Only the fan commands do anything.
    ElectricalCommand(u8),
}

#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeviceToHost {
    /// For `state/kill`: true while the vehicle is killed. Sent every 150 ms and whenever the state flips.
    KillState(bool),
    /// For `state/physkill_notify`: the physical switch changed, true if it now asserts kill (removed).
    PhysicalKillNotify(bool),
    /// For `state/electrical`, plus `state/batteries_balanced` (both batteries powering) on boards that have it.
    Electrical(ElectricalReadings),
    /// For `state/aux`: level of the aux switch.
    AuxSwitch(bool),
    /// For `state/temp/<board>` and `state/humidity/<board>`: a new SHT41 reading.
    TempHumidity { temperature_c: f32, humidity_rh: f32 },
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{MAX_FD_FRAME_LEN, decode, encode};

    #[test]
    fn worst_case_messages_fit_one_fd_frame() {
        let host = crate::HostToDevice::Power(HostToDevice::SoftwareKill(SoftwareKill {
            kill_switch_id: 255,
            asserting_kill: true,
            needs_update: true,
            sender_id: heapless::Vec::from_slice(&[b'x'; SENDER_ID_LEN]).unwrap(),
        }));
        let mut buf = [0u8; MAX_FD_FRAME_LEN];
        let encoded = encode(&host, &mut buf).expect("fits a CAN FD frame");
        assert_eq!(decode::<crate::HostToDevice>(encoded).unwrap(), host);

        let readings = ElectricalReadings {
            port_voltage: 16.8,
            stbd_voltage: 16.7,
            balanced_voltage: f32::NAN,
            twelve_volt_voltage: 12.0,
            five_volt_voltage: 5.0,
            three_volt_voltage: 3.3,
            stbd_powering: true,
            port_powering: false,
        };
        let device = crate::DeviceToHost::Power(DeviceToHost::Electrical(readings));
        let encoded = encode(&device, &mut buf).expect("fits a CAN FD frame");
        match decode::<crate::DeviceToHost>(encoded).unwrap() {
            crate::DeviceToHost::Power(DeviceToHost::Electrical(got)) => {
                assert!(got.balanced_voltage.is_nan());
                assert_eq!(got.port_voltage, 16.8);
                assert!(got.stbd_powering && !got.port_powering);
            }
            other => panic!("decoded {other:?}"),
        }
    }

    #[test]
    fn round_trip() {
        let device = [
            DeviceToHost::KillState(true),
            DeviceToHost::PhysicalKillNotify(false),
            DeviceToHost::AuxSwitch(true),
            DeviceToHost::TempHumidity { temperature_c: 31.5, humidity_rh: 40.25 },
        ];
        for msg in device {
            let msg = crate::DeviceToHost::Power(msg);
            let mut buf = [0u8; MAX_FD_FRAME_LEN];
            let encoded = encode(&msg, &mut buf).unwrap();
            assert_eq!(decode::<crate::DeviceToHost>(encoded).unwrap(), msg);
        }
        let msg = crate::HostToDevice::Power(HostToDevice::ElectricalCommand(6));
        let mut buf = [0u8; MAX_FD_FRAME_LEN];
        let encoded = encode(&msg, &mut buf).unwrap();
        assert_eq!(decode::<crate::HostToDevice>(encoded).unwrap(), msg);
    }
}
