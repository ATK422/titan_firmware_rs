//! Camera cage breakout board (`talos/CameraCageBB`) messages.

use serde::{Deserialize, Serialize};

/// Messages from the bridge to the board.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HostToDevice {
    /// From `command/led` (`riptide_msgs2/LedCommand`), forwarded whole; the board ignores commands whose
    /// `target` doesn't include the camera cage (`TARGET_CCB`).
    LedCommand(LedCommand),
    /// From `state/physkill_notify` (`std_msgs/Bool`): `true` when the physical kill switch was removed.
    PhysKillNotify(bool),
    /// From `command/electrical` (`riptide_msgs2/ElectricalCommand`): the `command` field.
    ElectricalCommand(u8),
}

/// Field for field copy of `riptide_msgs2/LedCommand`.
#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LedCommand {
    pub red: u8,
    pub green: u8,
    pub blue: u8,
    pub mode: u8,
    pub target: u8,
}

/// `riptide_msgs2/LedCommand` constants.
pub mod led {
    pub const MODE_SOLID: u8 = 0;
    pub const MODE_SLOW_FLASH: u8 = 1;
    pub const MODE_FAST_FLASH: u8 = 2;
    pub const MODE_BREATH: u8 = 3;
    pub const TARGET_CCB: u8 = 1;
}

/// `riptide_msgs2/ElectricalCommand` constants the board acts on.
pub mod electrical {
    pub const CYCLE_COMPUTER: u8 = 0;
    pub const ENABLE_FANS: u8 = 5;
    pub const DISABLE_FANS: u8 = 6;
    pub const CLEAR_DEPTH: u8 = 7;
    pub const DISABLE_LEDS: u8 = 8;
    pub const ENABLE_LEDS: u8 = 9;
}

/// Messages from the board to the bridge.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeviceToHost {
    /// For `state/depth/raw` (`riptide_msgs2/Depth`): the `depth` field in metres, negative below the surface as
    /// the C board published it. The bridge adds the stamp, frame and variance.
    Depth(f32),
    /// For `state/depth/temp` (`std_msgs/Float32`): water temperature in degrees Celsius.
    WaterTemp(f32),
    /// For `state/temp/cameracage` and `state/humidity/cameracage` (`std_msgs/Float32`): SHT41 reading.
    TempHumidity { temperature_c: f32, humidity_rh: f32 },
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{MAX_FD_FRAME_LEN, decode, encode};

    #[test]
    fn messages_round_trip_in_one_frame() {
        let host = [
            HostToDevice::LedCommand(LedCommand { red: 255, green: 128, blue: 0, mode: 3, target: 3 }),
            HostToDevice::PhysKillNotify(true),
            HostToDevice::ElectricalCommand(electrical::CLEAR_DEPTH),
        ];
        for msg in host {
            let msg = crate::HostToDevice::CameraCage(msg);
            let mut buf = [0u8; MAX_FD_FRAME_LEN];
            let encoded = encode(&msg, &mut buf).unwrap();
            assert_eq!(decode::<crate::HostToDevice>(encoded).unwrap(), msg);
        }
        let device = [
            DeviceToHost::Depth(-12.25),
            DeviceToHost::WaterTemp(18.5),
            DeviceToHost::TempHumidity { temperature_c: 31.0, humidity_rh: 45.5 },
        ];
        for msg in device {
            let msg = crate::DeviceToHost::CameraCage(msg);
            let mut buf = [0u8; MAX_FD_FRAME_LEN];
            let encoded = encode(&msg, &mut buf).unwrap();
            assert_eq!(decode::<crate::DeviceToHost>(encoded).unwrap(), msg);
        }
    }
}
