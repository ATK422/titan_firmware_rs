//! LED board (`talos/LEDBoard`) messages.
//!
//! The bridge forwards the board's three subscriptions field for field; the firmware interprets them as the C
//! executor callbacks did (target filtering, mode mapping, kill switch flash colours).

use serde::{Deserialize, Serialize};

/// Field for field copy of `riptide_msgs2/msg/LedCommand`.
#[derive(Serialize, Deserialize, Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LedCommand {
    pub red: u8,
    pub green: u8,
    pub blue: u8,
    /// `MODE_*` / `SINGLETON_FLASH`.
    pub mode: u8,
    /// `TARGET_*` bit mask; the board only acts on commands with `TARGET_ALU` set.
    pub target: u8,
}

/// Messages from the bridge to the board.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HostToDevice {
    /// From `command/led` (`riptide_msgs2/LedCommand`).
    Command(LedCommand),
    /// From `state/physkill_notify` (`std_msgs/Bool`): true when the physical kill switch is removed.
    PhysKillNotify(bool),
    /// `depth` from `state/depth/raw` (`riptide_msgs2/Depth`), in metres (negative under water).
    Depth(f32),
}

/// Messages from the board to the bridge. The board publishes only the common topics.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeviceToHost {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{MAX_FD_FRAME_LEN, decode, encode};

    #[test]
    fn round_trip_fits_one_frame() {
        let messages = [
            crate::HostToDevice::Led(HostToDevice::Command(LedCommand {
                red: 255,
                green: 128,
                blue: 1,
                mode: 4,
                target: 3,
            })),
            crate::HostToDevice::Led(HostToDevice::PhysKillNotify(true)),
            crate::HostToDevice::Led(HostToDevice::Depth(-1.25)),
        ];
        for msg in messages {
            let mut buf = [0u8; MAX_FD_FRAME_LEN];
            let encoded = encode(&msg, &mut buf).expect("fits one CAN FD frame");
            assert_eq!(decode::<crate::HostToDevice>(encoded).unwrap(), msg);
        }
    }
}
