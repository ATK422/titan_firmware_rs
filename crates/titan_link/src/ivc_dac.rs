//! IVC board with DAC transmitter (`talos/IVCDAC`) messages.
//!
//! IVCDAC is the IVCBoard2 firmware stripped down to a data gathering tool: it plays test tones on command and
//! timestamps them, and (built with `RX_SIDE`) streams the comms band magnitudes. Its other IVCBoard2 topics are
//! still created but have no effect, so the bridge does not forward them.

use serde::{Deserialize, Serialize};

/// Messages from the bridge to the board.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HostToDevice {
    /// From `ivc_dac/send_freq` (`std_msgs/Int8`, best effort): play one tone for 500 ms, 0 = low, 2 = sync,
    /// anything else = high.
    SendFreq(u8),
    /// From `ivc/pinger/enable` (`std_msgs/Bool`): the receive side then filters with the pinger band.
    PingerEnable(bool),
    /// From `ivc/pinger/set_freq_khz` (`std_msgs/Int32`): pinger filter selection in kHz.
    PingerSetFreqKhz(i32),
}

/// Messages from the board to the bridge.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeviceToHost {
    /// For `ivc_dac/tx_start` (`builtin_interfaces/Time`): a tone just started. Sent at once, so the bridge stamps
    /// it on arrival.
    TxStart,
    /// For `ivc_dac/tx_end` (`builtin_interfaces/Time`): the tone just stopped.
    TxEnd,
    /// For `ivc_dac/rx_state` (`riptide_msgs2/IvcRx`): comms band magnitudes of one capture (receive side builds).
    RxState {
        low: f32,
        high: f32,
        sync: f32,
        /// Time from the end of the capture to sending this, so the bridge can stamp the capture itself.
        age_us: u32,
    },
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{MAX_FD_FRAME_LEN, decode, encode};

    #[test]
    fn round_trip_in_one_frame() {
        let host = [HostToDevice::SendFreq(2), HostToDevice::PingerEnable(false), HostToDevice::PingerSetFreqKhz(37)];
        for msg in host {
            let msg = crate::HostToDevice::IvcDac(msg);
            let mut buf = [0u8; MAX_FD_FRAME_LEN];
            let encoded = encode(&msg, &mut buf).unwrap();
            assert_eq!(decode::<crate::HostToDevice>(encoded).unwrap(), msg);
        }
        let device = [
            DeviceToHost::TxStart,
            DeviceToHost::TxEnd,
            DeviceToHost::RxState { low: 1.0, high: f32::MAX, sync: -0.0, age_us: u32::MAX },
        ];
        for msg in device {
            let msg = crate::DeviceToHost::IvcDac(msg);
            let mut buf = [0u8; MAX_FD_FRAME_LEN];
            let encoded = encode(&msg, &mut buf).unwrap();
            assert_eq!(decode::<crate::DeviceToHost>(encoded).unwrap(), msg);
        }
    }
}
