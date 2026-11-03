//! IVC board, second revision firmware (`talos/IVCBoard2`) messages.
//!
//! Only topics the C firmware actually publishes have a [`DeviceToHost`] message. Its `ivc/adc_dump`,
//! `ivc/new_transducer`, `ivc/rx_sample_stream` and `ivc/rx_sample_snr_stream` publishers exist but their publish
//! calls are commented out, so the bridge creates those topics and leaves them silent.

use serde::{Deserialize, Serialize};

/// Messages from the bridge to the board.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HostToDevice {
    /// From `ivc/enqueue_data` (`std_msgs/Int8`): queue one byte for acoustic transmission.
    EnqueueData(u8),
    /// From `ivc/pinger/enable` (`std_msgs/Bool`): switch between pinger listening and comms.
    PingerEnable(bool),
    /// From `ivc/pinger/set_freq_khz` (`std_msgs/Int32`): pinger frequency in kHz (20, 25, 30, 35, 37 or 40;
    /// others are ignored by the board).
    PingerSetFreqKhz(i32),
    /// From `ivc/storage/configure` (`std_msgs/Int8`): 0 = recorder off, 1 = write flash, 2 = dump flash.
    StorageConfigure(i8),
    /// From `ivc/set_min_snr` (`std_msgs/Float32`): minimum SNR for a tone to count as present.
    SetMinSnr(f32),
    /// From `ivc/debug/send_freq` (`std_msgs/Int8`, best effort): play one tone for 500 ms, 0 = low, 2 = sync,
    /// anything else = high.
    DebugSendFreq(u8),
}

/// Messages from the board to the bridge.
#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeviceToHost {
    /// For `ivc/rx_new` (`std_msgs/Int8`): a byte received acoustically.
    RxNew(u8),
    /// For `ivc/pinger/selected_freq_amp_stream` (`std_msgs/Float32`): amplitude of the pinger band, once per
    /// capture while pinger mode is on.
    PingerAmplitude(f32),
    /// For `ivc/pinger/selected_freq_khz` (`std_msgs/Int32`): the selected pinger frequency, every second while
    /// pinger mode is on.
    PingerFreqKhz(i32),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{MAX_FD_FRAME_LEN, decode, encode};

    #[test]
    fn round_trip_in_one_frame() {
        let host = [
            HostToDevice::EnqueueData(0xA5),
            HostToDevice::PingerEnable(true),
            HostToDevice::PingerSetFreqKhz(-40),
            HostToDevice::StorageConfigure(-1),
            HostToDevice::SetMinSnr(4.5),
            HostToDevice::DebugSendFreq(2),
        ];
        for msg in host {
            let msg = crate::HostToDevice::Ivc2(msg);
            let mut buf = [0u8; MAX_FD_FRAME_LEN];
            let encoded = encode(&msg, &mut buf).unwrap();
            assert_eq!(decode::<crate::HostToDevice>(encoded).unwrap(), msg);
        }
        let device =
            [DeviceToHost::RxNew(0xFF), DeviceToHost::PingerAmplitude(1234.5), DeviceToHost::PingerFreqKhz(i32::MIN)];
        for msg in device {
            let msg = crate::DeviceToHost::Ivc2(msg);
            let mut buf = [0u8; MAX_FD_FRAME_LEN];
            let encoded = encode(&msg, &mut buf).unwrap();
            assert_eq!(decode::<crate::DeviceToHost>(encoded).unwrap(), msg);
        }
    }
}
