//! IVC (inter-vehicle communication) board messages, `talos/IVCBoard`.

use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HostToDevice {
    /// From `ivc/tx`: queue one byte for acoustic transmission.
    Tx(u8),
    /// From `ivc/debug_tx_frequency`: play a tone at this frequency in Hz for one second.
    DebugTxFrequency(f32),
}

#[derive(Serialize, Deserialize, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeviceToHost {
    /// For `ivc/rx`: a byte received acoustically with a valid CRC.
    Rx(u8),
    /// For `ivc/debug_rx_mag`: low band magnitude, sent only by calibration builds.
    RxDebugMagnitude(f32),
}
