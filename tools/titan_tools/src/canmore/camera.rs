//! Camera feed receiver (`canmore/camera_feed_interface.h`), ported from `canmore_dbg_cpp/ImageReceiver.cpp`.
//!
//! A device streams JPEGs over CANmore utility channel 12: extended frames carry the JPEG data with a frame index in
//! the extra field (restarting at 0 for each image), and a standard frame ends each image with its length and CRC-32.
//! Images with a missing frame or a bad CRC are dropped.

use anyhow::Result;
use canmore::id::{self, Direction, Fields, FrameType};

use super::can::{CAN_EFF_FLAG, CAN_RTR_FLAG, CanBus, RawFrame};
use crate::crc32;

pub const CMD_ENABLE: u8 = 0;
pub const CMD_STREAM_ID: u8 = 1;
pub const CMD_QUALITY: u8 = 2;
pub const CMD_MAX_DIMENSION: u8 = 3;
pub const CMD_KEYPRESS: u8 = 4;
pub const CMD_MAX_LEN: usize = 3;

/// Reassembles JPEGs from camera feed frames.
#[derive(Default)]
pub struct JpegAssembler {
    buf: Vec<u8>,
    next_index: u32,
    stopped: bool,
}

impl JpegAssembler {
    /// Feeds one frame of the feed. Returns a complete, CRC checked JPEG when its last frame arrives.
    pub fn handle_frame(&mut self, frame: &RawFrame) -> Option<Vec<u8>> {
        if frame.extended {
            let index = Fields::decode(frame.id, true).extra;
            if index == 0 {
                self.next_index = 1;
                self.buf.clear();
                self.stopped = false;
            } else if self.stopped {
                return None;
            } else if index != self.next_index {
                // A frame was lost, drop the image
                self.stopped = true;
                return None;
            } else {
                self.next_index += 1;
            }
            self.buf.extend_from_slice(&frame.data);
            return None;
        }

        if self.stopped {
            return None;
        }
        self.stopped = true;
        let data: [u8; 8] = frame.data.as_slice().try_into().ok()?;
        let len = u32::from_le_bytes(data[..4].try_into().unwrap()) as usize;
        let crc = u32::from_le_bytes(data[4..].try_into().unwrap());
        if self.buf.len() < len {
            return None;
        }
        // CAN FD pads the last frame, trim it back
        self.buf.truncate(len);
        (crc32::compute(&self.buf) == crc).then(|| std::mem::take(&mut self.buf))
    }
}

/// Receives the camera feed of one device on a CAN bus.
pub struct ImageReceiver {
    bus: CanBus,
    client_id: u8,
    assembler: JpegAssembler,
}

impl ImageReceiver {
    pub fn open(interface: &str, client_id: u8) -> Result<Self> {
        let bus = CanBus::open(interface)?;
        let last_id = id::util_id(client_id, Direction::ClientToAgent, canmore::CHAN_CAMERA_FEED);
        let data_id = id::ext_id(client_id, FrameType::Util, Direction::ClientToAgent, canmore::CHAN_CAMERA_FEED, 0);
        bus.set_filters(&[
            (last_id, CAN_EFF_FLAG | CAN_RTR_FLAG | 0x7FF),
            (data_id | CAN_EFF_FLAG, CAN_EFF_FLAG | CAN_RTR_FLAG | (super::can::CAN_EFF_MASK & !0x3FFFF)),
        ])?;
        Ok(ImageReceiver { bus, client_id, assembler: JpegAssembler::default() })
    }

    pub fn bus(&self) -> &CanBus {
        &self.bus
    }

    fn command(&self, data: &[u8]) -> Result<()> {
        let mut cmd = [0u8; CMD_MAX_LEN];
        cmd[..data.len()].copy_from_slice(data);
        self.bus.write(
            id::util_id(self.client_id, Direction::AgentToClient, canmore::CHAN_CAMERA_FEED),
            false,
            &cmd,
        )?;
        Ok(())
    }

    pub fn set_stream_enabled(&self, enable: bool) -> Result<()> {
        self.command(&[CMD_ENABLE, enable as u8])
    }

    pub fn set_stream_id(&self, stream: u8) -> Result<()> {
        self.command(&[CMD_STREAM_ID, stream])
    }

    pub fn set_quality(&self, quality: u8) -> Result<()> {
        self.command(&[CMD_QUALITY, quality])
    }

    pub fn set_max_dimension(&self, max_dim: u16) -> Result<()> {
        let b = max_dim.to_le_bytes();
        self.command(&[CMD_MAX_DIMENSION, b[0], b[1]])
    }

    pub fn send_keypress(&self, key: u8) -> Result<()> {
        self.command(&[CMD_KEYPRESS, key])
    }

    /// Waits up to `timeout` for a frame. Returns a JPEG if it completed one.
    pub fn poll(&mut self, timeout: std::time::Duration) -> Result<Option<Vec<u8>>> {
        Ok(self.bus.read(timeout)?.and_then(|frame| self.assembler.handle_frame(&frame)))
    }
}

impl Drop for ImageReceiver {
    fn drop(&mut self) {
        let _ = self.set_stream_enabled(false);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frames(client: u8, jpeg: &[u8], chunk: usize) -> Vec<RawFrame> {
        let mut out: Vec<RawFrame> = jpeg
            .chunks(chunk)
            .enumerate()
            .map(|(i, c)| {
                let mut data = c.to_vec();
                data.resize(canmore::fd_frame_len(data.len()), 0);
                let id = id::ext_id(client, FrameType::Util, Direction::ClientToAgent, 12, i as u32);
                RawFrame { id, extended: true, data }
            })
            .collect();
        let mut last = (jpeg.len() as u32).to_le_bytes().to_vec();
        last.extend_from_slice(&crc32::compute(jpeg).to_le_bytes());
        out.push(RawFrame { id: id::util_id(client, Direction::ClientToAgent, 12), extended: false, data: last });
        out
    }

    #[test]
    fn reassembles_and_drops() {
        let jpeg: Vec<u8> = (0..1000u32).map(|i| (i * 13) as u8).collect();
        let mut asm = JpegAssembler::default();
        let mut result = None;
        for f in frames(4, &jpeg, 64) {
            result = asm.handle_frame(&f).or(result);
        }
        assert_eq!(result.as_deref(), Some(jpeg.as_slice()));

        // A missing frame drops the image
        let mut lossy = frames(4, &jpeg, 64);
        lossy.remove(3);
        assert!(lossy.iter().all(|f| asm.handle_frame(f).is_none()));

        // A corrupted frame fails the CRC
        let mut corrupt = frames(4, &jpeg, 64);
        corrupt[2].data[0] ^= 1;
        assert!(corrupt.iter().all(|f| asm.handle_frame(f).is_none()));
    }
}
