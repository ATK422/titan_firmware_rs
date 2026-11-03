//! File transfer with Linux machines over the Linux control interface, ported from
//! `canmore_cli/src/file_transfer_task.cpp`.
//!
//! Files move through the 1 KiB file buffer page. Each request carries the remote filename at the start of the
//! buffer (zero padded to a whole register), followed by data on uploads. Operations are started by writing the
//! operation register and polled through the status register. `cd`, `ls` and `pwd` exchange their argument and
//! result through a temporary file on the remote machine.

use std::time::{Duration, Instant};

use anyhow::{Result, bail};
use canmore::reg_mapped::interface::linux::{self as itf, file_control as fc};

use super::LinuxClient;
use crate::crc32;

/// Size of the file buffer used per request.
pub const FILE_BUFFER_SIZE: usize = 1024;
/// Transmission errors tolerated per chunk before giving up.
pub const MAX_ERRORS: usize = 5;
/// Temporary file on the remote machine used by `cd`, `ls` and `pwd`.
pub const SERVER_TMP_FILE: &str = "/tmp/canmore_ft_server_buffer";
/// Mode given to files uploaded from memory.
const DEFAULT_FILE_MODE: u32 = 0o100644;
const OPERATION_TIMEOUT: Duration = Duration::from_secs(1);

/// Rounds `name` up to whole registers, as it is laid out at the start of the file buffer.
fn padded_name(name: &str) -> Vec<u8> {
    let mut buf = name.as_bytes().to_vec();
    buf.resize(buf.len().next_multiple_of(4), 0);
    buf
}

impl LinuxClient {
    fn fc_write(&mut self, offset: u8, value: u32) -> Result<()> {
        self.client().write(itf::FILE_CONTROL_PAGE, offset, value)
    }

    fn fc_read(&mut self, offset: u8) -> Result<u32> {
        self.client().read(itf::FILE_CONTROL_PAGE, offset)
    }

    fn write_buffer(&mut self, data: &[u8]) -> Result<()> {
        self.client().write_bytes(itf::FILE_BUFFER_PAGE, 0, data)
    }

    fn read_buffer(&mut self, len: usize) -> Result<Vec<u8>> {
        let mut buf = vec![0u8; len];
        self.client().read_bytes(itf::FILE_BUFFER_PAGE, 0, &mut buf)?;
        Ok(buf)
    }

    /// Runs a file operation and returns the final status (one of `file_control::STATUS_*`).
    fn file_operation(&mut self, operation: u32) -> Result<u32> {
        // The controller only accepts an operation while it reports ready, which a NOP gets it back to
        self.fc_write(fc::OPERATION, fc::OPERATION_NOP)?;
        let start = Instant::now();
        let mut status = self.fc_read(fc::STATUS)?;
        while status != fc::STATUS_READY {
            if start.elapsed() > OPERATION_TIMEOUT {
                bail!("Timed out waiting for file status to become READY");
            }
            status = self.fc_read(fc::STATUS)?;
        }

        self.fc_write(fc::OPERATION, operation)?;
        let start = Instant::now();
        while (status == fc::STATUS_READY || status == fc::STATUS_BUSY) && start.elapsed() < OPERATION_TIMEOUT {
            status = self.fc_read(fc::STATUS)?;
        }
        Ok(status)
    }

    /// The error message a failed operation left in the file buffer.
    fn remote_error(&mut self) -> Result<String> {
        let len = (self.fc_read(fc::DATA_LENGTH)? as usize).min(FILE_BUFFER_SIZE - 1);
        let msg = self.read_buffer(len)?;
        Ok(String::from_utf8_lossy(&msg).trim_end_matches('\0').to_owned())
    }

    /// Runs an operation, turning a non-success status into an error mentioning `what`.
    fn checked_operation(&mut self, operation: u32, what: &str) -> Result<()> {
        match self.file_operation(operation)? {
            fc::STATUS_SUCCESS => Ok(()),
            fc::STATUS_FAIL_DEVICE_ERROR => bail!("{what}; Remote device reported error: {}", self.remote_error()?),
            fc::STATUS_FAIL_BAD_CRC => bail!("{what}: bad CRC"),
            fc::STATUS_BUSY | fc::STATUS_READY => bail!("{what}: timed out waiting for the remote device"),
            other => bail!("{what}: unknown status {other}"),
        }
    }

    /// Puts `name` at the start of the file buffer and sets the filename length.
    fn set_filename(&mut self, name: &str, length: usize) -> Result<()> {
        self.write_buffer(&padded_name(name))?;
        self.fc_write(fc::FILENAME_LENGTH, length as u32)
    }

    /// Writes `data` to `remote_path` (replacing it) with permissions `mode`. Calls `progress` with the fraction
    /// sent after every chunk.
    pub fn upload(&mut self, data: &[u8], remote_path: &str, mode: u32, progress: &mut dyn FnMut(f64)) -> Result<()> {
        let name = padded_name(remote_path);
        if name.len() + 4 > FILE_BUFFER_SIZE {
            bail!("Remote path too long: {remote_path}");
        }
        let chunk_capacity = FILE_BUFFER_SIZE - name.len();
        let mut file_crc = crc32::INITIAL;
        let mut sent = 0;
        let mut first = true;
        loop {
            let chunk = &data[sent..(sent + chunk_capacity).min(data.len())];
            let mut buf = name.clone();
            buf.extend_from_slice(chunk);

            let mut errors = 0;
            loop {
                self.write_buffer(&buf)?;
                self.fc_write(fc::FILENAME_LENGTH, remote_path.len() as u32)?;
                self.fc_write(fc::DATA_LENGTH, buf.len() as u32)?;
                self.fc_write(fc::CRC, crc32::compute(&buf))?;
                // The first chunk truncates the file, the rest append
                self.fc_write(fc::CLEAR, first as u32)?;
                match self.file_operation(fc::OPERATION_WRITE)? {
                    fc::STATUS_SUCCESS => break,
                    fc::STATUS_FAIL_DEVICE_ERROR => {
                        bail!("File write failed; Remote device reported error: {}", self.remote_error()?)
                    }
                    fc::STATUS_FAIL_BAD_CRC => {
                        errors += 1;
                        if errors > MAX_ERRORS {
                            bail!(
                                "Maximum number of transmission errors reached while trying to upload the file. Please \
                                 check the connections on the CAN and ensure that traffic is at a reasonable level."
                            );
                        }
                    }
                    _ => bail!("Timed out waiting for write status to become SUCCESS"),
                }
            }

            file_crc = crc32::update(file_crc, chunk);
            sent += chunk.len();
            first = false;
            progress(if data.is_empty() { 1.0 } else { sent as f64 / data.len() as f64 });
            // A full chunk may be followed by more data; an empty final chunk is harmless
            if chunk.len() < chunk_capacity {
                break;
            }
        }

        self.check_crc(remote_path, file_crc)?;
        self.fc_write(fc::FILE_MODE, mode)?;
        self.checked_operation(fc::OPERATION_SET_MODE, "Failed to set remote file permissions")
    }

    fn check_crc(&mut self, remote_path: &str, crc: u32) -> Result<()> {
        // The filename length is still set from the transfer
        self.write_buffer(&padded_name(remote_path))?;
        self.fc_write(fc::CRC, crc)?;
        match self.file_operation(fc::OPERATION_CHECK_CRC)? {
            fc::STATUS_SUCCESS => Ok(()),
            fc::STATUS_FAIL_DEVICE_ERROR => {
                bail!("Failed to check remote file CRC; Remote device reported error: {}", self.remote_error()?)
            }
            fc::STATUS_FAIL_BAD_CRC => bail!("Checksum of remote file does not match the local one. Please try again"),
            _ => bail!("Failed to check file CRC: Unknown error"),
        }
    }

    /// Reads `remote_path`, returning its contents and permissions.
    pub fn download(&mut self, remote_path: &str, progress: &mut dyn FnMut(f64)) -> Result<(Vec<u8>, u32)> {
        if padded_name(remote_path).len() + 4 > FILE_BUFFER_SIZE {
            bail!("Remote path too long: {remote_path}");
        }
        self.set_filename(remote_path, remote_path.len())?;
        self.checked_operation(fc::OPERATION_GET_FILE_LEN, "Failed to get file length")?;
        let file_size = self.fc_read(fc::DATA_LENGTH)? as usize;

        let mut data = Vec::with_capacity(file_size);
        let mut file_crc = crc32::INITIAL;
        let mut first = true;
        let mut errors = 0;
        loop {
            self.write_buffer(&padded_name(remote_path))?;
            self.fc_write(fc::READ_OFFSET, data.len() as u32)?;
            self.fc_write(fc::CLEAR, first as u32)?;
            self.checked_operation(fc::OPERATION_READ, "Failed to read file data")?;

            let batch_len = (self.fc_read(fc::DATA_LENGTH)? as usize).min(FILE_BUFFER_SIZE);
            let expected_crc = self.fc_read(fc::CRC)?;
            let batch = self.read_buffer(batch_len)?;
            if crc32::compute(&batch) != expected_crc {
                errors += 1;
                if errors > MAX_ERRORS {
                    bail!(
                        "Maximum number of transmission errors reached while trying to download the file. Please \
                         check the connections on the CAN and ensure that traffic is at a reasonable level."
                    );
                }
                continue;
            }
            errors = 0;
            first = false;
            file_crc = crc32::update(file_crc, &batch);
            data.extend_from_slice(&batch);
            progress(if file_size == 0 { 1.0 } else { data.len() as f64 / file_size as f64 });
            if batch_len < FILE_BUFFER_SIZE {
                break;
            }
        }

        self.check_crc(remote_path, file_crc)?;
        self.checked_operation(fc::OPERATION_GET_MODE, "Failed to get remote file permissions")?;
        let mode = self.fc_read(fc::FILE_MODE)?;
        Ok((data, mode))
    }

    /// Runs an operation on the remote temporary file (`cd`, `ls`, `pwd`).
    fn tmp_file_operation(&mut self, operation: u32, what: &str) -> Result<()> {
        // These operations take the filename length including the terminator, like the C++ tool sends it
        self.set_filename(SERVER_TMP_FILE, SERVER_TMP_FILE.len() + 1)?;
        self.checked_operation(operation, what)
    }

    /// Changes the file transfer working directory on the remote machine.
    pub fn cd(&mut self, directory: &str) -> Result<()> {
        self.upload(directory.as_bytes(), SERVER_TMP_FILE, DEFAULT_FILE_MODE, &mut |_| {})?;
        self.tmp_file_operation(fc::OPERATION_CD, "Failed to set working directory on remote")
    }

    /// Lists a directory on the remote machine (the working directory if empty).
    pub fn ls(&mut self, directory: &str) -> Result<String> {
        self.upload(directory.as_bytes(), SERVER_TMP_FILE, DEFAULT_FILE_MODE, &mut |_| {})?;
        self.tmp_file_operation(fc::OPERATION_LS, "Failed to list the directory on the remote")?;
        let (listing, _) = self.download(SERVER_TMP_FILE, &mut |_| {})?;
        Ok(String::from_utf8_lossy(&listing).into_owned())
    }

    /// The file transfer working directory on the remote machine.
    pub fn pwd(&mut self) -> Result<String> {
        self.tmp_file_operation(fc::OPERATION_PWD, "Failed to read the working directory on the remote")?;
        let (path, _) = self.download(SERVER_TMP_FILE, &mut |_| {})?;
        Ok(String::from_utf8_lossy(&path).into_owned())
    }
}
