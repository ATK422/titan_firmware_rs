//! Linux interface (`reg_mapped/interface/linux.h`), served by the debug daemon on Linux machines on the bus.
//!
//! ```text
//! 0x00: General control  (registers)
//! 0x01: Version string   RO memory
//! 0x02: TTY control      (registers)
//! 0x03: TTY terminal     RW memory (TERM for the remote TTY)
//! 0x04: TTY command      RW memory (command to run, empty for a login shell)
//! 0x05: File buffer      RW memory
//! 0x06: File control     (registers)
//! ```

pub const GEN_CONTROL_PAGE: u8 = 0x00;
pub const VERSION_STRING_PAGE: u8 = 0x01;
pub const TTY_CONTROL_PAGE: u8 = 0x02;
pub const TTY_TERMINAL_PAGE: u8 = 0x03;
pub const TTY_CMD_PAGE: u8 = 0x04;
pub const FILE_BUFFER_PAGE: u8 = 0x05;
pub const FILE_CONTROL_PAGE: u8 = 0x06;

/// General control page.
pub mod gen_control {
    /// RO: always [`MAGIC_VALUE`].
    pub const MAGIC: u8 = 0x00;
    pub const MAGIC_VALUE: u32 = 0x4E6D2B07;
    /// RO: lower 32 bits of the machine's unique ID.
    pub const LOWER_FLASH_ID: u8 = 0x01;
    /// RO: upper 32 bits of the machine's unique ID.
    pub const UPPER_FLASH_ID: u8 = 0x02;
    /// WO: writing [`REBOOT_DEVICE_MAGIC`] reboots the machine.
    pub const REBOOT_DEVICE: u8 = 0x03;
    pub const REBOOT_DEVICE_MAGIC: u32 = 0xFEE1DEAD;
    /// WO: writing [`RESTART_DAEMON_MAGIC`] restarts the debug daemon.
    pub const RESTART_DAEMON: u8 = 0x04;
    pub const RESTART_DAEMON_MAGIC: u32 = 0xDEADFA11;
}

/// TTY control page.
pub mod tty_control {
    /// RW: 1 starts the remote TTY session, 0 kills it.
    pub const ENABLE: u8 = 0x00;
    /// RW: initial window size, columns in the lower 16 bits and rows in the upper 16 bits.
    pub const WINDOW_SIZE: u8 = 0x01;
    /// RW: non-zero runs the TTY in the file transfer working directory instead of the home directory.
    pub const USE_UPLOAD_DIR: u8 = 0x02;
}

/// File control page.
pub mod file_control {
    /// WO: length of the filename at the start of the file buffer.
    pub const FILENAME_LENGTH: u8 = 0x00;
    /// RW: number of data bytes following the filename (rounded up to a word) in the file buffer.
    pub const DATA_LENGTH: u8 = 0x01;
    /// WO: CRC-32 of the buffer data (write) or of the whole file (check CRC).
    pub const CRC: u8 = 0x02;
    /// WO: non-zero truncates the file before a write instead of appending.
    pub const CLEAR: u8 = 0x03;
    /// WO: file offset for reads.
    pub const READ_OFFSET: u8 = 0x04;
    /// RW: file mode for set/get mode.
    pub const FILE_MODE: u8 = 0x05;
    /// WO: starts an operation (see the `OPERATION_*` values).
    pub const OPERATION: u8 = 0x06;
    /// RO: status of the last operation (see the `STATUS_*` values).
    pub const STATUS: u8 = 0x07;

    pub const OPERATION_NOP: u32 = 0;
    pub const OPERATION_WRITE: u32 = 1;
    pub const OPERATION_READ: u32 = 2;
    pub const OPERATION_SET_MODE: u32 = 3;
    pub const OPERATION_GET_MODE: u32 = 4;
    pub const OPERATION_GET_FILE_LEN: u32 = 5;
    pub const OPERATION_CHECK_CRC: u32 = 6;
    pub const OPERATION_CD: u32 = 7;
    pub const OPERATION_LS: u32 = 8;
    pub const OPERATION_PWD: u32 = 9;

    pub const STATUS_READY: u32 = 0;
    pub const STATUS_BUSY: u32 = 1;
    pub const STATUS_SUCCESS: u32 = 2;
    /// The buffer holds an error message, its length is in [`DATA_LENGTH`].
    pub const STATUS_FAIL_DEVICE_ERROR: u32 = 3;
    pub const STATUS_FAIL_BAD_CRC: u32 = 4;
}
