//! SMBus pieces of `pio_smbus.c` and the error reporting of `bq40z80.c` that don't touch hardware.

/// SMBus Packet Error Code: CRC-8 with polynomial x^8 + x^2 + x + 1 (0x07), initial value 0, over every byte on the
/// bus including the address bytes (the C `crc8_table`).
pub fn pec(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0, |crc, &b| pec_update(crc, b))
}

/// Feeds one byte into a running PEC (`crc8_update`).
pub const fn pec_update(crc: u8, byte: u8) -> u8 {
    let mut crc = crc ^ byte;
    let mut i = 0;
    while i < 8 {
        crc = if crc & 0x80 != 0 { (crc << 1) ^ 0x07 } else { crc << 1 };
        i += 1;
    }
    crc
}

/// SMBus transfer failures (`PIO_SMBUS_ERR_*`). The discriminants are the C error codes.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(i8)]
pub enum SmbusError {
    /// The address byte was NAKed: the device is offline.
    AddrNak = -1,
    /// The address byte after the repeated start was NAKed.
    AddrRestartNak = -2,
    /// A data byte was NAKed.
    Nak = -3,
    /// The clock was stretched beyond the SMBus timeout.
    Timeout = -4,
    /// SDA or SCL was low before the START.
    BusStuckLow = -5,
    /// The bus didn't carry the data we sent.
    ArbitrationLost = -6,
    /// The PEC byte did not match.
    BadChecksum = -7,
    /// The device sent a longer block than the buffer holds.
    BufTooSmall = -8,
    /// Attempted to write more than an SMBus block allows.
    BufTooLarge = -9,
    /// The response had the wrong shape (used by the gauge driver).
    InvalidResp = -10,
}

impl SmbusError {
    /// The C name, for the debug commands.
    pub fn name(self) -> &'static str {
        match self {
            SmbusError::AddrNak => "PIO_SMBUS_ERR_ADDR_NAK",
            SmbusError::AddrRestartNak => "PIO_SMBUS_ERR_ADDR_RESTART_NAK",
            SmbusError::Nak => "PIO_SMBUS_ERR_NAK",
            SmbusError::Timeout => "PIO_SMBUS_ERR_TIMEOUT",
            SmbusError::BusStuckLow => "PIO_SMBUS_ERR_BUS_STUCK_LOW",
            SmbusError::ArbitrationLost => "PIO_SMBUS_ERR_ABRITRATION_LOST",
            SmbusError::BadChecksum => "PIO_SMBUS_ERR_BAD_CHECKSUM",
            SmbusError::BufTooSmall => "PIO_SMBUS_ERR_BUF_TOO_SMALL",
            SmbusError::BufTooLarge => "PIO_SMBUS_ERR_BUF_TOO_LARGE",
            SmbusError::InvalidResp => "PIO_SMBUS_ERR_INVALID_RESP",
        }
    }
}

/// Why a gauge operation failed (`BQ_ERROR_*`, plus the SMBus errors, which C passed through as negative codes).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BqErrorKind {
    Smbus(SmbusError),
    /// `DeviceType` isn't a BQ40Z80.
    InvalidDeviceType,
    /// The serial number read as all zeros or all ones.
    InvalidSerial,
    /// The serial number changed since connecting.
    SerialChanged,
    /// The command isn't allowed in the current state.
    InvalidState,
    /// A manufacturer block access response was too short.
    InvalidMacRespLen,
    /// `BatteryStatus()` reported an error code for the last command.
    BattStatusError,
    /// A debug command had a bad parameter.
    BadDbgCommand,
}

impl BqErrorKind {
    /// The C error code (`bq_error_t.fields.error_code`).
    pub fn code(self) -> i8 {
        match self {
            BqErrorKind::Smbus(e) => e as i8,
            BqErrorKind::InvalidDeviceType => 1,
            BqErrorKind::InvalidSerial => 2,
            BqErrorKind::SerialChanged => 3,
            BqErrorKind::InvalidState => 4,
            BqErrorKind::InvalidMacRespLen => 5,
            BqErrorKind::BattStatusError => 6,
            BqErrorKind::BadDbgCommand => 7,
        }
    }
}

/// A gauge error with context (`bq_error_t`).
///
/// The C driver recorded the source line that failed. The Rust driver records the SBS command or manufacturer
/// access command it was working on instead, which says more and doesn't change with edits. [`BqError::raw`] packs
/// it the same way as the C union, so fault arguments and the "Err: 0x..." screens keep their layout:
/// byte 0 error code, bytes 1-2 command (C: line), byte 3 argument.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BqError {
    pub kind: BqErrorKind,
    /// The command being executed when the error happened.
    pub command: u16,
    /// Extra detail, such as the bad value (truncated to 8 bits like C).
    pub arg: u8,
}

impl BqError {
    pub fn new(kind: BqErrorKind, command: u16) -> Self {
        BqError { kind, command, arg: 0 }
    }

    pub fn with_arg(kind: BqErrorKind, command: u16, arg: u32) -> Self {
        BqError { kind, command, arg: arg as u8 }
    }

    pub fn smbus(err: SmbusError, command: u16) -> Self {
        Self::new(BqErrorKind::Smbus(err), command)
    }

    /// The packed form used as fault argument (`bq_error_t.data`).
    pub fn raw(&self) -> u32 {
        (self.kind.code() as u8 as u32) | (self.command as u32) << 8 | (self.arg as u32) << 24
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The first entries of the C lookup table.
    const C_TABLE_START: [u8; 8] = [0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15];

    #[test]
    fn pec_matches_the_c_table() {
        for (i, expected) in C_TABLE_START.iter().enumerate() {
            assert_eq!(pec_update(0, i as u8), *expected);
        }
        assert_eq!(pec_update(0, 0xFF), 0xF3);
        assert_eq!(pec_update(0, 0x80), 0x89);
    }

    #[test]
    fn pec_of_a_word_read() {
        // CRC-8/SMBUS check value
        assert_eq!(pec(b"123456789"), 0xF4);
        // A word read of SerialNumber() = 0x0005 from the gauge: addr/W, cmd, addr/R, lo, hi
        let frame = [0x16, 0x1C, 0x17, 0x05, 0x00];
        let crc = pec(&frame);
        // Appending the PEC gives a zero remainder
        assert_eq!(pec_update(crc, crc), 0);
        assert_eq!(pec(&[0x16, 0x1C, 0x17, 0x05, 0x00, crc]), 0);
    }

    #[test]
    fn error_packing_matches_the_c_union() {
        let err = BqError::smbus(SmbusError::AddrNak, 0x1C);
        assert_eq!(err.raw(), 0x0000_1CFF);
        let err = BqError::with_arg(BqErrorKind::BattStatusError, 0x270C, 4);
        assert_eq!(err.raw(), 0x0427_0C06);
        // Device type 0x4800 truncates to 0 in the 8 bit argument, as in C
        assert_eq!(BqError::with_arg(BqErrorKind::InvalidDeviceType, 1, 0x4800).arg, 0);
    }
}
