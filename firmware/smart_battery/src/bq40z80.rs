//! BQ40Z80 gas gauge driver, ported from `bq40z80.c`. All SMBus traffic to the gauge goes through here.
//!
//! The gauge sometimes NAKs transfers that follow each other within 150 us, so every transfer waits out the
//! throttle time since the previous one, as in C.

use embassy_time::{Duration, Instant, Timer};

use smart_battery_core::remote_cmd::{BLOCK_LEN, Width};
use smart_battery_core::sbs::{self, BatteryInfo, DaStatus1, MfgInfo, cmd, mfg, operation_status as op};
use smart_battery_core::smbus::{BqError, BqErrorKind, SmbusError};

use crate::pio_smbus::PioSmbus;

/// Minimum time between transfers (`BQ_THROTTLE_TIME_US`).
const THROTTLE: Duration = Duration::from_micros(150);
/// Time the gauge needs to commit a data flash write before its status is checked.
const DF_WRITE_SETTLE: Duration = Duration::from_millis(150);

type Result<T> = core::result::Result<T, BqError>;

pub struct Bq40z80 {
    smbus: PioSmbus<'static>,
    next_transfer: Instant,
}

/// Attaches the command being executed to an SMBus error (the C `I2CCHECK` with its line number).
trait Context<T> {
    fn at(self, command: u16) -> Result<T>;
}

impl<T> Context<T> for core::result::Result<T, SmbusError> {
    fn at(self, command: u16) -> Result<T> {
        self.map_err(|e| BqError::smbus(e, command))
    }
}

impl Bq40z80 {
    pub fn new(smbus: PioSmbus<'static>) -> Self {
        Bq40z80 { smbus, next_transfer: Instant::MIN }
    }

    async fn throttle(&mut self) {
        Timer::at(self.next_transfer).await;
    }

    fn transferred(&mut self) {
        self.next_transfer = Instant::now() + THROTTLE;
    }

    // ========================================
    // SMBus primitives (`bq_read_word` etc.)
    // ========================================

    async fn read_word(&mut self, command: u8) -> core::result::Result<u16, SmbusError> {
        self.throttle().await;
        let result = self.smbus.word_read_pec(sbs::ADDRESS, command).await;
        self.transferred();
        result
    }

    async fn read_block(&mut self, command: u8, buf: &mut [u8]) -> core::result::Result<usize, SmbusError> {
        self.throttle().await;
        let result = self.smbus.block_read_pec(sbs::ADDRESS, command, buf).await;
        self.transferred();
        result
    }

    /// Reads a manufacturer access block into `buf`, returning its length without the echoed command
    /// (`bq_mfg_access_read`).
    async fn mfg_access_read(&mut self, mac_cmd: u16, buf: &mut [u8]) -> core::result::Result<usize, SmbusError> {
        // C allowed up to 253 bytes on a stack VLA; SMBus blocks hold 32 and no caller reads more
        if buf.len() > BLOCK_LEN {
            return Err(SmbusError::BufTooLarge);
        }
        self.throttle().await;
        let written =
            self.smbus.block_write_pec(sbs::ADDRESS, cmd::MANUFACTURER_BLOCK_ACCESS, &mac_cmd.to_le_bytes()).await;
        if let Err(e) = written {
            self.transferred();
            return Err(e);
        }
        // No throttle needed between the write and the read
        let mut resp = [0u8; BLOCK_LEN + 2];
        let resp = &mut resp[..buf.len() + 2];
        let read = self.smbus.block_read_pec(sbs::ADDRESS, cmd::MANUFACTURER_BLOCK_ACCESS, resp).await;
        self.transferred();
        let data = sbs::mac_response(mac_cmd, &resp[..read?]).ok_or(SmbusError::InvalidResp)?;
        buf[..data.len()].copy_from_slice(data);
        Ok(data.len())
    }

    /// Sends a manufacturer access command without data (`bq_mfg_access_cmd`).
    async fn mfg_access_cmd(&mut self, mac_cmd: u16) -> core::result::Result<(), SmbusError> {
        self.throttle().await;
        let result =
            self.smbus.block_write_pec(sbs::ADDRESS, cmd::MANUFACTURER_BLOCK_ACCESS, &mac_cmd.to_le_bytes()).await;
        self.transferred();
        result
    }

    /// Writes a manufacturer access block (`bq_mfg_access_write`).
    async fn mfg_access_write(&mut self, mac_cmd: u16, data: &[u8]) -> core::result::Result<(), SmbusError> {
        self.throttle().await;
        if data.is_empty() || data.len() > BLOCK_LEN {
            return Err(SmbusError::BufTooLarge);
        }
        let mut buf = [0u8; BLOCK_LEN + 2];
        buf[..2].copy_from_slice(&mac_cmd.to_le_bytes());
        buf[2..2 + data.len()].copy_from_slice(data);
        let result =
            self.smbus.block_write_pec(sbs::ADDRESS, cmd::MANUFACTURER_BLOCK_ACCESS, &buf[..2 + data.len()]).await;
        self.transferred();
        result
    }

    /// A 4 byte block read as a little endian dword (`bq_read_dword`).
    async fn read_dword(&mut self, command: u8) -> core::result::Result<u32, SmbusError> {
        let mut buf = [0u8; 4];
        match self.read_block(command, &mut buf).await? {
            4 => Ok(u32::from_le_bytes(buf)),
            _ => Err(SmbusError::InvalidResp),
        }
    }

    /// A word read that must fit in 8 bits (`bq_read_byte`).
    async fn read_byte(&mut self, command: u8) -> core::result::Result<u8, SmbusError> {
        u8::try_from(self.read_word(command).await?).map_err(|_| SmbusError::InvalidResp)
    }

    async fn read_sword(&mut self, command: u8) -> core::result::Result<i16, SmbusError> {
        Ok(self.read_word(command).await? as i16)
    }

    /// A manufacturer block that must be exactly `buf.len()` long (`bq_read_mfg_block_fixedlen`).
    async fn read_mfg_block_fixed(&mut self, mac_cmd: u16, buf: &mut [u8]) -> core::result::Result<(), SmbusError> {
        let len = buf.len();
        if self.mfg_access_read(mac_cmd, buf).await? != len { Err(SmbusError::InvalidResp) } else { Ok(()) }
    }

    /// Checks the error code of the last command in `BatteryStatus()` (`bq_check_status_successful`).
    async fn check_status(&mut self) -> Result<()> {
        let status = self.read_word(cmd::BATTERY_STATUS).await.at(cmd::BATTERY_STATUS as u16)?;
        match status & sbs::battery_status::EC_MASK {
            0 => Ok(()),
            code => Err(BqError::with_arg(BqErrorKind::BattStatusError, cmd::BATTERY_STATUS as u16, code as u32)),
        }
    }

    /// Reads the first 4 bytes at a data flash address (`bq_read_data_flash_int`).
    async fn read_data_flash_u32(&mut self, addr: u16) -> Result<u32> {
        let mut buf = [0u8; 32];
        let len = self.mfg_access_read(addr, &mut buf).await.at(addr)?;
        if len < 4 {
            return Err(BqError::with_arg(BqErrorKind::InvalidMacRespLen, addr, len as u32));
        }
        Ok(u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]))
    }

    // ========================================
    // Refresh routines
    // ========================================

    /// Reads the pack identity after connecting (`bq_read_mfg_info`).
    pub async fn read_mfg_info(&mut self) -> Result<MfgInfo> {
        let mut info = MfgInfo::default();

        let mut buf = [0u8; 2];
        self.read_mfg_block_fixed(mfg::DEVICE_TYPE, &mut buf).await.at(mfg::DEVICE_TYPE)?;
        info.device_type = u16::from_le_bytes(buf);
        if info.device_type != sbs::DEVICE_TYPE_BQ40Z80 {
            return Err(BqError::with_arg(BqErrorKind::InvalidDeviceType, mfg::DEVICE_TYPE, info.device_type as u32));
        }

        let serial = self.read_word(cmd::SERIAL_NUMBER).await.at(cmd::SERIAL_NUMBER as u16)?;
        if serial == 0 || serial == 0xFFFF {
            // Probably bus trouble or an unprogrammed gauge: refuse to continue
            return Err(BqError::with_arg(BqErrorKind::InvalidSerial, cmd::SERIAL_NUMBER as u16, serial as u32));
        }
        info.serial = serial;

        let date = self.read_word(cmd::MANUFACTURER_DATE).await.at(cmd::MANUFACTURER_DATE as u16)?;
        (info.mfg_day, info.mfg_month, info.mfg_year) = sbs::decode_date(date);

        self.read_mfg_block_fixed(mfg::FIRMWARE_VERSION, &mut info.firmware_version).await.at(mfg::FIRMWARE_VERSION)?;

        info.scale_factor = self.read_data_flash_u32(sbs::DF_ADDR_SCALE_FACTOR).await? as u8;

        // A block read, but only 2 bytes long
        self.read_mfg_block_fixed(mfg::MANUFACTURING_STATUS, &mut buf).await.at(mfg::MANUFACTURING_STATUS)?;
        info.manufacturing_status = u16::from_le_bytes(buf);
        Ok(info)
    }

    /// Refreshes everything polled periodically (`bq_read_battery_info`). `info` keeps the old values of the
    /// registers that aren't read while the gauge initializes.
    pub async fn read_battery_info(&mut self, mfg_info: &MfgInfo, info: &mut BatteryInfo) -> Result<()> {
        // Final sanity check that we're still talking to the same gauge
        let serial = self.read_word(cmd::SERIAL_NUMBER).await.at(cmd::SERIAL_NUMBER as u16)?;
        if serial != mfg_info.serial {
            return Err(BqError::with_arg(BqErrorKind::SerialChanged, cmd::SERIAL_NUMBER as u16, serial as u32));
        }

        info.operation_status = self.read_dword(cmd::OPERATION_STATUS).await.at(cmd::OPERATION_STATUS as u16)?;
        if sbs::bit(info.operation_status, op::INIT) {
            // Nothing else can be read while initializing
            return Ok(());
        }

        let scale = mfg_info.scale_factor as i32;
        let mut da = [0u8; DaStatus1::LEN];
        self.read_mfg_block_fixed(mfg::DA_STATUS1, &mut da).await.at(mfg::DA_STATUS1)?;
        info.da_status1 = DaStatus1::parse(&da);
        info.current = self.read_sword(cmd::CURRENT).await.at(cmd::CURRENT as u16)? as i32 * scale;
        info.avg_current = self.read_sword(cmd::AVERAGE_CURRENT).await.at(cmd::AVERAGE_CURRENT as u16)? as i32 * scale;
        info.temperature = self.read_word(cmd::TEMPERATURE).await.at(cmd::TEMPERATURE as u16)?;
        info.max_error = self.read_byte(cmd::MAX_ERROR).await.at(cmd::MAX_ERROR as u16)?;
        info.relative_soc =
            self.read_byte(cmd::RELATIVE_STATE_OF_CHARGE).await.at(cmd::RELATIVE_STATE_OF_CHARGE as u16)?;
        info.time_to_empty = self.read_word(cmd::AVERAGE_TIME_TO_EMPTY).await.at(cmd::AVERAGE_TIME_TO_EMPTY as u16)?;
        info.time_to_full = self.read_word(cmd::AVERAGE_TIME_TO_FULL).await.at(cmd::AVERAGE_TIME_TO_FULL as u16)?;
        info.battery_status = self.read_word(cmd::BATTERY_STATUS).await.at(cmd::BATTERY_STATUS as u16)?;
        info.cell5_voltage = self.read_word(cmd::CELL_VOLTAGE_5).await.at(cmd::CELL_VOLTAGE_5 as u16)?;
        info.charging_voltage = self.read_word(cmd::CHARGING_VOLTAGE).await.at(cmd::CHARGING_VOLTAGE as u16)?;
        info.charging_current =
            self.read_sword(cmd::CHARGING_CURRENT).await.at(cmd::CHARGING_CURRENT as u16)? as i32 * scale;

        // OperationStatus has the logical OR of these, so only read them when set
        info.safety_status = if sbs::bit(info.operation_status, op::SS) {
            self.read_dword(cmd::SAFETY_STATUS).await.at(cmd::SAFETY_STATUS as u16)?
        } else {
            0
        };
        info.pf_status = if sbs::bit(info.operation_status, op::PF) {
            self.read_dword(cmd::PF_STATUS).await.at(cmd::PF_STATUS as u16)?
        } else {
            0
        };
        Ok(())
    }

    // ========================================
    // Commands
    // ========================================

    /// Enters manual FET control emergency shutdown with the two command sequence (`bq_emshut_enter`).
    pub async fn emshut_enter(&mut self) -> Result<()> {
        self.mfg_access_cmd(mfg::MFC_ENABLE_A).await.at(mfg::MFC_ENABLE_A)?;
        self.check_status().await?;
        self.mfg_access_cmd(mfg::MFC_ENABLE_B).await.at(mfg::MFC_ENABLE_B)?;
        self.check_status().await
    }

    /// Leaves emergency shutdown (`bq_emshut_exit`).
    pub async fn emshut_exit(&mut self) -> Result<()> {
        self.mfg_access_cmd(mfg::MFC_DISABLE).await.at(mfg::MFC_DISABLE)?;
        self.check_status().await
    }

    /// Reads the side detect input; true is port (`bq_read_side_detect`).
    pub async fn read_side_detect(&mut self) -> Result<bool> {
        let gpio = self.read_word(cmd::GPIO_READ).await.at(cmd::GPIO_READ as u16)?;
        Ok(sbs::bit(gpio as u32, sbs::gpio_read::RH1))
    }

    /// Reads a 3 byte status register (`bq_read_gauging_status` / `bq_read_charging_status`).
    pub async fn read_status3(&mut self, command: u8) -> Result<u32> {
        let mut buf = [0u8; 4];
        match self.read_block(command, &mut buf).await.at(command as u16)? {
            3 => Ok(u32::from_le_bytes(buf)),
            _ => Err(BqError::smbus(SmbusError::InvalidResp, command as u16)),
        }
    }

    /// SBS integer read for the debug commands (`bq_dbg_read_sbs_int`).
    pub async fn dbg_read_sbs_int(&mut self, command: u8, width: Width) -> Result<u32> {
        let at = command as u16;
        Ok(match width {
            Width::W8 => self.read_byte(command).await.at(at)? as u32,
            Width::W16 => self.read_word(command).await.at(at)? as u32,
            Width::W32 => self.read_dword(command).await.at(at)?,
        })
    }

    /// SBS block read for the debug commands (`bq_dbg_read_sbs_block`).
    pub async fn dbg_read_sbs_block(&mut self, command: u8, buf: &mut [u8; BLOCK_LEN]) -> Result<usize> {
        self.read_block(command, buf).await.at(command as u16)
    }

    /// Manufacturer block read for the debug commands (`bq_dbg_read_mfg_block`).
    pub async fn dbg_read_mfg_block(&mut self, mac_cmd: u16, buf: &mut [u8; BLOCK_LEN]) -> Result<usize> {
        let len = self.mfg_access_read(mac_cmd, buf).await.at(mac_cmd)?;
        self.check_status().await?;
        Ok(len)
    }

    /// Manufacturer command for the debug commands (`bq_dbg_mfg_cmd`).
    pub async fn dbg_mfg_cmd(&mut self, mac_cmd: u16) -> Result<()> {
        self.mfg_access_cmd(mac_cmd).await.at(mac_cmd)?;
        self.check_status().await
    }

    /// Data flash write for the debug commands (`bq_dbg_df_write`).
    pub async fn dbg_df_write(&mut self, addr: u16, data: &[u8]) -> Result<()> {
        self.mfg_access_write(addr, data).await.at(addr)?;
        Timer::after(DF_WRITE_SETTLE).await;
        self.check_status().await
    }
}
