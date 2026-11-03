//! Control table of the XL430-W250, the servo the driver supports (`dynamixel_controls.h`), and decoding of the
//! EEPROM and RAM areas the driver reads in one go (`dynamixel_reg.c`).
//!
//! See <https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#control-table>.

/// Model number reported by ping (`DYNAMIXEL_XL430`).
pub const XL430_MODEL_NUMBER: u16 = 1060;

/// First address of the EEPROM area read by [`Eeprom::decode`].
pub const EEPROM_START: u16 = 0;
/// Bytes read from the EEPROM area.
pub const EEPROM_LENGTH: u16 = 61;
/// First address of the RAM area read by [`Ram::decode`].
pub const RAM_START: u16 = 64;
/// Bytes read from the RAM area.
pub const RAM_LENGTH: u16 = 83;

/// Control table addresses.
pub mod addr {
    // EEPROM
    pub const MODEL_NUMBER: u16 = 0;
    pub const MODEL_INFO: u16 = 2;
    pub const FIRMWARE_VERSION: u16 = 6;
    pub const ID: u16 = 7;
    pub const BAUD_RATE: u16 = 8;
    pub const RETURN_DELAY_TIME: u16 = 9;
    pub const DRIVE_MODE: u16 = 10;
    pub const OPERATING_MODE: u16 = 11;
    pub const SECONDARY_ID: u16 = 12;
    pub const PROTOCOL_TYPE: u16 = 13;
    pub const HOMING_OFFSET: u16 = 20;
    pub const MOVING_THRESHOLD: u16 = 24;
    pub const TEMPERATURE_LIMIT: u16 = 31;
    pub const MAX_VOLTAGE_LIMIT: u16 = 32;
    pub const MIN_VOLTAGE_LIMIT: u16 = 34;
    pub const PWM_LIMIT: u16 = 36;
    pub const VELOCITY_LIMIT: u16 = 44;
    pub const MAX_POSITION_LIMIT: u16 = 48;
    pub const MIN_POSITION_LIMIT: u16 = 52;
    pub const STARTUP_CONFIG: u16 = 60;
    /// Outside the 61 bytes read by the driver, so [`super::Eeprom::shutdown`] reads as 0 (the C driver read a stale
    /// buffer byte for it).
    pub const SHUTDOWN: u16 = 63;

    // RAM
    pub const TORQUE_ENABLE: u16 = 64;
    pub const LED: u16 = 65;
    pub const STATUS_RETURN_LEVEL: u16 = 68;
    pub const REGISTERED_INSTRUCTION: u16 = 69;
    pub const HARDWARE_ERROR_STATUS: u16 = 70;
    pub const VELOCITY_I_GAIN: u16 = 76;
    pub const VELOCITY_P_GAIN: u16 = 78;
    pub const POSITION_D_GAIN: u16 = 80;
    pub const POSITION_I_GAIN: u16 = 82;
    pub const POSITION_P_GAIN: u16 = 84;
    pub const FEEDFORWARD_2ND_GAIN: u16 = 88;
    pub const FEEDFORWARD_1ST_GAIN: u16 = 90;
    pub const BUS_WATCHDOG: u16 = 98;
    pub const GOAL_PWM: u16 = 100;
    pub const GOAL_VELOCITY: u16 = 104;
    pub const PROFILE_ACCELERATION: u16 = 108;
    pub const PROFILE_VELOCITY: u16 = 112;
    pub const GOAL_POSITION: u16 = 116;
    pub const REALTIME_TICK: u16 = 120;
    pub const MOVING: u16 = 122;
    pub const MOVING_STATUS: u16 = 123;
    pub const PRESENT_PWM: u16 = 124;
    pub const PRESENT_LOAD: u16 = 126;
    pub const PRESENT_VELOCITY: u16 = 128;
    pub const PRESENT_POSITION: u16 = 132;
    pub const VELOCITY_TRAJECTORY: u16 = 136;
    pub const POSITION_TRAJECTORY: u16 = 140;
    pub const PRESENT_INPUT_VOLTAGE: u16 = 144;
    pub const PRESENT_TEMPERATURE: u16 = 146;
    /// Outside the 83 bytes read by the driver, so [`super::Ram::backup_ready`] reads as false (the C driver read a
    /// stale buffer byte for it).
    pub const BACKUP_READY: u16 = 147;
}

/// Reads little endian fields out of a block read starting at `base`. Bytes past the end read as 0.
struct Fields<'a> {
    data: &'a [u8],
    base: u16,
}

impl Fields<'_> {
    fn byte(&self, address: u16, i: u16) -> u8 {
        self.data.get((address - self.base + i) as usize).copied().unwrap_or(0)
    }
    fn u8(&self, address: u16) -> u8 {
        self.byte(address, 0)
    }
    fn u16(&self, address: u16) -> u16 {
        u16::from_le_bytes([self.byte(address, 0), self.byte(address, 1)])
    }
    fn u32(&self, address: u16) -> u32 {
        u32::from_le_bytes([self.byte(address, 0), self.byte(address, 1), self.byte(address, 2), self.byte(address, 3)])
    }
    fn bool(&self, address: u16) -> bool {
        self.u8(address) != 0
    }
}

/// The EEPROM area (`struct dynamixel_eeprom`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Eeprom {
    pub model_num: u16,
    pub model_info: u32,
    pub firmware_version: u8,
    pub id: u8,
    pub baud_rate: u8,
    pub return_delay_time: u8,
    pub drive_mode: u8,
    pub operating_mode: u8,
    pub secondary_id: u8,
    pub protocol_type: u8,
    pub homing_offset: i32,
    pub moving_threshold: u32,
    pub temperature_limit: u8,
    pub max_voltage_limit: u16,
    pub min_voltage_limit: u16,
    pub pwm_limit: u16,
    pub velocity_limit: u32,
    pub max_position_limit: u32,
    pub min_position_limit: u32,
    pub startup_config: u8,
    pub shutdown: u8,
}

impl Eeprom {
    /// Decodes the parameters of the reply to a read of [`EEPROM_LENGTH`] bytes at [`EEPROM_START`]
    /// (`dynamixel_reg_eeprom_decode`).
    pub fn decode(data: &[u8]) -> Self {
        let f = Fields { data, base: EEPROM_START };
        Eeprom {
            model_num: f.u16(addr::MODEL_NUMBER),
            model_info: f.u32(addr::MODEL_INFO),
            firmware_version: f.u8(addr::FIRMWARE_VERSION),
            id: f.u8(addr::ID),
            baud_rate: f.u8(addr::BAUD_RATE),
            return_delay_time: f.u8(addr::RETURN_DELAY_TIME),
            drive_mode: f.u8(addr::DRIVE_MODE),
            operating_mode: f.u8(addr::OPERATING_MODE),
            secondary_id: f.u8(addr::SECONDARY_ID),
            protocol_type: f.u8(addr::PROTOCOL_TYPE),
            homing_offset: f.u32(addr::HOMING_OFFSET) as i32,
            moving_threshold: f.u32(addr::MOVING_THRESHOLD),
            temperature_limit: f.u8(addr::TEMPERATURE_LIMIT),
            max_voltage_limit: f.u16(addr::MAX_VOLTAGE_LIMIT),
            min_voltage_limit: f.u16(addr::MIN_VOLTAGE_LIMIT),
            pwm_limit: f.u16(addr::PWM_LIMIT),
            velocity_limit: f.u32(addr::VELOCITY_LIMIT),
            max_position_limit: f.u32(addr::MAX_POSITION_LIMIT),
            min_position_limit: f.u32(addr::MIN_POSITION_LIMIT),
            startup_config: f.u8(addr::STARTUP_CONFIG),
            shutdown: f.u8(addr::SHUTDOWN),
        }
    }
}

/// The RAM area (`struct dynamixel_ram`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ram {
    pub torque_enable: bool,
    pub led: bool,
    pub status_return_level: u8,
    pub registered_instruction: bool,
    pub hardware_error_status: u8,
    pub velocity_i_gain: u16,
    pub velocity_p_gain: u16,
    pub position_d_gain: u16,
    pub position_i_gain: u16,
    pub position_p_gain: u16,
    pub feedforward_2nd_gain: u16,
    pub feedforward_1st_gain: u16,
    pub bus_watchdog: i8,
    pub goal_pwm: i16,
    pub goal_velocity: i32,
    pub profile_accel: u32,
    pub profile_vel: u32,
    pub goal_position: i32,
    pub realtime_tick: u16,
    pub moving: bool,
    pub moving_status: u8,
    pub present_pwm: i16,
    pub present_load: i16,
    pub present_velocity: i32,
    pub present_position: i32,
    pub velocity_trajectory: u32,
    pub position_trajectory: u32,
    /// In units of 0.1 V.
    pub present_input_voltage: u16,
    /// In °C.
    pub present_temperature: u8,
    pub backup_ready: bool,
}

impl Ram {
    /// Decodes the parameters of the reply to a read of [`RAM_LENGTH`] bytes at [`RAM_START`]
    /// (`dynamixel_reg_ram_decode`).
    pub fn decode(data: &[u8]) -> Self {
        let f = Fields { data, base: RAM_START };
        Ram {
            torque_enable: f.bool(addr::TORQUE_ENABLE),
            led: f.bool(addr::LED),
            status_return_level: f.u8(addr::STATUS_RETURN_LEVEL),
            registered_instruction: f.bool(addr::REGISTERED_INSTRUCTION),
            hardware_error_status: f.u8(addr::HARDWARE_ERROR_STATUS),
            velocity_i_gain: f.u16(addr::VELOCITY_I_GAIN),
            velocity_p_gain: f.u16(addr::VELOCITY_P_GAIN),
            position_d_gain: f.u16(addr::POSITION_D_GAIN),
            position_i_gain: f.u16(addr::POSITION_I_GAIN),
            position_p_gain: f.u16(addr::POSITION_P_GAIN),
            feedforward_2nd_gain: f.u16(addr::FEEDFORWARD_2ND_GAIN),
            feedforward_1st_gain: f.u16(addr::FEEDFORWARD_1ST_GAIN),
            bus_watchdog: f.u8(addr::BUS_WATCHDOG) as i8,
            goal_pwm: f.u16(addr::GOAL_PWM) as i16,
            goal_velocity: f.u32(addr::GOAL_VELOCITY) as i32,
            profile_accel: f.u32(addr::PROFILE_ACCELERATION),
            profile_vel: f.u32(addr::PROFILE_VELOCITY),
            goal_position: f.u32(addr::GOAL_POSITION) as i32,
            realtime_tick: f.u16(addr::REALTIME_TICK),
            moving: f.bool(addr::MOVING),
            moving_status: f.u8(addr::MOVING_STATUS),
            present_pwm: f.u16(addr::PRESENT_PWM) as i16,
            present_load: f.u16(addr::PRESENT_LOAD) as i16,
            present_velocity: f.u32(addr::PRESENT_VELOCITY) as i32,
            present_position: f.u32(addr::PRESENT_POSITION) as i32,
            velocity_trajectory: f.u32(addr::VELOCITY_TRAJECTORY),
            position_trajectory: f.u32(addr::POSITION_TRAJECTORY),
            present_input_voltage: f.u16(addr::PRESENT_INPUT_VOLTAGE),
            present_temperature: f.u8(addr::PRESENT_TEMPERATURE),
            backup_ready: f.bool(addr::BACKUP_READY),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_eeprom() {
        let mut data = [0u8; EEPROM_LENGTH as usize];
        data[0..2].copy_from_slice(&XL430_MODEL_NUMBER.to_le_bytes());
        data[6] = 45;
        data[7] = 2;
        data[8] = 1;
        data[9] = 250;
        data[11] = 3;
        data[20..24].copy_from_slice(&(-345i32).to_le_bytes());
        data[32..34].copy_from_slice(&140u16.to_le_bytes());
        data[34..36].copy_from_slice(&60u16.to_le_bytes());
        data[48..52].copy_from_slice(&4095u32.to_le_bytes());
        data[60] = 1;
        let e = Eeprom::decode(&data);
        assert_eq!(e.model_num, 1060);
        assert_eq!((e.firmware_version, e.id, e.baud_rate, e.return_delay_time), (45, 2, 1, 250));
        assert_eq!(e.operating_mode, 3);
        assert_eq!(e.homing_offset, -345);
        assert_eq!((e.max_voltage_limit, e.min_voltage_limit), (140, 60));
        assert_eq!(e.max_position_limit, 4095);
        assert_eq!(e.startup_config, 1);
        // Address 63 lies past the 61 bytes read, as in the C driver
        assert_eq!(e.shutdown, 0);
    }

    #[test]
    fn decodes_ram() {
        let mut data = [0u8; RAM_LENGTH as usize];
        let at = |a: u16| (a - RAM_START) as usize;
        data[at(addr::TORQUE_ENABLE)] = 1;
        data[at(addr::HARDWARE_ERROR_STATUS)] = 0x24;
        data[at(addr::BUS_WATCHDOG)] = 0xFF;
        data[at(addr::GOAL_POSITION)..][..4].copy_from_slice(&(-12288i32).to_le_bytes());
        data[at(addr::MOVING)] = 1;
        data[at(addr::PRESENT_LOAD)..][..2].copy_from_slice(&(-100i16).to_le_bytes());
        data[at(addr::PRESENT_POSITION)..][..4].copy_from_slice(&6144i32.to_le_bytes());
        data[at(addr::PRESENT_INPUT_VOLTAGE)..][..2].copy_from_slice(&121u16.to_le_bytes());
        data[at(addr::PRESENT_TEMPERATURE)] = 31;
        let r = Ram::decode(&data);
        assert!(r.torque_enable && r.moving && !r.led);
        assert_eq!(r.hardware_error_status, 0x24);
        assert_eq!(r.bus_watchdog, -1);
        assert_eq!(r.goal_position, -12288);
        assert_eq!(r.present_load, -100);
        assert_eq!(r.present_position, 6144);
        assert_eq!(r.present_input_voltage, 121);
        assert_eq!(r.present_temperature, 31);
        // The temperature is the last byte read; backup ready lies just past it and reads as false
        assert_eq!(at(addr::PRESENT_TEMPERATURE), RAM_LENGTH as usize - 1);
        assert!(!r.backup_ready);
    }
}
