//! BQ40Z80 Smart Battery System registers, ported from `bq40z80_reg_map.h`, plus the decoding the C driver did
//! inline in `bq40z80.c`.
//!
//! Only the registers and bits the firmware or its debug commands use are listed. Bit constants are bit *indices*
//! (like the C `SBS_*` defines); test them with [`bit`].

/// True if bit `index` of `value` is set (`sbs_check_bit`).
pub const fn bit(value: u32, index: u8) -> bool {
    value & (1 << index) != 0
}

/// SMBus address of the gauge (`BQ_ADDR`).
pub const ADDRESS: u8 = 0x0B;

/// Value of the `DeviceType` manufacturer block for a BQ40Z80.
pub const DEVICE_TYPE_BQ40Z80: u16 = 0x4800;

/// Data flash address of the current scale factor (`Settings:Configuration:Scale Factor` style `Cur Scale`).
pub const DF_ADDR_SCALE_FACTOR: u16 = 0x4AE8;

/// SBS command codes (`enum sbs_command`).
pub mod cmd {
    pub const MANUFACTURER_ACCESS: u8 = 0x00;
    pub const TEMPERATURE: u8 = 0x08;
    pub const VOLTAGE: u8 = 0x09;
    pub const CURRENT: u8 = 0x0A;
    pub const AVERAGE_CURRENT: u8 = 0x0B;
    pub const MAX_ERROR: u8 = 0x0C;
    pub const RELATIVE_STATE_OF_CHARGE: u8 = 0x0D;
    pub const ABSOLUTE_STATE_OF_CHARGE: u8 = 0x0E;
    pub const REMAINING_CAPACITY: u8 = 0x0F;
    pub const FULL_CHARGE_CAPACITY: u8 = 0x10;
    pub const RUN_TIME_TO_EMPTY: u8 = 0x11;
    pub const AVERAGE_TIME_TO_EMPTY: u8 = 0x12;
    pub const AVERAGE_TIME_TO_FULL: u8 = 0x13;
    pub const CHARGING_CURRENT: u8 = 0x14;
    pub const CHARGING_VOLTAGE: u8 = 0x15;
    pub const BATTERY_STATUS: u8 = 0x16;
    pub const CYCLE_COUNT: u8 = 0x17;
    pub const DESIGN_CAPACITY: u8 = 0x18;
    pub const DESIGN_VOLTAGE: u8 = 0x19;
    pub const MANUFACTURER_DATE: u8 = 0x1B;
    pub const SERIAL_NUMBER: u8 = 0x1C;
    pub const MANUFACTURER_NAME: u8 = 0x20;
    pub const DEVICE_NAME: u8 = 0x21;
    pub const DEVICE_CHEMISTRY: u8 = 0x22;
    pub const CELL_VOLTAGE_7: u8 = 0x3C;
    pub const CELL_VOLTAGE_6: u8 = 0x3D;
    pub const CELL_VOLTAGE_5: u8 = 0x3E;
    pub const CELL_VOLTAGE_4: u8 = 0x3F;
    pub const MANUFACTURER_BLOCK_ACCESS: u8 = 0x44;
    pub const GPIO_READ: u8 = 0x48;
    pub const STATE_OF_HEALTH: u8 = 0x4F;
    pub const SAFETY_ALERT: u8 = 0x50;
    pub const SAFETY_STATUS: u8 = 0x51;
    pub const PF_ALERT: u8 = 0x52;
    pub const PF_STATUS: u8 = 0x53;
    pub const OPERATION_STATUS: u8 = 0x54;
    pub const CHARGING_STATUS: u8 = 0x55;
    pub const GAUGING_STATUS: u8 = 0x56;
    pub const MANUFACTURING_STATUS: u8 = 0x57;
    pub const DA_STATUS1: u8 = 0x71;
    pub const DA_STATUS2: u8 = 0x72;
    pub const FILTERED_CAPACITY: u8 = 0x78;
}

/// Manufacturer access commands (`enum sbs_mfg_commands`), sent through [`cmd::MANUFACTURER_BLOCK_ACCESS`].
pub mod mfg {
    pub const DEVICE_TYPE: u16 = 0x0001;
    pub const FIRMWARE_VERSION: u16 = 0x0002;
    pub const HARDWARE_VERSION: u16 = 0x0003;
    pub const CHEM_ID: u16 = 0x0006;
    pub const SHUTDOWN_MODE: u16 = 0x0010;
    pub const SLEEP_MODE: u16 = 0x0011;
    pub const FET_CONTROL: u16 = 0x0022;
    pub const DEVICE_RESET: u16 = 0x0041;
    pub const SAFETY_STATUS: u16 = 0x0051;
    pub const PF_STATUS: u16 = 0x0053;
    pub const OPERATION_STATUS: u16 = 0x0054;
    pub const MANUFACTURING_STATUS: u16 = 0x0057;
    pub const DA_STATUS1: u16 = 0x0071;
    pub const DA_STATUS2: u16 = 0x0072;
    /// Second half of the manual FET control (emergency shutdown) enable sequence.
    pub const MFC_ENABLE_B: u16 = 0x043D;
    /// Leaves manual FET control (emergency shutdown).
    pub const MFC_DISABLE: u16 = 0x23A7;
    /// First half of the manual FET control (emergency shutdown) enable sequence.
    pub const MFC_ENABLE_A: u16 = 0x270C;
}

/// `GPIORead()` bits.
pub mod gpio_read {
    /// Side detect input (port vs starboard harness).
    pub const RH1: u8 = 3;
}

/// `BatteryStatus()` bits.
pub mod battery_status {
    /// The low nibble is the error code of the last command.
    pub const EC_MASK: u16 = 0xF;
    pub const FD: u8 = 4;
    pub const FC: u8 = 5;
    /// Discharging or relaxing; cleared while charging.
    pub const DSG: u8 = 6;
    pub const INIT: u8 = 7;
}

/// `OperationStatus()` bits.
pub mod operation_status {
    pub const EMSHUT: u8 = 29;
    pub const INIT: u8 = 24;
    pub const SDM: u8 = 16;
    pub const XCHG: u8 = 14;
    pub const XDSG: u8 = 13;
    pub const PF: u8 = 12;
    pub const SS: u8 = 11;
    pub const SDV: u8 = 10;
    pub const CHG: u8 = 2;
    pub const DSG: u8 = 1;
    pub const PRES: u8 = 0;
}

/// `SafetyStatus()` bits referenced outside the name tables.
pub mod safety_status {
    pub const CUV: u8 = 0;
    pub const COV: u8 = 1;
}

/// A named status bit, for logs, the debug commands and the display.
pub type BitName = (u8, &'static str);

/// Looks up the name of `bit` in a table.
pub fn bit_name(names: &[BitName], bit: u8) -> Option<&'static str> {
    names.iter().find(|(b, _)| *b == bit).map(|(_, name)| *name)
}

/// `ManufacturingStatus()` bit names (`manufacturing_status_names`).
pub const MANUFACTURING_STATUS_NAMES: &[BitName] = &[
    (0, "Precharge FET Test [PCHG_EN]"),
    (1, "Charge FET Test [CHG_EN]"),
    (2, "Discharge FET Test [DSG_EN]"),
    (3, "Gas Gauging [GAUGE_EN]"),
    (4, "All FET Action [FET_EN]"),
    (5, "Lifetime Data Collection [LF_EN]"),
    (6, "Permanent Failure [PF_EN]"),
    (7, "Black Box Recorder [BBR_EN]"),
    (8, "Fuse Action [FUSE_EN]"),
    (9, "LED Display [LED_EN]"),
    (13, "Pre-discharge FET test [PDSG_EN]"),
    (14, "Lifetime Speed Up [LT_TEST]"),
    (15, "Calibration Mode [CAL_EN]"),
];

/// `SafetyStatus()` bit names (`safety_status_names`).
pub const SAFETY_STATUS_NAMES: &[BitName] = &[
    (29, "Overcurrent in discharge [OCDL]"),
    (28, "Cell overvoltage latch [COVL]"),
    (27, "Undertemperature during discharge [UTD]"),
    (26, "Undertemperature during charge [UTC]"),
    (25, "Over-precharge current [PCHGC]"),
    (24, "Overcharging voltage [CHGV]"),
    (23, "Overcharging current [CHGC]"),
    (22, "Overcharge [OC]"),
    (20, "Charge timeout [CTO]"),
    (18, "Precharge timeout [PTO]"),
    (16, "Overtemperature FET [OTF]"),
    (14, "Cell undervoltage compensated [CUVC]"),
    (13, "Overtemperature during discharge [OTD]"),
    (12, "Overtemperature during charge [OTC]"),
    (11, "Short-circuit during discharge latch [ASCDL]"),
    (10, "Short-circuit during discharge [ASCD]"),
    (9, "Short-circuit during charge latch [ASCCL]"),
    (8, "Short-circuit during charge [ASCC]"),
    (7, "Overload during discharge latch [AOLDL]"),
    (6, "Overload during discharge [AOLD]"),
    (5, "Overcurrent during discharge 2 [OCD2]"),
    (4, "Overcurrent during discharge 1 [OCD1]"),
    (3, "Overcurrent during charge 2 [OCC2]"),
    (2, "Overcurrent during charge 1 [OCC1]"),
    (1, "Cell overvoltage [COV]"),
    (0, "Cell undervoltage [CUV]"),
];

/// `PFStatus()` bit names (`pf_status_names`).
pub const PF_STATUS_NAMES: &[BitName] = &[
    (31, "Open thermistor-TS4 failure [TS4]"),
    (30, "Open thermistor-TS3 failure [TS3]"),
    (29, "Open thermistor-TS2 failure [TS2]"),
    (28, "Open thermistor-TS1 failure [TS1]"),
    (26, "Data flash wearout failure [DFW]"),
    (24, "Instruction flash checksum failure [IFC]"),
    (23, "PTC failure [PTC]"),
    (22, "Second level protector failure [2LVL]"),
    (21, "AFE communication failure [AFEC]"),
    (20, "AFE register failure [AFER]"),
    (19, "Chemical fuse failure [FUSE]"),
    (18, "Overcurrent in discharge [OCDL]"),
    (17, "Discharge FET failure [DFETF]"),
    (16, "Charge FET failure [CFETF]"),
    (15, "Short circuit in discharge [ASCDL]"),
    (14, "Short circuit in charge [ASCCL]"),
    (13, "Overload in discharge [AOLDL]"),
    (12, "Voltage imbalance while pack is active failure [VIMA]"),
    (11, "Voltage imbalance while pack at rest failure [VIMR]"),
    (10, "Capacity degradation failure [CD]"),
    (9, "Impedance failure [IMP]"),
    (8, "Cell balancing failure [CB]"),
    (7, "QMax imbalance failure [QIM]"),
    (6, "Safety overtemperature FET failure [SOTF]"),
    (5, "Cell overvoltage latch [COVL]"),
    (4, "Safety overtemperature cell failure [SOT]"),
    (3, "Safety overcurrent in discharge [SOCD]"),
    (2, "Safety overcurrent in charge [SOCC]"),
    (1, "Safety cell overvoltage failure [SOV]"),
    (0, "Safety cell undervoltage failure [SUV]"),
];

/// `BatteryStatus()` bit names (`battery_status_names`).
pub const BATTERY_STATUS_NAMES: &[BitName] = &[
    (0, "Error Code Bit 0 [EC0]"),
    (1, "Error Code Bit 1 [EC1]"),
    (2, "Error Code Bit 2 [EC2]"),
    (3, "Error Code Bit 3 [EC3]"),
    (4, "Fully Discharged [FD]"),
    (5, "Fully Charged [FC]"),
    (6, "Discharging/Relax Mode [DSG]"),
    (7, "Initialization [INIT]"),
    (8, "Remaining Time Alarm [RTA]"),
    (9, "Remaining Capacity Alarm [RCA]"),
    (11, "Terminate Discharge Alarm [TDA]"),
    (12, "Overtemperature Alarm [OTA]"),
    (14, "Terminate Charge Alarm [TCA]"),
    (15, "Overcharged Alarm [OCA]"),
];

/// `OperationStatus()` bit names (`operation_status_names`).
pub const OPERATION_STATUS_NAMES: &[BitName] = &[
    (31, "IATA charge control [IATA_CTERM]"),
    (29, "Emergency FET shutdown [EMSHUT]"),
    (28, "Cell balancing status [CB]"),
    (27, "CC measurement in SLEEP mode [SLPCC]"),
    (26, "ADC measurement in SLEEP mode [SLPAD]"),
    (24, "Initialization after full reset [INIT]"),
    (23, "SLEEP mode triggered via command [SLEEPM]"),
    (22, "400-kHz SMBus mode [XL]"),
    (21, "CAL_OFFSET"),
    (20, "CAL"),
    (19, "AUTOCALM"),
    (18, "Auth in progress [AUTH]"),
    (17, "LED Display [LED]"),
    (16, "Shutdown triggered via command [SDM]"),
    (15, "SLEEP mode conditions met [SLEEP]"),
    (14, "Charging disabled [XCHG]"),
    (13, "Discharging disabled [XDSG]"),
    (12, "PERMANENT FAILURE status [PF]"),
    (11, "SAFETY status [SS]"),
    (10, "Shutdown triggered from low batt voltage [SDV]"),
    (9, "Security Mdoe Bit 1 [SEC1]"),
    (8, "Security Mode Bit 0 [SEC0]"),
    (7, "Battery trip point interrupt [BTP_INT]"),
    (5, "Fuse status [FUSE]"),
    (4, "Pre-discharge FET status [PDSG]"),
    (3, "Precharge FET status [PCHG]"),
    (2, "CHG FET status [CHG]"),
    (1, "DSG FET status [DSG]"),
    (0, "System present low [PRES]"),
];

/// `ChargingStatus()` bit names (`charging_status_names`).
pub const CHARGING_STATUS_NAMES: &[BitName] = &[
    (19, "Near charge termination [NCT]"),
    (18, "Charging loss compensation [CCC]"),
    (17, "Charging voltage rate of change [CVR]"),
    (16, "Charging current rate of change [CCR]"),
    (15, "Charge termination [VCT]"),
    (14, "Maintenance charge [MCHG]"),
    (13, "Suspend charge [SU]"),
    (12, "Charge inhibit [IN]"),
    (11, "High voltage region [HV]"),
    (10, "Mid voltage region [MV]"),
    (9, "Low voltage region [LV]"),
    (8, "Precharge voltage region [PV]"),
    (6, "Overtemperature region [OT]"),
    (5, "High temperature region [HT]"),
    (4, "Standard temperature high region [STH]"),
    (3, "Recommended temperature region [RT]"),
    (2, "Standard temperature low region [STL]"),
    (1, "Low temperature region [LT]"),
    (0, "Undertemperature region [UT]"),
];

/// `GaugingStatus()` bit names (`gauging_status_names`).
pub const GAUGING_STATUS_NAMES: &[BitName] = &[
    (20, "Open circuit voltage in flat region (during RELAX) [OCVFR]"),
    (19, "LOAD mode [LDMD]"),
    (18, "Resistance update (toggles after every resistance update) [RX]"),
    (17, "QMax update (toggles after every QMax update) [QMAX]"),
    (16, "Discharge qualified for learning (opposite of the R_DIS flag) [VDQ]"),
    (15, "Negative scale factor mode [NSFM]"),
    (13, "OCV update in SLEEP mode [SLPQMAX]"),
    (12, "Impedance Track Gauging (Ra and QMax updates are enabled) [QEN]"),
    (11, "Voltages are OK for QMax update. This flag is updated at exit of the RELAX mode. [VOK]"),
    (10, "Resistance updates [R_DIS]"),
    (8, "Rest [REST]"),
    (7, "Condition Flag [CF]"),
    (6, "Discharge/relax [DSG]"),
    (5, "End-of-discharge termination voltage [EDV]"),
    (4, "Cell balancing [BAL_EN]"),
    (3, "Terminate charge [TC]"),
    (2, "Terminate discharge [TD]"),
    (1, "Fully charged [FC]"),
    (0, "Fully discharged [FD]"),
];

/// Text of a `BatteryStatus()` error code (`bq40_decode_err`).
pub fn battery_status_error_name(code: u8) -> &'static str {
    match code {
        0x00 => "Ok",
        0x01 => "Busy",
        0x02 => "Reserved Command",
        0x03 => "Unsupported Command",
        0x04 => "Access Denied",
        0x05 => "Overflow/Underflow",
        0x06 => "Bad Size",
        0x07 => "Unknown Error",
        _ => "Invalid Error",
    }
}

/// `DAStatus1()` (`struct sbs_da_status1`): cell voltages in mV, currents in mA, powers in cW.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DaStatus1 {
    pub cell_voltage: [u16; 4],
    pub batt_voltage: u16,
    pub pack_voltage: u16,
    pub cell_current: [i16; 4],
    pub cell_power: [i16; 4],
    pub instant_power: i16,
    pub avg_power: i16,
}

impl DaStatus1 {
    /// Length of the manufacturer block.
    pub const LEN: usize = 32;

    /// Decodes the little endian manufacturer block.
    pub fn parse(raw: &[u8; Self::LEN]) -> Self {
        let u = |i: usize| u16::from_le_bytes([raw[2 * i], raw[2 * i + 1]]);
        let s = |i: usize| u(i) as i16;
        DaStatus1 {
            cell_voltage: [u(0), u(1), u(2), u(3)],
            batt_voltage: u(4),
            pack_voltage: u(5),
            cell_current: [s(6), s(7), s(8), s(9)],
            cell_power: [s(10), s(11), s(12), s(13)],
            instant_power: s(14),
            avg_power: s(15),
        }
    }
}

/// Pack identity read once after connecting (`bq_mfg_info_t`).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MfgInfo {
    pub serial: u16,
    pub mfg_day: u8,
    pub mfg_month: u8,
    pub mfg_year: u16,
    pub device_type: u16,
    pub firmware_version: [u8; 11],
    /// Multiplier for the current and capacity registers.
    pub scale_factor: u8,
    pub manufacturing_status: u16,
}

/// Splits `ManufacturerDate()` into `(day, month, year)`.
pub fn decode_date(raw: u16) -> (u8, u8, u16) {
    ((raw & 0x1F) as u8, ((raw >> 5) & 0xF) as u8, (raw >> 9) + 1980)
}

/// Everything refreshed every poll (`bq_battery_info_t`). Currents are in mA after applying the scale factor.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BatteryInfo {
    pub operation_status: u32,
    pub da_status1: DaStatus1,
    pub current: i32,
    pub avg_current: i32,
    /// In 0.1 K.
    pub temperature: u16,
    pub max_error: u8,
    pub relative_soc: u8,
    /// Minutes, 65535 when not discharging.
    pub time_to_empty: u16,
    /// Minutes, 65535 when not charging.
    pub time_to_full: u16,
    pub battery_status: u16,
    pub cell5_voltage: u16,
    pub charging_voltage: u16,
    pub charging_current: i32,
    pub safety_status: u32,
    pub pf_status: u32,
}

impl BatteryInfo {
    /// The gauge is still initializing; nothing past `OperationStatus()` was read.
    pub fn initializing(&self) -> bool {
        bit(self.operation_status, operation_status::INIT)
    }

    /// `BatteryStatus()[DSG]` is clear, so the pack is charging.
    pub fn is_charging(&self) -> bool {
        !bit(self.battery_status as u32, battery_status::DSG)
    }

    /// Time to full while charging, otherwise time to empty, with the charging flag (`core1_time_remaining`).
    pub fn time_remaining(&self) -> (u16, bool) {
        if self.is_charging() { (self.time_to_full, true) } else { (self.time_to_empty, false) }
    }

    /// The five cell voltages in mV (`core1_cell_voltages`). Cell 5 has its own register.
    pub fn cell_voltages(&self) -> [u16; 5] {
        let c = self.da_status1.cell_voltage;
        [c[0], c[1], c[2], c[3], self.cell5_voltage]
    }

    /// Battery voltage in mV (`core1_voltage`).
    pub fn voltage(&self) -> u16 {
        self.da_status1.batt_voltage
    }
}

/// Validates a manufacturer block access response, which echoes the command first, and returns the data after it.
pub fn mac_response(mac_cmd: u16, response: &[u8]) -> Option<&[u8]> {
    match response {
        [lo, hi, data @ ..] if u16::from_le_bytes([*lo, *hi]) == mac_cmd => Some(data),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn date_decoding() {
        // 2023-07-26: day 26, month 7, year 43 since 1980
        let raw = (43 << 9) | (7 << 5) | 26;
        assert_eq!(decode_date(raw), (26, 7, 2023));
    }

    #[test]
    fn da_status1_layout() {
        let mut raw = [0u8; 32];
        raw[0..2].copy_from_slice(&3700u16.to_le_bytes());
        raw[6..8].copy_from_slice(&3650u16.to_le_bytes());
        raw[8..10].copy_from_slice(&18_400u16.to_le_bytes());
        raw[10..12].copy_from_slice(&18_390u16.to_le_bytes());
        raw[12..14].copy_from_slice(&(-1200i16).to_le_bytes());
        raw[30..32].copy_from_slice(&(-55i16).to_le_bytes());
        let da = DaStatus1::parse(&raw);
        assert_eq!(da.cell_voltage, [3700, 0, 0, 3650]);
        assert_eq!(da.batt_voltage, 18_400);
        assert_eq!(da.pack_voltage, 18_390);
        assert_eq!(da.cell_current[0], -1200);
        assert_eq!(da.avg_power, -55);
    }

    #[test]
    fn mac_response_checks_the_echo() {
        assert_eq!(mac_response(0x0001, &[0x01, 0x00, 0x00, 0x48]), Some(&[0x00, 0x48][..]));
        assert_eq!(mac_response(0x0001, &[0x02, 0x00, 0x00, 0x48]), None);
        assert_eq!(mac_response(0x0001, &[0x01]), None);
        assert_eq!(mac_response(0x270C, &[0x0C, 0x27]), Some(&[][..]));
    }

    #[test]
    fn time_remaining_follows_dsg() {
        let mut info = BatteryInfo { time_to_empty: 90, time_to_full: 30, ..Default::default() };
        info.battery_status = 1 << battery_status::DSG;
        assert_eq!(info.time_remaining(), (90, false));
        info.battery_status = 0;
        assert_eq!(info.time_remaining(), (30, true));
    }

    #[test]
    fn name_tables_have_unique_bits() {
        for table in [
            MANUFACTURING_STATUS_NAMES,
            SAFETY_STATUS_NAMES,
            PF_STATUS_NAMES,
            BATTERY_STATUS_NAMES,
            OPERATION_STATUS_NAMES,
            CHARGING_STATUS_NAMES,
            GAUGING_STATUS_NAMES,
        ] {
            for (i, (bit, _)) in table.iter().enumerate() {
                assert!(*bit < 32);
                assert!(table[i + 1..].iter().all(|(b, _)| b != bit));
            }
        }
        assert_eq!(bit_name(OPERATION_STATUS_NAMES, 29), Some("Emergency FET shutdown [EMSHUT]"));
        assert_eq!(bit_name(OPERATION_STATUS_NAMES, 30), None);
    }
}
