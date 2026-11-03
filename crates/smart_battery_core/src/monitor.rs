//! The pack supervision logic of the C `core1.c`, without the SMBus I/O.
//!
//! The gauge task polls the BQ40Z80 every [`REFRESH_PERIOD_MS`]. After each poll it hands the result to [`Monitor`],
//! which tracks the connection, the emergency FET shutdown (power cycle / latch off) sequence and the pack state
//! shown on the display and used for the auto power off. The firmware performs whatever I/O the monitor asks for
//! (emergency shutdown exit, wake pulse) and raises the faults.
//!
//! Times are milliseconds on any monotonic clock.

use crate::sbs::{self, BatteryInfo, operation_status as op};

/// Time between gauge polls (`BQ40Z80_REFRESH_TIME_MS`).
pub const REFRESH_PERIOD_MS: u64 = 200;
/// Consecutive failed polls tolerated before the gauge counts as disconnected (`BQ_MAX_ERRORS_BEFORE_DISCONNECT`).
pub const MAX_ERRORS_BEFORE_DISCONNECT: u8 = 3;
/// How long the robot stays off for a power cycle (`ROBOT_POWER_CYCLE_TIME_MS`).
pub const ROBOT_POWER_CYCLE_TIME_MS: u32 = 10_000;
/// How long to wait for the gauge to report EMSHUT after a command (`EMSHUT_STATE_TIMEOUT`). Must exceed the gauge's
/// Manual FET Control Delay setting.
pub const EMSHUT_STATE_TIMEOUT_MS: u64 = 5000;
/// Length of the pulse on the wake pin (`BATT_WAKE_PULSE_DURATION_MS`).
pub const WAKE_PULSE_DURATION_MS: u64 = 500;
/// SOC above which the green LED is lit (`WARN_SOC_THRESH`).
pub const WARN_SOC_THRESH: u8 = 60;
/// SOC above which the yellow LED is lit; red at or below (`STOP_SOC_THRESH`).
pub const STOP_SOC_THRESH: u8 = 30;

/// The pack state (`batt_state_t`). The discriminants match C.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum BattState {
    /// The gauge task hasn't completed a poll yet.
    #[default]
    Uninitialized = 0,
    /// No connection to the gauge.
    Disconnected = 1,
    /// The pack isn't plugged into anything.
    Removed = 2,
    /// The gauge is shutting itself off; stop waking it.
    NeedsShutdown = 3,
    /// The gauge is initializing after startup.
    Initializing = 4,
    /// Plugged in, but discharging is disabled.
    Xdsg = 5,
    Discharging = 6,
    Charging = 7,
    /// Permanent failure: the pack needs disassembly.
    PermanentFail = 8,
    /// An emergency FET shutdown power cycle is in progress.
    PowerCycle = 9,
    /// Latched off until woken or unplugged.
    LatchOff = 10,
}

impl BattState {
    /// Display name (`batt_state_names`).
    pub fn name(self) -> &'static str {
        match self {
            BattState::Uninitialized => "Uninitialized",
            BattState::Disconnected => "Disconnected",
            BattState::Removed => "Removed",
            BattState::NeedsShutdown => "Shutting Off",
            BattState::Initializing => "Initializing",
            BattState::Xdsg => "Dsg Inhibited",
            BattState::Discharging => "Discharging",
            BattState::Charging => "Charging",
            BattState::PermanentFail => "Permanent Fail",
            BattState::PowerCycle => "Power Cycling",
            BattState::LatchOff => "Latched Off",
        }
    }

    /// The gauge has answered since the last disconnect (`shared_status_is_connected`).
    pub fn is_connected(self) -> bool {
        !matches!(self, BattState::Disconnected | BattState::Uninitialized)
    }

    /// Plugged in and powering (or being charged through) PACK+: the robot is using the pack.
    pub fn in_use(self) -> bool {
        matches!(self, BattState::Charging | BattState::Discharging)
    }

    /// The pack is idle or in use, so its manufacturer info can be reported (`pack_ready` in `main.c`).
    pub fn pack_ready(self) -> bool {
        matches!(self, BattState::Removed | BattState::Discharging | BattState::Charging)
    }
}

/// Emergency FET shutdown sequence (`enum emshut_state`).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum EmshutState {
    #[default]
    Normal = 0,
    PowerCycle = 1,
    LatchOff = 2,
}

/// Which SOC LED to light: 0 red, 1 yellow, 2 green (`bq_update_soc_leds`).
pub fn soc_led(soc: u8) -> usize {
    if soc > WARN_SOC_THRESH {
        2
    } else if soc > STOP_SOC_THRESH {
        1
    } else {
        0
    }
}

/// What to do about an emergency FET shutdown after a poll.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EmshutAction {
    None,
    /// Send `MFC_DISABLE` and report the result with [`Monitor::emshut_exit_result`].
    Exit,
    /// The gauge never reported EMSHUT after a command: raise `FAULT_BQ40_COMMAND_FAIL` (without argument).
    CommandTimedOut,
}

/// What to do after a failed poll.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ErrorAction {
    /// Within the error budget; try again next poll.
    Retry,
    /// Raise `FAULT_BQ40_NOT_CONNECTED` with the error and clear the SOC LEDs, and pulse the wake pin if `wake`.
    Disconnected { wake: bool },
}

/// Emergency shutdown commands (`FIFO_CMD_POWER_CYCLE` / `FIFO_CMD_KILL_POWER`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EmshutCommand {
    /// Turn the robot off for this many milliseconds.
    PowerCycle { duration_ms: u32 },
    /// Turn the robot off until the pack is woken or unplugged.
    KillPower,
}

/// Outcome of [`Monitor::request_emshut`].
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EmshutRequest {
    /// Accepted; the gauge task must now send the MFC enable sequence (`bq_emshut_enter`).
    Enter,
    /// Accepted, but no command is sent. See [`Monitor::request_emshut`].
    Wait,
    /// Refused: raise `FAULT_BQ40_COMMAND_FAIL` with this argument (`(state << 16) | emshut_state`).
    Refused(u32),
}

/// Connection, emergency shutdown and pack state tracking (the state kept by `core1_main`).
#[derive(Clone, Debug, Default)]
pub struct Monitor {
    state: BattState,
    error_count: u8,
    emshut: EmshutState,
    waiting_for_emshut_flag: bool,
    emshut_seen_timeout: u64,
    power_cycle_time_ms: u32,
    power_cycle_timeout: u64,
}

impl Monitor {
    pub const fn new() -> Self {
        Monitor {
            state: BattState::Uninitialized,
            error_count: 0,
            emshut: EmshutState::Normal,
            waiting_for_emshut_flag: false,
            emshut_seen_timeout: 0,
            power_cycle_time_ms: 0,
            power_cycle_timeout: 0,
        }
    }

    pub fn state(&self) -> BattState {
        self.state
    }

    pub fn emshut_state(&self) -> EmshutState {
        self.emshut
    }

    /// True if the next poll must start by reading the manufacturer info.
    pub fn needs_mfg_info(&self) -> bool {
        !self.state.is_connected()
    }

    /// A poll failed.
    pub fn poll_failed(&mut self) -> ErrorAction {
        if self.state != BattState::Disconnected && self.error_count < MAX_ERRORS_BEFORE_DISCONNECT {
            self.error_count += 1;
            return ErrorAction::Retry;
        }
        // Keep the state while the gauge shuts itself off, or this would keep waking it
        let wake = self.state != BattState::NeedsShutdown;
        if wake {
            self.state = BattState::Disconnected;
        }
        ErrorAction::Disconnected { wake }
    }

    /// First half of handling a successful poll: steps the emergency shutdown sequence.
    ///
    /// Call [`Monitor::emshut_exit_result`] if this returns [`EmshutAction::Exit`], then [`Monitor::update_state`].
    pub fn poll_succeeded(&mut self, now: u64, info: &BatteryInfo) -> EmshutAction {
        self.error_count = 0;
        let mut action = EmshutAction::None;
        if sbs::bit(info.operation_status, op::EMSHUT) {
            match self.emshut {
                EmshutState::PowerCycle if self.waiting_for_emshut_flag => {
                    // First time seeing the flag: the robot is off, start the power cycle timer
                    self.power_cycle_timeout = now + self.power_cycle_time_ms as u64;
                }
                EmshutState::PowerCycle if now >= self.power_cycle_timeout => action = EmshutAction::Exit,
                EmshutState::PowerCycle | EmshutState::LatchOff => {}
                // In shutdown without a reason (latch off released, or a timed out command went through late)
                EmshutState::Normal => action = EmshutAction::Exit,
            }
            self.waiting_for_emshut_flag = false;
        } else if self.waiting_for_emshut_flag && now >= self.emshut_seen_timeout {
            // The command never took effect. Don't cancel it: if it goes through later the code above recovers
            self.waiting_for_emshut_flag = false;
            self.emshut = EmshutState::Normal;
            action = EmshutAction::CommandTimedOut;
        }
        action
    }

    /// Reports whether the `MFC_DISABLE` asked for by [`Monitor::poll_succeeded`] worked. On failure the firmware
    /// raises `FAULT_BQ40_COMMAND_FAIL` and the exit is retried on the next poll.
    pub fn emshut_exit_result(&mut self, ok: bool) {
        if ok && self.emshut == EmshutState::PowerCycle {
            self.emshut = EmshutState::Normal;
        }
    }

    /// Second half of handling a successful poll: works out the new pack state.
    pub fn update_state(&mut self, info: &BatteryInfo) -> BattState {
        let ops = info.operation_status;
        self.state = if sbs::bit(ops, op::SDM) || sbs::bit(ops, op::SDV) {
            BattState::NeedsShutdown
        } else if info.pf_status != 0 {
            BattState::PermanentFail
        } else if sbs::bit(ops, op::INIT) {
            // The data is garbage while the gauge initializes
            BattState::Initializing
        } else if self.emshut == EmshutState::PowerCycle && !self.waiting_for_emshut_flag {
            BattState::PowerCycle
        } else if self.emshut == EmshutState::LatchOff && !self.waiting_for_emshut_flag {
            BattState::LatchOff
        } else if sbs::bit(ops, op::PRES) {
            if info.is_charging() {
                // Charge current above the threshold, only possible with power on PACK+. Takes priority over XDSG
                BattState::Charging
            } else if sbs::bit(ops, op::XDSG) || !sbs::bit(ops, op::DSG) {
                // Plugged in but not allowed to discharge: whoever plugged it in expects power, so flag it
                BattState::Xdsg
            } else {
                BattState::Discharging
            }
        } else {
            BattState::Removed
        };
        self.state
    }

    /// Handles a power cycle or kill power request (the `FIFO_CMD_POWER_CYCLE` / `FIFO_CMD_KILL_POWER` handler).
    ///
    /// Only accepted while the pack is in use and no shutdown is in progress.
    ///
    /// Like C, a power cycle request only arms the sequence and returns [`EmshutRequest::Wait`]: the C firmware
    /// never sends the MFC enable sequence for it (only for kill power), so a power cycle times out with
    /// `FAULT_BQ40_COMMAND_FAIL` after [`EMSHUT_STATE_TIMEOUT_MS`] unless something else puts the gauge into
    /// emergency shutdown. This looks like a C bug, but sending the command would cut robot power where the C
    /// firmware didn't, so it is kept until verified on hardware.
    pub fn request_emshut(&mut self, now: u64, command: EmshutCommand) -> EmshutRequest {
        if !self.state.in_use() || self.emshut != EmshutState::Normal {
            return EmshutRequest::Refused((self.state as u32) << 16 | self.emshut as u32);
        }
        self.emshut_seen_timeout = now + EMSHUT_STATE_TIMEOUT_MS;
        self.waiting_for_emshut_flag = true;
        match command {
            EmshutCommand::PowerCycle { duration_ms } => {
                self.emshut = EmshutState::PowerCycle;
                self.power_cycle_time_ms = duration_ms;
                EmshutRequest::Wait
            }
            EmshutCommand::KillPower => {
                // Big hammer, used on leak detection. The pack returns to normal when power cycled
                self.emshut = EmshutState::LatchOff;
                EmshutRequest::Enter
            }
        }
    }

    /// Whether the side detect pin may be read: PACK+ is only meaningful while the pack is in use.
    /// Returns the fault argument for `FAULT_BQ40_COMMAND_FAIL` when it may not.
    pub fn check_side_detect(&self) -> Result<(), u32> {
        if self.state.in_use() { Ok(()) } else { Err(self.state as u32) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sbs::battery_status;

    fn info(ops: &[u8], charging: bool) -> BatteryInfo {
        let mut info = BatteryInfo::default();
        for bit in ops {
            info.operation_status |= 1 << bit;
        }
        if !charging {
            info.battery_status = 1 << battery_status::DSG;
        }
        info
    }

    fn connected(state_ops: &[u8]) -> Monitor {
        let mut m = Monitor::new();
        let i = info(state_ops, false);
        assert_eq!(m.poll_succeeded(0, &i), EmshutAction::None);
        m.update_state(&i);
        m
    }

    #[test]
    fn soc_leds() {
        assert_eq!(soc_led(100), 2);
        assert_eq!(soc_led(61), 2);
        assert_eq!(soc_led(60), 1);
        assert_eq!(soc_led(31), 1);
        assert_eq!(soc_led(30), 0);
        assert_eq!(soc_led(0), 0);
    }

    #[test]
    fn state_classification() {
        let mut m = Monitor::new();
        assert_eq!(m.update_state(&info(&[op::PRES, op::DSG], false)), BattState::Discharging);
        assert_eq!(m.update_state(&info(&[op::PRES, op::DSG], true)), BattState::Charging);
        // Charging wins over XDSG
        assert_eq!(m.update_state(&info(&[op::PRES, op::XDSG], true)), BattState::Charging);
        assert_eq!(m.update_state(&info(&[op::PRES, op::DSG, op::XDSG], false)), BattState::Xdsg);
        assert_eq!(m.update_state(&info(&[op::PRES], false)), BattState::Xdsg);
        assert_eq!(m.update_state(&info(&[op::DSG], false)), BattState::Removed);
        assert_eq!(m.update_state(&info(&[op::PRES, op::INIT], false)), BattState::Initializing);
        let mut pf = info(&[op::PRES, op::DSG, op::INIT], false);
        pf.pf_status = 1;
        assert_eq!(m.update_state(&pf), BattState::PermanentFail);
        // Shutdown beats everything
        pf.operation_status |= 1 << op::SDV;
        assert_eq!(m.update_state(&pf), BattState::NeedsShutdown);
        assert_eq!(m.update_state(&info(&[op::SDM], false)), BattState::NeedsShutdown);
    }

    #[test]
    fn disconnect_after_three_errors_and_wake() {
        let mut m = Monitor::new();
        // Even before the first successful poll, three failures are tolerated
        for _ in 0..3 {
            assert_eq!(m.poll_failed(), ErrorAction::Retry);
        }
        assert_eq!(m.poll_failed(), ErrorAction::Disconnected { wake: true });
        assert_eq!(m.state(), BattState::Disconnected);
        assert!(m.needs_mfg_info());
        // While disconnected, every failure re-raises and wakes
        assert_eq!(m.poll_failed(), ErrorAction::Disconnected { wake: true });

        // A successful poll resets the budget
        let i = info(&[op::PRES, op::DSG], false);
        m.poll_succeeded(0, &i);
        assert_eq!(m.update_state(&i), BattState::Discharging);
        assert!(!m.needs_mfg_info());
        assert_eq!(m.poll_failed(), ErrorAction::Retry);
    }

    #[test]
    fn no_wake_while_shutting_down() {
        let mut m = connected(&[op::SDM]);
        assert_eq!(m.state(), BattState::NeedsShutdown);
        for _ in 0..3 {
            assert_eq!(m.poll_failed(), ErrorAction::Retry);
        }
        assert_eq!(m.poll_failed(), ErrorAction::Disconnected { wake: false });
        assert_eq!(m.state(), BattState::NeedsShutdown);
    }

    #[test]
    fn kill_power_latches_off() {
        let mut m = connected(&[op::PRES, op::DSG]);
        assert_eq!(m.request_emshut(1000, EmshutCommand::KillPower), EmshutRequest::Enter);
        // Before the gauge reports EMSHUT the state stays as it was
        let normal = info(&[op::PRES, op::DSG], false);
        assert_eq!(m.poll_succeeded(1200, &normal), EmshutAction::None);
        assert_eq!(m.update_state(&normal), BattState::Discharging);

        let off = info(&[op::PRES, op::EMSHUT], false);
        assert_eq!(m.poll_succeeded(1400, &off), EmshutAction::None);
        assert_eq!(m.update_state(&off), BattState::LatchOff);
        // Stays latched
        assert_eq!(m.poll_succeeded(60_000, &off), EmshutAction::None);
        assert_eq!(m.update_state(&off), BattState::LatchOff);
        // A second request is refused
        assert_eq!(
            m.request_emshut(60_000, EmshutCommand::KillPower),
            EmshutRequest::Refused((BattState::LatchOff as u32) << 16 | EmshutState::LatchOff as u32)
        );
    }

    #[test]
    fn power_cycle_sequence() {
        let mut m = connected(&[op::PRES, op::DSG]);
        assert_eq!(m.request_emshut(0, EmshutCommand::PowerCycle { duration_ms: 10_000 }), EmshutRequest::Wait);

        let off = info(&[op::PRES, op::EMSHUT], false);
        // First sighting starts the timer at 1000
        assert_eq!(m.poll_succeeded(1000, &off), EmshutAction::None);
        assert_eq!(m.update_state(&off), BattState::PowerCycle);
        assert_eq!(m.poll_succeeded(10_999, &off), EmshutAction::None);
        assert_eq!(m.poll_succeeded(11_000, &off), EmshutAction::Exit);
        // A failed exit is retried
        m.emshut_exit_result(false);
        assert_eq!(m.update_state(&off), BattState::PowerCycle);
        assert_eq!(m.poll_succeeded(11_200, &off), EmshutAction::Exit);
        m.emshut_exit_result(true);
        assert_eq!(m.emshut_state(), EmshutState::Normal);
        // Still flagged by the gauge until MFC_DISABLE takes effect, state shows the plain pack state
        assert_eq!(m.update_state(&off), BattState::Xdsg);
        // Seeing EMSHUT in normal state asks for another exit
        assert_eq!(m.poll_succeeded(11_400, &off), EmshutAction::Exit);
    }

    #[test]
    fn emshut_command_timeout() {
        let mut m = connected(&[op::PRES, op::DSG]);
        m.request_emshut(0, EmshutCommand::PowerCycle { duration_ms: 10_000 });
        let normal = info(&[op::PRES, op::DSG], false);
        assert_eq!(m.poll_succeeded(4999, &normal), EmshutAction::None);
        // While waiting for the flag the state isn't reported as power cycling
        assert_eq!(m.update_state(&normal), BattState::Discharging);
        assert_eq!(m.poll_succeeded(5000, &normal), EmshutAction::CommandTimedOut);
        assert_eq!(m.emshut_state(), EmshutState::Normal);
        assert_eq!(m.poll_succeeded(5200, &normal), EmshutAction::None);
    }

    #[test]
    fn requests_need_the_pack_in_use() {
        let mut m = connected(&[op::DSG]);
        assert_eq!(m.state(), BattState::Removed);
        assert_eq!(m.check_side_detect(), Err(BattState::Removed as u32));
        assert_eq!(
            m.request_emshut(0, EmshutCommand::KillPower),
            EmshutRequest::Refused((BattState::Removed as u32) << 16)
        );
        let m = connected(&[op::PRES, op::DSG]);
        assert_eq!(m.check_side_detect(), Ok(()));
    }
}
