//! Kill switch logic of the power boards, which are the vehicle's kill switch source.
//!
//! * Kill switch IDs follow `riptide_msgs2/KillSwitchReport`: [`KILL_SWITCH_PHYSICAL`] is the physical switch,
//!   IDs 1 to [`NUM_KILL_SWITCHES`]` - 1` take software kill commands from `command/software_kill`.
//! * A software kill switch that asserts kill is locked to the sender that killed it: only the same `sender_id` may
//!   release it ([`SoftwareKillLocks`], from `software_kill_subscription_callback`).
//! * `state/physkill_notify` is only published when the physical switch changes ([`PhysKillNotifier`]).
//! * `state/kill` is published periodically and as soon as the kill state flips ([`KillEdge`], from the
//!   `prev_kill_state` tracking in `safety_interface.c`).

/// Number of kill switches defined by `riptide_msgs2/KillSwitchReport` (`NUM_KILL_SWITCHES`).
pub const NUM_KILL_SWITCHES: u8 = 4;
/// The physical kill switch (`KILL_SWITCH_PHYSICAL`). It can't be driven by software kill commands.
pub const KILL_SWITCH_PHYSICAL: u8 = 0;
/// Size of the locking sender ID buffer (`SAFETY_SOFTWARE_KILL_FRAME_STR_SIZE`). Sender IDs must be shorter.
pub const FRAME_STR_SIZE: usize = 32;

/// A `riptide_msgs2/KillSwitchReport` received on `command/software_kill`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct SoftwareKill<'a> {
    pub kill_switch_id: u8,
    pub asserting_kill: bool,
    pub needs_update: bool,
    pub sender_id: &'a [u8],
}

/// Why a software kill command was refused. Each one raises `FAULT_ROS_BAD_COMMAND`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Rejected {
    /// The ID is out of range or the physical switch.
    InvalidSwitch(u8),
    /// The sender ID doesn't fit the lock buffer.
    SenderTooLong(usize),
    /// A different sender tried to release a switch it didn't kill.
    WrongSender,
}

impl Rejected {
    /// The fault's `extra` argument, as the C callback passed it.
    pub fn fault_extra(&self) -> u32 {
        match *self {
            Rejected::InvalidSwitch(id) => id as u32,
            Rejected::SenderTooLong(len) => len as u32,
            Rejected::WrongSender => 0,
        }
    }
}

/// The part of a byte string a C string comparison sees: everything before the first NUL.
fn c_str(bytes: &[u8]) -> &[u8] {
    bytes.iter().position(|b| *b == 0).map_or(bytes, |end| &bytes[..end])
}

/// The sender that last killed each software kill switch (`kill_switch_state::locking_frame`).
pub struct SoftwareKillLocks {
    frames: [heapless::Vec<u8, FRAME_STR_SIZE>; NUM_KILL_SWITCHES as usize],
}

impl SoftwareKillLocks {
    pub const fn new() -> Self {
        SoftwareKillLocks {
            frames: [heapless::Vec::new(), heapless::Vec::new(), heapless::Vec::new(), heapless::Vec::new()],
        }
    }

    /// Validates a software kill command. `switch_enabled` and `switch_asserting_kill` are the switch's current
    /// state as recorded by the safety core.
    ///
    /// On success the caller applies the command with `kill_switch_update(id, asserting_kill, needs_update)`. A
    /// command that asserts kill takes the lock for its sender.
    pub fn check(
        &mut self,
        cmd: &SoftwareKill,
        switch_enabled: bool,
        switch_asserting_kill: bool,
    ) -> Result<(), Rejected> {
        if cmd.kill_switch_id >= NUM_KILL_SWITCHES || cmd.kill_switch_id == KILL_SWITCH_PHYSICAL {
            return Err(Rejected::InvalidSwitch(cmd.kill_switch_id));
        }
        if cmd.sender_id.len() >= FRAME_STR_SIZE {
            return Err(Rejected::SenderTooLong(cmd.sender_id.len()));
        }

        let frame = &mut self.frames[cmd.kill_switch_id as usize];
        if switch_enabled && switch_asserting_kill && !cmd.asserting_kill && c_str(frame) != c_str(cmd.sender_id) {
            return Err(Rejected::WrongSender);
        }

        // Any sender may take over a killed switch by killing it again; see the C callback for why
        if cmd.asserting_kill {
            frame.clear();
            // Fits: checked against FRAME_STR_SIZE above
            let _ = frame.extend_from_slice(cmd.sender_id);
        }
        Ok(())
    }

    /// The sender currently holding the lock on `kill_switch_id`, for logging.
    pub fn locking_sender(&self, kill_switch_id: u8) -> &[u8] {
        self.frames.get(kill_switch_id as usize).map_or(&[], |f| c_str(f))
    }
}

impl Default for SoftwareKillLocks {
    fn default() -> Self {
        Self::new()
    }
}

/// Decides when to publish `state/physkill_notify`: only when the physical switch changes. The actuator and camera
/// cage boards flash their LEDs on it.
#[derive(Clone, Copy, Debug)]
pub struct PhysKillNotifier {
    last_asserting_kill: bool,
}

impl PhysKillNotifier {
    /// Assumes the switch is out, so an inserted switch is announced on the first update.
    pub const fn new() -> Self {
        PhysKillNotifier { last_asserting_kill: true }
    }

    /// Forgets the last state. Called when ROS (re)connects, like `ros_init`.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Returns the value to publish, if the switch changed.
    pub fn update(&mut self, asserting_kill: bool) -> Option<bool> {
        (asserting_kill != self.last_asserting_kill).then(|| {
            self.last_asserting_kill = asserting_kill;
            asserting_kill
        })
    }
}

impl Default for PhysKillNotifier {
    fn default() -> Self {
        Self::new()
    }
}

/// Tracks kill/enable callbacks to tell when the kill state actually flipped, so `state/kill` can be published
/// straight away. The safety core calls the kill callback again for every kill switch update that asserts kill.
#[derive(Clone, Copy, Debug)]
pub struct KillEdge {
    killed: bool,
}

impl KillEdge {
    /// Starts out enabled, so the first kill callback (normally at boot) counts as a change, as in C.
    pub const fn new() -> Self {
        KillEdge { killed: false }
    }

    /// Records a kill callback. Returns true if the vehicle was enabled before.
    pub fn on_kill(&mut self) -> bool {
        !core::mem::replace(&mut self.killed, true)
    }

    /// Records an enable callback. Returns true if the vehicle was killed before.
    pub fn on_enable(&mut self) -> bool {
        core::mem::replace(&mut self.killed, false)
    }
}

impl Default for KillEdge {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cmd(id: u8, asserting_kill: bool, sender: &str) -> SoftwareKill<'_> {
        SoftwareKill { kill_switch_id: id, asserting_kill, needs_update: false, sender_id: sender.as_bytes() }
    }

    #[test]
    fn rejects_physical_and_out_of_range_switches() {
        let mut locks = SoftwareKillLocks::new();
        assert_eq!(locks.check(&cmd(0, true, "rqt"), false, false), Err(Rejected::InvalidSwitch(0)));
        assert_eq!(locks.check(&cmd(4, true, "rqt"), false, false), Err(Rejected::InvalidSwitch(4)));
        assert_eq!(Rejected::InvalidSwitch(4).fault_extra(), 4);
        for id in 1..NUM_KILL_SWITCHES {
            assert_eq!(locks.check(&cmd(id, true, "rqt"), false, false), Ok(()));
        }
    }

    #[test]
    fn rejects_long_sender_ids() {
        let mut locks = SoftwareKillLocks::new();
        let long = "x".repeat(FRAME_STR_SIZE);
        assert_eq!(locks.check(&cmd(1, true, &long), false, false), Err(Rejected::SenderTooLong(32)));
        assert_eq!(Rejected::SenderTooLong(32).fault_extra(), 32);
        let longest = "x".repeat(FRAME_STR_SIZE - 1);
        assert_eq!(locks.check(&cmd(1, true, &longest), false, false), Ok(()));
        assert_eq!(locks.locking_sender(1), longest.as_bytes());
    }

    #[test]
    fn only_the_killing_sender_may_release() {
        let mut locks = SoftwareKillLocks::new();
        assert_eq!(locks.check(&cmd(1, true, "rqt"), false, false), Ok(()));
        assert_eq!(locks.locking_sender(1), b"rqt");
        // The switch is now enabled and asserting kill
        assert_eq!(locks.check(&cmd(1, false, "topside"), true, true), Err(Rejected::WrongSender));
        assert_eq!(locks.check(&cmd(1, false, "rqt"), true, true), Ok(()));
        // Other switches have their own locks
        assert_eq!(locks.check(&cmd(2, false, "topside"), true, false), Ok(()));
    }

    #[test]
    fn anyone_may_take_over_by_killing() {
        let mut locks = SoftwareKillLocks::new();
        locks.check(&cmd(3, true, "rqt"), false, false).unwrap();
        assert_eq!(locks.check(&cmd(3, true, "topside"), true, true), Ok(()));
        assert_eq!(locks.locking_sender(3), b"topside");
        assert_eq!(locks.check(&cmd(3, false, "rqt"), true, true), Err(Rejected::WrongSender));
        assert_eq!(locks.check(&cmd(3, false, "topside"), true, true), Ok(()));
    }

    #[test]
    fn release_allowed_when_switch_not_asserting() {
        let mut locks = SoftwareKillLocks::new();
        locks.check(&cmd(1, true, "rqt"), false, false).unwrap();
        // Not enabled (e.g. never applied) or no longer asserting: no lock to honour
        assert_eq!(locks.check(&cmd(1, false, "other"), false, true), Ok(()));
        assert_eq!(locks.check(&cmd(1, false, "other"), true, false), Ok(()));
        // A release doesn't change the lock owner
        assert_eq!(locks.locking_sender(1), b"rqt");
    }

    #[test]
    fn sender_ids_compare_as_c_strings() {
        let mut locks = SoftwareKillLocks::new();
        // A never locked switch has an empty lock, which an empty sender matches
        assert_eq!(locks.check(&cmd(2, false, ""), true, true), Ok(()));
        assert_eq!(locks.check(&cmd(2, false, "rqt"), true, true), Err(Rejected::WrongSender));
        locks.check(&cmd(2, true, "rqt"), true, true).unwrap();
        assert_eq!(locks.check(&cmd(2, false, "rqt\0junk"), true, true), Ok(()));
    }

    #[test]
    fn physkill_notifications_only_on_change() {
        let mut n = PhysKillNotifier::new();
        assert_eq!(n.update(true), None);
        assert_eq!(n.update(false), Some(false));
        assert_eq!(n.update(false), None);
        assert_eq!(n.update(true), Some(true));
        n.update(false);
        n.reset();
        // After a reconnect an inserted switch is announced again
        assert_eq!(n.update(false), Some(false));
    }

    #[test]
    fn kill_edges() {
        let mut edge = KillEdge::new();
        assert!(edge.on_kill());
        assert!(!edge.on_kill());
        assert!(!edge.on_kill());
        assert!(edge.on_enable());
        assert!(!edge.on_enable());
        assert!(edge.on_kill());
    }
}
