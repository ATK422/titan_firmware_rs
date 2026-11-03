//! Software kill switches from `command/software_kill` (`software_kill_subscription_callback` in `ros.c`).
//!
//! Kill switch IDs follow `riptide_msgs2/KillSwitchReport`. Switch 0 is the physical switch on the backplane and
//! can't be set from ROS. A node that kills through a switch takes that switch's lock: while the switch asserts
//! kill, only an update with the same `sender_id` can release it, so another node can't accidentally re-enable the
//! vehicle. Any node may kill, and killing takes the lock over, so a crashed node can't lock the vehicle out.

use heapless::Vec;
use titan_link::puddles_backplane::{SENDER_ID_LEN, SoftwareKill};

/// `KillSwitchReport.NUM_KILL_SWITCHES`.
pub const NUM_KILL_SWITCHES: u8 = 4;
/// `KillSwitchReport.KILL_SWITCH_PHYSICAL`: the switch on the backplane.
pub const KILL_SWITCH_PHYSICAL: u8 = 0;

/// Why a software kill update was rejected. Each raises `FAULT_ROS_BAD_COMMAND` with [`Rejected::fault_arg`].
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Rejected {
    /// The switch ID is out of range or the physical switch.
    InvalidSwitch(u8),
    /// The sender ID has [`SENDER_ID_LEN`] bytes or more.
    SenderTooLong(usize),
    /// The switch is locked by a different sender.
    WrongSender,
}

impl Rejected {
    /// The fault argument C used: the switch ID, the sender ID length, or 0.
    pub fn fault_arg(self) -> u32 {
        match self {
            Rejected::InvalidSwitch(id) => id as u32,
            Rejected::SenderTooLong(len) => len as u32,
            Rejected::WrongSender => 0,
        }
    }
}

/// The text of a C string: everything before the first NUL.
fn c_str(bytes: &[u8]) -> &[u8] {
    bytes.iter().position(|&b| b == 0).map_or(bytes, |end| &bytes[..end])
}

/// The sender locks of the software kill switches (`kill_switch_states[].locking_frame`).
#[derive(Clone, Debug, Default)]
pub struct SoftwareKillLocks {
    frames: [Vec<u8, SENDER_ID_LEN>; NUM_KILL_SWITCHES as usize],
}

impl SoftwareKillLocks {
    pub const fn new() -> Self {
        SoftwareKillLocks { frames: [const { Vec::new() }; NUM_KILL_SWITCHES as usize] }
    }

    /// Validates an update and takes the lock if it kills. `switch_killing` says whether the switch is currently
    /// enabled and asserting kill. On `Ok` the board applies the update to the kill switch.
    pub fn check(&mut self, msg: &SoftwareKill, switch_killing: bool) -> Result<(), Rejected> {
        let id = msg.kill_switch_id;
        if id >= NUM_KILL_SWITCHES || id == KILL_SWITCH_PHYSICAL {
            return Err(Rejected::InvalidSwitch(id));
        }
        if msg.sender_id.len() >= SENDER_ID_LEN {
            return Err(Rejected::SenderTooLong(msg.sender_id.len()));
        }
        let sender = c_str(&msg.sender_id);
        let lock = &mut self.frames[id as usize];
        if switch_killing && !msg.asserting_kill && lock.as_slice() != sender {
            return Err(Rejected::WrongSender);
        }
        if msg.asserting_kill {
            // Fits: shorter than SENDER_ID_LEN was checked above
            *lock = Vec::from_slice(sender).unwrap_or_default();
        }
        Ok(())
    }
}

/// Builds the message for a `KillSwitchReport`, cutting the sender ID to [`SENDER_ID_LEN`] bytes. A cut ID is
/// still too long for [`SoftwareKillLocks::check`], so it's rejected as in C.
pub fn software_kill(sender_id: &str, kill_switch_id: u8, asserting_kill: bool, needs_update: bool) -> SoftwareKill {
    let bytes = sender_id.as_bytes();
    let sender_id = Vec::from_slice(&bytes[..bytes.len().min(SENDER_ID_LEN)]).unwrap_or_default();
    SoftwareKill { sender_id, kill_switch_id, asserting_kill, needs_update }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_bad_switches_and_senders() {
        let mut locks = SoftwareKillLocks::new();
        for id in [KILL_SWITCH_PHYSICAL, NUM_KILL_SWITCHES, 255] {
            assert_eq!(locks.check(&software_kill("rqt", id, true, false), false), Err(Rejected::InvalidSwitch(id)));
        }
        let longest = "x".repeat(SENDER_ID_LEN - 1);
        assert_eq!(locks.check(&software_kill(&longest, 1, true, false), false), Ok(()));
        let too_long = "x".repeat(SENDER_ID_LEN);
        assert_eq!(
            locks.check(&software_kill(&too_long, 1, true, false), false),
            Err(Rejected::SenderTooLong(SENDER_ID_LEN))
        );
        let way_too_long = "y".repeat(100);
        let msg = software_kill(&way_too_long, 1, true, false);
        assert_eq!(msg.sender_id.len(), SENDER_ID_LEN);
        assert_eq!(locks.check(&msg, false), Err(Rejected::SenderTooLong(SENDER_ID_LEN)));
        assert_eq!(Rejected::InvalidSwitch(9).fault_arg(), 9);
        assert_eq!(Rejected::SenderTooLong(40).fault_arg(), 40);
        assert_eq!(Rejected::WrongSender.fault_arg(), 0);
    }

    #[test]
    fn only_the_killer_can_unkill() {
        let mut locks = SoftwareKillLocks::new();
        assert_eq!(locks.check(&software_kill("controller", 1, true, true), false), Ok(()));
        // The switch now asserts kill: someone else can't release it...
        assert_eq!(locks.check(&software_kill("other", 1, false, true), true), Err(Rejected::WrongSender));
        // ...but can kill again, taking the lock over
        assert_eq!(locks.check(&software_kill("other", 1, true, true), true), Ok(()));
        assert_eq!(locks.check(&software_kill("controller", 1, false, true), true), Err(Rejected::WrongSender));
        assert_eq!(locks.check(&software_kill("other", 1, false, true), true), Ok(()));
        // Locks are per switch
        assert_eq!(locks.check(&software_kill("anyone", 2, false, false), false), Ok(()));
    }

    #[test]
    fn released_switches_accept_anyone() {
        let mut locks = SoftwareKillLocks::new();
        assert_eq!(locks.check(&software_kill("a", 3, true, false), false), Ok(()));
        // Not asserting kill (released, or timed out and never enabled): no lock to respect
        assert_eq!(locks.check(&software_kill("b", 3, false, false), false), Ok(()));
    }

    #[test]
    fn sender_compares_as_c_string() {
        let mut locks = SoftwareKillLocks::new();
        assert_eq!(locks.check(&software_kill("node\0junk", 1, true, false), false), Ok(()));
        assert_eq!(locks.check(&software_kill("node", 1, false, false), true), Ok(()));
    }
}
