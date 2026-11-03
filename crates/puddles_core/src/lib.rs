//! Hardware independent core of the Puddles backplane (`puddles/Backplane`).
//!
//! * [`wire`]: how `titan_link` messages travel over UDP between the backplane and `titan_bridge`, and the bridge's
//!   `--interface` spec
//! * [`dshot`]: DShot300 framing, the bidirectional eRPM reply, and the thruster command/timeout state machine of
//!   `dshot.c`
//! * [`electrical`]: rail voltages from the MCP3426 readings (`adc_update` in `ros.c`)
//! * [`kill`]: validation and sender locks of `command/software_kill` (`software_kill_subscription_callback`)
//! * [`Interval`]: the main loop's periodic timers (`timer_ready` in `main.c`)
//!
//! Time is passed in as microseconds since boot so everything here runs on the host in tests.

#![no_std]

pub mod dshot;
pub mod electrical;
pub mod kill;
pub mod wire;

/// A periodic timer polled from a loop, like `timer_ready` in the C `main.c`.
#[derive(Clone, Copy, Debug)]
pub struct Interval {
    next_us: u64,
    period_us: u64,
}

impl Interval {
    /// A timer that first fires one period after `now_us`, like `start_ros_timers`.
    pub const fn starting_at(now_us: u64, period_us: u64) -> Self {
        Interval { next_us: now_us + period_us, period_us }
    }

    /// If the timer is due, advances it past `now_us` and returns how many whole periods were skipped (the C code
    /// logs those, and raises `FAULT_TIMER_MISSED` for critical timers). Returns `None` if it isn't due yet.
    pub fn fire(&mut self, now_us: u64) -> Option<u32> {
        if now_us < self.next_us {
            return None;
        }
        let mut next = self.next_us + self.period_us;
        let mut missed = 0;
        while next <= now_us {
            next += self.period_us;
            missed += 1;
        }
        self.next_us = next;
        Some(missed)
    }

    /// When the timer next fires.
    pub fn next_us(&self) -> u64 {
        self.next_us
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interval_fires_once_per_period() {
        let mut timer = Interval::starting_at(1_000, 100);
        assert_eq!(timer.fire(1_000), None);
        assert_eq!(timer.fire(1_099), None);
        assert_eq!(timer.fire(1_100), Some(0));
        assert_eq!(timer.fire(1_150), None);
        // Firing late but within the period keeps the original schedule
        assert_eq!(timer.fire(1_250), Some(0));
        assert_eq!(timer.next_us(), 1_300);
    }

    #[test]
    fn interval_reports_missed_runs() {
        let mut timer = Interval::starting_at(0, 100);
        // Due at 100; at 450 the runs at 200, 300 and 400 were missed
        assert_eq!(timer.fire(450), Some(3));
        assert_eq!(timer.next_us(), 500);
        assert_eq!(timer.fire(500), Some(0));
    }
}
