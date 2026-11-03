//! A periodic timer that notices missed runs, the C `timer_ready`, on plain millisecond timestamps.

/// Fires every `period` ms, starting one period after it's created, and reports runs it skipped.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Interval {
    next: u64,
    period: u64,
}

impl Interval {
    /// A timer whose first run is due at `now + period` (`make_timeout_time_ms` in `start_ros_timers`).
    pub const fn starting_at(now: u64, period: u64) -> Self {
        Interval { next: now + period, period }
    }

    /// When the timer is next due.
    pub const fn next(&self) -> u64 {
        self.next
    }

    /// If the timer is due at `now`, advances it past `now` and returns the number of whole periods that were
    /// skipped (usually 0). Returns `None` if it isn't due yet.
    pub fn fire(&mut self, now: u64) -> Option<u32> {
        if now < self.next {
            return None;
        }
        let mut next = self.next + self.period;
        let mut missed = 0;
        while next <= now {
            next += self.period;
            missed += 1;
        }
        self.next = next;
        Some(missed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fires_once_per_period() {
        let mut t = Interval::starting_at(1000, 100);
        assert_eq!(t.fire(1099), None);
        assert_eq!(t.fire(1100), Some(0));
        assert_eq!(t.fire(1150), None);
        assert_eq!(t.next(), 1200);
        assert_eq!(t.fire(1210), Some(0));
        assert_eq!(t.next(), 1300);
    }

    #[test]
    fn counts_skipped_runs() {
        let mut t = Interval::starting_at(0, 30);
        // Due at 30; at 100 the runs at 60 and 90 were skipped
        assert_eq!(t.fire(100), Some(2));
        assert_eq!(t.next(), 120);
    }
}
