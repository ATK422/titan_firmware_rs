//! Power stage shared by the boards that are the vehicle's kill switch source: the Talos power board (this crate's
//! binary), `poac_board` and `po_half_board`. Their C firmwares share `safety_interface.c`, the kill switch and
//! electrical parts of `ros.c` and the SHT41/MCP3426 setup line for line.
//!
//! * [`kill`]: the physical kill switch, the software kill output, `command/software_kill`, and publishing
//!   `state/kill` and `state/physkill_notify`
//! * [`sensors`]: the board I2C bus shared between the SHT41 and the MCP3426 (and the PoHalf depth sensor)
//! * [`fan`]: the fan switch and `command/electrical`
//! * [`Interval`] and [`wait_connected`]: the timer and ROS connection helpers the board publishing tasks use
//!
//! The hardware independent logic lives in `power_core`, where it is tested on the host.

#![no_std]

pub mod fan;
pub mod kill;
pub mod sensors;

use embassy_time::{Duration, Instant, Timer};
use titan_link::{DeviceToHost, power};
use titan_rp2040::{link, safety};

/// How often the board publishing tasks check their timers, standing in for the C main loop.
pub const ROS_TICK: Duration = Duration::from_millis(10);

/// Wraps power stage messages into the board's `DeviceToHost` variant.
pub type Wrap = fn(power::DeviceToHost) -> DeviceToHost;

/// Queues a power stage message for the bridge. Messages are dropped if the transport queue is full.
pub fn send(wrap: Wrap, msg: power::DeviceToHost) {
    if !link::send(&wrap(msg)) {
        defmt::debug!("Transport queue full, dropped power stage message");
    }
}

/// Waits until the bridge is connected.
pub async fn wait_connected() {
    while !link::is_connected() {
        Timer::after(ROS_TICK).await;
    }
}

/// A periodic timer polled from a loop, like `timer_ready` in the C `main.c`.
pub struct Interval {
    next: Instant,
    period: Duration,
    critical: bool,
}

impl Interval {
    /// First fires one period from now, like the timers set by `start_ros_timers`. A `critical` timer raises
    /// `FAULT_TIMER_MISSED` when it skips runs.
    pub fn starting_in(period: Duration, critical: bool) -> Self {
        Interval { next: Instant::now() + period, period, critical }
    }

    /// When the timer fires next.
    pub fn next(&self) -> Instant {
        self.next
    }

    /// Returns true if the timer fired, advancing it past `now`.
    pub fn fire(&mut self, now: Instant) -> bool {
        if now < self.next {
            return false;
        }
        let mut next = self.next + self.period;
        let mut missed = 0u32;
        while next <= now {
            next += self.period;
            missed += 1;
        }
        if missed > 0 {
            defmt::warn!("Missed {} runs of a {} ms timer", missed, self.period.as_millis());
            if self.critical {
                safety::raise_fault(safety::FAULT_TIMER_MISSED, missed);
            }
        }
        self.next = next;
        true
    }
}
