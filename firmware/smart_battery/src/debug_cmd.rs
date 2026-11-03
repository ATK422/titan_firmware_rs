//! The `bq40_*` remote debug commands (`bq40_canmore_cmds.h`), ready for the CANmore debug interface.
//!
//! HOOK: `titan_rp2040` doesn't serve the CANmore register mapped debug interface yet (see the README), so nothing
//! can call [`remote_command`] and `canmore_cli` can't reach these commands. Once it does, register every entry
//! of [`smart_battery_core::remote_cmd::COMMANDS`] and dispatch to [`remote_command`] with the command line split
//! into words. Parsing, output and the gauge round trips are done and tested in `smart_battery_core::remote_cmd`.
#![allow(dead_code)] // Unreachable until the debug interface exists, see above

use core::fmt::Write;

use embassy_time::{Duration, with_timeout};

use smart_battery_core::remote_cmd::{self, DebugOp, DebugResult, Gauge, StatusRegister};
use smart_battery_core::screens::Snapshot;
use titan_rp2040::safety;

use crate::background;
use crate::fault;
use crate::gauge::{self, Request};

/// How long a command waits for core 1, like the C `dbg_cmd_xfer_and_wait(..., make_timeout_time_ms(200), ...)`.
/// Data flash writes take 150 ms of that.
const TIMEOUT: Duration = Duration::from_millis(200);

/// The gauge task, reached through the core 1 request queue.
struct Core1;

impl Gauge for Core1 {
    async fn execute(&mut self, op: DebugOp) -> DebugResult {
        gauge::DEBUG_REPLY.reset();
        gauge::REQUESTS.send(Request::Debug(op)).await;
        with_timeout(TIMEOUT, gauge::DEBUG_REPLY.wait()).await.unwrap_or(DebugResult::Timeout)
    }

    async fn read_status(&mut self, register: StatusRegister) -> Option<u32> {
        gauge::STATUS_REPLY.reset();
        gauge::REQUESTS.send(Request::Status(register)).await;
        with_timeout(TIMEOUT, gauge::STATUS_REPLY.wait()).await.ok().flatten()
    }
}

/// Runs one debug command and writes its response to `out`. Returns the command's return code.
pub async fn remote_command<W: Write>(argv: &[&str], out: &mut W) -> i32 {
    let shared = gauge::shared();
    let (not_connected_arg, safety_status_arg, pf_status_arg) = fault::gauge_fault_args();
    let snapshot = Snapshot {
        now: embassy_time::Instant::now().as_millis(),
        state: shared.state,
        info: shared.info,
        mfg: shared.mfg,
        mfg_readout_okay: background::mfg_readout().is_some(),
        faults: safety::faults(),
        not_connected_arg,
        safety_status_arg,
        pf_status_arg,
    };
    remote_cmd::run(argv, &mut Core1, &snapshot, out).await
}
