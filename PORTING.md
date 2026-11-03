# Porting a board

How a board is put together here, for porting a new C board or adding a new one. Read `README.md` first for the
architecture.

## The four places a board lives

| What | Where | C counterpart |
|---|---|---|
| Firmware crate | `firmware/<crate>/` (copy `firmware/template`, add it to `firmware/Cargo.toml`) | `talos/<Board>/src` |
| Firmware ⇄ bridge messages | `crates/titan_link/src/<board>.rs`, plus a variant appended to `HostToDevice`/`DeviceToHost` in `lib.rs` (bump `PROTOCOL_VERSION`) | the topics and messages in the board's `ros.c` |
| ROS topics | `host/titan_bridge/src/boards/<board>.rs` with a `PROFILE` listed in `BOARDS` (`boards/mod.rs`) | the publishers/subscribers in `ros.c` |
| Pure logic worth testing on the host | an optional new crate `crates/<name>/` (`no_std`, picked up by the `crates/*` glob) | — |

`firmware/ivc_board`, `crates/titan_link/src/ivc.rs`, `host/titan_bridge/src/boards/ivc.rs` and `crates/ivc_core` are the
worked example.

## Firmware

* Start `main` with `titan_rp2040::board::start(spawner, titan_rp2040::can_block!(p, Irqs), BoardConfig { .. })`. It
  sets up safety, the status LED, CAN and the link. `can_block!` takes WATCHDOG, PWM slices 6 and 7, GPIO 13-15 and
  17-22, SPI0 and DMA channels 1 and 2; everything else in `p` stays yours. Bind DMA_IRQ_0 handlers for DMA_CH1 and
  DMA_CH2 (plus any channels you use) in your `bind_interrupts!`.
* Board specific faults: add names after `safety::STANDARD_FAULT_NAMES` (IDs from `safety::FIRST_BOARD_FAULT`) in a
  `static` slice passed as `BoardConfig::fault_names`, and keep the same IDs as the C `safety_interface.h`.
* Actuators must stop in `BoardConfig::on_kill` (runs in a critical section, keep it short), like
  `safety_handle_kill`. Restore in `on_enable`.
* `BoardConfig::on_board_message` receives your `titan_link` messages; queue work for a board task (embassy `Channel`
  or `Signal`) rather than doing it inline, like the C executor callbacks.
* Send to the bridge with `titan_rp2040::link::send(&DeviceToHost::<Board>(..))`. Only send while
  `link::is_connected()` when the C code only published while ROS was up.
* CANmore utility frames (e.g. thruster commands on channel 0) arrive on `canbus::UTIL_RX` and go out on
  `canbus::UTIL_TX`.
* Periodic work: `embassy_time::Ticker`. Interrupt handlers and repeating timers become async tasks.
* Shared drivers live in `titan_rp2040::drivers` (sht41, mcp3426, depth, cd74hc4051, status_strip). Board-only
  drivers go in the board crate. The C `driver_async_i2c` queue is replaced by embassy-rp's async `I2c`; share a bus
  between drivers with `embassy_embedded_hal::shared_bus`.
* Keep behaviour, timing, thresholds and pin usage the same as C. When you deliberately differ (a C bug, something
  that can't work the same way), say why in a comment.

## Messages and topics

* Mirror every topic of the C `ros.c`: same name (relative to the robot namespace), message type, direction and QoS
  (`rclc_*_init_best_effort` → `.best_effort()`). Use the dynamic message helpers in `host/titan_bridge/src/ros.rs`
  (scalar accessors, `set_header`, `set_stamp`, `set_time`) and add missing ones there rather than in the board.
* If the C board published its heartbeat on `state/fw_heartbeat`, or publishes `state/kill` itself (it owns the
  physical kill switch), add it to `FW_HEARTBEAT_BOARDS` / `KILL_SOURCE_BOARDS` in `boards/mod.rs`.
* Keep `titan_link` messages small and typed. Use fixed-size types (`heapless`) instead of strings where you can.
  Messages up to `titan_link::MAX_ENCODED_LEN` (256 bytes) work; frequent ones should fit one CAN FD frame (64).
* Services (`rclc_service_*`) become a request message plus a response message, with the bridge creating the ROS
  service and matching the response by sequence number. rclrs 0.8 has no dynamic services, so the bridge uses the
  hand-written `std_srvs` bindings in `boards/actuator.rs` (see the Actuator board).

## Building

```bash
cd firmware && cargo clippy -p <crate>
cargo test -p titan_link -p <your core crate>        # from the repo root
riptide-shell                                       # then:
cargo clippy -p titan_bridge --all-targets && cargo test -p titan_bridge
```

Finish with zero clippy warnings and `cargo fmt` (120 columns, see `rustfmt.toml`).
