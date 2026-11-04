# Titan Firmware RS

Rust port of [`titan_firmware`](../titan_firmware) for OSU UWRT's RP2040 boards. The firmware runs on
[embassy](https://embassy.dev). The ROS 2 side runs on the host and is written with
[rclrs](https://github.com/ros2-rust/ros2_rust). The host tools (`upload_tool`, `canmore_cli`) are ported too.

Every board in the C repository has a Rust counterpart; see [Boards](#boards). Nothing has run on hardware yet:
see [Before trusting it on a robot](#before-trusting-it-on-a-robot).

## Architecture

The C firmware runs micro-ROS on the microcontroller and talks DDS-XRCE over CANmore to a micro-ROS agent on the
robot computer. rclrs needs `std` and the full ROS 2 C stack, so it can't run on an RP2040. The Rust port splits the
work in two:

```
 RP2040 (embassy)                                    Robot computer (rclrs)
┌──────────────────────────────┐   CAN / CAN FD     ┌───────────────────────────────┐
│ <board> firmware             │  CANmore frames    │ titan_bridge --board <board>  │
│  ├ board tasks               │  subtype 1 =       │  node /<robot>/<node name>    │
│  └ titan_rp2040              │  titan_link        │  ├ heartbeat, state/firmware  │
│     ├ link  ◄────────────────┼────────────────────┼─►├ state/kill                 │
│     ├ canbus + mcp251xfd     │  (postcard msgs)   │  └ the board's own topics     │
│     ├ safety, led, drivers   │  or UDP (Puddles)  │                               │
└──────────────────────────────┘                    └───────────────────────────────┘
```

* The firmware exchanges small typed `titan_link` messages with the bridge, carried as CANmore messages on the
  existing buses (or as UDP datagrams on Puddles, which has Ethernet instead of CAN).
* `titan_bridge` owns the ROS node. It uses the same node name, namespace, topics, message types and QoS as the C
  board's micro-ROS node, so the rest of the riptide stack doesn't notice the change. It uses rclrs *dynamic*
  messages, so it builds against the stock `riptide-shell` (`std_msgs`, `riptide_msgs2`, `std_srvs`) without
  generated Rust message crates.
* The connection works as before. The bridge pings every 100 ms. The firmware counts ROS as connected while pings
  keep arriving, and then initializes safety (kill switches honoured, watchdog tightened to 250 ms) and publishes
  the heartbeat every 100 ms and the firmware status every 1 s. If the pings stop for 700 ms, ROS counts as
  disconnected and the vehicle is killed.

## Boards

| C board | Firmware crate | `titan_bridge --board` | ROS node | CANmore client | Bus |
|---|---|---|---|---|---|
| `talos/Actuator` | `actuator_board` (`--features v1` for the V1 hardware) | `actuator` | `/talos/actuator_mk2` | 5 | internal FD |
| `talos/CameraCageBB` | `camera_cage_bb` | `camera_cage` | `/talos/camera_cage_bb` | 4 | internal FD |
| `talos/ESCBoard` | `esc_board` (ID from GPIO 29) | `esc0`, `esc1` | `/talos/esc_board_0`, `_1` | 2, 3 | internal FD |
| `talos/IVCBoard` | `ivc_board` (`--features calibration`, `adaptive-threshold`) | `ivc` | `/talos/ivc_board` | 8 | internal FD |
| `talos/IVCBoard2` | `ivc_board2` | `ivc2` | `/talos/IVCBoard2` | 8 | internal FD |
| `talos/IVCDAC` | `ivc_dac` (`--features rx-side`) | `ivc_dac` | `/talos/ivc_dac` | 8 | internal FD |
| `talos/LEDBoard` | `led_board` | `led` | `/talos/led_board` | 6 | internal FD |
| `talos/LEDBoard2` | `led_board2` | `led2` | `/talos/led_board2` | 6 | internal FD |
| `talos/PoacBoard` | `poac_board` | `poac` | `/talos/poac_board` | 1 | internal FD |
| `talos/PowerBoard` | `power_board` | `power` | `/talos/power_board` | 1 | internal FD |
| `talos/SmartBattery` | `smart_battery` (ID = board serial) | `smart_battery --client-id <serial>` | `/talos/smart_battery_<serial>` | serial | external classic |
| `SBCBoard` | `sbc_board` | `sbc` | `/talos/sbc_board` | 7 | internal FD |
| `puddles/Backplane` | `puddles_backplane` | `puddles_backplane` | `/puddles/backplane_firmware` | — | Ethernet/UDP |
| `lilboat/HBridgeBoard` | `hbridge_board` | `hbridge` | `/liltank/hbridge_board` | 2 | internal FD |
| `lilboat/PoHalfBoard` | `po_half_board` | `po_half` | `/liltank/pohalf_board` | 1 | internal FD |
| `examples/micro_ros_template` | `template` | — | — | — | — |
| `examples/dynamixel_test` | `dynamixel_test` | — | — | — | — |

Each board's topics, services and QoS are listed at the top of its bridge module
(`host/titan_bridge/src/boards/<board>.rs`) and its firmware `main.rs`. Board specific faults keep their C IDs
unless a board's `main.rs` says otherwise (IDs 0-4 are fixed by the Rust safety core).

## Layout

| Path | What it is | C counterpart |
|---|---|---|
| `firmware/titan_rp2040` | Safety, status LED, MCP2517FD driver, CANmore client, link task, one-call board bring-up, shared drivers (SHT41, MCP3426, MS5837 depth, CD74HC4051, WS2812 status strip) | `titan_safety`, `driver_led`, `driver_canbus`, `micro_ros_pico`, the `driver_*` sensors, template `main.c`/`ros.c` |
| `firmware/<board>` | One crate per board (table above) | `talos/*`, `puddles/*`, `lilboat/*`, `SBCBoard` |
| `firmware/dynamixel` | Async Dynamixel servo driver | `driver_dynamixel` |
| `crates/titan_control` | Control interface register maps (debug, remote commands, bootloader), not yet wired into firmware | `titan_debug`, `titan_bootloader` |
| `crates/canmore` | CANmore IDs, CRC-18, message fragmentation, heartbeat, register-mapped protocol client and server (`no_std`) | `titan_canmore` |
| `crates/titan_link` | Firmware ⇄ bridge protocol, one module per board (`no_std`, serde + postcard) | the micro-ROS message set in each `ros.c` |
| `crates/titan_sensors` | Sensor math: CRCs, MS5837 compensation, conversions | inside the `driver_*` sensors |
| `crates/titan_build` | Build script helper: memory map, linker scripts, version info | `titan_version`, `global_defs.cmake` |
| `crates/*_core` | Hardware independent logic of each board, host-tested (`ivc_core`, `ivc2_core`, `esc_core`, `ledc_core`, `power_core`, `hbridge_core`, `puddles_core`, `smart_battery_core`, `actuator_core`) | the board sources |
| `crates/dynamixel_protocol` | Dynamixel 2.0 packets, CRC-16, byte stuffing, XL430 control table | `driver_dynamixel` |
| `host/titan_bridge` | rclrs ROS 2 node serving one Rust board over SocketCAN or UDP | micro-ROS agent + board `ros.c` |
| `tools/upload_tool`, `tools/canmore_cli`, `tools/titan_tools` | Flashing and debug tools, shared host library | `tools/upload_tool`, `tools/canmore_cli`, `tools/lib` |
| `tools/smart_battery_id` | Builds the UF2 that programs a smart battery's serial | `talos/SmartBattery/id_gen` |

`firmware/` is a separate cargo workspace that targets `thumbv6m-none-eabi`. The root workspace holds the
portable crates, the bridge and the tools. [PORTING.md](PORTING.md) describes how a board is put together.

## Building

Portable crates, tools and their tests (any machine; the tools build libusb from source, so a C compiler is needed):

```bash
cargo test
```

Firmware:

```bash
cd firmware
cargo build --workspace --release
```

The compiler is pinned in [`rust-toolchain.toml`](rust-toolchain.toml) (Rust 1.99.0, with the
`thumbv6m-none-eabi` target, rustfmt and clippy), so every machine builds the same images. rustup picks it up in
this repository; if your rustup doesn't install it on first use, run `rustup toolchain install` in the repository
root. A different compiler produces slightly different code and timing, which changes the firmware simulator's
bench fingerprints and can move timing-sensitive behaviour such as the IVC detector, so bump the pin deliberately.

Bridge, plus its ROS round-trip tests (inside the riptide ROS 2 Humble environment):

```bash
riptide-shell
cargo build -p titan_bridge --release
cargo test -p titan_bridge
```

## Emulator CI

[`.github/workflows/emulator.yml`](.github/workflows/emulator.yml) runs on every push and pull request. It builds
the firmware images with the pinned compiler and boots them in the RP2040 emulator
([`ATK422/firmware_simulator`](https://github.com/ATK422/firmware_simulator), at `master`), and then runs the
emulator's firmware tests against them:

* `smoke`: every board boots on its board, connects to a mock `titan_bridge` and reports its status. The job
  summary shows the table.
* `faults`: CAN errors, bus off, sensor failures, watchdog, panics, brownouts, and Puddles' cable.
* `robot`: Talos's boards share their buses.
* `ivc_link`: IVC boards exchange bytes across the pool.

This checkout takes the place of the emulator's `external/titan_firmware_rs` submodule. The emulator therefore
builds against this commit's `canmore` and `titan_link` crates and tests this commit's images. A test that skips
for want of an image counts as a failure here. The tests take about 1.5 minutes once everything is built.

* **Secret:** `FIRMWARE_SIMULATOR_TOKEN`, a token that can read `ATK422/firmware_simulator`. A fine-grained
  personal access token with "Contents: read" on that repository works. Without it the job fails at its first
  step, and pull requests from forks get no secrets, so they fail there too.
* **Firmware behaviour changes:** the emulator's tests encode what the firmware is expected to do. This covers
  timings, fault bits and log lines. It also covers the `firmware_finding_*` tests, which pin down known firmware
  bugs as they are today. An intentional behaviour change, including a fix for one of those findings, therefore
  fails here until the emulator's tests change to match. Make that change in firmware_simulator, then run the
  workflow by hand (*Run workflow*) with `emulator_ref` set to that branch. Merge the two together.

To run the same tests locally, point a firmware_simulator checkout at your images:

```bash
cd firmware && cargo build --workspace --release && cd ..
cd ../firmware_simulator
TITAN_RS_RELEASE_DIR=$OLDPWD/firmware/target/thumbv6m-none-eabi/release \
  cargo test --release -p sim_test --test smoke -- --nocapture
```

## Flashing

With an SWD probe (logs stream over defmt/RTT):

```bash
cd firmware
cargo run -p <board crate> --release
```

That uses `probe-rs run --chip RP2040` (see `firmware/.cargo/config.toml`). Without a probe, hold BOOTSEL and flash
over USB with the Rust upload tool, which reads ELF files directly:

```bash
cargo run -p upload_tool -- -b talos/ivc_board firmware/target/thumbv6m-none-eabi/release/ivc_board
```

`-b` names the board, because Rust firmware carries no pico-sdk binary info for auto-detection. `picotool load -u
-v -x -t elf <elf>` from `riptide-shell` works too. Flashing over CAN or Ethernet needs the bootloader; see
[Control interface and bootloader](#control-interface-and-bootloader).

## Running the bridge

Run one bridge per board, with the CAN interface already up (FD mode on the internal bus) as for the micro-ROS
boards. Don't run a micro-ROS agent for the same client at the same time.

```bash
titan_bridge --board ivc                                  # talos, can0, client 8
titan_bridge --board esc1                                 # second ESC board
titan_bridge --board smart_battery --interface can1 --client-id 5
titan_bridge --board puddles_backplane --interface 0.0.0.0:2203,192.168.1.42:2203
```

`--namespace` and `--client-id` override the board defaults; anything after `--ros-args` goes to rcl. Puddles uses
UDP port 2203 at both ends: one datagram is the bytes `TL` followed by one postcard-encoded `titan_link` message.

## Tools

* `upload_tool [-f] [-i] [-p] [-w] [-b BOARD] <file.uf2|elf> [device]` flashes over USB (BOOTSEL / PICOBOOT), CAN
  (every `can*`/`vcan*` interface) and Ethernet, with the C tool's flags. OpenOCD/picoprobe upload (`-o`) isn't
  ported; use probe-rs.
* `canmore_cli [device [command ...]]` is the CANmore debug CLI for applications, bootloaders and Linux machines
  (uptime, version, faults, crash log, memory, `@remote` commands, gdbserver, remote shell, file transfer, camera
  feed). Device selection is a numbered list rather than the C arrow-key menu.
* USB access needs the udev rules from the C repository (`rp2040.rules`).

## Control interface and bootloader

Not finished. `crates/titan_control` has the portable, host-tested half: the debug register map served in Normal
mode (version, uptime, reboot, enter bootloader, faults, crash log, memory access, remote commands) and the
bootloader register map with its flash layout and image check. It isn't wired into `titan_rp2040` or a bootloader
firmware yet, so Rust boards still report control interface mode `None` in their CANmore heartbeat, and
`canmore_cli` and CAN/Ethernet uploads only work with C firmware and the C bootloader.

## Differences from the C firmware

Behaviour, timing, topics and pins follow the C firmware. The deliberate differences are documented where they
happen (module docs and comments); the ones you are most likely to notice:

**Everywhere**
* **ROS on the host.** Topics are served by `titan_bridge`, so message stamps come from the bridge's clock instead
  of `rmw_uros_epoch_nanos`, and Rust boards always use CAN (IVCBoard2/IVCDAC used the micro-ROS USB transport).
* **Logging** goes out over defmt/RTT instead of UART0 stdio.
* **Crash log** is smaller: watchdog scratch registers 0-3 hold the panic and watchdog-timeout counts, the uptime and
  the faults active at the reset. A panic forces a watchdog reset, so the next boot raises `FAULT_WATCHDOG_RESET`.
* **Fixed C bugs** are listed in each board's docs. The ones that change behaviour on the robot:
  * ESC: utility-frame thruster commands are decoded as documented big-endian values (C used the high byte twice);
    the controller's feed-forward no longer overflows above ~7150 RPM.
  * LEDBoard2: the controllers are actually enabled (C's register decoding kept them off).
  * Puddles: a power-cycled rail comes back on after 1 s (C left it off for good).
  * Actuator V2: the claw open/closed window is ±margin (C almost always reported unknown).
  * Several drivers: corrupt data is no longer accepted (SHT41 bad CRC, MCP3426 negative codes, Dynamixel byte
    stuffing).

**Per board**
* **IVCBoard:** a Goertzel filter replaces the 500-point FFT (same magnitudes, tested against a reference DFT).
  Because it is far faster, receive and ACK timing are anchored to the capture time so Rust and C boards still line
  up. The sender listens for the ACK over a window that covers the round trip to 200 m instead of sampling it once,
  which limited ACKs to about 25 m (C: about 37 m); `crates/ivc_core/src/timing.rs` explains and tests this.
  `--features adaptive-threshold` raises the fixed detection thresholds above a tracked noise floor in loud water
  (`crates/ivc_core/src/noise.rs`); off by default until it has been tried against the real hydrophone's noise.
* **IVCBoard2 / IVCDAC:** the same trick for their FIR+Goertzel receiver; the flash recorder erases one sector per
  step so the watchdog survives. `ivc_dac/rx_state` needs `riptide_msgs2/IvcRx`, which the installed
  `riptide_msgs2` lacks; until it is added, the bridge drops that topic.
* **ESC board:** ROS thruster commands travel as `titan_link` messages so the board can answer with RPM feedback;
  utility-frame commands from other software work as before.
* **Power boards:** I2C runs at 1 MHz (C asked for an out-of-spec 2 MHz).
* **Puddles:** fault IDs moved up by one (5-9); the WIZnet chip is assumed to be a W5500.
* **Smart battery:** the node is named `smart_battery_<serial>` and run with `--client-id <serial>`; the power-cycle
  command is kept as broken as in C (it never sends the MFC enable sequence) until it can be checked on hardware.

## Before trusting it on a robot

Everything builds and lints clean, about 400 host tests pass, and the bridge's ROS side is exercised through real
type support and DDS. No board, CAN bus or USB device was available, so check these on hardware first:

* MCP2517FD bring-up on both buses, coexistence with C boards, and the CAN heartbeat/online LED
* **IVC:** symbol and ACK timing against a C IVC board; IVCBoard2 capture processing time and flash erase timing;
  its hydrophone on ADC1 / GPIO 27 (as in C's `fft.c`, not the header's GPIO 26)
* **ESC:** DShot reply timing, eRPM feedback from a real ESC, core 1 keeping up
* **Actuator:** Dynamixel PIO UART timing, torpedo PIO timing (V1)
* **H-bridge:** DRV8243 wake sequence and nFAULT reads through the mux, direction of the inverted H3, Hiwonder bus
* **LEDBoard2:** bench-check before powering the LED module dry (the controllers now really turn on)
* **Smart battery:** the ~2 ms power-latch release during `embassy_rp::init`, gauge communication, display
* **Puddles:** which WIZnet chip is fitted (a W5200 needs a native driver), DShot timing, Ethernet link
* **Camera cage:** status strip orientation; depth surface pressure surviving a watchdog reset (`.uninit` RAM)
