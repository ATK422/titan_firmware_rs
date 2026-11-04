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
| `talos/IVCBoard` | `ivc_board` (`--features calibration`) | `ivc` | `/talos/ivc_board` | 8 | internal FD |
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
| `crates/titan_control` | Control interface register maps (debug, remote commands, bootloader) and the crash log, served by `titan_rp2040::control` / `crash` | `titan_debug`, `titan_bootloader`, `crash_reporting.c` |
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

Firmware (needs `rustup target add thumbv6m-none-eabi`):

```bash
cd firmware
cargo build --workspace --release
```

Bridge, plus its ROS round-trip tests (inside the riptide ROS 2 Humble environment):

```bash
riptide-shell
cargo build -p titan_bridge --release
cargo test -p titan_bridge
```

## Flashing

With an SWD probe (logs stream over defmt/RTT):

Images are linked behind the C Titan bootloader, as the C applications are (boot2 at 0x10004000, vector table at
0x10004100), so the board needs the bootloader in its first 16 KiB. A board that came from the C firmware has it;
on a blank board, flash a C `*_with_bl.uf2` (or the bootloader alone) once with `upload_tool -f`. Then:

```bash
cd firmware
cargo run -p <board crate> --release
```

That uses `probe-rs run --chip RP2040` (see `firmware/.cargo/config.toml`). Without a probe, flash the ELF over CAN,
Ethernet or USB (BOOTSEL) with the Rust upload tool, which reads ELF files directly and treats them as OTA images:

```bash
cargo run -p upload_tool -- -b talos/ivc_board firmware/target/thumbv6m-none-eabi/release/ivc_board
```

`-b` names the board, because Rust firmware carries no pico-sdk binary info for auto-detection. Over CAN the board
is reached through its control interface, which reboots it into the bootloader. To run on a board without the
bootloader, build with `TITAN_BOOTLOADER=0`, which links at the start of flash as before.

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

Every board serves the CANmore control interface in Normal mode like `titan_debug` (`titan_rp2040::control`, on top
of the host-tested register map in `crates/titan_control`): on CAN utility channel 14 (classic frames, no multiword,
as C) and, on Puddles, UDP port 2202 (multiword up to 512 bytes). The heartbeat reports mode Normal, and
`canmore_cli` and `upload_tool` work as with C firmware: version, uptime, reboot, enter bootloader, faults (raise,
lower, names, records), the crash log, gdb memory access, and remote commands. The commands are `help`, `memstats`
(statics and stack; there is no heap), `candbg` on CAN boards, and the board commands of the C boards (Actuator V2,
PoAC, HBridge, LEDBoard2, IVCBoard2, IVCDAC). Board commands that need async drivers are deferred to a board task
(`control::next_deferred`) and time out after 1 s.

There is no Rust bootloader: the C `titan_bootloader` runs the Rust applications, which are linked for it. Entering
the bootloader writes its scratch 4 magic and resets, as `safety_enter_bootloader` does.

The crash log is the C one (`titan_rp2040::crash`, `titan_control::crash_log`): watchdog scratch 0-2 hold what the
firmware is doing (`UNKNOWN_SAFETY_PREINIT` / `ACTIVE`) or the crash being taken (`PANIC`, `HARD_FAULT` with the
faulting PC, `WATCHDOG_TIMEOUT` with the interrupted PC from an NMI 1 µs before the watchdog fires, `CORE1_TIMEOUT`),
and a 24-entry CRC-checked log in `.uninit` RAM keeps the resets with their faults and uptime (SysTick, 10 ms ticks).
The boot log line and `canmore_cli`'s crash log read the same as with C. Differences: a panic stores the address of
its file name and its line (Rust has no message address or caller to store); `CORE1_TIMEOUT` stores the
milliseconds since core 1's last check-in rather than its PC; core 1 isn't halted by a crash on core 0; the
profiler (`profiler_dump`) isn't ported; and the version string has no robot, builder or build date.

## Differences from the C firmware

Behaviour, timing, topics and pins follow the C firmware. The deliberate differences are documented where they
happen (module docs and comments); the ones you are most likely to notice:

**Everywhere**
* **ROS on the host.** Topics are served by `titan_bridge`, so message stamps come from the bridge's clock instead
  of `rmw_uros_epoch_nanos`, and Rust boards always use CAN (IVCBoard2/IVCDAC used the micro-ROS USB transport).
* **Logging** goes out over defmt/RTT instead of UART0 stdio.
* **Crash log** follows C (see [Control interface and bootloader](#control-interface-and-bootloader)), with the
  small differences listed there.
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
  up; `crates/ivc_core/src/timing.rs` explains and tests this.
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
* **Control interface and crash log:** `canmore_cli` and `upload_tool` over real CAN and Ethernet, the C
  bootloader starting a Rust image and taking it back over (`enter bootloader`, OTA upload), and the
  `WATCHDOG_TIMEOUT` NMI firing before the watchdog (the emulator doesn't model the NMI mask, so it reports such a
  hang as `UNKNOWN_SAFETY_*`, as it does for C)
