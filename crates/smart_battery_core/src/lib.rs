//! Hardware independent logic of the Talos smart battery housing MCU (`talos/SmartBattery`), tested on the host.
//!
//! * [`sbs`]: BQ40Z80 Smart Battery System registers and their decoding (`bq40z80_reg_map.h`, `bq40z80.c`)
//! * [`smbus`]: SMBus packet error checking and the gauge error codes (`pio_smbus.c`, `bq40z80.h`)
//! * [`monitor`]: connection, pack state and emergency FET shutdown tracking (`core1.c`)
//! * [`gfx`], [`fonts`], [`bitmaps`]: the SSD1306 frame buffer, drawing and fonts (`driver_ssd1306`, `uwrt_logo.h`)
//! * [`display`], [`screens`]: the OLED menu state machine and screen layouts (`display.c`)
//! * [`remote_cmd`]: the `bq40_*` debug commands (`bq40_canmore_cmds.h`)
//! * [`faults`]: the board fault IDs (`safety_interface.h`)
//! * [`uf2`]: the board ID image (`id_gen/id.uf2`)

#![cfg_attr(not(test), no_std)]

pub mod bitmaps;
pub mod display;
pub mod faults;
pub mod fonts;
pub mod gfx;
pub mod monitor;
pub mod remote_cmd;
pub mod sbs;
pub mod screens;
pub mod smbus;
pub mod uf2;
