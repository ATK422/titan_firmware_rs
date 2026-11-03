//! Host-side library shared by the titan firmware tools, ported from `titan_firmware/tools/lib`.
//!
//! * [`flash`]: the flash interface every upload path implements (`rp2040_flash_itf`)
//! * [`image`]: UF2 and ELF firmware images
//! * [`binary_info`]: pico-sdk binary info extraction (`binary_info_cpp`)
//! * [`device_map`]: the board serial number database (`DeviceList.jsonc`)
//! * [`canmore`]: CANmore transports (SocketCAN, UDP), heartbeat discovery and the debug, bootloader and Linux
//!   control interface clients (`canmore_dbg_cpp`, `canmore_cpp`)
//! * [`usb`]: RP2040 USB bootrom (PICOBOOT) access (`pico_usb`)

pub mod binary_info;
pub mod canmore;
pub mod crc32;
pub mod device_map;
pub mod flash;
pub mod image;
pub mod ui;
pub mod usb;
