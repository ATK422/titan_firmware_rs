//! SSD1306 128x32 OLED over I2C, the transport half of the C `driver_ssd1306` (`ssd1306.c`). The frame buffer,
//! drawing and fonts live in `smart_battery_core::gfx`.
//!
//! Like C, every command is its own I2C write and errors are ignored: a missing display must not stop the board.
//! Each write is bounded by [`TIMEOUT`] (`SSD1306_TIMEOUT_MS`).

use embassy_time::{Duration, with_timeout};
use embedded_hal_async::i2c::I2c;

use smart_battery_core::gfx::{Framebuffer, PAGES, WIDTH};

/// 7-bit I2C address (`SSD1306_I2C_ADDR`).
pub const ADDRESS: u8 = 0x3C;
/// Bound on each I2C write.
const TIMEOUT: Duration = Duration::from_millis(20);

/// Control byte for a command.
const CONTROL_COMMAND: u8 = 0x00;
/// Control byte for display data.
const CONTROL_DATA: u8 = 0x40;

pub struct Ssd1306<I> {
    i2c: I,
}

impl<I: I2c> Ssd1306<I> {
    pub fn new(i2c: I) -> Self {
        Ssd1306 { i2c }
    }

    async fn write(&mut self, bytes: &[u8]) {
        match with_timeout(TIMEOUT, self.i2c.write(ADDRESS, bytes)).await {
            Ok(Ok(())) => {}
            Ok(Err(_)) => defmt::trace!("SSD1306 write failed"),
            Err(_) => defmt::trace!("SSD1306 write timed out"),
        }
    }

    async fn command(&mut self, command: u8) {
        self.write(&[CONTROL_COMMAND, command]).await;
    }

    /// Configures the panel, clears it and turns it on (`ssd1306_Init`, for a 128x32 panel without mirroring).
    pub async fn init(&mut self) {
        self.set_display_on(false).await;
        for command in [
            0x20, // Memory addressing mode:
            0x00, // horizontal
            0xB0, // Page start address for page addressing mode
            0xC8, // COM output scan direction: remapped
            0x00, // Low column address
            0x10, // High column address
            0x40, // Start line address
        ] {
            self.command(command).await;
        }
        self.set_contrast(0xFF).await;
        for command in [
            0xA1, // Segment re-map 0 to 127
            0xA6, // Normal colour
            0xA8, // Multiplex ratio:
            0x1F, // 32 lines
            0xA4, // Output follows RAM content
            0xD3, // Display offset:
            0x00, // none
            0xD5, // Display clock divide ratio / oscillator frequency:
            0xF0, // divide ratio
            0xD9, // Pre-charge period:
            0x22, //
            0xDA, // COM pins hardware configuration:
            0x02, // for 32 lines
            0xDB, // VCOMH:
            0x20, // 0.77 x Vcc
            0x8D, // DC-DC:
            0x14, // enabled
        ] {
            self.command(command).await;
        }
        self.set_display_on(true).await;
        self.update(&Framebuffer::new()).await;
    }

    /// Sends the frame buffer one page at a time (`ssd1306_UpdateScreen`).
    pub async fn update(&mut self, fb: &Framebuffer) {
        let mut data = [0u8; WIDTH + 1];
        data[0] = CONTROL_DATA;
        for page in 0..PAGES {
            self.command(0xB0 + page as u8).await;
            self.command(0x00).await;
            self.command(0x10).await;
            data[1..].copy_from_slice(fb.page(page));
            self.write(&data).await;
        }
    }

    /// Sets the contrast, 0 to 255 (`ssd1306_SetContrast`).
    pub async fn set_contrast(&mut self, value: u8) {
        self.command(0x81).await;
        self.command(value).await;
    }

    /// Turns the panel on or off (`ssd1306_SetDisplayOn`).
    pub async fn set_display_on(&mut self, on: bool) {
        self.command(if on { 0xAF } else { 0xAE }).await;
    }
}
