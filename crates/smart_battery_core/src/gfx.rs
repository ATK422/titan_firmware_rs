//! Frame buffer and drawing primitives of the C `driver_ssd1306` (`ssd1306.c`), without the I2C transport.
//!
//! The buffer uses the SSD1306 page layout: byte `x + (y / 8) * WIDTH` holds the 8 vertical pixels of column `x` in
//! page `y / 8`, LSB on top. The firmware sends [`Framebuffer::page`] to the display one page at a time.
//!
//! The drawing code follows the C library pixel for pixel, including its cursor based text output and its arc
//! approximation, so screens look the same as on the C firmware.

use core::fmt;

/// Display width in pixels (`SSD1306_WIDTH`).
pub const WIDTH: usize = 128;
/// Display height in pixels (`SSD1306_HEIGHT`; the smart battery has the 128x32 panel).
pub const HEIGHT: usize = 32;
/// Number of 8 pixel high pages.
pub const PAGES: usize = HEIGHT / 8;

/// Pixel colour (`SSD1306_COLOR`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Color {
    /// Pixel off.
    Black,
    /// Pixel on.
    White,
}

impl Color {
    /// The other colour, used for glyph backgrounds.
    pub fn inverse(self) -> Color {
        match self {
            Color::Black => Color::White,
            Color::White => Color::Black,
        }
    }
}

/// A fixed width bitmap font (`FontDef`). See [`crate::fonts`].
#[derive(Clone, Copy)]
pub struct Font {
    pub width: u8,
    pub height: u8,
    /// `height` rows per glyph for characters 32..=126, MSB is the leftmost pixel.
    pub data: &'static [u16],
}

/// A monochrome bitmap with byte padded rows, MSB first (the `*_bin` arrays).
#[derive(Clone, Copy)]
pub struct Bitmap {
    pub width: u8,
    pub height: u8,
    pub data: &'static [u8],
}

/// The screen contents plus the text cursor.
#[derive(Clone)]
pub struct Framebuffer {
    buf: [u8; WIDTH * PAGES],
    cursor_x: u16,
    cursor_y: u16,
}

impl Default for Framebuffer {
    fn default() -> Self {
        Self::new()
    }
}

impl Framebuffer {
    pub const fn new() -> Self {
        Framebuffer { buf: [0; WIDTH * PAGES], cursor_x: 0, cursor_y: 0 }
    }

    /// The raw buffer in SSD1306 page layout.
    pub fn as_bytes(&self) -> &[u8; WIDTH * PAGES] {
        &self.buf
    }

    /// One 128 byte page (8 pixel rows) of the buffer.
    pub fn page(&self, page: usize) -> &[u8] {
        &self.buf[page * WIDTH..(page + 1) * WIDTH]
    }

    /// Fills the whole screen (`ssd1306_Fill`).
    pub fn fill(&mut self, color: Color) {
        self.buf.fill(if color == Color::Black { 0x00 } else { 0xFF });
    }

    /// Sets one pixel, ignoring coordinates off the screen (`ssd1306_DrawPixel`).
    pub fn draw_pixel(&mut self, x: u8, y: u8, color: Color) {
        let (x, y) = (x as usize, y as usize);
        if x >= WIDTH || y >= HEIGHT {
            return;
        }
        let byte = &mut self.buf[x + (y / 8) * WIDTH];
        match color {
            Color::White => *byte |= 1 << (y % 8),
            Color::Black => *byte &= !(1 << (y % 8)),
        }
    }

    /// Reads one pixel (off screen reads as black).
    pub fn pixel(&self, x: u8, y: u8) -> Color {
        let (x, y) = (x as usize, y as usize);
        if x >= WIDTH || y >= HEIGHT || self.buf[x + (y / 8) * WIDTH] & (1 << (y % 8)) == 0 {
            Color::Black
        } else {
            Color::White
        }
    }

    /// Moves the text cursor (`ssd1306_SetCursor`).
    pub fn set_cursor(&mut self, x: u8, y: u8) {
        self.cursor_x = x as u16;
        self.cursor_y = y as u16;
    }

    /// Draws one character at the cursor and advances it. Returns false (drawing nothing) if the character is not
    /// printable ASCII or does not fit on the screen (`ssd1306_WriteChar`).
    pub fn write_char(&mut self, ch: char, font: &Font, color: Color) -> bool {
        if !(' '..='~').contains(&ch) {
            return false;
        }
        if WIDTH < (self.cursor_x + font.width as u16) as usize
            || HEIGHT < (self.cursor_y + font.height as u16) as usize
        {
            return false;
        }
        let glyph = (ch as usize - 32) * font.height as usize;
        for row in 0..font.height as u16 {
            let bits = font.data[glyph + row as usize];
            for col in 0..font.width as u16 {
                let set = (bits << col) & 0x8000 != 0;
                let x = (self.cursor_x + col) as u8;
                let y = (self.cursor_y + row) as u8;
                self.draw_pixel(x, y, if set { color } else { color.inverse() });
            }
        }
        self.cursor_x += font.width as u16;
        true
    }

    /// Draws a string at the cursor, stopping at the first character that can't be drawn (`ssd1306_WriteString`).
    /// Returns false if it stopped early.
    pub fn write_str(&mut self, s: &str, font: &Font, color: Color) -> bool {
        s.chars().all(|ch| self.write_char(ch, font, color))
    }

    /// Formats text straight into the buffer at the cursor, with the same clipping as [`Framebuffer::write_str`].
    pub fn write_fmt(&mut self, font: &Font, color: Color, args: fmt::Arguments) {
        struct Writer<'a> {
            fb: &'a mut Framebuffer,
            font: &'a Font,
            color: Color,
            stopped: bool,
        }
        impl fmt::Write for Writer<'_> {
            fn write_str(&mut self, s: &str) -> fmt::Result {
                if !self.stopped && !self.fb.write_str(s, self.font, self.color) {
                    self.stopped = true;
                }
                Ok(())
            }
        }
        let _ = fmt::write(&mut Writer { fb: self, font, color, stopped: false }, args);
    }

    /// Bresenham line, both ends included (`ssd1306_Line`).
    pub fn line(&mut self, x1: u8, y1: u8, x2: u8, y2: u8, color: Color) {
        let (mut x, mut y) = (x1 as i32, y1 as i32);
        let (x2, y2) = (x2 as i32, y2 as i32);
        let dx = (x2 - x).abs();
        let dy = (y2 - y).abs();
        let sx = if x < x2 { 1 } else { -1 };
        let sy = if y < y2 { 1 } else { -1 };
        let mut error = dx - dy;

        self.draw_pixel(x2 as u8, y2 as u8, color);
        while x != x2 || y != y2 {
            self.draw_pixel(x as u8, y as u8, color);
            let error2 = error * 2;
            if error2 > -dy {
                error -= dy;
                x += sx;
            }
            if error2 < dx {
                error += dx;
                y += sy;
            }
        }
    }

    /// Rectangle outline with corners `(x1, y1)` and `(x2, y2)` (`ssd1306_DrawRectangle`).
    pub fn draw_rectangle(&mut self, x1: u8, y1: u8, x2: u8, y2: u8, color: Color) {
        self.line(x1, y1, x2, y1, color);
        self.line(x2, y1, x2, y2, color);
        self.line(x2, y2, x1, y2, color);
        self.line(x1, y2, x1, y1, color);
    }

    /// Filled rectangle, corners included and clipped to the screen (`ssd1306_FillRectangle`).
    pub fn fill_rectangle(&mut self, x1: u8, y1: u8, x2: u8, y2: u8, color: Color) {
        let (x_start, x_end) = (x1.min(x2), x1.max(x2));
        let (y_start, y_end) = (y1.min(y2), y1.max(y2));
        for y in y_start..=y_end.min(HEIGHT as u8 - 1) {
            for x in x_start..=x_end.min(WIDTH as u8 - 1) {
                self.draw_pixel(x, y, color);
            }
        }
    }

    /// Draws the set pixels of a bitmap with its top left corner at `(x, y)` (`ssd1306_DrawBitmap`).
    pub fn draw_bitmap(&mut self, x: u8, y: u8, bitmap: &Bitmap, color: Color) {
        if x as usize >= WIDTH || y as usize >= HEIGHT {
            return;
        }
        let stride = bitmap.width.div_ceil(8) as usize;
        for j in 0..bitmap.height {
            for i in 0..bitmap.width {
                let byte = bitmap.data[j as usize * stride + i as usize / 8];
                if byte & (0x80 >> (i % 8)) != 0 {
                    self.draw_pixel(x.wrapping_add(i), y.wrapping_add(j), color);
                }
            }
        }
    }

    /// An arc from `start_angle` sweeping `sweep` degrees clockwise from 12 o'clock, plus the two radius lines
    /// (`ssd1306_DrawArcWithRadiusLine`). Like C, the circle is approximated by 36 segments.
    pub fn draw_arc_with_radius_line(&mut self, x: u8, y: u8, radius: u8, start_angle: u16, sweep: u16, color: Color) {
        const SEGMENTS: u32 = 36;
        let sweep = normalize_to_0_360(sweep) as u32;
        let mut count = normalize_to_0_360(start_angle) as u32 * SEGMENTS / 360;
        let approx_segments = sweep * SEGMENTS / 360;
        // C divides by zero here for sweeps under 10 degrees; such arcs draw no segments anyway
        let approx_degree = if approx_segments == 0 { 0.0 } else { sweep as f32 / approx_segments as f32 };

        let point = |deg: f32| {
            let rad = deg_to_rad(deg) as f64;
            let px = x.wrapping_add((libm::sin(rad) * radius as f64) as i8 as u8);
            let py = y.wrapping_sub((libm::cos(rad) * radius as f64) as i8 as u8);
            (px, py)
        };

        let first = point(count as f32 * approx_degree);
        let mut last = (0, 0);
        while count < approx_segments {
            let p1 = point(count as f32 * approx_degree);
            count += 1;
            let p2 = if count != approx_segments { point(count as f32 * approx_degree) } else { point(sweep as f32) };
            self.line(p1.0, p1.1, p2.0, p2.1, color);
            last = p2;
        }
        self.line(x, y, first.0, first.1, color);
        self.line(x, y, last.0, last.1, color);
    }
}

/// The C library's approximation of pi; using the real one would move some arc pixels.
#[allow(clippy::approx_constant)] // Deliberately the C value
const C_PI: f64 = 3.14;

/// `ssd1306_DegToRad`, with [`C_PI`] and the C rounding to `float`.
fn deg_to_rad(deg: f32) -> f32 {
    (deg as f64 * C_PI / 180.0) as f32
}

/// `ssd1306_NormalizeTo0_360`. The C version returns angles above 360 unchanged (it overwrites the modulo), so
/// this does too; callers never pass more than 360.
fn normalize_to_0_360(deg: u16) -> u16 {
    deg
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fonts::{FONT_6X8, FONT_11X18};

    fn count_white(fb: &Framebuffer) -> u32 {
        fb.as_bytes().iter().map(|b| b.count_ones()).sum()
    }

    #[test]
    fn pixels_use_page_layout() {
        let mut fb = Framebuffer::new();
        fb.draw_pixel(3, 10, Color::White);
        assert_eq!(fb.as_bytes()[3 + WIDTH], 1 << 2);
        assert_eq!(fb.pixel(3, 10), Color::White);
        fb.draw_pixel(3, 10, Color::Black);
        assert_eq!(count_white(&fb), 0);
        // Off screen writes are ignored
        fb.draw_pixel(200, 0, Color::White);
        fb.draw_pixel(0, 40, Color::White);
        assert_eq!(count_white(&fb), 0);
    }

    #[test]
    fn text_draws_glyphs_and_clips_at_the_edge() {
        let mut fb = Framebuffer::new();
        fb.set_cursor(0, 0);
        assert!(fb.write_str("!", &FONT_6X8, Color::White));
        // '!' in the 6x8 font is a 5 pixel bar plus a dot in column 2
        assert_eq!(count_white(&fb), 6);
        assert_eq!(fb.pixel(2, 0), Color::White);
        assert_eq!(fb.pixel(2, 5), Color::Black);
        assert_eq!(fb.pixel(2, 6), Color::White);

        // 11 px wide glyphs: 11 fit on 128 columns, the 12th is refused
        fb.set_cursor(0, 0);
        assert!(!fb.write_str("AAAAAAAAAAAA", &FONT_11X18, Color::White));
        assert_eq!(fb.cursor_x, 121);
        // Too tall for the remaining rows
        fb.set_cursor(0, 20);
        assert!(!fb.write_char('A', &FONT_11X18, Color::White));
    }

    #[test]
    fn inverted_text_clears_its_cell() {
        let mut fb = Framebuffer::new();
        fb.fill(Color::White);
        fb.set_cursor(0, 0);
        fb.write_str(" ", &FONT_6X8, Color::Black);
        // A black space paints its whole 6x8 cell with the background colour... which is white for black text
        assert_eq!(count_white(&fb), (WIDTH * HEIGHT) as u32);
        fb.set_cursor(0, 0);
        fb.write_str(" ", &FONT_6X8, Color::White);
        assert_eq!(count_white(&fb), (WIDTH * HEIGHT) as u32 - 48);
    }

    #[test]
    fn rectangles() {
        let mut fb = Framebuffer::new();
        fb.draw_rectangle(118, 0, 127, 11, Color::White);
        assert_eq!(count_white(&fb), 2 * 10 + 2 * 10);
        let mut fb = Framebuffer::new();
        // Clipped at the bottom like the SOC box (rows 11..=32 on a 32 row screen)
        fb.fill_rectangle(0, 11, 79, 32, Color::White);
        assert_eq!(count_white(&fb), 80 * 21);
    }

    #[test]
    fn lines_include_both_ends() {
        let mut fb = Framebuffer::new();
        fb.line(0, 0, 3, 3, Color::White);
        assert_eq!(count_white(&fb), 4);
        let mut fb = Framebuffer::new();
        fb.line(5, 1, 1, 1, Color::White);
        assert_eq!(count_white(&fb), 5);
    }

    #[test]
    fn bitmaps_are_msb_first() {
        static DATA: [u8; 2] = [0b1000_0001, 0b0100_0000];
        let bmp = Bitmap { width: 10, height: 1, data: &DATA };
        let mut fb = Framebuffer::new();
        fb.draw_bitmap(0, 0, &bmp, Color::White);
        assert_eq!(fb.pixel(0, 0), Color::White);
        assert_eq!(fb.pixel(7, 0), Color::White);
        assert_eq!(fb.pixel(9, 0), Color::White);
        assert_eq!(count_white(&fb), 3);
    }

    #[test]
    fn full_arc_is_a_closed_ring() {
        let mut fb = Framebuffer::new();
        fb.draw_arc_with_radius_line(64, 16, 4, 0, 360, Color::White);
        // Like C, offsets are truncated and pi is 3.14, so only the top reaches the full radius
        assert_eq!(fb.pixel(64, 12), Color::White);
        assert_eq!(fb.pixel(67, 16), Color::White);
        assert_eq!(fb.pixel(68, 16), Color::Black);
        assert_eq!(fb.pixel(64, 19), Color::White);
        assert_eq!(fb.pixel(61, 16), Color::White);
        // The centre is on the radius line
        assert_eq!(fb.pixel(64, 16), Color::White);
        // Short sweeps don't divide by zero
        fb.draw_arc_with_radius_line(64, 16, 4, 0, 5, Color::White);
    }
}
