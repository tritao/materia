//! The pendant's 128 x 64 monochrome OLED as data: a character screen of 21 columns by 8 rows, and the framebuffer it renders to in
//! the SSD1309's own layout (eight pages of 128 columns, each byte eight vertical pixels, bit 0 at the top), so the firmware can
//! send the bytes to the display as they are and a simulation can show the same pixels.

use crate::font::{self, ADVANCE, GLYPH_WIDTH};

pub const COLS: usize = 21;
pub const ROWS: usize = 8;
pub const WIDTH: usize = 128;
pub const HEIGHT: usize = 64;
pub const PAGES: usize = HEIGHT / 8;
/// Where a row of 21 characters starts, so that its 126 pixels are centred in 128.
const LEFT: usize = (WIDTH - COLS * ADVANCE) / 2;
/// Characters in a double-size row: each is 12 pixels wide.
pub const DOUBLE_COLS: usize = WIDTH / (2 * ADVANCE);

/// How a row of text is drawn.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Style {
    Normal,
    /// Light on dark swapped: for an alert.
    Inverse,
    /// Twice the width and height; takes this row and the next, and holds up to ten characters.
    Double,
}

/// 128 x 64 pixels in SSD1309 page layout.
#[derive(Clone, PartialEq, Eq)]
pub struct Framebuffer {
    pub pages: [[u8; WIDTH]; PAGES],
}

impl Framebuffer {
    pub const BYTES: usize = WIDTH * PAGES;

    pub const fn new() -> Self { Framebuffer { pages: [[0; WIDTH]; PAGES] } }

    pub fn pixel(&self, x: usize, y: usize) -> bool {
        x < WIDTH && y < HEIGHT && self.pages[y / 8][x] & (1 << (y % 8)) != 0
    }

    pub fn set(&mut self, x: usize, y: usize, on: bool) {
        if x < WIDTH && y < HEIGHT {
            let bit = 1 << (y % 8);
            if on { self.pages[y / 8][x] |= bit } else { self.pages[y / 8][x] &= !bit }
        }
    }

    /// The bytes to stream to the display, page 0 first.
    pub fn write_to(&self, out: &mut [u8; Self::BYTES]) {
        for (page, columns) in self.pages.iter().enumerate() {
            out[page * WIDTH..(page + 1) * WIDTH].copy_from_slice(columns);
        }
    }

    /// FNV-1a over the bytes: a short fingerprint for tests that pin a whole frame.
    pub fn checksum(&self) -> u32 {
        let mut hash: u32 = 0x811C_9DC5;
        for columns in &self.pages {
            for &byte in columns {
                hash = (hash ^ byte as u32).wrapping_mul(0x0100_0193);
            }
        }
        hash
    }

    pub fn lit(&self) -> usize {
        self.pages.iter().flat_map(|p| p.iter()).map(|b| b.count_ones() as usize).sum()
    }
}

/// 21 x 8 characters and how each row is drawn.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Screen {
    pub rows: [[u8; COLS]; ROWS],
    pub styles: [Style; ROWS],
}

impl Screen {
    pub const fn new() -> Self { Screen { rows: [[b' '; COLS]; ROWS], styles: [Style::Normal; ROWS] } }

    /// Writes `text` at column `col` of `row` (clipped to the row; lower case and unknown characters are drawn as the font draws them).
    pub fn put(&mut self, row: usize, col: usize, text: &[u8]) {
        if row >= ROWS { return; }
        let width = if self.styles[row] == Style::Double { DOUBLE_COLS } else { COLS };
        for (i, &c) in text.iter().enumerate() {
            if col + i < width { self.rows[row][col + i] = c; }
        }
    }

    /// Centres `text` on `row`.
    pub fn centre(&mut self, row: usize, text: &[u8]) {
        let width = if self.styles[row] == Style::Double { DOUBLE_COLS } else { COLS };
        let shown = text.len().min(width);
        self.put(row, (width - shown) / 2, &text[..shown]);
    }

    pub fn style(&mut self, row: usize, style: Style) {
        if row < ROWS { self.styles[row] = style; }
    }

    /// The row's characters as text (trailing blanks kept), for tests and logs.
    pub fn row_text(&self, row: usize, out: &mut [u8; COLS]) { *out = self.rows[row]; }

    pub fn render(&self) -> Framebuffer {
        let mut frame = Framebuffer::new();
        let mut row = 0;
        while row < ROWS {
            match self.styles[row] {
                Style::Double if row + 1 < ROWS => {
                    // Twice the size: every glyph column is drawn twice, every pixel row twice, across two pages.
                    let left = (WIDTH - DOUBLE_COLS * 2 * ADVANCE) / 2;
                    for col in 0..DOUBLE_COLS {
                        let glyph = font::glyph(self.rows[row][col]);
                        for (i, &column) in glyph.iter().enumerate() {
                            let mut wide: u16 = 0;
                            for bit in 0..7 {
                                if column & (1 << bit) != 0 { wide |= 0b11 << (2 * bit); }
                            }
                            for repeat in 0..2 {
                                let x = left + col * 2 * ADVANCE + i * 2 + repeat;
                                frame.pages[row][x] = wide as u8;
                                frame.pages[row + 1][x] = (wide >> 8) as u8;
                            }
                        }
                    }
                    row += 2;
                    continue;
                }
                _ => {
                    for col in 0..COLS {
                        let glyph = font::glyph(self.rows[row][col]);
                        for (i, &column) in glyph.iter().enumerate().take(GLYPH_WIDTH) {
                            frame.pages[row][LEFT + col * ADVANCE + i] = column;
                        }
                    }
                    if self.styles[row] == Style::Inverse {
                        for x in 0..WIDTH { frame.pages[row][x] = !frame.pages[row][x]; }
                    }
                }
            }
            row += 1;
        }
        frame
    }
}

impl Default for Screen { fn default() -> Self { Screen::new() } }
impl Default for Framebuffer { fn default() -> Self { Framebuffer::new() } }

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_character_lands_where_the_layout_says() {
        let mut screen = Screen::new();
        screen.put(0, 0, b"A");
        let frame = screen.render();
        // The A's left column is 0x7E and starts at the left margin; the blank column after it stays dark.
        assert_eq!(frame.pages[0][LEFT], 0x7E);
        assert_eq!(frame.pages[0][LEFT + 5], 0);
        assert!(frame.pixel(LEFT, 1) && !frame.pixel(LEFT, 0) && !frame.pixel(LEFT, 7));
        // Row 1 is the second page.
        let mut second = Screen::new();
        second.put(1, 0, b"A");
        assert_eq!(second.render().pages[1][LEFT], 0x7E);
    }

    #[test]
    fn the_last_column_fits() {
        let mut screen = Screen::new();
        screen.put(7, COLS - 1, b"Z");
        let frame = screen.render();
        let last = LEFT + (COLS - 1) * ADVANCE;
        assert_eq!(frame.pages[7][last], 0x61);
        assert!(last + GLYPH_WIDTH <= WIDTH);
    }

    #[test]
    fn text_is_clipped_to_the_row() {
        let mut screen = Screen::new();
        screen.put(2, 18, b"ABCDEF");
        assert_eq!(&screen.rows[2][18..], b"ABC");
        screen.put(9, 0, b"X");
    }

    #[test]
    fn an_inverse_row_swaps_light_and_dark() {
        let mut screen = Screen::new();
        screen.put(3, 0, b"A");
        let normal = screen.render();
        screen.style(3, Style::Inverse);
        let inverse = screen.render();
        for x in 0..WIDTH {
            assert_eq!(inverse.pages[3][x], !normal.pages[3][x]);
        }
        assert_eq!(inverse.pages[2], normal.pages[2]);
    }

    #[test]
    fn a_double_row_is_twice_the_size_over_two_pages() {
        let mut screen = Screen::new();
        screen.style(0, Style::Double);
        screen.put(0, 0, b"I");
        let frame = screen.render();
        // The I's centre column 0x7F (seven rows) becomes 14 rows: all of page 0 and the low six bits of page 1, drawn twice wide.
        let left = (WIDTH - DOUBLE_COLS * 2 * ADVANCE) / 2;
        let x = left + 2 * 2;
        assert_eq!((frame.pages[0][x], frame.pages[1][x]), (0xFF, 0x3F));
        assert_eq!(frame.pages[0][x + 1], 0xFF);
        // The row below a double row is part of it, not drawn as its own text.
        assert_eq!(frame.pages[2], [0; WIDTH]);
    }

    #[test]
    fn the_checksum_pins_a_frame() {
        let mut screen = Screen::new();
        screen.put(0, 0, b"PENDANT");
        let a = screen.render();
        assert_eq!(a.checksum(), screen.render().checksum());
        screen.put(0, 0, b"PENDANS");
        assert_ne!(a.checksum(), screen.render().checksum());
        let mut bytes = [0u8; Framebuffer::BYTES];
        a.write_to(&mut bytes);
        assert_eq!(&bytes[LEFT..LEFT + 5], &font::glyph(b'P'));
    }
}
