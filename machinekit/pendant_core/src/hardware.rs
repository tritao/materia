//! What the pendant board's parts say and want, as pure functions the firmware calls and the host tests: the selector chain's bits,
//! the E-stop read-back and the SSD1309 display's command bytes. Nothing here touches a peripheral.

use crate::core::DisplayPower;

/// Which positions the two selector switches read as, `None` for a switch between two positions (or with two closed).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Selectors {
    /// 0 to 11: the closed one of AXIS_0 to AXIS_11, where 0 is OFF.
    pub position: Option<u8>,
    /// 0, 1, 2: the closed one of STEP_1, STEP_10, STEP_100.
    pub step: Option<u8>,
}

/// Reads the 16 bits of the two chained 74HC165, first bit received in bit 15 (the board's SPEC, "Selector bits"): the first register
/// holds AXIS_0 to AXIS_7 in D0 to D7, the second AXIS_8 to AXIS_11 in D0 to D3, STEP_1, STEP_10, STEP_100 in D4 to D6 and a spare in D7.
/// The chain shifts D7 first, so the word is AXIS_7 .. AXIS_0, then the spare, STEP_100, STEP_10, STEP_1, then AXIS_11 .. AXIS_8.
/// A closed contact reads 0.
pub fn selectors(word: u16) -> Selectors {
    let closed = |bit: u32| word & (1 << bit) == 0;
    // Axis n, as the bit it arrives in.
    let axis_bit = |n: u32| if n < 8 { 8 + n } else { n - 8 };
    let step_bit = |n: u32| 4 + n;
    Selectors { position: only((0..12).filter(|&n| closed(axis_bit(n)))), step: only((0..3).filter(|&n| closed(step_bit(n)))) }
}

fn only(mut closed: impl Iterator<Item = u32>) -> Option<u8> {
    let first = closed.next()?;
    if closed.next().is_some() { None } else { Some(first as u8) }
}

/// The E-stop read-back is high while the loop is energised: the chain is closed.
pub fn chain_closed(sense_high: bool) -> bool { sense_high }

/// SSD1309, I2C: the 7-bit address with the module's SA0 low.
pub const OLED_ADDRESS: u8 = 0x3C;
/// Control byte for a command stream and for display data.
pub const OLED_COMMAND: u8 = 0x00;
pub const OLED_DATA: u8 = 0x40;

/// The power-up commands for a 128 x 64 panel, ending with the display on. The reset line has already been pulsed.
pub const OLED_INIT: [u8; 23] = [
    0xAE, // display off
    0xD5, 0x80, // clock divide and oscillator
    0xA8, 0x3F, // multiplex 64
    0xD3, 0x00, // no display offset
    0x40, // start line 0
    0x20, 0x02, // page addressing
    0xA1, // column 127 is segment 0: mirrored so the picture reads left to right as wired
    0xC8, // scan from COM63 down
    0xDA, 0x12, // alternative COM pins
    0x81, 0xCF, // contrast
    0xD9, 0xF1, // pre-charge
    0xDB, 0x40, // VCOMH
    0xA4, // follow RAM
    0xA6, // normal, not inverted
    0xAF, // display on
];

/// The commands that put the display in `power`: contrast, and on or off.
pub fn oled_power(power: DisplayPower) -> [u8; 4] {
    match power {
        DisplayPower::On => [0x81, 0xCF, 0xAF, 0xA4],
        DisplayPower::Dim => [0x81, 0x10, 0xAF, 0xA4],
        DisplayPower::Off => [0x81, 0x10, 0xAE, 0xA4],
    }
}

/// The three commands that point the write at the start of `page`.
pub fn oled_page(page: u8) -> [u8; 3] { [0xB0 | (page & 0x07), 0x00, 0x10] }

#[cfg(test)]
mod tests {
    use super::*;

    /// All contacts open: every line reads 1.
    const OPEN: u16 = 0xFFFF;

    fn with_closed(bits: &[u32]) -> u16 { bits.iter().fold(OPEN, |w, &b| w & !(1 << b)) }

    #[test]
    fn a_switch_between_positions_reads_nothing() {
        assert_eq!(selectors(OPEN), Selectors { position: None, step: None });
    }

    #[test]
    fn axis_lines_arrive_where_the_board_wires_them() {
        // AXIS_0 is the first register's D0, the eighth bit received (bit 8); AXIS_7 the very first (bit 15).
        assert_eq!(selectors(with_closed(&[8])).position, Some(0));
        assert_eq!(selectors(with_closed(&[15])).position, Some(7));
        // AXIS_8 is the second register's D0, the last bit received (bit 0); AXIS_11 is its D3 (bit 3).
        assert_eq!(selectors(with_closed(&[0])).position, Some(8));
        assert_eq!(selectors(with_closed(&[3])).position, Some(11));
        let mut seen = [false; 12];
        for bit in [15, 14, 13, 12, 11, 10, 9, 8, 0, 1, 2, 3] {
            seen[selectors(with_closed(&[bit])).position.unwrap() as usize] = true;
        }
        assert!(seen.iter().all(|&s| s), "every axis line is one of the twelve bits");
    }

    #[test]
    fn step_lines_and_the_spare() {
        assert_eq!(selectors(with_closed(&[4])).step, Some(0));
        assert_eq!(selectors(with_closed(&[5])).step, Some(1));
        assert_eq!(selectors(with_closed(&[6])).step, Some(2));
        // The spare (bit 7) is wired high on the board and means nothing.
        assert_eq!(selectors(with_closed(&[7])), Selectors { position: None, step: None });
    }

    #[test]
    fn two_closed_contacts_are_not_a_position() {
        assert_eq!(selectors(with_closed(&[8, 15])).position, None);
        assert_eq!(selectors(with_closed(&[4, 5])).step, None);
    }

    #[test]
    fn the_init_sequence_turns_the_display_off_first_and_on_last() {
        assert_eq!(OLED_INIT[0], 0xAE);
        assert_eq!(OLED_INIT[OLED_INIT.len() - 1], 0xAF);
    }

    #[test]
    fn power_states_set_contrast_and_on_off() {
        assert_eq!(oled_power(DisplayPower::Off)[2], 0xAE);
        assert_eq!(oled_power(DisplayPower::On)[2], 0xAF);
        assert!(oled_power(DisplayPower::Dim)[1] < oled_power(DisplayPower::On)[1]);
    }

    #[test]
    fn pages_address_the_eight_rows_of_the_panel() {
        assert_eq!(oled_page(0), [0xB0, 0x00, 0x10]);
        assert_eq!(oled_page(7), [0xB7, 0x00, 0x10]);
    }
}
