//! What the pendant shows, from what it knows: the axis switch and step, the wheel enable, the E-stop chain read-back, the lease and the
//! controller's status line, and the labels the profile gave the axes and the F keys. `layout` makes the 21 x 8 screen; it allocates
//! nothing and is the same code on the firmware and in a simulation.
//!
//! ```text
//! rows 0-1  the axis in double size ("OFF" with the switch off), or NO LINK
//! row 2     POS 3  STEP X10   WHL
//! row 3     an alert: E-STOP CHAIN OPEN, or LEASE LOST (inverse)
//! row 5     the controller's status line (why a jog is refused)
//! row 7     F1, F2, F3 labels, six characters each in seven columns, under the keys
//! ```

use crate::screen::{Screen, Style, COLS};
use operator_link::label;
use operator_link::operator_wire::LeaseState;

pub const AXES: usize = 11;
pub const KEYS: usize = 3;
pub const LABEL: usize = 12;
/// One F key's share of the bottom row.
pub const KEY_COLS: usize = COLS / KEYS;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct View {
    /// 0 is OFF, 1 to 11 an axis of the profile.
    pub position: u8,
    /// 0, 1, 2 for x1, x10, x100.
    pub step: u8,
    pub wheel_enable: bool,
    /// The E-stop chain as the pendant reads it back; never part of the chain.
    pub chain_closed: bool,
    pub lease: LeaseState,
    /// Whether the controller's output frames are arriving.
    pub link_up: bool,
    /// The controller's one-line status text, NUL padded.
    pub status: [u8; 21],
    pub axis_names: [[u8; LABEL]; AXES],
    pub key_names: [[u8; LABEL]; KEYS],
}

impl View {
    pub const fn new() -> Self {
        View {
            position: 0,
            step: 0,
            wheel_enable: false,
            chain_closed: true,
            lease: LeaseState::NoLease,
            link_up: false,
            status: [0; 21],
            axis_names: [[0; LABEL]; AXES],
            key_names: [[0; LABEL]; KEYS],
        }
    }
}

impl Default for View { fn default() -> Self { View::new() } }

/// A small fixed buffer to build a line of text without an allocator.
struct Line { bytes: [u8; COLS], len: usize }

impl Line {
    fn new() -> Self { Line { bytes: [b' '; COLS], len: 0 } }
    fn text(&mut self, text: &[u8]) -> &mut Self {
        for &c in text { if self.len < COLS { self.bytes[self.len] = c; self.len += 1; } }
        self
    }
    fn number(&mut self, mut n: u32) -> &mut Self {
        let mut digits = [0u8; 10];
        let mut count = 0;
        loop {
            digits[count] = b'0' + (n % 10) as u8;
            count += 1;
            n /= 10;
            if n == 0 { break; }
        }
        while count > 0 { count -= 1; self.text(&digits[count..count + 1]); }
        self
    }
    fn at(&mut self, column: usize) -> &mut Self { self.len = column.min(COLS); self }
}

pub fn step_text(step: u8) -> &'static [u8] {
    match step { 0 => b"X1", 1 => b"X10", _ => b"X100" }
}

pub fn layout(view: &View) -> Screen {
    let mut screen = Screen::new();
    if !view.link_up {
        // No controller output: say so, large; nothing else on the face can be trusted.
        screen.style(0, Style::Double);
        screen.centre(0, b"NO LINK");
        screen.centre(3, b"NO CONTROLLER");
        screen.put(4, 0, b"CHECK THE CABLE");
        return screen;
    }
    // The axis, large.
    screen.style(0, Style::Double);
    let axis = if view.position == 0 {
        &b"OFF"[..]
    } else {
        let name = label(&view.axis_names[view.position as usize - 1]);
        if name.is_empty() { &b"AXIS"[..] } else { name }
    };
    let shown = &axis[..axis.len().min(crate::screen::DOUBLE_COLS)];
    screen.centre(0, shown);
    if view.position != 0 && label(&view.axis_names[view.position as usize - 1]).is_empty() {
        // An unnamed axis shows its switch number after "AXIS".
        let mut line = Line::new();
        line.text(b"AXIS ").number(view.position as u32);
        screen.centre(0, &line.bytes[..line.len]);
    }
    // Position, step and wheel.
    let mut line = Line::new();
    line.text(b"POS ").number(view.position as u32);
    line.at(7).text(b"STEP ").text(step_text(view.step));
    line.at(18).text(if view.wheel_enable { b"WHL" } else { b"---" });
    screen.put(2, 0, &line.bytes);
    // An alert, in inverse.
    if !view.chain_closed {
        screen.style(3, Style::Inverse);
        screen.centre(3, b"E-STOP CHAIN OPEN");
    } else if view.lease == LeaseState::Lost {
        screen.style(3, Style::Inverse);
        screen.centre(3, b"LEASE LOST");
    }
    // The controller's status line.
    screen.put(5, 0, label(&view.status));
    // The F keys' labels under their keys.
    for key in 0..KEYS {
        let name = label(&view.key_names[key]);
        screen.put(7, key * KEY_COLS, &name[..name.len().min(KEY_COLS - 1)]);
    }
    screen
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::string::String;

    pub fn text(screen: &Screen, row: usize) -> String {
        screen.rows[row].iter().map(|&c| c as char).collect()
    }

    fn name<const N: usize>(s: &str) -> [u8; N] {
        let mut out = [0u8; N];
        out[..s.len()].copy_from_slice(s.as_bytes());
        out
    }

    fn ready() -> View {
        let mut view = View::new();
        view.link_up = true;
        view.axis_names[2] = name("Z");
        view.axis_names[4] = name("B hob");
        view.key_names = [name("DOOR"), name("SPINDLE"), name("LIGHT")];
        view
    }

    #[test]
    fn without_a_controller_it_says_so() {
        let screen = layout(&View::new());
        assert_eq!(screen.styles[0], Style::Double);
        assert!(text(&screen, 0).trim() == "NO LINK");
        assert_eq!(text(&screen, 3).trim(), "NO CONTROLLER");
        assert_eq!(text(&screen, 7).trim(), "");
    }

    #[test]
    fn the_axis_is_named_in_double_size() {
        let mut view = ready();
        view.position = 3;
        view.step = 1;
        view.wheel_enable = true;
        let screen = layout(&view);
        assert_eq!(screen.styles[0], Style::Double);
        assert_eq!(text(&screen, 0).trim(), "Z");
        assert_eq!(text(&screen, 2), "POS 3  STEP X10   WHL");
        // Each key has seven columns, six of them for its label (a longer label is cut), so neighbours never run together.
        assert_eq!(text(&screen, 7), "DOOR   SPINDL LIGHT  ");
    }

    #[test]
    fn an_off_switch_and_an_unnamed_axis() {
        let mut view = ready();
        assert_eq!(text(&layout(&view), 0).trim(), "OFF");
        view.position = 9;
        assert_eq!(text(&layout(&view), 0).trim(), "AXIS 9");
    }

    #[test]
    fn long_names_are_cut_to_the_double_row() {
        let mut view = ready();
        view.axis_names[0] = name("EXTRUDER_A12");
        view.position = 1;
        let screen = layout(&view);
        assert_eq!(text(&screen, 0).trim(), "EXTRUDER_A");
    }

    #[test]
    fn an_open_chain_is_an_alert_before_a_lost_lease() {
        let mut view = ready();
        view.position = 1;
        view.lease = LeaseState::Lost;
        assert_eq!(text(&layout(&view), 3).trim(), "LEASE LOST");
        view.chain_closed = false;
        let screen = layout(&view);
        assert_eq!(text(&screen, 3).trim(), "E-STOP CHAIN OPEN");
        assert_eq!(screen.styles[3], Style::Inverse);
    }

    #[test]
    fn the_status_line_is_the_controllers_text() {
        let mut view = ready();
        view.position = 5;
        view.status = name("Gearbox engaged");
        assert_eq!(text(&layout(&view), 5).trim_end(), "Gearbox engaged");
    }

    #[test]
    fn step_names() {
        assert_eq!(step_text(0), b"X1");
        assert_eq!(step_text(2), b"X100");
    }
}
