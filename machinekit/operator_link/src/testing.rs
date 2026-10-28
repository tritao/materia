//! Fixtures shared by the unit tests: hand-written configs, a frame bench, and the vectors the
//! Haxe compiler produces (`vectors/profiles.txt`, checked byte for byte by the Haxe tests).

use crate::authority::{Config, JogRule, UNUSED};
use crate::operator_wire::PendantState;
use crate::{FLAG_CHAIN_CLOSED, FLAG_WHEEL_ENABLE};

// Drill-tap layout: x y z a spindle at positions 1 to 5; F2 is the spindle jog.
pub const DRILL: Config<'static> = Config {
    pendant_axes: [0, 1, 2, 3, 4, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED],
    rules: &[], has_spindle: true, spindle_key: Some(1), step_units: [1, 10, 100], timeout_us: 50_000,
};
// Hobber: x y z a b c at positions 1 to 6; B and C refused while the gearbox (bit 0) is engaged.
pub const RULES: [JogRule; 2] = [JogRule { axis: 4, condition: 0, reason: 7 }, JogRule { axis: 5, condition: 0, reason: 7 }];
pub const HOBBER: Config<'static> = Config {
    pendant_axes: [0, 1, 2, 3, 4, 5, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED],
    rules: &RULES, has_spindle: true, spindle_key: Some(1), step_units: [1, 10, 100], timeout_us: 50_000,
};

pub struct Bench { pub seq: u16, pub t: u64 }
impl Bench {
    pub fn new() -> Self { Bench { seq: 0, t: 0 } }
    pub fn frame(&mut self, position: u8, delta: i16, enable: bool, buttons: u8, step: u8, closed: bool) -> PendantState {
        self.seq = self.seq.wrapping_add(1);
        let flags = if enable { FLAG_WHEEL_ENABLE } else { 0 } | if closed { FLAG_CHAIN_CLOSED } else { 0 };
        PendantState { sequence: self.seq, wheel_delta: delta, position, step, buttons, flags }
    }
    pub fn nx(&mut self, ms: u64) -> u64 { self.t += ms * 1000; self.t }
}


pub const VECTORS: &str = include_str!("../vectors/profiles.txt");

pub fn unhex(s: &str) -> std::vec::Vec<u8> {
    (0..s.len() / 2).map(|i| u8::from_str_radix(&s[i * 2..i * 2 + 2], 16).unwrap()).collect()
}
pub fn list(s: &str) -> std::vec::Vec<std::vec::Vec<u8>> {
    if s == "-" { std::vec::Vec::new() } else { s.split(';').map(unhex).collect() }
}
pub struct Vector {
    pub digest: u32,
    pub profile: std::vec::Vec<u8>,
    pub reasons: std::vec::Vec<std::vec::Vec<u8>>,
    pub labels: std::vec::Vec<std::vec::Vec<u8>>,
}
pub fn vector(name: &str) -> Vector {
    let line = VECTORS.lines().find(|l| l.starts_with(name)).unwrap();
    let f: std::vec::Vec<&str> = line.split(' ').collect();
    Vector { digest: u32::from_str_radix(f[1], 16).unwrap(), profile: unhex(f[2]), reasons: list(f[3]), labels: list(f[4]) }
}
pub fn same(a: &Config, b: &Config) {
    assert_eq!(a.pendant_axes, b.pendant_axes);
    assert_eq!(a.rules, b.rules);
    assert_eq!((a.has_spindle, a.spindle_key, a.step_units, a.timeout_us), (b.has_spindle, b.spindle_key, b.step_units, b.timeout_us));
}

