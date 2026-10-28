//! Pendant jog authority for the cabinet controller: the Rust port of `PendantAuthority`
//! (machinekit.operator), with the same rules and the same scenarios in the tests. Not a safety
//! function; the E-stop chain is hard wired and `chain_closed` is only an observation.
//!
//! - Frames at or behind the last accepted sequence are dropped (16-bit wrap aware).
//! - No frame within `timeout_us` ends the lease; it returns only after a frame arrives with the
//!   wheel and the spindle key released.
//! - Wheel counts count only while the enable was held in this frame and the previous one, and
//!   only in Jog mode. A lost frame loses its counts: travel is short, never long.
//!
//! Axes are profile indices and conditions are bits of a mask, so no strings reach the controller.

use crate::pendant_wire::{OperatorCondition, OperatorState, PendantEvent, PendantRefusal, PendantState};
use crate::{FLAG_CHAIN_CLOSED, FLAG_WHEEL_ENABLE};

pub const BUTTON_START: u8 = 1;
pub const BUTTON_HOLD: u8 = 2;
pub const BUTTON_STOP: u8 = 4;
pub const BUTTON_F1: u8 = 8;
pub const UNUSED: u8 = 0xFF;

/// The bit of an OperatorCondition in the active-condition mask the controller builds itself.
pub const fn condition_bit(c: OperatorCondition) -> u32 { 1 << c as u8 }

/// Refuse jogging `axis` while bit `condition` of the active mask is set; `reason` is a profile text id.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct JogRule {
    pub axis: u8,
    pub condition: u8,
    pub reason: u8,
}

#[derive(Clone, Copy, Debug)]
pub struct Config<'a> {
    /// Axis index at switch position `i + 1`, or `UNUSED`.
    pub pendant_axes: [u8; 11],
    pub rules: &'a [JogRule],
    pub has_spindle: bool,
    /// Which of F1 to F3 (0 to 2) is the hold-to-run spindle jog.
    pub spindle_key: Option<u8>,
    /// Distance per wheel count for x1, x10, x100, in the controller's integer position units.
    pub step_units: [i32; 3],
    pub timeout_us: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refusal {
    None,
    NotJogMode,
    NoAxis,
    ChainOpen,
    /// A program is running; the pendant never jogs then, whatever the profile says.
    Busy,
    ReleaseKeys,
    Rule(u8),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Move {
    pub axis: u8,
    pub distance: i32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Output {
    pub movement: Option<Move>,
    pub refusal: Refusal,
    /// The pendant holds jog authority; the HMI's jog controls are refused.
    pub authority: bool,
    pub spindle_jog: bool,
    /// Buttons newly pressed in this frame.
    pub pressed: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProfileError {
    Wire(crate::WireError),
    /// A rule slot at or beyond `rule_count` is not zero.
    Rule,
    /// The spindle key is not F1 to F3 or "none", or disagrees with `has_spindle`.
    SpindleKey,
}

impl From<crate::WireError> for ProfileError {
    fn from(e: crate::WireError) -> Self { ProfileError::Wire(e) }
}

/// A `PendantProfile` record from the host, decoded into the fixed tables `Authority` runs on.
/// The host builds it with `PendantCompiler` (machinekit.operator); `digest` lets both sides
/// confirm they hold the same profile before the pendant is accepted.
#[derive(Clone, Copy, Debug)]
pub struct CompiledProfile {
    pendant_axes: [u8; 11],
    rules: [JogRule; 8],
    rule_count: usize,
    has_spindle: bool,
    spindle_key: Option<u8>,
    step_units: [i32; 3],
    timeout_us: u64,
    host_timeout_us: u64,
    reasons: [[u8; 21]; 8],
}

impl CompiledProfile {
    pub fn from_record(input: &[u8]) -> Result<Self, ProfileError> {
        let r = crate::pendant_wire::PendantProfile::decode(input)?;
        let count = r.rule_count as usize;
        let mut rules = [JogRule { axis: 0, condition: 0, reason: 0 }; 8];
        for i in 0..8 {
            if i < count {
                rules[i] = JogRule { axis: r.rule_axis[i], condition: r.rule_condition[i], reason: r.rule_reason[i] };
            } else if r.rule_axis[i] != 0 || r.rule_condition[i] != 0 || r.rule_reason[i] != 0 {
                return Err(ProfileError::Rule);
            }
        }
        let has_spindle = r.has_spindle == 1;
        let spindle_key = match r.spindle_key {
            UNUSED => None,
            k @ 0..=2 if has_spindle => Some(k),
            _ => return Err(ProfileError::SpindleKey),
        };
        Ok(CompiledProfile {
            pendant_axes: r.axes, rules, rule_count: count, has_spindle, spindle_key, step_units: r.step_units,
            timeout_us: r.timeout_ms as u64 * 1000, host_timeout_us: r.host_timeout_ms as u64 * 1000,
            reasons: [[0; 21]; 8],
        })
    }

    /// Store one `PendantReasonText` record; the status line for `Refusal::Rule(id)`.
    pub fn set_reason(&mut self, input: &[u8]) -> Result<(), ProfileError> {
        let t = crate::pendant_wire::PendantReasonText::decode(input)?;
        self.reasons[t.index as usize] = t.text;
        Ok(())
    }

    /// The status text for a rule reason id, up to its NUL; empty when none was stored.
    pub fn reason_text(&self, id: u8) -> &[u8] {
        match self.reasons.get(id as usize) {
            Some(text) => crate::label(text),
            None => &[],
        }
    }

    /// How long an OperatorState stays fresh; give it to `HostLink::new`.
    pub fn host_timeout_us(&self) -> u64 { self.host_timeout_us }

    pub fn config(&self) -> Config<'_> {
        Config {
            pendant_axes: self.pendant_axes, rules: &self.rules[..self.rule_count], has_spindle: self.has_spindle,
            spindle_key: self.spindle_key, step_units: self.step_units, timeout_us: self.timeout_us,
        }
    }
}

/// What the host has said it allows. The host sends only intent (`jog_allowed`); machine facts come
/// from the controller. No fresh state means "not allowed", the same fail-safe as the pendant lease.
pub struct HostLink {
    timeout_us: u64,
    last_sequence: Option<u16>,
    last_seen: u64,
    jog_allowed: bool,
}

impl HostLink {
    pub fn new(timeout_us: u64) -> Self {
        HostLink { timeout_us, last_sequence: None, last_seen: 0, jog_allowed: false }
    }

    /// False when the state was stale or a duplicate and changed nothing.
    pub fn accept(&mut self, s: &OperatorState, now_us: u64) -> bool {
        if let Some(last) = self.last_sequence {
            let ahead = s.sequence.wrapping_sub(last);
            if ahead == 0 || ahead >= 0x8000 { return false; }
        }
        self.last_sequence = Some(s.sequence);
        self.last_seen = now_us;
        self.jog_allowed = s.jog_allowed == 1;
        true
    }

    /// The `jog_mode` argument of `Authority::accept`.
    pub fn jog_allowed(&self, now_us: u64) -> bool {
        self.jog_allowed && self.last_sequence.is_some() && now_us.wrapping_sub(self.last_seen) <= self.timeout_us
    }
}

/// 32-bit FNV-1a over the records in order: the profile, its reason texts, then its labels.
pub fn digest(records: &[&[u8]]) -> u32 {
    let mut h: u32 = 0x811C_9DC5;
    for record in records {
        for &b in *record {
            h ^= b as u32;
            h = h.wrapping_mul(0x0100_0193);
        }
    }
    h
}

impl Output {
    /// The controller's report of this decision to the host.
    pub fn to_event(&self, sequence: u16) -> PendantEvent {
        let (refusal, reason) = match self.refusal {
            Refusal::None => (PendantRefusal::None, 0),
            Refusal::NotJogMode => (PendantRefusal::NotJogMode, 0),
            Refusal::NoAxis => (PendantRefusal::NoAxis, 0),
            Refusal::ChainOpen => (PendantRefusal::ChainOpen, 0),
            Refusal::Busy => (PendantRefusal::Busy, 0),
            Refusal::ReleaseKeys => (PendantRefusal::ReleaseKeys, 0),
            Refusal::Rule(reason) => (PendantRefusal::Rule, reason),
        };
        PendantEvent {
            sequence,
            axis: self.movement.map_or(UNUSED, |m| m.axis),
            flags: self.authority as u8 | (self.spindle_jog as u8) << 1,
            distance: self.movement.map_or(0, |m| m.distance),
            pressed: self.pressed,
            refusal: refusal as u8,
            reason,
            reserved: 0,
        }
    }
}

pub struct Authority<'a> {
    cfg: Config<'a>,
    last_sequence: Option<u16>,
    last_seen: u64,
    alive: bool,
    needs_release: bool,
    previous: Option<PendantState>,
}

impl<'a> Authority<'a> {
    pub fn new(cfg: Config<'a>) -> Self {
        Authority { cfg, last_sequence: None, last_seen: 0, alive: false, needs_release: false, previous: None }
    }

    /// True while a live pendant has its axis switch away from OFF. The HMI must not jog then.
    pub fn holds_authority(&self, now_us: u64) -> bool {
        self.alive
            && now_us.wrapping_sub(self.last_seen) <= self.cfg.timeout_us
            && self.previous.map_or(false, |p| p.position != 0)
    }

    /// Call periodically even without frames; ends the lease when the heartbeat stops.
    pub fn tick(&mut self, now_us: u64) {
        if self.alive && now_us.wrapping_sub(self.last_seen) > self.cfg.timeout_us {
            self.alive = false;
            self.needs_release = true;
            self.previous = None;
        }
    }

    /// `None` means the frame was stale or a duplicate and changes nothing.
    pub fn accept(&mut self, s: &PendantState, now_us: u64, jog_mode: bool, active: u32) -> Option<Output> {
        self.tick(now_us);
        if let Some(last) = self.last_sequence {
            let ahead = s.sequence.wrapping_sub(last);
            if ahead == 0 || ahead >= 0x8000 { return None; }
        }
        self.last_sequence = Some(s.sequence);
        self.last_seen = now_us;
        let before = self.previous;
        self.previous = Some(*s);
        let wheel_enable = s.flags & FLAG_WHEEL_ENABLE != 0;
        let spindle_bit = self.cfg.spindle_key.map_or(0, |k| BUTTON_F1 << k);
        if self.needs_release {
            if wheel_enable || s.buttons & spindle_bit != 0 {
                return Some(Output { movement: None, refusal: Refusal::ReleaseKeys, authority: false, spindle_jog: false, pressed: 0 });
            }
            self.needs_release = false;
        }
        self.alive = true;
        let pressed = before.map_or(0, |b| s.buttons & !b.buttons);
        let authority = s.position != 0;
        if s.flags & FLAG_CHAIN_CLOSED == 0 {
            return Some(Output { movement: None, refusal: Refusal::ChainOpen, authority, spindle_jog: false, pressed });
        }
        let mut refusal = Refusal::None;
        let mut movement = None;
        if authority && active & condition_bit(OperatorCondition::ProgramRunning) != 0 {
            return Some(Output { movement: None, refusal: Refusal::Busy, authority, spindle_jog: false, pressed });
        }
        if s.position != 0 {
            let axis = self.cfg.pendant_axes[s.position as usize - 1];
            if !jog_mode {
                refusal = Refusal::NotJogMode;
            } else if axis == UNUSED {
                refusal = Refusal::NoAxis;
            } else if let Some(rule) = self.cfg.rules.iter().find(|r| r.axis == axis && active & (1 << r.condition) != 0) {
                refusal = Refusal::Rule(rule.reason);
            } else if wheel_enable && before.map_or(false, |b| b.flags & FLAG_WHEEL_ENABLE != 0) && s.wheel_delta != 0 {
                movement = Some(Move { axis, distance: s.wheel_delta as i32 * self.cfg.step_units[s.step as usize] });
            }
        }
        let spindle_jog = self.cfg.has_spindle && authority && jog_mode && refusal == Refusal::None && s.buttons & spindle_bit != 0;
        Some(Output { movement, refusal, authority, spindle_jog, pressed })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Drill-tap layout: x y z a spindle at positions 1 to 5; F2 is the spindle jog.
    const DRILL: Config<'static> = Config {
        pendant_axes: [0, 1, 2, 3, 4, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED],
        rules: &[], has_spindle: true, spindle_key: Some(1), step_units: [1, 10, 100], timeout_us: 50_000,
    };
    // Hobber: x y z a b c at positions 1 to 6; B and C refused while the gearbox (bit 0) is engaged.
    const RULES: [JogRule; 2] = [JogRule { axis: 4, condition: 0, reason: 7 }, JogRule { axis: 5, condition: 0, reason: 7 }];
    const HOBBER: Config<'static> = Config {
        pendant_axes: [0, 1, 2, 3, 4, 5, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED],
        rules: &RULES, has_spindle: true, spindle_key: Some(1), step_units: [1, 10, 100], timeout_us: 50_000,
    };

    struct Bench { seq: u16, t: u64 }
    impl Bench {
        fn new() -> Self { Bench { seq: 0, t: 0 } }
        fn frame(&mut self, position: u8, delta: i16, enable: bool, buttons: u8, step: u8, closed: bool) -> PendantState {
            self.seq = self.seq.wrapping_add(1);
            let flags = if enable { FLAG_WHEEL_ENABLE } else { 0 } | if closed { FLAG_CHAIN_CLOSED } else { 0 };
            PendantState { sequence: self.seq, wheel_delta: delta, position, step, buttons, flags }
        }
        fn nx(&mut self, ms: u64) -> u64 { self.t += ms * 1000; self.t }
    }

    #[test]
    fn first_enabled_frame_never_jogs() {
        let (mut a, mut b) = (Authority::new(DRILL), Bench::new());
        let f = b.frame(3, 7, true, 0, 1, true);
        let out = a.accept(&f, 0, true, 0).unwrap();
        assert_eq!((out.movement, out.authority), (None, true));
        let f = b.frame(3, 5, true, 0, 1, true);
        let out = a.accept(&f, b.nx(10), true, 0).unwrap();
        assert_eq!(out.movement, Some(Move { axis: 2, distance: 50 }));
        let f = b.frame(3, 9, false, 0, 1, true);
        let out = a.accept(&f, b.nx(10), true, 0).unwrap();
        assert_eq!((out.movement, out.refusal), (None, Refusal::None));
    }

    #[test]
    fn drops_duplicates_and_stale_and_handles_wrap() {
        let (mut a, mut b) = (Authority::new(DRILL), Bench::new());
        let f = b.frame(3, 0, true, 0, 1, true);
        a.accept(&f, 0, true, 0).unwrap();
        assert!(a.accept(&f, 1000, true, 0).is_none());
        let stale = PendantState { sequence: f.sequence.wrapping_sub(1), ..f };
        assert!(a.accept(&stale, 1000, true, 0).is_none());
        for jump in [30000u16, 60000, 65534] {
            b.seq = jump - 1;
            let f = b.frame(3, 0, true, 0, 1, true);
            assert!(a.accept(&f, b.nx(10), true, 0).is_some());
        }
        let f = b.frame(3, 0, true, 0, 1, true); // 65535
        assert!(a.accept(&f, b.nx(10), true, 0).is_some());
        let f = b.frame(3, 0, true, 0, 1, true); // wraps to 0
        assert_eq!(f.sequence, 0);
        assert!(a.accept(&f, b.nx(10), true, 0).is_some());
    }

    #[test]
    fn gates_mode_position_and_chain() {
        let (mut a, mut b) = (Authority::new(DRILL), Bench::new());
        let f = b.frame(3, 0, true, 0, 1, true);
        a.accept(&f, 0, true, 0);
        let f = b.frame(3, 1, true, 0, 1, true);
        assert_eq!(a.accept(&f, b.nx(10), false, 0).unwrap().refusal, Refusal::NotJogMode);
        let f = b.frame(9, 1, true, 0, 1, true);
        assert_eq!(a.accept(&f, b.nx(10), true, 0).unwrap().refusal, Refusal::NoAxis);
        let f = b.frame(3, 1, true, 0, 1, false);
        let out = a.accept(&f, b.nx(10), true, 0).unwrap();
        assert_eq!((out.movement, out.refusal), (None, Refusal::ChainOpen));
    }

    #[test]
    fn hobber_gearbox_rule() {
        let (mut a, mut b) = (Authority::new(HOBBER), Bench::new());
        let f = b.frame(5, 0, true, 0, 1, true);
        a.accept(&f, 0, true, 1);
        let f = b.frame(5, 3, true, 0, 1, true);
        let out = a.accept(&f, b.nx(10), true, 1).unwrap();
        assert_eq!((out.movement, out.refusal), (None, Refusal::Rule(7)));
        let f = b.frame(3, 3, true, 0, 1, true);
        assert_eq!(a.accept(&f, b.nx(10), true, 1).unwrap().movement, Some(Move { axis: 2, distance: 30 }));
        let f = b.frame(5, 3, true, 0, 1, true);
        assert_eq!(a.accept(&f, b.nx(10), true, 0).unwrap().movement, Some(Move { axis: 4, distance: 30 }));
    }

    #[test]
    fn spindle_jog_is_hold_to_run() {
        let (mut a, mut b) = (Authority::new(DRILL), Bench::new());
        let f = b.frame(5, 0, false, 0, 1, true);
        a.accept(&f, 0, true, 0);
        let f = b.frame(5, 0, false, BUTTON_F1 << 1, 1, true);
        let out = a.accept(&f, b.nx(10), true, 0).unwrap();
        assert!(out.spindle_jog && out.pressed == BUTTON_F1 << 1);
        let f = b.frame(5, 0, false, 0, 1, true);
        assert!(!a.accept(&f, b.nx(10), true, 0).unwrap().spindle_jog);
    }

    #[test]
    fn link_loss_needs_release_and_hands_back_to_hmi() {
        let (mut a, mut b) = (Authority::new(DRILL), Bench::new());
        let f = b.frame(5, 0, false, 0, 1, true);
        a.accept(&f, 0, true, 0);
        let f = b.frame(5, 0, false, BUTTON_F1 << 1, 1, true);
        a.accept(&f, b.nx(10), true, 0);
        let t = b.t;
        assert!(a.holds_authority(t + 40_000));
        assert!(!a.holds_authority(t + 60_000));
        a.tick(t + 60_000);
        let f = b.frame(5, 0, true, BUTTON_F1 << 1, 1, true);
        let out = a.accept(&f, t + 70_000, true, 0).unwrap();
        assert_eq!((out.spindle_jog, out.refusal), (false, Refusal::ReleaseKeys));
        let f = b.frame(5, 0, false, 0, 1, true);
        let out = a.accept(&f, t + 80_000, true, 0).unwrap();
        assert_eq!((out.refusal, out.authority), (Refusal::None, true));
        let f = b.frame(0, 0, false, 0, 1, true);
        a.accept(&f, t + 90_000, true, 0).unwrap();
        assert!(!a.holds_authority(t + 90_000));
    }

    // Compiled by the Haxe PendantCompiler; the Haxe tests check the same file byte for byte.
    const VECTORS: &str = include_str!("../vectors/profiles.txt");

    fn unhex(s: &str) -> std::vec::Vec<u8> {
        (0..s.len() / 2).map(|i| u8::from_str_radix(&s[i * 2..i * 2 + 2], 16).unwrap()).collect()
    }
    fn list(s: &str) -> std::vec::Vec<std::vec::Vec<u8>> {
        if s == "-" { std::vec::Vec::new() } else { s.split(';').map(unhex).collect() }
    }
    struct Vector { digest: u32, profile: std::vec::Vec<u8>, reasons: std::vec::Vec<std::vec::Vec<u8>>, labels: std::vec::Vec<std::vec::Vec<u8>> }
    fn vector(name: &str) -> Vector {
        let line = VECTORS.lines().find(|l| l.starts_with(name)).unwrap();
        let f: std::vec::Vec<&str> = line.split(' ').collect();
        Vector { digest: u32::from_str_radix(f[1], 16).unwrap(), profile: unhex(f[2]), reasons: list(f[3]), labels: list(f[4]) }
    }
    fn same(a: &Config, b: &Config) {
        assert_eq!(a.pendant_axes, b.pendant_axes);
        assert_eq!(a.rules, b.rules);
        assert_eq!((a.has_spindle, a.spindle_key, a.step_units, a.timeout_us), (b.has_spindle, b.spindle_key, b.step_units, b.timeout_us));
    }

    #[test]
    fn compiled_profiles_match_the_hand_written_configs() {
        // The scenarios above run on DRILL and HOBBER, which mirror the Haxe tests; the host-compiled
        // vectors must decode to exactly those configs.
        let drill = CompiledProfile::from_record(&vector("drill-tap").profile).unwrap();
        same(&drill.config(), &DRILL);
        let hobber = CompiledProfile::from_record(&vector("hobber").profile).unwrap();
        // The hobber profile indexes its axes c b x y z a, so positions differ from HOBBER's x y z a b c.
        let c = hobber.config();
        assert_eq!(c.pendant_axes[..6], [2, 3, 4, 5, 1, 0]);
        assert_eq!(c.rules, [JogRule { axis: 1, condition: 0, reason: 0 }, JogRule { axis: 0, condition: 0, reason: 0 }]);
        assert_eq!((c.has_spindle, c.spindle_key, c.step_units, c.timeout_us), (true, Some(1), [1, 10, 100], 50_000));
        let arm_profile = CompiledProfile::from_record(&vector("arm").profile).unwrap();
        let arm = arm_profile.config();
        assert_eq!((arm.has_spindle, arm.spindle_key, arm.rules.len()), (false, None, 0));
    }

    #[test]
    fn compiled_hobber_behaves_like_the_scenarios() {
        let v = vector("hobber");
        let mut p = CompiledProfile::from_record(&v.profile).unwrap();
        for r in &v.reasons { p.set_reason(r).unwrap(); }
        assert_eq!(p.reason_text(0), b"Gearbox engaged");
        let mut a = Authority::new(p.config());
        let mut b = Bench::new();
        let f = b.frame(5, 0, true, 0, 1, true); // position 5 is B
        a.accept(&f, 0, true, 1);
        let f = b.frame(5, 3, true, 0, 1, true);
        let out = a.accept(&f, b.nx(10), true, 1).unwrap();
        assert_eq!((out.movement, out.refusal), (None, Refusal::Rule(0)));
        let f = b.frame(3, 3, true, 0, 1, true); // position 3 is Z, index 4
        assert_eq!(a.accept(&f, b.nx(10), true, 1).unwrap().movement, Some(Move { axis: 4, distance: 30 }));
        let f = b.frame(5, 3, true, 0, 1, true);
        assert_eq!(a.accept(&f, b.nx(10), true, 0).unwrap().movement, Some(Move { axis: 1, distance: 30 }));
    }

    #[test]
    fn digest_matches_the_host() {
        for name in ["drill-tap", "hobber", "arm"] {
            let v = vector(name);
            let mut records: std::vec::Vec<&[u8]> = std::vec![&v.profile[..]];
            records.extend(v.reasons.iter().map(|r| &r[..]));
            records.extend(v.labels.iter().map(|r| &r[..]));
            assert_eq!(digest(&records), v.digest, "{name}");
        }
        assert_eq!(digest(&[b"a"]), 0xE40C_292C); // FNV-1a reference value
    }

    #[test]
    fn rejects_inconsistent_records() {
        let good = vector("hobber").profile;
        let mut bad = good.clone();
        bad[11] = 9; // rule_count above 8
        assert_eq!(CompiledProfile::from_record(&bad).unwrap_err(), ProfileError::Wire(crate::WireError::Constraint("PendantProfile.rule_count")));
        let mut bad = good.clone();
        bad[12 + 5] = 1; // a rule slot beyond rule_count
        assert_eq!(CompiledProfile::from_record(&bad).unwrap_err(), ProfileError::Rule);
        let mut bad = good.clone();
        bad[37] = 3; // spindle key F4
        assert_eq!(CompiledProfile::from_record(&bad).unwrap_err(), ProfileError::SpindleKey);
        let mut bad = good.clone();
        bad[36] = 0; // spindle key without a spindle
        assert_eq!(CompiledProfile::from_record(&bad).unwrap_err(), ProfileError::SpindleKey);
        assert!(CompiledProfile::from_record(&good[..51]).is_err());
    }

    #[test]
    fn host_link_fails_safe_when_stale() {
        let mut h = HostLink::new(250_000);
        assert!(!h.jog_allowed(0));
        let allow = |sequence| OperatorState { sequence, jog_allowed: 1, reserved: 0 };
        assert!(h.accept(&allow(1), 0));
        assert!(h.jog_allowed(200_000));
        assert!(!h.jog_allowed(300_000));
        assert!(!h.accept(&allow(1), 310_000), "duplicate");
        assert!(h.accept(&allow(2), 310_000));
        assert!(h.jog_allowed(320_000));
        assert!(h.accept(&OperatorState { sequence: 3, jog_allowed: 0, reserved: 0 }, 330_000));
        assert!(!h.jog_allowed(340_000), "the host withdrew permission");
        for (i, sequence) in [30_000u16, 60_000, 0xFFFF, 0].into_iter().enumerate() {
            assert!(h.accept(&allow(sequence), 350_000 + i as u64), "sequence {sequence}");
        }
    }

    #[test]
    fn a_running_program_blocks_the_pendant_whatever_the_profile_says() {
        let (mut a, mut b) = (Authority::new(DRILL), Bench::new());
        let f = b.frame(3, 0, true, 0, 1, true);
        a.accept(&f, 0, true, 0);
        let f = b.frame(3, 4, true, 0, 1, true);
        let busy = condition_bit(OperatorCondition::ProgramRunning);
        let out = a.accept(&f, b.nx(10), true, busy).unwrap();
        assert_eq!((out.movement, out.refusal, out.spindle_jog), (None, Refusal::Busy, false));
        let f = b.frame(3, 4, true, 0, 1, true);
        assert!(a.accept(&f, b.nx(10), true, 0).unwrap().movement.is_some());
    }
}
