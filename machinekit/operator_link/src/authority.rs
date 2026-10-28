//! Pendant jog authority for the cabinet controller. Not a safety function; the E-stop chain is hard wired and `chain_closed` is only an observation.
//!
//! - Frames at or behind the last accepted sequence are dropped (16-bit wrap aware).
//! - No frame within `timeout_us` ends the lease; it returns only after a frame arrives with the
//!   wheel and the spindle key released.
//! - Wheel counts count only while the enable was held in this frame and the previous one, and
//!   only in Jog mode. A lost frame loses its counts: travel is short, never long.
//!
//! Axes are profile indices and conditions are bits of a mask, so no strings reach the controller.

use crate::operator_wire::{OperatorCondition, PendantEvent, PendantRefusal, PendantState};
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
            && now_us.saturating_sub(self.last_seen) <= self.cfg.timeout_us
            && self.previous.map_or(false, |p| p.position != 0)
    }

    /// Call periodically even without frames; ends the lease when the heartbeat stops.
    pub fn tick(&mut self, now_us: u64) {
        if self.alive && now_us.saturating_sub(self.last_seen) > self.cfg.timeout_us {
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
    use crate::testing::{Bench, DRILL, HOBBER};

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

    #[test]
    fn a_clock_slightly_behind_a_frame_timestamp_is_not_a_timeout() {
        let (mut a, mut b) = (Authority::new(DRILL), Bench::new());
        let f = b.frame(3, 0, true, 0, 1, true);
        a.accept(&f, 10_000, true, 0);
        assert!(a.holds_authority(9_975), "the consumer's clock may read a little earlier than the producer's");
        a.tick(9_975);
        assert!(a.holds_authority(10_020));
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
