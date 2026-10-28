//! What the host has said it allows, as the controller tracks it.

use crate::operator_wire::OperatorState;

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
        self.jog_allowed && self.last_sequence.is_some() && now_us.saturating_sub(self.last_seen) <= self.timeout_us
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_clock_slightly_behind_a_frame_timestamp_is_not_a_timeout() {
        let mut h = HostLink::new(250_000);
        h.accept(&OperatorState { sequence: 1, jog_allowed: 1, reserved: 0 }, 10_000);
        assert!(h.jog_allowed(9_975), "the consumer's clock may read a little earlier than the producer's");
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
}
