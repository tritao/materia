//! SR15 portable router timing and stop policy. No register access or wire-format changes.
//! The SR18 adapter must apply `SafeOutputs` to timer/DMA gates and physical outputs.
#[path = "nucleo-g474re/src/router_profile.rs"]
pub mod g474;

pub const TIMER_HZ: u64 = 10_000_000;
pub const STEP_HIGH: u64 = 20; // 2 us
pub const STEP_PERIOD: u64 = 50; // 200 kHz
pub const DIR_SETUP: u64 = 50; // 5 us
pub const DIR_HOLD: u64 = 50; // 5 us after falling edge (conservative)
pub const DMA_HORIZON: u64 = 30_000; // less than half a 16-bit timer cycle

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TimingFault {
    TooSoon,
    Rate,
    DirectionHold,
    Horizon,
    Overflow,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pulse {
    pub direction_at: Option<u64>,
    pub direction: bool,
    pub rise: u64,
    pub fall: u64,
}
#[derive(Default)]
pub struct StepDir {
    last: Option<Pulse>,
}
impl StepDir {
    /// Absolute timer ticks. A failed request never mutates the accepted sequence.
    /// The adapter arms idle channels explicitly; do not leave a toggling CCR armed
    /// across a long idle/wrap. Emit a complete rise/fall pair, never half a pulse.
    pub fn pulse(&mut self, now: u64, rise: u64, direction: bool) -> Result<Pulse, TimingFault> {
        let fall = rise.checked_add(STEP_HIGH).ok_or(TimingFault::Overflow)?;
        if rise < now || rise - now < DIR_SETUP {
            return Err(TimingFault::TooSoon);
        }
        if fall - now > DMA_HORIZON {
            return Err(TimingFault::Horizon);
        }
        let changed = self.last.map(|p| p.direction != direction).unwrap_or(true);
        let direction_at = if changed {
            Some(rise.checked_sub(DIR_SETUP).ok_or(TimingFault::TooSoon)?)
        } else {
            None
        };
        if let Some(at) = direction_at {
            if at < now {
                return Err(TimingFault::TooSoon);
            }
        }
        if let Some(last) = self.last {
            if rise < last.rise || rise - last.rise < STEP_PERIOD {
                return Err(TimingFault::Rate);
            }
            if let Some(at) = direction_at {
                let earliest = last
                    .fall
                    .checked_add(DIR_HOLD)
                    .ok_or(TimingFault::Overflow)?;
                if at < earliest {
                    return Err(TimingFault::DirectionHold);
                }
            }
        }
        let p = Pulse {
            direction_at,
            direction,
            rise,
            fall,
        };
        self.last = Some(p);
        Ok(p)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RouterStop {
    Clock,
    Lease,
    FeedbackStale,
    DriveAlarm(usize),
    FollowingError(usize),
    Configuration,
    Watchdog,
}
#[derive(Clone, Copy, Debug)]
pub struct Feedback {
    /// Commanded *emitted* pulses, never the end of a queued DMA buffer.
    pub emitted_steps: i64,
    pub encoder_counts: i64,
    pub counts_per_step_num: i64,
    pub counts_per_step_den: i64,
    pub limit_counts: i64,
    pub alarm: bool,
    pub ready: bool,
}
impl Feedback {
    fn fault(&self, axis: usize) -> Option<RouterStop> {
        if self.counts_per_step_num <= 0 || self.counts_per_step_den <= 0 || self.limit_counts <= 0
        {
            return Some(RouterStop::Configuration);
        }
        if self.alarm || !self.ready {
            return Some(RouterStop::DriveAlarm(axis));
        }
        let expected = (self.emitted_steps as i128).checked_mul(self.counts_per_step_num as i128);
        let actual = (self.encoder_counts as i128).checked_mul(self.counts_per_step_den as i128);
        let bound = (self.limit_counts as i128).checked_mul(self.counts_per_step_den as i128);
        let difference = expected
            .and_then(|e| actual.and_then(|a| e.checked_sub(a)))
            .and_then(|v| v.checked_abs());
        match (difference, bound) {
            (Some(error), Some(limit)) if error <= limit => None,
            (Some(_), Some(_)) => Some(RouterStop::FollowingError(axis)),
            _ => Some(RouterStop::Configuration),
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SafeOutputs {
    pub enables: bool,
    pub step_dma: bool,
    pub spindle_run: bool,
    pub spindle_dac: u16,
    pub release_valve: bool,
    pub brake_release: bool,
    pub feed_iwdg: bool,
}
impl SafeOutputs {
    pub const OFF: Self = Self {
        enables: false,
        step_dma: false,
        spindle_run: false,
        spindle_dac: 0,
        release_valve: false,
        brake_release: false,
        feed_iwdg: false,
    };
}
/// 1 kHz supervisor; fault latches across healthy samples. Only explicit manual
/// reset at rest may re-arm. Hardware IWDG still resets a CPU which never calls tick.
pub struct RouterGuard {
    last_tick: u64,
    armed: bool,
    fault: Option<RouterStop>,
}
impl RouterGuard {
    pub fn new(now_us: u64) -> Self {
        Self {
            last_tick: now_us,
            armed: false,
            fault: None,
        }
    }
    fn inspect<const A: usize>(
        &self,
        now: u64,
        feedback_at: u64,
        lease_until: u64,
        axes: &[Feedback; A],
    ) -> Option<RouterStop> {
        if now < self.last_tick || feedback_at > now {
            return Some(RouterStop::Clock);
        }
        if now - self.last_tick >= 20_000 {
            return Some(RouterStop::Watchdog);
        }
        if now >= lease_until {
            return Some(RouterStop::Lease);
        }
        if now - feedback_at > 2_000 {
            return Some(RouterStop::FeedbackStale);
        }
        if A == 0 || A > 5 {
            return Some(RouterStop::Configuration);
        }
        for (i, a) in axes.iter().enumerate() {
            if let Some(f) = a.fault(i) {
                return Some(f);
            }
        }
        None
    }
    pub fn reset<const A: usize>(
        &mut self,
        now: u64,
        feedback_at: u64,
        lease_until: u64,
        axes: &[Feedback; A],
        manual_reset: bool,
        at_rest: bool,
    ) -> bool {
        // A reset after a long disabled interval establishes a new watchdog epoch,
        // but cannot hide a backward clock or a bad live feedback/lease.
        if !manual_reset || !at_rest || now < self.last_tick {
            return false;
        }
        let old = self.last_tick;
        self.last_tick = now;
        if self.inspect(now, feedback_at, lease_until, axes).is_some() {
            self.last_tick = old;
            return false;
        }
        self.fault = None;
        self.armed = true;
        true
    }
    pub fn tick<const A: usize>(
        &mut self,
        now: u64,
        feedback_at: u64,
        lease_until: u64,
        axes: &[Feedback; A],
    ) -> SafeOutputs {
        if self.fault.is_none() {
            self.fault = self.inspect(now, feedback_at, lease_until, axes);
        }
        self.last_tick = now;
        if self.fault.is_some() {
            return SafeOutputs::OFF;
        }
        if !self.armed {
            return SafeOutputs {
                feed_iwdg: true,
                ..SafeOutputs::OFF
            };
        }
        // Process outputs remain separately gated; this is permission, not a run request.
        SafeOutputs {
            enables: true,
            step_dma: true,
            feed_iwdg: true,
            ..SafeOutputs::OFF
        }
    }
    pub fn fault(&self) -> Option<RouterStop> {
        self.fault
    }
}

/// Extend a 16-bit hardware encoder counter. Sample before 32768 edges can elapse;
/// timestamp/freshness enforcement belongs to RouterGuard, not a guessed rollover.
pub struct Encoder16 {
    previous: u16,
    total: i64,
}
impl Encoder16 {
    pub fn new(raw: u16, total: i64) -> Self {
        Self {
            previous: raw,
            total,
        }
    }
    pub fn sample(&mut self, raw: u16) -> Result<i64, RouterStop> {
        let delta = raw.wrapping_sub(self.previous) as i16;
        if delta == i16::MIN {
            return Err(RouterStop::Configuration);
        }
        let value = self
            .total
            .checked_add(delta as i64)
            .ok_or(RouterStop::Configuration)?;
        self.previous = raw;
        self.total = value;
        Ok(value)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn router_encoder_extension() {
        let mut e = Encoder16::new(65_000, 0);
        let mut raw = 65_000u16;
        for i in 1..201 {
            raw = raw.wrapping_add(500);
            assert_eq!(e.sample(raw), Ok(i * 500));
        }
        for i in 1..201 {
            raw = raw.wrapping_sub(500);
            assert_eq!(e.sample(raw), Ok(100_000 - i * 500));
        }
    }
    #[test]
    fn router_encoder_ambiguous_and_overflow() {
        let mut e = Encoder16::new(0, 0);
        assert_eq!(e.sample(32768), Err(RouterStop::Configuration));
        let mut e = Encoder16::new(0, i64::MAX);
        assert_eq!(e.sample(1), Err(RouterStop::Configuration));
    }
    fn feedback() -> [Feedback; 5] {
        [Feedback {
            emitted_steps: 100,
            encoder_counts: 200,
            counts_per_step_num: 2,
            counts_per_step_den: 1,
            limit_counts: 50,
            alarm: false,
            ready: true,
        }; 5]
    }
    fn armed() -> RouterGuard {
        let mut g = RouterGuard::new(0);
        assert!(g.reset(0, 0, 100_000, &feedback(), true, true));
        g
    }
    #[test]
    fn router_rate_and_width() {
        let mut s = StepDir::default();
        for i in 0..500 {
            let p = s.pulse(i * 50, i * 50 + 100, true).unwrap();
            assert_eq!(p.fall - p.rise, 20);
        }
        assert_eq!(s.pulse(24_999, 25_049, true), Err(TimingFault::Rate));
    }
    #[test]
    fn router_direction_setup_hold() {
        let mut s = StepDir::default();
        assert_eq!(s.pulse(0, 100, true).unwrap().direction_at, Some(50));
        assert_eq!(s.pulse(0, 200, false), Err(TimingFault::DirectionHold));
        assert_eq!(s.pulse(0, 220, false).unwrap().direction_at, Some(170));
    }
    #[test]
    fn router_failed_pair_does_not_mutate() {
        let mut s = StepDir::default();
        s.pulse(0, 100, true).unwrap();
        assert!(s.pulse(0, 120, false).is_err());
        assert!(s.pulse(0, 150, true).is_ok());
    }
    #[test]
    fn router_wrap_projection() {
        let mut s = StepDir::default();
        let p = s.pulse(65_500, 65_600, true).unwrap();
        assert_eq!(p.rise as u16, 64);
        assert_eq!(p.fall as u16, 84);
    }
    #[test]
    fn router_horizon_and_overflow() {
        let mut s = StepDir::default();
        assert_eq!(s.pulse(0, 30_000, true), Err(TimingFault::Horizon));
        assert_eq!(s.pulse(0, u64::MAX, true), Err(TimingFault::Overflow));
        assert_eq!(s.pulse(100, 99, true), Err(TimingFault::TooSoon));
    }
    #[test]
    fn router_reset_required() {
        let mut g = RouterGuard::new(0);
        assert_eq!(
            g.tick(1, 1, 100, &feedback()),
            SafeOutputs {
                feed_iwdg: true,
                ..SafeOutputs::OFF
            }
        );
        assert!(!g.reset(2, 2, 100, &feedback(), false, true));
        assert!(!g.reset(2, 2, 100, &feedback(), true, false));
    }
    #[test]
    fn router_following_error_both_signs() {
        for delta in [-51, 51] {
            let mut g = armed();
            let mut a = feedback();
            a[2].encoder_counts += delta;
            assert_eq!(g.tick(1000, 1000, 100_000, &a), SafeOutputs::OFF);
            assert_eq!(g.fault(), Some(RouterStop::FollowingError(2)));
        }
    }
    #[test]
    fn router_boundary_and_ratio() {
        let mut g = armed();
        let mut a = feedback();
        a[0].encoder_counts += 50;
        assert!(g.tick(1000, 1000, 100_000, &a).enables);
        a[0].counts_per_step_num = 3;
        a[0].counts_per_step_den = 2;
        a[0].encoder_counts = 150;
        assert!(g.tick(2000, 2000, 100_000, &a).enables);
    }
    #[test]
    fn router_alarm_stops_all_outputs() {
        for i in 0..5 {
            let mut g = armed();
            let mut a = feedback();
            a[i].alarm = true;
            assert_eq!(g.tick(1000, 1000, 100_000, &a), SafeOutputs::OFF);
            assert_eq!(g.fault(), Some(RouterStop::DriveAlarm(i)));
        }
    }
    #[test]
    fn router_not_ready() {
        let mut g = armed();
        let mut a = feedback();
        a[0].ready = false;
        assert_eq!(g.tick(1000, 1000, 100_000, &a), SafeOutputs::OFF);
    }
    #[test]
    fn router_lease_and_feedback_age() {
        let mut g = armed();
        assert_eq!(g.tick(1000, 1000, 1000, &feedback()), SafeOutputs::OFF);
        assert_eq!(g.fault(), Some(RouterStop::Lease));
        let mut g = armed();
        g.tick(2001, 0, 100_000, &feedback());
        assert_eq!(g.fault(), Some(RouterStop::FeedbackStale));
    }
    #[test]
    fn router_watchdog_latches() {
        let mut g = armed();
        assert_eq!(
            g.tick(20_000, 20_000, 100_000, &feedback()),
            SafeOutputs::OFF
        );
        assert_eq!(g.fault(), Some(RouterStop::Watchdog));
        assert_eq!(
            g.tick(20_001, 20_001, 100_000, &feedback()),
            SafeOutputs::OFF
        );
        assert!(g.reset(21_000, 21_000, 100_000, &feedback(), true, true));
        assert!(g.tick(22_000, 22_000, 100_000, &feedback()).feed_iwdg);
    }
    #[test]
    fn router_clock_and_bad_ratio() {
        let mut g = armed();
        g.tick(1000, 1001, 100_000, &feedback());
        assert_eq!(g.fault(), Some(RouterStop::Clock));
        let mut g = armed();
        let mut a = feedback();
        a[1].counts_per_step_den = 0;
        g.tick(1000, 1000, 100_000, &a);
        assert_eq!(g.fault(), Some(RouterStop::Configuration));
    }
    #[test]
    fn router_fault_cannot_reset_while_present() {
        let mut g = armed();
        let mut a = feedback();
        a[1].alarm = true;
        g.tick(1000, 1000, 100_000, &a);
        assert!(!g.reset(2000, 2000, 100_000, &a, true, true));
        assert_eq!(g.tick(2001, 2001, 100_000, &feedback()), SafeOutputs::OFF);
    }
}
