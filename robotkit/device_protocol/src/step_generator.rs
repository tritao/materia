//! Device-tick step and direction generation; no heap or host clock needed.
use crate::Board;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SkewGroup {
    pub first: usize,
    pub second: usize,
    pub first_ratio: f64,
    pub second_ratio: f64,
    pub bound: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StepFault { DualDriveSkew, InvalidActuatorCount }

pub struct StepGenerator<const A: usize> {
    steps_per_unit: [f64; A],
    setup_ticks: [u64; A],
    min_interval_ticks: [u64; A],
    direction: [Option<bool>; A],
    direction_since: [u64; A],
    last_step: [Option<u64>; A],
    skew: [Option<SkewGroup>; A],
    skew_count: usize,
}

impl<const A: usize> StepGenerator<A> {
    pub fn new(steps_per_unit: [f64; A], setup_ticks: [u64; A],
        max_rate: [f64; A], tick_hz: u64) -> Option<Self> {
        if A == 0 || A > 64 || tick_hz == 0 ||
            steps_per_unit.iter().any(|v| !v.is_finite() || *v <= 0.0) ||
            max_rate.iter().any(|v| !v.is_finite() || *v < 0.0) { return None; }
        let mut min_interval_ticks = [0; A];
        for a in 0..A {
            if max_rate[a] > 0.0 {
                let interval = tick_hz as f64 / (max_rate[a] * steps_per_unit[a]);
                if !interval.is_finite() || interval > u64::MAX as f64 { return None; }
                min_interval_ticks[a] = (interval as u64) +
                    u64::from(interval > interval as u64 as f64);
            }
        }
        Some(Self { steps_per_unit, setup_ticks, min_interval_ticks,
            direction: [None; A], direction_since: [0; A], last_step: [None; A],
            skew: [None; A], skew_count: 0 })
    }

    pub fn set_skew_group(&mut self, group: SkewGroup) -> bool {
        if group.first >= A || group.second >= A || group.first == group.second ||
            !group.first_ratio.is_finite() || group.first_ratio == 0.0 ||
            !group.second_ratio.is_finite() || group.second_ratio == 0.0 ||
            !group.bound.is_finite() || group.bound <= 0.0 || self.skew_count >= A { return false; }
        self.skew[self.skew_count] = Some(group);
        self.skew_count += 1;
        true
    }

    pub fn tick<B: Board>(&mut self, board: &mut B, targets: [f32; A]) -> Result<(), StepFault> {
        self.tick_active(board, targets, A)
    }

    /// Only the negotiated outputs may emit pulses, including after a smaller session replaces a larger one.
    pub fn tick_active<B: Board>(&mut self, board: &mut B, targets: [f32; A], count: usize) -> Result<(), StepFault> {
        if count > A { return Err(StepFault::InvalidActuatorCount); }
        let now = board.now_ticks();
        for a in 0..count {
            let raw = targets[a] as f64 * self.steps_per_unit[a];
            let truncated = raw as i64;
            let desired = truncated.saturating_sub(i64::from(raw < truncated as f64));
            let actual = board.step_count(a);
            if desired == actual { continue; }
            let forward = desired > actual;
            if self.direction[a] != Some(forward) {
                self.direction[a] = Some(forward);
                self.direction_since[a] = now;
                board.set_direction(a, forward);
            }
            if now.saturating_sub(self.direction_since[a]) < self.setup_ticks[a] { continue; }
            if let Some(last) = self.last_step[a] {
                if now.saturating_sub(last) < self.min_interval_ticks[a] { continue; }
            }
            board.step_pulse(a, forward);
            self.last_step[a] = Some(now);
        }
        for group in self.skew[..self.skew_count].iter().flatten() {
            if group.first >= count || group.second >= count { continue; }
            let first = board.step_count(group.first) as f64 /
                self.steps_per_unit[group.first] / group.first_ratio;
            let second = board.step_count(group.second) as f64 /
                self.steps_per_unit[group.second] / group.second_ratio;
            if (first - second).abs() > group.bound { return Err(StepFault::DualDriveSkew); }
        }
        Ok(())
    }
}
