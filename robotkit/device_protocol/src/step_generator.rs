//! Device-tick step and direction generation; no heap or host clock needed.
use crate::{Board, InputBinding, InputCapture, InputObservation};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SkewGroup {
    pub first: usize,
    pub second: usize,
    pub first_ratio: f64,
    pub second_ratio: f64,
    pub bound: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StepFault { DualDriveSkew, InvalidActuatorCount, InputCounterOverflow, InvalidHomingPurpose }

pub struct StepGenerator<const A: usize> {
    inputs: InputCapture,
    homing_pair: Option<(usize, usize)>,
    held: [Option<i64>; A],
    alignment_steps: [f64; A],
    nominal_steps: [f64; A],
    steps_per_unit: [f64; A],
    setup_ticks: [u64; A],
    min_interval_ticks: [u64; A],
    direction: [Option<bool>; A],
    direction_since: [u64; A],
    last_step: [Option<u64>; A],
    skew: [Option<SkewGroup>; A],
    skew_count: usize,
    squaring_bound: [Option<f64>; A],
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
        Some(Self { inputs: InputCapture::new(), homing_pair: None, held: [None; A],
            alignment_steps: [0.0; A], nominal_steps: [0.0; A], steps_per_unit, setup_ticks, min_interval_ticks,
            direction: [None; A], direction_since: [0; A], last_step: [None; A],
            skew: [None; A], skew_count: 0, squaring_bound: [None; A] })
    }

    pub fn bind_input<B: Board>(&mut self, board: &B, channel: usize,
        binding: InputBinding, actuator_count: usize) -> bool {
        if actuator_count > A { return false; }
        self.inputs.bind(board, channel, binding, actuator_count)
    }

    pub fn input_observation(&self, channel: usize) -> Option<InputObservation> {
        self.inputs.observation(channel)
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

    /// Explicitly relax one configured pair for a bounded squaring move.
    /// The homing owner must call end_squaring on completion, cancellation and faults.
    pub fn begin_squaring(&mut self, first: usize, second: usize, bound: f64) -> bool {
        if !bound.is_finite() { return false; }
        let mut selected = None;
        for i in 0..self.skew_count {
            if let Some(group) = self.skew[i] {
                if (group.first == first && group.second == second) ||
                    (group.first == second && group.second == first) {
                    if selected.is_some() || bound < group.bound || self.squaring_bound[i].is_some() {
                        return false;
                    }
                    selected = Some(i);
                }
            }
        }
        if let Some(i) = selected { self.squaring_bound[i] = Some(bound); true } else { false }
    }

    pub fn end_squaring(&mut self) { self.squaring_bound.fill(None); }

    pub fn begin_homing_pair(&mut self, first: usize, second: usize, bound: f64) -> bool {
        if self.homing_pair.is_some() || !self.begin_squaring(first, second, bound) { return false; }
        self.homing_pair = Some((first, second));
        true
    }

    pub fn hold_homing_side<B: Board>(&mut self, board: &B, actuator: usize) -> bool {
        let Some((first, second)) = self.homing_pair else { return false; };
        if actuator != first && actuator != second { return false; }
        if self.held[actuator].is_some() { return false; }
        let physical = board.step_count(actuator);
        self.held[actuator] = Some(physical);
        self.alignment_steps[actuator] = physical as f64 - self.nominal_steps[actuator];
        true
    }

    pub fn release_homing_side(&mut self, actuator: usize) -> bool {
        let Some((first, second)) = self.homing_pair else { return false; };
        if (actuator != first && actuator != second) || self.held[actuator].is_none() { return false; }
        self.held[actuator] = None;
        true
    }

    /// Preserve the physical alignment while restoring normal skew bounds.
    pub fn end_homing_pair(&mut self) {
        self.held.fill(None);
        self.homing_pair = None;
        self.end_squaring();
    }

    pub fn tick<B: Board>(&mut self, board: &mut B, targets: [f32; A]) -> Result<(), StepFault> {
        self.tick_active_with_purpose(board, targets, A, 0)
    }

    pub fn tick_with_purpose<B: Board>(&mut self, board: &mut B, targets: [f32; A], purpose: u8) -> Result<(), StepFault> {
        self.tick_active_with_purpose(board, targets, A, purpose)
    }

    pub fn tick_active<B: Board>(&mut self, board: &mut B, targets: [f32; A], count: usize) -> Result<(), StepFault> {
        self.tick_active_with_purpose(board, targets, count, 0)
    }

    pub fn tick_active_with_purpose<B: Board>(&mut self, board: &mut B, targets: [f32; A], count: usize, purpose: u8) -> Result<(), StepFault> {
        if count > A { return Err(StepFault::InvalidActuatorCount); }
        if self.homing_pair.is_some() && purpose != 2 {
            self.end_homing_pair();
            return Err(StepFault::InvalidHomingPurpose);
        }
        if !self.inputs.sample(board, None) { self.end_homing_pair(); return Err(StepFault::InputCounterOverflow); }
        let now = board.now_ticks();
        for a in 0..count {
            self.nominal_steps[a] = targets[a] as f64 * self.steps_per_unit[a];
            if let Some(physical) = self.held[a] {
                self.alignment_steps[a] = physical as f64 - self.nominal_steps[a];
                continue;
            }
            let raw = self.nominal_steps[a] + self.alignment_steps[a];
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
            if !self.inputs.sample(board, Some(a)) { self.end_homing_pair(); return Err(StepFault::InputCounterOverflow); }
            self.last_step[a] = Some(now);
        }
        for (i, configured) in self.skew[..self.skew_count].iter().enumerate() {
            let Some(group) = configured else { continue; };
            if group.first >= count || group.second >= count { continue; }
            let first = board.step_count(group.first) as f64 /
                self.steps_per_unit[group.first] / group.first_ratio;
            let second = board.step_count(group.second) as f64 /
                self.steps_per_unit[group.second] / group.second_ratio;
            let bound = self.squaring_bound[i].unwrap_or(group.bound);
            if (first - second).abs() > bound { self.end_homing_pair(); return Err(StepFault::DualDriveSkew); }
        }
        Ok(())
    }
}
