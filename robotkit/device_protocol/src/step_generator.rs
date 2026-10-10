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
    counter_origin_steps: [f64; A],
    nominal_steps: [f64; A],
    steps_per_unit: [f64; A],
    setup_ticks: [u64; A],
    /// Least step ticks between two steps of each actuator.
    min_step_ticks: [u64; A],
    direction: [Option<bool>; A],
    direction_since: [u64; A],
    /// Step ticks so far: one per `tick` call.
    step_tick: u64,
    /// The step tick of each actuator's last step.
    last_step: [Option<u64>; A],
    skew: [Option<SkewGroup>; A],
    skew_count: usize,
    squaring_bound: [Option<f64>; A],
}

impl<const A: usize> StepGenerator<A> {
    /// `min_step_ticks` is each actuator's least whole number of step ticks between steps, as the
    /// host chose it (at least one); the device enforces it and never derives it from a rate. Each
    /// `tick` call is one step tick; `setup_ticks` are board clock ticks.
    pub fn new(steps_per_unit: [f64; A], setup_ticks: [u64; A],
        min_step_ticks: [u64; A]) -> Option<Self> {
        if A == 0 || A > 64 ||
            steps_per_unit.iter().any(|v| !v.is_finite() || *v <= 0.0) ||
            min_step_ticks.iter().any(|v| *v == 0) { return None; }
        Some(Self { inputs: InputCapture::new(), homing_pair: None, held: [None; A],
            alignment_steps: [0.0; A], counter_origin_steps: [0.0; A], nominal_steps: [0.0; A], steps_per_unit, setup_ticks, min_step_ticks, step_tick: 0,
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
        self.alignment_steps[actuator] = physical as f64 - self.nominal_steps[actuator] - self.counter_origin_steps[actuator];
        true
    }

    pub fn release_homing_side(&mut self, actuator: usize) -> bool {
        let Some((first, second)) = self.homing_pair else { return false; };
        if (actuator != first && actuator != second) || self.held[actuator].is_none() { return false; }
        self.held[actuator] = None;
        true
    }

    /// Shift independent counter origins as one transaction. The device owner must
    /// require drained, stationary motion before calling this method.
    pub fn rebase_homing_counters(&mut self, deltas: &[(usize, f64)]) -> bool {
        let Some((first, second)) = self.homing_pair else { return false; };
        if deltas.len() != 2 || deltas[0].0 == deltas[1].0 || self.held.iter().any(Option::is_some) {
            return false;
        }
        for &(actuator, delta) in deltas {
            if actuator != first && actuator != second { return false; }
            let shift = delta * self.steps_per_unit[actuator];
            let origin = self.counter_origin_steps[actuator] + shift;
            let alignment = self.alignment_steps[actuator] - shift;
            if !shift.is_finite() || !origin.is_finite() || !alignment.is_finite() ||
                origin.abs() > 9_007_199_254_740_991.0 || alignment.abs() > 9_007_199_254_740_991.0 {
                return false;
            }
        }
        for &(actuator, delta) in deltas {
            let shift = delta * self.steps_per_unit[actuator];
            self.counter_origin_steps[actuator] += shift;
            self.alignment_steps[actuator] -= shift;
        }
        true
    }

    pub fn counter_position<B: Board>(&self, board: &B, actuator: usize) -> Option<f64> {
        if actuator >= A { return None; }
        Some((board.step_count(actuator) as f64 - self.counter_origin_steps[actuator]) / self.steps_per_unit[actuator])
    }

    /// Fit stationary physical counters to the deployment's independent axes.
    /// Individual motor measurements remain separate; paired queue coordinates
    /// share their leader, as they do in host feedback reconstruction.
    pub fn stopped_targets<B: Board>(&self, board: &B, joints: &[u8; A],
        ratios: &[f32; A], count: usize) -> Option<[f32; A]> {
        if count > A { return None; }
        let mut positions = [0.0; A];
        for a in 0..count {
            let ratio = ratios[a] as f64;
            if !ratio.is_finite() || ratio == 0.0 { return None; }
            let mut numerator = 0.0;
            let mut denominator = 0.0;
            for b in 0..count {
                if joints[b] != joints[a] { continue; }
                let r = ratios[b] as f64;
                if !r.is_finite() || r == 0.0 { return None; }
                numerator += r * self.counter_position(board, b)?;
                denominator += r * r;
            }
            positions[a] = (ratio * numerator / denominator) as f32;
        }
        Some(positions)
    }

    /// Adopt a new stationary queue frame without issuing a physical step.
    /// The owner must validate rest and the queue's expected state first.
    pub fn anchor_stopped_targets<B: Board>(&mut self, board: &B, targets: [f32; A]) -> bool {
        if targets.iter().any(|v| !v.is_finite()) { return false; }
        for a in 0..A {
            self.nominal_steps[a] = targets[a] as f64 * self.steps_per_unit[a];
            self.alignment_steps[a] = board.step_count(a) as f64 -
                self.counter_origin_steps[a] - self.nominal_steps[a];
        }
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
        self.step_tick += 1;
        for a in 0..count {
            self.nominal_steps[a] = targets[a] as f64 * self.steps_per_unit[a];
            if let Some(physical) = self.held[a] {
                self.alignment_steps[a] = physical as f64 - self.nominal_steps[a] - self.counter_origin_steps[a];
                continue;
            }
            let raw = self.nominal_steps[a] + self.counter_origin_steps[a] + self.alignment_steps[a];
            // A captured integer counter converted to f32 can fall just below
            // its own step boundary. Preserve that boundary when the excess
            // is only representation error, without rounding ordinary targets.
            let nearest = (raw + if raw >= 0.0 { 0.5 } else { -0.5 }) as i64 as f64;
            let boundary_error = (f32::EPSILON as f64 * raw.abs().max(1.0)).min(1e-4);
            let raw = if (raw - nearest).abs() <= boundary_error { nearest } else { raw };
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
                if self.step_tick - last < self.min_step_ticks[a] { continue; }
            }
            board.step_pulse(a, forward);
            if !self.inputs.sample(board, Some(a)) { self.end_homing_pair(); return Err(StepFault::InputCounterOverflow); }
            self.last_step[a] = Some(self.step_tick);
        }
        for (i, configured) in self.skew[..self.skew_count].iter().enumerate() {
            let Some(group) = configured else { continue; };
            if group.first >= count || group.second >= count { continue; }
            let first = (board.step_count(group.first) as f64 - self.counter_origin_steps[group.first]) /
                self.steps_per_unit[group.first] / group.first_ratio;
            let second = (board.step_count(group.second) as f64 - self.counter_origin_steps[group.second]) /
                self.steps_per_unit[group.second] / group.second_ratio;
            let bound = self.squaring_bound[i].unwrap_or(group.bound);
            if (first - second).abs() > bound { self.end_homing_pair(); return Err(StepFault::DualDriveSkew); }
        }
        Ok(())
    }
}
