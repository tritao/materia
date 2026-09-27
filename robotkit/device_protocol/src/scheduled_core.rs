//! Fixed-capacity RKD6 scheduled segment execution.
use crate::Board;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QueueError { StaleRevision, Committed, BadBoundary, BadExpectedState, Full, InvalidSegment, InvalidCommit }
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StopReason { Underflow, LinkLost, Abort, Stop, EmergencyStop }

#[derive(Clone, Copy, Debug)]
pub struct ScheduledSegment<const A: usize> {
    pub plan_id: u64,
    pub t0_ticks: u64,
    pub duration_ticks: u64,
    pub degree: u8,
    pub coefficients: [[f32; 6]; A],
    pub ends_at_rest: bool,
}

impl<const A: usize> ScheduledSegment<A> {
    pub fn new(plan_id: u64, t0_ticks: u64, duration_ticks: u64, degree: u8,
               coefficients: [[f32; 6]; A], ends_at_rest: bool) -> Result<Self, QueueError> {
        if A == 0 || A > 64 || plan_id == 0 || duration_ticks == 0 || degree > 5 ||
           !coefficients.iter().flat_map(|v| v.iter()).all(|v| v.is_finite()) {
            return Err(QueueError::InvalidSegment);
        }
        t0_ticks.checked_add(duration_ticks).ok_or(QueueError::InvalidSegment)?;
        Ok(Self { plan_id, t0_ticks, duration_ticks, degree, coefficients, ends_at_rest })
    }
    pub fn end_ticks(&self) -> u64 { self.t0_ticks + self.duration_ticks }
    pub fn evaluate(&self, path_ticks: u64, tick_hz: u64) -> ([f32; A], [f32; A]) {
        let elapsed = path_ticks.saturating_sub(self.t0_ticks).min(self.duration_ticks);
        let tau = elapsed as f32 / tick_hz as f32;
        let mut position = [0.0; A];
        let mut velocity = [0.0; A];
        for a in 0..A {
            let c = &self.coefficients[a];
            let mut q = c[self.degree as usize];
            for k in (0..self.degree as usize).rev() { q = q * tau + c[k]; }
            position[a] = q;
            if self.degree != 0 {
                let mut v = self.degree as f32 * c[self.degree as usize];
                for k in (1..self.degree as usize).rev() { v = v * tau + k as f32 * c[k]; }
                velocity[a] = v;
            }
        }
        (position, velocity)
    }
    pub fn acceleration(&self, path_ticks: u64, tick_hz: u64) -> [f32; A] {
        let elapsed = path_ticks.saturating_sub(self.t0_ticks).min(self.duration_ticks);
        let tau = elapsed as f32 / tick_hz as f32;
        let mut acceleration = [0.0; A];
        if self.degree >= 2 {
            for a in 0..A {
                let c = &self.coefficients[a];
                let d = self.degree as usize;
                let mut value = (d * (d - 1)) as f32 * c[d];
                for k in (2..d).rev() { value = value * tau + (k * (k - 1)) as f32 * c[k]; }
                acceleration[a] = value;
            }
        }
        acceleration
    }
}

pub struct ScheduledCore<const A: usize, const CAP: usize> {
    tick_hz: u64,
    max_acceleration: [f32; A],
    lower: [f32; A],
    upper: [f32; A],
    link_loss_ticks: u64,
    segments: [Option<ScheduledSegment<A>>; CAP],
    len: usize,
    revision: u64,
    replace_after: u64,
    expected_start_position: [f32; A],
    committed_until: u64,
    path_clock: u64,
    path_fraction: f32,
    rate: f32,
    target_rate: f32,
    last_device_tick: u64,
    last_frame_tick: u64,
    position: [f32; A],
    velocity: [f32; A],
    stopping: Option<StopReason>,
    stopped: bool,
    underflow: bool,
    abort_at_end: bool,
}

impl<const A: usize, const CAP: usize> ScheduledCore<A, CAP> {
    pub fn new(tick_hz: u64, max_acceleration: [f32; A], lower: [f32; A], upper: [f32; A],
               link_loss_ticks: u64) -> Self {
        assert!(A > 0 && A <= 64 && CAP > 0 && tick_hz > 0 &&
                max_acceleration.iter().all(|a| a.is_finite() && *a > 0.0) &&
                lower.iter().zip(upper.iter()).all(|(lo, hi)| lo.is_finite() && hi.is_finite() && lo < hi));
        Self { tick_hz, max_acceleration, lower, upper, link_loss_ticks,
            segments: [None; CAP], len: 0, revision: 0, replace_after: 0,
            expected_start_position: [0.0; A], committed_until: 0,
            path_clock: 0, path_fraction: 0.0, rate: 1.0, target_rate: 1.0,
            last_device_tick: 0, last_frame_tick: 0, position: [0.0; A], velocity: [0.0; A],
            stopping: None, stopped: false, underflow: false, abort_at_end: false }
    }
    pub fn revision(&self) -> u64 { self.revision }
    pub fn committed_until(&self) -> u64 { self.committed_until }
    pub fn path_clock(&self) -> u64 { self.path_clock }
    pub fn rate(&self) -> f32 { self.rate }
    pub fn underflow(&self) -> bool { self.underflow }
    pub fn stop_reason(&self) -> Option<StopReason> { self.stopping }
    pub fn remaining_capacity(&self) -> usize { CAP - self.len }
    fn committed_len(&self) -> usize {
        self.segments[..self.len].iter().take_while(|s|
            s.unwrap().end_ticks() <= self.committed_until).count()
    }
    pub fn note_host_frame(&mut self, now_ticks: u64) { self.last_frame_tick = now_ticks; }
    pub fn queue_begin(&mut self, revision: u64, replace_after: u64) -> Result<(), QueueError> {
        if revision <= self.revision { return Err(QueueError::StaleRevision); }
        if replace_after < self.committed_until { return Err(QueueError::Committed); }
        if replace_after < self.path_clock { return Err(QueueError::BadBoundary); }
        let mut keep = 0;
        while keep < self.len && self.segments[keep].unwrap().end_ticks() <= replace_after { keep += 1; }
        if keep < self.len && self.segments[keep].unwrap().t0_ticks != replace_after {
            return Err(QueueError::BadBoundary);
        }
        self.expected_start_position = if keep == 0 { self.position } else {
            let previous = self.segments[keep - 1].unwrap();
            previous.evaluate(previous.end_ticks(), self.tick_hz).0
        };
        for slot in &mut self.segments[keep..] { *slot = None; }
        self.len = keep;
        self.revision = revision;
        self.replace_after = replace_after;
        Ok(())
    }
    pub fn queue_begin_with_state(&mut self, revision: u64, replace_after: u64,
                                  expected_position: [f32; A], expected_velocity: [f32; A])
                                  -> Result<(), QueueError> {
        if !expected_position.iter().chain(expected_velocity.iter()).all(|v| v.is_finite()) {
            return Err(QueueError::BadExpectedState);
        }
        let mut anchor = (self.position, self.velocity);
        for segment in self.segments[..self.len].iter().flatten() {
            if segment.end_ticks() == replace_after {
                anchor = segment.evaluate(replace_after, self.tick_hz);
                if segment.ends_at_rest { anchor.1.fill(0.0); }
                break;
            }
        }
        if (0..A).any(|a| (expected_position[a] - anchor.0[a]).abs() > 1e-4 ||
                           (expected_velocity[a] - anchor.1[a]).abs() > 1e-4) {
            return Err(QueueError::BadExpectedState);
        }
        self.queue_begin(revision, replace_after)
    }
    pub fn push_segment(&mut self, segment: ScheduledSegment<A>) -> Result<(), QueueError> {
        if self.len == CAP { return Err(QueueError::Full); }
        if self.len == 0 {
            if segment.t0_ticks != self.replace_after || segment.t0_ticks < self.path_clock {
                return Err(QueueError::BadBoundary);
            }
            let (start_position, _) = segment.evaluate(segment.t0_ticks, self.tick_hz);
            if (0..A).any(|a| (start_position[a] - self.expected_start_position[a]).abs() > 1e-4) {
                return Err(QueueError::BadBoundary);
            }
        } else {
            let previous = self.segments[self.len - 1].unwrap();
            if segment.t0_ticks != previous.end_ticks() { return Err(QueueError::BadBoundary); }
            let (end_position, _) = previous.evaluate(previous.end_ticks(), self.tick_hz);
            let (start_position, _) = segment.evaluate(segment.t0_ticks, self.tick_hz);
            if (0..A).any(|a| (end_position[a] - start_position[a]).abs() > 1e-4) {
                return Err(QueueError::BadBoundary);
            }
        }
        self.segments[self.len] = Some(segment);
        self.len += 1;
        Ok(())
    }
    pub fn commit(&mut self, through_ticks: u64) -> Result<(), QueueError> {
        if through_ticks < self.committed_until || self.len == 0 ||
           through_ticks > self.segments[self.len - 1].unwrap().end_ticks() ||
           !self.segments[..self.len].iter().any(|s| s.unwrap().end_ticks() == through_ticks) {
            return Err(QueueError::InvalidCommit);
        }
        self.committed_until = through_ticks;
        Ok(())
    }
    pub fn hold(&mut self) { self.target_rate = 0.0; }
    pub fn resume(&mut self) { if self.stopping.is_none() { self.target_rate = 1.0; } }
    pub fn abort(&mut self) {
        let committed_len = self.committed_len();
        if committed_len != 0 {
            let final_segment = self.segments[committed_len - 1].unwrap();
            let remaining = final_segment.end_ticks().saturating_sub(self.path_clock) as f32 /
                self.tick_hz as f32;
            let braking = self.velocity.iter().enumerate().fold(0.0f32, |t, (a, v)|
                t.max(v.abs() / self.max_acceleration[a]));
            if final_segment.ends_at_rest && remaining <= braking {
                self.abort_at_end = true;
                return;
            }
        }
        self.stop(StopReason::Abort);
    }
    pub fn stop(&mut self, reason: StopReason) {
        self.stopping = Some(reason);
        self.target_rate = 0.0;
        self.abort_at_end = false;
        self.len = 0;
        self.segments.fill(None);
    }
    pub fn emergency_stop<B: Board>(&mut self, board: &mut B) {
        self.stop(StopReason::EmergencyStop);
        self.velocity.fill(0.0);
        board.stop_all();
        self.stopped = true;
    }
    pub fn tick<B: Board>(&mut self, board: &mut B) {
        if self.stopped { return; }
        let now = board.now_ticks();
        if now < self.last_device_tick { self.stop(StopReason::LinkLost); }
        let dt_ticks = now.saturating_sub(self.last_device_tick);
        self.last_device_tick = now;
        let dt = dt_ticks as f32 / self.tick_hz as f32;
        let committed_len = self.committed_len();
        let active_motion = committed_len > 0 && {
            let last = self.segments[committed_len - 1].unwrap();
            self.path_clock < last.end_ticks() || !last.ends_at_rest
        };
        if self.stopping.is_none() && active_motion &&
           now.saturating_sub(self.last_frame_tick) > self.link_loss_ticks {
            self.stop(StopReason::LinkLost);
        }
        if self.stopping.is_some() {
            let mut moving = false;
            for a in 0..A {
                let old = self.velocity[a];
                let decel = self.max_acceleration[a] * dt;
                let new = if old > 0.0 { (old - decel).max(0.0) } else { (old + decel).min(0.0) };
                let proposed = self.position[a] + (old + new) * 0.5 * dt;
                self.position[a] = proposed.clamp(self.lower[a], self.upper[a]);
                self.velocity[a] = if proposed != self.position[a] { 0.0 } else { new };
                board.position_target(a, self.position[a]);
                board.velocity_target(a, self.velocity[a]);
                moving |= self.velocity[a] != 0.0;
            }
            if !moving && !self.stopped { board.stop_all(); self.stopped = true; }
            return;
        }
        if self.committed_len() == 0 {
            self.path_clock = self.path_clock.saturating_add(dt_ticks);
            return;
        }
        let mut rate_step = 1.0f32;
        for segment in self.segments[..self.committed_len()].iter().flatten() {
            if self.path_clock <= segment.end_ticks() {
                let (_, path_velocity) = segment.evaluate(self.path_clock, self.tick_hz);
                let path_acceleration = segment.acceleration(self.path_clock, self.tick_hz);
                for a in 0..A {
                    let margin = (self.max_acceleration[a] -
                        path_acceleration[a].abs() * self.rate * self.rate).max(0.0);
                    rate_step = rate_step.min(0.9 * margin / path_velocity[a].abs().max(1e-6) * dt);
                }
                break;
            }
        }
                if self.rate < self.target_rate { self.rate = (self.rate + rate_step).min(self.target_rate); }
        else { self.rate = if self.rate - self.target_rate <= rate_step + 1e-5 {
            self.target_rate
        } else { self.rate - rate_step }; }
        let advance = dt_ticks as f32 * self.rate + self.path_fraction;
        let whole = advance as u64;
        self.path_fraction = advance - whole as f32;
        self.path_clock = self.path_clock.saturating_add(whole);
        let mut current = None;
        let committed_len = self.committed_len();
        for segment in self.segments[..committed_len].iter().flatten() {
            if self.path_clock < segment.t0_ticks {
                let (position, _) = segment.evaluate(segment.t0_ticks, self.tick_hz);
                self.position = position;
                self.velocity.fill(0.0);
                for a in 0..A { board.position_target(a, position[a]); board.velocity_target(a, 0.0); }
                return;
            }
            if self.path_clock <= segment.end_ticks() { current = Some(*segment); break; }
        }
        if let Some(segment) = current {
            let (position, velocity) = segment.evaluate(self.path_clock, self.tick_hz);
            for a in 0..A {
                self.position[a] = position[a].clamp(self.lower[a], self.upper[a]);
                self.velocity[a] = velocity[a] * self.rate;
                board.position_target(a, self.position[a]);
                board.velocity_target(a, self.velocity[a]);
            }
            if self.path_clock == segment.end_ticks() && segment.ends_at_rest {
                self.velocity.fill(0.0);
                for a in 0..A { board.velocity_target(a, 0.0); }
            }
            if self.abort_at_end && self.path_clock >= self.segments[committed_len - 1].unwrap().end_ticks() {
                self.stop(StopReason::Abort);
                board.stop_all();
                self.stopped = true;
            }
        } else if committed_len != 0 {
            let final_segment = self.segments[committed_len - 1].unwrap();
            if final_segment.ends_at_rest {
                let (position, _) = final_segment.evaluate(final_segment.end_ticks(), self.tick_hz);
                self.position = position;
                self.velocity.fill(0.0);
                for a in 0..A { board.position_target(a, position[a]); board.velocity_target(a, 0.0); }
                if self.abort_at_end {
                    self.stop(StopReason::Abort);
                    board.stop_all();
                    self.stopped = true;
                }
            } else {
                self.underflow = true;
                self.stop(StopReason::Underflow);
            }
        }
    }
}
