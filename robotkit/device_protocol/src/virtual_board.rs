use crate::Board;
use std::vec::Vec;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Output {
    Position(usize, f32), Velocity(usize, f32), Direction(usize, bool), Step(usize, bool),
    Digital(usize, bool), Analog(usize, f32), Stop,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OutputRecord { pub ticks: u64, pub output: Output }

/// Deterministic board model; `advance_host_ns` is driven by the owner clock.
pub struct VirtualBoard<const ACTUATORS: usize, const CHANNELS: usize> {
    tick_hz: u64,
    offset_ticks: u64,
    drift_ppm: i32,
    host_ns: u64,
    ticks: u64,
    steps_per_unit: [f64; ACTUATORS],
    steps: [i64; ACTUATORS],
    missed_steps: [u32; ACTUATORS],
    targets: [f32; ACTUATORS],
    velocities: [f32; ACTUATORS],
    digital: [bool; CHANNELS],
    analog: [f32; CHANNELS],
    records: Vec<OutputRecord>,
}

impl<const A: usize, const C: usize> VirtualBoard<A, C> {
    pub fn new(tick_hz: u64, offset_ticks: u64, drift_ppm: i32,
               steps_per_unit: [f64; A]) -> Self {
        assert!(tick_hz > 0 && drift_ppm > -1_000_000);
        assert!(steps_per_unit.iter().all(|v| v.is_finite() && *v > 0.0));
        Self { tick_hz, offset_ticks, drift_ppm, host_ns: 0, ticks: offset_ticks,
            steps_per_unit, steps: [0; A], missed_steps: [0; A],
            targets: [0.0; A], velocities: [0.0; A],
            digital: [false; C], analog: [0.0; C], records: Vec::new() }
    }
    pub fn advance_host_ns(&mut self, host_ns: u64) {
        assert!(host_ns >= self.host_ns);
        self.host_ns = host_ns;
        let scaled = u128::from(host_ns) * u128::from(self.tick_hz) *
            (1_000_000i64 + i64::from(self.drift_ppm)) as u128;
        self.ticks = self.offset_ticks.saturating_add((scaled / 1_000_000_000_000_000) as u64);
    }
    pub fn step_counts(&self) -> [i64; A] { self.steps }
    pub fn miss_next_steps(&mut self, actuator: usize, count: u32) -> bool {
        if actuator >= A { return false; }
        self.missed_steps[actuator] = count;
        true
    }
    pub fn actuator_positions(&self) -> [f64; A] {
        std::array::from_fn(|i| self.steps[i] as f64 / self.steps_per_unit[i])
    }
    pub fn position_targets(&self) -> [f32; A] { self.targets }
    pub fn velocity_targets(&self) -> [f32; A] { self.velocities }
    pub fn steps_per_unit(&self) -> [f64; A] { self.steps_per_unit }
    pub fn records(&self) -> &[OutputRecord] { &self.records }
    pub fn digital(&self, i: usize) -> Option<bool> { self.digital.get(i).copied() }
    pub fn analog(&self, i: usize) -> Option<f32> { self.analog.get(i).copied() }
    fn record(&mut self, output: Output) { self.records.push(OutputRecord { ticks: self.ticks, output }); }
}

impl<const A: usize, const C: usize> Board for VirtualBoard<A, C> {
    fn now_ticks(&self) -> u64 { self.ticks }
    fn tick_hz(&self) -> u64 { self.tick_hz }
    fn position_target(&mut self, i: usize, value: f32) {
        self.targets[i] = value; self.record(Output::Position(i, value));
    }
    fn velocity_target(&mut self, i: usize, value: f32) {
        self.velocities[i] = value; self.record(Output::Velocity(i, value));
    }
    fn step_pulse(&mut self, i: usize, forward: bool) {
        self.record(Output::Step(i, forward));
        if self.missed_steps[i] > 0 {
            self.missed_steps[i] -= 1;
        } else {
            self.steps[i] += if forward { 1 } else { -1 };
        }
    }
    fn step_count(&self, i: usize) -> i64 { self.steps[i] }
    fn set_direction(&mut self, i: usize, forward: bool) {
        self.record(Output::Direction(i, forward));
    }
    fn set_digital(&mut self, i: usize, value: bool) {
        self.digital[i] = value; self.record(Output::Digital(i, value));
    }
    fn set_analog(&mut self, i: usize, value: f32) {
        self.analog[i] = value; self.record(Output::Analog(i, value));
    }
    fn stop_all(&mut self) {
        self.velocities.fill(0.0);
        self.digital.fill(false);
        self.analog.fill(0.0);
        self.record(Output::Stop);
    }
}
