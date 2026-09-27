//! Hardware boundary for scheduled execution. Time is monotonic device ticks.

pub trait Board {
    fn now_ticks(&self) -> u64;
    fn tick_hz(&self) -> u64;
    fn position_target(&mut self, actuator: usize, position: f32);
    fn velocity_target(&mut self, actuator: usize, velocity: f32);
    fn step_pulse(&mut self, actuator: usize, forward: bool);
    fn step_count(&self, actuator: usize) -> i64;
    fn set_direction(&mut self, _actuator: usize, _forward: bool) {}
    fn set_digital(&mut self, channel: usize, value: bool);
    fn set_analog(&mut self, channel: usize, value: f32);
    fn stop_all(&mut self);
}
