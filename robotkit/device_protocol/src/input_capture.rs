//! Bounded switch observations captured in the device step-clock domain.
use crate::Board;

pub const MAX_DEVICE_INPUTS: usize = 64;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InputBinding {
    pub actuator: usize,
    pub active_high: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct InputObservation {
    pub active: bool,
    pub closing_count: u64,
    pub opening_count: u64,
    pub captured_steps: i64,
    pub captured_ticks: u64,
}

pub struct InputCapture {
    bindings: [Option<InputBinding>; MAX_DEVICE_INPUTS],
    observations: [InputObservation; MAX_DEVICE_INPUTS],
}

impl Default for InputCapture {
    fn default() -> Self { Self::new() }
}

impl InputCapture {
    pub const fn new() -> Self {
        Self {
            bindings: [None; MAX_DEVICE_INPUTS],
            observations: [InputObservation { active: false, closing_count: 0,
                opening_count: 0, captured_steps: 0, captured_ticks: 0 }; MAX_DEVICE_INPUTS],
        }
    }

    /// Seed the present level without inventing an edge when a session opens.
    pub fn bind<B: Board>(&mut self, board: &B, channel: usize,
        binding: InputBinding, actuator_count: usize) -> bool {
        if channel >= MAX_DEVICE_INPUTS || binding.actuator >= actuator_count {
            return false;
        }
        self.bindings[channel] = Some(binding);
        self.observations[channel] = InputObservation {
            active: board.read_input(channel) == binding.active_high,
            ..InputObservation::default()
        };
        true
    }

    pub fn observation(&self, channel: usize) -> Option<InputObservation> {
        self.bindings.get(channel)?.as_ref()?;
        Some(self.observations[channel])
    }

    /// Sample external edges before stepping, or switches associated with a pulse just emitted.
    /// False indicates exhausted edge counters; the owner must fault rather than reuse an edge.
    pub fn sample<B: Board>(&mut self, board: &B, actuator: Option<usize>) -> bool {
        for channel in 0..MAX_DEVICE_INPUTS {
            let Some(binding) = self.bindings[channel] else { continue; };
            if actuator.is_some() && actuator != Some(binding.actuator) { continue; }
            let active = board.read_input(channel) == binding.active_high;
            let observation = &mut self.observations[channel];
            if active == observation.active { continue; }
            if active {
                let Some(count) = observation.closing_count.checked_add(1) else { return false; };
                observation.closing_count = count;
                observation.captured_steps = board.step_count(binding.actuator);
                observation.captured_ticks = board.now_ticks();
            } else {
                let Some(count) = observation.opening_count.checked_add(1) else { return false; };
                observation.opening_count = count;
            }
            observation.active = active;
        }
        true
    }
}
