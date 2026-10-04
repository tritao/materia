//! Device-side arc simulation. The scheduler owns output safety; this model observes its outputs.
use robotkit_device_protocol::weld_contract as contract;

#[derive(Clone, Copy, Debug)]
pub struct WelderConfig {
    pub sensor_slot: u8,
    pub arc_channel: usize,
    pub wire_channel: usize,
    pub voltage_channel: usize,
    pub ignition_seconds: f64,
    pub no_arc_seconds: f64,
    pub efficiency: f32,
}
impl WelderConfig {
    pub fn valid(&self) -> bool {
        self.sensor_slot < 8 && self.arc_channel < 32 && self.wire_channel < 32 && self.voltage_channel < 32
            && self.arc_channel != self.wire_channel && self.arc_channel != self.voltage_channel
            && self.wire_channel != self.voltage_channel
            && self.ignition_seconds.is_finite() && self.ignition_seconds >= 0.0
            && self.no_arc_seconds.is_finite() && self.no_arc_seconds > self.ignition_seconds
            && self.efficiency.is_finite() && self.efficiency > 0.0 && self.efficiency <= 1.0
    }
}

pub struct VirtualWelder {
    pub config: WelderConfig,
    pub grounded: bool,
    elapsed: f64,
    established: bool,
    fault: u8,
    values: [f32; contract::SENSOR_COUNT],
}
impl VirtualWelder {
    pub fn new(config: WelderConfig) -> Option<Self> {
        config.valid().then_some(Self { config, grounded: false, elapsed: 0.0,
            established: false, fault: 0, values: [0.0; contract::SENSOR_COUNT] })
    }
    pub fn values(&self) -> [f32; contract::SENSOR_COUNT] { self.values }
    pub fn faulted(&self) -> bool { self.fault != 0 }
    pub fn reset(&mut self) { self.elapsed = 0.0; self.established = false; self.fault = 0; self.values = [0.0; contract::SENSOR_COUNT]; }

    pub fn tick(&mut self, dt: f64, arc: bool, wire_m_per_min: f32, voltage_v: f32) {
        if !arc {
            self.elapsed = 0.0;
            self.established = false;
        } else if self.fault == 0 {
            self.elapsed += dt;
            if self.established && !self.grounded {
                self.fault = 2;
                self.established = false;
            } else if self.grounded && wire_m_per_min > 0.0 && voltage_v > 0.0
                && self.elapsed >= self.config.ignition_seconds {
                self.established = true;
            } else if !self.established && self.elapsed >= self.config.no_arc_seconds {
                self.fault = 1;
            }
        }
        let current = if self.established { (wire_m_per_min * 30.0).min(350.0) } else { 0.0 };
        let voltage = if self.established { voltage_v } else { 0.0 };
        self.values[contract::ARC] = self.established as u8 as f32;
        self.values[contract::CURRENT] = current;
        self.values[contract::VOLTAGE] = voltage;
        self.values[contract::TOUCH] = (!arc && self.grounded) as u8 as f32;
        self.values[contract::FAULT] = self.fault as f32;
        self.values[contract::POWER] = current * voltage / self.config.efficiency;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn welder() -> VirtualWelder {
        VirtualWelder::new(WelderConfig { sensor_slot: 0, arc_channel: 0, wire_channel: 1, voltage_channel: 2,
            ignition_seconds: 0.02, no_arc_seconds: 0.2, efficiency: 0.9 }).unwrap()
    }
    #[test]
    fn established_arc_follows_safe_outputs() {
        let mut w = welder(); w.grounded = true;
        w.tick(0.01, true, 8.0, 24.0); assert_eq!(w.values()[0], 0.0);
        w.tick(0.01, true, 8.0, 24.0); assert_eq!(w.values()[0], 1.0);
        assert_eq!(w.values()[1], 240.0); assert!(w.values()[5] > 5760.0);
        w.tick(0.01, false, 0.0, 24.0);
        assert_eq!(w.values(), [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]);
    }
    #[test]
    fn fault_latches_until_reset() {
        let mut w = welder(); w.tick(0.2, true, 8.0, 24.0);
        assert_eq!(w.values()[4], 1.0);
        w.grounded = true; w.tick(0.1, true, 8.0, 24.0); assert_eq!(w.values()[0], 0.0);
        w.reset(); w.tick(0.02, true, 8.0, 24.0); assert_eq!(w.values()[0], 1.0);
        w.grounded = false; w.tick(0.01, true, 8.0, 24.0);
        assert_eq!(w.values()[4], 2.0); assert_eq!(w.values()[0], 0.0);
    }
    #[test]
    fn invalid_profile_rejected() {
        let mut c = welder().config; c.wire_channel = c.arc_channel;
        assert!(VirtualWelder::new(c).is_none());
        c = welder().config; c.efficiency = 0.0; assert!(VirtualWelder::new(c).is_none());
    }
}
