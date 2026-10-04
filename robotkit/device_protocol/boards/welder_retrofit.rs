//! Retrofit supply I/O: digital optoMOS trigger, isolated 0–10 V outputs and calibrated sensing.
use crate::weld_contract as contract;

#[derive(Clone, Copy)]
pub struct RetrofitConfig {
    pub wire_at_10v: f32,
    pub voltage_at_10v: f32,
    pub current_a_per_v: f32,
    pub current_zero_v: f32,
    pub arc_v_per_v: f32,
    pub arc_threshold_a: f32,
    pub no_arc_seconds: f64,
    pub short_voltage_v: f32,
    pub short_seconds: f64,
    pub efficiency: f32,
}
impl RetrofitConfig {
    pub fn valid(&self) -> bool {
        [self.wire_at_10v, self.voltage_at_10v, self.current_a_per_v, self.arc_v_per_v,
            self.arc_threshold_a, self.short_voltage_v, self.efficiency].iter()
            .all(|v| v.is_finite() && *v > 0.0)
            && self.efficiency <= 1.0 && self.current_zero_v.is_finite()
            && self.no_arc_seconds.is_finite() && self.no_arc_seconds > 0.0
            && self.short_seconds.is_finite() && self.short_seconds > 0.0
    }
}

/// Analog input values are volts at the ADC, after isolation; output values are external 0–10 V.
pub trait RetrofitIo {
    fn trigger(&mut self, on: bool);
    fn wire_output_v(&mut self, volts: f32);
    fn voltage_output_v(&mut self, volts: f32);
    fn current_input_v(&self) -> f32;
    fn arc_voltage_input_v(&self) -> f32;
    fn touch_input(&self) -> bool;
}

pub struct RetrofitWelder {
    pub config: RetrofitConfig,
    no_arc_elapsed: f64,
    short_elapsed: f64,
    fault: u8,
}
impl RetrofitWelder {
    pub fn new(config: RetrofitConfig) -> Option<Self> {
        if !config.valid() { return None; }
        Some(Self { config, no_arc_elapsed: 0.0, short_elapsed: 0.0, fault: 0 })
    }
    pub fn safe<I: RetrofitIo>(&self, io: &mut I) {
        io.wire_output_v(0.0); io.trigger(false); io.voltage_output_v(0.0);
    }
    /// Reset only while the scheduler holds safe outputs; a fault cannot auto-reignite.
    pub fn reset<I: RetrofitIo>(&mut self, io: &mut I) {
        self.safe(io); self.fault = 0; self.no_arc_elapsed = 0.0; self.short_elapsed = 0.0;
    }
    pub fn tick<I: RetrofitIo>(&mut self, io: &mut I, dt: f64, arc: bool, wire: f32, volts: f32) -> [f32; contract::SENSOR_COUNT] {
        let c = self.config;
        let current_input = io.current_input_v(); let voltage_input = io.arc_voltage_input_v();
        if !dt.is_finite() || dt < 0.0 || !wire.is_finite() || wire < 0.0 ||
            !volts.is_finite() || volts < 0.0 || !current_input.is_finite() || !voltage_input.is_finite() {
            self.fault = 3;
        }
        let current = if current_input.is_finite() { ((current_input - c.current_zero_v) * c.current_a_per_v).max(0.0) } else { 0.0 };
        let voltage = if voltage_input.is_finite() { (voltage_input * c.arc_v_per_v).max(0.0) } else { 0.0 };
        let established = current >= c.arc_threshold_a;
        if arc && self.fault == 0 {
            self.no_arc_elapsed = if established { 0.0 } else { self.no_arc_elapsed + dt };
            self.short_elapsed = if established && voltage <= c.short_voltage_v { self.short_elapsed + dt } else { 0.0 };
            if self.no_arc_elapsed >= c.no_arc_seconds { self.fault = 1; }
            // MIG shorts are normal: only a sustained low-voltage/high-current condition faults.
            if self.short_elapsed >= c.short_seconds { self.fault = 3; }
        } else if !arc { self.no_arc_elapsed = 0.0; self.short_elapsed = 0.0; }
        if self.fault != 0 || !arc { self.safe(io); }
        else {
            io.voltage_output_v((volts / c.voltage_at_10v * 10.0).min(10.0));
            io.wire_output_v((wire / c.wire_at_10v * 10.0).min(10.0));
            io.trigger(true);
        }
        let mut values = [0.0; contract::SENSOR_COUNT];
        values[contract::ARC] = established as u8 as f32;
        values[contract::CURRENT] = current;
        values[contract::VOLTAGE] = voltage;
        values[contract::TOUCH] = io.touch_input() as u8 as f32;
        values[contract::FAULT] = self.fault as f32;
        values[contract::POWER] = current * voltage / c.efficiency;
        values
    }
}
