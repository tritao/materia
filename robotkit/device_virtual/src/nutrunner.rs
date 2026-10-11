//! Hardware-in-loop nutrunner ADC. The CAD joint plant remains the canonical Haxe model.
//! Samples are atomic acquisitions, not values re-stamped by the transport publisher.
use robotkit_device_protocol::screwdriver_contract as contract;

#[derive(Clone, Copy, Debug)]
pub struct NutrunnerConfig {
    pub sensor_slot: u8,
    pub start_channel: usize,
    pub direction_channel: usize,
    pub program_channel: usize,
    pub vacuum_channel: usize,
    pub maximum_age_ns: u64,
}
impl NutrunnerConfig {
    pub fn valid(&self) -> bool {
        let channels = [self.start_channel, self.direction_channel, self.program_channel, self.vacuum_channel];
        self.sensor_slot < 8 && self.maximum_age_ns > 0 && channels.iter().all(|&i| i < 32)
            && (0..channels.len()).all(|i| (0..i).all(|j| channels[i] != channels[j]))
    }
}

pub struct VirtualNutrunner {
    pub config: NutrunnerConfig,
    pending: [f64; contract::SENSOR_COUNT],
    written: u8,
    pub sequence: u64,
    pub source_ticks: u64,
    acquired_ns: Option<u64>,
    pub sent_sequence: u64,
    values: [f32; contract::SENSOR_COUNT],
    fault: bool,
}
impl VirtualNutrunner {
    pub fn new(config: NutrunnerConfig) -> Option<Self> {
        config.valid().then_some(Self { config, pending: [0.0; contract::SENSOR_COUNT], written: 0,
            sequence: 0, source_ticks: 0, acquired_ns: None, sent_sequence: 0,
            values: [0.0; contract::SENSOR_COUNT], fault: false })
    }
    pub fn reset(&mut self) {
        self.pending = [0.0; contract::SENSOR_COUNT]; self.written = 0;
        self.sequence = 0; self.source_ticks = 0; self.acquired_ns = None; self.sent_sequence = 0;
        self.values = [0.0; contract::SENSOR_COUNT]; self.fault = false;
    }
    pub fn values(&self) -> [f32; contract::SENSOR_COUNT] { self.values }
    pub fn input(&mut self, input: usize, value: f64) -> bool {
        if self.fault || input >= contract::SENSOR_COUNT || !value.is_finite() || !(value as f32).is_finite() { return false; }
        self.pending[input] = value; self.written |= 1 << input; true
    }
    pub fn can_capture(&self, host_ns: u64) -> bool {
        !self.fault && self.written == (1 << contract::SENSOR_COUNT) - 1
            && self.acquired_ns.is_none_or(|previous| host_ns > previous)
            && self.pending[contract::TORQUE] >= 0.0 && self.pending[contract::ANGLE] >= 0.0
            && self.pending[contract::STROKE] >= 0.0
            && integral(self.pending[contract::PHASE], contract::COMPLETE)
            && integral(self.pending[contract::RESULT], contract::SCREW_LOST)
            && integral(self.pending[contract::VACUUM], 1)
    }
    pub fn capture(&mut self, host_ns: u64, ticks: u64) -> bool {
        if !self.can_capture(host_ns) || self.sequence == u64::MAX { return false; }
        self.values = self.pending.map(|v| v as f32); self.written = 0;
        self.sequence += 1; self.source_ticks = ticks; self.acquired_ns = Some(host_ns); true
    }
    /** Idle startup needs no drive measurement; active drive/reverse must have a live acquisition. */
    pub fn tick(&mut self, host_ns: u64, driving: bool) -> bool {
        if driving && self.acquired_ns.is_none_or(|time| host_ns.saturating_sub(time) >= self.config.maximum_age_ns) {
            self.fault = true;
        }
        self.fault
    }
}
fn integral(value: f64, maximum: usize) -> bool {
    value >= 0.0 && value <= maximum as f64 && value.fract() == 0.0
}

#[cfg(test)]
mod tests {
    use super::*;
    fn runner() -> VirtualNutrunner {
        VirtualNutrunner::new(NutrunnerConfig {sensor_slot:0,start_channel:0,direction_channel:1,
            program_channel:2,vacuum_channel:3,maximum_age_ns:50_000_000}).unwrap()
    }
    fn sample(n: &mut VirtualNutrunner) {
        for (i, value) in [16.5,12.5,0.0,5.0,5.0,1.0,1.0].iter().enumerate() { assert!(n.input(i,*value)); }
    }
    #[test]
    fn atomic_sample_retains_acquisition_clock_and_sequence() {
        let mut n = runner(); assert!(n.input(0,16.5)); assert!(!n.can_capture(0));
        sample(&mut n); assert!(n.capture(0,50_000));
        assert_eq!(n.sequence,1); assert_eq!(n.source_ticks,50_000); assert_eq!(n.values()[0],16.5);
        assert!(!n.capture(1,50_001)); assert!(!n.tick(1,false));
        assert_eq!(n.sequence,1); assert_eq!(n.source_ticks,50_000);
        sample(&mut n); assert!(!n.capture(0,50_000)); assert!(n.capture(1,50_001)); assert_eq!(n.sequence,2);
    }
    #[test]
    fn stale_drive_fault_latches_and_reset_requires_new_sample() {
        let mut n = runner(); sample(&mut n); assert!(n.capture(0,50_000));
        assert!(!n.tick(49_999_999,true)); assert!(n.tick(50_000_000,true));
        assert!(!n.input(0,1.0)); n.reset(); assert_eq!(n.sequence,0); assert!(!n.tick(0,false));
        assert!(n.tick(0,true));
    }
    #[test]
    fn invalid_numeric_samples_and_channel_aliases_rejected() {
        let mut n = runner(); assert!(!n.input(0,f64::NAN)); assert!(!n.input(0,f64::MAX));
        sample(&mut n); assert!(n.input(contract::RESULT,1.5)); assert!(!n.can_capture(0));
        assert!(n.input(contract::RESULT,1.0)); assert!(n.input(contract::VACUUM,2.0)); assert!(!n.can_capture(0));
        let mut c = n.config; c.direction_channel = c.start_channel; assert!(VirtualNutrunner::new(c).is_none());
    }
}
