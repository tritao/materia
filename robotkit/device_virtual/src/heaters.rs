//! Virtual heaters: the board's heater control (`robotkit_device_protocol::heater`) run against a lumped thermal
//! plant, `C dT/dt = P duty - (h + h_fan fan) (T - T_amb)`. The controller code is the board's; only the
//! block, its element and its thermistor are simulated, with values assumed per part.
use robotkit_device_protocol::heater::{self, Heater, HeaterConfig, HeaterIo};

/// Seconds between control updates, as a board's timer would set them.
pub const CONTROL_PERIOD: f64 = 0.05;
/// A 12-bit ADC.
const ADC_STEPS: f32 = 4095.0;

#[derive(Clone, Copy, Debug)]
pub struct Wiring {
    pub sensor_slot: u8,
    /// The analog channel the host sets the target on, in degrees Celsius.
    pub target_channel: usize,
    /// An analog channel carrying a fan's duty (0 to 1), which cools this heater when it blows on it.
    pub fan_channel: Option<usize>,
    /// The actuator of the extruder this heater feeds: pushing it forward while cold is a fault.
    pub extruder_actuator: Option<usize>,
}

#[derive(Clone, Copy, Debug)]
pub struct Plant {
    /// Heat capacity, J/K.
    pub capacity: f32,
    /// Power at full duty, W.
    pub power: f32,
    /// Loss to the room, W/K.
    pub loss: f32,
    /// Extra loss with the fan at full duty, W/K.
    pub fan_loss: f32,
    pub ambient: f32,
}

impl Plant {
    pub fn valid(&self) -> bool {
        [self.capacity, self.power, self.loss].iter().all(|v| v.is_finite() && *v > 0.0)
            && self.fan_loss.is_finite() && self.fan_loss >= 0.0
            && self.ambient.is_finite() && self.ambient > -50.0 && self.ambient < 60.0
    }
}

/// What an experiment can do to a heater from outside: the meaning of `VirtualHeaters::set_input`.
pub mod input {
    /// 1 (default): the heating element is connected; 0: it is not.
    pub const ELEMENT: u32 = 0;
    /// 1 (default): the thermistor is on its block; 0: it has fallen off and reads the room.
    pub const SENSOR_ATTACHED: u32 = 1;
    /// Extra loss to the room, W/K: a draught, a blast of air from a fan not on the board.
    pub const BLAST: u32 = 2;
    /// 1: the thermistor wire is broken (reads open); 0: it is whole.
    pub const SENSOR_OPEN: u32 = 3;
    /// 1: the thermistor shorts; 0: it does not.
    pub const SENSOR_SHORT: u32 = 4;
    /// Inputs per heater: heater `h`'s input `kind` is `h * COUNT + kind`.
    pub const COUNT: u32 = 8;
}

struct Rig {
    config: HeaterConfig,
    celsius: f32,
    attached: bool,
    ambient: f32,
    open: bool,
    short: bool,
    duty: f32,
}
impl HeaterIo for Rig {
    fn set_duty(&mut self, duty: f32) { self.duty = duty; }
    fn adc_volts(&self) -> f32 {
        if self.open { return self.config.v_ref; }
        if self.short { return 0.0; }
        let volts = self.config.volts(if self.attached { self.celsius } else { self.ambient });
        (volts / self.config.v_ref * ADC_STEPS).round() / ADC_STEPS * self.config.v_ref
    }
}

pub struct VirtualHeater {
    pub wiring: Wiring,
    pub plant: Plant,
    heater: Heater,
    rig: Rig,
    element: bool,
    blast: f32,
    pending: f64,
    extruder_steps: Option<i64>,
    extruding: bool,
}

impl VirtualHeater {
    pub fn new(wiring: Wiring, config: HeaterConfig, plant: Plant) -> Option<Self> {
        if wiring.sensor_slot >= 8 || wiring.target_channel >= 32 || !plant.valid() { return None; }
        if wiring.fan_channel.map_or(false, |c| c >= 32 || c == wiring.target_channel) { return None; }
        let heater = Heater::new(config)?;
        Some(Self { wiring, plant, heater,
            rig: Rig { config, celsius: plant.ambient, attached: true, ambient: plant.ambient, open: false, short: false, duty: 0.0 },
            element: true, blast: 0.0, pending: 0.0, extruder_steps: None, extruding: false })
    }

    pub fn values(&self) -> [f32; heater::VALUES] { self.heater.values() }
    pub fn faulted(&self) -> bool { self.heater.fault() != 0 }
    /// The block's true temperature, which the thermistor may not read.
    pub fn block_celsius(&self) -> f32 { self.rig.celsius }

    /// A new session: the faults clear while the outputs are held safe, the block keeps its temperature.
    pub fn reset(&mut self) {
        self.heater.reset(&mut self.rig);
        self.pending = 0.0; self.extruder_steps = None; self.extruding = false;
    }

    pub fn set_input(&mut self, kind: u32, value: f64) -> bool {
        if !value.is_finite() { return false; }
        match kind {
            input::ELEMENT if value == 0.0 || value == 1.0 => self.element = value == 1.0,
            input::SENSOR_ATTACHED if value == 0.0 || value == 1.0 => self.rig.attached = value == 1.0,
            input::BLAST if value >= 0.0 => self.blast = value as f32,
            input::SENSOR_OPEN if value == 0.0 || value == 1.0 => self.rig.open = value == 1.0,
            input::SENSOR_SHORT if value == 0.0 || value == 1.0 => self.rig.short = value == 1.0,
            _ => return false,
        }
        true
    }

    /// Advances `dt` seconds with the host's `target`, the fan's duty, and the extruder actuator's step count.
    pub fn tick(&mut self, dt: f64, target: f32, fan: f32, extruder_steps: Option<i64>) {
        if let (Some(now), Some(before)) = (extruder_steps, self.extruder_steps) {
            if now > before { self.extruding = true; }
        }
        self.extruder_steps = extruder_steps;
        self.pending += dt;
        while self.pending >= CONTROL_PERIOD {
            self.pending -= CONTROL_PERIOD;
            self.heater.tick(&mut self.rig, CONTROL_PERIOD, target, self.extruding);
            self.extruding = false;
            // The block, in a few steps of the period.
            let p = &self.plant;
            let loss = p.loss + p.fan_loss * fan.max(0.0).min(1.0) + self.blast;
            let power = if self.element { p.power * self.rig.duty } else { 0.0 };
            let step = (CONTROL_PERIOD / 5.0) as f32;
            for _ in 0..5 {
                self.rig.celsius += (power - loss * (self.rig.celsius - p.ambient)) / p.capacity * step;
            }
        }
    }
}

/// All of a device's heaters.
#[derive(Default)]
pub struct VirtualHeaters {
    pub heaters: Vec<VirtualHeater>,
}

impl VirtualHeaters {
    pub fn set_input(&mut self, input: u32, value: f64) -> bool {
        let (heater, kind) = ((input / self::input::COUNT) as usize, input % self::input::COUNT);
        self.heaters.get_mut(heater).map_or(false, |h| h.set_input(kind, value))
    }
}

/// Parameters per heater in a peripheral profile's list: wiring, controller, plant.
pub const PARAMETERS_PER_HEATER: usize = 4 + 17 + 5;

/// One heater's entry in the parameter list: the inverse of what `from_parameters` reads.
pub fn parameters(wiring: Wiring, c: &HeaterConfig, plant: &Plant) -> Vec<f64> {
    let index = |v: Option<usize>| v.map_or(-1.0, |v| v as f64);
    vec![wiring.sensor_slot as f64, wiring.target_channel as f64, index(wiring.fan_channel), index(wiring.extruder_actuator),
        c.r0_ohm as f64, c.t0_c as f64, c.beta as f64, c.pullup_ohm as f64, c.v_ref as f64, c.kp as f64, c.ki as f64,
        c.kd as f64, c.max_duty as f64, c.max_temperature_c as f64, c.min_extrude_c as f64, c.band_c as f64,
        c.pid_range_c as f64, c.gain_window_s, c.gain_min_c as f64, c.hysteresis_c as f64, c.hysteresis_window_s,
        plant.capacity as f64, plant.power as f64, plant.loss as f64, plant.fan_loss as f64, plant.ambient as f64]
}

/// Builds the heaters of peripheral profile 2: the heater count, then per heater its sensor slot, target
/// channel, fan channel (-1 for none), extruder actuator (-1 for none), the controller's `r0, t0, beta, pullup,
/// v_ref, kp, ki, kd, max_duty, max_temperature, min_extrude, band, pid_range, gain_window, gain_min, hysteresis,
/// hysteresis_window`, and the plant's `capacity, power, loss, fan_loss, ambient`.
pub fn from_parameters(p: &[f64]) -> Option<VirtualHeaters> {
    if p.is_empty() || p.iter().any(|v| !v.is_finite()) { return None; }
    let count = p[0];
    if count < 1.0 || count > 8.0 || count.fract() != 0.0 || p.len() != 1 + count as usize * PARAMETERS_PER_HEATER { return None; }
    let mut result = VirtualHeaters::default();
    for h in 0..count as usize {
        let q = &p[1 + h * PARAMETERS_PER_HEATER..1 + (h + 1) * PARAMETERS_PER_HEATER];
        let index = |v: f64| -> Option<Option<usize>> {
            if v == -1.0 { Some(None) } else if v >= 0.0 && v.fract() == 0.0 && v < 64.0 { Some(Some(v as usize)) } else { None }
        };
        if q[0] < 0.0 || q[0].fract() != 0.0 || q[1] < 0.0 || q[1].fract() != 0.0 { return None; }
        let wiring = Wiring { sensor_slot: q[0] as u8, target_channel: q[1] as usize, fan_channel: index(q[2])?,
            extruder_actuator: index(q[3])? };
        let c = &q[4..21];
        let config = HeaterConfig { r0_ohm: c[0] as f32, t0_c: c[1] as f32, beta: c[2] as f32, pullup_ohm: c[3] as f32,
            v_ref: c[4] as f32, kp: c[5] as f32, ki: c[6] as f32, kd: c[7] as f32, max_duty: c[8] as f32,
            max_temperature_c: c[9] as f32, min_extrude_c: c[10] as f32, band_c: c[11] as f32,
            pid_range_c: c[12] as f32, gain_window_s: c[13], gain_min_c: c[14] as f32, hysteresis_c: c[15] as f32,
            hysteresis_window_s: c[16] };
        let plant = Plant { capacity: q[21] as f32, power: q[22] as f32, loss: q[23] as f32, fan_loss: q[24] as f32,
            ambient: q[25] as f32 };
        result.heaters.push(VirtualHeater::new(wiring, config, plant)?);
    }
    // Two heaters cannot share a slot or a target channel.
    for (i, a) in result.heaters.iter().enumerate() {
        for b in &result.heaters[i + 1..] {
            if a.wiring.sensor_slot == b.wiring.sensor_slot || a.wiring.target_channel == b.wiring.target_channel { return None; }
        }
    }
    Some(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> HeaterConfig {
        HeaterConfig { r0_ohm: 100_000.0, t0_c: 25.0, beta: 3950.0, pullup_ohm: 4_700.0, v_ref: 3.3,
            kp: 0.08, ki: 0.004, kd: 0.5, max_duty: 1.0, max_temperature_c: 275.0, min_extrude_c: 170.0,
            band_c: 3.0, pid_range_c: 10.0, gain_window_s: 20.0, gain_min_c: 2.0, hysteresis_c: 15.0, hysteresis_window_s: 10.0 }
    }
    fn hotend() -> VirtualHeater {
        VirtualHeater::new(Wiring { sensor_slot: 0, target_channel: 0, fan_channel: Some(1), extruder_actuator: Some(3) }, config(),
            Plant { capacity: 10.0, power: 40.0, loss: 0.06, fan_loss: 0.05, ambient: 22.0 }).unwrap()
    }
    fn bed() -> VirtualHeater {
        VirtualHeater::new(Wiring { sensor_slot: 1, target_channel: 2, fan_channel: None, extruder_actuator: None },
            HeaterConfig { kp: 0.2, ki: 0.0008, kd: 2.0, min_extrude_c: 0.0, max_temperature_c: 120.0, gain_window_s: 120.0,
                hysteresis_c: 10.0, hysteresis_window_s: 20.0, ..config() },
            Plant { capacity: 500.0, power: 250.0, loss: 0.7, fan_loss: 0.0, ambient: 22.0 }).unwrap()
    }
    /// Seconds until the heater reports at target, and the highest block temperature on the way.
    fn heat_up(h: &mut VirtualHeater, target: f32, fan: f32) -> (f64, f32) {
        let mut peak = 0.0f32;
        for step in 0..60_000 {
            h.tick(0.01, target, fan, None);
            peak = peak.max(h.block_celsius());
            if h.values()[heater::AT_TARGET] == 1.0 { return (step as f64 * 0.01, peak); }
            assert_eq!(h.values()[heater::FAULT], 0.0, "faulted at step {step}: {:?}", h.values());
        }
        panic!("never at target");
    }

    #[test]
    fn the_hotend_and_the_bed_heat_up_in_assumed_times() {
        let (hotend_seconds, hotend_peak) = heat_up(&mut hotend(), 210.0, 0.0);
        let (bed_seconds, bed_peak) = heat_up(&mut bed(), 60.0, 0.0);
        eprintln!("hotend to 210 C in {hotend_seconds:.1} s (peak {hotend_peak:.1}), bed to 60 C in {bed_seconds:.1} s (peak {bed_peak:.1})");
        assert!(hotend_seconds > 30.0 && hotend_seconds < 90.0 && hotend_peak < 216.0);
        assert!(bed_seconds > 40.0 && bed_seconds < 200.0 && bed_peak < 65.0);
    }

    #[test]
    fn holding_a_target_ripples_by_under_a_degree_quantized_and_with_the_part_fan_on() {
        let mut h = hotend();
        heat_up(&mut h, 210.0, 0.0);
        for _ in 0..3_000 { h.tick(0.01, 210.0, 0.0, None); }
        let (mut low, mut high) = (f32::MAX, f32::MIN);
        for _ in 0..6_000 { h.tick(0.01, 210.0, 0.0, None); low = low.min(h.block_celsius()); high = high.max(h.block_celsius()); }
        eprintln!("hotend holds {low:.2} to {high:.2} C");
        assert!(high - low < 1.0, "ripple {}", high - low);
        // The part fan on full takes heat away; the controller makes it up.
        let (mut low, mut high) = (f32::MAX, f32::MIN);
        for _ in 0..9_000 { h.tick(0.01, 210.0, 1.0, None); low = low.min(h.block_celsius()); high = high.max(h.block_celsius()); }
        eprintln!("hotend holds {low:.2} to {high:.2} C with the fan on");
        assert!(low > 207.0 && high < 213.0 && h.values()[heater::AT_TARGET] == 1.0 && h.values()[heater::DUTY] > 0.2);
    }

    #[test]
    fn runaway_is_caught_for_a_dead_element_a_fallen_thermistor_and_a_blast() {
        let mut dead = hotend(); assert!(dead.set_input(input::ELEMENT, 0.0));
        for _ in 0..4_000 { dead.tick(0.01, 210.0, 0.0, None); }
        assert_eq!(dead.values()[heater::FAULT], heater::fault::NOT_HEATING as f32);
        assert_eq!(dead.values()[heater::DUTY], 0.0);
        let mut fallen = hotend(); assert!(fallen.set_input(input::SENSOR_ATTACHED, 0.0));
        for _ in 0..4_000 { fallen.tick(0.01, 210.0, 0.0, None); }
        assert_eq!(fallen.values()[heater::FAULT], heater::fault::NOT_HEATING as f32);
        // It heats for the window before the watchdog stops it, far from the limit.
        assert!(fallen.block_celsius() < 150.0, "{}", fallen.block_celsius());
        let mut blasted = hotend(); assert!(blasted.set_input(input::BLAST, 3.0));
        for _ in 0..6_000 { blasted.tick(0.01, 210.0, 0.0, None); }
        assert_eq!(blasted.values()[heater::FAULT], heater::fault::NOT_HEATING as f32);
        // A broken or shorted wire reads as one.
        let mut open = hotend(); open.set_input(input::SENSOR_OPEN, 1.0);
        open.tick(0.1, 210.0, 0.0, None);
        assert_eq!(open.values()[heater::FAULT], heater::fault::SENSOR_OPEN as f32);
        let mut short = hotend(); short.set_input(input::SENSOR_SHORT, 1.0);
        short.tick(0.1, 210.0, 0.0, None);
        assert_eq!(short.values()[heater::FAULT], heater::fault::SENSOR_SHORT as f32);
        // The faults clear with the session, and nothing restarts by itself.
        open.set_input(input::SENSOR_OPEN, 0.0);
        open.tick(0.1, 210.0, 0.0, None);
        assert_eq!(open.values()[heater::FAULT], heater::fault::SENSOR_OPEN as f32);
        open.reset();
        open.tick(0.1, 0.0, 0.0, None);
        assert_eq!(open.values()[heater::FAULT], 0.0);
    }

    #[test]
    fn the_extruder_pushing_while_cold_is_a_fault_and_while_hot_is_not() {
        let mut h = hotend();
        for step in 0..100 { h.tick(0.01, 210.0, 0.0, Some(step * 10)); }
        assert_eq!(h.values()[heater::FAULT], heater::fault::COLD_EXTRUSION as f32);
        let mut hot = hotend();
        heat_up(&mut hot, 210.0, 0.0);
        for step in 0..200 { hot.tick(0.01, 210.0, 0.0, Some(step * 10)); }
        assert_eq!(hot.values()[heater::FAULT], 0.0);
        // A retraction (steps going down) is not pushing.
        let mut cold = hotend();
        for step in 0..100 { cold.tick(0.01, 0.0, 0.0, Some(-step * 10)); }
        assert_eq!(cold.values()[heater::FAULT], 0.0);
    }

    #[test]
    fn parameters_build_heaters_and_reject_nonsense() {
        let c = config();
        let one = |slot: u8, target: usize| -> Vec<f64> {
            parameters(Wiring { sensor_slot: slot, target_channel: target, fan_channel: None, extruder_actuator: None }, &c,
                &Plant { capacity: 10.0, power: 40.0, loss: 0.06, fan_loss: 0.05, ambient: 22.0 })
        };
        let mut list = vec![2.0]; list.extend(one(0, 0)); list.extend(one(1, 2));
        let heaters = from_parameters(&list).unwrap();
        assert_eq!(heaters.heaters.len(), 2);
        let mut clash = vec![2.0]; clash.extend(one(0, 0)); clash.extend(one(0, 2));
        assert!(from_parameters(&clash).is_none());
        let mut short = list.clone(); short.pop();
        assert!(from_parameters(&short).is_none());
        let mut nan = list.clone(); nan[5] = f64::NAN;
        assert!(from_parameters(&nan).is_none());
        let mut bad = list.clone(); bad[10] = -1.0; // a negative gain
        assert!(from_parameters(&bad).is_none());
    }
}
