use robotkit_device_protocol::heater::*;

/// A thermistor circuit and a lumped plant: `C dT/dt = P duty - h (T - T_amb)`, as the virtual device models them.
struct Rig {
    config: HeaterConfig,
    celsius: f32,
    sensed: Option<f32>,
    duty: f32,
}
impl HeaterIo for Rig {
    fn set_duty(&mut self, duty: f32) { self.duty = duty; }
    fn adc_volts(&self) -> f32 { self.config.volts(self.sensed.unwrap_or(self.celsius)) }
}

struct Plant { capacity: f32, power: f32, loss: f32, ambient: f32, element: bool, extra_loss: f32 }
impl Plant {
    fn step(&self, rig: &mut Rig, dt: f32) {
        let power = if self.element { self.power * rig.duty } else { 0.0 };
        rig.celsius += (power - (self.loss + self.extra_loss) * (rig.celsius - self.ambient)) / self.capacity * dt;
    }
}

fn hotend_config() -> HeaterConfig {
    HeaterConfig { r0_ohm: 100_000.0, t0_c: 25.0, beta: 3950.0, pullup_ohm: 4_700.0, v_ref: 3.3,
        kp: 0.08, ki: 0.004, kd: 0.5, max_duty: 1.0, max_temperature_c: 275.0, min_extrude_c: 170.0,
        band_c: 3.0, pid_range_c: 10.0, gain_window_s: 20.0, gain_min_c: 2.0, hysteresis_c: 15.0, hysteresis_window_s: 10.0 }
}
fn hotend() -> Plant { Plant { capacity: 10.0, power: 40.0, loss: 0.06, ambient: 25.0, element: true, extra_loss: 0.0 } }

fn rig(config: HeaterConfig, celsius: f32) -> Rig { Rig { config, celsius, sensed: None, duty: 0.0 } }

/// Runs the heater for `seconds` and returns the last frame.
fn run(heater: &mut Heater, rig: &mut Rig, plant: &Plant, seconds: f64, target: f32, extruding: bool) -> [f32; VALUES] {
    let dt = 0.05;
    let mut frame = heater.values();
    for _ in 0..(seconds / dt).round() as usize {
        frame = heater.tick(rig, dt, target, extruding);
        plant.step(rig, dt as f32);
    }
    frame
}

#[test]
fn the_thermistor_reads_back_the_temperature_it_was_given() {
    let c = hotend_config();
    for celsius in [0.0f32, 25.0, 60.0, 100.0, 150.0, 210.0, 260.0, 300.0] {
        let read = c.temperature(c.volts(celsius)).unwrap();
        assert!((read - celsius).abs() < 0.05, "{celsius} read as {read}");
    }
    // 100k at 25 C: 3.15 V under a 4.7k pull-up to 3.3 V.
    assert!((c.volts(25.0) - 3.3 * 100_000.0 / 104_700.0).abs() < 1e-3);
    assert_eq!(c.temperature(3.3), Err(fault::SENSOR_OPEN));
    assert_eq!(c.temperature(0.0), Err(fault::SENSOR_SHORT));
    assert_eq!(c.temperature(f32::NAN), Err(fault::INVALID));
}

#[test]
fn a_hotend_heats_to_target_and_holds_it() {
    let c = hotend_config();
    let mut heater = Heater::new(c).unwrap();
    let mut rig = rig(c, 25.0); let plant = hotend();
    let mut reached = None; let mut peak = 0.0f32;
    let dt = 0.05;
    for step in 0..(240.0 / dt) as usize {
        let frame = heater.tick(&mut rig, dt, 210.0, false);
        plant.step(&mut rig, dt as f32);
        peak = peak.max(rig.celsius);
        if frame[AT_TARGET] == 1.0 && reached.is_none() { reached = Some(step as f64 * dt); }
    }
    let reached = reached.expect("the hotend reaches 210 C");
    eprintln!("hotend reaches 210 C in {reached:.1} s, peaks at {peak:.1} C");
    assert!(reached > 30.0 && reached < 90.0, "{reached}");
    assert!(peak < 215.0, "overshoot to {peak}");
    // Held for a minute, within a degree.
    let (mut low, mut high) = (f32::MAX, f32::MIN);
    for _ in 0..(60.0 / dt) as usize {
        heater.tick(&mut rig, dt, 210.0, false); plant.step(&mut rig, dt as f32);
        low = low.min(rig.celsius); high = high.max(rig.celsius);
    }
    eprintln!("hotend holds {low:.2} to {high:.2} C");
    assert!(high - low < 1.0, "ripple {}", high - low);
    assert_eq!(heater.fault(), 0);
}

#[test]
fn a_bed_heats_to_target_and_holds_it() {
    let c = HeaterConfig { kp: 0.2, ki: 0.0008, kd: 2.0, min_extrude_c: 0.0, max_temperature_c: 120.0,
        gain_window_s: 120.0, gain_min_c: 2.0, hysteresis_c: 10.0, hysteresis_window_s: 20.0, ..hotend_config() };
    let mut heater = Heater::new(c).unwrap();
    let mut rig = rig(c, 25.0);
    let plant = Plant { capacity: 500.0, power: 250.0, loss: 0.7, ambient: 25.0, element: true, extra_loss: 0.0 };
    let mut reached = None; let mut peak = 0.0f32;
    let dt = 0.05;
    for step in 0..(400.0 / dt) as usize {
        let frame = heater.tick(&mut rig, dt, 60.0, false);
        plant.step(&mut rig, dt as f32);
        peak = peak.max(rig.celsius);
        if frame[AT_TARGET] == 1.0 && reached.is_none() { reached = Some(step as f64 * dt); }
    }
    let reached = reached.expect("the bed reaches 60 C");
    eprintln!("bed reaches 60 C in {reached:.1} s, peaks at {peak:.1} C");
    assert!(reached > 30.0 && reached < 200.0, "{reached}");
    assert!(peak < 64.0, "overshoot to {peak}");
    assert_eq!(heater.fault(), 0);
}

#[test]
fn every_fault_turns_the_output_off_and_stays_until_reset() {
    let c = hotend_config();
    let plant = hotend();
    // Sensor open and short.
    for (volts_of, code) in [(3.3f32, fault::SENSOR_OPEN), (0.0, fault::SENSOR_SHORT)] {
        let mut heater = Heater::new(c).unwrap();
        let mut rig = rig(c, 25.0);
        run(&mut heater, &mut rig, &plant, 2.0, 210.0, false);
        assert!(rig.duty > 0.0);
        struct Fixed(f32, f32);
        impl HeaterIo for Fixed { fn set_duty(&mut self, d: f32) { self.1 = d; } fn adc_volts(&self) -> f32 { self.0 } }
        let mut io = Fixed(volts_of, 1.0);
        let frame = heater.tick(&mut io, 0.05, 210.0, false);
        assert_eq!(frame[FAULT], code as f32); assert_eq!(io.1, 0.0); assert_eq!(frame[AT_TARGET], 0.0);
        // The sensor comes back and the fault stays.
        let frame = heater.tick(&mut rig, 0.05, 210.0, false);
        assert_eq!(frame[FAULT], code as f32); assert_eq!(rig.duty, 0.0);
        heater.reset(&mut rig);
        assert_eq!(heater.tick(&mut rig, 0.05, 210.0, false)[FAULT], 0.0);
    }
    // Over temperature, whatever the target.
    let mut heater = Heater::new(c).unwrap();
    let mut rig_hot = self::rig(c, 280.0);
    assert_eq!(heater.tick(&mut rig_hot, 0.05, 0.0, false)[FAULT], fault::OVER_TEMPERATURE as f32);
    // A disconnected heater: the block never warms.
    let mut heater = Heater::new(c).unwrap();
    let mut rig_cold = self::rig(c, 25.0);
    let disconnected = Plant { element: false, ..hotend() };
    let frame = run(&mut heater, &mut rig_cold, &disconnected, 30.0, 210.0, false);
    assert_eq!(frame[FAULT], fault::NOT_HEATING as f32, "{frame:?}");
    assert_eq!(rig_cold.duty, 0.0);
    // A thermistor that has fallen off its block reads room temperature while the block heats.
    let mut heater = Heater::new(c).unwrap();
    let mut rig_off = self::rig(c, 25.0); rig_off.sensed = Some(25.0);
    let frame = run(&mut heater, &mut rig_off, &plant, 30.0, 210.0, false);
    assert_eq!(frame[FAULT], fault::NOT_HEATING as f32);
    // A blast of air takes more than the heater gives.
    let mut heater = Heater::new(c).unwrap();
    let mut rig_fan = self::rig(c, 25.0);
    let blast = Plant { extra_loss: 3.0, ..hotend() };
    let frame = run(&mut heater, &mut rig_fan, &blast, 60.0, 210.0, false);
    assert_eq!(frame[FAULT], fault::NOT_HEATING as f32);
}

#[test]
fn leaving_the_band_after_reaching_it_is_a_fault() {
    let c = hotend_config();
    let mut heater = Heater::new(c).unwrap();
    let mut rig = rig(c, 25.0);
    let plant = hotend();
    let frame = run(&mut heater, &mut rig, &plant, 120.0, 210.0, false);
    assert_eq!(frame[AT_TARGET], 1.0);
    // The element drops out: the block cools towards the room.
    let dead = Plant { element: false, ..hotend() };
    let frame = run(&mut heater, &mut rig, &dead, 300.0, 210.0, false);
    assert_eq!(frame[FAULT], fault::LEFT_BAND as f32, "{frame:?}");
    assert_eq!(rig.duty, 0.0);
}

#[test]
fn a_cold_hotend_may_not_extrude() {
    let c = hotend_config();
    let mut heater = Heater::new(c).unwrap();
    let mut rig = rig(c, 100.0);
    // Heating with the extruder idle is fine; pushing filament at 100 C is not.
    assert_eq!(heater.tick(&mut rig, 0.05, 210.0, false)[FAULT], 0.0);
    assert_eq!(heater.tick(&mut rig, 0.05, 210.0, true)[FAULT], fault::COLD_EXTRUSION as f32);
    // At the minimum it may.
    let mut warm = self::rig(c, 200.0);
    let mut heater = Heater::new(c).unwrap();
    assert_eq!(heater.tick(&mut warm, 0.05, 210.0, true)[FAULT], 0.0);
    // The check is off when the material has no minimum.
    let mut heater = Heater::new(HeaterConfig { min_extrude_c: 0.0, ..c }).unwrap();
    assert_eq!(heater.tick(&mut rig, 0.05, 210.0, true)[FAULT], 0.0);
}

#[test]
fn a_target_off_switches_the_output_off_and_a_bad_command_faults() {
    let c = hotend_config();
    let mut heater = Heater::new(c).unwrap();
    let mut rig = rig(c, 25.0);
    run(&mut heater, &mut rig, &hotend(), 5.0, 210.0, false);
    assert!(rig.duty > 0.0);
    heater.tick(&mut rig, 0.05, 0.0, false);
    assert_eq!(rig.duty, 0.0);
    assert_eq!(heater.tick(&mut rig, 0.05, f32::NAN, false)[FAULT], fault::INVALID as f32);
    heater.reset(&mut rig);
    assert_eq!(heater.tick(&mut rig, -1.0, 0.0, false)[FAULT], fault::INVALID as f32);
    assert!(Heater::new(HeaterConfig { kp: f32::NAN, ..c }).is_none());
    assert!(Heater::new(HeaterConfig { max_duty: 1.5, ..c }).is_none());
    assert!(Heater::new(HeaterConfig { min_extrude_c: 300.0, ..c }).is_none());
}
