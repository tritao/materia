//! Heater control: a PWM duty out, a thermistor in, PID, and a watchdog that latches a fault.
//!
//! The controller owns the heater (a host that stalls must not leave one on): the host sets a target
//! and reads back the temperature, the duty, whether it is at target, and the fault. Every fault turns
//! the output off and stays until `reset`, which a session start does while the outputs are held safe.
//! The same code runs on the board and in the virtual device, against a thermal plant there.

/// Time constant of the filter on the measured rate of change, seconds.
const RATE_FILTER_SECONDS: f32 = 2.0;

/// Values of a heater's sensor frame, in order.
pub const VALUES: usize = 5;
pub const TEMPERATURE: usize = 0;
pub const TARGET: usize = 1;
pub const DUTY: usize = 2;
pub const AT_TARGET: usize = 3;
pub const FAULT: usize = 4;

/// What stopped a heater, as the frame's fault value carries it. With a sensor fault the temperature reads 0:
/// frame values are finite numbers, and there is no reading.
pub mod fault {
    pub const NONE: u8 = 0;
    /// The thermistor reads as an open circuit: unplugged, or the wire broke.
    pub const SENSOR_OPEN: u8 = 1;
    /// The thermistor reads as a short.
    pub const SENSOR_SHORT: u8 = 2;
    pub const OVER_TEMPERATURE: u8 = 3;
    /// Heating towards a target for a whole window without gaining enough: the heater is not
    /// connected, the thermistor has fallen off its block, or the fan blows the heat away.
    pub const NOT_HEATING: u8 = 4;
    /// At target once, then below it by more than the hysteresis for a whole window.
    pub const LEFT_BAND: u8 = 5;
    /// The extruder pushed filament while the hotend was below the material's minimum.
    pub const COLD_EXTRUSION: u8 = 6;
    /// A configuration or a command that is not a number.
    pub const INVALID: u8 = 7;
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HeaterConfig {
    /// Thermistor: a negative-coefficient resistor to ground under a pull-up to `v_ref`; Beta model.
    pub r0_ohm: f32,
    pub t0_c: f32,
    pub beta: f32,
    pub pullup_ohm: f32,
    pub v_ref: f32,
    /// PID gains in duty per degree, per degree-second and degree-per-second.
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub max_duty: f32,
    /// Hotter than this is a fault whatever the target.
    pub max_temperature_c: f32,
    /// The material's minimum extrusion temperature: below it the extruder must not push. Zero: unchecked.
    pub min_extrude_c: f32,
    /// At target means within this many degrees of it.
    pub band_c: f32,
    /// PID acts within this many degrees of the target; further off the output is full or off, so the
    /// integral does not wind up on the way.
    pub pid_range_c: f32,
    /// Heating must gain `gain_min_c` within every `gain_window_s` until the target is within the band.
    pub gain_window_s: f64,
    pub gain_min_c: f32,
    /// Once at target, below it by more than `hysteresis_c` for `hysteresis_window_s` is a fault.
    pub hysteresis_c: f32,
    pub hysteresis_window_s: f64,
}

impl HeaterConfig {
    pub fn valid(&self) -> bool {
        [self.r0_ohm, self.beta, self.pullup_ohm, self.v_ref, self.max_duty, self.band_c, self.pid_range_c,
            self.gain_min_c, self.hysteresis_c, self.max_temperature_c]
            .iter().all(|v| v.is_finite() && *v > 0.0)
            && [self.kp, self.ki, self.kd, self.min_extrude_c].iter().all(|v| v.is_finite() && *v >= 0.0)
            && self.t0_c.is_finite() && self.t0_c > -273.0
            && self.max_duty <= 1.0
            && [self.gain_window_s, self.hysteresis_window_s].iter().all(|v| v.is_finite() && *v > 0.0)
            && self.min_extrude_c < self.max_temperature_c
    }

    /// Temperature in degrees Celsius for the volts at the ADC, or the fault the voltage means.
    pub fn temperature(&self, volts: f32) -> Result<f32, u8> {
        if !volts.is_finite() { return Err(fault::INVALID); }
        // Within 1 % of either rail is no thermistor: open (reads the pull-up) or shorted (reads ground).
        if volts >= 0.99 * self.v_ref { return Err(fault::SENSOR_OPEN); }
        if volts <= 0.01 * self.v_ref { return Err(fault::SENSOR_SHORT); }
        let resistance = self.pullup_ohm * volts / (self.v_ref - volts);
        let kelvin = 1.0 / (1.0 / (self.t0_c + 273.15) + libm_ln(resistance / self.r0_ohm) / self.beta);
        Ok(kelvin - 273.15)
    }

    /// Volts at the ADC for a thermistor at `celsius`: the inverse of `temperature`.
    pub fn volts(&self, celsius: f32) -> f32 {
        let resistance = self.r0_ohm * libm_exp(self.beta * (1.0 / (celsius + 273.15) - 1.0 / (self.t0_c + 273.15)));
        self.v_ref * resistance / (resistance + self.pullup_ohm)
    }
}

/// The board's heater output and thermistor input.
pub trait HeaterIo {
    /// Fraction of the supply to apply, 0 to 1.
    fn set_duty(&mut self, duty: f32);
    /// Volts at the thermistor's ADC pin.
    fn adc_volts(&self) -> f32;
}

pub struct Heater {
    pub config: HeaterConfig,
    integral: f32,
    previous: Option<f32>,
    rate: f32,
    fault: u8,
    target: f32,
    at_target_once: bool,
    gain_reference: f32,
    gain_elapsed: f64,
    below_elapsed: f64,
    last: [f32; VALUES],
}

impl Heater {
    pub fn new(config: HeaterConfig) -> Option<Self> {
        if !config.valid() { return None; }
        Some(Self { config, integral: 0.0, previous: None, rate: 0.0, fault: 0, target: 0.0, at_target_once: false,
            gain_reference: 0.0, gain_elapsed: 0.0, below_elapsed: 0.0, last: [0.0; VALUES] })
    }

    pub fn values(&self) -> [f32; VALUES] { self.last }
    pub fn fault(&self) -> u8 { self.fault }

    /// Output off. A fault, a stop and a session end all come here.
    pub fn safe<I: HeaterIo>(&self, io: &mut I) { io.set_duty(0.0); }

    /// Clears a fault; only while the outputs are held safe, so it cannot restart a heater.
    pub fn reset<I: HeaterIo>(&mut self, io: &mut I) {
        self.safe(io);
        self.integral = 0.0; self.previous = None; self.rate = 0.0; self.fault = 0; self.target = 0.0;
        self.at_target_once = false; self.gain_elapsed = 0.0; self.below_elapsed = 0.0;
        self.last = [0.0; VALUES];
    }

    /// One control period of `dt` seconds with the host's `target` (0 is off) and whether the extruder
    /// is pushing filament. Returns the sensor frame.
    pub fn tick<I: HeaterIo>(&mut self, io: &mut I, dt: f64, target: f32, extruding: bool) -> [f32; VALUES] {
        let c = self.config;
        if !dt.is_finite() || dt < 0.0 || !target.is_finite() || target < 0.0 { self.latch(fault::INVALID); }
        let temperature = match c.temperature(io.adc_volts()) {
            Ok(value) => value,
            Err(code) => { self.latch(code); self.last[TEMPERATURE] = 0.0; return self.finish(io, 0.0, false); }
        };
        if self.fault == 0 {
            if temperature > c.max_temperature_c { self.latch(fault::OVER_TEMPERATURE); }
            if extruding && c.min_extrude_c > 0.0 && temperature < c.min_extrude_c { self.latch(fault::COLD_EXTRUSION); }
        }
        self.last[TEMPERATURE] = temperature;
        if self.fault != 0 { return self.finish(io, temperature, false); }
        // A new target restarts what the watchdog was watching for.
        if target != self.target {
            self.target = target; self.at_target_once = false; self.gain_elapsed = 0.0; self.below_elapsed = 0.0;
            self.gain_reference = temperature;
            if target == 0.0 { self.integral = 0.0; }
        }
        let error = target - temperature;
        let at = target > 0.0 && libm_abs(error) <= c.band_c;
        if at { self.at_target_once = true; }
        if target > 0.0 && dt > 0.0 {
            // Heating up: a window that gains too little is a heater that is not heating.
            if error > c.band_c && !self.at_target_once {
                if temperature >= self.gain_reference + c.gain_min_c { self.gain_reference = temperature; self.gain_elapsed = 0.0; }
                else {
                    self.gain_elapsed += dt;
                    if self.gain_elapsed >= c.gain_window_s { self.latch(fault::NOT_HEATING); }
                }
            } else { self.gain_reference = temperature; self.gain_elapsed = 0.0; }
            // Held: below the band after having been in it.
            if self.at_target_once && error > c.hysteresis_c {
                self.below_elapsed += dt;
                if self.below_elapsed >= c.hysteresis_window_s { self.latch(fault::LEFT_BAND); }
            } else { self.below_elapsed = 0.0; }
        }
        if self.fault != 0 { return self.finish(io, temperature, false); }
        let duty = if target <= 0.0 { 0.0 }
            else if error > c.pid_range_c { self.integral = 0.0; self.previous = Some(temperature); self.rate = 0.0; c.max_duty }
            else if error < -c.pid_range_c { self.integral = 0.0; self.previous = Some(temperature); self.rate = 0.0; 0.0 }
            else { self.pid(temperature, error, dt) };
        self.finish(io, duty, at)
    }

    fn pid(&mut self, temperature: f32, error: f32, dt: f64) -> f32 {
        let c = self.config;
        let dt = dt as f32;
        // Derivative on the measurement, so a new target does not kick the output.
        // The ADC steps in tenths of a degree, so the raw rate is noise: it is filtered over RATE_FILTER_SECONDS.
        if let Some(before) = self.previous {
            if dt > 0.0 {
                let alpha = dt / (RATE_FILTER_SECONDS + dt);
                self.rate += alpha * ((temperature - before) / dt - self.rate);
            }
        }
        self.previous = Some(temperature);
        let unsaturated = c.kp * error + self.integral - c.kd * self.rate;
        let duty = unsaturated.max(0.0).min(c.max_duty);
        // Anti-windup: not while the output is driven against a limit by an error that pushes it further.
        let pushing = (unsaturated >= c.max_duty && error > 0.0) || (unsaturated <= 0.0 && error < 0.0);
        if c.ki > 0.0 && !pushing {
            self.integral = (self.integral + c.ki * error * dt).max(0.0).min(c.max_duty);
        }
        duty
    }

    fn latch(&mut self, code: u8) { if self.fault == 0 { self.fault = code; } }

    fn finish<I: HeaterIo>(&mut self, io: &mut I, duty: f32, at: bool) -> [f32; VALUES] {
        let duty = if self.fault != 0 { 0.0 } else { duty };
        io.set_duty(duty);
        self.last[TARGET] = self.target;
        self.last[DUTY] = duty;
        self.last[AT_TARGET] = (at && self.fault == 0) as u8 as f32;
        self.last[FAULT] = self.fault as f32;
        self.last
    }
}

// `no_std` has no `ln`, `exp` or `abs` for floats: small wrappers over the compiler's intrinsics where they
// exist, and series otherwise.
#[inline] fn libm_abs(x: f32) -> f32 { if x < 0.0 { -x } else { x } }

/// Natural logarithm for positive finite `x`: range reduction to [1, 2) and an atanh series.
fn libm_ln(x: f32) -> f32 {
    if !(x > 0.0) || !x.is_finite() { return f32::NAN; }
    let mut m = x; let mut e = 0i32;
    while m >= 2.0 { m *= 0.5; e += 1; }
    while m < 1.0 { m *= 2.0; e -= 1; }
    let z = (m - 1.0) / (m + 1.0); let z2 = z * z;
    // ln m = 2 (z + z^3/3 + z^5/5 + ...), z <= 1/3 on [1, 2).
    let series = z * (1.0 + z2 * (1.0 / 3.0 + z2 * (1.0 / 5.0 + z2 * (1.0 / 7.0 + z2 * (1.0 / 9.0 + z2 / 11.0)))));
    2.0 * series + e as f32 * core::f32::consts::LN_2
}

/// `e^x` for finite `x`: `x = k ln 2 + r`, `e^r` by Taylor, scaled by `2^k`.
fn libm_exp(x: f32) -> f32 {
    if !x.is_finite() { return f32::NAN; }
    if x > 88.0 { return f32::INFINITY; }
    if x < -87.0 { return 0.0; }
    let k = (x / core::f32::consts::LN_2 + if x >= 0.0 { 0.5 } else { -0.5 }) as i32;
    let r = x - k as f32 * core::f32::consts::LN_2;
    let mut term = 1.0f32; let mut sum = 1.0f32;
    for n in 1..9 { term *= r / n as f32; sum += term; }
    let mut scale = 1.0f32;
    if k >= 0 { for _ in 0..k { scale *= 2.0; } } else { for _ in 0..(-k) { scale *= 0.5; } }
    sum * scale
}
