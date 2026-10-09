//! Placeholder connector assignment for two heaters (hotend and bed): a gate each for a MOSFET and a
//! thermistor on an ADC pin. The bench UART image does not enable these outputs; the pins are placeholders
//! until a board is wired, and the PWM timers are chosen then (the gate is driven on/off here).
use embedded_hal_old::adc::Channel;
use embedded_hal_old::digital::v2::OutputPin;
use robotkit_device_protocol::heater::{Heater, HeaterConfig, HeaterIo};
use stm32g4xx_hal::{adc::Ad, gpio::{Analog, Output, PushPull, PA0, PA1, PB6, PB7}, stm32};

#[allow(dead_code)]
pub struct HeaterPins {
    pub hotend_thermistor: PA0<Analog>,
    pub bed_thermistor: PA1<Analog>,
    pub hotend_gate: PB6<Output<PushPull>>,
    pub bed_gate: PB7<Output<PushPull>>,
}

/// One heater's output and input: the gate is on for the fraction of a period `duty` asks for, driven by
/// the control loop's calls (a timer's PWM replaces this when the pins are real).
#[allow(dead_code)]
pub struct GateIo<G: OutputPin> {
    pub gate: G,
    pub adc_volts: f32,
    pub duty: f32,
}
impl<G: OutputPin> HeaterIo for GateIo<G> {
    fn set_duty(&mut self, duty: f32) {
        self.duty = duty;
        let _ = if duty >= 0.5 { self.gate.set_high() } else { self.gate.set_low() };
    }
    fn adc_volts(&self) -> f32 { self.adc_volts }
}

#[allow(dead_code)]
fn check_pin_capabilities() {
    fn adc<P: Channel<Ad<stm32::ADC1>, ID = u8>>() {}
    adc::<PA0<Analog>>(); adc::<PA1<Analog>>();
}

/// The control loop type-checks for the target: one heater tick against a gate.
#[allow(dead_code)]
pub fn control<G: OutputPin>(heater: &mut Heater, io: &mut GateIo<G>, dt: f64, target: f32, extruding: bool) -> [f32; 5] {
    heater.tick(io, dt, target, extruding)
}

#[allow(dead_code)]
pub fn heater(config: HeaterConfig) -> Option<Heater> { Heater::new(config) }
