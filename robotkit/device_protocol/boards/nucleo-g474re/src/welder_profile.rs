//! Optional retrofit connector assignment; the bench UART image does not enable these outputs.
use stm32g4xx_hal::{adc::Ad, dac::Pins, gpio::{Analog, Input, Output, PushPull, PA4, PA5, PB0, PB1, PB2, PB10}, stm32};
use embedded_hal_old::adc::Channel;

/// DAC outputs require isolated external amplifiers from 0–3.3 V to 0–10 V.
#[allow(dead_code)]
pub struct RetrofitPins {
    pub wire: PA4<Analog>,
    pub voltage: PA5<Analog>,
    pub current: PB0<Analog>,
    pub arc_voltage: PB1<Analog>,
    pub trigger: PB2<Output<PushPull>>,
    pub touch: PB10<Input>,
}

#[allow(dead_code)]
fn check_pin_capabilities() {
    fn dac<P: Pins<stm32::DAC1>>() {}
    fn adc<P: Channel<Ad<stm32::ADC1>, ID = u8>>() {}
    dac::<(PA4<Analog>, PA5<Analog>)>();
    adc::<PB0<Analog>>(); adc::<PB1<Analog>>();
}
