use robotkit_device_protocol::welder_retrofit::*;
#[derive(Default)]
struct Io { trigger: bool, wire: f32, voltage: f32, current_input: f32, voltage_input: f32, touch: bool }
impl RetrofitIo for Io {
    fn trigger(&mut self, v: bool) { self.trigger = v; }
    fn wire_output_v(&mut self, v: f32) { self.wire = v; }
    fn voltage_output_v(&mut self, v: f32) { self.voltage = v; }
    fn current_input_v(&self) -> f32 { self.current_input }
    fn arc_voltage_input_v(&self) -> f32 { self.voltage_input }
    fn touch_input(&self) -> bool { self.touch }
}
fn profile() -> RetrofitWelder {
    RetrofitWelder::new(RetrofitConfig { wire_at_10v: 20.0, voltage_at_10v: 40.0,
        current_a_per_v: 100.0, current_zero_v: 0.5, arc_v_per_v: 20.0,
        arc_threshold_a: 5.0, no_arc_seconds: 0.2, short_voltage_v: 8.0,
        short_seconds: 0.1, efficiency: 0.9 }).unwrap()
}
#[test]
fn retrofit_scales_outputs_and_reads_independent_touch() {
    let mut w = profile(); let mut io = Io { current_input: 2.9, voltage_input: 1.2, touch: true, ..Io::default() };
    let r = w.tick(&mut io, 0.01, true, 8.0, 24.0);
    assert!(io.trigger); assert_eq!(io.wire, 4.0); assert_eq!(io.voltage, 6.0);
    assert_eq!(r[0], 1.0); assert!((r[1] - 240.0).abs() < 0.001); assert_eq!(r[2], 24.0); assert_eq!(r[3], 1.0);
    w.safe(&mut io); assert!(!io.trigger); assert_eq!(io.wire, 0.0); assert_eq!(io.voltage, 0.0);
}
#[test]
fn retrofit_no_arc_and_persistent_short_latch_safe_outputs() {
    let mut w = profile(); let mut io = Io::default();
    assert_eq!(w.tick(&mut io, 0.2, true, 8.0, 24.0)[4], 1.0); assert!(!io.trigger);
    io.current_input = 2.9; io.voltage_input = 1.2;
    assert_eq!(w.tick(&mut io, 0.01, true, 8.0, 24.0)[4], 1.0); assert!(!io.trigger);
    w.reset(&mut io); io.voltage_input = 0.1;
    assert_eq!(w.tick(&mut io, 0.02, true, 8.0, 24.0)[4], 0.0);
    io.voltage_input = 1.2; assert_eq!(w.tick(&mut io, 0.01, true, 8.0, 24.0)[4], 0.0);
    io.voltage_input = 0.1; assert_eq!(w.tick(&mut io, 0.1, true, 8.0, 24.0)[4], 3.0);
    assert!(!io.trigger); assert_eq!(io.wire, 0.0);
}
#[test]
fn retrofit_rejects_invalid_scaling_and_safes_invalid_commands() {
    let mut c = profile().config; c.wire_at_10v = 0.0; assert!(RetrofitWelder::new(c).is_none());
    let mut w = profile(); let mut io = Io::default();
    assert_eq!(w.tick(&mut io, 0.01, true, f32::NAN, 24.0)[4], 3.0); assert!(!io.trigger);
}
