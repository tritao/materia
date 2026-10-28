//! The whole chain with no hardware: RKD6 frames to a VirtualDevice (session, jog segment, commit),
//! pendant CAN frames to the operator service, the service installed as the device's jog gate.
//! When the pendant lease lapses the device stops the jog; without the gate the same jog runs on.

use cabinet_node::OperatorService;
use operator_link::profile::CompiledProfile;
use operator_link::operator_wire::PendantState;
use robotkit_device_protocol::device_wire6::*;
use robotkit_device_protocol::frame6::{decode_frame6, encode_frame6, MAX_FRAME_SIZE};
use robotkit_device_protocol::{JogGate, JogVerdict};
use robotkit_device_virtual::VirtualDevice;
use std::cell::RefCell;
use std::rc::Rc;

const VECTORS: &str = include_str!("../../operator_link/vectors/profiles.txt");
const CONTROLLER: [u8; 16] = [7; 16];

fn profile() -> &'static CompiledProfile {
    let line = VECTORS.lines().find(|l| l.starts_with("drill-tap")).unwrap();
    let hex = line.split(' ').nth(2).unwrap();
    let bytes: Vec<u8> = (0..hex.len() / 2).map(|i| u8::from_str_radix(&hex[i * 2..i * 2 + 2], 16).unwrap()).collect();
    Box::leak(Box::new(CompiledProfile::from_record(&bytes).unwrap()))
}

struct Shared(Rc<RefCell<OperatorService<'static>>>);
impl JogGate for Shared {
    fn verdict(&mut self, now_us: u64) -> JogVerdict { self.0.borrow_mut().poll(now_us) }
}

fn send(device: &mut VirtualDevice, kind: u8, payload: &[u8]) {
    let mut frame = vec![0; MAX_FRAME_SIZE];
    let size = encode_frame6(kind, payload, &mut frame).unwrap();
    assert!(device.feed(&frame[..size]), "frame {kind} refused");
}

/// A device with one jog segment of `seconds` committed, moving at 0.5 units per second.
fn jogging_device(seconds: u64) -> VirtualDevice {
    let mut scale = [1.0; 64];
    scale[0] = 1_000.0;
    let mut device = VirtualDevice::new(1_000_000, 40_000, 0, 0, 1, scale, CONTROLLER, 1).unwrap();
    let begin = SessionBegin6 {
        session: 9, protocol_version: PROTOCOL_VERSION, expected_controller: CONTROLLER, actuator_count: 1,
        max_degree: 5, step_tick_hz: 40_000, max_acceleration: 10.0, actuator_max_acceleration: [10.0; 64],
        steps_per_unit: [1_000.0; 64], min_step_ticks: [1; 64], direction_setup_ticks: [0; 64],
        actuator_joint: [0; 64], actuator_ratio: [1.0; 64], dual_drive_skew_bound: [0.0; 64],
        link_loss_timeout_ns: 5_000_000_000, input_count: 0, input_actuator: [0; 64], input_active_high: 0,
        channel_count: 0, channel_id: [0; 1536], channel_kind: [0; 32], safe_digital: [0; 32],
        safe_analog: [0.0; 32], safe_argument: [0.0; 32], safe_command: [0; 1536], channel_stop_policy: [0; 32],
    };
    let mut body = vec![0; SessionBegin6::SIZE];
    begin.encode(&mut body).unwrap();
    send(&mut device, 1, &body);
    let mut queue = vec![0; QueueBegin6::SIZE];
    QueueBegin6 { queue_revision: 1, replace_after_ticks: 50_000, expected_position: [0.0; 64], expected_velocity: [0.0; 64], actuator_count: 1 }
        .encode(&mut queue).unwrap();
    send(&mut device, 5, &queue);
    let mut segment = vec![0; Segment6Header::SIZE + Segment6Coefficients::SIZE];
    Segment6Header { queue_revision: 1, plan_id: 8, t0_ticks: 50_000, duration_ticks: seconds * 1_000_000, degree: 1,
        actuator_count: 1, ends_at_rest: 1, purpose: 1 }.encode(&mut segment[..Segment6Header::SIZE]).unwrap();
    Segment6Coefficients { actuator: 0, base_steps: 0, c0: 0.0, c1: 0.5, c2: 0.0, c3: 0.0, c4: 0.0, c5: 0.0 }
        .encode(&mut segment[Segment6Header::SIZE..]).unwrap();
    send(&mut device, 6, &segment);
    let mut commit = [0; Commit6::SIZE];
    Commit6 { through_ticks: 50_000 + seconds * 1_000_000 }.encode(&mut commit).unwrap();
    send(&mut device, 7, &commit);
    device
}

/// Steps counted so far, from the latest State6 the device published.
fn steps(device: &mut VirtualDevice, last: &mut i64) -> i64 {
    while let Some(frame) = device.take_frame() {
        let (kind, payload) = decode_frame6(&frame).unwrap();
        if kind == 15 {
            *last = ActuatorState6::decode(&payload[State6Header::SIZE..State6Header::SIZE + ActuatorState6::SIZE]).unwrap().step_count;
        }
    }
    *last
}

fn pendant_state(sequence: u16) -> [u8; 8] {
    let mut out = [0u8; 8];
    // Position 3, x10, wheel enabled, chain closed: the pendant holds authority and keeps jogging.
    PendantState { sequence, wheel_delta: 1, position: 3, step: 1, buttons: 0, flags: 3 }.encode(&mut out).unwrap();
    out
}

/// Runs for `until_ms`; the pendant sends every 10 ms while ms < `pendant_until`. Returns the steps
/// at each of the sample times.
fn run(gate: bool, pendant_until: u64, samples: &[u64]) -> Vec<i64> {
    let mut device = jogging_device(5);
    let service = Rc::new(RefCell::new(OperatorService::new(profile())));
    if gate { device.set_jog_gate(Box::new(Shared(service.clone()))); }
    let (mut seq, mut last, mut out) = (0u16, 0i64, vec![]);
    for ms in 0..=*samples.last().unwrap() {
        if ms % 10 == 0 && ms < pendant_until {
            seq += 1;
            service.borrow_mut().on_pendant_frame(&pendant_state(seq), ms * 1000, 0).unwrap();
        }
        assert!(device.advance(ms * 1_000_000));
        if samples.contains(&ms) { out.push(steps(&mut device, &mut last)); }
    }
    out
}

#[test]
fn the_gated_device_stops_the_jog_when_the_pendant_lease_lapses_and_an_ungated_one_runs_on() {
    // Motion starts at 50 ms (0.5 units/s, 1000 steps/unit). The pendant goes silent after its frame
    // at 490 ms; the 50 ms lease ends after 540 ms, then the device decelerates (10 units/s^2).
    let gated = run(true, 500, &[100, 400, 540, 700, 1200]);
    let free = run(false, 500, &[100, 400, 540, 700, 1200]);
    assert!(gated[0] > 0 && gated[1] > gated[0], "the jog starts under the pendant: {gated:?}");
    assert_eq!(gated[3], gated[4], "stopped and holding at rest: {gated:?}");
    assert!(gated[4] < 300, "stopped near 540 ms (about 245 steps), not later: {gated:?}");
    assert!(free[4] > 550, "without the gate the same jog runs on: {free:?}");
    assert!(free[4] > 2 * gated[4]);
}

#[test]
fn a_pendant_that_keeps_sending_is_never_stopped() {
    let steady = run(true, 10_000, &[100, 600, 1200]);
    assert!(steady[1] > steady[0] && steady[2] > steady[1], "keeps moving: {steady:?}");
}
