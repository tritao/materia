//! End to end: a host-planned jog segment executing on the real ScheduledCore, gated by the
//! operator service. The pendant lease, the host link and a running program each change what the
//! core is allowed to do.

use cabinet_node::{core_conditions, enforce_jog, OperatorService, ticks_to_us, PURPOSE_JOG, PURPOSE_PROGRAM};
use operator_link::profile::CompiledProfile;
use operator_link::operator_frame;
use operator_link::operator_wire::{OperatorMessage, OperatorState, PendantState};
use robotkit_device_protocol::{Board, ScheduledCore, ScheduledSegment, StopReason};

const VECTORS: &str = include_str!("../../operator_link/vectors/profiles.txt");

#[derive(Default)]
struct TestBoard { tick: u64, stopped: bool }
impl Board for TestBoard {
    fn read_input(&self, _: usize) -> bool { false }
    fn now_ticks(&self) -> u64 { self.tick }
    fn tick_hz(&self) -> u64 { 1_000 }
    fn position_target(&mut self, _: usize, _: f64) {}
    fn velocity_target(&mut self, _: usize, _: f32) {}
    fn step_pulse(&mut self, _: usize, _: bool) {}
    fn step_count(&self, _: usize) -> i64 { 0 }
    fn set_digital(&mut self, _: usize, _: bool) {}
    fn set_analog(&mut self, _: usize, _: f32) {}
    fn stop_all(&mut self) { self.stopped = true; }
}

fn profile(name: &str) -> CompiledProfile {
    let line = VECTORS.lines().find(|l| l.starts_with(name)).unwrap();
    let hex = line.split(' ').nth(2).unwrap();
    let bytes: Vec<u8> = (0..hex.len() / 2).map(|i| u8::from_str_radix(&hex[i * 2..i * 2 + 2], 16).unwrap()).collect();
    CompiledProfile::from_record(&bytes).unwrap()
}

fn segment(purpose: u8) -> ScheduledSegment<1> {
    ScheduledSegment::new_with_purpose(1, 0, 3_000, 1, [[0.0, 1.0, 0.0, 0.0, 0.0, 0.0]], true, purpose).unwrap()
}

fn core_with(purpose: u8) -> ScheduledCore<1, 4> {
    // The host link is alive for the whole test: its own lease is not what is under test here.
    let mut core = ScheduledCore::<1, 4>::new(1_000, [10.0], [-10.0], [10.0], 1_000_000);
    core.queue_begin(1, 0).unwrap();
    core.push_segment(segment(purpose)).unwrap();
    core.commit(3_000).unwrap();
    core
}

fn pendant_frame(sequence: u16, position: u8, delta: i16, enable: bool) -> [u8; 8] {
    let mut out = [0u8; 8];
    PendantState { sequence, wheel_delta: delta, position, step: 1, buttons: 0, flags: if enable { 3 } else { 2 } }
        .encode(&mut out).unwrap();
    out
}

fn host_frame(sequence: u16, allowed: bool) -> Vec<u8> {
    let mut payload = [0u8; 4];
    OperatorState { sequence, jog_allowed: allowed as u8, reserved: 0 }.encode(&mut payload).unwrap();
    let mut frame = vec![0u8; operator_frame::MAX_FRAME_SIZE];
    let n = operator_frame::encode(OperatorMessage::State as u8, &payload, &mut frame).unwrap();
    frame.truncate(n);
    frame
}

/// Runs the node for `until` ms. The pendant sends every 10 ms while `pendant_until` has not
/// passed; the host sends an allowing OperatorState every 100 ms until `host_allows_until`, then goes silent.
struct Run { board: TestBoard, core: ScheduledCore<1, 4>, service_stopped_at: Option<u64>, events: Vec<operator_link::operator_wire::PendantEvent> }

fn run(purpose: u8, position: u8, pendant_until: u64, host_allows_until: Option<u64>, until: u64) -> Run {
    let compiled = profile("drill-tap");
    let mut service = OperatorService::new(&compiled);
    let mut r = Run { board: TestBoard::default(), core: core_with(purpose), service_stopped_at: None, events: vec![] };
    let mut seq = 0u16;
    let mut host_seq = 0u16;
    for ms in 0..=until {
        r.board.tick = ms;
        let now = ticks_to_us(&r.board);
        r.core.note_host_frame(r.board.tick);
        if let Some(h) = host_allows_until {
            if ms % 100 == 0 && ms < h {
                host_seq += 1;
                service.on_operator_frame(&host_frame(host_seq, true), now).unwrap();
            }
        }
        if ms % 10 == 0 && ms < pendant_until {
            seq += 1;
            let active = core_conditions(&r.core);
            if let Some(e) = service.on_pendant_frame(&pendant_frame(seq, position, 5, true), now, active).unwrap() { r.events.push(e); }
        }
        let verdict = service.poll(now);
        if enforce_jog(&mut r.core, &verdict) && r.service_stopped_at.is_none() { r.service_stopped_at = Some(ms); }
        r.core.tick(&mut r.board);
    }
    r
}

#[test]
fn a_pendant_jog_stops_when_the_pendant_lease_lapses() {
    let r = run(PURPOSE_JOG, 3, 500, None, 800);
    // The last frame arrived at 490 ms; the 50 ms lease ends after 540 ms.
    let at = r.service_stopped_at.expect("the jog must stop");
    assert!((541..=550).contains(&at), "stopped at {at} ms");
    assert_eq!(r.core.stop_reason(), Some(StopReason::Stop));
    assert!(r.core.is_stopped() && r.board.stopped, "it decelerates to rest");
}

#[test]
fn a_pendant_jog_stops_on_lease_loss_even_when_the_host_still_allows_jogging() {
    let r = run(PURPOSE_JOG, 3, 500, Some(10_000), 800);
    let at = r.service_stopped_at.expect("the jog must stop");
    assert!((541..=550).contains(&at), "stopped at {at} ms");
}

#[test]
fn with_the_switch_off_the_host_link_carries_the_jog_and_its_loss_stops_it() {
    // The pendant stays connected with the axis switch OFF; the host allows jogging until 1000 ms.
    let r = run(PURPOSE_JOG, 0, 10_000, Some(1_000), 1_600);
    let at = r.service_stopped_at.expect("the jog must stop");
    // The last allowing state arrived at 900 ms; its 250 ms freshness ends after 1150 ms.
    assert!((1151..=1160).contains(&at), "stopped at {at} ms");
    assert!(r.events.iter().all(|e| e.flags & 1 == 0), "OFF never holds authority");
}

#[test]
fn a_jog_with_no_permit_never_starts() {
    let r = run(PURPOSE_JOG, 0, 10_000, None, 100);
    assert_eq!(r.service_stopped_at, Some(0), "no pendant authority and no host state: stopped at once");
}

#[test]
fn programs_are_never_stopped_by_the_jog_gate() {
    let r = run(PURPOSE_PROGRAM, 3, 100, None, 800);
    assert_eq!(r.service_stopped_at, None);
    assert_eq!(r.core.stop_reason(), None);
}

#[test]
fn a_running_program_blocks_pendant_moves_and_the_event_says_why() {
    let r = run(PURPOSE_PROGRAM, 3, 300, None, 300);
    assert!(r.events.len() >= 5);
    assert!(r.events.iter().all(|e| e.distance == 0 && e.refusal == 4), "busy, no move");
}

#[test]
fn pendant_moves_reach_the_host_as_events() {
    // No program running (the jog segment is only the core's workload): wheel moves are forwarded.
    let compiled = profile("drill-tap");
    let mut service = OperatorService::new(&compiled);
    service.on_operator_frame(&host_frame(1, true), 0).unwrap();
    assert!(service.on_pendant_frame(&pendant_frame(1, 3, 0, true), 0, 0).unwrap().is_some());
    let e = service.on_pendant_frame(&pendant_frame(2, 3, 5, true), 10_000, 0).unwrap().unwrap();
    assert_eq!((e.axis, e.distance, e.flags & 1, e.refusal), (2, 50, 1, 0));
    assert_eq!(e.sequence, 2);
    assert!(service.on_pendant_frame(&pendant_frame(2, 3, 5, true), 20_000, 0).unwrap().is_none(), "duplicate");
}

#[test]
fn bad_frames_are_refused() {
    let compiled = profile("drill-tap");
    let mut service = OperatorService::new(&compiled);
    let mut bad = pendant_frame(1, 3, 0, true);
    bad[4] = 12; // position 12
    assert!(service.on_pendant_frame(&bad, 0, 0).is_err());
    let mut host = host_frame(1, true);
    host[10] ^= 1;
    assert!(service.on_operator_frame(&host, 0).is_err());
    let mut payload = [0u8; 16];
    let mut frame = vec![0u8; operator_frame::MAX_FRAME_SIZE];
    operator_link::operator_wire::PendantConfig { kind: 0, index: 1, text: [0; 12], reserved: [0; 2] }.encode(&mut payload).unwrap();
    let n = operator_frame::encode(OperatorMessage::Config as u8, &payload, &mut frame).unwrap();
    assert_eq!(service.on_operator_frame(&frame[..n], 0), Err(cabinet_node::ServiceError::WrongType));
}
