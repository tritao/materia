use robotkit_device_protocol::{Board, ScheduledCore, ScheduledSegment, QueueError, StopReason};

#[derive(Default)]
struct TestBoard { tick: u64, position: f32, velocity: f32, stopped: bool }
impl Board for TestBoard {
    fn now_ticks(&self) -> u64 { self.tick }
    fn tick_hz(&self) -> u64 { 1_000 }
    fn position_target(&mut self, _: usize, value: f32) { self.position = value; }
    fn velocity_target(&mut self, _: usize, value: f32) { self.velocity = value; }
    fn step_pulse(&mut self, _: usize, _: bool) {}
    fn step_count(&self, _: usize) -> i64 { 0 }
    fn set_digital(&mut self, _: usize, _: bool) {}
    fn set_analog(&mut self, _: usize, _: f32) {}
    fn stop_all(&mut self) { self.stopped = true; }
}
fn line(t0: u64, duration: u64, at_rest: bool) -> ScheduledSegment<1> {
    ScheduledSegment::new(1, t0, duration, 1, [[t0 as f32 / 1_000.0, 1.0, 0.0, 0.0, 0.0, 0.0]], at_rest).unwrap()
}
#[test]
fn replace_respects_committed_horizon() {
    let mut core = ScheduledCore::<1, 4>::new(1_000, [10.0], [-10.0], [10.0], 500);
    core.queue_begin(1, 0).unwrap();
    core.push_segment(line(0, 1_000, false)).unwrap();
    core.push_segment(line(1_000, 1_000, true)).unwrap();
    core.commit(1_000).unwrap();
    assert_eq!(core.queue_begin(2, 999), Err(QueueError::Committed));
    core.queue_begin(2, 1_000).unwrap();
    core.push_segment(line(1_000, 1_000, true)).unwrap();
    let jump = ScheduledSegment::new(1, 2_000, 1_000, 1,
        [[0.0, 1.0, 0.0, 0.0, 0.0, 0.0]], true).unwrap();
    assert_eq!(core.push_segment(jump), Err(QueueError::BadBoundary));
}
#[test]
fn evaluates_and_stops_on_link_loss() {
    let mut core = ScheduledCore::<1, 4>::new(1_000, [2.0], [-10.0], [10.0], 500);
    let mut board = TestBoard::default();
    core.queue_begin(1, 0).unwrap();
    core.push_segment(line(0, 2_000, true)).unwrap();
    core.commit(2_000).unwrap();
    board.tick = 500;
    core.tick(&mut board);
    assert!((board.position - 0.5).abs() < 1e-5);
    board.tick = 1_100;
    core.tick(&mut board);
    assert_eq!(core.stop_reason(), Some(StopReason::LinkLost));
    assert!(board.velocity >= 0.0 && board.velocity < 1.0);
    for tick in 1_101..2_000 { board.tick = tick; core.tick(&mut board); }
    assert_eq!(board.velocity, 0.0);
    assert!(board.stopped);
}

#[test]
fn hold_resume_respects_acceleration() {
    let mut core = ScheduledCore::<1, 4>::new(1_000, [2.0], [-10.0], [10.0], 5_000);
    let mut board = TestBoard::default();
    core.queue_begin(1, 0).unwrap();
    core.push_segment(line(0, 4_000, true)).unwrap();
    core.commit(4_000).unwrap();
    core.tick(&mut board);
    core.hold();
    for t in 1..=600 {
        board.tick = t;
        let previous = board.velocity;
        core.tick(&mut board);
        assert!((board.velocity - previous).abs() <= 0.00201,
                "t={t} previous={previous} current={}", board.velocity);
    }
    assert_eq!(core.rate(), 0.0);
    let held_at = core.path_clock();
    for t in 601..=650 { board.tick = t; core.tick(&mut board); }
    assert_eq!(core.path_clock(), held_at);
    core.resume();
    board.tick = 651;
    core.tick(&mut board);
    assert!(core.rate() > 0.0);
}

#[test]
fn continuation_underflows_but_declared_rest_does_not() {
    let mut board = TestBoard::default();
    let mut continuation = ScheduledCore::<1, 4>::new(1_000, [2.0], [-10.0], [10.0], 5_000);
    continuation.queue_begin(1, 0).unwrap();
    continuation.push_segment(line(0, 1_000, false)).unwrap();
    continuation.commit(1_000).unwrap();
    board.tick = 1_001;
    continuation.tick(&mut board);
    assert!(continuation.underflow());
    assert_eq!(continuation.stop_reason(), Some(StopReason::Underflow));
    let mut final_plan = ScheduledCore::<1, 4>::new(1_000, [2.0], [-10.0], [10.0], 5_000);
    final_plan.queue_begin(1, 0).unwrap();
    final_plan.push_segment(line(0, 1_000, true)).unwrap();
    final_plan.commit(1_000).unwrap();
    final_plan.tick(&mut board);
    assert!(!final_plan.underflow());
    assert_eq!(board.velocity, 0.0);
}

#[test]
fn controlled_stop_clamps_at_travel_limit() {
    let mut core = ScheduledCore::<1, 4>::new(1_000, [1.0], [0.0], [0.6], 500);
    let mut board = TestBoard::default();
    core.queue_begin(1, 0).unwrap();
    core.push_segment(line(0, 2_000, false)).unwrap();
    core.commit(2_000).unwrap();
    board.tick = 400;
    core.tick(&mut board);
    core.stop(StopReason::Stop);
    for t in 401..=1_000 { board.tick = t; core.tick(&mut board); }
    assert!(board.position <= 0.6);
    assert_eq!(board.velocity, 0.0);
}

#[test]
fn abort_uses_nearby_declared_rest_end() {
    let mut core = ScheduledCore::<1, 4>::new(1_000, [0.1], [-10.0], [10.0], 5_000);
    let mut board = TestBoard::default();
    core.queue_begin(1, 0).unwrap();
    core.push_segment(line(0, 1_000, true)).unwrap();
    core.commit(1_000).unwrap();
    board.tick = 500;
    core.tick(&mut board);
    core.abort();
    assert_eq!(core.stop_reason(), None);
    board.tick = 1_000;
    core.tick(&mut board);
    assert_eq!(core.stop_reason(), Some(StopReason::Abort));
    assert!(board.stopped);
    assert!((board.position - 1.0).abs() < 1e-5);
}

#[test]
fn uncommitted_segment_never_executes() {
    let mut core = ScheduledCore::<1, 4>::new(1_000, [2.0], [-10.0], [10.0], 5_000);
    let mut board = TestBoard::default();
    core.queue_begin(1, 0).unwrap();
    core.push_segment(line(0, 1_000, false)).unwrap();
    core.push_segment(line(1_000, 1_000, true)).unwrap();
    core.commit(1_000).unwrap();
    board.tick = 1_500;
    core.tick(&mut board);
    assert!(core.underflow());
    assert_eq!(core.stop_reason(), Some(StopReason::Underflow));
}

#[test]
fn future_device_tick_start_waits_for_its_boundary() {
    let mut core = ScheduledCore::<1, 4>::new(1_000, [2.0], [-10.0], [10.0], 5_000);
    let mut board = TestBoard::default();
    core.queue_begin(1, 1_000).unwrap();
    let segment = ScheduledSegment::new(1, 1_000, 1_000, 1,
        [[0.0, 1.0, 0.0, 0.0, 0.0, 0.0]], true).unwrap();
    core.push_segment(segment).unwrap();
    core.commit(2_000).unwrap();
    board.tick = 500;
    core.tick(&mut board);
    assert_eq!(board.velocity, 0.0);
    board.tick = 1_000;
    core.tick(&mut board);
    assert_eq!(board.velocity, 1.0);
}

#[test]
fn queue_begin_checks_expected_start_state() {
    let mut core = ScheduledCore::<1, 4>::new(1_000, [2.0], [-10.0], [10.0], 5_000);
    assert_eq!(core.queue_begin_with_state(1, 0, [1.0], [0.0]),
               Err(QueueError::BadExpectedState));
    core.queue_begin_with_state(1, 0, [0.0], [0.0]).unwrap();
}

#[test]
fn emergency_stop_latches_and_stops_outputs_immediately() {
    let mut core = ScheduledCore::<1, 4>::new(1_000, [2.0], [-10.0], [10.0], 5_000);
    let mut board = TestBoard::default();
    core.queue_begin(1, 0).unwrap();
    core.push_segment(line(0, 1_000, false)).unwrap();
    core.commit(1_000).unwrap();
    board.tick = 200;
    core.tick(&mut board);
    core.emergency_stop(&mut board);
    assert!(board.stopped);
    assert_eq!(core.stop_reason(), Some(StopReason::EmergencyStop));
    board.tick = 300;
    core.tick(&mut board);
    assert_eq!(core.path_clock(), 200);
}

#[test]
fn hold_accounts_for_path_acceleration() {
    let mut core = ScheduledCore::<1, 4>::new(1_000, [2.0], [-10.0], [10.0], 5_000);
    let mut board = TestBoard::default();
    core.queue_begin(1, 0).unwrap();
    let curve = ScheduledSegment::new(1, 0, 1_000, 2,
        [[0.0, 1.0, -0.5, 0.0, 0.0, 0.0]], true).unwrap();
    core.push_segment(curve).unwrap();
    core.commit(1_000).unwrap();
    core.tick(&mut board);
    core.hold();
    for t in 1..=100 {
        board.tick = t;
        let previous = board.velocity;
        core.tick(&mut board);
        assert!((board.velocity - previous).abs() <= 0.00201,
                "t={t} previous={previous} current={}", board.velocity);
    }
}

#[test]
fn hold_brakes_when_toppra_path_uses_full_acceleration() {
    let mut core = ScheduledCore::<1, 4>::new(1_000, [2.0], [-10.0], [10.0], 5_000);
    let mut board = TestBoard::default();
    core.queue_begin(1, 0).unwrap();
    // q=t² is at the full 2 units/s² acceleration limit.
    core.push_segment(ScheduledSegment::new(1, 0, 1_000, 2,
        [[0.0, 0.0, 1.0, 0.0, 0.0, 0.0]], true).unwrap()).unwrap();
    core.commit(1_000).unwrap();
    board.tick = 250;
    core.tick(&mut board);
    let rate_before = core.rate();
    assert!(rate_before > 0.0);
    core.hold();
    board.tick = 251;
    core.tick(&mut board);
    assert!(core.rate() < rate_before, "HOLD must brake in its first cycle");
}

#[test]
fn completed_rest_plan_does_not_fault_when_link_goes_idle() {
    let mut core = ScheduledCore::<1, 4>::new(1_000, [2.0], [-10.0], [10.0], 1_500);
    let mut board = TestBoard::default();
    core.queue_begin(1, 0).unwrap();
    core.push_segment(line(0, 1_000, true)).unwrap();
    core.commit(1_000).unwrap();
    board.tick = 1_000;
    core.tick(&mut board);
    board.tick = 2_000;
    core.tick(&mut board);
    assert_eq!(core.stop_reason(), None);
}
