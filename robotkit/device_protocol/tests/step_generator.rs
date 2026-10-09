#![cfg(feature = "std")]
use robotkit_device_protocol::{Board, Output, SkewGroup, StepFault, StepGenerator, VirtualBoard};

#[test]
fn captured_integer_counter_does_not_retract_after_f32_conversion() {
    let mut board = VirtualBoard::<1, 1>::new(1_000_000, 0, 0, [1000.0]);
    let mut generator = StepGenerator::new([1000.0], [0], [1]).unwrap();
    for _ in 0..166 { board.step_pulse(0, true); }
    board.advance_host_ns(1_000_000);
    let held = generator.counter_position(&board, 0).unwrap() as f32;
    assert!((held as f64 * 1000.0) < 166.0);
    generator.tick(&mut board, [held]).unwrap();
    assert_eq!(board.step_count(0), 166);
    board.advance_host_ns(2_000_000);
    generator.tick(&mut board, [0.1655]).unwrap();
    assert_eq!(board.step_count(0), 165);
}

#[test]
fn crossing_setup_and_rate_limit() {
    // A 40 kHz step tick on a 1 MHz board: one call every 25 board ticks. Direction setup is 50
    // board ticks; at most one step every 10 step ticks (250 board ticks).
    let mut board = VirtualBoard::<1, 1>::new(1_000_000, 0, 0, [400.0]);
    let mut generator = StepGenerator::new([400.0], [50], [10]).unwrap();
    let mut step_at = |micros: u64, board: &mut VirtualBoard<1, 1>| {
        board.advance_host_ns(micros * 1_000);
        generator.tick(board, [0.01]).unwrap();
        board.step_count(0)
    };
    assert_eq!(step_at(1_000, &mut board), 0);
    assert_eq!(step_at(1_025, &mut board), 0);
    assert_eq!(step_at(1_050, &mut board), 1);
    for micros in (1_075..1_300).step_by(25) { assert_eq!(step_at(micros, &mut board), 1); }
    assert_eq!(step_at(1_300, &mut board), 2);
    let steps: Vec<_> = board.records().iter().filter(|r| matches!(r.output, Output::Step(..))).collect();
    assert!(steps[1].ticks - steps[0].ticks >= 250);
}

#[test]
fn feedback_skew_latches_fault() {
    let mut board = VirtualBoard::<2, 1>::new(1_000_000, 0, 0, [400.0, 400.0]);
    let mut generator = StepGenerator::new([400.0; 2], [0; 2], [1; 2]).unwrap();
    assert!(generator.set_skew_group(SkewGroup { first: 0, second: 1,
        first_ratio: 1.0, second_ratio: 1.0, bound: 0.001 }));
    board.advance_host_ns(1_000_000);
    assert_eq!(generator.tick(&mut board, [0.01, 0.01]), Ok(()));
    board.step_pulse(0, true);
    assert_eq!(generator.tick(&mut board, [0.01, 0.01]), Err(StepFault::DualDriveSkew));
}

#[test]
fn lead_screw_crosses_400_steps_per_mm() {
    let mut board = VirtualBoard::<1, 1>::new(1_000_000, 0, 0, [400_000.0]);
    let mut generator = StepGenerator::new([400_000.0], [0], [10]).unwrap();
    for tick in 1..=4_000 {
        let host_ns = tick * 25_000;
        board.advance_host_ns(host_ns);
        let position = host_ns as f32 * 0.01 / 1e9;
        generator.tick(&mut board, [position]).unwrap();
        let expected = (position as f64 * 400_000.0) as i64;
        assert!((board.step_count(0) - expected).abs() <= 1);
    }
}

#[test]
fn homing_side_hold_preserves_alignment_and_requires_homing_purpose() {
    let mut board = VirtualBoard::<3, 1>::new(1000, 0, 0, [1000.0; 3]);
    let mut generator = StepGenerator::new([1000.0; 3], [0; 3], [1; 3]).unwrap();
    assert!(generator.set_skew_group(SkewGroup {
        first: 0, second: 1, first_ratio: 1.0, second_ratio: 1.0, bound: 0.001,
    }));
    assert!(!generator.hold_homing_side(&board, 0));
    assert!(generator.begin_homing_pair(0, 1, 0.02));
    assert!(!generator.begin_homing_pair(0, 1, 0.02));
    assert!(!generator.hold_homing_side(&board, 2));
    assert!(generator.hold_homing_side(&board, 0));
    for tick in 1..=3 {
        board.advance_host_ns(tick * 1_000_000);
        generator.tick_with_purpose(&mut board, [0.003, 0.003, 0.0], 2).unwrap();
    }
    assert_eq!(board.step_count(0), 0);
    assert_eq!(board.step_count(1), 3);
    assert!(generator.release_homing_side(0));
    board.advance_host_ns(4_000_000);
    generator.tick_with_purpose(&mut board, [0.003, 0.003, 0.0], 2).unwrap();
    assert_eq!(board.step_count(0), 0);
    assert_eq!(generator.tick(&mut board, [0.003, 0.003, 0.0]), Err(StepFault::InvalidHomingPurpose));
    assert!(!generator.hold_homing_side(&board, 0));
    // Invalid-purpose cleanup restored the normal skew bound.
    assert_eq!(generator.tick(&mut board, [0.003, 0.003, 0.0]), Err(StepFault::DualDriveSkew));
}

#[test]
fn independent_counter_rebase_is_atomic_and_preserves_physical_steps() {
    let mut board = VirtualBoard::<2, 1>::new(1000, 0, 0, [1000.0; 2]);
    let mut generator = StepGenerator::new([1000.0; 2], [0; 2], [1; 2]).unwrap();
    assert!(generator.set_skew_group(SkewGroup {
        first: 0, second: 1, first_ratio: 1.0, second_ratio: 1.0, bound: 0.001,
    }));
    assert!(generator.begin_homing_pair(0, 1, 0.02));
    assert!(generator.hold_homing_side(&board, 0));
    for tick in 1..=3 {
        board.advance_host_ns(tick * 1_000_000);
        generator.tick_with_purpose(&mut board, [0.003; 2], 2).unwrap();
    }
    assert!(!generator.rebase_homing_counters(&[(0, 0.0), (1, 0.003)]));
    assert!(generator.release_homing_side(0));
    assert!(!generator.rebase_homing_counters(&[(0, 0.001), (1, f64::NAN)]));
    assert_eq!(generator.counter_position(&board, 0), Some(0.0));
    assert_eq!(generator.counter_position(&board, 1), Some(0.003));
    assert!(!generator.rebase_homing_counters(&[(0, 0.001), (0, 0.002)]));
    assert!(generator.rebase_homing_counters(&[(0, 0.0), (1, 0.003)]));
    assert_eq!(generator.counter_position(&board, 0), Some(0.0));
    assert_eq!(generator.counter_position(&board, 1), Some(0.0));
    generator.end_homing_pair();
    board.advance_host_ns(4_000_000);
    generator.tick(&mut board, [0.003; 2]).unwrap();
    assert_eq!(board.step_count(0), 0);
    assert_eq!(board.step_count(1), 3);
}

#[test]
fn stopped_pair_queue_handoff_preserves_steps_and_uses_the_shared_leader() {
    let mut board = VirtualBoard::<2, 1>::new(1000, 0, 0, [1000.0; 2]);
    let mut generator = StepGenerator::new([1000.0; 2], [0; 2], [1; 2]).unwrap();
    assert!(generator.set_skew_group(SkewGroup {
        first: 0, second: 1, first_ratio: 1.0, second_ratio: 1.0, bound: 0.01,
    }));
    assert!(generator.begin_homing_pair(0, 1, 0.02));
    assert!(generator.hold_homing_side(&board, 0));
    for tick in 1..=4 {
        board.advance_host_ns(tick * 1_000_000);
        generator.tick_with_purpose(&mut board, [0.004; 2], 2).unwrap();
    }
    assert!(generator.release_homing_side(0));
    assert_eq!((board.step_count(0), board.step_count(1)), (0, 4));
    let targets = generator.stopped_targets(&board, &[0, 0], &[1.0, 1.0], 2).unwrap();
    assert_eq!(targets, [0.002; 2]);
    assert!(generator.anchor_stopped_targets(&board, targets));
    board.advance_host_ns(5_000_000);
    generator.tick_with_purpose(&mut board, targets, 2).unwrap();
    assert_eq!((board.step_count(0), board.step_count(1)), (0, 4));
    assert!(generator.rebase_homing_counters(&[(0, 0.0), (1, 0.004)]));
    let rebased = generator.stopped_targets(&board, &[0, 0], &[1.0, 1.0], 2).unwrap();
    assert_eq!(rebased, [0.0; 2]);
    assert!(generator.anchor_stopped_targets(&board, rebased));
    generator.end_homing_pair();
    for tick in 6..=9 {
        board.advance_host_ns(tick * 1_000_000);
        generator.tick(&mut board, rebased).unwrap();
    }
    assert_eq!((board.step_count(0), board.step_count(1)), (0, 4));
    for tick in 10..=12 {
        board.advance_host_ns(tick * 1_000_000);
        generator.tick(&mut board, [0.003; 2]).unwrap();
    }
    assert_eq!((board.step_count(0), board.step_count(1)), (3, 7));
}
