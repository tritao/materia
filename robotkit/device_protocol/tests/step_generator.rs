#![cfg(feature = "std")]
use robotkit_device_protocol::{Board, Output, SkewGroup, StepFault, StepGenerator, VirtualBoard};

#[test]
fn crossing_setup_and_rate_limit() {
    let mut board = VirtualBoard::<1, 1>::new(1_000_000, 0, 0, [400.0]);
    let mut generator = StepGenerator::new([400.0], [50], [10.0], 1_000_000).unwrap();
    board.advance_host_ns(1_000_000);
    generator.tick(&mut board, [0.01]).unwrap();
    assert_eq!(board.step_count(0), 0);
    board.advance_host_ns(1_040_000);
    generator.tick(&mut board, [0.01]).unwrap();
    assert_eq!(board.step_count(0), 0);
    board.advance_host_ns(1_050_000);
    generator.tick(&mut board, [0.01]).unwrap();
    assert_eq!(board.step_count(0), 1);
    board.advance_host_ns(1_100_000);
    generator.tick(&mut board, [0.01]).unwrap();
    assert_eq!(board.step_count(0), 1);
    board.advance_host_ns(1_300_000);
    generator.tick(&mut board, [0.01]).unwrap();
    assert_eq!(board.step_count(0), 2);
    let steps: Vec<_> = board.records().iter().filter(|r| matches!(r.output, Output::Step(..))).collect();
    assert!(steps[1].ticks - steps[0].ticks >= 250);
}

#[test]
fn feedback_skew_latches_fault() {
    let mut board = VirtualBoard::<2, 1>::new(1_000_000, 0, 0, [400.0, 400.0]);
    let mut generator = StepGenerator::new([400.0; 2], [0; 2], [0.0; 2], 1_000_000).unwrap();
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
    let mut generator = StepGenerator::new([400_000.0], [0], [0.01], 1_000_000).unwrap();
    for tick in 1..=4_000 {
        let host_ns = tick * 25_000;
        board.advance_host_ns(host_ns);
        let position = host_ns as f32 * 0.01 / 1e9;
        generator.tick(&mut board, [position]).unwrap();
        let expected = (position as f64 * 400_000.0) as i64;
        assert!((board.step_count(0) - expected).abs() <= 1);
    }
}
