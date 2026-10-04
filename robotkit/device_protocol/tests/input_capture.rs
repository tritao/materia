#![cfg(feature = "std")]
use robotkit_device_protocol::{Board, InputBinding, StepGenerator, VirtualBoard, VirtualSwitch};

#[test]
fn edge_capture_survives_later_motion_and_counts_repeated_closures() {
    let mut board = VirtualBoard::<2, 1>::new(1000, 0, 0, [1000.0; 2]);
    assert!(board.configure_switch(3, VirtualSwitch {
        actuator: 1, threshold_steps: 2, active_above: true, active_high: false,
    }));
    let mut steps = StepGenerator::new([1000.0; 2], [0; 2], [0.0; 2], 1000).unwrap();
    assert!(steps.bind_input(&board, 3, InputBinding { actuator: 1, active_high: false }, 2));
    assert!(!steps.bind_input(&board, 64, InputBinding { actuator: 0, active_high: true }, 2));
    assert!(!steps.bind_input(&board, 4, InputBinding { actuator: 2, active_high: true }, 2));
    assert!(steps.input_observation(4).is_none());
    board.advance_host_ns(1_000_000);
    steps.tick(&mut board, [0.01, 0.01]).unwrap();
    assert_eq!(steps.input_observation(3).unwrap().closing_count, 0);
    board.advance_host_ns(2_000_000);
    steps.tick(&mut board, [0.01, 0.01]).unwrap();
    let edge = steps.input_observation(3).unwrap();
    assert!(edge.active);
    assert_eq!(edge.closing_count, 1);
    assert_eq!(edge.captured_steps, 2);
    assert_eq!(edge.captured_ticks, 2);
    board.advance_host_ns(3_000_000);
    steps.tick(&mut board, [0.01, 0.01]).unwrap();
    assert_eq!(board.step_count(1), 3);
    assert_eq!(steps.input_observation(3).unwrap(), edge);
    for tick in 4..=5 {
        board.advance_host_ns(tick * 1_000_000);
        steps.tick(&mut board, [0.01, 0.0]).unwrap();
    }
    let opened = steps.input_observation(3).unwrap();
    assert!(!opened.active);
    assert_eq!(opened.opening_count, 1);
    assert_eq!(opened.captured_steps, 2);
    board.advance_host_ns(6_000_000);
    steps.tick(&mut board, [0.01, 0.01]).unwrap();
    let second = steps.input_observation(3).unwrap();
    assert_eq!(second.closing_count, 2);
    assert_eq!(second.captured_steps, 2);
    assert_eq!(second.captured_ticks, 6);
}

#[test]
fn initially_closed_input_does_not_create_a_capture() {
    let mut board = VirtualBoard::<1, 1>::new(1000, 0, 0, [1000.0]);
    assert!(board.set_input(0, true));
    let mut steps = StepGenerator::new([1000.0], [0], [0.0], 1000).unwrap();
    assert!(steps.bind_input(&board, 0, InputBinding { actuator: 0, active_high: true }, 1));
    steps.tick(&mut board, [0.0]).unwrap();
    let observed = steps.input_observation(0).unwrap();
    assert!(observed.active);
    assert_eq!(observed.closing_count, 0);
    assert!(board.set_input(0, false));
    steps.tick(&mut board, [0.0]).unwrap();
    assert_eq!(steps.input_observation(0).unwrap().opening_count, 1);
}
