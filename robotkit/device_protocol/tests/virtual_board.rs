#![cfg(feature = "std")]

use robotkit_device_protocol::{Board, VirtualBoard};

#[test]
fn virtual_clock_applies_offset_and_drift() {
    let mut board = VirtualBoard::<2, 2>::new(1_000_000, 50_000, 1_000, [200.0, 400.0]);
    board.advance_host_ns(1_000_000_000);
    assert_eq!(board.now_ticks(), 1_051_000);
    assert_eq!(board.tick_hz(), 1_000_000);
    board.step_pulse(0, true);
    board.step_pulse(1, false);
    assert_eq!(board.step_counts(), [1, -1]);
    assert_eq!(board.actuator_positions(), [0.005, -0.0025]);
    assert_eq!(board.records()[0].ticks, 1_051_000);
}

/// A stop halts motion; the channels are the device events' to make safe, by each one's policy.
#[test]
fn stop_halts_motion_and_leaves_channels_to_the_events() {
    let mut board = VirtualBoard::<1, 2>::new(1_000_000, 0, 0, [100.0]);
    board.velocity_target(0, 0.5);
    board.set_digital(0, true);
    board.set_analog(1, 0.75);
    board.stop_all();
    assert_eq!(board.velocity_targets(), [0.0]);
    assert_eq!(board.digital(0), Some(true));
    assert_eq!(board.analog(1), Some(0.75));
}

#[test]
fn ten_seconds_of_output_uses_bounded_storage_and_active_actuators() {
    let mut board = VirtualBoard::<64, 0>::new_with_actuator_count(
        1_000_000, 0, 0, [1_000.0; 64], 1);
    for cycle in 0..400_000 {
        board.advance_host_ns(cycle * 25_000);
        board.position_target(0, cycle as f32 / 1_000.0);
        board.velocity_target(0, 1.0);
        board.step_pulse(0, true);
        board.position_target(63, 1.0); // inactive physical channel
    }
    assert!(board.records().len() <= 65_536);
    assert!(board.step_records().len() <= 65_536);
    assert!(board.records().iter().all(|r| match r.output {
        robotkit_device_protocol::Output::Position(i, _) |
        robotkit_device_protocol::Output::Velocity(i, _) => i == 0,
        _ => true,
    }));
}

#[test]
fn switch_inputs_follow_actual_steps_and_declared_polarity() {
    use robotkit_device_protocol::{Board, VirtualBoard, VirtualSwitch};
    let mut board = VirtualBoard::<2, 1>::new(1_000, 0, 0, [1000.0; 2]);
    assert!(board.configure_switch(5, VirtualSwitch {
        actuator: 1, threshold_steps: 2, active_above: true, active_high: true,
    }));
    assert!(board.configure_switch(6, VirtualSwitch {
        actuator: 1, threshold_steps: 2, active_above: true, active_high: false,
    }));
    assert!(!board.read_input(5));
    assert!(board.read_input(6));
    board.step_pulse(0, true);
    assert!(!board.read_input(5));
    assert!(board.miss_next_steps(1, 1));
    board.step_pulse(1, true);
    assert_eq!(board.step_count(1), 0);
    assert!(!board.read_input(5));
    board.step_pulse(1, true);
    assert!(!board.read_input(5));
    board.step_pulse(1, true);
    assert!(board.read_input(5));
    assert!(!board.read_input(6));
    board.step_pulse(1, false);
    assert!(!board.read_input(5));
    assert!(board.read_input(6));
    assert!(!board.configure_switch(64, VirtualSwitch {
        actuator: 0, threshold_steps: 0, active_above: true, active_high: true,
    }));
    assert!(!board.configure_switch(1, VirtualSwitch {
        actuator: 2, threshold_steps: 0, active_above: true, active_high: true,
    }));
    assert!(board.set_input(5, true));
    assert!(board.read_input(5));
}
