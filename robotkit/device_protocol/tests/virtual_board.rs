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

#[test]
fn stop_sets_channels_safe() {
    let mut board = VirtualBoard::<1, 2>::new(1_000_000, 0, 0, [100.0]);
    board.set_digital(0, true);
    board.set_analog(1, 0.75);
    board.stop_all();
    assert_eq!(board.digital(0), Some(false));
    assert_eq!(board.analog(1), Some(0.0));
}
