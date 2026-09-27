#![cfg(feature = "std")]
use robotkit_device_protocol::device_wire6::{Event6, SessionBegin6};
use robotkit_device_protocol::{DeviceEvents, Output, VirtualBoard};

fn session() -> SessionBegin6 {
    let mut session = SessionBegin6::decode(&[0; SessionBegin6::SIZE]).unwrap();
    session.channel_count = 1;
    session.channel_kind[0] = 1;
    session.channel_id[..6].copy_from_slice(b"spray\0");
    session
}

fn digital_event(revision: u64, path_ticks: u64, value: bool) -> Event6 {
    let mut event = Event6::decode(&[0; Event6::SIZE]).unwrap();
    event.queue_revision = revision;
    event.plan_id = 7;
    event.path_ticks = path_ticks;
    event.channel = 0;
    event.kind = 1;
    event.hold_policy = 2;
    event.digital = value as u8;
    event
}

#[test]
fn event_pair_follows_path_clock_across_hold_and_stop() {
    let mut board = VirtualBoard::<1, 32>::new(1_000_000, 0, 0, [1.0]);
    let mut events = DeviceEvents::<4>::new(&session());
    events.queue_begin(1, 0, 0).unwrap();
    events.push(digital_event(1, 100, true)).unwrap();
    events.push(digital_event(1, 300, false)).unwrap();
    events.commit(400);
    board.advance_host_ns(99_000);
    events.tick(99, &mut board);
    assert_eq!(board.digital(0), Some(false));
    board.advance_host_ns(100_000);
    events.tick(100, &mut board);
    assert_eq!(board.digital(0), Some(true));
    events.hold(&mut board);
    assert_eq!(board.digital(0), Some(false));
    board.advance_host_ns(200_000);
    events.tick(100, &mut board);
    assert_eq!(board.digital(0), Some(false));
    events.resume(&mut board);
    assert_eq!(board.digital(0), Some(true));
    board.advance_host_ns(299_000);
    events.tick(299, &mut board);
    assert_eq!(board.digital(0), Some(true));
    board.advance_host_ns(300_000);
    events.tick(300, &mut board);
    assert_eq!(board.digital(0), Some(false));
    let fired: Vec<_> = board.records().iter().filter(|r|
        matches!(r.output, Output::Digital(_, _))).collect();
    assert_eq!(fired[0].ticks, 100);
    assert_eq!(fired[3].ticks, 300);
    events.stop(&mut board);
    assert_eq!(board.digital(0), Some(false));
}

#[test]
fn replacement_discards_uncommitted_events() {
    let mut board = VirtualBoard::<1, 32>::new(1_000_000, 0, 0, [1.0]);
    let mut events = DeviceEvents::<4>::new(&session());
    events.queue_begin(1, 0, 0).unwrap();
    events.push(digital_event(1, 100, true)).unwrap();
    events.push(digital_event(1, 300, false)).unwrap();
    events.commit(200);
    events.tick(100, &mut board);
    assert_eq!(board.digital(0), Some(true));
    events.queue_begin(2, 200, 200).unwrap();
    events.commit(400);
    events.tick(400, &mut board);
    assert_eq!(board.digital(0), Some(true));
    events.stop(&mut board);
    assert_eq!(board.digital(0), Some(false));
}
