use robotkit_device_protocol::device_wire6::{Event6, SessionBegin6, Segment6Header, TimeSyncRequest};
use robotkit_device_protocol::frame6::{decode_frame6, encode_frame6,
    slide_to_frame_marker, Frame6Error, MAX_FRAME_SIZE};

#[test]
fn frame_sync_slides_across_a_bad_marker() {
    let mut input = [0u8; MAX_FRAME_SIZE];
    input[..5].copy_from_slice(b"RRKD6");
    let mut len = 5;
    slide_to_frame_marker(&mut input, &mut len);
    assert_eq!(len, 4);
    assert_eq!(&input[..4], b"RKD6");
}

#[test]
fn rkd6_records_round_trip() {
    let begin = SessionBegin6 { session: 7, protocol_version: 10, model_fingerprint: [3; 16],
        actuator_count: 2, max_degree: 5, step_tick_hz: 40_000,
        max_acceleration: 4.0, actuator_max_acceleration: [4.0; 64], steps_per_unit: [400.0; 64], max_rate: [0.0; 64], direction_setup_ticks: [0; 64], actuator_joint: [0; 64], actuator_ratio: [1.0; 64], dual_drive_skew_bound: [0.0; 64], link_loss_timeout_ns: 500_000_000,
        channel_count: 0, channel_id: [0; 1536], channel_kind: [0; 32],
        safe_digital: [0; 32], safe_analog: [0.0; 32],
        safe_argument: [0.0; 32], safe_command: [0; 1536] };
    let mut bytes = [0; SessionBegin6::SIZE];
    begin.encode(&mut bytes).unwrap();
    assert_eq!(SessionBegin6::decode(&bytes).unwrap(), begin);
    let sync = TimeSyncRequest { host_send_ns: 123456789 };
    let mut bytes = [0; TimeSyncRequest::SIZE];
    sync.encode(&mut bytes).unwrap();
    assert_eq!(TimeSyncRequest::decode(&bytes).unwrap(), sync);
    let segment = Segment6Header { queue_revision: 3, plan_id: 2,
        t0_ticks: 12_000, duration_ticks: 8_000, degree: 5,
        actuator_count: 2, ends_at_rest: 1, reserved: 0 };
    let mut bytes = [0; Segment6Header::SIZE];
    segment.encode(&mut bytes).unwrap();
    assert_eq!(Segment6Header::decode(&bytes).unwrap(), segment);
}

#[test]
fn frame_rejects_corruption_and_wrong_lengths() {
    let mut frame = [0; MAX_FRAME_SIZE];
    let size = encode_frame6(3, &[0; TimeSyncRequest::SIZE], &mut frame).unwrap();
    assert_eq!(decode_frame6(&frame[..size]), Ok((3, &[0; TimeSyncRequest::SIZE][..])));
    frame[8] ^= 1;
    assert_eq!(decode_frame6(&frame[..size]), Err(Frame6Error::BadCrc));
    assert_eq!(encode_frame6(3, &[0; 7], &mut frame), Err(Frame6Error::BadLength));
    assert_eq!(encode_frame6(16, &[], &mut frame), Err(Frame6Error::BadLength));
    assert_eq!(encode_frame6(99, &[], &mut frame), Err(Frame6Error::BadType));
    for kind in 8..=13 {
        assert_eq!(encode_frame6(kind, &[1], &mut frame), Err(Frame6Error::BadLength));
    }
    for (kind, size) in [(2, 45), (3, 8), (4, 24), (5, 529),
                         (7, 8), (14, 44)] {
        assert_eq!(encode_frame6(kind, &vec![0; size - 1], &mut frame),
                   Err(Frame6Error::BadLength));
    }
    for kind in [6, 15] {
        assert_eq!(encode_frame6(kind, &[], &mut frame), Err(Frame6Error::BadLength));
    }
}

#[test]
fn session_begin_carries_per_actuator_acceleration_limits() {
    let mut limits = [0.0; 64];
    limits[0] = 2.0;
    limits[1] = 4.0;
    let begin = SessionBegin6 { session: 7, protocol_version: 10,
        model_fingerprint: [3; 16], actuator_count: 2, max_degree: 5,
        step_tick_hz: 40_000, max_acceleration: 4.0,
        actuator_max_acceleration: limits, steps_per_unit: [400.0; 64], max_rate: [0.0; 64], direction_setup_ticks: [0; 64], actuator_joint: [0; 64], actuator_ratio: [1.0; 64], dual_drive_skew_bound: [0.0; 64], link_loss_timeout_ns: 500_000_000,
        channel_count: 0, channel_id: [0; 1536], channel_kind: [0; 32],
        safe_digital: [0; 32], safe_analog: [0.0; 32],
        safe_argument: [0.0; 32], safe_command: [0; 1536] };
    let mut body = [0; SessionBegin6::SIZE];
    begin.encode(&mut body).unwrap();
    let mut frame = [0; MAX_FRAME_SIZE];
    let size = encode_frame6(1, &body, &mut frame).unwrap();
    assert_eq!(decode_frame6(&frame[..size]), Ok((1, &body[..])));
    assert_eq!(encode_frame6(1, &body[..body.len() - 1], &mut frame),
               Err(Frame6Error::BadLength));
}

#[test]
fn event_frame_validates_type_and_value() {
    let mut event = Event6::decode(&[0; Event6::SIZE]).unwrap();
    event.queue_revision = 1;
    event.plan_id = 2;
    event.path_ticks = 123;
    event.kind = 1;
    event.digital = 1;
    let mut body = [0; Event6::SIZE];
    event.encode(&mut body).unwrap();
    let mut frame = [0; MAX_FRAME_SIZE];
    let size = encode_frame6(16, &body, &mut frame).unwrap();
    assert_eq!(decode_frame6(&frame[..size]), Ok((16, &body[..])));
    event.digital = 2;
    event.encode(&mut body).unwrap();
    assert_eq!(encode_frame6(16, &body, &mut frame), Err(Frame6Error::BadPayload));
}

#[test]
fn shared_frame_vectors() {
    let vectors = include_str!("../../schema/device_frame6_vectors.tsv");
    for line in vectors.lines() {
        let (name, hex) = line.split_once('\t').unwrap();
        let bytes: Vec<u8> = hex.as_bytes().chunks_exact(2)
            .map(|pair| u8::from_str_radix(core::str::from_utf8(pair).unwrap(), 16).unwrap())
            .collect();
        let (kind, payload) = decode_frame6(&bytes).unwrap();
        let expected = match name { "time_sync_request" => 3, "hold" => 8, "stop" => 11,
            "session_begin6_v10" => 1, "digital_event" => 16,
            _ => panic!("unknown vector") };
        assert_eq!(kind, expected);
        let mut encoded = [0; MAX_FRAME_SIZE];
        let size = encode_frame6(kind, payload, &mut encoded).unwrap();
        assert_eq!(&encoded[..size], bytes);
    }
}
