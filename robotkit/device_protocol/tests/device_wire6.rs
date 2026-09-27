use robotkit_device_protocol::device_wire6::{ActuatorLimit6, SessionBegin6, Segment6Header, TimeSyncRequest};
use robotkit_device_protocol::frame6::{decode_frame6, encode_frame6, Frame6Error, MAX_FRAME_SIZE};

#[test]
fn rkd6_records_round_trip() {
    let begin = SessionBegin6 { session: 7, protocol_version: 6, model_fingerprint: [3; 16],
        actuator_count: 2, max_degree: 5, step_tick_hz: 40_000,
        link_loss_ticks: 500_000, max_acceleration: 4.0 };
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
    assert_eq!(encode_frame6(16, &[], &mut frame), Err(Frame6Error::BadPayload));
    assert_eq!(encode_frame6(99, &[], &mut frame), Err(Frame6Error::BadType));
    for kind in 8..=13 {
        assert_eq!(encode_frame6(kind, &[1], &mut frame), Err(Frame6Error::BadLength));
    }
    for (kind, size) in [(2, 44), (3, 8), (4, 24), (5, 529),
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
    let begin = SessionBegin6 { session: 7, protocol_version: 6,
        model_fingerprint: [3; 16], actuator_count: 2, max_degree: 5,
        step_tick_hz: 40_000, link_loss_ticks: 500_000, max_acceleration: 4.0 };
    let mut body = [0; SessionBegin6::SIZE + 2 * ActuatorLimit6::SIZE];
    begin.encode(&mut body[..SessionBegin6::SIZE]).unwrap();
    ActuatorLimit6 { max_acceleration: 2.0 }
        .encode(&mut body[SessionBegin6::SIZE..SessionBegin6::SIZE + ActuatorLimit6::SIZE]).unwrap();
    ActuatorLimit6 { max_acceleration: 4.0 }
        .encode(&mut body[SessionBegin6::SIZE + ActuatorLimit6::SIZE..]).unwrap();
    let mut frame = [0; MAX_FRAME_SIZE];
    let size = encode_frame6(1, &body, &mut frame).unwrap();
    assert_eq!(decode_frame6(&frame[..size]), Ok((1, &body[..])));
    assert_eq!(encode_frame6(1, &body[..body.len() - 1], &mut frame),
               Err(Frame6Error::BadPayload));
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
            _ => panic!("unknown vector") };
        assert_eq!(kind, expected);
        let mut encoded = [0; MAX_FRAME_SIZE];
        let size = encode_frame6(kind, payload, &mut encoded).unwrap();
        assert_eq!(&encoded[..size], bytes);
    }
}
