use robotkit_device_protocol::*;

#[derive(Default)]
struct FakeDevice {
    stops: usize,
    resets: usize,
    applied: usize,
    accept_reset: bool,
    accept_targets: bool,
    state_safety: u8,
    bad_state: bool,
    joint_count: u8,
}

impl Device for FakeDevice {
    fn stop_all(&mut self) { self.stops += 1; }
    fn apply_targets(&mut self, targets: &[JointTarget]) -> bool {
        if self.accept_targets { self.applied += targets.len(); true } else { false }
    }
    fn reset_safety(&mut self) -> bool {
        self.resets += 1;
        self.accept_reset
    }
    fn read_state(&mut self, joints: &mut [JointState; MAX_JOINTS as usize]) -> StateMeta {
        joints[0] = JointState { position: if self.bad_state { f32::NAN } else { 1.0 }, velocity: -2.0, effort: 0.5 };
        StateMeta { timestamp_ns: 123456789, safety: self.state_safety, fault: 0,
            joint_count: if self.joint_count == 0 { 1 } else { self.joint_count } }
    }
}

fn fixture(name: &str) -> Vec<u8> {
    let text = include_str!("../../schema/device_frame_vectors.tsv");
    let hex = text.lines().filter_map(|line| line.split_once('\t'))
        .find(|(key, _)| *key == name).unwrap().1;
    hex.as_bytes().chunks_exact(2)
        .map(|pair| u8::from_str_radix(core::str::from_utf8(pair).unwrap(), 16).unwrap())
        .collect()
}

fn command(session: u64, sequence: u64, kind: u8, targets: &[JointTarget]) -> Vec<u8> {
    let mut payload = vec![0u8; CommandHeader::SIZE + targets.len() * JointTarget::SIZE];
    CommandHeader { session, sequence, kind, target_count: targets.len() as u8, reserved: 0 }
        .encode(&mut payload).unwrap();
    for (index, target) in targets.iter().enumerate() {
        let offset = CommandHeader::SIZE + index * JointTarget::SIZE;
        target.encode(&mut payload[offset..offset + JointTarget::SIZE]).unwrap();
    }
    let mut frame = vec![0u8; HEADER_SIZE + payload.len() + CRC_SIZE];
    encode_frame(3, &payload, &mut frame).unwrap();
    frame
}

#[test]
fn every_fragment_boundary_and_canonical_outputs() {
    let begin = fixture("session_begin");
    let reset = fixture("command_reset");
    let target = fixture("command_target");
    let mut stream = begin.clone();
    stream.extend_from_slice(&reset);
    stream.extend_from_slice(&target);
    let fingerprint = core::array::from_fn(|index| index as u8);
    for split in 0..=stream.len() {
        let mut core = DeviceProtocol::new(2, fingerprint).unwrap();
        let mut device = FakeDevice { accept_reset: true, accept_targets: true, joint_count: 2, ..Default::default() };
        let mut emitted = Vec::new();
        let first = core.feed(&stream[..split], 100, &mut device, |bytes| emitted.push(bytes.to_vec()));
        let second = core.feed(&stream[split..], 100, &mut device, |bytes| emitted.push(bytes.to_vec()));
        assert_eq!(first + second, 2, "split {split}");
        assert_eq!(device.stops, 1);
        assert_eq!(device.applied, 1);
        assert_eq!(core.last_sequence(), 2);
        assert!(!core.latched());
        assert_eq!(emitted.len(), 2);
        assert_eq!(emitted[0], fixture("session_ack"));
        let initial = StateHeader::decode(&emitted[1][HEADER_SIZE..HEADER_SIZE + StateHeader::SIZE]).unwrap();
        assert_eq!(initial.safety, 2);
        assert_eq!(initial.accepted_sequence, 0);
    }

    let mut core = DeviceProtocol::new(1, fingerprint).unwrap();
    let mut device = FakeDevice { accept_reset: true, state_safety: 1, ..Default::default() };
    core.feed(&begin, 10, &mut device, |_| {});
    let mut output = [0u8; MAX_FRAME_SIZE];
    device.state_safety = 0;
    let size = core.encode_state(&mut device, &mut output).unwrap();
    // A fresh session must report emergency-stop even if the adapter says ready.
    let state = StateHeader::decode(&output[HEADER_SIZE..HEADER_SIZE + StateHeader::SIZE]).unwrap();
    assert_eq!(state.safety, 2);
    assert_eq!(state.accepted_sequence, 0);
    assert_eq!(size, HEADER_SIZE + StateHeader::SIZE + JointState::SIZE + CRC_SIZE);
    assert_eq!(core.feed(&reset, 20, &mut device, |_| {}), 1);
    assert_eq!(core.feed(&command(0x0102_0304_0506_0708, 2, 2, &[]), 30, &mut device, |_| {}), 1);
    device.state_safety = 1;
    let size = core.encode_state(&mut device, &mut output).unwrap();
    assert_eq!(&output[..size], fixture("state"));
    device.bad_state = true;
    assert_eq!(core.encode_state(&mut device, &mut output), Err(FrameError::InvalidState));
}

#[test]
fn rejected_traffic_never_changes_watermark_or_watchdog() {
    let session = 0x0102_0304_0506_0708;
    let fingerprint = core::array::from_fn(|index| index as u8);
    let mut core = DeviceProtocol::new(2, fingerprint).unwrap();
    let mut device = FakeDevice { accept_reset: true, accept_targets: true, joint_count: 2, ..Default::default() };
    let begin = fixture("session_begin");
    let reset = fixture("command_reset");
    let target = fixture("command_target");
    core.feed(&begin, 10, &mut device, |_| {});
    assert_eq!(core.feed(&target, 11, &mut device, |_| {}), 0); // Latched.
    assert!(core.watchdog_expired(11, 100));
    assert_eq!(core.feed(&reset, 20, &mut device, |_| {}), 1);
    assert_eq!(core.feed(&target, 30, &mut device, |_| {}), 1);
    assert_eq!(core.last_sequence(), 2);

    let invalid = [
        command(session, 2, 2, &[]), // replay
        command(session + 1, 3, 2, &[]), // wrong session
        command(session, 3, 1, &[JointTarget { joint: 0, mode: 2, flags: 0, value: f32::NAN }]),
        command(session, 3, 1, &[JointTarget { joint: 0, mode: 2, flags: 0, value: 1.0 }, JointTarget { joint: 0, mode: 2, flags: 0, value: 2.0 }]),
        command(session, 3, 1, &[JointTarget { joint: 0, mode: 9, flags: 0, value: 1.0 }]),
    ];
    for bytes in &invalid { assert_eq!(core.feed(bytes, 140, &mut device, |_| {}), 0); }
    let mut corrupt = command(session, 3, 2, &[]);
    *corrupt.last_mut().unwrap() ^= 0x40;
    core.feed(&corrupt, 140, &mut device, |_| {});
    let mut oversized = b"garbageRKD5".to_vec();
    oversized.extend_from_slice(&[3, 0, 0xff, 0xff]);
    oversized.extend_from_slice(&corrupt);
    core.feed(&oversized, 140, &mut device, |_| {});
    assert!(core.statistics().bad_length >= 1 && core.statistics().bad_crc >= 1);
    assert_eq!(core.last_sequence(), 2);
    assert!(core.watchdog_expired(140, 100));
    assert!(core.poll_watchdog(140, 100, &mut device));
    assert!(core.latched());
    assert!(!core.poll_watchdog(141, 100, &mut device));
    assert_eq!(core.feed(&command(session, 3, 1, &[JointTarget { joint: 0, mode: 2, flags: 0, value: 1.0 }]), 150, &mut device, |_| {}), 0);
    assert_eq!(core.last_sequence(), 2);

    let mut wrong = fingerprint;
    wrong[0] ^= 1;
    let mut wrong_payload = [0u8; SessionBegin::SIZE];
    SessionBegin { session: session + 2, model_fingerprint: wrong }.encode(&mut wrong_payload).unwrap();
    let mut wrong_frame = [0u8; HEADER_SIZE + SessionBegin::SIZE + CRC_SIZE];
    encode_frame(1, &wrong_payload, &mut wrong_frame).unwrap();
    let mut ack = Vec::new();
    core.feed(&wrong_frame, 160, &mut device, |bytes| {
        if bytes[4] == 2 { ack = bytes.to_vec(); }
    });
    assert_eq!(SessionAck::decode(&ack[HEADER_SIZE..HEADER_SIZE + SessionAck::SIZE]).unwrap().status, 2);
    assert!(!core.model_matches());
    assert_eq!(core.last_sequence(), 0);
    assert_eq!(core.feed(&command(session + 2, 1, 4, &[]), 170, &mut device, |_| {}), 0);
    assert!(core.watchdog_expired(170, 100));

    // A later valid host session replaces the mismatched session safely.
    core.feed(&begin, 180, &mut device, |_| {});
    assert!(core.model_matches() && core.latched());
    assert_eq!(core.last_sequence(), 0);
    assert_eq!(device.stops, 4); // Initial begin, watchdog, mismatch, restart.
}

#[test]
fn corrupt_length_and_crc_resynchronize_to_valid_command() {
    let fingerprint = core::array::from_fn(|index| index as u8);
    let session = 0x0102_0304_0506_0708;
    let mut core = DeviceProtocol::new(1, fingerprint).unwrap();
    let mut device = FakeDevice { accept_reset: true, ..Default::default() };
    core.feed(&fixture("session_begin"), 10, &mut device, |_| {});
    let mut damaged = command(session, 1, 4, &[]);
    *damaged.last_mut().unwrap() ^= 0x40;
    let valid = command(session, 1, 4, &[]);
    let mut stream = b"junkRKD5".to_vec();
    stream.extend_from_slice(&[3, 0, 0xff, 0xff]);
    stream.extend_from_slice(&damaged);
    stream.extend_from_slice(&valid);
    assert_eq!(core.feed(&stream, 20, &mut device, |_| {}), 1);
    assert_eq!(core.last_sequence(), 1);
    assert!(core.statistics().bad_length >= 1);
    assert!(core.statistics().bad_crc >= 1);
    assert_eq!(device.resets, 1);
}
