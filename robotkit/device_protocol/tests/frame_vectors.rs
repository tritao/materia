use robotkit_device_wire::*;

fn fixture(name: &str) -> Vec<u8> {
    let text = include_str!("../../schema/device_frame_vectors.tsv");
    let hex = text.lines().filter_map(|line| line.split_once('\t'))
        .find(|(key, _)| *key == name).unwrap().1;
    hex.as_bytes().chunks_exact(2)
        .map(|pair| u8::from_str_radix(core::str::from_utf8(pair).unwrap(), 16).unwrap())
        .collect()
}

fn frame(kind: u8, payload: &[u8]) -> Vec<u8> {
    let mut bytes = b"RKD5".to_vec();
    bytes.extend_from_slice(&[kind, 0]);
    bytes.extend_from_slice(&(payload.len() as u16).to_le_bytes());
    bytes.extend_from_slice(payload);
    let mut crc = 0xffff_ffffu32;
    for byte in &bytes {
        crc ^= u32::from(*byte);
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xedb8_8320u32 & (0u32.wrapping_sub(crc & 1)));
        }
    }
    bytes.extend_from_slice(&(!crc).to_le_bytes());
    bytes
}

fn payload<T>(value: &T, size: usize, encode: impl Fn(&T, &mut [u8]) -> Result<usize, Error>) -> Vec<u8> {
    let mut bytes = vec![0u8; size];
    assert_eq!(encode(value, &mut bytes), Ok(size));
    bytes
}

#[test]
fn complete_frame_vectors() {
    let session = 0x0102_0304_0506_0708;
    let fingerprint = core::array::from_fn(|index| index as u8);
    let begin = payload(&SessionBegin { session, model_fingerprint: fingerprint }, SessionBegin::SIZE, SessionBegin::encode);
    assert_eq!(frame(1, &begin), fixture("session_begin"));

    let reset = payload(&CommandHeader { session, sequence: 1, kind: 4, target_count: 0, reserved: 0 }, CommandHeader::SIZE, CommandHeader::encode);
    assert_eq!(frame(3, &reset), fixture("command_reset"));

    let mut target = payload(&CommandHeader { session, sequence: 2, kind: 1, target_count: 1, reserved: 0 }, CommandHeader::SIZE, CommandHeader::encode);
    target.extend(payload(&JointTarget { joint: 1, mode: 2, flags: 0, value: 1.5 }, JointTarget::SIZE, JointTarget::encode));
    assert_eq!(frame(3, &target), fixture("command_target"));

    let mut state = payload(&StateHeader { session, timestamp_ns: 123456789, accepted_sequence: 2, safety: 1, fault: 0, joint_count: 1, reserved: 0 }, StateHeader::SIZE, StateHeader::encode);
    state.extend(payload(&JointState { position: 1.0, velocity: -2.0, effort: 0.5 }, JointState::SIZE, JointState::encode));
    assert_eq!(frame(4, &state), fixture("state"));
}
