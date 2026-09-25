use robotkit_device_protocol::*;

fn fixture(name: &str) -> Vec<u8> {
    let text = include_str!("../../schema/device_wire_vectors.tsv");
    let hex = text.lines()
        .filter_map(|line| line.split_once('\t'))
        .find(|(key, _)| *key == name).unwrap().1;
    hex.as_bytes().chunks_exact(2)
        .map(|pair| u8::from_str_radix(core::str::from_utf8(pair).unwrap(), 16).unwrap())
        .collect()
}

fn check<T: core::fmt::Debug + PartialEq + Copy>(name: &str, value: T, size: usize,
    encode: impl Fn(&T, &mut [u8]) -> Result<usize, Error>,
    decode: impl Fn(&[u8]) -> Result<T, Error>) {
    let expected = fixture(name);
    assert_eq!(expected.len(), size);
    let mut bytes = vec![0u8; size];
    assert_eq!(encode(&value, &mut bytes), Ok(size));
    assert_eq!(bytes, expected);
    assert_eq!(decode(&bytes), Ok(value));
    assert_eq!(encode(&value, &mut bytes[..size - 1]), Err(Error::ShortBuffer));
    assert_eq!(decode(&bytes[..size - 1]), Err(Error::WrongLength));
}

#[test]
fn canonical_packed_records() {
    let session = 0x0102_0304_0506_0708;
    let fingerprint = core::array::from_fn(|index| index as u8);
    check("SessionBegin", SessionBegin { session, model_fingerprint: fingerprint }, SessionBegin::SIZE,
        SessionBegin::encode, SessionBegin::decode);
    check("SessionAck", SessionAck { session, device_fingerprint: fingerprint, status: 1, reserved: [0; 3] }, SessionAck::SIZE,
        SessionAck::encode, SessionAck::decode);
    check("CommandHeader", CommandHeader { session, sequence: 9, kind: 1, target_count: 2, reserved: 0 }, CommandHeader::SIZE,
        CommandHeader::encode, CommandHeader::decode);
    check("JointTarget", JointTarget { joint: 0x1234, mode: 2, flags: 0, value: 1.5 }, JointTarget::SIZE,
        JointTarget::encode, JointTarget::decode);
    check("StateHeader", StateHeader { session, timestamp_ns: 123456789, accepted_sequence: 9, safety: 1, fault: 0, joint_count: 2, reserved: 0 }, StateHeader::SIZE,
        StateHeader::encode, StateHeader::decode);
    check("JointState", JointState { position: 1.0, velocity: -2.0, effort: 0.5 }, JointState::SIZE,
        JointState::encode, JointState::decode);
}
