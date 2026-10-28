//! Frame of the host to cabinet controller operator channel, beside (not inside) the RKD6 motion
//! protocol so that protocol's wire revision stays untouched. Marker "OPR1", message type,
//! reserved zero, little-endian payload length, payload, then CRC-32/IEEE over everything before
//! it (the RKD6 core's `crc32`). Mirrors `OperatorFrame` in machinekit.operator.

use crate::operator_wire::{OperatorMessage, OperatorState, PendantConfig, PendantEvent, PendantProfile, PendantReasonText};

pub const MARKER: [u8; 4] = *b"OPR1";
pub const HEADER_SIZE: usize = 8;
pub const CRC_SIZE: usize = 4;
pub const MAX_FRAME_SIZE: usize = HEADER_SIZE + PendantProfile::SIZE + CRC_SIZE;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameError { ShortBuffer, BadMarker, BadType, BadLength, BadCrc }

/// The same CRC-32/IEEE as the RKD6 frames.
pub use robotkit_device_protocol::frame6::crc32;

/// The payload size each message type must carry.
pub fn payload_size(kind: u8) -> Option<usize> {
    match kind {
        k if k == OperatorMessage::Profile as u8 => Some(PendantProfile::SIZE),
        k if k == OperatorMessage::ReasonText as u8 => Some(PendantReasonText::SIZE),
        k if k == OperatorMessage::Config as u8 => Some(PendantConfig::SIZE),
        k if k == OperatorMessage::State as u8 => Some(OperatorState::SIZE),
        k if k == OperatorMessage::Event as u8 => Some(PendantEvent::SIZE),
        _ => None,
    }
}

pub fn encode(kind: u8, payload: &[u8], out: &mut [u8]) -> Result<usize, FrameError> {
    if payload_size(kind) != Some(payload.len()) { return Err(FrameError::BadLength); }
    let size = HEADER_SIZE + payload.len() + CRC_SIZE;
    if out.len() < size { return Err(FrameError::ShortBuffer); }
    out[..4].copy_from_slice(&MARKER);
    out[4] = kind;
    out[5] = 0;
    out[6..8].copy_from_slice(&(payload.len() as u16).to_le_bytes());
    out[HEADER_SIZE..HEADER_SIZE + payload.len()].copy_from_slice(payload);
    let crc = crc32(&out[..HEADER_SIZE + payload.len()]);
    out[HEADER_SIZE + payload.len()..size].copy_from_slice(&crc.to_le_bytes());
    Ok(size)
}

/// Check one complete frame and return its message type and payload.
pub fn decode(input: &[u8]) -> Result<(u8, &[u8]), FrameError> {
    if input.len() < HEADER_SIZE + CRC_SIZE { return Err(FrameError::ShortBuffer); }
    if input[..4] != MARKER { return Err(FrameError::BadMarker); }
    if input[5] != 0 { return Err(FrameError::BadLength); }
    let length = u16::from_le_bytes([input[6], input[7]]) as usize;
    if input.len() != HEADER_SIZE + length + CRC_SIZE { return Err(FrameError::BadLength); }
    let kind = input[4];
    match payload_size(kind) {
        None => return Err(FrameError::BadType),
        Some(n) if n != length => return Err(FrameError::BadLength),
        _ => {}
    }
    let crc = u32::from_le_bytes(input[HEADER_SIZE + length..].try_into().unwrap());
    if crc != crc32(&input[..HEADER_SIZE + length]) { return Err(FrameError::BadCrc); }
    Ok((kind, &input[HEADER_SIZE..HEADER_SIZE + length]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_matches_the_standard_check_value() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn round_trips_and_rejects_damage() {
        let state = OperatorState { sequence: 7, jog_allowed: 1, reserved: 0 };
        let mut payload = [0u8; 4];
        state.encode(&mut payload).unwrap();
        let mut frame = [0u8; MAX_FRAME_SIZE];
        let n = encode(OperatorMessage::State as u8, &payload, &mut frame).unwrap();
        assert_eq!(n, 16);
        let (kind, body) = decode(&frame[..n]).unwrap();
        assert_eq!((kind, OperatorState::decode(body).unwrap()), (4, state));
        let mut bad = frame; bad[0] = b'X';
        assert_eq!(decode(&bad[..n]), Err(FrameError::BadMarker));
        let mut bad = frame; bad[8] ^= 1;
        assert_eq!(decode(&bad[..n]), Err(FrameError::BadCrc));
        let mut bad = frame; bad[5] = 1;
        assert_eq!(decode(&bad[..n]), Err(FrameError::BadLength));
        assert_eq!(decode(&frame[..n - 1]), Err(FrameError::BadLength));
        assert_eq!(decode(&frame[..8]), Err(FrameError::ShortBuffer));
        assert_eq!(encode(9, &payload, &mut frame), Err(FrameError::BadLength));
        assert_eq!(encode(OperatorMessage::Profile as u8, &payload, &mut frame), Err(FrameError::BadLength));
        let mut wrong_type = frame; wrong_type[4] = 9;
        assert_eq!(decode(&wrong_type[..n]), Err(FrameError::BadType));
    }

    // Compiled by the Haxe side, which asserts the same file byte for byte.
    const FRAMES: &str = include_str!("../vectors/frames.txt");
    const PROFILES: &str = include_str!("../vectors/profiles.txt");

    fn unhex(s: &str) -> std::vec::Vec<u8> {
        (0..s.len() / 2).map(|i| u8::from_str_radix(&s[i * 2..i * 2 + 2], 16).unwrap()).collect()
    }
    fn frame(name: &str) -> std::vec::Vec<u8> {
        unhex(FRAMES.lines().find_map(|l| l.strip_prefix(name)).unwrap().trim())
    }

    #[test]
    fn host_frames_decode_and_carry_the_compiled_records() {
        let state_frame = frame("state-allowed ");
        let (kind, body) = decode(&state_frame).unwrap();
        assert_eq!(kind, OperatorMessage::State as u8);
        assert_eq!(OperatorState::decode(body).unwrap(), OperatorState { sequence: 7, jog_allowed: 1, reserved: 0 });

        let event_frame = frame("event-move ");
        let (kind, body) = decode(&event_frame).unwrap();
        assert_eq!(kind, OperatorMessage::Event as u8);
        let event = PendantEvent::decode(body).unwrap();
        assert_eq!((event.sequence, event.axis, event.flags, event.distance, event.pressed, event.refusal),
            (9, 2, 1, -50, 8, 6));

        let profile_frame = frame("profile-hobber ");
        let (kind, body) = decode(&profile_frame).unwrap();
        assert_eq!(kind, OperatorMessage::Profile as u8);
        // The profile frame carries exactly the record in profiles.txt.
        let hobber = PROFILES.lines().find(|l| l.starts_with("hobber ")).unwrap().split(' ').nth(2).unwrap();
        assert_eq!(body, &unhex(hobber)[..]);
        let compiled = crate::profile::CompiledProfile::from_record(body).unwrap();
        assert_eq!(compiled.host_timeout_us(), 250_000);

        let reason_frame = frame("reason-hobber-0 ");
        let (kind, body) = decode(&reason_frame).unwrap();
        assert_eq!(kind, OperatorMessage::ReasonText as u8);
        let reason = PendantReasonText::decode(body).unwrap();
        assert_eq!(crate::label(&reason.text), b"Gearbox engaged");

        let config_frame = frame("config-hobber-axis-5 ");
        let (kind, body) = decode(&config_frame).unwrap();
        assert_eq!(kind, OperatorMessage::Config as u8);
        let label = crate::decode_config(body).unwrap();
        assert_eq!((label.kind, label.index, crate::label(&label.text)), (0, 5, &b"B hob"[..]));
    }
}
