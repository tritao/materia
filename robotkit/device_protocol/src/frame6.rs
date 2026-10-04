//! RKD6 length and CRC framing. Payload records live in `device_wire6`.

use crate::device_wire6::*;
fn crc32(bytes: &[u8]) -> u32 {
    let mut value = 0xffff_ffffu32;
    for &byte in bytes {
        value ^= u32::from(byte);
        for _ in 0..8 {
            value = (value >> 1) ^ (0xedb_88320 & 0u32.wrapping_sub(value & 1));
        }
    }
    !value
}

pub const HEADER_SIZE: usize = 8;
pub const CRC_SIZE: usize = 4;
pub const MAX_PAYLOAD_SIZE: usize = if SessionBegin6::SIZE > Segment6Header::SIZE +
    MAX_ACTUATORS as usize * Segment6Coefficients::SIZE { SessionBegin6::SIZE } else {
    Segment6Header::SIZE + MAX_ACTUATORS as usize * Segment6Coefficients::SIZE };
pub const MAX_FRAME_SIZE: usize = HEADER_SIZE + MAX_PAYLOAD_SIZE + CRC_SIZE;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Frame6Error { ShortBuffer, BadMagic, BadType, BadLength, BadCrc, BadPayload }

/// Keep a partial RKD6 marker when a noisy byte precedes a frame.
pub fn slide_to_frame_marker(input: &mut [u8], len: &mut usize) {
    while *len >= 4 && &input[..4] != b"RKD6" {
        input.copy_within(1..*len, 0);
        *len -= 1;
    }
}

pub fn encode_frame6(kind: u8, payload: &[u8], out: &mut [u8]) -> Result<usize, Frame6Error> {
    validate_payload(kind, payload)?;
    let size = HEADER_SIZE + payload.len() + CRC_SIZE;
    if out.len() < size { return Err(Frame6Error::ShortBuffer); }
    out[..4].copy_from_slice(b"RKD6");
    out[4] = kind;
    out[5] = 0;
    out[6..8].copy_from_slice(&(payload.len() as u16).to_le_bytes());
    out[8..8 + payload.len()].copy_from_slice(payload);
    let crc = crc32(&out[..8 + payload.len()]);
    out[8 + payload.len()..size].copy_from_slice(&crc.to_le_bytes());
    Ok(size)
}

pub fn decode_frame6(input: &[u8]) -> Result<(u8, &[u8]), Frame6Error> {
    if input.len() < HEADER_SIZE + CRC_SIZE { return Err(Frame6Error::ShortBuffer); }
    if &input[..4] != b"RKD6" { return Err(Frame6Error::BadMagic); }
    if input[5] != 0 { return Err(Frame6Error::BadPayload); }
    let size = u16::from_le_bytes([input[6], input[7]]) as usize;
    if size > MAX_PAYLOAD_SIZE || input.len() != HEADER_SIZE + size + CRC_SIZE {
        return Err(Frame6Error::BadLength);
    }
    let crc = u32::from_le_bytes(input[8 + size..].try_into().unwrap());
    if crc != crc32(&input[..8 + size]) { return Err(Frame6Error::BadCrc); }
    let payload = &input[8..8 + size];
    validate_payload(input[4], payload)?;
    Ok((input[4], payload))
}

fn validate_payload(kind: u8, bytes: &[u8]) -> Result<(), Frame6Error> {
    let exact = match kind {
        1 => None, 2 => Some(SessionAck6::SIZE),
        3 => Some(TimeSyncRequest::SIZE), 4 => Some(TimeSyncReply::SIZE),
        5 => Some(QueueBegin6::SIZE), 7 => Some(Commit6::SIZE),
        8..=13 => Some(0), 14 => Some(QueueStatus6::SIZE),
        15 | 6 => None,
        16 => Some(Event6::SIZE),
        17 => None,
        _ => return Err(Frame6Error::BadType),
    };
    if let Some(size) = exact {
        if bytes.len() != size { return Err(Frame6Error::BadLength); }
    } else if kind == 1 {
        if bytes.len() != SessionBegin6::SIZE { return Err(Frame6Error::BadLength); }
        let head = SessionBegin6::decode(bytes)
            .map_err(|_| Frame6Error::BadPayload)?;
        if head.protocol_version != PROTOCOL_VERSION || head.actuator_count == 0 ||
           head.actuator_count > MAX_ACTUATORS || !head.max_acceleration.is_finite() ||
           head.max_acceleration <= 0.0 || head.link_loss_timeout_ns == 0 ||
           head.channel_count > 32 || head.input_count > 64 {
            return Err(Frame6Error::BadPayload);
        }
        for (a, &limit) in head.actuator_max_acceleration[..head.actuator_count as usize].iter().enumerate() {
            if !limit.is_finite() || limit <= 0.0 || limit > head.max_acceleration ||
               !head.steps_per_unit[a].is_finite() || head.steps_per_unit[a] <= 0.0 ||
               !head.max_rate[a].is_finite() || head.max_rate[a] < 0.0 ||
               head.actuator_joint[a] >= MAX_ACTUATORS ||
               !head.actuator_ratio[a].is_finite() || head.actuator_ratio[a] == 0.0 ||
               !head.dual_drive_skew_bound[a].is_finite() || head.dual_drive_skew_bound[a] < 0.0 {
                return Err(Frame6Error::BadPayload);
            }
        }
        for channel in 0..head.input_count as usize {
            if head.input_actuator[channel] >= head.actuator_count { return Err(Frame6Error::BadPayload); }
        }
        for channel in 0..head.channel_count as usize {
            let kind = head.channel_kind[channel];
            let id = &head.channel_id[channel * 48..(channel + 1) * 48];
            if !(1..=3).contains(&kind) || id[0] == 0 ||
               !id.contains(&0) || head.safe_digital[channel] > 1 ||
               head.channel_stop_policy[channel] > 1 ||
               !head.safe_analog[channel].is_finite() ||
               !head.safe_argument[channel].is_finite() {
                return Err(Frame6Error::BadPayload);
            }
        }
    } else if kind == 6 {
        if bytes.len() < Segment6Header::SIZE { return Err(Frame6Error::BadLength); }
        let head = Segment6Header::decode(&bytes[..Segment6Header::SIZE])
            .map_err(|_| Frame6Error::BadPayload)?;
        if head.degree > 5 || head.actuator_count == 0 || head.actuator_count > MAX_ACTUATORS ||
           head.ends_at_rest > 1 || head.purpose > 2 || head.duration_ticks == 0 ||
           bytes.len() != Segment6Header::SIZE + head.actuator_count as usize * Segment6Coefficients::SIZE {
            return Err(Frame6Error::BadPayload);
        }
    } else if kind == 15 {
        if bytes.len() < State6Header::SIZE { return Err(Frame6Error::BadLength); }
        let head = State6Header::decode(&bytes[..State6Header::SIZE])
            .map_err(|_| Frame6Error::BadPayload)?;
        if head.actuator_count > MAX_ACTUATORS || head.input_count > 64 || head.reserved != 0 ||
           bytes.len() != State6Header::SIZE + head.actuator_count as usize * ActuatorState6::SIZE + head.input_count as usize * InputState6::SIZE {
            return Err(Frame6Error::BadPayload);
        }
    }
    if kind == 17 {
        if bytes.len() < Sensor6Header::SIZE { return Err(Frame6Error::BadLength); }
        let header = Sensor6Header::decode(&bytes[..Sensor6Header::SIZE])
            .map_err(|_| Frame6Error::BadPayload)?;
        if header.session == 0 || header.sequence == 0 || header.slot >= 8 ||
            header.value_count == 0 || header.value_count > 360 ||
            bytes.len() != Sensor6Header::SIZE + header.value_count as usize * Sensor6Value::SIZE {
            return Err(Frame6Error::BadPayload);
        }
        for chunk in bytes[Sensor6Header::SIZE..].chunks_exact(Sensor6Value::SIZE) {
            if !Sensor6Value::decode(chunk).map_err(|_| Frame6Error::BadPayload)?.value.is_finite() {
                return Err(Frame6Error::BadPayload);
            }
        }
    }
    if kind == 16 {
        let event = Event6::decode(bytes).map_err(|_| Frame6Error::BadPayload)?;
        if event.plan_id == 0 || event.queue_revision == 0 || event.channel >= 32 ||
           !(1..=3).contains(&event.kind) || event.hold_policy > 2 ||
           event.digital > 1 || !event.analog.is_finite() ||
           !event.argument.is_finite() {
            return Err(Frame6Error::BadPayload);
        }
    }
    Ok(())
}
