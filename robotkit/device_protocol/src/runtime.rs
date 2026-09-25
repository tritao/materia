//! Hardware-independent RKD5 device state machine and frame codec.

use crate::{CommandHeader, JointState, JointTarget, SessionAck, SessionBegin, StateHeader, MAX_JOINTS};

pub const HEADER_SIZE: usize = 8;
pub const CRC_SIZE: usize = 4;
pub const MAX_COMMAND_PAYLOAD: usize = CommandHeader::SIZE + MAX_JOINTS as usize * JointTarget::SIZE;
pub const MAX_STATE_PAYLOAD: usize = StateHeader::SIZE + MAX_JOINTS as usize * JointState::SIZE;
pub const MAX_FRAME_SIZE: usize = HEADER_SIZE + MAX_STATE_PAYLOAD + CRC_SIZE;
const MAGIC: &[u8; 4] = b"RKD5";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameError { BufferTooShort, InvalidType, PayloadTooLarge, InvalidState }

pub fn crc32(bytes: &[u8]) -> u32 {
    let mut value = 0xffff_ffffu32;
    for &byte in bytes {
        value ^= u32::from(byte);
        for _ in 0..8 {
            value = (value >> 1) ^ (0xedb_88320 & 0u32.wrapping_sub(value & 1));
        }
    }
    !value
}

pub fn encode_frame(kind: u8, payload: &[u8], output: &mut [u8]) -> Result<usize, FrameError> {
    if !(1..=4).contains(&kind) { return Err(FrameError::InvalidType); }
    if payload.len() > MAX_STATE_PAYLOAD { return Err(FrameError::PayloadTooLarge); }
    let size = HEADER_SIZE + payload.len() + CRC_SIZE;
    if output.len() < size { return Err(FrameError::BufferTooShort); }
    output[..4].copy_from_slice(MAGIC);
    output[4] = kind;
    output[5] = 0;
    output[6..8].copy_from_slice(&(payload.len() as u16).to_le_bytes());
    output[8..8 + payload.len()].copy_from_slice(payload);
    let crc = crc32(&output[..8 + payload.len()]);
    output[8 + payload.len()..size].copy_from_slice(&crc.to_le_bytes());
    Ok(size)
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StateMeta {
    pub timestamp_ns: u64,
    pub safety: u8,
    pub fault: u8,
    pub joint_count: u8,
}

/// The adapter owns all hardware. `stop_all` must stop outputs and clear active
/// targets synchronously. Hardware must start safe before the first session.
pub trait Device {
    fn stop_all(&mut self);
    fn apply_targets(&mut self, targets: &[JointTarget]) -> bool;
    fn reset_safety(&mut self) -> bool;
    fn read_state(&mut self, joints: &mut [JointState; MAX_JOINTS as usize]) -> StateMeta;
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Statistics {
    pub bad_length: u64,
    pub bad_crc: u64,
    pub invalid: u64,
    pub rejected: u64,
}

/// Fixed-capacity device runtime. Call `poll_watchdog` from a local clock task.
pub struct DeviceProtocol {
    fingerprint: [u8; 16],
    joint_count: u8,
    buffer: [u8; MAX_FRAME_SIZE],
    buffered: usize,
    session: u64,
    last_sequence: u64,
    last_command_ns: u64,
    has_command: bool,
    latched: bool,
    model_matches: bool,
    stats: Statistics,
}

impl DeviceProtocol {
    pub fn new(joint_count: u8, fingerprint: [u8; 16]) -> Option<Self> {
        if joint_count > MAX_JOINTS || fingerprint == [0; 16] { return None; }
        Some(Self {
            fingerprint, joint_count, buffer: [0; MAX_FRAME_SIZE], buffered: 0,
            session: 0, last_sequence: 0, last_command_ns: 0, has_command: false,
            latched: true, model_matches: false, stats: Statistics::default(),
        })
    }

    pub fn session(&self) -> u64 { self.session }
    pub fn last_sequence(&self) -> u64 { self.last_sequence }
    pub fn latched(&self) -> bool { self.latched }
    pub fn model_matches(&self) -> bool { self.model_matches }
    pub fn statistics(&self) -> Statistics { self.stats }

    pub fn watchdog_expired(&self, now_ns: u64, timeout_ns: u64) -> bool {
        !self.has_command || timeout_ns == 0 || now_ns < self.last_command_ns ||
            now_ns - self.last_command_ns >= timeout_ns
    }

    /// Stops and latches once when accepted command traffic times out.
    pub fn poll_watchdog<D: Device>(&mut self, now_ns: u64, timeout_ns: u64, device: &mut D) -> bool {
        if self.has_command && self.watchdog_expired(now_ns, timeout_ns) {
            device.stop_all();
            self.latched = true;
            self.has_command = false;
            return true;
        }
        false
    }

    /// Calls `emit` synchronously for each session ACK and initial safe STATE. The caller
    /// must copy or queue bytes before returning; the slice uses a local buffer.
    pub fn feed<D: Device>(&mut self, input: &[u8], now_ns: u64, device: &mut D,
                           mut emit: impl FnMut(&[u8])) -> usize {
        let mut accepted = 0;
        for &byte in input {
            if self.buffered == MAX_FRAME_SIZE {
                self.discard(1);
                self.stats.bad_length += 1;
            }
            self.buffer[self.buffered] = byte;
            self.buffered += 1;
            self.process(now_ns, device, &mut emit, &mut accepted);
        }
        accepted
    }

    /// Encode a complete STATE frame into a caller-owned buffer.
    pub fn encode_state<D: Device>(&self, device: &mut D, output: &mut [u8]) -> Result<usize, FrameError> {
        let mut joints = [JointState { position: 0.0, velocity: 0.0, effort: 0.0 }; MAX_JOINTS as usize];
        let meta = device.read_state(&mut joints);
        if meta.joint_count != self.joint_count || meta.joint_count > MAX_JOINTS || meta.safety > 3 {
            return Err(FrameError::InvalidState);
        }
        if joints[..meta.joint_count as usize].iter().any(|joint|
            !joint.position.is_finite() || !joint.velocity.is_finite() || !joint.effort.is_finite()) {
            return Err(FrameError::InvalidState);
        }
        let payload_size = StateHeader::SIZE + meta.joint_count as usize * JointState::SIZE;
        let frame_size = HEADER_SIZE + payload_size + CRC_SIZE;
        if output.len() < frame_size { return Err(FrameError::BufferTooShort); }
        let head = StateHeader {
            session: self.session, timestamp_ns: meta.timestamp_ns,
            accepted_sequence: self.last_sequence,
            safety: if self.latched && meta.safety < 2 { 2 } else { meta.safety },
            fault: meta.fault, joint_count: meta.joint_count, reserved: 0,
        };
        let mut payload = [0u8; MAX_STATE_PAYLOAD];
        head.encode(&mut payload[..StateHeader::SIZE]).map_err(|_| FrameError::BufferTooShort)?;
        for (index, joint) in joints[..meta.joint_count as usize].iter().enumerate() {
            let offset = StateHeader::SIZE + index * JointState::SIZE;
            joint.encode(&mut payload[offset..offset + JointState::SIZE]).map_err(|_| FrameError::BufferTooShort)?;
        }
        encode_frame(4, &payload[..payload_size], output)
    }

    fn discard(&mut self, count: usize) {
        if count >= self.buffered { self.buffered = 0; return; }
        self.buffer.copy_within(count..self.buffered, 0);
        self.buffered -= count;
    }

    fn resync(&mut self) {
        if self.buffered >= 4 && self.buffer[..4] == *MAGIC { return; }
        if let Some(start) = self.buffer[..self.buffered].windows(4).position(|window| window == MAGIC) {
            self.discard(start);
            return;
        }
        for count in (1..=self.buffered.min(3)).rev() {
            if self.buffer[self.buffered - count..self.buffered] == MAGIC[..count] {
                self.discard(self.buffered - count);
                return;
            }
        }
        self.buffered = 0;
    }

    fn process<D: Device>(&mut self, now_ns: u64, device: &mut D,
                          emit: &mut impl FnMut(&[u8]), accepted: &mut usize) {
        loop {
            self.resync();
            if self.buffered < HEADER_SIZE { return; }
            let payload_size = u16::from_le_bytes([self.buffer[6], self.buffer[7]]) as usize;
            if payload_size > MAX_STATE_PAYLOAD {
                self.stats.bad_length += 1;
                self.discard(1);
                continue;
            }
            let frame_size = HEADER_SIZE + payload_size + CRC_SIZE;
            if self.buffered < frame_size { return; }
            let expected = u32::from_le_bytes(self.buffer[HEADER_SIZE + payload_size..frame_size].try_into().unwrap());
            if expected != crc32(&self.buffer[..HEADER_SIZE + payload_size]) {
                self.stats.bad_crc += 1;
                self.discard(1);
                continue;
            }
            let kind = self.buffer[4];
            if self.buffer[5] != 0 {
                self.stats.invalid += 1;
            } else if kind == 1 {
                if let Ok(begin) = SessionBegin::decode(&self.buffer[HEADER_SIZE..HEADER_SIZE + payload_size]) {
                    if begin.session == 0 { self.stats.invalid += 1; }
                    else {
                        self.session = 0;
                        self.model_matches = false;
                        self.latched = true;
                        self.last_sequence = 0;
                        self.has_command = false;
                        self.last_command_ns = 0;
                        device.stop_all();
                        self.session = begin.session;
                        self.model_matches = begin.model_fingerprint == self.fingerprint;
                        let ack = SessionAck {
                            session: self.session, device_fingerprint: self.fingerprint,
                            status: if self.model_matches { 1 } else { 2 }, reserved: [0; 3],
                        };
                        let mut body = [0u8; SessionAck::SIZE];
                        let mut frame = [0u8; HEADER_SIZE + SessionAck::SIZE + CRC_SIZE];
                        if ack.encode(&mut body).is_ok() {
                            if let Ok(size) = encode_frame(2, &body, &mut frame) { emit(&frame[..size]); }
                        }
                        let mut state_frame = [0u8; MAX_FRAME_SIZE];
                        if let Ok(size) = self.encode_state(device, &mut state_frame) {
                            emit(&state_frame[..size]);
                        }
                    }
                } else { self.stats.invalid += 1; }
            } else if kind == 3 {
                let mut payload = [0u8; MAX_COMMAND_PAYLOAD];
                if payload_size > MAX_COMMAND_PAYLOAD {
                    self.stats.rejected += 1;
                    self.discard(frame_size);
                    continue;
                }
                payload[..payload_size].copy_from_slice(&self.buffer[HEADER_SIZE..HEADER_SIZE + payload_size]);
                if self.accept_command(&payload[..payload_size], now_ns, device) {
                    *accepted += 1;
                } else { self.stats.rejected += 1; }
            } else { self.stats.invalid += 1; }
            self.discard(frame_size);
        }
    }

    fn accept_command<D: Device>(&mut self, payload: &[u8], now_ns: u64, device: &mut D) -> bool {
        if payload.len() < CommandHeader::SIZE { return false; }
        let Ok(head) = CommandHeader::decode(&payload[..CommandHeader::SIZE]) else { return false; };
        let count = head.target_count as usize;
        if head.reserved != 0 || count > self.joint_count as usize ||
            payload.len() != CommandHeader::SIZE + count * JointTarget::SIZE ||
            !(1..=4).contains(&head.kind) || (head.kind == 1) != (count > 0) ||
            self.session == 0 || !self.model_matches || head.session != self.session ||
            head.sequence == 0 || head.sequence <= self.last_sequence ||
            (self.latched && head.kind == 1) { return false; }
        let mut targets = [JointTarget { joint: 0, mode: 0, flags: 0, value: 0.0 }; MAX_JOINTS as usize];
        let mut used = [false; MAX_JOINTS as usize];
        for (index, target) in targets[..count].iter_mut().enumerate() {
            let offset = CommandHeader::SIZE + index * JointTarget::SIZE;
            let Ok(value) = JointTarget::decode(&payload[offset..offset + JointTarget::SIZE]) else { return false; };
            if value.joint >= u16::from(self.joint_count) || used[value.joint as usize] ||
                !(1..=3).contains(&value.mode) || value.flags != 0 || !value.value.is_finite() {
                return false;
            }
            used[value.joint as usize] = true;
            *target = value;
        }
        let applied = match head.kind {
            1 => device.apply_targets(&targets[..count]),
            2 => { device.stop_all(); true },
            3 => { device.stop_all(); self.latched = true; true },
            4 => device.reset_safety(),
            _ => false,
        };
        if !applied { return false; }
        if head.kind == 4 { self.latched = false; }
        self.last_sequence = head.sequence;
        self.last_command_ns = now_ns;
        self.has_command = true;
        true
    }
}
