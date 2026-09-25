// Generated from a .wire.idl schema. Do not edit.
#![no_std]

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error { ShortBuffer, WrongLength }

pub const PROTOCOL_VERSION: u8 = 5;
pub const MAX_JOINTS: u8 = 64;

#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MessageType {
    SessionBegin = 1,
    SessionAck = 2,
    Command = 3,
    State = 4,
}

#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SessionStatus {
    LatchedSafe = 1,
    ModelMismatch = 2,
}

#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandKind {
    Targets = 1,
    Stop = 2,
    EmergencyStop = 3,
    ResetSafety = 4,
}

#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TargetMode {
    Position = 1,
    Velocity = 2,
    Effort = 3,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SessionBegin {
    pub session: u64,
    pub model_fingerprint: [u8; 16],
}

impl SessionBegin {
    pub const SIZE: usize = 24;

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, Error> {
        if out.len() < Self::SIZE { return Err(Error::ShortBuffer); }
        let mut offset = 0usize;
        out[offset..offset + 8].copy_from_slice(&self.session.to_le_bytes());
        offset += 8;
        for value in self.model_fingerprint {
            out[offset..offset + 1].copy_from_slice(&value.to_le_bytes());
            offset += 1;
        }
        Ok(offset)
    }

    pub fn decode(input: &[u8]) -> Result<Self, Error> {
        if input.len() != Self::SIZE { return Err(Error::WrongLength); }
        let mut offset = 0usize;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let session = u64::from_le_bytes(bytes);
        offset += 8;
        let mut model_fingerprint = [0 as u8; 16];
        for item in &mut model_fingerprint {
            let mut bytes = [0u8; 1];
            bytes.copy_from_slice(&input[offset..offset + 1]);
            *item = u8::from_le_bytes(bytes);
            offset += 1;
        }
        let _ = offset;
        Ok(Self { session, model_fingerprint })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SessionAck {
    pub session: u64,
    pub device_fingerprint: [u8; 16],
    pub status: u8,
    pub reserved: [u8; 3],
}

impl SessionAck {
    pub const SIZE: usize = 28;

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, Error> {
        if out.len() < Self::SIZE { return Err(Error::ShortBuffer); }
        let mut offset = 0usize;
        out[offset..offset + 8].copy_from_slice(&self.session.to_le_bytes());
        offset += 8;
        for value in self.device_fingerprint {
            out[offset..offset + 1].copy_from_slice(&value.to_le_bytes());
            offset += 1;
        }
        out[offset..offset + 1].copy_from_slice(&self.status.to_le_bytes());
        offset += 1;
        for value in self.reserved {
            out[offset..offset + 1].copy_from_slice(&value.to_le_bytes());
            offset += 1;
        }
        Ok(offset)
    }

    pub fn decode(input: &[u8]) -> Result<Self, Error> {
        if input.len() != Self::SIZE { return Err(Error::WrongLength); }
        let mut offset = 0usize;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let session = u64::from_le_bytes(bytes);
        offset += 8;
        let mut device_fingerprint = [0 as u8; 16];
        for item in &mut device_fingerprint {
            let mut bytes = [0u8; 1];
            bytes.copy_from_slice(&input[offset..offset + 1]);
            *item = u8::from_le_bytes(bytes);
            offset += 1;
        }
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let status = u8::from_le_bytes(bytes);
        offset += 1;
        let mut reserved = [0 as u8; 3];
        for item in &mut reserved {
            let mut bytes = [0u8; 1];
            bytes.copy_from_slice(&input[offset..offset + 1]);
            *item = u8::from_le_bytes(bytes);
            offset += 1;
        }
        let _ = offset;
        Ok(Self { session, device_fingerprint, status, reserved })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CommandHeader {
    pub session: u64,
    pub sequence: u64,
    pub kind: u8,
    pub target_count: u8,
    pub reserved: u16,
}

impl CommandHeader {
    pub const SIZE: usize = 20;

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, Error> {
        if out.len() < Self::SIZE { return Err(Error::ShortBuffer); }
        let mut offset = 0usize;
        out[offset..offset + 8].copy_from_slice(&self.session.to_le_bytes());
        offset += 8;
        out[offset..offset + 8].copy_from_slice(&self.sequence.to_le_bytes());
        offset += 8;
        out[offset..offset + 1].copy_from_slice(&self.kind.to_le_bytes());
        offset += 1;
        out[offset..offset + 1].copy_from_slice(&self.target_count.to_le_bytes());
        offset += 1;
        out[offset..offset + 2].copy_from_slice(&self.reserved.to_le_bytes());
        offset += 2;
        Ok(offset)
    }

    pub fn decode(input: &[u8]) -> Result<Self, Error> {
        if input.len() != Self::SIZE { return Err(Error::WrongLength); }
        let mut offset = 0usize;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let session = u64::from_le_bytes(bytes);
        offset += 8;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let sequence = u64::from_le_bytes(bytes);
        offset += 8;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let kind = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let target_count = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 2];
        bytes.copy_from_slice(&input[offset..offset + 2]);
        let reserved = u16::from_le_bytes(bytes);
        offset += 2;
        let _ = offset;
        Ok(Self { session, sequence, kind, target_count, reserved })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointTarget {
    pub joint: u16,
    pub mode: u8,
    pub flags: u8,
    pub value: f32,
}

impl JointTarget {
    pub const SIZE: usize = 8;

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, Error> {
        if out.len() < Self::SIZE { return Err(Error::ShortBuffer); }
        let mut offset = 0usize;
        out[offset..offset + 2].copy_from_slice(&self.joint.to_le_bytes());
        offset += 2;
        out[offset..offset + 1].copy_from_slice(&self.mode.to_le_bytes());
        offset += 1;
        out[offset..offset + 1].copy_from_slice(&self.flags.to_le_bytes());
        offset += 1;
        out[offset..offset + 4].copy_from_slice(&self.value.to_le_bytes());
        offset += 4;
        Ok(offset)
    }

    pub fn decode(input: &[u8]) -> Result<Self, Error> {
        if input.len() != Self::SIZE { return Err(Error::WrongLength); }
        let mut offset = 0usize;
        let mut bytes = [0u8; 2];
        bytes.copy_from_slice(&input[offset..offset + 2]);
        let joint = u16::from_le_bytes(bytes);
        offset += 2;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let mode = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let flags = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 4];
        bytes.copy_from_slice(&input[offset..offset + 4]);
        let value = f32::from_le_bytes(bytes);
        offset += 4;
        let _ = offset;
        Ok(Self { joint, mode, flags, value })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StateHeader {
    pub session: u64,
    pub timestamp_ns: u64,
    pub accepted_sequence: u64,
    pub safety: u8,
    pub fault: u8,
    pub joint_count: u8,
    pub reserved: u8,
}

impl StateHeader {
    pub const SIZE: usize = 28;

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, Error> {
        if out.len() < Self::SIZE { return Err(Error::ShortBuffer); }
        let mut offset = 0usize;
        out[offset..offset + 8].copy_from_slice(&self.session.to_le_bytes());
        offset += 8;
        out[offset..offset + 8].copy_from_slice(&self.timestamp_ns.to_le_bytes());
        offset += 8;
        out[offset..offset + 8].copy_from_slice(&self.accepted_sequence.to_le_bytes());
        offset += 8;
        out[offset..offset + 1].copy_from_slice(&self.safety.to_le_bytes());
        offset += 1;
        out[offset..offset + 1].copy_from_slice(&self.fault.to_le_bytes());
        offset += 1;
        out[offset..offset + 1].copy_from_slice(&self.joint_count.to_le_bytes());
        offset += 1;
        out[offset..offset + 1].copy_from_slice(&self.reserved.to_le_bytes());
        offset += 1;
        Ok(offset)
    }

    pub fn decode(input: &[u8]) -> Result<Self, Error> {
        if input.len() != Self::SIZE { return Err(Error::WrongLength); }
        let mut offset = 0usize;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let session = u64::from_le_bytes(bytes);
        offset += 8;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let timestamp_ns = u64::from_le_bytes(bytes);
        offset += 8;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let accepted_sequence = u64::from_le_bytes(bytes);
        offset += 8;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let safety = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let fault = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let joint_count = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let reserved = u8::from_le_bytes(bytes);
        offset += 1;
        let _ = offset;
        Ok(Self { session, timestamp_ns, accepted_sequence, safety, fault, joint_count, reserved })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointState {
    pub position: f32,
    pub velocity: f32,
    pub effort: f32,
}

impl JointState {
    pub const SIZE: usize = 12;

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, Error> {
        if out.len() < Self::SIZE { return Err(Error::ShortBuffer); }
        let mut offset = 0usize;
        out[offset..offset + 4].copy_from_slice(&self.position.to_le_bytes());
        offset += 4;
        out[offset..offset + 4].copy_from_slice(&self.velocity.to_le_bytes());
        offset += 4;
        out[offset..offset + 4].copy_from_slice(&self.effort.to_le_bytes());
        offset += 4;
        Ok(offset)
    }

    pub fn decode(input: &[u8]) -> Result<Self, Error> {
        if input.len() != Self::SIZE { return Err(Error::WrongLength); }
        let mut offset = 0usize;
        let mut bytes = [0u8; 4];
        bytes.copy_from_slice(&input[offset..offset + 4]);
        let position = f32::from_le_bytes(bytes);
        offset += 4;
        let mut bytes = [0u8; 4];
        bytes.copy_from_slice(&input[offset..offset + 4]);
        let velocity = f32::from_le_bytes(bytes);
        offset += 4;
        let mut bytes = [0u8; 4];
        bytes.copy_from_slice(&input[offset..offset + 4]);
        let effort = f32::from_le_bytes(bytes);
        offset += 4;
        let _ = offset;
        Ok(Self { position, velocity, effort })
    }
}
