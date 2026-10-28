// Generated from a .wire.idl schema. Do not edit.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error { ShortBuffer, WrongLength }

pub const PROTOCOL_VERSION: u8 = 1;
pub const POSITIONS: u8 = 11;

#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LeaseState {
    NoLease = 0,
    Granted = 1,
    Lost = 2,
}

#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConfigKind {
    Axis = 0,
    Key = 1,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PendantState {
    pub sequence: u16,
    pub wheel_delta: i16,
    pub position: u8,
    pub step: u8,
    pub buttons: u8,
    pub flags: u8,
}

impl PendantState {
    pub const SIZE: usize = 8;

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, Error> {
        if out.len() < Self::SIZE { return Err(Error::ShortBuffer); }
        let mut offset = 0usize;
        out[offset..offset + 2].copy_from_slice(&self.sequence.to_le_bytes());
        offset += 2;
        out[offset..offset + 2].copy_from_slice(&self.wheel_delta.to_le_bytes());
        offset += 2;
        out[offset..offset + 1].copy_from_slice(&self.position.to_le_bytes());
        offset += 1;
        out[offset..offset + 1].copy_from_slice(&self.step.to_le_bytes());
        offset += 1;
        out[offset..offset + 1].copy_from_slice(&self.buttons.to_le_bytes());
        offset += 1;
        out[offset..offset + 1].copy_from_slice(&self.flags.to_le_bytes());
        offset += 1;
        Ok(offset)
    }

    pub fn decode(input: &[u8]) -> Result<Self, Error> {
        if input.len() != Self::SIZE { return Err(Error::WrongLength); }
        let mut offset = 0usize;
        let mut bytes = [0u8; 2];
        bytes.copy_from_slice(&input[offset..offset + 2]);
        let sequence = u16::from_le_bytes(bytes);
        offset += 2;
        let mut bytes = [0u8; 2];
        bytes.copy_from_slice(&input[offset..offset + 2]);
        let wheel_delta = i16::from_le_bytes(bytes);
        offset += 2;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let position = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let step = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let buttons = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let flags = u8::from_le_bytes(bytes);
        offset += 1;
        let _ = offset;
        Ok(Self { sequence, wheel_delta, position, step, buttons, flags })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PendantConfig {
    pub kind: u8,
    pub index: u8,
    pub text: [u8; 12],
    pub reserved: [u8; 2],
}

impl PendantConfig {
    pub const SIZE: usize = 16;

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, Error> {
        if out.len() < Self::SIZE { return Err(Error::ShortBuffer); }
        let mut offset = 0usize;
        out[offset..offset + 1].copy_from_slice(&self.kind.to_le_bytes());
        offset += 1;
        out[offset..offset + 1].copy_from_slice(&self.index.to_le_bytes());
        offset += 1;
        for value in self.text {
            out[offset..offset + 1].copy_from_slice(&value.to_le_bytes());
            offset += 1;
        }
        for value in self.reserved {
            out[offset..offset + 1].copy_from_slice(&value.to_le_bytes());
            offset += 1;
        }
        Ok(offset)
    }

    pub fn decode(input: &[u8]) -> Result<Self, Error> {
        if input.len() != Self::SIZE { return Err(Error::WrongLength); }
        let mut offset = 0usize;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let kind = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let index = u8::from_le_bytes(bytes);
        offset += 1;
        let mut text = [0 as u8; 12];
        for item in &mut text {
            let mut bytes = [0u8; 1];
            bytes.copy_from_slice(&input[offset..offset + 1]);
            *item = u8::from_le_bytes(bytes);
            offset += 1;
        }
        let mut reserved = [0 as u8; 2];
        for item in &mut reserved {
            let mut bytes = [0u8; 1];
            bytes.copy_from_slice(&input[offset..offset + 1]);
            *item = u8::from_le_bytes(bytes);
            offset += 1;
        }
        let _ = offset;
        Ok(Self { kind, index, text, reserved })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PendantOutput {
    pub leds: u8,
    pub lease: u8,
    pub text: [u8; 21],
    pub reserved: u8,
}

impl PendantOutput {
    pub const SIZE: usize = 24;

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, Error> {
        if out.len() < Self::SIZE { return Err(Error::ShortBuffer); }
        let mut offset = 0usize;
        out[offset..offset + 1].copy_from_slice(&self.leds.to_le_bytes());
        offset += 1;
        out[offset..offset + 1].copy_from_slice(&self.lease.to_le_bytes());
        offset += 1;
        for value in self.text {
            out[offset..offset + 1].copy_from_slice(&value.to_le_bytes());
            offset += 1;
        }
        out[offset..offset + 1].copy_from_slice(&self.reserved.to_le_bytes());
        offset += 1;
        Ok(offset)
    }

    pub fn decode(input: &[u8]) -> Result<Self, Error> {
        if input.len() != Self::SIZE { return Err(Error::WrongLength); }
        let mut offset = 0usize;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let leds = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let lease = u8::from_le_bytes(bytes);
        offset += 1;
        let mut text = [0 as u8; 21];
        for item in &mut text {
            let mut bytes = [0u8; 1];
            bytes.copy_from_slice(&input[offset..offset + 1]);
            *item = u8::from_le_bytes(bytes);
            offset += 1;
        }
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let reserved = u8::from_le_bytes(bytes);
        offset += 1;
        let _ = offset;
        Ok(Self { leds, lease, text, reserved })
    }
}
