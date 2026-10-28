// Generated from a .wire.idl schema. Do not edit.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error { ShortBuffer, WrongLength, Constraint(&'static str) }

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
        self.check()?;
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
        let value = Self { sequence, wheel_delta, position, step, buttons, flags };
        value.check()?;
        Ok(value)
    }

    /// Constraints declared in the schema; checked on encode and decode.
    pub fn check(&self) -> Result<(), Error> {
        if !(0..=11).contains(&self.position) { return Err(Error::Constraint("PendantState.position")); }
        if !(0..=2).contains(&self.step) { return Err(Error::Constraint("PendantState.step")); }
        if self.buttons & !63u8 != 0 { return Err(Error::Constraint("PendantState.buttons")); }
        if self.flags & !3u8 != 0 { return Err(Error::Constraint("PendantState.flags")); }
        Ok(())
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
        self.check()?;
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
        let value = Self { kind, index, text, reserved };
        value.check()?;
        Ok(value)
    }

    /// Constraints declared in the schema; checked on encode and decode.
    pub fn check(&self) -> Result<(), Error> {
        if !matches!(self.kind, 0 | 1) { return Err(Error::Constraint("PendantConfig.kind")); }
        if !(0..=11).contains(&self.index) { return Err(Error::Constraint("PendantConfig.index")); }
        if { let end = self.text.iter().position(|&b| b == 0).unwrap_or(self.text.len()); self.text[..end].iter().any(|&b| !(32..=126).contains(&b)) || self.text[end..].iter().any(|&b| b != 0) } { return Err(Error::Constraint("PendantConfig.text")); }
        if self.reserved.iter().any(|&v| v != 0) { return Err(Error::Constraint("PendantConfig.reserved")); }
        Ok(())
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
        self.check()?;
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
        let value = Self { leds, lease, text, reserved };
        value.check()?;
        Ok(value)
    }

    /// Constraints declared in the schema; checked on encode and decode.
    pub fn check(&self) -> Result<(), Error> {
        if self.leds & !63u8 != 0 { return Err(Error::Constraint("PendantOutput.leds")); }
        if !matches!(self.lease, 0 | 1 | 2) { return Err(Error::Constraint("PendantOutput.lease")); }
        if { let end = self.text.iter().position(|&b| b == 0).unwrap_or(self.text.len()); self.text[..end].iter().any(|&b| !(32..=126).contains(&b)) || self.text[end..].iter().any(|&b| b != 0) } { return Err(Error::Constraint("PendantOutput.text")); }
        if self.reserved != 0 { return Err(Error::Constraint("PendantOutput.reserved")); }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PendantProfile {
    pub axes: [u8; 11],
    pub rule_count: u8,
    pub rule_axis: [u8; 8],
    pub rule_condition: [u8; 8],
    pub rule_reason: [u8; 8],
    pub has_spindle: u8,
    pub spindle_key: u8,
    pub step_units: [i32; 3],
    pub timeout_ms: u16,
}

impl PendantProfile {
    pub const SIZE: usize = 52;

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, Error> {
        if out.len() < Self::SIZE { return Err(Error::ShortBuffer); }
        self.check()?;
        let mut offset = 0usize;
        for value in self.axes {
            out[offset..offset + 1].copy_from_slice(&value.to_le_bytes());
            offset += 1;
        }
        out[offset..offset + 1].copy_from_slice(&self.rule_count.to_le_bytes());
        offset += 1;
        for value in self.rule_axis {
            out[offset..offset + 1].copy_from_slice(&value.to_le_bytes());
            offset += 1;
        }
        for value in self.rule_condition {
            out[offset..offset + 1].copy_from_slice(&value.to_le_bytes());
            offset += 1;
        }
        for value in self.rule_reason {
            out[offset..offset + 1].copy_from_slice(&value.to_le_bytes());
            offset += 1;
        }
        out[offset..offset + 1].copy_from_slice(&self.has_spindle.to_le_bytes());
        offset += 1;
        out[offset..offset + 1].copy_from_slice(&self.spindle_key.to_le_bytes());
        offset += 1;
        for value in self.step_units {
            out[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
            offset += 4;
        }
        out[offset..offset + 2].copy_from_slice(&self.timeout_ms.to_le_bytes());
        offset += 2;
        Ok(offset)
    }

    pub fn decode(input: &[u8]) -> Result<Self, Error> {
        if input.len() != Self::SIZE { return Err(Error::WrongLength); }
        let mut offset = 0usize;
        let mut axes = [0 as u8; 11];
        for item in &mut axes {
            let mut bytes = [0u8; 1];
            bytes.copy_from_slice(&input[offset..offset + 1]);
            *item = u8::from_le_bytes(bytes);
            offset += 1;
        }
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let rule_count = u8::from_le_bytes(bytes);
        offset += 1;
        let mut rule_axis = [0 as u8; 8];
        for item in &mut rule_axis {
            let mut bytes = [0u8; 1];
            bytes.copy_from_slice(&input[offset..offset + 1]);
            *item = u8::from_le_bytes(bytes);
            offset += 1;
        }
        let mut rule_condition = [0 as u8; 8];
        for item in &mut rule_condition {
            let mut bytes = [0u8; 1];
            bytes.copy_from_slice(&input[offset..offset + 1]);
            *item = u8::from_le_bytes(bytes);
            offset += 1;
        }
        let mut rule_reason = [0 as u8; 8];
        for item in &mut rule_reason {
            let mut bytes = [0u8; 1];
            bytes.copy_from_slice(&input[offset..offset + 1]);
            *item = u8::from_le_bytes(bytes);
            offset += 1;
        }
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let has_spindle = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let spindle_key = u8::from_le_bytes(bytes);
        offset += 1;
        let mut step_units = [0 as i32; 3];
        for item in &mut step_units {
            let mut bytes = [0u8; 4];
            bytes.copy_from_slice(&input[offset..offset + 4]);
            *item = i32::from_le_bytes(bytes);
            offset += 4;
        }
        let mut bytes = [0u8; 2];
        bytes.copy_from_slice(&input[offset..offset + 2]);
        let timeout_ms = u16::from_le_bytes(bytes);
        offset += 2;
        let _ = offset;
        let value = Self { axes, rule_count, rule_axis, rule_condition, rule_reason, has_spindle, spindle_key, step_units, timeout_ms };
        value.check()?;
        Ok(value)
    }

    /// Constraints declared in the schema; checked on encode and decode.
    pub fn check(&self) -> Result<(), Error> {
        if !(0..=8).contains(&self.rule_count) { return Err(Error::Constraint("PendantProfile.rule_count")); }
        if self.rule_condition.iter().any(|&v| !(0..=31).contains(&v)) { return Err(Error::Constraint("PendantProfile.rule_condition")); }
        if self.rule_reason.iter().any(|&v| !(0..=7).contains(&v)) { return Err(Error::Constraint("PendantProfile.rule_reason")); }
        if !(0..=1).contains(&self.has_spindle) { return Err(Error::Constraint("PendantProfile.has_spindle")); }
        if !(1..=1000).contains(&self.timeout_ms) { return Err(Error::Constraint("PendantProfile.timeout_ms")); }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PendantReasonText {
    pub index: u8,
    pub text: [u8; 21],
    pub reserved: [u8; 2],
}

impl PendantReasonText {
    pub const SIZE: usize = 24;

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, Error> {
        if out.len() < Self::SIZE { return Err(Error::ShortBuffer); }
        self.check()?;
        let mut offset = 0usize;
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
        let index = u8::from_le_bytes(bytes);
        offset += 1;
        let mut text = [0 as u8; 21];
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
        let value = Self { index, text, reserved };
        value.check()?;
        Ok(value)
    }

    /// Constraints declared in the schema; checked on encode and decode.
    pub fn check(&self) -> Result<(), Error> {
        if !(0..=7).contains(&self.index) { return Err(Error::Constraint("PendantReasonText.index")); }
        if { let end = self.text.iter().position(|&b| b == 0).unwrap_or(self.text.len()); self.text[..end].iter().any(|&b| !(32..=126).contains(&b)) || self.text[end..].iter().any(|&b| b != 0) } { return Err(Error::Constraint("PendantReasonText.text")); }
        if self.reserved.iter().any(|&v| v != 0) { return Err(Error::Constraint("PendantReasonText.reserved")); }
        Ok(())
    }
}
