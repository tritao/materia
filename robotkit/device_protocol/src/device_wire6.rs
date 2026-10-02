// Generated from a .wire.idl schema. Do not edit.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error { ShortBuffer, WrongLength }

pub const PROTOCOL_VERSION: u8 = 12;
pub const MAX_ACTUATORS: u8 = 64;

#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MessageType6 {
    SessionBegin6 = 1,
    SessionAck6 = 2,
    TimeSyncRequest = 3,
    TimeSyncReply = 4,
    QueueBegin = 5,
    Segment = 6,
    Commit = 7,
    Hold = 8,
    Resume = 9,
    Abort = 10,
    Stop = 11,
    EmergencyStop = 12,
    ResetSafety = 13,
    QueueStatus = 14,
    State6 = 15,
    Event = 16,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SessionBegin6 {
    pub session: u64,
    pub protocol_version: u8,
    pub expected_controller: [u8; 16],
    pub actuator_count: u8,
    pub max_degree: u8,
    pub step_tick_hz: u32,
    pub max_acceleration: f32,
    pub actuator_max_acceleration: [f32; 64],
    pub steps_per_unit: [f32; 64],
    pub max_rate: [f32; 64],
    pub direction_setup_ticks: [u16; 64],
    pub actuator_joint: [u8; 64],
    pub actuator_ratio: [f32; 64],
    pub dual_drive_skew_bound: [f32; 64],
    pub link_loss_timeout_ns: u64,
    pub channel_count: u8,
    pub channel_id: [u8; 1536],
    pub channel_kind: [u8; 32],
    pub safe_digital: [u8; 32],
    pub safe_analog: [f32; 32],
    pub safe_argument: [f32; 32],
    pub safe_command: [u8; 1536],
    pub channel_stop_policy: [u8; 32],
}

impl SessionBegin6 {
    pub const SIZE: usize = 4940;

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, Error> {
        if out.len() < Self::SIZE { return Err(Error::ShortBuffer); }
        let mut offset = 0usize;
        out[offset..offset + 8].copy_from_slice(&self.session.to_le_bytes());
        offset += 8;
        out[offset..offset + 1].copy_from_slice(&self.protocol_version.to_le_bytes());
        offset += 1;
        for value in self.expected_controller {
            out[offset..offset + 1].copy_from_slice(&value.to_le_bytes());
            offset += 1;
        }
        out[offset..offset + 1].copy_from_slice(&self.actuator_count.to_le_bytes());
        offset += 1;
        out[offset..offset + 1].copy_from_slice(&self.max_degree.to_le_bytes());
        offset += 1;
        out[offset..offset + 4].copy_from_slice(&self.step_tick_hz.to_le_bytes());
        offset += 4;
        out[offset..offset + 4].copy_from_slice(&self.max_acceleration.to_le_bytes());
        offset += 4;
        for value in self.actuator_max_acceleration {
            out[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
            offset += 4;
        }
        for value in self.steps_per_unit {
            out[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
            offset += 4;
        }
        for value in self.max_rate {
            out[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
            offset += 4;
        }
        for value in self.direction_setup_ticks {
            out[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
            offset += 2;
        }
        for value in self.actuator_joint {
            out[offset..offset + 1].copy_from_slice(&value.to_le_bytes());
            offset += 1;
        }
        for value in self.actuator_ratio {
            out[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
            offset += 4;
        }
        for value in self.dual_drive_skew_bound {
            out[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
            offset += 4;
        }
        out[offset..offset + 8].copy_from_slice(&self.link_loss_timeout_ns.to_le_bytes());
        offset += 8;
        out[offset..offset + 1].copy_from_slice(&self.channel_count.to_le_bytes());
        offset += 1;
        for value in self.channel_id {
            out[offset..offset + 1].copy_from_slice(&value.to_le_bytes());
            offset += 1;
        }
        for value in self.channel_kind {
            out[offset..offset + 1].copy_from_slice(&value.to_le_bytes());
            offset += 1;
        }
        for value in self.safe_digital {
            out[offset..offset + 1].copy_from_slice(&value.to_le_bytes());
            offset += 1;
        }
        for value in self.safe_analog {
            out[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
            offset += 4;
        }
        for value in self.safe_argument {
            out[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
            offset += 4;
        }
        for value in self.safe_command {
            out[offset..offset + 1].copy_from_slice(&value.to_le_bytes());
            offset += 1;
        }
        for value in self.channel_stop_policy {
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
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let protocol_version = u8::from_le_bytes(bytes);
        offset += 1;
        let mut expected_controller = [0 as u8; 16];
        for item in &mut expected_controller {
            let mut bytes = [0u8; 1];
            bytes.copy_from_slice(&input[offset..offset + 1]);
            *item = u8::from_le_bytes(bytes);
            offset += 1;
        }
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let actuator_count = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let max_degree = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 4];
        bytes.copy_from_slice(&input[offset..offset + 4]);
        let step_tick_hz = u32::from_le_bytes(bytes);
        offset += 4;
        let mut bytes = [0u8; 4];
        bytes.copy_from_slice(&input[offset..offset + 4]);
        let max_acceleration = f32::from_le_bytes(bytes);
        offset += 4;
        let mut actuator_max_acceleration = [0 as f32; 64];
        for item in &mut actuator_max_acceleration {
            let mut bytes = [0u8; 4];
            bytes.copy_from_slice(&input[offset..offset + 4]);
            *item = f32::from_le_bytes(bytes);
            offset += 4;
        }
        let mut steps_per_unit = [0 as f32; 64];
        for item in &mut steps_per_unit {
            let mut bytes = [0u8; 4];
            bytes.copy_from_slice(&input[offset..offset + 4]);
            *item = f32::from_le_bytes(bytes);
            offset += 4;
        }
        let mut max_rate = [0 as f32; 64];
        for item in &mut max_rate {
            let mut bytes = [0u8; 4];
            bytes.copy_from_slice(&input[offset..offset + 4]);
            *item = f32::from_le_bytes(bytes);
            offset += 4;
        }
        let mut direction_setup_ticks = [0 as u16; 64];
        for item in &mut direction_setup_ticks {
            let mut bytes = [0u8; 2];
            bytes.copy_from_slice(&input[offset..offset + 2]);
            *item = u16::from_le_bytes(bytes);
            offset += 2;
        }
        let mut actuator_joint = [0 as u8; 64];
        for item in &mut actuator_joint {
            let mut bytes = [0u8; 1];
            bytes.copy_from_slice(&input[offset..offset + 1]);
            *item = u8::from_le_bytes(bytes);
            offset += 1;
        }
        let mut actuator_ratio = [0 as f32; 64];
        for item in &mut actuator_ratio {
            let mut bytes = [0u8; 4];
            bytes.copy_from_slice(&input[offset..offset + 4]);
            *item = f32::from_le_bytes(bytes);
            offset += 4;
        }
        let mut dual_drive_skew_bound = [0 as f32; 64];
        for item in &mut dual_drive_skew_bound {
            let mut bytes = [0u8; 4];
            bytes.copy_from_slice(&input[offset..offset + 4]);
            *item = f32::from_le_bytes(bytes);
            offset += 4;
        }
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let link_loss_timeout_ns = u64::from_le_bytes(bytes);
        offset += 8;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let channel_count = u8::from_le_bytes(bytes);
        offset += 1;
        let mut channel_id = [0 as u8; 1536];
        for item in &mut channel_id {
            let mut bytes = [0u8; 1];
            bytes.copy_from_slice(&input[offset..offset + 1]);
            *item = u8::from_le_bytes(bytes);
            offset += 1;
        }
        let mut channel_kind = [0 as u8; 32];
        for item in &mut channel_kind {
            let mut bytes = [0u8; 1];
            bytes.copy_from_slice(&input[offset..offset + 1]);
            *item = u8::from_le_bytes(bytes);
            offset += 1;
        }
        let mut safe_digital = [0 as u8; 32];
        for item in &mut safe_digital {
            let mut bytes = [0u8; 1];
            bytes.copy_from_slice(&input[offset..offset + 1]);
            *item = u8::from_le_bytes(bytes);
            offset += 1;
        }
        let mut safe_analog = [0 as f32; 32];
        for item in &mut safe_analog {
            let mut bytes = [0u8; 4];
            bytes.copy_from_slice(&input[offset..offset + 4]);
            *item = f32::from_le_bytes(bytes);
            offset += 4;
        }
        let mut safe_argument = [0 as f32; 32];
        for item in &mut safe_argument {
            let mut bytes = [0u8; 4];
            bytes.copy_from_slice(&input[offset..offset + 4]);
            *item = f32::from_le_bytes(bytes);
            offset += 4;
        }
        let mut safe_command = [0 as u8; 1536];
        for item in &mut safe_command {
            let mut bytes = [0u8; 1];
            bytes.copy_from_slice(&input[offset..offset + 1]);
            *item = u8::from_le_bytes(bytes);
            offset += 1;
        }
        let mut channel_stop_policy = [0 as u8; 32];
        for item in &mut channel_stop_policy {
            let mut bytes = [0u8; 1];
            bytes.copy_from_slice(&input[offset..offset + 1]);
            *item = u8::from_le_bytes(bytes);
            offset += 1;
        }
        let _ = offset;
        Ok(Self { session, protocol_version, expected_controller, actuator_count, max_degree, step_tick_hz, max_acceleration, actuator_max_acceleration, steps_per_unit, max_rate, direction_setup_ticks, actuator_joint, actuator_ratio, dual_drive_skew_bound, link_loss_timeout_ns, channel_count, channel_id, channel_kind, safe_digital, safe_analog, safe_argument, safe_command, channel_stop_policy })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Event6 {
    pub queue_revision: u64,
    pub plan_id: u64,
    pub path_ticks: u64,
    pub channel: u8,
    pub kind: u8,
    pub hold_policy: u8,
    pub digital: u8,
    pub analog: f32,
    pub argument: f32,
    pub command: [u8; 48],
}

impl Event6 {
    pub const SIZE: usize = 84;

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, Error> {
        if out.len() < Self::SIZE { return Err(Error::ShortBuffer); }
        let mut offset = 0usize;
        out[offset..offset + 8].copy_from_slice(&self.queue_revision.to_le_bytes());
        offset += 8;
        out[offset..offset + 8].copy_from_slice(&self.plan_id.to_le_bytes());
        offset += 8;
        out[offset..offset + 8].copy_from_slice(&self.path_ticks.to_le_bytes());
        offset += 8;
        out[offset..offset + 1].copy_from_slice(&self.channel.to_le_bytes());
        offset += 1;
        out[offset..offset + 1].copy_from_slice(&self.kind.to_le_bytes());
        offset += 1;
        out[offset..offset + 1].copy_from_slice(&self.hold_policy.to_le_bytes());
        offset += 1;
        out[offset..offset + 1].copy_from_slice(&self.digital.to_le_bytes());
        offset += 1;
        out[offset..offset + 4].copy_from_slice(&self.analog.to_le_bytes());
        offset += 4;
        out[offset..offset + 4].copy_from_slice(&self.argument.to_le_bytes());
        offset += 4;
        for value in self.command {
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
        let queue_revision = u64::from_le_bytes(bytes);
        offset += 8;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let plan_id = u64::from_le_bytes(bytes);
        offset += 8;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let path_ticks = u64::from_le_bytes(bytes);
        offset += 8;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let channel = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let kind = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let hold_policy = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let digital = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 4];
        bytes.copy_from_slice(&input[offset..offset + 4]);
        let analog = f32::from_le_bytes(bytes);
        offset += 4;
        let mut bytes = [0u8; 4];
        bytes.copy_from_slice(&input[offset..offset + 4]);
        let argument = f32::from_le_bytes(bytes);
        offset += 4;
        let mut command = [0 as u8; 48];
        for item in &mut command {
            let mut bytes = [0u8; 1];
            bytes.copy_from_slice(&input[offset..offset + 1]);
            *item = u8::from_le_bytes(bytes);
            offset += 1;
        }
        let _ = offset;
        Ok(Self { queue_revision, plan_id, path_ticks, channel, kind, hold_policy, digital, analog, argument, command })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SessionAck6 {
    pub session: u64,
    pub protocol_version: u8,
    pub controller: [u8; 16],
    pub status: u8,
    pub device_tick_hz: u64,
    pub segment_capacity: u16,
    pub event_capacity: u16,
    pub step_tick_hz: u32,
    pub max_degree: u8,
    pub actuator_count: u8,
    pub profile: u8,
    pub config_digest: u64,
}

impl SessionAck6 {
    pub const SIZE: usize = 53;

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, Error> {
        if out.len() < Self::SIZE { return Err(Error::ShortBuffer); }
        let mut offset = 0usize;
        out[offset..offset + 8].copy_from_slice(&self.session.to_le_bytes());
        offset += 8;
        out[offset..offset + 1].copy_from_slice(&self.protocol_version.to_le_bytes());
        offset += 1;
        for value in self.controller {
            out[offset..offset + 1].copy_from_slice(&value.to_le_bytes());
            offset += 1;
        }
        out[offset..offset + 1].copy_from_slice(&self.status.to_le_bytes());
        offset += 1;
        out[offset..offset + 8].copy_from_slice(&self.device_tick_hz.to_le_bytes());
        offset += 8;
        out[offset..offset + 2].copy_from_slice(&self.segment_capacity.to_le_bytes());
        offset += 2;
        out[offset..offset + 2].copy_from_slice(&self.event_capacity.to_le_bytes());
        offset += 2;
        out[offset..offset + 4].copy_from_slice(&self.step_tick_hz.to_le_bytes());
        offset += 4;
        out[offset..offset + 1].copy_from_slice(&self.max_degree.to_le_bytes());
        offset += 1;
        out[offset..offset + 1].copy_from_slice(&self.actuator_count.to_le_bytes());
        offset += 1;
        out[offset..offset + 1].copy_from_slice(&self.profile.to_le_bytes());
        offset += 1;
        out[offset..offset + 8].copy_from_slice(&self.config_digest.to_le_bytes());
        offset += 8;
        Ok(offset)
    }

    pub fn decode(input: &[u8]) -> Result<Self, Error> {
        if input.len() != Self::SIZE { return Err(Error::WrongLength); }
        let mut offset = 0usize;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let session = u64::from_le_bytes(bytes);
        offset += 8;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let protocol_version = u8::from_le_bytes(bytes);
        offset += 1;
        let mut controller = [0 as u8; 16];
        for item in &mut controller {
            let mut bytes = [0u8; 1];
            bytes.copy_from_slice(&input[offset..offset + 1]);
            *item = u8::from_le_bytes(bytes);
            offset += 1;
        }
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let status = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let device_tick_hz = u64::from_le_bytes(bytes);
        offset += 8;
        let mut bytes = [0u8; 2];
        bytes.copy_from_slice(&input[offset..offset + 2]);
        let segment_capacity = u16::from_le_bytes(bytes);
        offset += 2;
        let mut bytes = [0u8; 2];
        bytes.copy_from_slice(&input[offset..offset + 2]);
        let event_capacity = u16::from_le_bytes(bytes);
        offset += 2;
        let mut bytes = [0u8; 4];
        bytes.copy_from_slice(&input[offset..offset + 4]);
        let step_tick_hz = u32::from_le_bytes(bytes);
        offset += 4;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let max_degree = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let actuator_count = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let profile = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let config_digest = u64::from_le_bytes(bytes);
        offset += 8;
        let _ = offset;
        Ok(Self { session, protocol_version, controller, status, device_tick_hz, segment_capacity, event_capacity, step_tick_hz, max_degree, actuator_count, profile, config_digest })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TimeSyncRequest {
    pub host_send_ns: u64,
}

impl TimeSyncRequest {
    pub const SIZE: usize = 8;

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, Error> {
        if out.len() < Self::SIZE { return Err(Error::ShortBuffer); }
        let mut offset = 0usize;
        out[offset..offset + 8].copy_from_slice(&self.host_send_ns.to_le_bytes());
        offset += 8;
        Ok(offset)
    }

    pub fn decode(input: &[u8]) -> Result<Self, Error> {
        if input.len() != Self::SIZE { return Err(Error::WrongLength); }
        let mut offset = 0usize;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let host_send_ns = u64::from_le_bytes(bytes);
        offset += 8;
        let _ = offset;
        Ok(Self { host_send_ns })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TimeSyncReply {
    pub host_send_ns: u64,
    pub device_rx_ticks: u64,
    pub device_tx_ticks: u64,
}

impl TimeSyncReply {
    pub const SIZE: usize = 24;

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, Error> {
        if out.len() < Self::SIZE { return Err(Error::ShortBuffer); }
        let mut offset = 0usize;
        out[offset..offset + 8].copy_from_slice(&self.host_send_ns.to_le_bytes());
        offset += 8;
        out[offset..offset + 8].copy_from_slice(&self.device_rx_ticks.to_le_bytes());
        offset += 8;
        out[offset..offset + 8].copy_from_slice(&self.device_tx_ticks.to_le_bytes());
        offset += 8;
        Ok(offset)
    }

    pub fn decode(input: &[u8]) -> Result<Self, Error> {
        if input.len() != Self::SIZE { return Err(Error::WrongLength); }
        let mut offset = 0usize;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let host_send_ns = u64::from_le_bytes(bytes);
        offset += 8;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let device_rx_ticks = u64::from_le_bytes(bytes);
        offset += 8;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let device_tx_ticks = u64::from_le_bytes(bytes);
        offset += 8;
        let _ = offset;
        Ok(Self { host_send_ns, device_rx_ticks, device_tx_ticks })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QueueBegin6 {
    pub queue_revision: u64,
    pub replace_after_ticks: u64,
    pub expected_position: [f32; 64],
    pub expected_velocity: [f32; 64],
    pub actuator_count: u8,
}

impl QueueBegin6 {
    pub const SIZE: usize = 529;

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, Error> {
        if out.len() < Self::SIZE { return Err(Error::ShortBuffer); }
        let mut offset = 0usize;
        out[offset..offset + 8].copy_from_slice(&self.queue_revision.to_le_bytes());
        offset += 8;
        out[offset..offset + 8].copy_from_slice(&self.replace_after_ticks.to_le_bytes());
        offset += 8;
        for value in self.expected_position {
            out[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
            offset += 4;
        }
        for value in self.expected_velocity {
            out[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
            offset += 4;
        }
        out[offset..offset + 1].copy_from_slice(&self.actuator_count.to_le_bytes());
        offset += 1;
        Ok(offset)
    }

    pub fn decode(input: &[u8]) -> Result<Self, Error> {
        if input.len() != Self::SIZE { return Err(Error::WrongLength); }
        let mut offset = 0usize;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let queue_revision = u64::from_le_bytes(bytes);
        offset += 8;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let replace_after_ticks = u64::from_le_bytes(bytes);
        offset += 8;
        let mut expected_position = [0 as f32; 64];
        for item in &mut expected_position {
            let mut bytes = [0u8; 4];
            bytes.copy_from_slice(&input[offset..offset + 4]);
            *item = f32::from_le_bytes(bytes);
            offset += 4;
        }
        let mut expected_velocity = [0 as f32; 64];
        for item in &mut expected_velocity {
            let mut bytes = [0u8; 4];
            bytes.copy_from_slice(&input[offset..offset + 4]);
            *item = f32::from_le_bytes(bytes);
            offset += 4;
        }
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let actuator_count = u8::from_le_bytes(bytes);
        offset += 1;
        let _ = offset;
        Ok(Self { queue_revision, replace_after_ticks, expected_position, expected_velocity, actuator_count })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Segment6Header {
    pub queue_revision: u64,
    pub plan_id: u64,
    pub t0_ticks: u64,
    pub duration_ticks: u64,
    pub degree: u8,
    pub actuator_count: u8,
    pub ends_at_rest: u8,
    pub reserved: u8,
}

impl Segment6Header {
    pub const SIZE: usize = 36;

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, Error> {
        if out.len() < Self::SIZE { return Err(Error::ShortBuffer); }
        let mut offset = 0usize;
        out[offset..offset + 8].copy_from_slice(&self.queue_revision.to_le_bytes());
        offset += 8;
        out[offset..offset + 8].copy_from_slice(&self.plan_id.to_le_bytes());
        offset += 8;
        out[offset..offset + 8].copy_from_slice(&self.t0_ticks.to_le_bytes());
        offset += 8;
        out[offset..offset + 8].copy_from_slice(&self.duration_ticks.to_le_bytes());
        offset += 8;
        out[offset..offset + 1].copy_from_slice(&self.degree.to_le_bytes());
        offset += 1;
        out[offset..offset + 1].copy_from_slice(&self.actuator_count.to_le_bytes());
        offset += 1;
        out[offset..offset + 1].copy_from_slice(&self.ends_at_rest.to_le_bytes());
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
        let queue_revision = u64::from_le_bytes(bytes);
        offset += 8;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let plan_id = u64::from_le_bytes(bytes);
        offset += 8;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let t0_ticks = u64::from_le_bytes(bytes);
        offset += 8;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let duration_ticks = u64::from_le_bytes(bytes);
        offset += 8;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let degree = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let actuator_count = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let ends_at_rest = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let reserved = u8::from_le_bytes(bytes);
        offset += 1;
        let _ = offset;
        Ok(Self { queue_revision, plan_id, t0_ticks, duration_ticks, degree, actuator_count, ends_at_rest, reserved })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Segment6Coefficients {
    pub actuator: u8,
    pub c0: f32,
    pub c1: f32,
    pub c2: f32,
    pub c3: f32,
    pub c4: f32,
    pub c5: f32,
}

impl Segment6Coefficients {
    pub const SIZE: usize = 25;

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, Error> {
        if out.len() < Self::SIZE { return Err(Error::ShortBuffer); }
        let mut offset = 0usize;
        out[offset..offset + 1].copy_from_slice(&self.actuator.to_le_bytes());
        offset += 1;
        out[offset..offset + 4].copy_from_slice(&self.c0.to_le_bytes());
        offset += 4;
        out[offset..offset + 4].copy_from_slice(&self.c1.to_le_bytes());
        offset += 4;
        out[offset..offset + 4].copy_from_slice(&self.c2.to_le_bytes());
        offset += 4;
        out[offset..offset + 4].copy_from_slice(&self.c3.to_le_bytes());
        offset += 4;
        out[offset..offset + 4].copy_from_slice(&self.c4.to_le_bytes());
        offset += 4;
        out[offset..offset + 4].copy_from_slice(&self.c5.to_le_bytes());
        offset += 4;
        Ok(offset)
    }

    pub fn decode(input: &[u8]) -> Result<Self, Error> {
        if input.len() != Self::SIZE { return Err(Error::WrongLength); }
        let mut offset = 0usize;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let actuator = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 4];
        bytes.copy_from_slice(&input[offset..offset + 4]);
        let c0 = f32::from_le_bytes(bytes);
        offset += 4;
        let mut bytes = [0u8; 4];
        bytes.copy_from_slice(&input[offset..offset + 4]);
        let c1 = f32::from_le_bytes(bytes);
        offset += 4;
        let mut bytes = [0u8; 4];
        bytes.copy_from_slice(&input[offset..offset + 4]);
        let c2 = f32::from_le_bytes(bytes);
        offset += 4;
        let mut bytes = [0u8; 4];
        bytes.copy_from_slice(&input[offset..offset + 4]);
        let c3 = f32::from_le_bytes(bytes);
        offset += 4;
        let mut bytes = [0u8; 4];
        bytes.copy_from_slice(&input[offset..offset + 4]);
        let c4 = f32::from_le_bytes(bytes);
        offset += 4;
        let mut bytes = [0u8; 4];
        bytes.copy_from_slice(&input[offset..offset + 4]);
        let c5 = f32::from_le_bytes(bytes);
        offset += 4;
        let _ = offset;
        Ok(Self { actuator, c0, c1, c2, c3, c4, c5 })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Commit6 {
    pub through_ticks: u64,
}

impl Commit6 {
    pub const SIZE: usize = 8;

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, Error> {
        if out.len() < Self::SIZE { return Err(Error::ShortBuffer); }
        let mut offset = 0usize;
        out[offset..offset + 8].copy_from_slice(&self.through_ticks.to_le_bytes());
        offset += 8;
        Ok(offset)
    }

    pub fn decode(input: &[u8]) -> Result<Self, Error> {
        if input.len() != Self::SIZE { return Err(Error::WrongLength); }
        let mut offset = 0usize;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let through_ticks = u64::from_le_bytes(bytes);
        offset += 8;
        let _ = offset;
        Ok(Self { through_ticks })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QueueStatus6 {
    pub queue_revision: u64,
    pub committed_until_ticks: u64,
    pub executing_plan_id: u64,
    pub executing_segment: u16,
    pub path_clock_ticks: u64,
    pub rate: f32,
    pub remaining_segments: u16,
    pub remaining_events: u16,
    pub underflow: u8,
    pub fault: u8,
    pub received_until_ticks: u64,
    pub received_bytes: u64,
}

impl QueueStatus6 {
    pub const SIZE: usize = 60;

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, Error> {
        if out.len() < Self::SIZE { return Err(Error::ShortBuffer); }
        let mut offset = 0usize;
        out[offset..offset + 8].copy_from_slice(&self.queue_revision.to_le_bytes());
        offset += 8;
        out[offset..offset + 8].copy_from_slice(&self.committed_until_ticks.to_le_bytes());
        offset += 8;
        out[offset..offset + 8].copy_from_slice(&self.executing_plan_id.to_le_bytes());
        offset += 8;
        out[offset..offset + 2].copy_from_slice(&self.executing_segment.to_le_bytes());
        offset += 2;
        out[offset..offset + 8].copy_from_slice(&self.path_clock_ticks.to_le_bytes());
        offset += 8;
        out[offset..offset + 4].copy_from_slice(&self.rate.to_le_bytes());
        offset += 4;
        out[offset..offset + 2].copy_from_slice(&self.remaining_segments.to_le_bytes());
        offset += 2;
        out[offset..offset + 2].copy_from_slice(&self.remaining_events.to_le_bytes());
        offset += 2;
        out[offset..offset + 1].copy_from_slice(&self.underflow.to_le_bytes());
        offset += 1;
        out[offset..offset + 1].copy_from_slice(&self.fault.to_le_bytes());
        offset += 1;
        out[offset..offset + 8].copy_from_slice(&self.received_until_ticks.to_le_bytes());
        offset += 8;
        out[offset..offset + 8].copy_from_slice(&self.received_bytes.to_le_bytes());
        offset += 8;
        Ok(offset)
    }

    pub fn decode(input: &[u8]) -> Result<Self, Error> {
        if input.len() != Self::SIZE { return Err(Error::WrongLength); }
        let mut offset = 0usize;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let queue_revision = u64::from_le_bytes(bytes);
        offset += 8;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let committed_until_ticks = u64::from_le_bytes(bytes);
        offset += 8;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let executing_plan_id = u64::from_le_bytes(bytes);
        offset += 8;
        let mut bytes = [0u8; 2];
        bytes.copy_from_slice(&input[offset..offset + 2]);
        let executing_segment = u16::from_le_bytes(bytes);
        offset += 2;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let path_clock_ticks = u64::from_le_bytes(bytes);
        offset += 8;
        let mut bytes = [0u8; 4];
        bytes.copy_from_slice(&input[offset..offset + 4]);
        let rate = f32::from_le_bytes(bytes);
        offset += 4;
        let mut bytes = [0u8; 2];
        bytes.copy_from_slice(&input[offset..offset + 2]);
        let remaining_segments = u16::from_le_bytes(bytes);
        offset += 2;
        let mut bytes = [0u8; 2];
        bytes.copy_from_slice(&input[offset..offset + 2]);
        let remaining_events = u16::from_le_bytes(bytes);
        offset += 2;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let underflow = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let fault = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let received_until_ticks = u64::from_le_bytes(bytes);
        offset += 8;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let received_bytes = u64::from_le_bytes(bytes);
        offset += 8;
        let _ = offset;
        Ok(Self { queue_revision, committed_until_ticks, executing_plan_id, executing_segment, path_clock_ticks, rate, remaining_segments, remaining_events, underflow, fault, received_until_ticks, received_bytes })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct State6Header {
    pub session: u64,
    pub timestamp_ticks: u64,
    pub accepted_sequence: u64,
    pub safety: u8,
    pub fault: u8,
    pub actuator_count: u8,
    pub reserved: u8,
    pub path_clock_ticks: u64,
}

impl State6Header {
    pub const SIZE: usize = 36;

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, Error> {
        if out.len() < Self::SIZE { return Err(Error::ShortBuffer); }
        let mut offset = 0usize;
        out[offset..offset + 8].copy_from_slice(&self.session.to_le_bytes());
        offset += 8;
        out[offset..offset + 8].copy_from_slice(&self.timestamp_ticks.to_le_bytes());
        offset += 8;
        out[offset..offset + 8].copy_from_slice(&self.accepted_sequence.to_le_bytes());
        offset += 8;
        out[offset..offset + 1].copy_from_slice(&self.safety.to_le_bytes());
        offset += 1;
        out[offset..offset + 1].copy_from_slice(&self.fault.to_le_bytes());
        offset += 1;
        out[offset..offset + 1].copy_from_slice(&self.actuator_count.to_le_bytes());
        offset += 1;
        out[offset..offset + 1].copy_from_slice(&self.reserved.to_le_bytes());
        offset += 1;
        out[offset..offset + 8].copy_from_slice(&self.path_clock_ticks.to_le_bytes());
        offset += 8;
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
        let timestamp_ticks = u64::from_le_bytes(bytes);
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
        let actuator_count = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 1];
        bytes.copy_from_slice(&input[offset..offset + 1]);
        let reserved = u8::from_le_bytes(bytes);
        offset += 1;
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let path_clock_ticks = u64::from_le_bytes(bytes);
        offset += 8;
        let _ = offset;
        Ok(Self { session, timestamp_ticks, accepted_sequence, safety, fault, actuator_count, reserved, path_clock_ticks })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ActuatorState6 {
    pub position: f32,
    pub velocity: f32,
    pub effort: f32,
    pub step_count: i64,
}

impl ActuatorState6 {
    pub const SIZE: usize = 20;

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, Error> {
        if out.len() < Self::SIZE { return Err(Error::ShortBuffer); }
        let mut offset = 0usize;
        out[offset..offset + 4].copy_from_slice(&self.position.to_le_bytes());
        offset += 4;
        out[offset..offset + 4].copy_from_slice(&self.velocity.to_le_bytes());
        offset += 4;
        out[offset..offset + 4].copy_from_slice(&self.effort.to_le_bytes());
        offset += 4;
        out[offset..offset + 8].copy_from_slice(&self.step_count.to_le_bytes());
        offset += 8;
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
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(&input[offset..offset + 8]);
        let step_count = i64::from_le_bytes(bytes);
        offset += 8;
        let _ = offset;
        Ok(Self { position, velocity, effort, step_count })
    }
}
