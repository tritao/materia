//! Fixed-capacity process events driven by the scheduled path clock.
use crate::device_wire6::{Event6, SessionBegin6};
use crate::{Board, StopReason};

const CHANNELS: usize = 32;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EventError { BadRevision, Committed, Full, Invalid, OutOfOrder }

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FiredEventRecord {
    pub plan_id: u64,
    pub scheduled_path_ticks: u64,
    pub applied_path_ticks: u64,
    pub device_ticks: u64,
    pub channel: u8,
    pub kind: u8,
    pub digital: u8,
}

#[derive(Clone, Copy)]
struct Value {
    kind: u8,
    digital: u8,
    analog: f32,
    argument: f32,
    command: [u8; 48],
}

impl Value {
    const ZERO: Self = Self { kind: 0, digital: 0, analog: 0.0,
        argument: 0.0, command: [0; 48] };
    fn apply<B: Board>(&self, channel: usize, board: &mut B) {
        match self.kind {
            1 => board.set_digital(channel, self.digital != 0),
            2 => board.set_analog(channel, self.analog),
            3 => board.set_process(channel, &self.command, self.argument),
            _ => {}
        }
    }
}

pub struct DeviceEvents<const CAP: usize> {
    events: [Option<Event6>; CAP],
    len: usize,
    next: usize,
    revision: u64,
    replace_after: u64,
    committed_until: u64,
    held: bool,
    stopped: bool,
    channel_count: usize,
    channel_kind: [u8; CHANNELS],
    safe: [Value; CHANNELS],
    live: [Value; CHANNELS],
    /// Channels that keep their output through a commanded stop, as a gripper holding a part must.
    keep_on_stop: [bool; CHANNELS],
    fired: [Value; CHANNELS],
    fired_policy: [u8; CHANNELS],
    has_fired: [bool; CHANNELS],
    records: [Option<FiredEventRecord>; CAP],
    record_count: usize,
}

impl<const CAP: usize> DeviceEvents<CAP> {
    pub fn new(session: &SessionBegin6) -> Self {
        let mut safe = [Value::ZERO; CHANNELS];
        let mut keep_on_stop = [false; CHANNELS];
        for (i, keep) in keep_on_stop.iter_mut().enumerate().take(session.channel_count as usize) {
            *keep = session.channel_stop_policy[i] == 1;
        }
        for (i, value) in safe.iter_mut().enumerate().take(session.channel_count as usize) {
            let mut command = [0; 48];
            command.copy_from_slice(&session.safe_command[i * 48..(i + 1) * 48]);
            *value = Value { kind: session.channel_kind[i],
                digital: session.safe_digital[i], analog: session.safe_analog[i],
                argument: session.safe_argument[i], command };
        }
        Self { events: [None; CAP], len: 0, next: 0, revision: 0,
            replace_after: 0, committed_until: 0, held: false, stopped: false,
            channel_count: session.channel_count as usize,
            channel_kind: session.channel_kind, safe, live: safe, keep_on_stop, fired: [Value::ZERO; CHANNELS],
            fired_policy: [0; CHANNELS], has_fired: [false; CHANNELS],
            records: [None; CAP], record_count: 0 }
    }

    pub fn remaining_capacity(&self) -> usize { CAP - self.len }
    /// A safe-on-stop output away from its safe value still needs a live owner,
    /// even after the arm reaches rest. Held channels already made safe do not.
    pub fn requires_link(&self) -> bool {
        if self.stopped { return false; }
        (0..self.channel_count).any(|i| {
            if self.keep_on_stop[i] { return false; }
            let value = self.live[i]; let safe = self.safe[i];
            match value.kind {
                1 => value.digital != safe.digital,
                2 => value.analog != safe.analog,
                3 => value.command != safe.command || value.argument != safe.argument,
                _ => false,
            }
        })
    }
    pub fn records(&self) -> &[Option<FiredEventRecord>] {
        &self.records[..self.record_count]
    }

    pub fn queue_begin(&mut self, revision: u64, replace_after: u64,
        committed_until: u64) -> Result<(), EventError> {
        if revision <= self.revision || replace_after < committed_until {
            return Err(EventError::Committed);
        }
        let mut keep = 0;
        while keep < self.len && self.events[keep].unwrap().path_ticks < replace_after {
            keep += 1;
        }
        for slot in &mut self.events[keep..] { *slot = None; }
        self.len = keep;
        self.next = self.next.min(keep);
        self.revision = revision;
        self.replace_after = replace_after;
        self.committed_until = committed_until;
        if self.stopped {
            self.stopped = false;
            self.held = false;
        }
        Ok(())
    }

    pub fn push(&mut self, event: Event6) -> Result<(), EventError> {
        if event.queue_revision != self.revision { return Err(EventError::BadRevision); }
        if event.path_ticks < self.replace_after || event.path_ticks < self.committed_until {
            return Err(EventError::Committed);
        }
        if event.channel as usize >= self.channel_count ||
            event.kind != self.channel_kind[event.channel as usize] ||
            event.hold_policy > 2 || event.digital > 1 ||
            !event.analog.is_finite() || !event.argument.is_finite() {
            return Err(EventError::Invalid);
        }
        if self.len == CAP { return Err(EventError::Full); }
        if self.len > 0 && event.path_ticks < self.events[self.len - 1].unwrap().path_ticks {
            return Err(EventError::OutOfOrder);
        }
        self.events[self.len] = Some(event);
        self.len += 1;
        Ok(())
    }

    pub fn commit(&mut self, through_ticks: u64) {
        self.committed_until = through_ticks;
    }

    pub fn tick<B: Board>(&mut self, path_ticks: u64, board: &mut B) {
        if self.stopped || self.held { return; }
        while self.next < self.len {
            let event = self.events[self.next].unwrap();
            if event.path_ticks > path_ticks || event.path_ticks > self.committed_until { break; }
            let channel = event.channel as usize;
            let value = Value { kind: event.kind, digital: event.digital,
                analog: event.analog, argument: event.argument, command: event.command };
            value.apply(channel, board);
            self.live[channel] = value;
            if self.record_count == CAP {
                self.records.copy_within(1..CAP, 0);
                self.record_count -= 1;
            }
            self.records[self.record_count] = Some(FiredEventRecord {
                plan_id: event.plan_id, scheduled_path_ticks: event.path_ticks,
                applied_path_ticks: path_ticks, device_ticks: board.now_ticks(),
                channel: event.channel, kind: event.kind, digital: event.digital });
            self.record_count += 1;
            self.fired[channel] = value;
            self.fired_policy[channel] = event.hold_policy;
            self.has_fired[channel] = true;
            self.next += 1;
        }
        if self.next == self.len {
            self.events.fill(None);
            self.len = 0;
            self.next = 0;
        }
    }

    pub fn hold<B: Board>(&mut self, board: &mut B) {
        self.held = true;
        for i in 0..self.channel_count {
            if self.has_fired[i] && self.fired_policy[i] != 0 {
                self.safe[i].apply(i, board);
                self.live[i] = self.safe[i];
            }
        }
    }

    pub fn resume<B: Board>(&mut self, board: &mut B) {
        for i in 0..self.channel_count {
            if self.has_fired[i] && self.fired_policy[i] == 2 {
                self.fired[i].apply(i, board);
                self.live[i] = self.fired[i];
            }
        }
        self.held = false;
    }

    pub fn stop<B: Board>(&mut self, board: &mut B, reason: StopReason) {
        if self.stopped { return; }
        self.apply_safe(board, reason);
        self.events.fill(None);
        self.len = 0;
        self.next = 0;
        self.stopped = true;
    }

    /// Every channel to its safe value, but a commanded stop or abort leaves the channels that
    /// keep their output on stop as they are; an emergency stop or a fault safes them all.
    pub fn apply_safe<B: Board>(&self, board: &mut B, reason: StopReason) {
        let commanded = matches!(reason, StopReason::Stop | StopReason::Abort);
        for i in 0..self.channel_count {
            if commanded && self.keep_on_stop[i] { continue; }
            self.safe[i].apply(i, board);
        }
    }
}
