//! In-process RKD6 device. The C ABI only moves complete validated frames.
use robotkit_device_protocol::config_digest::{config_digest6, controller_matches};
use robotkit_device_protocol::device_wire6::*;
use robotkit_device_protocol::frame6::{decode_frame6, encode_frame6, MAX_FRAME_SIZE};
use robotkit_device_protocol::{
    Board, DeviceEvents, InputBinding, Output, ScheduledCore, ScheduledSegment, SkewGroup,
    StepGenerator, StopReason, VirtualBoard, VirtualSwitch,
};
use std::collections::VecDeque;
pub mod welder;
#[cfg(test)]
mod retrofit_tests;

const ACTUATORS: usize = 64;
const CHANNELS: usize = 32;
const CAPACITY: usize = 128;
const EVENT_CAPACITY: usize = 256;
const MINIMAL_CAPACITY: usize = 8;

pub struct VirtualDevice {
    board: VirtualBoard<ACTUATORS, CHANNELS>,
    core: Option<ScheduledCore<ACTUATORS, CAPACITY>>,
    events: Option<DeviceEvents<EVENT_CAPACITY>>,
    final_safe_applied: bool,
    steps: StepGenerator<ACTUATORS>,
    controller: [u8; 16],
    count: usize,
    active_count: usize,
    session: u64,
    channel_kind: [u8; CHANNELS],
    input_count: usize,
    actuator_joint: [u8; ACTUATORS],
    actuator_ratio: [f32; ACTUATORS],
    homing_scope: Option<u64>,
    control_sequence: u64,
    homing_stop: bool,
    step_tick_hz: u32,
    profile: u8,
    host_ns: u64,
    last_publish_ns: u64,
    outbox: VecDeque<Vec<u8>>,
    /// Bytes received since the session began, as the status reports them.
    received_bytes: u64,
    welder: Option<welder::VirtualWelder>,
    weld_sequence: u64,
}

impl VirtualDevice {
    pub fn new(
        tick_hz: u64,
        step_tick_hz: u32,
        offset_ticks: u64,
        drift_ppm: i32,
        count: usize,
        steps_per_unit: [f64; ACTUATORS],
        controller: [u8; 16],
        profile: u8,
    ) -> Option<Self> {
        if tick_hz == 0
            || step_tick_hz == 0
            || count == 0
            || count > ACTUATORS
            || drift_ppm <= -1_000_000
            || steps_per_unit.iter().any(|v| !v.is_finite() || *v <= 0.0)
            || (profile != 1 && profile != 2)
        {
            return None;
        }
        Some(Self {
            steps: StepGenerator::new(steps_per_unit, [0; ACTUATORS],
                [0.0; ACTUATORS], tick_hz)?,
            board: VirtualBoard::new_with_actuator_count(
                tick_hz, offset_ticks, drift_ppm, steps_per_unit, count),
            core: None,
            events: None,
            final_safe_applied: false,
            controller,
            count,
            active_count: 0,
            session: 0,
            channel_kind: [0; CHANNELS],
            input_count: 0,
            actuator_joint: [0; ACTUATORS],
            actuator_ratio: [1.0; ACTUATORS],
            homing_scope: None,
            control_sequence: 0,
            homing_stop: false,
            step_tick_hz,
            profile,
            host_ns: 0,
            last_publish_ns: 0,
            outbox: VecDeque::new(),
            received_bytes: 0,
            welder: None,
            weld_sequence: 0,
        })
    }

    /// The process profile is configured before opening its deployment session.
    pub fn configure_welder(&mut self, config: welder::WelderConfig) -> bool {
        if self.core.is_some() { return false; }
        self.welder = welder::VirtualWelder::new(config);
        self.welder.is_some()
    }

    pub fn set_welder_grounded(&mut self, grounded: bool) -> bool {
        let Some(welder) = self.welder.as_mut() else { return false; };
        welder.grounded = grounded;
        true
    }

    pub fn weld_values(&self) -> Option<[f32; 6]> { self.welder.as_ref().map(|w| w.values()) }

    fn valid_weld_channels(&self, begin: &SessionBegin6) -> bool {
        let Some(welder) = self.welder.as_ref() else { return true; };
        let c = welder.config;
        [c.arc_channel, c.wire_channel, c.voltage_channel].iter()
            .all(|&i| i < begin.channel_count as usize)
            && begin.channel_kind[c.arc_channel] == 1
            && begin.channel_kind[c.wire_channel] == 2
            && begin.channel_kind[c.voltage_channel] == 2
            && begin.safe_digital[c.arc_channel] == 0
            && begin.safe_analog[c.wire_channel] == 0.0
            && begin.channel_stop_policy[c.arc_channel] == 0
            && begin.channel_stop_policy[c.wire_channel] == 0
    }

    fn update_welder(&mut self, dt: f64) {
        let Some(welder) = self.welder.as_mut() else { return; };
        let c = welder.config;
        welder.tick(dt, self.board.digital(c.arc_channel).unwrap_or(false),
            self.board.analog(c.wire_channel).unwrap_or(0.0),
            self.board.analog(c.voltage_channel).unwrap_or(0.0));
        if welder.faulted() {
            if let Some(core) = self.core.as_mut() {
                if core.stop_reason().is_none() { core.stop(StopReason::ProcessFault); }
                self.events.as_mut().unwrap().stop(&mut self.board, StopReason::ProcessFault);
                self.events.as_ref().unwrap().apply_safe(&mut self.board, StopReason::ProcessFault);
                welder.tick(0.0, false, 0.0, 0.0);
                self.final_safe_applied = true;
            }
        }
    }

    fn emit(&mut self, kind: u8, payload: &[u8]) {
        let mut frame = vec![0; MAX_FRAME_SIZE];
        if let Ok(size) = encode_frame6(kind, payload, &mut frame) {
            frame.truncate(size);
            self.outbox.push_back(frame);
        }
    }

    pub fn feed(&mut self, frame: &[u8]) -> bool {
        // Every byte off the line counts, valid or not: the host measures what is in flight by it.
        self.received_bytes += frame.len() as u64;
        let Ok((kind, payload)) = decode_frame6(frame) else {
            return false;
        };
        let now = self.board.now_ticks();
        if kind != 1 {
            let Some(core) = self.core.as_mut() else {
                return false;
            };
            core.note_host_frame(now);
        }
        match kind {
            1 => {
                let Ok(begin) = SessionBegin6::decode(&payload[..SessionBegin6::SIZE]) else {
                    return false;
                };
                let mut ack = SessionAck6 {
                    session: begin.session,
                    protocol_version: PROTOCOL_VERSION,
                    controller: self.controller,
                    config_digest: config_digest6(&payload[..SessionBegin6::SIZE]),
                    status: 0,
                    device_tick_hz: self.board.tick_hz(),
                    segment_capacity: if self.profile == 2 { MINIMAL_CAPACITY as u16 } else { CAPACITY as u16 },
                    event_capacity: EVENT_CAPACITY as u16,
                    step_tick_hz: self.step_tick_hz,
                    max_degree: if self.profile == 2 { 1 } else { 5 },
                    actuator_count: self.count as u8,
                    profile: self.profile,
                };
                if self.valid_weld_channels(&begin)
                    && controller_matches(&begin.expected_controller, &self.controller)
                    && begin.actuator_count > 0
                    && begin.actuator_count as usize <= self.count
                    && begin.step_tick_hz == self.step_tick_hz
                    && begin.session != 0
                    && (0..begin.actuator_count as usize).all(|i| {
                        let physical = self.board.steps_per_unit()[i];
                        ((begin.steps_per_unit[i] as f64 - physical) / physical).abs() < 1e-6
                    })
                {
                    let mut limits = [1.0f32; ACTUATORS];
                    for (i, limit) in limits.iter_mut().enumerate().take(begin.actuator_count as usize) {
                        *limit = begin.actuator_max_acceleration[i];
                    }
                    let mut steps_per_unit = [1.0; ACTUATORS];
                    let mut max_rate = [0.0; ACTUATORS];
                    let mut setup = [0; ACTUATORS];
                    for i in 0..begin.actuator_count as usize {
                        steps_per_unit[i] = begin.steps_per_unit[i] as f64;
                        max_rate[i] = begin.max_rate[i] as f64;
                        setup[i] = begin.direction_setup_ticks[i] as u64;
                    }
                    let Some(mut generator) = StepGenerator::new(
                        steps_per_unit, setup, max_rate, self.board.tick_hz()) else {
                        return false;
                    };
                    for i in 0..begin.actuator_count as usize {
                        for j in i + 1..begin.actuator_count as usize {
                            if begin.actuator_joint[i] == begin.actuator_joint[j] {
                                let bound = begin.dual_drive_skew_bound[i]
                                    .max(begin.dual_drive_skew_bound[j]) as f64;
                                if bound > 0.0 {
                                    if !generator.set_skew_group(SkewGroup { first: i, second: j,
                                        first_ratio: begin.actuator_ratio[i] as f64,
                                        second_ratio: begin.actuator_ratio[j] as f64, bound }) {
                                        return false;
                                    }
                                }
                            }
                        }
                    }
                    for channel in 0..begin.input_count as usize {
                        if !generator.bind_input(&self.board, channel, InputBinding {
                            actuator: begin.input_actuator[channel] as usize,
                            active_high: begin.input_active_high & (1u64 << channel) != 0,
                        }, self.count) { return false; }
                    }
                    let link_loss_ticks = ((begin.link_loss_timeout_ns as u128
                        * self.board.tick_hz() as u128)
                        / 1_000_000_000) as u64;
                    let mut core = ScheduledCore::new(
                        self.board.tick_hz(),
                        limits,
                        [-1.0e12; ACTUATORS],
                        [1.0e12; ACTUATORS],
                        link_loss_ticks.max(1),
                    );
                    core.initialize_clock(self.board.now_ticks());
                    if let Some(welder) = self.welder.as_mut() { welder.reset(); }
                    self.weld_sequence = 0;
                    self.steps = generator;
                    self.actuator_joint = begin.actuator_joint;
                    self.actuator_ratio = begin.actuator_ratio;
                    self.homing_scope = None;
                    self.control_sequence = 0;
                    self.homing_stop = false;
                    self.core = Some(core);
                    self.events = Some(DeviceEvents::new(&begin));
                    self.final_safe_applied = false;
                    self.active_count = begin.actuator_count as usize;
                    self.session = begin.session;
                    // The count starts after the frame that begins the session, as the host's does.
                    self.received_bytes = 0;
                    self.channel_kind = begin.channel_kind;
                    self.input_count = begin.input_count as usize;
                    ack.status = 1;
                    ack.actuator_count = begin.actuator_count;
                }
                let mut bytes = [0; SessionAck6::SIZE];
                ack.encode(&mut bytes).unwrap();
                self.emit(2, &bytes);
                self.publish_state();
                true
            }
            20 => {
                let command = HomingCounterBatch6::decode(payload).unwrap();
                let mut accepted = false;
                if command.session == self.session && command.sequence > self.control_sequence &&
                    self.profile == 1 {
                    self.control_sequence = command.sequence;
                    let core = self.core.as_ref().unwrap();
                    if self.homing_scope == Some(command.scope) && self.homing_stop && core.is_stopped() &&
                        core.remaining_capacity() == CAPACITY &&
                        core.velocities().iter().all(|value| value.abs() <= 1e-6) {
                        accepted = self.steps.rebase_homing_counters(&[
                            (command.first as usize, command.first_delta),
                            (command.second as usize, command.second_delta),
                        ]);
                    }
                }
                let ack = HomingControlAck6 { session: self.session, sequence: command.sequence,
                    scope: command.scope, accepted: accepted as u8 };
                let mut bytes = [0; HomingControlAck6::SIZE];
                ack.encode(&mut bytes).unwrap(); self.emit(19, &bytes);
                if accepted { self.publish_state(); }
                true
            }
            17 | 18 => {
                let (session, sequence, scope) = if kind == 17 {
                    let command = HomingScope6::decode(payload).unwrap();
                    (command.session, command.sequence, command.scope)
                } else {
                    let command = HomingSide6::decode(payload).unwrap();
                    (command.session, command.sequence, command.scope)
                };
                let mut accepted = false;
                if session == self.session && sequence > self.control_sequence && self.profile == 1 {
                    self.control_sequence = sequence;
                    if kind == 17 {
                        let command = HomingScope6::decode(payload).unwrap();
                        if command.action == 0 && self.homing_scope.is_none() &&
                            self.core.as_ref().unwrap().remaining_capacity() == CAPACITY &&
                            self.core.as_ref().unwrap().velocities().iter().all(|v| v.abs() <= 1e-6) {
                            accepted = self.steps.begin_homing_pair(command.first as usize,
                                command.second as usize, command.skew_bound as f64);
                            if accepted { self.homing_scope = Some(scope); }
                        } else if command.action == 2 && self.homing_scope == Some(scope) {
                            self.core.as_mut().unwrap().stop(StopReason::Stop);
                            self.events.as_mut().unwrap().stop(&mut self.board, StopReason::Stop);
                            self.homing_stop = true; accepted = true;
                        } else if command.action == 1 && self.homing_scope == Some(scope) {
                            self.steps.end_homing_pair(); self.homing_scope = None; self.homing_stop = false; accepted = true;
                        }
                    } else if self.homing_scope == Some(scope) {
                        let command = HomingSide6::decode(payload).unwrap();
                        accepted = if command.hold != 0 {
                            self.steps.hold_homing_side(&self.board, command.actuator as usize)
                        } else { self.steps.release_homing_side(command.actuator as usize) };
                    }
                }
                let ack = HomingControlAck6 { session: self.session, sequence, scope, accepted: accepted as u8 };
                let mut bytes = [0; HomingControlAck6::SIZE];
                ack.encode(&mut bytes).unwrap(); self.emit(19, &bytes);
                if accepted { self.publish_state(); }
                true
            }
            3 => {
                let Ok(request) = TimeSyncRequest::decode(payload) else {
                    return false;
                };
                let reply = TimeSyncReply {
                    host_send_ns: request.host_send_ns,
                    device_rx_ticks: now,
                    device_tx_ticks: now,
                };
                let mut bytes = [0; TimeSyncReply::SIZE];
                reply.encode(&mut bytes).unwrap();
                self.emit(4, &bytes);
                true
            }
            5 => {
                let Ok(begin) = QueueBegin6::decode(payload) else {
                    return false;
                };
                if begin.actuator_count as usize != self.active_count {
                    return false;
                }
                let stopped = self.core.as_ref().unwrap().is_stopped() &&
                    self.core.as_ref().unwrap().stop_reason() == Some(StopReason::Stop);
                if stopped {
                    let mut positions = self.core.as_ref().unwrap().positions();
                    if self.profile == 1 {
                        let Some(projected) = self.steps.stopped_targets(&self.board,
                            &self.actuator_joint, &self.actuator_ratio, self.count) else { return false; };
                        positions = projected;
                    }
                    if self.core.as_mut().unwrap().prepare_stopped_queue(now, positions).is_err() {
                        return false;
                    }
                }
                let result = self.core
                    .as_mut()
                    .unwrap()
                    .queue_begin_with_state(
                        begin.queue_revision,
                        begin.replace_after_ticks,
                        begin.expected_position,
                        begin.expected_velocity,
                    );
                if let Err(error) = result {
                    eprintln!("Virtual RKD6 rejected QueueBegin: {:?}, revision={}, boundary={}",
                        error, begin.queue_revision, begin.replace_after_ticks);
                    let core = self.core.as_ref().unwrap();
                    eprintln!("  expected positions={:?} velocities={:?}; actual positions={:?} velocities={:?}",
                        &begin.expected_position[..self.count], &begin.expected_velocity[..self.count],
                        &core.positions()[..self.count], &core.velocities()[..self.count]);
                    return false;
                }
                if stopped && self.profile == 1 &&
                    !self.steps.anchor_stopped_targets(&self.board, self.core.as_ref().unwrap().positions()) {
                    return false;
                }
                self.events.as_mut().unwrap().queue_begin(begin.queue_revision,
                    begin.replace_after_ticks, self.core.as_ref().unwrap().committed_until()).is_ok()
            }
            6 => {
                let Ok(header) = Segment6Header::decode(&payload[..Segment6Header::SIZE]) else {
                    return false;
                };
                if header.actuator_count as usize != self.active_count
                    || (self.profile == 2 && (header.degree > 1 ||
                        self.core.as_ref().unwrap().remaining_capacity() <= CAPACITY - MINIMAL_CAPACITY))
                {
                    return false;
                }
                let mut coefficients = [[0.0f32; 6]; ACTUATORS];
                for (i, slot) in coefficients.iter_mut().enumerate().take(self.active_count) {
                    let start = Segment6Header::SIZE + i * Segment6Coefficients::SIZE;
                    let Ok(row) = Segment6Coefficients::decode(
                        &payload[start..start + Segment6Coefficients::SIZE],
                    ) else {
                        return false;
                    };
                    if row.actuator as usize != i {
                        return false;
                    }
                    *slot = [row.c0, row.c1, row.c2, row.c3, row.c4, row.c5];
                }
                let Ok(segment) = ScheduledSegment::new_with_purpose(
                    header.plan_id,
                    header.t0_ticks,
                    header.duration_ticks,
                    header.degree,
                    coefficients,
                    header.ends_at_rest != 0, header.purpose,
                ) else {
                    return false;
                };
                self.core.as_mut().unwrap()
                    .push_segment_for_revision(header.queue_revision, segment).is_ok()
            }
            7 => {
                let Ok(commit) = Commit6::decode(payload) else {
                    return false;
                };
                let result = self.core
                    .as_mut()
                    .unwrap()
                    .commit(commit.through_ticks);
                if result.is_err() { return false; }
                self.events.as_mut().unwrap().commit(commit.through_ticks);
                true
            }
            8 => {
                self.core.as_mut().unwrap().hold();
                self.events.as_mut().unwrap().hold(&mut self.board);
                true
            }
            9 => {
                self.core.as_mut().unwrap().resume();
                self.events.as_mut().unwrap().resume(&mut self.board);
                true
            }
            10 => {
                self.steps.end_homing_pair(); self.homing_scope = None; self.homing_stop = false;
                self.core.as_mut().unwrap().abort();
                if let Some(reason) = self.core.as_ref().unwrap().stop_reason() {
                    self.events.as_mut().unwrap().stop(&mut self.board, reason);
                }
                self.update_welder(0.0);
                true
            }
            11 => {
                self.steps.end_homing_pair(); self.homing_scope = None; self.homing_stop = false;
                self.core.as_mut().unwrap().stop(StopReason::Stop);
                self.events.as_mut().unwrap().stop(&mut self.board, StopReason::Stop);
                self.update_welder(0.0);
                true
            }
            12 => {
                self.steps.end_homing_pair(); self.homing_scope = None; self.homing_stop = false;
                self.core.as_mut().unwrap().emergency_stop(&mut self.board);
                self.events.as_mut().unwrap().stop(&mut self.board, StopReason::EmergencyStop);
                self.final_safe_applied = true;
                self.update_welder(0.0);
                true
            }
            13 => false,
            16 => {
                let Ok(event) = Event6::decode(payload) else { return false; };
                self.events.as_mut().unwrap().push(event).is_ok()
            }
            _ => false,
        }
    }

    pub fn advance(&mut self, host_ns: u64) -> bool {
        if host_ns < self.host_ns {
            return false;
        }
        if host_ns == self.host_ns {
            return true;
        }
        let step_ns = (1_000_000_000u64 / self.step_tick_hz as u64).max(1);
        while self.host_ns + step_ns <= host_ns {
            self.host_ns += step_ns;
            self.board.advance_host_ns(self.host_ns);
            if let Some(core) = self.core.as_mut() {
                core.tick_with_process(&mut self.board, self.events.as_ref().unwrap().requires_link());
                if let Some(reason) = core.stop_reason() {
                    self.events.as_mut().unwrap().stop(&mut self.board, reason);
                    if core.is_stopped() && !self.final_safe_applied {
                        self.events.as_ref().unwrap().apply_safe(&mut self.board, reason);
                        self.final_safe_applied = true;
                    }
                } else {
                    self.events.as_mut().unwrap().tick(core.path_clock(), &mut self.board);
                }
                if core.stop_reason().is_some() && !(self.homing_stop && core.stop_reason() == Some(StopReason::Stop)) {
                    self.steps.end_homing_pair(); self.homing_scope = None; self.homing_stop = false;
                }
                let targets = self.board.position_targets();
                let purpose = core.executing_purpose().unwrap_or(if self.homing_scope.is_some() { 2 } else { 0 });
                if self.profile == 1 && self.steps.tick_active_with_purpose(&mut self.board, targets, self.active_count, purpose).is_err() {
                    self.steps.end_homing_pair(); self.homing_scope = None; self.homing_stop = false;
                    core.stop(StopReason::DualDriveSkew);
                }
            }
            self.update_welder(step_ns as f64 / 1_000_000_000.0);
        }
        self.update_welder(0.0);
        if host_ns.saturating_sub(self.last_publish_ns) >= 10_000_000 {
            self.publish_state();
            self.last_publish_ns = host_ns;
        }
        true
    }

    fn publish_state(&mut self) {
        let Some(core) = self.core.as_ref() else {
            return;
        };
        let fault = match core.stop_reason() {
            None => 0,
            Some(StopReason::Stop) => 0,
            Some(StopReason::Underflow) => 2,
            Some(StopReason::LinkLost) => 3,
            Some(StopReason::DualDriveSkew) => 4,
            Some(_) => 1,
        };
        let status = QueueStatus6 {
            queue_revision: core.revision(),
            committed_until_ticks: core.committed_until(),
            executing_plan_id: core.executing_plan_id(),
            executing_segment: core.executing_segment(),
            path_clock_ticks: core.path_clock(),
            rate: core.rate(),
            remaining_segments: if self.profile == 2 {
                core.remaining_capacity().saturating_sub(CAPACITY - MINIMAL_CAPACITY) as u16
            } else { core.remaining_capacity() as u16 },
            remaining_events: self.events.as_ref().map_or(0,
                |events| events.remaining_capacity() as u16),
            underflow: core.underflow() as u8,
            fault,
            received_until_ticks: core.received_until(),
            received_bytes: self.received_bytes,
        };
        let mut bytes = [0; QueueStatus6::SIZE];
        status.encode(&mut bytes).unwrap();
        self.emit(14, &bytes);
        let core = self.core.as_ref().unwrap();
        let header = State6Header {
            session: self.session,
            timestamp_ticks: self.board.now_ticks(),
            accepted_sequence: self.control_sequence,
            safety: if fault != 0 { 3 } else { 0 },
            fault,
            actuator_count: self.active_count as u8,
            reserved: 0,
            path_clock_ticks: core.path_clock(),
            input_count: self.input_count as u8,
            input_bits: (0..self.input_count).fold(0u64, |bits, channel|
                bits | (u64::from(self.board.read_input(channel)) << channel)),
        };
        let targets = self.board.position_targets();
        let velocity = core.velocities();
        let counts = self.board.step_counts();
        let mut body = vec![0; State6Header::SIZE + self.active_count * ActuatorState6::SIZE + self.input_count * InputState6::SIZE];
        header.encode(&mut body[..State6Header::SIZE]).unwrap();
        for i in 0..self.active_count {
            let row = ActuatorState6 {
                position: if self.profile == 2 { targets[i] } else { self.steps.counter_position(&self.board, i).unwrap() as f32 },
                velocity: velocity[i],
                effort: 0.0,
                step_count: counts[i],
            };
            let start = State6Header::SIZE + i * ActuatorState6::SIZE;
            row.encode(&mut body[start..start + ActuatorState6::SIZE])
                .unwrap();
        }
        for channel in 0..self.input_count {
            let observed = self.steps.input_observation(channel).unwrap();
            let row = InputState6 { closing_count: observed.closing_count,
                opening_count: observed.opening_count, captured_steps: observed.captured_steps,
                captured_ticks: observed.captured_ticks };
            let start = State6Header::SIZE + self.active_count * ActuatorState6::SIZE + channel * InputState6::SIZE;
            row.encode(&mut body[start..start + InputState6::SIZE]).unwrap();
        }
        self.emit(15, &body);
        if let Some(welder) = self.welder.as_ref() {
            self.weld_sequence += 1;
            let header = Sensor6Header { session: self.session, timestamp_ticks: self.board.now_ticks(),
                sequence: self.weld_sequence, slot: welder.config.sensor_slot, value_count: 6 };
            let values = welder.values();
            let mut body = [0; Sensor6Header::SIZE + 6 * Sensor6Value::SIZE];
            header.encode(&mut body[..Sensor6Header::SIZE]).unwrap();
            for (i, value) in values.iter().enumerate() {
                let start = Sensor6Header::SIZE + i * Sensor6Value::SIZE;
                Sensor6Value { value: *value }.encode(&mut body[start..start + Sensor6Value::SIZE]).unwrap();
            }
            self.emit(21, &body);
        }
    }

    pub fn take_frame(&mut self) -> Option<Vec<u8>> { self.outbox.pop_front() }
}

#[no_mangle]
pub unsafe extern "C" fn rkd_virtual_create(
    tick_hz: u64,
    step_tick_hz: u32,
    offset_ticks: u64,
    drift_ppm: i32,
    actuator_count: u32,
    steps_per_unit: *const f64,
    controller: *const u8,
    profile: u8,
) -> *mut VirtualDevice {
    if steps_per_unit.is_null() || controller.is_null() || actuator_count as usize > ACTUATORS {
        return std::ptr::null_mut();
    }
    let mut scale = [1.0; ACTUATORS];
    scale[..actuator_count as usize].copy_from_slice(std::slice::from_raw_parts(
        steps_per_unit,
        actuator_count as usize,
    ));
    let mut id = [0; 16];
    id.copy_from_slice(std::slice::from_raw_parts(controller, 16));
    VirtualDevice::new(
        tick_hz,
        step_tick_hz,
        offset_ticks,
        drift_ppm,
        actuator_count as usize,
        scale,
        id,
        profile,
    )
    .map_or(std::ptr::null_mut(), |v| Box::into_raw(Box::new(v)))
}

// Profile 1 is the board's welding simulator. Host transport treats its parameters as opaque data.
#[no_mangle]
pub unsafe extern "C" fn rkd_virtual_configure_peripheral(
    device: *mut VirtualDevice, kind: u32, parameters: *const f64, count: usize,
) -> i32 {
    if parameters.is_null() || kind != 1 || count != 7 { return 0; }
    let p = std::slice::from_raw_parts(parameters, count);
    if p.iter().any(|v| !v.is_finite()) || p[..4].iter().any(|v| *v < 0.0 || v.fract() != 0.0)
        || p[0] >= 8.0 || p[1..4].iter().any(|v| *v >= 32.0) { return 0; }
    rkd_virtual_configure_welder(device, p[0] as u8, p[1] as u32, p[2] as u32,
        p[3] as u32, p[4], p[5], p[6] as f32)
}
#[no_mangle]
pub unsafe extern "C" fn rkd_virtual_set_peripheral_input(
    device: *mut VirtualDevice, input: u32, value: f64,
) -> i32 {
    if input != 0 || (value != 0.0 && value != 1.0) { return 0; }
    rkd_virtual_set_welder_grounded(device, value as u8)
}

#[no_mangle]
pub unsafe extern "C" fn rkd_virtual_configure_welder(
    device: *mut VirtualDevice, sensor_slot: u8, arc_channel: u32, wire_channel: u32,
    voltage_channel: u32, ignition_seconds: f64, no_arc_seconds: f64, efficiency: f32,
) -> i32 {
    let Some(device) = device.as_mut() else { return 0; };
    device.configure_welder(welder::WelderConfig { sensor_slot, arc_channel: arc_channel as usize,
        wire_channel: wire_channel as usize, voltage_channel: voltage_channel as usize,
        ignition_seconds, no_arc_seconds, efficiency }) as i32
}

#[no_mangle]
pub unsafe extern "C" fn rkd_virtual_set_welder_grounded(device: *mut VirtualDevice, grounded: u8) -> i32 {
    let Some(device) = device.as_mut() else { return 0; };
    if grounded > 1 { return 0; }
    device.set_welder_grounded(grounded != 0) as i32
}

/// Install physical switch geometry before opening the deployment session.
#[no_mangle]
pub unsafe extern "C" fn rkd_virtual_configure_switch(
    device: *mut VirtualDevice, channel: u32, actuator: u32,
    threshold_steps: i64, active_above: u8, active_high: u8,
) -> i32 {
    let Some(device) = device.as_mut() else { return 0; };
    if device.core.is_some() || device.profile != 1 || active_above > 1 || active_high > 1 { return 0; }
    device.board.configure_switch(channel as usize, VirtualSwitch {
        actuator: actuator as usize, threshold_steps,
        active_above: active_above != 0, active_high: active_high != 0,
    }) as i32
}

#[no_mangle]
pub unsafe extern "C" fn rkd_virtual_destroy(device: *mut VirtualDevice) {
    if !device.is_null() {
        drop(Box::from_raw(device));
    }
}

#[no_mangle]
pub unsafe extern "C" fn rkd_virtual_step(device: *mut VirtualDevice, host_ns: u64) -> i32 {
    if let Some(device) = device.as_mut() {
        device.advance(host_ns) as i32
    } else {
        0
    }
}

#[no_mangle]
pub unsafe extern "C" fn rkd_virtual_miss_next_steps(
    device: *mut VirtualDevice, actuator: u32, count: u32,
) -> i32 {
    device.as_mut().is_some_and(|v| v.board.miss_next_steps(actuator as usize, count)) as i32
}

#[no_mangle]
pub unsafe extern "C" fn rkd_virtual_link_host_to_device(
    device: *mut VirtualDevice,
    bytes: *const u8,
    length: usize,
) -> i32 {
    if let (Some(device), false) = (device.as_mut(), bytes.is_null()) {
        device.feed(std::slice::from_raw_parts(bytes, length)) as i32
    } else {
        0
    }
}

#[no_mangle]
pub unsafe extern "C" fn rkd_virtual_link_device_to_host(
    device: *mut VirtualDevice,
    bytes: *mut u8,
    capacity: usize,
) -> usize {
    if let (Some(device), false) = (device.as_mut(), bytes.is_null()) {
        let Some(frame) = device.outbox.front() else {
            return 0;
        };
        if capacity < frame.len() {
            return 0;
        }
        std::ptr::copy_nonoverlapping(frame.as_ptr(), bytes, frame.len());
        let len = frame.len();
        device.outbox.pop_front();
        len
    } else {
        0
    }
}

#[no_mangle]
pub unsafe extern "C" fn rkd_virtual_actuator_positions(
    device: *const VirtualDevice,
    positions: *mut f64,
    capacity: usize,
) -> usize {
    if let (Some(device), false) = (device.as_ref(), positions.is_null()) {
        if capacity < device.count {
            return 0;
        }
        let values = device.board.actuator_positions();
        let targets = device.board.position_targets();
        if device.profile == 2 {
            for (i, value) in targets.iter().enumerate().take(device.count) {
                *positions.add(i) = *value as f64;
            }
            return device.count;
        }
        std::ptr::copy_nonoverlapping(values.as_ptr(), positions, device.count);
        device.count
    } else {
        0
    }
}

#[no_mangle]
pub unsafe extern "C" fn rkd_virtual_channel_values(
    device: *const VirtualDevice,
    values: *mut f32,
    capacity: usize,
) -> usize {
    if let (Some(device), false) = (device.as_ref(), values.is_null()) {
        if capacity < CHANNELS {
            return 0;
        }
        for i in 0..CHANNELS {
            *values.add(i) = match device.channel_kind[i] {
                1 => device.board.digital(i).unwrap_or(false) as u8 as f32,
                2 => device.board.analog(i).unwrap_or(0.0),
                3 => device.board.process_argument(i).unwrap_or(0.0),
                _ => 0.0,
            };
        }
        CHANNELS
    } else {
        0
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StepRecord {
    pub ticks: u64,
    pub actuator: u32,
    pub forward: u8,
    pub reserved: [u8; 3],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EventRecord {
    pub plan_id: u64,
    pub scheduled_path_ticks: u64,
    pub applied_path_ticks: u64,
    pub device_ticks: u64,
    pub channel: u32,
    pub kind: u8,
    pub digital: u8,
    pub reserved: [u8; 2],
}

#[no_mangle]
pub unsafe extern "C" fn rkd_virtual_event_log(
    device: *const VirtualDevice, records: *mut EventRecord, capacity: usize,
) -> usize {
    let Some(events) = device.as_ref().and_then(|v| v.events.as_ref()) else { return 0; };
    let rows = events.records();
    if records.is_null() || capacity < rows.len() { return rows.len(); }
    for (i, row) in rows.iter().flatten().enumerate() {
        *records.add(i) = EventRecord { plan_id: row.plan_id,
            scheduled_path_ticks: row.scheduled_path_ticks,
            applied_path_ticks: row.applied_path_ticks, device_ticks: row.device_ticks,
            channel: row.channel as u32, kind: row.kind, digital: row.digital,
            reserved: [0; 2] };
    }
    rows.len()
}

#[no_mangle]
pub unsafe extern "C" fn rkd_virtual_step_log(
    device: *const VirtualDevice,
    records: *mut StepRecord,
    capacity: usize,
) -> usize {
    let Some(device) = device.as_ref() else {
        return 0;
    };
    let count = device
        .board
        .step_records()
        .iter()
        .filter(|row| matches!(row.output, Output::Step(_, _)))
        .count();
    if records.is_null() || capacity < count {
        return count;
    }
    let mut index = 0;
    for row in device.board.step_records() {
        if let Output::Step(actuator, forward) = row.output {
            *records.add(index) = StepRecord {
                ticks: row.ticks,
                actuator: actuator as u32,
                forward: forward as u8,
                reserved: [0; 3],
            };
            index += 1;
        }
    }
    count
}

#[cfg(test)]
mod tests {
    use super::*;

    fn send<T: Sized>(device: &mut VirtualDevice, kind: u8, payload: &[u8]) -> bool {
        let _ = std::mem::size_of::<T>();
        let mut frame = vec![0; MAX_FRAME_SIZE];
        let size = encode_frame6(kind, payload, &mut frame).unwrap();
        device.feed(&frame[..size])
    }

    #[test]
    fn session_queue_and_step_position() {
        let controller = [7; 16];
        let mut scale = [1.0; ACTUATORS];
        scale[0] = 1_000.0;
        let mut device =
            VirtualDevice::new(1_000_000, 40_000, 50_000, 0, 3, scale, controller, 1).unwrap();
        device.board.step_pulse(1, true); // An unused output need not already be at zero.
        let begin = SessionBegin6 {
            session: 9,
            protocol_version: PROTOCOL_VERSION,
            expected_controller: controller,
            actuator_count: 1,
            max_degree: 5,
            step_tick_hz: 40_000,
            max_acceleration: 10.0,
            actuator_max_acceleration: [10.0; 64],
            steps_per_unit: [1_000.0; 64], max_rate: [0.0; 64],
            direction_setup_ticks: [0; 64], actuator_joint: [0; 64],
            actuator_ratio: [1.0; 64], dual_drive_skew_bound: [0.0; 64],
            link_loss_timeout_ns: 2_000_000_000,
            input_count: 0, input_actuator: [0; 64], input_active_high: 0,
            channel_count: 0, channel_id: [0; 1536], channel_kind: [0; 32],
            safe_digital: [0; 32], safe_analog: [0.0; 32],
            safe_argument: [0.0; 32], safe_command: [0; 1536], channel_stop_policy: [0; 32],
        };
        let mut session = vec![0; SessionBegin6::SIZE];
        begin.encode(&mut session).unwrap();
        assert!(send::<SessionBegin6>(&mut device, 1, &session));
        let (_, payload) = decode_frame6(device.outbox.front().unwrap()).unwrap();
        let ack = SessionAck6::decode(payload).unwrap();
        assert_eq!(ack.status, 1);
        assert_eq!(ack.actuator_count, 1);
        assert_eq!(device.active_count, 1);
        let queue = QueueBegin6 {
            queue_revision: 1,
            replace_after_ticks: 50_000,
            expected_position: [0.0; 64],
            expected_velocity: [0.0; 64],
            actuator_count: 1,
        };
        let mut body = vec![0; QueueBegin6::SIZE];
        queue.encode(&mut body).unwrap();
        assert!(send::<QueueBegin6>(&mut device, 5, &body));
        let header = Segment6Header {
            queue_revision: 1,
            plan_id: 8,
            t0_ticks: 50_000,
            duration_ticks: 1_000_000,
            degree: 1,
            actuator_count: 1,
            ends_at_rest: 1,
            purpose: 0,
        };
        let row = Segment6Coefficients {
            actuator: 0,
            c0: 0.0,
            c1: 0.5,
            c2: 0.0,
            c3: 0.0,
            c4: 0.0,
            c5: 0.0,
        };
        let mut segment = vec![0; Segment6Header::SIZE + Segment6Coefficients::SIZE];
        header.encode(&mut segment[..Segment6Header::SIZE]).unwrap();
        row.encode(&mut segment[Segment6Header::SIZE..]).unwrap();
        assert!(send::<Segment6Header>(&mut device, 6, &segment));
        let mut commit = [0; Commit6::SIZE];
        Commit6 {
            through_ticks: 1_050_000,
        }
        .encode(&mut commit)
        .unwrap();
        assert!(send::<Commit6>(&mut device, 7, &commit));
        assert!(device.advance(500_000_000));
        assert!(
            (device.board.step_counts()[0] - 250).abs() <= 1,
            "steps={} path={}",
            device.board.step_counts()[0],
            device.core.as_ref().unwrap().path_clock()
        );
        assert!(!device.core.as_ref().unwrap().underflow());
        assert_eq!(device.board.step_counts()[1..3], [1, 0]);
    }

    fn begin_for(expected: [u8; 16]) -> SessionBegin6 {
        SessionBegin6 {
            session: 9,
            protocol_version: PROTOCOL_VERSION,
            expected_controller: expected,
            actuator_count: 1,
            max_degree: 5,
            step_tick_hz: 40_000,
            max_acceleration: 10.0,
            actuator_max_acceleration: [10.0; 64],
            steps_per_unit: [1_000.0; 64], max_rate: [0.0; 64],
            direction_setup_ticks: [0; 64], actuator_joint: [0; 64],
            actuator_ratio: [1.0; 64], dual_drive_skew_bound: [0.0; 64],
            link_loss_timeout_ns: 2_000_000_000,
            input_count: 0, input_actuator: [0; 64], input_active_high: 0,
            channel_count: 0, channel_id: [0; 1536], channel_kind: [0; 32],
            safe_digital: [0; 32], safe_analog: [0.0; 32],
            safe_argument: [0.0; 32], safe_command: [0; 1536], channel_stop_policy: [0; 32],
        }
    }

    fn ack_to(device: &mut VirtualDevice, begin: &SessionBegin6) -> (SessionAck6, Vec<u8>) {
        let mut payload = vec![0; SessionBegin6::SIZE];
        begin.encode(&mut payload).unwrap();
        device.outbox.clear();
        assert!(send::<SessionBegin6>(device, 1, &payload));
        let frame = device.outbox.pop_front().unwrap();
        let (kind, body) = decode_frame6(&frame).unwrap();
        assert_eq!(kind, 2);
        (SessionAck6::decode(body).unwrap(), payload)
    }

    #[test]
    fn session_names_its_controller_and_acknowledges_the_digest() {
        let controller = [7; 16];
        let mut scale = [1.0; ACTUATORS];
        scale[0] = 1_000.0;
        let mut device =
            VirtualDevice::new(1_000_000, 40_000, 50_000, 0, 1, scale, controller, 1).unwrap();
        // A configuration for another board, and one that asks the board to identify itself, are
        // refused, and the refusal still says who the board is.
        for expected in [[8; 16], [0; 16]] {
            let (ack, _) = ack_to(&mut device, &begin_for(expected));
            assert_eq!(ack.status, 0);
            assert_eq!(ack.controller, controller);
            assert!(device.core.is_none());
        }
        let (ack, payload) = ack_to(&mut device, &begin_for(controller));
        assert_eq!(ack.status, 1);
        assert_eq!(ack.controller, controller);
        assert_eq!(ack.config_digest, config_digest6(&payload));
        // The session id is not part of the digest; any other field is.
        let mut other = begin_for(controller);
        other.session = 10;
        let (again, _) = ack_to(&mut device, &other);
        assert_eq!(again.config_digest, ack.config_digest);
        other.max_acceleration = 11.0;
        let (changed, _) = ack_to(&mut device, &other);
        assert_ne!(changed.config_digest, ack.config_digest);
    }
    fn welding_device(grounded: bool) -> VirtualDevice {
        welding_device_with_duration(grounded, 1_000_000)
    }
    fn welding_device_with_duration(grounded: bool, duration_ticks: u64) -> VirtualDevice {
        let mut d = VirtualDevice::new(1_000_000, 40_000, 0, 0, 1,
            [1_000.0; ACTUATORS], [7; 16], 1).unwrap();
        assert!(d.configure_welder(welder::WelderConfig { sensor_slot: 0, arc_channel: 0, wire_channel: 1,
            voltage_channel: 2, ignition_seconds: 0.002, no_arc_seconds: 0.02, efficiency: 0.9 }));
        assert!(d.set_welder_grounded(grounded));
        let mut begin = begin_for([7; 16]);
        begin.channel_count = 3; begin.channel_kind[0] = 1; begin.channel_kind[1] = 2; begin.channel_kind[2] = 2;
        for (i, name) in [b"arc".as_slice(), b"wire".as_slice(), b"voltage".as_slice()].iter().enumerate() {
            begin.channel_id[i * 48..i * 48 + name.len()].copy_from_slice(name);
        }
        begin.channel_stop_policy[2] = 1; begin.link_loss_timeout_ns = 100_000_000;
        assert_eq!(ack_to(&mut d, &begin).0.status, 1);
        let queue = QueueBegin6 { queue_revision: 1, replace_after_ticks: 0,
            expected_position: [0.0; 64], expected_velocity: [0.0; 64], actuator_count: 1 };
        let mut q = [0; QueueBegin6::SIZE]; queue.encode(&mut q).unwrap();
        assert!(send::<QueueBegin6>(&mut d, 5, &q));
        let header = Segment6Header { queue_revision: 1, plan_id: 1, t0_ticks: 0,
            duration_ticks, degree: 0, actuator_count: 1, ends_at_rest: 1, reserved: 0 };
        let row = Segment6Coefficients { actuator: 0, c0: 0.0, c1: 0.0, c2: 0.0,
            c3: 0.0, c4: 0.0, c5: 0.0 };
        let mut segment = vec![0; Segment6Header::SIZE + Segment6Coefficients::SIZE];
        header.encode(&mut segment[..Segment6Header::SIZE]).unwrap();
        row.encode(&mut segment[Segment6Header::SIZE..]).unwrap();
        assert!(send::<Segment6Header>(&mut d, 6, &segment));
        for (channel, kind, digital, analog) in [(2, 2, 0, 24.0), (1, 2, 0, 8.0), (0, 1, 1, 0.0)] {
            let event = Event6 { queue_revision: 1, plan_id: 1, path_ticks: 0, channel,
                kind, hold_policy: 1, digital, analog, argument: 0.0, command: [0; 48] };
            let mut body = [0; Event6::SIZE]; event.encode(&mut body).unwrap();
            assert!(send::<Event6>(&mut d, 16, &body));
        }
        let mut body = [0; Commit6::SIZE];
        Commit6 { through_ticks: duration_ticks }.encode(&mut body).unwrap();
        assert!(send::<Commit6>(&mut d, 7, &body));
        d
    }

    fn assert_weld_off(d: &VirtualDevice) {
        assert_eq!(d.board.digital(0), Some(false));
        assert_eq!(d.board.analog(1), Some(0.0));
        assert_eq!(d.weld_values().unwrap()[0], 0.0);
        assert_eq!(d.weld_values().unwrap()[5], 0.0);
    }

    #[test]
    fn welding_stop_paths_safe_device_outputs() {
        for kind in [10, 11, 12] {
            let mut d = welding_device(true); assert!(d.advance(10_000_000));
            assert_eq!(d.weld_values().unwrap()[0], 1.0);
            assert_eq!(d.weld_values().unwrap()[1], 240.0);
            assert!(send::<SessionBegin6>(&mut d, kind, &[]));
            assert!(d.advance(11_000_000)); assert_weld_off(&d);
        }
        let mut d = welding_device(true); assert!(d.advance(10_000_000));
        assert_eq!(d.weld_values().unwrap()[0], 1.0);
        assert!(d.advance(150_000_000)); assert_weld_off(&d);
        assert_eq!(d.core.as_ref().unwrap().stop_reason(), Some(StopReason::LinkLost));
    }

    #[test]
    fn welding_fault_safes_channels_on_device() {
        let mut d = welding_device(false); assert!(d.advance(30_000_000)); assert_weld_off(&d);
        assert_eq!(d.weld_values().unwrap()[4], 1.0);
        assert_eq!(d.core.as_ref().unwrap().stop_reason(), Some(StopReason::ProcessFault));
        let mut d = welding_device(true); assert!(d.advance(10_000_000));
        assert!(d.set_welder_grounded(false)); assert!(d.advance(11_000_000)); assert_weld_off(&d);
        assert_eq!(d.weld_values().unwrap()[4], 2.0);
    }

    #[test]
    fn welding_at_rest_still_requires_the_device_link_lease() {
        let mut d = welding_device_with_duration(true, 50_000);
        assert!(d.advance(60_000_000));
        assert_eq!(d.weld_values().unwrap()[0], 1.0);
        assert_eq!(d.core.as_ref().unwrap().stop_reason(), None);
        assert!(d.advance(150_000_000)); assert_weld_off(&d);
        assert_eq!(d.core.as_ref().unwrap().stop_reason(), Some(StopReason::LinkLost));
    }

    #[test]
    fn welding_rejects_unsafe_channel_policy() {
        let mut d = welding_device(true);
        let mut begin = begin_for([7; 16]); begin.channel_count = 3;
        begin.channel_kind[0] = 1; begin.channel_kind[1] = 2; begin.channel_kind[2] = 2; begin.channel_stop_policy[0] = 1;
        for (i, name) in [b"arc".as_slice(), b"wire".as_slice(), b"voltage".as_slice()].iter().enumerate() {
            begin.channel_id[i * 48..i * 48 + name.len()].copy_from_slice(name);
        }
        assert_eq!(ack_to(&mut d, &begin).0.status, 0);
    }

    #[test]
    fn welding_feedback_travels_in_sensor_frames() {
        let mut d = welding_device(true); d.outbox.clear(); assert!(d.advance(10_000_000));
        let frame = d.outbox.iter().find(|f| decode_frame6(f).unwrap().0 == 21).unwrap();
        let (_, body) = decode_frame6(frame).unwrap();
        let header = Sensor6Header::decode(&body[..Sensor6Header::SIZE]).unwrap();
        assert_eq!(header.session, 9); assert_eq!(header.slot, 0); assert_eq!(header.value_count, 6);
        assert_eq!(header.timestamp_ticks, 10_000); assert!(header.sequence > 1);
        for (i, expected) in d.weld_values().unwrap().iter().enumerate() {
            let start = Sensor6Header::SIZE + i * Sensor6Value::SIZE;
            assert_eq!(Sensor6Value::decode(&body[start..start + Sensor6Value::SIZE]).unwrap().value, *expected);
        }
    }

    #[test]
    fn rejects_more_actuators_than_the_board_has() {
        let controller = [7; 16];
        let mut device = VirtualDevice::new(1_000_000, 40_000, 0, 0, 2,
            [1_000.0; ACTUATORS], controller, 1).unwrap();
        let mut begin = begin_for(controller);
        begin.actuator_count = 3;
        assert_eq!(ack_to(&mut device, &begin).0.status, 0);
        assert_eq!(device.active_count, 0);
        assert!(device.core.is_none());
    }

    #[test]
    fn paired_homing_rebase_hands_off_a_stationary_wire_queue() {
        let controller = [7; 16];
        let mut device = VirtualDevice::new(1_000_000, 40_000, 0, 0, 2,
            [1000.0; ACTUATORS], controller, 1).unwrap();
        let mut begin = begin_for(controller);
        begin.actuator_count = 2;
        begin.dual_drive_skew_bound[0] = 0.01;
        begin.dual_drive_skew_bound[1] = 0.01;
        assert_eq!(ack_to(&mut device, &begin).0.status, 1);
        let scope = |device: &mut VirtualDevice, sequence, action| {
            let record = HomingScope6 { session: 9, sequence, scope: 1,
                action, first: 0, second: 1, skew_bound: 0.02 };
            let mut body = [0; HomingScope6::SIZE];
            record.encode(&mut body).unwrap();
            assert!(send::<HomingScope6>(device, 17, &body));
        };
        let side = |device: &mut VirtualDevice, sequence, hold| {
            let record = HomingSide6 { session: 9, sequence, scope: 1, actuator: 0, hold };
            let mut body = [0; HomingSide6::SIZE];
            record.encode(&mut body).unwrap();
            assert!(send::<HomingSide6>(device, 18, &body));
        };
        let queue = |device: &mut VirtualDevice, revision, position, speed, duration, purpose| {
            let t0 = device.board.now_ticks();
            let mut expected = [0.0; ACTUATORS]; expected[..2].fill(position);
            let record = QueueBegin6 { queue_revision: revision, replace_after_ticks: t0,
                expected_position: expected, expected_velocity: [0.0; ACTUATORS], actuator_count: 2 };
            let mut body = [0; QueueBegin6::SIZE]; record.encode(&mut body).unwrap();
            assert!(send::<QueueBegin6>(device, 5, &body));
            let header = Segment6Header { queue_revision: revision, plan_id: revision,
                t0_ticks: t0, duration_ticks: duration, degree: 1,
                actuator_count: 2, ends_at_rest: 1, purpose };
            let mut segment = vec![0; Segment6Header::SIZE + 2 * Segment6Coefficients::SIZE];
            header.encode(&mut segment[..Segment6Header::SIZE]).unwrap();
            for actuator in 0..2 {
                let row = Segment6Coefficients { actuator, c0: position, c1: speed,
                    c2: 0.0, c3: 0.0, c4: 0.0, c5: 0.0 };
                let start = Segment6Header::SIZE + actuator as usize * Segment6Coefficients::SIZE;
                row.encode(&mut segment[start..start + Segment6Coefficients::SIZE]).unwrap();
            }
            assert!(send::<Segment6Header>(device, 6, &segment));
            let mut commit = [0; Commit6::SIZE];
            Commit6 { through_ticks: t0 + duration }.encode(&mut commit).unwrap();
            assert!(send::<Commit6>(device, 7, &commit));
        };
        scope(&mut device, 1, 0);
        assert_eq!(device.homing_scope, Some(1));
        side(&mut device, 2, 1);
        queue(&mut device, 1, 0.0, 0.01, 400_000, 2);
        assert!(device.advance(400_000_000));
        assert_eq!((device.board.step_count(0), device.board.step_count(1)), (0, 4));
        scope(&mut device, 3, 2);
        assert!(device.advance(401_000_000));
        side(&mut device, 4, 0);
        // The normalized leader is halfway between the separate physical sides.
        queue(&mut device, 2, 0.002, 0.0, 1_000_000, 2);
        assert!(device.advance(411_000_000));
        assert_eq!((device.board.step_count(0), device.board.step_count(1)), (0, 4));
        scope(&mut device, 5, 2);
        assert!(device.advance(412_000_000));
        let batch = HomingCounterBatch6 { session: 9, sequence: 6, scope: 1,
            first: 0, second: 1, first_delta: 0.0, second_delta: 0.004 };
        let mut body = [0; HomingCounterBatch6::SIZE]; batch.encode(&mut body).unwrap();
        assert!(send::<HomingCounterBatch6>(&mut device, 20, &body));
        assert_eq!(device.steps.counter_position(&device.board, 1), Some(0.0));
        scope(&mut device, 7, 1);
        assert_eq!(device.homing_scope, None);
        queue(&mut device, 3, 0.0, 0.0, 1_000_000, 0);
        assert!(device.advance(425_000_000));
        assert_eq!((device.board.step_count(0), device.board.step_count(1)), (0, 4));
        assert_eq!(device.core.as_ref().unwrap().stop_reason(), None);
        for sequence in 1..=7 {
            let ack = device.outbox.iter().filter_map(|frame| {
                let (kind, body) = decode_frame6(frame).ok()?;
                if kind == 19 { HomingControlAck6::decode(body).ok() } else { None }
            }).find(|ack| ack.sequence == sequence).unwrap();
            assert_eq!(ack.accepted, 1);
        }
    }

}
