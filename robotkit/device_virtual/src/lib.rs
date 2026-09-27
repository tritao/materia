//! In-process RKD6 device. The C ABI only moves complete validated frames.
use robotkit_device_protocol::device_wire6::*;
use robotkit_device_protocol::frame6::{decode_frame6, encode_frame6, MAX_FRAME_SIZE};
use robotkit_device_protocol::{
    Board, Output, ScheduledCore, ScheduledSegment, SkewGroup, StepGenerator, StopReason, VirtualBoard,
};
use std::collections::VecDeque;

const ACTUATORS: usize = 64;
const CHANNELS: usize = 32;
const CAPACITY: usize = 128;

pub struct VirtualDevice {
    board: VirtualBoard<ACTUATORS, CHANNELS>,
    core: Option<ScheduledCore<ACTUATORS, CAPACITY>>,
    steps: StepGenerator<ACTUATORS>,
    fingerprint: [u8; 16],
    count: usize,
    session: u64,
    step_tick_hz: u32,
    host_ns: u64,
    outbox: VecDeque<Vec<u8>>,
}

impl VirtualDevice {
    fn new(
        tick_hz: u64,
        step_tick_hz: u32,
        offset_ticks: u64,
        drift_ppm: i32,
        count: usize,
        steps_per_unit: [f64; ACTUATORS],
        fingerprint: [u8; 16],
    ) -> Option<Self> {
        if tick_hz == 0
            || step_tick_hz == 0
            || count == 0
            || count > ACTUATORS
            || drift_ppm <= -1_000_000
            || steps_per_unit.iter().any(|v| !v.is_finite() || *v <= 0.0)
        {
            return None;
        }
        Some(Self {
            steps: StepGenerator::new(steps_per_unit, [0; ACTUATORS],
                [0.0; ACTUATORS], tick_hz)?,
            board: VirtualBoard::new(tick_hz, offset_ticks, drift_ppm, steps_per_unit),
            core: None,
            fingerprint,
            count,
            session: 0,
            step_tick_hz,
            host_ns: 0,
            outbox: VecDeque::new(),
        })
    }

    fn emit(&mut self, kind: u8, payload: &[u8]) {
        let mut frame = vec![0; MAX_FRAME_SIZE];
        if let Ok(size) = encode_frame6(kind, payload, &mut frame) {
            frame.truncate(size);
            self.outbox.push_back(frame);
        }
    }

    fn feed(&mut self, frame: &[u8]) -> bool {
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
                    device_fingerprint: self.fingerprint,
                    status: 0,
                    device_tick_hz: self.board.tick_hz(),
                    segment_capacity: CAPACITY as u16,
                    event_capacity: 0,
                    step_tick_hz: self.step_tick_hz,
                    max_degree: 5,
                    actuator_count: self.count as u8,
                };
                if begin.model_fingerprint == self.fingerprint
                    && begin.actuator_count as usize == self.count
                    && begin.step_tick_hz == self.step_tick_hz
                    && begin.session != 0
                    && (0..self.count).all(|i| {
                        let physical = self.board.steps_per_unit()[i];
                        ((begin.steps_per_unit[i] as f64 - physical) / physical).abs() < 1e-6
                    })
                {
                    let mut limits = [1.0f32; ACTUATORS];
                    for (i, limit) in limits.iter_mut().enumerate().take(self.count) {
                        *limit = begin.actuator_max_acceleration[i];
                    }
                    let mut steps_per_unit = [1.0; ACTUATORS];
                    let mut max_rate = [0.0; ACTUATORS];
                    let mut setup = [0; ACTUATORS];
                    for i in 0..self.count {
                        steps_per_unit[i] = begin.steps_per_unit[i] as f64;
                        max_rate[i] = begin.max_rate[i] as f64;
                        setup[i] = begin.direction_setup_ticks[i] as u64;
                    }
                    let Some(mut generator) = StepGenerator::new(
                        steps_per_unit, setup, max_rate, self.board.tick_hz()) else {
                        return false;
                    };
                    for i in 0..self.count {
                        for j in i + 1..self.count {
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
                    self.steps = generator;
                    self.core = Some(core);
                    self.session = begin.session;
                    ack.status = 1;
                }
                let mut bytes = [0; SessionAck6::SIZE];
                ack.encode(&mut bytes).unwrap();
                self.emit(2, &bytes);
                self.publish_state();
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
                if begin.actuator_count as usize != self.count {
                    return false;
                }
                self.core
                    .as_mut()
                    .unwrap()
                    .queue_begin_with_state(
                        begin.queue_revision,
                        begin.replace_after_ticks,
                        begin.expected_position,
                        begin.expected_velocity,
                    )
                    .is_ok()
            }
            6 => {
                let Ok(header) = Segment6Header::decode(&payload[..Segment6Header::SIZE]) else {
                    return false;
                };
                if header.actuator_count as usize != self.count
                    || header.queue_revision != self.core.as_ref().unwrap().revision()
                {
                    return false;
                }
                let mut coefficients = [[0.0f32; 6]; ACTUATORS];
                for (i, slot) in coefficients.iter_mut().enumerate().take(self.count) {
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
                let Ok(segment) = ScheduledSegment::new(
                    header.plan_id,
                    header.t0_ticks,
                    header.duration_ticks,
                    header.degree,
                    coefficients,
                    header.ends_at_rest != 0,
                ) else {
                    return false;
                };
                self.core.as_mut().unwrap().push_segment(segment).is_ok()
            }
            7 => {
                let Ok(commit) = Commit6::decode(payload) else {
                    return false;
                };
                self.core
                    .as_mut()
                    .unwrap()
                    .commit(commit.through_ticks)
                    .is_ok()
            }
            8 => {
                self.core.as_mut().unwrap().hold();
                true
            }
            9 => {
                self.core.as_mut().unwrap().resume();
                true
            }
            10 => {
                self.core.as_mut().unwrap().abort();
                true
            }
            11 => {
                self.core.as_mut().unwrap().stop(StopReason::Stop);
                true
            }
            12 => {
                self.core.as_mut().unwrap().emergency_stop(&mut self.board);
                true
            }
            13 => false,
            _ => false,
        }
    }

    fn advance(&mut self, host_ns: u64) -> bool {
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
                core.tick(&mut self.board);
                let targets = self.board.position_targets();
                if self.steps.tick(&mut self.board, targets).is_err() {
                    core.stop(StopReason::DualDriveSkew);
                }
            }
        }
        self.publish_state();
        true
    }

    fn publish_state(&mut self) {
        let Some(core) = self.core.as_ref() else {
            return;
        };
        let fault = match core.stop_reason() {
            None => 0,
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
            remaining_segments: core.remaining_capacity() as u16,
            remaining_events: 0,
            underflow: core.underflow() as u8,
            fault,
        };
        let mut bytes = [0; QueueStatus6::SIZE];
        status.encode(&mut bytes).unwrap();
        self.emit(14, &bytes);
        let core = self.core.as_ref().unwrap();
        let header = State6Header {
            session: self.session,
            timestamp_ticks: self.board.now_ticks(),
            accepted_sequence: 0,
            safety: if core.stop_reason().is_some() { 3 } else { 0 },
            fault,
            actuator_count: self.count as u8,
            reserved: 0,
            path_clock_ticks: core.path_clock(),
        };
        let positions = self.board.actuator_positions();
        let velocity = core.velocities();
        let counts = self.board.step_counts();
        let mut body = vec![0; State6Header::SIZE + self.count * ActuatorState6::SIZE];
        header.encode(&mut body[..State6Header::SIZE]).unwrap();
        for i in 0..self.count {
            let row = ActuatorState6 {
                position: positions[i] as f32,
                velocity: velocity[i],
                effort: 0.0,
                step_count: counts[i],
            };
            let start = State6Header::SIZE + i * ActuatorState6::SIZE;
            row.encode(&mut body[start..start + ActuatorState6::SIZE])
                .unwrap();
        }
        self.emit(15, &body);
    }
}

#[no_mangle]
pub unsafe extern "C" fn rkd_virtual_create(
    tick_hz: u64,
    step_tick_hz: u32,
    offset_ticks: u64,
    drift_ppm: i32,
    actuator_count: u32,
    steps_per_unit: *const f64,
    fingerprint: *const u8,
) -> *mut VirtualDevice {
    if steps_per_unit.is_null() || fingerprint.is_null() || actuator_count as usize > ACTUATORS {
        return std::ptr::null_mut();
    }
    let mut scale = [1.0; ACTUATORS];
    scale[..actuator_count as usize].copy_from_slice(std::slice::from_raw_parts(
        steps_per_unit,
        actuator_count as usize,
    ));
    let mut fp = [0; 16];
    fp.copy_from_slice(std::slice::from_raw_parts(fingerprint, 16));
    VirtualDevice::new(
        tick_hz,
        step_tick_hz,
        offset_ticks,
        drift_ppm,
        actuator_count as usize,
        scale,
        fp,
    )
    .map_or(std::ptr::null_mut(), |v| Box::into_raw(Box::new(v)))
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
            *values.add(i) = device.board.analog(i).unwrap_or(0.0);
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
        .records()
        .iter()
        .filter(|row| matches!(row.output, Output::Step(_, _)))
        .count();
    if records.is_null() || capacity < count {
        return count;
    }
    let mut index = 0;
    for row in device.board.records() {
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
        let fingerprint = [7; 16];
        let mut scale = [1.0; ACTUATORS];
        scale[0] = 1_000.0;
        let mut device =
            VirtualDevice::new(1_000_000, 40_000, 50_000, 0, 1, scale, fingerprint).unwrap();
        let begin = SessionBegin6 {
            session: 9,
            protocol_version: PROTOCOL_VERSION,
            model_fingerprint: fingerprint,
            actuator_count: 1,
            max_degree: 5,
            step_tick_hz: 40_000,
            max_acceleration: 10.0,
            actuator_max_acceleration: [10.0; 64],
            steps_per_unit: [1_000.0; 64], max_rate: [0.0; 64],
            direction_setup_ticks: [0; 64], actuator_joint: [0; 64],
            actuator_ratio: [1.0; 64], dual_drive_skew_bound: [0.0; 64],
            link_loss_timeout_ns: 2_000_000_000,
        };
        let mut session = vec![0; SessionBegin6::SIZE];
        begin.encode(&mut session).unwrap();
        assert!(send::<SessionBegin6>(&mut device, 1, &session));
        assert_eq!(decode_frame6(device.outbox.front().unwrap()).unwrap().0, 2);
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
            reserved: 0,
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
    }
}
