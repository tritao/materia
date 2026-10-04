#![no_std]
#![no_main]

use cortex_m::peripheral::DWT;
use cortex_m_rt::entry;
use embedded_hal_old::serial::{Read, Write};
use nb::Error::WouldBlock;
use panic_halt as _;
use robotkit_device_protocol::{Board, ScheduledCore, ScheduledSegment, StopReason};
use robotkit_device_protocol::config_digest::{config_digest6, controller_matches};
use robotkit_device_protocol::device_wire6::*;
use robotkit_device_protocol::frame6::{decode_frame6, encode_frame6,
    slide_to_frame_marker, MAX_FRAME_SIZE};
use stm32g4xx_hal::{prelude::*, pwr::PwrExt, rcc, serial::FullConfig, stm32};

/// Base of the STM32G4's factory-programmed 96-bit unique device ID (RM0440, "Unique device ID").
const UID_BASE: *const u32 = 0x1FFF_7590 as *const u32;

/// The board's controller id: its 12 unique-ID bytes, zero-padded to the 16 the protocol carries.
fn controller_id() -> [u8; 16] {
    let mut id = [0u8; 16];
    for word in 0..3 {
        // SAFETY: the unique-ID words are read-only memory present on every STM32G4.
        let value = unsafe { core::ptr::read_volatile(UID_BASE.add(word)) };
        id[word * 4..word * 4 + 4].copy_from_slice(&value.to_le_bytes());
    }
    id
}

const JOINTS: usize = 2;
const ACTUATORS: usize = 64;
const CAPACITY: usize = 8;
const TICK_HZ: u64 = 1_000_000;
const STATE_PERIOD_TICKS: u64 = 25_000;
const HSI_CYCLES_PER_MICROSECOND: u64 = 16;

struct StubBoard {
    ticks: u64,
    position: [f32; ACTUATORS],
    velocity: [f32; ACTUATORS],
}
impl StubBoard {
    fn new() -> Self { Self { ticks: 0, position: [0.0; ACTUATORS], velocity: [0.0; ACTUATORS] } }
}
impl Board for StubBoard {
    fn now_ticks(&self) -> u64 { self.ticks }
    fn tick_hz(&self) -> u64 { TICK_HZ }
    fn position_target(&mut self, i: usize, value: f32) { self.position[i] = value; }
    fn velocity_target(&mut self, i: usize, value: f32) { self.velocity[i] = value; }
    fn step_pulse(&mut self, _i: usize, _forward: bool) {}
    fn step_count(&self, _i: usize) -> i64 { 0 }
    fn set_digital(&mut self, _i: usize, _value: bool) {}
    fn set_analog(&mut self, _i: usize, _value: f32) {}
    fn stop_all(&mut self) { self.velocity.fill(0.0); }
}

fn send<T: Write<u8>>(tx: &mut T, kind: u8, payload: &[u8], frame: &mut [u8; MAX_FRAME_SIZE]) {
    if let Ok(size) = encode_frame6(kind, payload, frame) {
        for &byte in &frame[..size] { nb::block!(tx.write(byte)).ok(); }
    }
}

fn publish<T: Write<u8>>(tx: &mut T, core: &ScheduledCore<ACTUATORS, CAPACITY>,
    board: &StubBoard, session: u64, active_count: usize, received_bytes: u64, frame: &mut [u8; MAX_FRAME_SIZE]) {
    let fault = match core.stop_reason() {
        None => 0, Some(StopReason::Underflow) => 2,
        Some(StopReason::LinkLost) => 3, Some(StopReason::DualDriveSkew) => 4,
        Some(_) => 1,
    };
    let status = QueueStatus6 {
        queue_revision: core.revision(), committed_until_ticks: core.committed_until(),
        executing_plan_id: core.executing_plan_id(), executing_segment: core.executing_segment(),
        path_clock_ticks: core.path_clock(), rate: core.rate(),
        remaining_segments: core.remaining_capacity() as u16, remaining_events: 0,
        underflow: core.underflow() as u8, fault,
        received_until_ticks: core.received_until(), received_bytes,
    };
    let mut body = [0u8; State6Header::SIZE + JOINTS * ActuatorState6::SIZE];
    status.encode(&mut body).ok();
    send(tx, 14, &body[..QueueStatus6::SIZE], frame);
    let header = State6Header {
        session, timestamp_ticks: board.ticks, accepted_sequence: 0,
        safety: if fault == 0 { 0 } else { 3 }, fault, actuator_count: active_count as u8,
        reserved: 0, path_clock_ticks: core.path_clock(),
    };
    header.encode(&mut body[..State6Header::SIZE]).ok();
    for i in 0..active_count {
        let row = ActuatorState6 { position: board.position[i], velocity: board.velocity[i],
            effort: 0.0, step_count: 0 };
        let start = State6Header::SIZE + i * ActuatorState6::SIZE;
        row.encode(&mut body[start..start + ActuatorState6::SIZE]).ok();
    }
    send(tx, 15, &body[..State6Header::SIZE + active_count * ActuatorState6::SIZE], frame);
}

fn handle<T: Write<u8>>(input: &[u8], board: &mut StubBoard,
    core: &mut Option<ScheduledCore<ACTUATORS, CAPACITY>>, session: &mut u64, active_count: &mut usize,
    received_bytes: &mut u64, tx: &mut T, frame: &mut [u8; MAX_FRAME_SIZE]) {
    let Ok((kind, payload)) = decode_frame6(input) else { return; };
    if kind != 1 { if let Some(core) = core.as_mut() { core.note_host_frame(board.ticks); } }
    match kind {
        1 => {
            let Ok(begin) = SessionBegin6::decode(payload) else { return; };
            // The board reports its own id even when it refuses, so `robotd identify` can read it.
            let own = controller_id();
            let accepted = controller_matches(&begin.expected_controller, &own) &&
                begin.actuator_count > 0 && begin.actuator_count as usize <= JOINTS && begin.session != 0 &&
                begin.step_tick_hz == 40_000 && begin.channel_count == 0 &&
                begin.actuator_max_acceleration[..begin.actuator_count as usize].iter().all(|v| v.is_finite() && *v > 0.0);
            let mut ack = SessionAck6 {
                session: begin.session, protocol_version: PROTOCOL_VERSION,
                controller: own, config_digest: config_digest6(payload), status: accepted as u8,
                device_tick_hz: TICK_HZ, segment_capacity: CAPACITY as u16,
                event_capacity: 0, step_tick_hz: 40_000, max_degree: 1,
                actuator_count: if accepted { begin.actuator_count } else { JOINTS as u8 }, profile: 2,
            };
            if accepted {
                let mut limits = [1.0; ACTUATORS];
                limits[..begin.actuator_count as usize].copy_from_slice(&begin.actuator_max_acceleration[..begin.actuator_count as usize]);
                let link_ticks = begin.link_loss_timeout_ns / 1_000;
                let mut next = ScheduledCore::new(TICK_HZ, limits,
                    [-1.0e12; ACTUATORS], [1.0e12; ACTUATORS], link_ticks.max(1));
                next.initialize_clock(board.ticks);
                *core = Some(next);
                *session = begin.session;
                *active_count = begin.actuator_count as usize;
                // The count starts after the frame that begins the session, as the host's does.
                *received_bytes = 0;
            } else { ack.status = 0; }
            let mut bytes = [0u8; SessionAck6::SIZE];
            ack.encode(&mut bytes).ok();
            send(tx, 2, &bytes, frame);
            if let Some(core) = core.as_ref() { publish(tx, core, board, *session, *active_count, *received_bytes, frame); }
        }
        3 => {
            let Ok(request) = TimeSyncRequest::decode(payload) else { return; };
            let reply = TimeSyncReply { host_send_ns: request.host_send_ns,
                device_rx_ticks: board.ticks, device_tx_ticks: board.ticks };
            let mut bytes = [0u8; TimeSyncReply::SIZE];
            reply.encode(&mut bytes).ok();
            send(tx, 4, &bytes, frame);
        }
        5 => {
            let Ok(begin) = QueueBegin6::decode(payload) else { return; };
            if begin.actuator_count as usize != *active_count { return; }
            if let Some(core) = core.as_mut() {
                core.queue_begin_with_state(begin.queue_revision, begin.replace_after_ticks,
                    begin.expected_position, begin.expected_velocity).ok();
            }
        }
        6 => {
            let Ok(header) = Segment6Header::decode(&payload[..Segment6Header::SIZE]) else { return; };
            if header.degree > 1 || header.actuator_count as usize != *active_count { return; }
            let mut coefficients = [[0.0; 6]; ACTUATORS];
            for (i, slot) in coefficients.iter_mut().enumerate().take(*active_count) {
                let offset = Segment6Header::SIZE + i * Segment6Coefficients::SIZE;
                let Ok(row) = Segment6Coefficients::decode(&payload[offset..offset + Segment6Coefficients::SIZE]) else { return; };
                if row.actuator as usize != i { return; }
                *slot = [row.c0, row.c1, 0.0, 0.0, 0.0, 0.0];
            }
            let Ok(segment) = ScheduledSegment::new(header.plan_id, header.t0_ticks,
                header.duration_ticks, header.degree, coefficients, header.ends_at_rest != 0) else { return; };
            if let Some(core) = core.as_mut() {
                core.push_segment_for_revision(header.queue_revision, segment).ok();
            }
        }
        7 => {
            let Ok(commit) = Commit6::decode(payload) else { return; };
            if let Some(core) = core.as_mut() { core.commit(commit.through_ticks).ok(); }
        }
        8 => { if let Some(core) = core.as_mut() { core.hold(); } }
        9 => { if let Some(core) = core.as_mut() { core.resume(); } }
        10 => { if let Some(core) = core.as_mut() { core.abort(); } }
        11 => { if let Some(core) = core.as_mut() { core.stop(StopReason::Stop); } }
        12 => { if let Some(core) = core.as_mut() { core.emergency_stop(board); } }
        _ => {}
    }
}

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let pwr = dp.PWR.constrain().freeze();
    let mut rcc = dp.RCC.freeze(rcc::Config::hsi(), pwr);
    let gpioc = dp.GPIOC.split(&mut rcc);
    let serial = dp.USART1.usart(gpioc.pc4.into_alternate(), gpioc.pc5.into_alternate(),
        FullConfig::default().baudrate(921_600.bps()), &mut rcc).unwrap();
    let (mut tx, mut rx) = serial.split();
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();
    let mut previous_cycles = DWT::cycle_count();
    let mut elapsed_cycles = 0u64;
    let mut board = StubBoard::new();
    let mut core: Option<ScheduledCore<ACTUATORS, CAPACITY>> = None;
    let mut session = 0u64;
    let mut active_count = 0usize;
    let mut received_bytes = 0u64;
    let mut last_state = 0u64;
    let mut input = [0u8; MAX_FRAME_SIZE];
    let mut input_len = 0usize;
    let mut output = [0u8; MAX_FRAME_SIZE];
    loop {
        let current = DWT::cycle_count();
        elapsed_cycles += current.wrapping_sub(previous_cycles) as u64;
        previous_cycles = current;
        board.ticks = elapsed_cycles / HSI_CYCLES_PER_MICROSECOND;
        if let Some(core) = core.as_mut() { core.tick(&mut board); }
        if board.ticks.saturating_sub(last_state) >= STATE_PERIOD_TICKS {
            if let Some(core) = core.as_ref() {
                publish(&mut tx, core, &board, session, active_count, received_bytes, &mut output);
            }
            last_state = board.ticks;
        }
        match rx.read() {
            Ok(byte) => {
                // Every byte off the line counts, valid or not: the host measures what is in flight by it.
                received_bytes += 1;
                if input_len == input.len() {
                    input.copy_within(1..input_len, 0);
                    input_len -= 1;
                }
                input[input_len] = byte;
                input_len += 1;
                slide_to_frame_marker(&mut input, &mut input_len);
                if input_len >= 8 {
                    let size = 8 + u16::from_le_bytes([input[6], input[7]]) as usize + 4;
                    if size > input.len() {
                        input.copy_within(1..input_len, 0);
                        input_len -= 1;
                        slide_to_frame_marker(&mut input, &mut input_len);
                        continue;
                    }
                    if input_len == size {
                        if decode_frame6(&input[..size]).is_ok() {
                            handle(&input[..size], &mut board, &mut core, &mut session, &mut active_count,
                                &mut received_bytes, &mut tx, &mut output);
                            input_len = 0;
                        } else {
                            input.copy_within(1..input_len, 0);
                            input_len -= 1;
                            slide_to_frame_marker(&mut input, &mut input_len);
                        }
                    }
                }
            }
            Err(WouldBlock) => {}
            Err(nb::Error::Other(_)) => {}
        }
    }
}
