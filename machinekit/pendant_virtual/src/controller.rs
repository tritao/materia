//! The controller side of a simulated pendant: the real cabinet-node operator service, with the host's part played here (an allowing
//! `OperatorState` and the profile's labels), so the pendant core shows what it would on a machine.

use std::collections::VecDeque;

use cabinet_node::OperatorService;
use operator_link::operator_frame;
use operator_link::operator_wire::{OperatorMessage, OperatorState, PendantConfig, PendantOutput};
use operator_link::profile::CompiledProfile;
use operator_link::{CONFIG_ID, OUTPUT_ID};
use pendant_core::PendantCore;

const HOST_PERIOD_MS: u32 = 100;
const OUTPUT_PERIOD_MS: u32 = 100;
const MAX_JOGS: usize = 256;

pub struct Controller {
    profile: *mut CompiledProfile,
    service: Option<OperatorService<'static>>,
    host_sequence: u16,
    next_host_ms: u32,
    next_output_ms: u32,
    /// The condition mask the controller builds from its own facts (see the profile's conditions).
    pub active: u32,
    /// The jogs the controller has forwarded to the host since they were last taken: the profile's axis index and the distance in
    /// thousandths of a unit, consecutive jogs of one axis added together.
    jogs: VecDeque<(u8, i32)>,
}

// SAFETY: the raw pointer is a uniquely owned heap allocation that only this controller and its service reach.
unsafe impl Send for Controller {}

fn due(now_ms: u32, at_ms: u32) -> bool {
    now_ms.wrapping_sub(at_ms) as i32 >= 0
}

fn frame_of(kind: OperatorMessage, payload: &[u8]) -> Vec<u8> {
    let mut frame = vec![0u8; operator_frame::MAX_FRAME_SIZE];
    let n = operator_frame::encode(kind as u8, payload, &mut frame).expect("a payload that fits a frame");
    frame.truncate(n);
    frame
}

impl Controller {
    /// `reasons` and `labels` are the compiled profile's refusal-text and label records. Fails when the profile or a record is invalid.
    pub fn new(now_ms: u32, profile: &[u8], reasons: &[Vec<u8>], labels: &[Vec<u8>]) -> Result<Self, ()> {
        let mut compiled = CompiledProfile::from_record(profile).map_err(|_| ())?;
        for reason in reasons {
            compiled.set_reason(reason).map_err(|_| ())?;
        }
        let profile = Box::into_raw(Box::new(compiled));
        // SAFETY: the profile is on the heap until `drop`, which drops the service that borrows it first.
        let service = OperatorService::new(unsafe { &*profile });
        let mut controller = Controller {
            profile,
            service: Some(service),
            host_sequence: 0,
            next_host_ms: now_ms,
            next_output_ms: now_ms.wrapping_add(OUTPUT_PERIOD_MS / 2),
            active: 0,
            jogs: VecDeque::new(),
        };
        for label in labels {
            if controller.service().on_operator_frame(&frame_of(OperatorMessage::Config, label), 0).is_err() {
                return Err(());
            }
        }
        Ok(controller)
    }

    fn service(&mut self) -> &mut OperatorService<'static> {
        self.service.as_mut().expect("the service lives as long as the controller")
    }

    /// Runs the controller up to `now_ms`: the host's allowing state, the pendant's heartbeat if it sent one, and the controller's
    /// own outputs when they are due.
    pub fn step(&mut self, now_ms: u32, heartbeat: Option<[u8; 8]>, core: &mut PendantCore) {
        let now_us = now_ms as u64 * 1000;
        let active = self.active;
        if due(now_ms, self.next_host_ms) {
            self.host_sequence = self.host_sequence.wrapping_add(1);
            let mut payload = [0u8; OperatorState::SIZE];
            if (OperatorState { sequence: self.host_sequence, jog_allowed: 1, reserved: 0 }).encode(&mut payload).is_ok() {
                let _ = self.service().on_operator_frame(&frame_of(OperatorMessage::State, &payload), now_us);
            }
            self.next_host_ms = now_ms.wrapping_add(HOST_PERIOD_MS);
        }
        if let Some(frame) = heartbeat {
            if let Ok(Some(event)) = self.service().on_pendant_frame(&frame, now_us, active) {
                if event.distance != 0 {
                    match self.jogs.back_mut() {
                        Some((axis, distance)) if *axis == event.axis => *distance = distance.saturating_add(event.distance),
                        _ => self.jogs.push_back((event.axis, event.distance)),
                    }
                    // A host that never takes them cannot grow this without bound.
                    while self.jogs.len() > MAX_JOGS {
                        self.jogs.pop_front();
                    }
                }
            }
        }
        if due(now_ms, self.next_output_ms) {
            let mut output = [0u8; PendantOutput::SIZE];
            if self.service().pendant_output(now_us, active).encode(&mut output).is_ok() {
                let _ = core.on_frame(now_ms, OUTPUT_ID, &output);
            }
            if let Some(config) = self.service().next_config() {
                let mut payload = [0u8; PendantConfig::SIZE];
                if config.encode(&mut payload).is_ok() {
                    let _ = core.on_frame(now_ms, CONFIG_ID, &payload);
                }
            }
            self.next_output_ms = now_ms.wrapping_add(OUTPUT_PERIOD_MS);
        }
        self.service().poll(now_us);
    }

    /// The oldest jog forwarded and not yet taken: the profile's axis index and the distance in thousandths of a unit.
    pub fn take_jog(&mut self) -> Option<(u8, i32)> {
        self.jogs.pop_front()
    }
}

impl Drop for Controller {
    fn drop(&mut self) {
        self.service = None;
        // SAFETY: `profile` came from `Box::into_raw` in `new` and the service that borrowed it is gone.
        drop(unsafe { Box::from_raw(self.profile) });
    }
}
