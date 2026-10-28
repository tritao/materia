//! The pendant and operator side of the RKD6 cabinet node: decodes pendant CAN frames and host
//! operator frames, runs the jog authority, and decides whether a jog may keep executing.
//!
//! The host plans jog segments (`purpose = 1`) and the device executes them as usual; this crate
//! only gates them. A jog may execute while the pendant holds authority (its lease is live and its
//! axis switch is not OFF) or, with the switch OFF, while the host's `OperatorState` allows it.
//! Whenever that stops being true, and also the moment a pendant that held authority loses it, an
//! executing jog is stopped. Programs and homing (other purposes) are never touched here.
//! Not a safety function: the E-stop chain is hard wired.
#![no_std]

#[cfg(test)]
extern crate std;

use operator_link::authority::{condition_bit, Authority, Output};
use operator_link::host_link::HostLink;
use operator_link::profile::CompiledProfile;
use operator_link::operator_frame::{self, FrameError};
use operator_link::operator_wire::{OperatorCondition, OperatorMessage, OperatorState, PendantEvent};
use operator_link::{decode_state, Invalid, WireError};
use robotkit_device_protocol::{jog_must_stop, Board, JogGate, JogVerdict, ScheduledCore, StopReason};

/// `Segment6Header.purpose` values (schema/device_wire6.wire.idl).
pub use robotkit_device_protocol::PURPOSE_JOG;
pub const PURPOSE_PROGRAM: u8 = 0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ServiceError {
    Pendant(Invalid),
    Frame(FrameError),
    Wire(WireError),
    /// An operator frame the node does not take here (only `OperatorState` arrives after setup).
    WrongType,
}

/// What the gate decides each tick; the core defines it so any deployment can host a gate.
pub type Verdict = JogVerdict;

pub struct OperatorService<'a> {
    authority: Authority<'a>,
    host: HostLink,
    was_holding: bool,
    event_sequence: u16,
}

impl<'a> OperatorService<'a> {
    /// `profile` is the host's compiled profile, already checked against its digest.
    pub fn new(profile: &'a CompiledProfile) -> Self {
        OperatorService {
            authority: Authority::new(profile.config()),
            host: HostLink::new(profile.host_timeout_us()),
            was_holding: false,
            event_sequence: 0,
        }
    }

    /// A PENDANT_STATE payload from CAN. `active` is the condition mask the controller built from
    /// its own state. Returns the event to forward to the host, or `None` for a stale frame.
    pub fn on_pendant_frame(&mut self, payload: &[u8], now_us: u64, active: u32) -> Result<Option<PendantEvent>, ServiceError> {
        let state = decode_state(payload).map_err(ServiceError::Pendant)?;
        let jog_mode = self.host.jog_allowed(now_us);
        Ok(self.authority.accept(&state, now_us, jog_mode, active).map(|output| self.event(&output)))
    }

    /// One whole `OPR1` frame from the host. After setup only `OperatorState` is taken.
    pub fn on_operator_frame(&mut self, frame: &[u8], now_us: u64) -> Result<bool, ServiceError> {
        let (kind, payload) = operator_frame::decode(frame).map_err(ServiceError::Frame)?;
        if kind != OperatorMessage::State as u8 { return Err(ServiceError::WrongType); }
        let state = OperatorState::decode(payload).map_err(ServiceError::Wire)?;
        Ok(self.host.accept(&state, now_us))
    }

    /// Call every control tick, before the core's own tick.
    pub fn poll(&mut self, now_us: u64) -> Verdict {
        self.authority.tick(now_us);
        let holding = self.authority.holds_authority(now_us);
        let permit = holding || self.host.jog_allowed(now_us);
        // Losing pendant authority stops its jog even if the host would still allow one.
        let lapsed = self.was_holding && !holding;
        self.was_holding = holding;
        Verdict { permit, stop_jog: !permit || lapsed }
    }

    fn event(&mut self, output: &Output) -> PendantEvent {
        self.event_sequence = self.event_sequence.wrapping_add(1);
        output.to_event(self.event_sequence)
    }
}

/// Stops an executing jog when the verdict says so. Returns true if it stopped one.
pub fn enforce_jog<const A: usize, const CAP: usize>(core: &mut ScheduledCore<A, CAP>, verdict: &Verdict) -> bool {
    if jog_must_stop(core, verdict) {
        core.stop(StopReason::Stop);
        true
    } else {
        false
    }
}

/// The condition mask from what the core knows itself; the node ORs in the facts it reads from
/// inputs (gearbox, door, spindle) with `condition_bit`.
pub fn core_conditions<const A: usize, const CAP: usize>(core: &ScheduledCore<A, CAP>) -> u32 {
    if core.executing_purpose() == Some(PURPOSE_PROGRAM) { condition_bit(OperatorCondition::ProgramRunning) } else { 0 }
}

/// Used by the node's setup path to tie the board clock to the service's microseconds.
pub fn ticks_to_us<B: Board>(board: &B) -> u64 { board.now_ticks() * 1_000_000 / board.tick_hz() }

/// The virtual device (and later the board) asks the service directly each control tick.
impl JogGate for OperatorService<'_> {
    fn verdict(&mut self, now_us: u64) -> JogVerdict { self.poll(now_us) }
}
