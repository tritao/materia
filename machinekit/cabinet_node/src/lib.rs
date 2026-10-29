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

use operator_link::authority::{condition_bit, Authority, Output, Refusal};
use operator_link::host_link::HostLink;
use operator_link::profile::CompiledProfile;
use operator_link::operator_frame::{self, FrameError};
use operator_link::operator_wire::{ConfigKind, OperatorCondition, OperatorMessage, OperatorState, PendantConfig, PendantEvent, PendantOutput};
use operator_link::{decode_config, decode_state, label, Invalid, WireError};
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

/// Axis names (switch positions 1 to 11) and F-key labels the host gave, to be shown on the pendant.
struct Labels {
    axes: [[u8; 12]; 11],
    keys: [[u8; 12]; 3],
    /// One bit per label: axes in bits 0 to 10, keys in bits 11 to 13.
    set: u16,
    next: u8,
}

pub struct OperatorService<'a> {
    profile: &'a CompiledProfile,
    authority: Authority<'a>,
    host: HostLink,
    was_holding: bool,
    event_sequence: u16,
    last: Option<Output>,
    labels: Labels,
}

impl<'a> OperatorService<'a> {
    /// `profile` is the host's compiled profile, already checked against its digest.
    pub fn new(profile: &'a CompiledProfile) -> Self {
        OperatorService {
            profile,
            authority: Authority::new(profile.config()),
            host: HostLink::new(profile.host_timeout_us()),
            was_holding: false,
            event_sequence: 0,
            last: None,
            labels: Labels { axes: [[0; 12]; 11], keys: [[0; 12]; 3], set: 0, next: 0 },
        }
    }

    /// A PENDANT_STATE payload from CAN. `active` is the condition mask the controller built from
    /// its own state. Returns the event to forward to the host, or `None` for a stale frame.
    pub fn on_pendant_frame(&mut self, payload: &[u8], now_us: u64, active: u32) -> Result<Option<PendantEvent>, ServiceError> {
        let state = decode_state(payload).map_err(ServiceError::Pendant)?;
        let jog_mode = self.host.jog_allowed(now_us);
        Ok(self.authority.accept(&state, now_us, jog_mode, active).map(|output| {
            self.last = Some(output);
            self.event(&output)
        }))
    }

    /// One whole `OPR1` frame from the host. `OperatorState` is the host's jog permission (the result says whether it was fresh);
    /// `Config` is an axis name or F-key label for the pendant's display, kept to be sent on `PENDANT_CONFIG`.
    pub fn on_operator_frame(&mut self, frame: &[u8], now_us: u64) -> Result<bool, ServiceError> {
        let (kind, payload) = operator_frame::decode(frame).map_err(ServiceError::Frame)?;
        if kind == OperatorMessage::Config as u8 {
            let config = decode_config(payload).map_err(ServiceError::Pendant)?;
            let slot = if config.kind == ConfigKind::Axis as u8 {
                self.labels.axes[config.index as usize - 1] = config.text;
                config.index as u16 - 1
            } else {
                self.labels.keys[config.index as usize] = config.text;
                11 + config.index as u16
            };
            self.labels.set |= 1 << slot;
            return Ok(true);
        }
        if kind != OperatorMessage::State as u8 { return Err(ServiceError::WrongType); }
        let state = OperatorState::decode(payload).map_err(ServiceError::Wire)?;
        Ok(self.host.accept(&state, now_us))
    }

    /// The next label to send on `PENDANT_CONFIG`, one per call, cycling through the ones the host gave; `None` before any arrived.
    /// Call it about every 100 ms so a pendant that has just started, or lost a frame, soon has all of them.
    pub fn next_config(&mut self) -> Option<PendantConfig> {
        for _ in 0..14 {
            let slot = self.labels.next;
            self.labels.next = (slot + 1) % 14;
            if self.labels.set & (1 << slot) == 0 { continue; }
            return Some(if slot < 11 {
                PendantConfig { kind: ConfigKind::Axis as u8, index: slot + 1, text: self.labels.axes[slot as usize], reserved: [0; 2] }
            } else {
                PendantConfig { kind: ConfigKind::Key as u8, index: slot - 11, text: self.labels.keys[slot as usize - 11], reserved: [0; 2] }
            });
        }
        None
    }

    /// `PENDANT_OUTPUT`: the LEDs, the lease and the status line, from the last decision on a pendant frame. The LEDs follow the buttons'
    /// order: Start while a program runs (`active` is the same condition mask the pendant frames were judged with), Stop when the
    /// E-stop chain reads open, and the spindle-jog key while it is held. The status line says why a jog was refused, from the profile's
    /// reason text for a rule and fixed text for the rest.
    pub fn pendant_output(&self, now_us: u64, active: u32) -> PendantOutput {
        let mut leds = 0u8;
        if active & condition_bit(OperatorCondition::ProgramRunning) != 0 { leds |= 1; }
        let mut text = [0u8; 21];
        if let Some(last) = &self.last {
            if last.refusal == Refusal::ChainOpen { leds |= 4; }
            if last.spindle_jog {
                if let Some(key) = self.profile.config().spindle_key { leds |= 8 << key; }
            }
            let shown: &[u8] = match last.refusal {
                Refusal::None => b"",
                Refusal::NotJogMode => b"Not in jog mode",
                Refusal::NoAxis => b"Select an axis",
                Refusal::ChainOpen => b"E-stop chain open",
                Refusal::Busy => b"Program running",
                Refusal::ReleaseKeys => b"Release the keys",
                Refusal::Rule(id) => label(self.profile.reason_text_bytes(id)),
            };
            let n = shown.len().min(21);
            text[..n].copy_from_slice(&shown[..n]);
        }
        PendantOutput { leds, lease: self.authority.lease(now_us) as u8, text, reserved: 0 }
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
