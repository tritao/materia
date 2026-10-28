//! External authority over jogging. A deployment can place an operator device (a pendant) between
//! the host and the motion core: the host still plans jog segments (`purpose = 1`) and the core
//! still executes them, but the gate decides whether an executing jog may continue. Programs and
//! homing are never gated here. Not a safety function.

use crate::ScheduledCore;

/// `Segment6Header.purpose` value of a jog segment (schema/device_wire6.wire.idl).
pub const PURPOSE_JOG: u8 = 1;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct JogVerdict {
    /// A jog segment may execute.
    pub permit: bool,
    /// An executing jog must stop now.
    pub stop_jog: bool,
}

/// Asked once per control tick, with the device clock in microseconds, before the core ticks.
pub trait JogGate {
    fn verdict(&mut self, now_us: u64) -> JogVerdict;
}

/// True when the core is executing a jog that the verdict says must stop. The caller then issues
/// an ordinary commanded stop, exactly as for a STOP frame.
pub fn jog_must_stop<const A: usize, const CAP: usize>(core: &ScheduledCore<A, CAP>, verdict: &JogVerdict) -> bool {
    verdict.stop_jog && core.stop_reason().is_none() && core.executing_purpose() == Some(PURPOSE_JOG)
}
