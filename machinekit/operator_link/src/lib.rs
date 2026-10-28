//! Jog pendant CAN-FD payloads. The layouts come from `schema/operator.wire.idl` and are generated
//! into `operator_wire.rs`; this file adds the validation the schema language cannot express
//! (value ranges, reserved bits, NUL-terminated text). `#![no_std]` for the pendant and the
//! cabinet controller.
#![no_std]

#[cfg(test)]
extern crate std;

pub mod authority;
pub mod host_link;
pub mod operator_frame;
pub mod profile;
pub mod operator_wire;
#[cfg(test)]
mod testing;

use operator_wire::{ConfigKind, PendantConfig, PendantOutput, PendantState};

pub use operator_wire::Error as WireError;

pub const STATE_ID: u32 = 0x181;
pub const CONFIG_ID: u32 = 0x201;
pub const OUTPUT_ID: u32 = 0x202;

pub const FLAG_WHEEL_ENABLE: u8 = 1;
pub const FLAG_CHAIN_CLOSED: u8 = 2;

/// Field constraints (ranges, masks, reserved bytes, enum values, text) are declared in the schema
/// and checked by the generated codec on encode and decode, reported as `WireError::Constraint`.
/// What remains here involves two fields at once.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Invalid {
    Wire(WireError),
    /// The label index is out of range for its kind.
    Index,
}

impl From<WireError> for Invalid {
    fn from(e: WireError) -> Self { Invalid::Wire(e) }
}

/// Decode a PENDANT_STATE payload.
pub fn decode_state(input: &[u8]) -> Result<PendantState, Invalid> {
    Ok(PendantState::decode(input)?)
}

/// Decode a PENDANT_CONFIG payload; the label is `text` up to its NUL.
pub fn decode_config(input: &[u8]) -> Result<PendantConfig, Invalid> {
    let c = PendantConfig::decode(input)?;
    let limit = if c.kind == ConfigKind::Axis as u8 { operator_wire::POSITIONS } else { 2 };
    let first = if c.kind == ConfigKind::Axis as u8 { 1 } else { 0 };
    if c.index < first || c.index > limit { return Err(Invalid::Index); }
    Ok(c)
}

/// Decode a PENDANT_OUTPUT payload.
pub fn decode_output(input: &[u8]) -> Result<PendantOutput, Invalid> {
    Ok(PendantOutput::decode(input)?)
}

/// The label bytes before the NUL terminator.
pub fn label(text: &[u8]) -> &[u8] {
    let end = text.iter().position(|&b| b == 0).unwrap_or(text.len());
    &text[..end]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn text<const N: usize>(s: &str) -> [u8; N] {
        let mut out = [0u8; N];
        out[..s.len()].copy_from_slice(s.as_bytes());
        out
    }

    #[test]
    fn state_layout_is_pinned() {
        let s = PendantState { sequence: 0x1234, wheel_delta: -2, position: 11, step: 2, buttons: 0x2A, flags: 1 };
        let mut b = [0u8; 8];
        assert_eq!(s.encode(&mut b), Ok(8));
        assert_eq!(b, [0x34, 0x12, 0xFE, 0xFF, 0x0B, 0x02, 0x2A, 0x01]);
        assert_eq!(decode_state(&b), Ok(s));
    }

    #[test]
    fn state_round_trips_extremes() {
        for delta in [i16::MIN, -1, 0, 1, i16::MAX] {
            for sequence in [0u16, 1, 65535] {
                let s = PendantState { sequence, wheel_delta: delta, position: 7, step: 1, buttons: 63, flags: 2 };
                let mut b = [0u8; 8];
                s.encode(&mut b).unwrap();
                assert_eq!(decode_state(&b), Ok(s));
            }
        }
    }

    #[test]
    fn state_rejects_bad_values() {
        let good = PendantState { sequence: 1, wheel_delta: 0, position: 3, step: 1, buttons: 0, flags: 3 };
        let mut b = [0u8; 8];
        good.encode(&mut b).unwrap();
        assert_eq!(decode_state(&b[..7]), Err(Invalid::Wire(WireError::WrongLength)));
        for (index, value, field) in [(4, 12, "PendantState.position"), (5, 3, "PendantState.step"), (6, 0x40, "PendantState.buttons"), (7, 4, "PendantState.flags")] {
            let mut bad = b;
            bad[index] = value;
            assert_eq!(decode_state(&bad), Err(Invalid::Wire(WireError::Constraint(field))));
        }
    }

    #[test]
    fn config_round_trips_and_validates() {
        let c = PendantConfig { kind: 0, index: 5, text: text("B hob"), reserved: [0; 2] };
        let mut b = [0u8; 16];
        assert_eq!(c.encode(&mut b), Ok(16));
        assert_eq!(&b[..7], &[0, 5, b'B', b' ', b'h', b'o', b'b']);
        let back = decode_config(&b).unwrap();
        assert_eq!(label(&back.text), b"B hob");
        for (kind, index) in [(0u8, 0u8), (1, 3)] {
            let bad = PendantConfig { kind, index, text: text("X"), reserved: [0; 2] };
            bad.encode(&mut b).unwrap();
            assert_eq!(decode_config(&b), Err(Invalid::Index));
        }
        for (kind, index) in [(0u8, 12u8), (2, 1)] {
            let bad = PendantConfig { kind, index, text: text("X"), reserved: [0; 2] };
            assert!(bad.encode(&mut b).is_err());
        }
        let mut junk = PendantConfig { kind: 0, index: 1, text: text("X"), reserved: [0; 2] };
        junk.text[5] = b'A';
        assert_eq!(junk.encode(&mut b), Err(WireError::Constraint("PendantConfig.text")));
        junk.text[5] = 0;
        junk.encode(&mut b).unwrap();
        b[7] = b'A';
        assert_eq!(decode_config(&b), Err(Invalid::Wire(WireError::Constraint("PendantConfig.text"))));
        let padded = PendantConfig { kind: 0, index: 1, text: text("X"), reserved: [0, 1] };
        assert_eq!(padded.encode(&mut b), Err(WireError::Constraint("PendantConfig.reserved")));
    }

    #[test]
    fn output_round_trips_and_validates() {
        let full = *b"123456789012345678901";
        let o = PendantOutput { leds: 0x15, lease: 1, text: full, reserved: 0 };
        let mut b = [0u8; 24];
        assert_eq!(o.encode(&mut b), Ok(24));
        assert_eq!(decode_output(&b).map(|v| v.text), Ok(full));
        let short = PendantOutput { leds: 0, lease: 2, text: text("Gearbox engaged"), reserved: 0 };
        short.encode(&mut b).unwrap();
        assert_eq!(label(&decode_output(&b).unwrap().text), b"Gearbox engaged");
        assert_eq!(decode_output(&b[..23]), Err(Invalid::Wire(WireError::WrongLength)));
        for (lease, leds, reserved, field) in [(9u8, 0u8, 0u8, "PendantOutput.lease"), (0, 0x40, 0, "PendantOutput.leds"), (0, 0, 1, "PendantOutput.reserved")] {
            let bad = PendantOutput { leds, lease, text: text(""), reserved };
            // The generated encoder refuses the same values the decoder does.
            assert_eq!(bad.encode(&mut b), Err(WireError::Constraint(field)));
            let mut raw = [0u8; 24];
            raw[0] = leds; raw[1] = lease; raw[23] = reserved;
            assert_eq!(decode_output(&raw), Err(Invalid::Wire(WireError::Constraint(field))));
        }
    }
}
