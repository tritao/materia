//! Jog pendant CAN-FD payloads. The layouts come from `schema/pendant.wire.idl` and are generated
//! into `pendant_wire.rs`; this file adds the validation the schema language cannot express
//! (value ranges, reserved bits, NUL-terminated text). `#![no_std]` for the pendant and the
//! cabinet controller.
#![no_std]

pub mod pendant_wire;

use pendant_wire::{ConfigKind, LeaseState, PendantConfig, PendantOutput, PendantState};

pub use pendant_wire::Error as WireError;

pub const STATE_ID: u32 = 0x181;
pub const CONFIG_ID: u32 = 0x201;
pub const OUTPUT_ID: u32 = 0x202;

pub const FLAG_WHEEL_ENABLE: u8 = 1;
pub const FLAG_CHAIN_CLOSED: u8 = 2;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Invalid {
    Wire(WireError),
    Position,
    Step,
    Buttons,
    ReservedBits,
    Kind,
    Index,
    Lease,
    Text,
}

impl From<WireError> for Invalid {
    fn from(e: WireError) -> Self { Invalid::Wire(e) }
}

/// Decode and validate a PENDANT_STATE payload.
pub fn decode_state(input: &[u8]) -> Result<PendantState, Invalid> {
    let s = PendantState::decode(input)?;
    if s.position > pendant_wire::POSITIONS { return Err(Invalid::Position); }
    if s.step > 2 { return Err(Invalid::Step); }
    if s.buttons > 0x3F { return Err(Invalid::Buttons); }
    if s.flags & !(FLAG_WHEEL_ENABLE | FLAG_CHAIN_CLOSED) != 0 { return Err(Invalid::ReservedBits); }
    Ok(s)
}

/// Decode and validate a PENDANT_CONFIG payload; the label is `text` up to its NUL.
pub fn decode_config(input: &[u8]) -> Result<PendantConfig, Invalid> {
    let c = PendantConfig::decode(input)?;
    if c.reserved != [0; 2] { return Err(Invalid::ReservedBits); }
    if c.kind == ConfigKind::Axis as u8 {
        if c.index < 1 || c.index > pendant_wire::POSITIONS { return Err(Invalid::Index); }
    } else if c.kind == ConfigKind::Key as u8 {
        if c.index > 2 { return Err(Invalid::Index); }
    } else {
        return Err(Invalid::Kind);
    }
    check_text(&c.text)?;
    Ok(c)
}

/// Decode and validate a PENDANT_OUTPUT payload.
pub fn decode_output(input: &[u8]) -> Result<PendantOutput, Invalid> {
    let o = PendantOutput::decode(input)?;
    if o.reserved != 0 { return Err(Invalid::ReservedBits); }
    if o.leds > 0x3F { return Err(Invalid::Buttons); }
    if o.lease != LeaseState::NoLease as u8 && o.lease != LeaseState::Granted as u8 && o.lease != LeaseState::Lost as u8 {
        return Err(Invalid::Lease);
    }
    check_text(&o.text)?;
    Ok(o)
}

/// The label bytes before the NUL terminator.
pub fn label(text: &[u8]) -> &[u8] {
    let end = text.iter().position(|&b| b == 0).unwrap_or(text.len());
    &text[..end]
}

/// Printable ASCII up to the first NUL, and only NUL after it.
fn check_text(text: &[u8]) -> Result<(), Invalid> {
    let end = label(text).len();
    if text[..end].iter().any(|&b| !(32..=126).contains(&b)) { return Err(Invalid::Text); }
    if text[end..].iter().any(|&b| b != 0) { return Err(Invalid::Text); }
    Ok(())
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
        for (index, value, error) in [(4, 12, Invalid::Position), (5, 3, Invalid::Step), (6, 0x40, Invalid::Buttons), (7, 4, Invalid::ReservedBits)] {
            let mut bad = b;
            bad[index] = value;
            assert_eq!(decode_state(&bad), Err(error));
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
        for (kind, index) in [(0u8, 0u8), (0, 12), (1, 3), (2, 1)] {
            let bad = PendantConfig { kind, index, text: text("X"), reserved: [0; 2] };
            bad.encode(&mut b).unwrap();
            assert!(decode_config(&b).is_err());
        }
        let mut junk = PendantConfig { kind: 0, index: 1, text: text("X"), reserved: [0; 2] };
        junk.text[5] = b'A';
        junk.encode(&mut b).unwrap();
        assert_eq!(decode_config(&b), Err(Invalid::Text));
        let padded = PendantConfig { kind: 0, index: 1, text: text("X"), reserved: [0, 1] };
        padded.encode(&mut b).unwrap();
        assert_eq!(decode_config(&b), Err(Invalid::ReservedBits));
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
        for (lease, leds, reserved, error) in [(9u8, 0u8, 0u8, Invalid::Lease), (0, 0x40, 0, Invalid::Buttons), (0, 0, 1, Invalid::ReservedBits)] {
            PendantOutput { leds, lease, text: text(""), reserved }.encode(&mut b).unwrap();
            assert_eq!(decode_output(&b), Err(error));
        }
    }
}
