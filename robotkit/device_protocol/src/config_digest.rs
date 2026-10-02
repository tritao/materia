//! The session configuration digest both ends compute over a SESSION_BEGIN6 payload.

const OFFSET_BASIS: u64 = 0xcbf2_9ce4_8422_2325;
const PRIME: u64 = 0x0000_0100_0000_01b3;
/// Bytes of the session id that open every SESSION_BEGIN6 payload; the digest skips them.
const SESSION_FIELD_BYTES: usize = 8;

/// 64-bit FNV-1a.
pub fn fnv1a64(bytes: &[u8]) -> u64 {
    let mut hash = OFFSET_BASIS;
    for &byte in bytes {
        hash ^= byte as u64;
        hash = hash.wrapping_mul(PRIME);
    }
    hash
}

/// Digest of a SESSION_BEGIN6 payload after its `session` field: everything the device is
/// configured with. The host compares it with the one the device acknowledges.
pub fn config_digest6(begin_payload: &[u8]) -> u64 {
    fnv1a64(begin_payload.get(SESSION_FIELD_BYTES..).unwrap_or(&[]))
}

/// A session may begin only on the controller its configuration names.
pub fn controller_matches(expected: &[u8; 16], own: &[u8; 16]) -> bool {
    expected == own
}
