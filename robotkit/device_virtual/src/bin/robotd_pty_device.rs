mod shared;
fn main() {
    let hex = std::env::args().nth(3).expect("deployment fingerprint");
    assert_eq!(hex.len(), 32);
    let mut fingerprint = [0u8; 16];
    for (i, byte) in fingerprint.iter_mut().enumerate() {
        *byte = u8::from_str_radix(&hex[i * 2..i * 2 + 2], 16).unwrap();
    }
    shared::run(3, fingerprint);
}
