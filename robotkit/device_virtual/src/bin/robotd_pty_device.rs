mod shared;
fn main() {
    let hex = std::env::args().nth(3).expect("deployment controller");
    assert_eq!(hex.len(), 32);
    let mut controller = [0u8; 16];
    for (i, byte) in controller.iter_mut().enumerate() {
        *byte = u8::from_str_radix(&hex[i * 2..i * 2 + 2], 16).unwrap();
    }
    shared::run(3, controller);
}
