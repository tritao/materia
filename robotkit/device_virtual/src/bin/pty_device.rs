mod shared;
fn main() { shared::run(1, std::array::from_fn(|i| i as u8)); }
