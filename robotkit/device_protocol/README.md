# RobotKit RKD6 device core

This `#![no_std]` crate contains the generated RKD6 records, frame codec,
fixed-capacity scheduled executor, event queue, step generator and board trait.
The `std` feature adds a deterministic virtual board. It has no allocator or
HAL dependency in the MCU build.

Run `cargo test --manifest-path robotkit/device_protocol/Cargo.toml --features std`
and `robotkit/device_protocol/tools/check-mcu-build.sh`. The latter compiles
the core and the Nucleo minimal-profile stub for `thumbv7em-none-eabihf`.
The desktop virtual device and PTY binaries are in `robotkit/device_virtual`.
