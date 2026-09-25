# RobotKit device protocol core

This `#![no_std]` crate contains generated fixed-record codecs and a bounded
RKD5 device runtime. It has no allocator, HAL, RTIC, GPIO, motor, encoder, or
CAN dependency. Build it with `cargo test --manifest-path
robotkit/device_protocol/Cargo.toml` on a host.

Construct `DeviceProtocol` with the compiled joint count and 16-byte model
fingerprint. Feed serial bytes and a monotonic receive time to `feed`. The
`Device` adapter must synchronously stop outputs and clear targets in
`stop_all`, validate and apply targets in `apply_targets`, decide whether a
safety reset is allowed, and return a bounded joint snapshot. After a session
begin, `feed` emits `SESSION_ACK` followed by an initial latched-safe `STATE`
through a callback; that callback must copy or queue each borrowed frame
before returning. `encode_state` writes later `STATE` frames into a
caller-owned buffer. The adapter must call it periodically even without new
commands; the host runtime faults if state stops arriving. The desktop PTY
adapter uses a 25 ms state interval.

Call `poll_watchdog` from a device-local clock task with the configured timeout
even when no bytes arrive. On expiry it calls `stop_all`, latches safety, and
clears watchdog credit. The physical safety circuit and RTIC scheduling remain
the responsibility of the MCU adapter.

The host-side POSIX `HostLink` and PTY exchange test live under
`robotkit/runtime`. The host runtime uses RKD5; physical UART and MCU validation
remain to be done.
