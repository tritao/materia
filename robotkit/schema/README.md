# RobotKit RKD6 payload schema

`device_wire6.wire.idl` is the fixed-record source. `../wire6.json` generates
`device_protocol/src/device_wire6.rs` and `runtime/generated/device_wire6.hpp`.
The lock records field order and sizes; protocol version bumps permit in-place
revisions until the first hardware release. See [the protocol](../runtime/DEVICE_PROTOCOL.md)
for framing, timing, device profiles and the hardware freeze policy.

Session identity and agreement are checked while the session opens (controller id
and a 64-bit FNV-1a configuration digest), not by a compiled constant; see
the protocol.
