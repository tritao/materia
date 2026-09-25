# RobotKit device payload schema (draft v5)

`device_wire.wire.idl` defines fixed binary payload records only. The generated
C++ and Rust codecs have no MessagePack or transport dependency. `wire.json`
selects the packed backends; the lock records published field order and sizes.
The [v5 device protocol](../runtime/DEVICE_PROTOCOL_V5.md) specifies framing,
session behavior, limits, and timing around these records.

The hardware-independent Rust device core in `device_protocol/src/runtime.rs`
now owns framing, CRC, session state, command validation, and watchdog state.
The device adapter still owns physical stop outputs, safety inputs, and joint
I/O. The MCU must compare the 16-byte `SessionBegin.model_fingerprint` with
its compiled model identity and stay latched safe on mismatch.

`COMMAND` payloads will be `CommandHeader` followed by exactly
`target_count * JointTarget::SIZE` bytes. `STATE` payloads will be `StateHeader`
followed by exactly `joint_count * JointState::SIZE` bytes. Counts are capped by
`MAX_JOINTS`. Reserved bytes must be zero when transmitted and checked by the
protocol decoder. The outer frame supplies the message type and payload size.

The `f32` joint values are a proposed wire precision. The host's double-to-f32
target path now requires an explicit absolute error budget; physical resolution
and state range still need validation before freezing the v5 schema. The
current v4 serial endpoint remains active until v5 conformance and safety
tests pass.
