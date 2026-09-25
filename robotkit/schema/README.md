# RobotKit device payload schema (draft v5)

`device_wire.wire.idl` defines fixed binary payload records only. The generated
C++ and Rust codecs have no MessagePack or transport dependency. `wire.json`
selects the packed backends; the lock records published field order and sizes.

The proposed v5 stream frame, CRC, session state machine, model-fingerprint
comparison, command validation, and watchdog remain separate protocol work.
This schema does not authorize motion by itself. The device must compare the
16-byte `SessionBegin.model_fingerprint` with its compiled model identity and
stay latched safe on mismatch.

`COMMAND` payloads will be `CommandHeader` followed by exactly
`target_count * JointTarget::SIZE` bytes. `STATE` payloads will be `StateHeader`
followed by exactly `joint_count * JointState::SIZE` bytes. Counts are capped by
`MAX_JOINTS`. Reserved bytes must be zero when transmitted and checked by the
protocol decoder. The outer frame supplies the message type and payload size.

The `f32` joint values are a proposed wire precision. Validate physical
resolution and range before freezing the v5 protocol and schema lock. The
current v4 serial endpoint remains active until v5 conformance and safety
tests pass.
