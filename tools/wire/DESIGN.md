# Materia wire tooling

`tools/wire` parses `.wire.idl` schemas, validates published locks, and renders
selected artifacts. It defines typed payloads and fixed packed layouts. It does
not define frames, sessions, RPC, CRCs, watchdogs, or bus abstractions.

## Consumers

| Consumer | Schema use | Encoding | Framing |
| --- | --- | --- | --- |
| SensorKit | numbered messages and packed sensor samples | Haxeon MessagePack plus packed arrays | HMPK where used |
| RobotKit device protocol | constants, enums, packed records | fixed binary with explicit endianness | RobotKit v5 |

SensorKit's MessagePack record codecs belong to its Haxe/C++ host path.
RobotKit needs only fixed packed records shared by C++ and Rust. There is no
Rust MessagePack backend in the device-protocol scope.

## Compiler and outputs

`python -m tools.wire {validate,check,generate} --config <consumer>/wire.json`
reads one schema and a checked-in compatibility lock. The shared driver owns
validation, output planning, stale-file checks, and writes. Backend renderers
return text; they do not write files. Every output path is relative to the
manifest directory and must remain inside it. Consumer-specific test vectors
may be rendered by a configured adapter; SensorKit's HMPK vectors stay in
SensorKit.

`tools/wire/PROFILE.md` defines the current language and compatibility rules.
A schema may have messages, packed layouts, or both. The existing SensorKit
lock and generated wire bytes remain stable across tooling changes.

## Next backend and RobotKit boundary

Add a Rust `no_std` backend for **packed layouts only**. It generates ordinary
value structs and endian-aware `encode(&T, &mut [u8])` and
`decode(&[u8]) -> Result<T, Error>` functions. No allocation, MessagePack tree,
reflection, or RTIC dependency is needed. Compare its packed bytes against
C++ using checked-in vectors.

RobotKit's future `device_wire.wire.idl` should describe fixed records such as
`SessionBegin`, `SessionAck`, `CommandHeader`, `JointTarget`, `StateHeader`,
and `JointState`. Variable joint arrays are a count followed by repeated
fixed records; the schema does not need a generic repeated-object feature.
RobotKit owns its versioned outer frame and safety state machine.
