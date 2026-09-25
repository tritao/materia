# RobotKit device protocol v5 (draft)

This is the UART byte-stream protocol between a Linux host and a device MCU.
Version 4 remains the active `SerialRobotEndpoint` protocol during migration.
The fixed payload records come from `robotkit/schema/device_wire.wire.idl`.
The hardware-independent Rust `robotkit-device-protocol` crate implements the
device parser, session and watchdog state, and outgoing ACK/STATE encoding.
The POSIX `HostLink` implements v5 negotiation, commands, and state sampling
beside the v4 endpoint. Its PTY test runs the Rust device core in a separate
process against the C++ host, covering session negotiation, commands, a
simulated watchdog expiry, restart, and model rejection. It uses a virtual
serial port; physical UART timing and MCU integration remain untested.

## Frame

| Offset | Type | Meaning |
| ---: | --- | --- |
| 0 | 4 bytes | ASCII `RKD5` synchronization marker |
| 4 | u8 | `MessageType`: 1 session begin, 2 session ack, 3 command, 4 state |
| 5 | u8 | flags, zero in v5 |
| 6 | u16 little-endian | payload size |
| 8 | bytes | exactly one typed payload |
| 8 + size | u32 little-endian | CRC-32/ISO-HDLC of bytes 0 through 7 + size |

CRC uses reflected polynomial `0xedb88320`, initial value `0xffffffff`,
and final XOR `0xffffffff`. A receiver searches for `RKD5` after a corrupt
frame. A bogus length is rejected as soon as the 8-byte header is available.
The maximum payload is 796 bytes; the maximum complete frame is 808 bytes.
There is no HMPK or MessagePack in this protocol.

## Four messages

| Type | Direction | Payload size and rule |
| --- | --- | --- |
| `SESSION_BEGIN` | host to device | exactly 24 bytes: `SessionBegin` |
| `SESSION_ACK` | device to host | exactly 28 bytes: `SessionAck` |
| `COMMAND` | host to device | `CommandHeader` (20) + `target_count` × `JointTarget` (8), at most 532 bytes |
| `STATE` | device to host | `StateHeader` (28) + `joint_count` × `JointState` (12), at most 796 bytes |

Counts are at most 64. The device rejects an invalid length, nonzero reserved
byte, unknown command kind or target mode, nonzero target flags, duplicate or
out-of-range joint index, and nonfinite target value. Targets commands carry
at least one target; stop, emergency stop, and safety reset carry none. `STATE`
contains only machine-control state. Telemetry is a future independent message.

## Session and safety

The host chooses a fresh nonzero random `u64` session. Every valid session
begin causes the device to stop all actuators, clear targets, latch safety,
reset the accepted sequence, and clear watchdog credit *before* it acknowledges
the session. The device compares the host's 16-byte model fingerprint with its
compiled fingerprint. Mismatch is acknowledged with `model_mismatch`; no
commands may be accepted in that session. The fingerprint catches accidental
model/layout mismatch; it is not an authentication mechanism.

A matching session is acknowledged `latched_safe`. Motion remains forbidden
until an explicit accepted `reset_safety` command, subject to device safety
hardware and application checks. Sequence numbers must start above zero and
increase strictly within the session. Only a fully validated command accepted
by the device application advances the watermark and refreshes the watchdog.
Malformed, stale, wrong-session, mismatched-model, or rejected commands do
neither. Emergency stop latches safety. Watchdog expiry is evaluated on the
device's local monotonic clock and must stop and latch actuators. Fault and
emergency state must be sent without waiting for command acknowledgement.

The device timestamp in `STATE` is a monotonic nanosecond clock. `STATE`
reports the active session and last accepted command sequence. The host may
ignore ordinary stale states while still publishing fault or emergency state
immediately. The outer-frame message type and each payload count must agree
with the exact payload length.

## Capacity and response time

At 8N1, a frame of `N` bytes takes `10N / baud` seconds on the wire. With 64
joints, command is at most 544 bytes including framing and state is at most
808 bytes. A full command plus state exchange therefore transmits 1,352 bytes.

| Baud | Max command | Max state | Both directions |
| ---: | ---: | ---: | ---: |
| 115,200 | 47.23 ms | 70.14 ms | 117.36 ms |
| 230,400 | 23.61 ms | 35.07 ms | 58.68 ms |
| 460,800 | 11.81 ms | 17.53 ms | 29.34 ms |
| 921,600 | 5.90 ms | 8.77 ms | 14.67 ms |

Supported v5 UART rates begin at 115,200 baud. A response deadline should be
derived as the full command-plus-state transmission time at the configured
baud plus measured device processing allowance and host scheduling margin.
For the initial implementation, reserve 10 ms processing and 20 ms scheduling;
at 115,200 baud this yields a minimum 148 ms deadline after rounding up.
The device watchdog period must exceed the command period plus worst-case
command transmission and processing time, with explicit margin. Do not inherit
v4's 9,600 baud support or its fixed 250 ms response constant by accident.

The `f32` target/state values and 16-byte model fingerprint are draft choices.
At magnitudes of 1, 100, and 1,000 SI units, adjacent `f32` values are about
1.19e-7, 7.63e-6, and 6.10e-5 units apart respectively. A rounded target can
therefore differ by up to half that spacing. `HostLink::send_targets` accepts
`double` values only when their actual conversion error stays within a caller
supplied absolute SI-unit budget; nonfinite, overflowing, and nonzero values
that round to zero are rejected. A deployed robot must set that budget from
its encoder resolution, gearing, travel range, and control accuracy. Validate
state precision over the same physical range before freezing the schema.

The fingerprint must be derived from a deployed model/device layout artifact,
including the ordered joint-to-channel map and safety-relevant calibration.
RobotKit does not yet have that artifact, so tests use explicit fingerprints.
Do not derive it from a mutable model name or process-local hash.
