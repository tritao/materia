# RobotKit serial protocol, version 4

Version 4 remains the active endpoint. The proposed packed-record successor is
specified in [DEVICE_PROTOCOL_V5.md](DEVICE_PROTOCOL_V5.md).

This protocol connects SerialRobotEndpoint to a device controller over a
POSIX serial port. Frames are little-endian. Floating-point fields use IEEE
754 binary64. All payload lengths and array counts are checked before data is
copied into runtime state.

## Frame envelope

Every frame has this envelope:

| Offset | Type | Meaning |
| ---: | --- | --- |
| 0 | 4 bytes | Direction and version magic (RKH4, RKC4, or RKS4) |
| 4 | u32 | Payload byte length, excluding envelope and CRC |
| 8 | bytes | Payload |
| 8 + length | u32 | CRC-32/ISO-HDLC over magic, length, and payload |

CRC-32 uses reflected polynomial 0xedb88320, initial value 0xffffffff,
and final XOR 0xffffffff. The CRC is written little-endian. Invalid CRCs are
discarded while the receiver searches for the next frame magic.

The host begins a session with RKH4, then sends RKC4 command frames. The device
returns RKS4 state frames. The maximum state payload is 5,828 bytes. A frame
with larger lengths or counts is rejected.

## Session start

The host sends an RKH4 frame with an 8-byte payload containing a nonzero,
random u64 session ID. Before acknowledging it, the device must stop all
actuators, latch its emergency stop, clear active targets, set the accepted
command sequence to zero, and switch to the requested session ID. This makes a
host restart a safe sequence boundary.

The device confirms the session in its next RKS4 state frame. The host opens
the endpoint only after receiving a fresh state frame with the requested
session ID, accepted command sequence zero, and safety state emergency-stop
(or fault). RobotRuntime starts in that confirmed safety state; the
application must explicitly reset safety before sending motion targets.

## Host command payload

The payload starts with a 28-byte header:

| Offset | Type | Meaning |
| ---: | --- | --- |
| 0 | u32 | rk_command_kind |
| 4 | u64 | Active host session ID |
| 12 | u64 | Strictly increasing command sequence within the session |
| 20 | u32 | Number of joint targets, at most 64 |
| 24 | u32 | Reserved; transmit zero |

The header is followed by target_count 16-byte records:

| Record offset | Type | Meaning |
| ---: | --- | --- |
| 0 | u32 | Compiled joint index |
| 4 | u32 | rk_joint_target_mode |
| 8 | f64 | Position, velocity, or effort target in SI units |

The joint index is the position in the compiled RobotModel.joints array.
Sparse batches and mixed target modes are preserved as one frame. Lifecycle
commands such as stop and emergency stop carry zero targets. The device should
reject invalid modes, duplicate joint indices, invalid CRCs, and sequence
regressions. It must also stop actuators locally if command traffic disappears;
the host process cannot provide a watchdog after a cable or process failure.

The exact payload length is 28 + 16 * target_count.

`robotkit_serial_protocol.hpp` provides a fixed-capacity `SerialCommandDecoder`
for RKH4 and RKC4. Its session handler must stop and latch the device before
accepting a new host session. The decoder delivers a complete
`SerialCommandFrame` only after validating payload size, CRC, active session,
command kind, target modes, finite values, duplicate joints, configured joint
range, and sequence order. Only delivered commands refresh its watchdog.
Call `watchdog_expired()` on the device's local clock and stop actuators when
it returns true. The decoder does not implement motor drivers, safety circuits,
or the device's application of stop, emergency-stop, and reset commands.

## Device state payload

The payload begins with a 36-byte header:

| Offset | Type | Meaning |
| ---: | --- | --- |
| 0 | u32 | Joint count; must match the compiled model |
| 4 | u64 | Strictly increasing device clock timestamp in nanoseconds |
| 12 | u32 | Sensor slot count, at most 8 |
| 16 | u64 | Active host session ID |
| 24 | u64 | Last accepted command sequence in this session |
| 32 | u32 | Current rk_safety_state |

Each joint contributes 24 bytes in compiled joint order: position f64,
velocity f64, then effort f64.

Each sensor slot is:

| Field | Type | Meaning |
| --- | --- | --- |
| sequence | u64 | Sensor acquisition sequence; zero means no sample |
| source timestamp | u64 | Sensor clock timestamp in nanoseconds |
| value count | u32 | Number of following f64 values, at most 64 |
| values | f64[] | Encoder, IMU, or LiDAR values for that compiled slot |

Sensor records follow the same order as RobotRuntimeBlueprint.sensors and
must include empty slots for sensors that have not acquired a sample. A device
may send sensor_count = 0 to omit physical sensor data; RobotKit will still
project configured joint encoders from the joint state. Otherwise the sensor
slot count must exactly match the compiled model so values cannot be assigned
to the wrong sensor or frame. IMU values follow RobotKit's six-value order and
LiDAR values follow the model's configured first-ray bearing and angular
coverage.

The exact payload length is 36 + 24 * joint_count + sum(20 + 8 * value_count).
Sensor source timestamps can differ from the enclosing joint-state timestamp
because each sensor may run at a different rate.

## Timing and failures

The host waits up to 250 ms for a fresh state frame on each runtime sample.
It ignores duplicate or older device timestamps, state from another session,
and states that have not yet observed the latest command. Emergency-stop and
fault states are published immediately even if their accepted command sequence
is behind. A silent or disconnected link faults the runtime, which issues a
best-effort emergency-stop frame. Command writes also wait up to 250 ms for a
blocked serial output queue. A hardware watchdog remains required because a
detached device cannot receive that frame. The endpoint supports 9,600 through
115,200 baud and configures 8 data bits, no parity, one stop bit, and no
hardware flow control.

Configure the device with the same compiled joint and sensor order as the host
model before enabling actuator power. Version 4 establishes each host session
in a latched safe state and waits for device state to confirm applied command
sequences.
