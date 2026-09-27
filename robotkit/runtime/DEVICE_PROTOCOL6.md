# RobotKit scheduled device protocol (RKD6)

RKD6 is a separate protocol from [RKD5](DEVICE_PROTOCOL.md). Its four-byte
sync marker is `RKD6`. A frame is marker (4), message type (1), reserved zero
(1), little-endian payload length (2), payload, then little-endian CRC-32/IEEE
(4), calculated over every preceding byte. The largest current payload is the
version 9 session record (4,908 bytes), so the frame maximum is 4,920 bytes.
A receiver rejects unknown types, wrong lengths, a nonzero reserved byte,
CRC mismatch, or malformed session, segment and event records.

`device_wire6.wire.idl` is the canonical fixed-record schema; `wire6.json`
generates Rust and C++ codecs. Segment coefficients are `f32` in local seconds,
with degree at most five. Segment start and duration are device ticks. One
`Segment6Coefficients` record follows the header per actuator. `STATE6` has
one `ActuatorState6` record per actuator. Queue control and safety command
frames carry the fixed records specified by the schema. The session ACK
reports the tick rate, step tick rate, degree limit and queue capacities.
`SESSION_BEGIN6` is one fixed record. Its 64-slot acceleration array carries
the active actuator limits, each positive and no greater than the global cap.
It also carries the link-loss timeout in nanoseconds. The device converts that
timeout using its own clock after the session begins.
Protocol version 8 also carries steps per actuator unit, actuator rate limits,
direction setup ticks, source joint indices, transmission ratios and dual-drive
skew bounds. Stable actuator IDs, channel order and transmission fields are
part of the RKD6 endpoint fingerprint.
The host converts joint polynomials to actuator polynomials before encoding and
rejects motion faster than either the authored actuator rate or one step per
device step tick. The no_std device generator uses the configured direction
setup and minimum step interval, and checks dual-drive skew from step feedback.
Protocol version 9 declares up to 32 channels in the session, including stable
IDs, kinds and safe values. An `EVENT` carries its queue revision, plan ID,
device path tick, channel index, typed value and HOLD policy. The host maps
each `TimedEvent` from plan-relative path time with the same frozen clock
mapping as its trajectory segments. The device fires only committed events as
its path clock crosses their ticks. HOLD makes configured channels safe,
RESUME restores `RestoreOnResume` values, and STOP or a fault discards future
events and sets every channel to its declared safe value. Replacement discards
events at or after the replacement boundary. The virtual event log records
scheduled path ticks, applied path ticks and device ticks.

No RKD6 hardware has shipped. Before the first hardware release, an in-place
schema revision is permitted with a `PROTOCOL_VERSION` bump and regenerated
Rust and C++ codecs, lock, and shared vectors. The schema freezes at the first
hardware release; subsequent changes must preserve that released wire format
or negotiate a new version.

| Quantity | RKD6 bound | Source |
| --- | ---: | --- |
| Maximum actuators | 64 | schema |
| Maximum frame | 4,920 bytes | 8 + 4,908 + 4 |
| Maximum segment frame | 1,648 bytes | 8 + 36 + 64 × 25 + 4 |
| Step tick default | 40 kHz | LA-D2; board may configure another rate |
| Segment queue depth | negotiated | `SESSION_ACK6.segment_capacity` |
| Event queue depth | negotiated | `SESSION_ACK6.event_capacity` |
| Committed horizon | link latency + 2 × clock uncertainty beyond host horizon | A3 |
| Wire transmission time | `10 × frame_bytes / baud` seconds for 8N1 | serial qualification input |

A host must qualify baud and queue depth against the declared segment rate
before motion. Clock-sync uncertainty and link-loss timeout are deployment
bounds, not hard-coded protocol constants. `TIME_SYNC_REPLY` carries device
receive and transmit ticks for the estimator in A3. The RKD5 target-streaming
path remains supported separately.

The host estimator fits device ticks against host monotonic nanoseconds from
the lowest RTT samples in a bounded window. Its uncertainty is half the
minimum RTT plus the worst selected fit residual. It exposes request cadence,
time mapping and the extra commit horizon (`link latency + 2 × uncertainty`).
When a new sample steps outside the deployment bound, it latches
`clock_sync_lost` and disallows further commits. An RKD6 endpoint sends the
periodic requests and reports that reason in its runtime snapshot.

The host device compiler maps plan-relative knots to device ticks, converts
segment-local coefficients to `f32`, and runs `mk_validate` on the converted
trajectory at the step-tick resolution. It samples the converted `f32` Horner
evaluation against the original plan at that resolution and rejects a plan
that exceeds `target_error`. `Rkd6Endpoint` forwards queue revisions, segments
and commits over a complete-frame transport. It qualifies baud and queue
depth at construction and checks the shortest submitted segment again before
sending a plan. The runtime leaves per-cycle target streaming to RKD5
endpoints; RKD6 snapshots use device queue status.
# In-process virtual device

`robotkit/device_virtual` links the no_std scheduled core and step generator to the `std`
virtual board as a static library. Its C ABI in
`robotkit/device_virtual/include/rkd_virtual.h` accepts and returns complete
RKD6 frames and advances on the caller's simulated host clock. The board
applies offset and ppm drift before each 40 kHz step tick. A step pulse is
recorded with its device tick, actuator and direction. Missed-step injection
can exercise the latched dual-drive skew fault.

`VirtualDeviceEndpoint` owns that library behind a deterministic complete
frame link. Baud, latency, jitter, frame drop, corruption and RNG seed are
configurable. A sampled line error delays a frame by one packet time before
retry; `cut_link(true)` suppresses host-to-device frames while leaving
telemetry available to observe device-side `link_lost`. The SimKit robot
descriptor can select this endpoint, and `Simulation.addRobot` exposes it as
`VirtualDeviceOptions`. The step-count position drives each SimKit joint
through a position target. Device status and state are sampled on the
simulation owner clock.
