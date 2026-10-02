# RobotKit scheduled device protocol (RKD6)

The four-byte sync marker is `RKD6`. A frame is marker (4), message type (1), reserved zero
(1), little-endian payload length (2), payload, then little-endian CRC-32/IEEE
(4), calculated over every preceding byte. The largest current payload is the
version 10 session record (4,908 bytes), so the frame maximum is 4,920 bytes.
A receiver rejects unknown types, wrong lengths, a nonzero reserved byte,
CRC mismatch, or malformed session, segment and event records.

`device_wire6.wire.idl` is the canonical fixed-record schema; `wire6.json`
generates Rust and C++ codecs. Segment coefficients are `f32` in local seconds,
with degree at most five. Segment start and duration are device ticks. One
`Segment6Coefficients` record follows the header per actuator. `STATE6` has
one `ActuatorState6` record per actuator. Queue control and safety command
frames carry the fixed records specified by the schema. The session ACK reports the tick rate, step tick rate, degree limit, queue capacities and profile.
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
RESUME restores `RestoreOnResume` values, and STOP, ABORT or a fault discards
future events and sets every channel to its declared safe value, except that a
commanded STOP or ABORT leaves a channel whose stop policy is keep-on-stop as it
is (protocol version 12), so a vacuum holding a part does not drop it when the
arm is stopped. An emergency stop, link loss or fault still makes every channel
safe. Stopping the motion never touches the channels: they are the events'. Replacement discards
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
receive and transmit ticks for the estimator in A3.

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
sending a plan. RKD6 snapshots use device queue status. Per-cycle setpoint sampling remains available for cyclic-control endpoints such as SimKit.
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
configurable. Frames leave the line one after another and each arrives the
link latency later, so latency is not serialized; a sampled line error delays
a frame by one packet time before retry; `cut_link(true)` suppresses host-to-device frames while leaving
telemetry available to observe device-side `link_lost`. The SimKit robot
descriptor can select this endpoint, and `Simulation.addRobot` exposes it as
`VirtualDeviceOptions`. The step-count position drives each SimKit joint
through a position target. Device status and state are sampled on the
simulation owner clock.

## Device profiles and serial deployment

Protocol version 10 adds `SESSION_ACK6.profile`. Full devices report profile 1,
support degree up to their declared maximum, queue segments and generate steps.
Minimal devices report profile 2, maximum degree 1, and a small queue (the
reference device has eight slots). They output position setpoints and generate
no steps. The host lowers source segments at the owner period, checks the
converted f32 path against the original trajectory at step-tick resolution,
and revalidates the lowered path. It accounts for frames in flight when
streaming into a small queue. Qualification reports minimum baud, queue depth
and period before construction succeeds.

Protocol version 12 adds `channel_stop_policy` to `SESSION_BEGIN6`, one byte per
channel: 0 makes the channel safe on every stop, 1 keeps it through a commanded
STOP or ABORT, as the host runtime's `RK_CHANNEL_KEEP_ON_STOP` does.

Protocol version 11 adds two `QUEUE_STATUS6` fields. `received_until_ticks` is
the end of the last segment the device holds, so a replacement is accepted only
at a boundary the device has, as it reports, rather than one a commit implied.
`received_bytes` counts every byte the device has read since the session began
(the `SESSION_BEGIN6` frame excluded), in 64 bits so it never wraps; the host
keeps the same count of bytes sent, and the difference is what is still in
flight (none when line noise puts the device's count ahead), in buffers no host
API reports, such as a USB adapter's. The host sends segments only while the
line would clear within its segment budget: a commit falls due within
`link latency + 2 × uncertainty + 2 × owner period` of the device's committed
horizon, is seen up to an owner period late, then waits for the line and
crosses the link, so the budget is that margin less the latency, a period and a
commit frame. The backlog is the largest of what the host has handed the line
by its own count, what the transport reports (`TIOCOUTQ` on a serial port), and
what the device's last status has not received, less the line time since and
the latency.

Each submitted chunk keeps the clock mapping it was compiled with. A
continuation or replacement starts on the device tick the queued path has at
that path time through that mapping, and commits map the same way, so a time
sync that refines the estimate between chunks never moves a boundary off the
queued segments. An append adds segments to the current queue revision; only
a replacement or a fresh queue opens a new one.

A queue revision begins only at a segment boundary the device holds. A
replacement inside a segment begins its revision at that segment's start and
sends the segment again cut short at the boundary (a segment's polynomial runs
from its own start, so only its length changes), followed by the new plan; it
also sends again the replaced plan's events in that stretch, since a revision
drops the device's events from its boundary on. That segment must start at or
beyond the committed horizon. The host commits through the first held segment
ending half a second (or two commit margins, if more) beyond the device's path
clock: far enough that the device keeps moving through a host stall such as a
garbage collection, and no further, so the path beyond stays open to
replacement.

Deployment schema v4 implies RKD6 and omits `protocol`. The v3 reader accepts
only an explicit `rkd6` declaration. Layout fingerprints use the canonical
RKD6 schema lock and exact layout bytes; ordered actuator and process-channel
fields further specialize the endpoint fingerprint. The POSIX serial endpoint,
virtual endpoint, PTY harness and two-joint Nucleo stub share this protocol.
