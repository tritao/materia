# RobotKit scheduled device protocol (generation 6, wire revision 13)

The protocol name is **RKD generation 6, wire revision 13**. The four-byte sync
marker `RKD6` identifies the scheduled-execution generation, not the wire revision.
`PROTOCOL_VERSION` identifies the wire revision; peers must agree on its exact value
(currently 13). A receiver never infers a revision from the marker or accepts an older
revision. The `6` suffix in `device_wire6`, `device_frame6`, `Rkd6Endpoint`,
`rkd6_endpoint`, `device_compiler6`, `robotkit_device_compiler6` and the serial C API
likewise means generation 6. Keep these generation names when incrementing a wire
revision. A new generation requires a new marker and a new set of identifiers.

This naming scheme is fixed for the first hardware release. No hardware has shipped;
wire layout changes still require a revision bump and regenerated current fixtures.
After hardware ships, a wire revision is immutable.

The four-byte sync marker is `RKD6`. A frame is marker (4), message type (1), reserved zero
(1), little-endian payload length (2), payload, then little-endian CRC-32/IEEE
(4), calculated over every preceding byte. The largest current payload is the
session record (4,940 bytes), so the frame maximum is 4,952 bytes.
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
The lease also covers safe-on-stop process outputs away from their safe values,
including during rest and between motion chunks. Device event policy supplies
this requirement to the scheduled core; a completed, safe program can remain idle.

Wire revision 13 adds `SENSOR6` (message 17). A `Sensor6Header` carries the session,
device acquisition ticks, a nonzero sequence, the compiled sensor slot (0–7), and
value count (1–360). Exactly that many `Sensor6Value` records follow; each is a
finite `f32`. The host assigns semantic IDs to these deployment slots. A welding
slot carries the six values specified by ProcessKit's `WeldContract`; the device
transport does not interpret welding units or fault codes. Sequence zero means
absent in host snapshots, and duplicate or older samples must not replace newer
ones. Revision 12 is rejected rather than migrated. The current schema lock and
shared Rust/C++ frame vectors were regenerated for revision 13.
Protocol version 8 also carries steps per actuator unit, actuator rate limits,
direction setup ticks, source joint indices, transmission ratios and dual-drive
skew bounds, all derived by the host from the model and the deployment's
wiring. Nothing about them is compiled into firmware.
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

Protocol version 12 (one in-place revision, before any hardware release) has two
changes to `SESSION_BEGIN6`. `channel_stop_policy` is one byte per channel: 0
makes the channel safe on every stop, 1 keeps it through a commanded STOP or
ABORT, as the host runtime's `RK_CHANNEL_KEEP_ON_STOP` does. The configuration
digest below covers these bytes, so a board that disagrees about a stop policy
is refused. The second change replaces the compiled layout fingerprint with
identity and agreement checked while the session opens. `SESSION_BEGIN6.expected_controller`
names the board the configuration is for. `SESSION_ACK6.controller` is the
board's own unique id (on an STM32G4, its 96-bit unique-ID register padded to 16
bytes); a board reports it even when it refuses the session, so `robotd identify`
can read it by asking for the all-zero id, which no board accepts. A board
refuses a session whose expected controller is not its own. `config_digest` is
the 64-bit FNV-1a of the `SESSION_BEGIN6` payload after its `session` field,
computed by the device over the bytes it received. The host computes the same
over the bytes it sent and refuses the board when the digests differ, or when
the controller is not the expected one, the board has fewer channels than the
layout wires, or its step tick is not the one the deployment plans for. Each
refusal says which check failed on stderr; a wrong controller, digest, channel
count or step tick is `RK_ERROR_MODEL_MISMATCH`. Firmware is therefore per
board type: it carries the channel count, pins, tick rates and the unique id
the chip already has, and a configuration change never needs a reflash.

Deployment schema v5 implies RKD6 and omits `protocol`. Its `device.controller`
is the 32-hex-digit unique id of the board; the layout wires channels to the
model's actuators (see `robotkit.device.DeviceBinding`). The motor's full steps
and driver microstepping come from the machine model. The layout states direction
and uses the driver's microsteps from the model. Layout schema v1 rejects
driver settings on channels.
The step tick comes from `device.step_tick_hz`; the binding derives each channel's
steps per radian, ratio, rate ceiling and direction setup, and refuses a
stepper without a channel or a channel without a stepper. Pulse frequency is bounded
by both the board tick and the driver's maximum step input rate; a faster board clock
can idle between pulses. Earlier deployment
versions, which named a compiled `fingerprint`, are rejected. The POSIX serial
endpoint, virtual endpoint, PTY harness and two-joint Nucleo stub share this
protocol.
