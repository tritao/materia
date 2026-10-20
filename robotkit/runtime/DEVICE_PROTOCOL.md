# RobotKit scheduled device protocol (generation 6, wire revision 16)

The protocol name is **RKD generation 6, wire revision 16**. The four-byte sync
marker `RKD6` identifies the scheduled-execution generation, not the wire revision.
`PROTOCOL_VERSION` identifies the wire revision; peers must agree on its exact value
(currently 16). A receiver never infers a revision from the marker or accepts an older
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
session record (5,013 bytes), so the frame maximum is 5,025 bytes.
A receiver rejects unknown types, wrong lengths, a nonzero reserved byte,
CRC mismatch, or malformed session, segment and event records.

| Type | Message | Direction | Payload |
| ---: | --- | --- | --- |
| 1 | `SESSION_BEGIN6` | host → device | `SessionBegin6`: expected controller, actuators, channels, inputs |
| 2 | `SESSION_ACK6` | device → host | `SessionAck6`: own id, status, tick rates, capacities, profile, digest |
| 3, 4 | `TIME_SYNC_REQUEST`, `TIME_SYNC_REPLY` | both | clock samples |
| 5 | `QUEUE_BEGIN` | host → device | `QueueBegin6`: revision, replacement boundary, expected state |
| 6 | `SEGMENT` | host → device | `Segment6Header` + one `Segment6Coefficients` per actuator |
| 7 | `COMMIT` | host → device | `Commit6`: committed through tick |
| 8–12 | `HOLD`, `RESUME`, `ABORT`, `STOP`, `EMERGENCY_STOP` | host → device | none |
| 13 | `RESET_SAFETY` | host → device | none (see [Recovery](#recovery-after-a-stop)) |
| 14 | `QUEUE_STATUS` | device → host | `QueueStatus6` |
| 15 | `STATE6` | device → host | `State6Header` + `ActuatorState6` per actuator + `InputState6` per input |
| 16 | `EVENT` | host → device | `Event6`: a channel value at a path tick |
| 17 | `SENSOR6` | device → host | `Sensor6Header` + `Sensor6Value` rows |
| 18 | `CONTROLLER_OUTPUT` | host → device | `ControllerOutput6` |
| 21–24 | `HOMING_SCOPE`, `HOMING_SIDE`, `HOMING_CONTROL_ACK`, `HOMING_COUNTER_BATCH` | see [Homing](#inputs-and-homing) | |

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
transport does not interpret welding units or fault codes. A heater slot carries five
values (`robotkit.tool.HeaterSensor`): temperature in degrees Celsius, the target the board holds,
duty, at target, and a fault code; the host sets a target on the heater's analog channel and the
board's controller (`device_protocol/boards/heater.rs`: PID, a runaway watchdog, a safe state) does
the rest, so a host that stalls does not leave a heater on. No wire revision is needed. Sequence zero means
absent in host snapshots, and duplicate or older samples must not replace newer
ones. Revision 12 is rejected rather than migrated. The current schema lock and
shared Rust/C++ frame vectors were regenerated for revision 13.

Wire revision 14 replaces `SESSION_BEGIN6`'s `max_rate` (`f32[64]`, a rate the
device turned into ticks) with `min_step_ticks` (`u32[64]`): each active
actuator's least whole number of step-tick periods between steps, at least one.
The host's device binding chooses it once from the driver's step-rate ceiling
and the motor's speed (`robotkit.runtime.StepTicks`), and plans at the rate it
reaches; the device only enforces it. As a rate, the tick ceiling could round
under itself in f32 and cost a whole tick, halving the motor's speed. Revision
13 is rejected rather than migrated.
Wire revision 16 carries an actuator's position as an integer and a small float. A `Segment6Coefficients`
row gains `base_steps` (`i64`, the position's whole steps), and `c0` to `c5` are relative to
`base_steps / steps_per_unit`; the device adds the base in 64 bits. An extruder is a continuous joint whose angle
grows the whole print, and one f32 cannot name a step of it past a few thousand radians (4,096 rad is 5 m of
filament): half the float's step there is more than a tenth of a motor step, so a long print was refused
or stepped with jitter. The absolute positions the wire carries elsewhere are 64-bit floats too: `QueueBegin6`'s
`expected_position` and `ActuatorState6.position`. The device's scheduled core holds positions as `f64`
(`Board::position_target` takes one), and the step generator multiplies them by steps per unit in 64 bits.
Velocities, accelerations and the polynomial's coefficients stay `f32`. Revisions 14 and 15 are rejected rather than migrated.

The same revision adds `CONTROLLER_OUTPUT6` (message 18). Its fixed record
carries the session, a strictly increasing per-session output sequence, a
declared channel, kind and canonical digital or float analog value. These
controller-owned outputs execute when the board receives them, independently
of the arm's committed path queue. Scheduled events retain their path clock and HOLD policies. Controller writes
keep their value through an arm HOLD and supersede any obsolete scheduled
HOLD/resume latch, including a write of the same current value. STOP safety
continues to use the separately declared channel stop policy.
Digital/analog kinds must match the session declaration; faults and emergency
stops accept safe values only. A controlled arm stop may preserve vacuum and
accept independent process controls, which still require the owner link lease.
The runtime records a controller output only after the endpoint accepts its
transmission. Measured device feedback remains the authority for execution,
not that host record. Revisions 14 and 15 are rejected rather than migrated. The
current schema lock and shared fixtures are regenerated; reproduce the fixtures
with `cargo run --manifest-path robotkit/device_protocol/Cargo.toml --example
frame_vectors`. Native and Rust framing validate the same record.


Wire revision 8 also carries steps per actuator unit, actuator rate limits,
direction setup ticks, source joint indices, transmission ratios and dual-drive
skew bounds, all derived by the host from the model and the deployment's
wiring. Nothing about them is compiled into firmware.
The host converts joint polynomials to actuator polynomials before encoding and
rejects motion faster than either the authored actuator rate or one step per
device step tick. The no_std device generator uses the configured direction
setup and minimum step interval, and checks dual-drive skew from step feedback.
Wire revision 9 declares up to 32 channels in the session, including stable
IDs, kinds and safe values. An `EVENT` carries its queue revision, plan ID,
device path tick, channel index, typed value and HOLD policy. The host maps
each `TimedEvent` from plan-relative path time with the same frozen clock
mapping as its trajectory segments. The device fires only committed events as
its path clock crosses their ticks. HOLD makes configured channels safe,
RESUME restores `RestoreOnResume` values, and STOP, ABORT or a fault discards
future events and sets every channel to its declared safe value, except that a
commanded STOP or ABORT leaves a channel whose stop policy is keep-on-stop as it
is (wire revision 12), so a vacuum holding a part does not drop it when the
arm is stopped. An emergency stop, link loss or fault still makes every channel
safe. Stopping the motion never touches the channels: they are the events'. Replacement discards
events at or after the replacement boundary. The virtual event log records
scheduled path ticks, applied path ticks and device ticks.

No RKD6 hardware has shipped. Before the first hardware release, an in-place
schema revision is permitted with a `PROTOCOL_VERSION` bump and regenerated
Rust and C++ codecs, lock, and shared vectors. There is no version negotiation:
peers require the exact wire revision. After the first hardware release, a
change to the wire format is a new revision that every board must be reflashed
for, or a new generation.

| Quantity | RKD6 bound | Source |
| --- | ---: | --- |
| Maximum actuators | 64 | schema |
| Maximum frame | 5,025 bytes | 8 + 5,013 (`SessionBegin6`) + 4 |
| Maximum segment frame | 2,160 bytes | 8 + 36 + 64 × 33 + 4 |
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

Wire revision 10 adds `SESSION_ACK6.profile`. Full devices report profile 1,
support degree up to their declared maximum, queue segments and generate steps.
Minimal devices report profile 2, maximum degree 1, and a small queue (the
reference device has eight slots). They output position setpoints and generate
no steps. The host lowers source segments at the owner period, checks the
converted f32 path against the original trajectory at step-tick resolution,
and revalidates the lowered path. It accounts for frames in flight when
streaming into a small queue. Qualification reports minimum baud, queue depth
and period before construction succeeds.

Wire revision 11 adds two `QUEUE_STATUS6` fields. `received_until_ticks` is
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

Wire revision 12 (one in-place revision, before any hardware release) has two
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

Deployment schema v6 implies RKD6 and omits `protocol`; v6 added the robot's
`profile` (its roles, kept separate from the mechanical model). Its `device.controller`
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

Explicit STOP and ABORT commands reapply the declared channel safety policy,
including outputs independently reactivated after an earlier arm stop. Ordinary
stopped-clock updates preserve those independently controlled outputs. A normal
stop or abort cannot clear an existing fault or emergency-stop reason. Host
output writes transmit even when the desired host value is unchanged, because
the device may have made it safe independently; unchanged values do not add
another host output-change record.

The host's optional segment merge uses `trajectory_core` directly. Its position
budget is one tenth of the finest actuator step on each joint; its velocity
budget is one percent of the allowed actuator step rate (including
`min_step_ticks`), and its acceleration budget is one percent of the claimed
joint acceleration. Original follower coefficients are checked before merging;
merged coefficients retain a check in normalized segment time before projection.

Merge windows derive from the actual transport, including the virtual UART.
For frame size `B` bytes, baud `b`, service rate `r`, queue capacity `Q`, and
refill delay `D`, the service time is `s = max(10 B / b, 1/r)` seconds.
The threshold is `ceil((Q s + D)/(Q-1))`, in nanoseconds: refill `Q` rows while
`Q-1` queued rows remain behind the active row. A replacement may span twice
that threshold so two adjacent short rows can fit. The endpoint derives `r`
from whole segment frames per owner period after reserving a commit frame;
`D` uses the same processing, commit and backlog margins as link qualification.
A missing link description disables optional merging. These windows guide
consolidation; the existing queue-delivery qualification still admits or refuses
the actual sequence.

Compiler failures have typed reasons. A conversion or limit failure involving a
merged row (or its seam) may retry the original rows. Clock failures, invalid
source data, and failures confined to untouched rows return directly. The result
records whether a retry occurred and its cause. None of this changes wire records.

## Inputs and homing

`SESSION_BEGIN6` declares up to 64 switch inputs: `input_actuator` names the
actuator each input belongs to and `input_active_high` its polarity. The device
captures every edge in its step-clock domain (`device_protocol/src/input_capture.rs`)
and `STATE6` appends one `InputState6` row per input after the actuator rows:
closing and opening counts, and the step count and device tick of the last
captured edge. Limit-switch faults are the host's: its runtime latches
`RK_FAULT_LIMIT_SWITCH` from these observations; the device only captures them.

Homing a dual-drive axis (two motors on one joint, such as a gantry's two Y
sides) uses messages 21 to 24, each carrying the session, a strictly increasing
sequence and a homing scope:

- `HOMING_SCOPE` begins or ends a homing scope over a pair of actuators with its
  skew bound, or stops homing in a controlled way while keeping the scope.
- `HOMING_SIDE` holds one side while the other keeps moving, so each side can
  stop on its own switch and the axis is squared.
- `HOMING_COUNTER_BATCH` shifts both sides' origins at once, in actuator units;
  physical step counts do not change.
- `HOMING_CONTROL_ACK` is the device's answer: accepted or rejected.

## Recovery after a stop

After a commanded `STOP` has brought the machine to rest, the device accepts a
new queue revision and motion continues in the same session. After `ABORT`,
`EMERGENCY_STOP`, link loss, underflow, dual-drive skew or a process fault, the
scheduled core refuses every new queue (`QueueError::Stopped`); recovery is a
new session. `RESET_SAFETY` (13) is defined and sent by the host, which clears
its own queue state, but the reference devices do not act on it: the virtual
device answers it as unhandled and the Nucleo board ignores it. Whether a device
should clear a latched stop on `RESET_SAFETY` within a session, and under what
conditions, is an open decision.

