# RobotKit architecture

RobotKit separates the logical robot API from the mechanism that produces a
robot's state. This is what lets the same world and behavior code work with a
remote physical robot, a local simulation, a replay source, or a future
hardware-in-the-loop adapter.

## Public object model

```text
RobotWorld
└── Robot[]
    ├── RemoteRobot
    │   └── RobotClient ── RobotProtocol ── robotd / physical process
    └── SimulatedRobot
        └── RobotRuntime ──┐
                            ▼
                        Simulation
```

`RobotWorld` owns the logical `Robot` collection. It does not know whether a
robot is remote or simulated. It routes `RobotCommand`, collects
`RobotSnapshot`, tracks lifecycle changes, and builds `WorldSnapshot` values.
The world has one application owner, recorded as a native thread token.
Non-owner calls that would touch its maps, adapters, or observers are rejected.
Adapter callbacks never mutate its maps or sequence directly: they enqueue
`RobotWorldEvent` values, and the owner applies them during `pump()`/snapshot
construction. The event queue is the cross-thread boundary; attach, detach,
submit, stop, and snapshot are owner operations.

Snapshots own their data. Robot and world maps are private, arrays are copied
into read-only views, and sensor frames are copied as well. Every observation
has both `sourceTimestampNs` (the robot, simulator, or sensor clock) and
`receivedTimestampNs` (the runtime/world receive clock). The old
`timestampNs` names remain read-only compatibility aliases only.

## Core contracts

These rules apply across the editable Haxeon model, compiled runtime,
simulation, protocol, and endpoint adapters. Backend identifiers and clock
values must not escape their boundary as if they were domain identity or a
shared time base.

### Identity

`RobotId`, `LinkId`, `JointId`, `SensorId`, and `FrameId` are semantic,
stable identities. Names are presentation values and may change without
changing identity. Runtime array indices, SceneKit occurrence IDs, and MuJoCo
IDs are temporary mappings owned by their backend; they must not be serialized
as model identity or retained as document references. Imports and compilation
must preserve semantic IDs while rebuilding those mappings.

Links, joints, sensors, and frames expose immutable string IDs, unique within each
entity kind in a robot model, and editable names. Existing constructors default
the ID to the initial name once; importers must supply the persisted ID.
Compilation rejects empty or duplicate IDs and attaches a copied
`RobotRuntimeIdentity` mapping to the blueprint. Native indices remain local
to that compiled revision. Sensor indices in this mapping identify model
slots, not SensorKit registrations. Manually constructed native blueprints
may omit the mapping. A `Frame` stores editable `link_T_frame` translation and
unit quaternion; a `Sensor.frame` references a frame in the same model. The
compiler freezes IDs, attachments, mounts, and sensor configuration into the
blueprint. Runtime samples map native sensor slots back to semantic IDs. Wire
messages and recordings retain sensor/frame/link IDs and mount transforms.
An unmounted sensor uses the root link's origin and ID. Models without sensor
declarations get the compatibility encoder/IMU/LiDAR set; manually constructed
blueprints without metadata retain the `base_link` fallback frame name.

### Frames and units

RobotKit uses a right-handed world with `+Z` up and `+X` forward. Lengths are
meters, angles are radians, and durations are seconds. Quaternions use
`x, y, z, w` component order. A transform `A_T_B` maps coordinates expressed
in frame `B` into frame `A`; composition follows
`world_T_sensor = world_T_link * link_T_sensor`. Matrix representations use
column-major storage and multiply column vectors, matching SceneKit. Adapters
for external conventions, including ROS, convert at their boundary and leave
the internal model unchanged.

### Time and command provenance

Keep source and receive clocks distinct. The API names below describe the
meaning of a timestamp; each command or observation carries only the fields
that apply to it:

| Field | Clock or meaning |
| --- | --- |
| `sourceTimestamp` | Clock of the robot, simulator, or sensor that produced the value |
| `receivedTimestamp` | Local monotonic clock when a runtime or world accepted it |
| `captureTimestamp` | Sensor clock at acquisition |
| `deliveryTimestamp` | Receiver clock when a sensor sample was delivered |
| `commandIssuedTimestamp` | Issuing client's monotonic clock |
| `commandDeadline` | Receiving endpoint's clock; checked only in that clock domain |
| `simulationTime` | Time owned and advanced by `Simulation` |

When an issuer and receiver use different clocks, a session must establish an
explicit mapping before a deadline can cross that boundary. Source timestamps
must not be reused as receive times or deadlines. Commands should carry an
identifier, monotonic sequence, and the snapshot revision they were derived
from so an endpoint can reject stale or superseded intent. Until those fields
are represented end to end, adapters must not infer snapshot provenance.
Native command `timestamp_ns` is issuer metadata, not an enforced deadline.
World command adapters and robotd reject nonzero absolute expiry values until
deadline enforcement and clock negotiation exist; they never silently compare
client and server clock epochs. Local `IntentBuffer` deadlines use the same
process monotonic clock and expire at (not after) the deadline.

Runtime receive time is stamped after sample validation using `steady_clock`;
source epoch zero is preserved. Missing receive timestamps are zero (unknown),
never copied from source time. `RobotWorld` stamps its snapshot publication in
the local clock and leaves aggregate source time zero: unrelated robot clocks
cannot be meaningfully combined using a maximum. Per-robot source times remain
available without reinterpretation.

### Ownership

The existing ownership matrix below is normative: `RobotWorld` owns attached
logical adapters, `Simulation` owns simulated execution and its clock, and
`robotd` owns the deployed runtime and endpoint. Adapters may translate values
at those boundaries, but may not transfer runtime, physics, or process
lifecycle ownership to another layer.

`RemoteRobot` owns a network client. `SimulatedRobot` is only an adapter over a
`RobotRuntime` supplied by an externally owned `Simulation`; it never creates,
steps, stops, or disposes that simulation.

The ownership matrix is deliberately boring:

| Object | Owns | Does not own |
| --- | --- | --- |
| `RobotWorld` | attached adapters and world event delivery | simulation runtimes or remote process lifetime |
| `Simulation` | SceneKit/SimKit state, clock, simulated runtimes and environment objects | `RobotWorld` adapters |
| `robotd` | one deployed `RobotRuntime`, endpoint, protocol sessions and control lease | fleet/world composition |
| `RobotClient`/`RemoteRobot` | its transport connection | the deployed runtime |

## Runtime versus simulation

`RobotRuntime` is the command mailbox and state publication boundary for one
robot. `RobotRuntimeCompiler` turns the editable `RobotModel` directly into a
`RobotRuntimeBlueprint`. Native layout details remain internal to runtime
creation, and `RobotRuntime.snapshot()` returns the runtime snapshot value
directly.

Compilation has two deliberate entry points. `RobotRuntimeCompiler.validate()`
returns every `RobotCompileDiagnostic` with a stable code, field path, and
human-readable message. `compile()` uses that same validation pass and throws a
`RobotCompileException` carrying the complete list, so an editor or import
pipeline can present all model errors in one report. Validation includes link
and joint identity, backend-supported joint kinds, limits and actuator values,
sensor metadata, and the link topology (one root, one parent per child, no
cycles, and no disconnected links).

The high-rate path is intentionally one boundary:

```text
command mailbox → validation → arbitration/intent → controllers
               → limits + safety → RobotEndpoint
               → sample → immutable RobotSnapshot
```

The runtime rejects stale command sequences, validates compiled joint and
actuator limits before endpoint application, latches endpoint/sample failures
as faults, and exposes explicit stop/reset-safety commands. `RobotEndpoint`
only has `apply()` and `sample()` plus the rollback hook needed by a shared
transactional simulation tick. Backends do not receive runtime ownership.

Position targets are retained as runtime intent and emitted on every owner
tick. When a command supplies `max_rate`, the runtime advances a deterministic
position reference by at most `max_rate × period` per tick before passing it to
the endpoint. A new position command therefore changes the common controller
reference path, not backend-specific interpolation. The loopback endpoint tracks
those setpoints exactly; physical endpoints and simulation consume the same
per-tick commands. A zero `max_rate` leaves interpolation to the endpoint.
Every sample is checked for valid shape and finite values, and observed joint
positions are checked against the compiled envelope. Stale/malformed samples,
limit violations, and endpoint failures latch a runtime fault and trigger a
best-effort emergency-stop command through the same endpoint boundary.

The first concrete physical backend is `SerialRobotEndpoint`: a fixed framed
POSIX command/state stream with non-blocking reads. It is intentionally a
specific endpoint, not a universal driver hierarchy. Disconnects and incomplete
state frames become runtime faults, which keeps stale sensor data visible to
the same safety boundary used by simulation.

`Simulation` is the shared ownership boundary for local physics. It owns the
SceneKit scene, SimKit world and host, physics resources, simulation clock,
world snapshot, and all simulation robot bindings:

```text
Simulation
├── SceneKit scene
├── SimKit world / host / clock
├── SimulationRobot (internal binding)
├── SimulationRobot (internal binding)
└── RobotRuntime handles
```

The internal `SimulationRobot` maps one runtime's joints and bodies into the
shared world. It is not a second public endpoint abstraction.

## One simulation tick

Applications advance a shared simulation with `Simulation.step(timestamp)`:

1. Every runtime mailbox is drained.
2. Every robot command is applied to its internal simulation binding.
3. All staged joint targets are submitted to the common physics host.
4. The host advances exactly once.
5. One immutable world snapshot is acquired.
6. Every runtime publishes its robot state from that snapshot.

There is intentionally no public per-runtime tick operation. Standalone
in-memory runtimes own a worker lifecycle through `start()` and `stop()`;
runtimes attached to a shared `Simulation` are externally driven and cannot be
started independently. This keeps the clock owner explicit and prevents a
robot from accidentally advancing only part of a multi-robot world.

While stopped, the simulation owner can reset the whole world, reset one robot,
teleport a robot, and spawn/remove/teleport environment objects. The editable
scene is therefore the source of initial/configuration state. After a running
host starts, physics owns the live state and runtime commands are the only
normal way to change robot motion. Sensor measurements use the same physics
snapshot as joint encoders. Sensor-frame IMU values are angular velocity followed
by specific force, `R^-1 * (dv/dt - gravity)`; gravity is `(0, 0, -9.81)`.
The mount-point velocity includes `omega × offset`, so an offset rotating IMU
measures tangential and centripetal acceleration rather than only link-origin
acceleration. Mount orientation rotates both IMU vectors and LiDAR rays.
The first sample after creation, reset, or teleport primes the derivative and
does not publish IMU. LiDAR casts 1–64 planar rays counterclockwise from +X,
with a configurable positive maximum range, against oriented boxes at their
physics poses. It excludes all own links and includes other robots and
environment objects. Mesh queries are not implemented. Defaults are 8 rays,
10 m, every shared tick, and no noise. `updateRate` is Hz (zero = every tick),
quantized to available physics ticks without interpolating invented samples.
Each sensor has its own sequence and source/receive timestamps, held unchanged
between acquisitions. Seeded Gaussian noise is optional per sensor; LiDAR
clips noisy distances to `[0, maxRange]`. Reset restarts the schedule and PRNG.

Native ABI version 3 carries up to 8 sensor configurations and bounded sample
payloads of up to 64 values each; a zero sample sequence means no acquisition.
Both `SimulatedRobot` and robotd project only valid measurements through the
same immutable sensor frames; endpoints without those measurements do not
invent them. Rebuild native consumers for the extended structs.

The default backend is still the deterministic SimKit test backend. Build with
`-DNKSIM_BUILD_MUJOCO=ON` and select `rk_simulation_desc.backend = 1` (Haxe
`new Simulation(dt, substeps, 1)`) for MuJoCo; requesting it in an unconfigured
build returns `RK_ERROR_UNSUPPORTED`, never a silent fallback. MuJoCo tests
exercise articulated IMU/offset acceleration, moving-object LiDAR occlusion,
collision response, angular limits in radians, world-oriented body velocities,
and deterministic replay. Rebuilds preserve articulated rest transforms;
reset clears native joint state and targets. Unsupported independent teleports
of constrained links fail without replacing the cached body state.
Adjacent joint-connected bodies are excluded from
self contact, including static-root pairs. The robot geometry remains the
current simple box representation, not an imported CAD collision model.

## Deployment boundary

`robotd` remains one robot. It assigns every connection a unique session ID,
gives at most one controller a control lease, rejects commands from observers
or stale sessions, and streams state/sensor/fault messages to read-only
observers. Reconnection creates a new session and command sequence domain;
world composition and multi-robot discovery remain in `RobotWorld`.

Recording stores commands, snapshots, sensor-bearing world snapshots, faults,
and world events. `ReplayRobot` and `WorldBehaviorRunner` consume the same
public boundary, so behavior code can be exercised against simulated, remote,
or recorded state without backend conditionals. `worldd` is only a headless
composition of `RobotWorld` plus `Simulation`; it does not create a parallel
domain model.

World-level behaviors leave command deadlines unset. Nonzero world-command
deadlines are currently rejected rather than ignored or translated without a
clock mapping. Local runtime behaviors may use bounded `IntentBuffer` expiry;
source timestamps are never silently reused as receive or command time.

## Ownership and shutdown

The embedding application owns `Simulation` and creates runtimes from it. A
`RobotWorld` may own `SimulatedRobot` adapters, but closing the world only
closes those adapters; it does not stop or destroy the shared simulation. The
recommended shutdown order is:

```text
RobotWorld → SimulatedRobot adapters → Simulation
```

Remote adapters may be attached to the same world and have their own network
lifecycle. Their connection state does not affect the simulation clock.
