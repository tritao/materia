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
    ├── SerialRobot
    │   └── RuntimeRobotAdapter ── RobotRuntime ── DeviceSerialEndpoint
    └── SimulatedRobot
        └── RuntimeRobotAdapter ── RobotRuntime ──┐
                                                   ▼
                                               Simulation
```

`RobotWorld` owns the logical `Robot` collection. It does not know whether a
robot is remote, serial-connected, or simulated. It routes `RobotCommand`, collects
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
changing identity. Runtime array indices, SceneKit node IDs, and MuJoCo
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
steps, stops, or disposes that simulation. `SerialRobot` compiles a model,
creates and starts a serial-backed `RobotRuntime`, and disposes it on close.
`robotd --server --serial=DEVICE` owns the same serial-backed runtime behind
the existing remote protocol; without `--serial`, the server uses Simulation.

The ownership matrix is deliberately boring:

| Object | Owns | Does not own |
| --- | --- | --- |
| `RobotWorld` | attached adapters and world event delivery | simulation runtimes or remote process lifetime |
| `Simulation` | SceneKit/SimKit state, clock, simulated runtimes and environment objects | `RobotWorld` adapters |
| `SerialRobot` | serial runtime and open device descriptor | physical device firmware or controller state |
| `robotd` | one deployed `RobotRuntime`, endpoint, protocol sessions and control lease | fleet/world composition |
| `RobotClient`/`RemoteRobot` | its transport connection | the deployed runtime |

## Runtime versus simulation

`RobotRuntime` is the command mailbox and state publication boundary for one
robot. `RobotRuntimeCompiler` turns the editable `RobotModel` directly into a
`RobotRuntimeBlueprint`. Native layout details remain internal to runtime
creation, and `RobotRuntime.snapshot()` returns the runtime snapshot value
directly.

`RobotModelCodec` stores the complete semantic robot definition as a versioned
JSON artifact shared by editor, simulation, and `robotd`. Deployment files refer
to that artifact and keep `DeviceLayout` as the separate ordered mapping from
model joint IDs to RKD5 channels. The RKD5 fingerprint covers the exact device
layout and wire schema lock, not mutable semantic model fields.

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

The physical backend is `DeviceSerialEndpoint`: a framed POSIX command/state
stream with bounded payloads and CRC-32. It waits for fresh control state and
faults on a disconnected or silent device. Sensor traffic stays outside its
control acknowledgement path. The device protocol document defines the
firmware contract.

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

## Joint command batches

`RobotCommand.JointTargets` is the transport-neutral command boundary. Each
`JointTarget` names one joint, a position/velocity/effort interpretation, and
one SI target value. A command can contain several different modes at once.
Joint indices must be unique within a batch, and adapters validate the whole
batch before queuing it. `RobotWorld` routes the command without expanding it
into per-joint submissions.

The remote protocol carries a batch in one `JointTargets` frame. `robotd`
validates its session, sequence, deadline, joint indices, and target modes, then
submits one native `rk_robot_command` to the runtime mailbox. Simulated robots
use that same mailbox batch directly; replay records newly generated batches
without changing their source observations. The runtime owns position bounds,
velocity-rate bounds, and effort limits. This boundary intentionally contains
joint targets rather than mobile-base or forklift-specific commands.

## Mobile kinematics

`robotkit.mobile.MobileBase` composes a `Robot`, `DriveModel`, motion limits,
and optional footprint. It is a view over the existing robot contract, not a
new robot subclass. A drive model translates one `Twist2` into a complete joint
target batch: differential drive emits both wheel rates, while Ackermann drive
emits steering position and drive-wheel rate together. Runtime joint limits and
safety remain authoritative below this application-level mapping.

`Pose2` and `Twist2` describe planar geometry and body velocity. The initial
wheel odometry utility consumes immutable snapshots and uses source clock IDs
to avoid integrating across a reboot or clock reset. Its pose is derived state;
it is not inserted into `RobotSnapshot`. The localization package wraps this
odometry with explicit frames, covariance, and source/receive clock metadata.

`robotkit.localization.LocalizationState` keeps that derived pose outside the
robot snapshot boundary. It names the reference and body frames, carries a
planar covariance and quality, and preserves source/receive timestamps and
clock IDs from its input observation. `WheelOdometryLocalization` supplies an
`odom` to `base` estimate; `SimulationTruthLocalization` reads the simulation's
owned base pose and supplies a `map` to `base` estimate without changing the
simulation state.

`robotkit.navigation.Navigation` follows a frame-tagged `Path` using a
pure-pursuit controller at application update frequency. It obtains a fresh
`LocalizationState`, computes a body twist, and sends that through `MobileBase`.
`Trajectory` stores time-stamped references for consumers that need them; the
first controller does not require a trajectory planner. Native `RobotRuntime`
continues to own joint limits and hard safety enforcement.

`robotkit.navigation.MotionGuard` is an application-level command filter between
navigation and `MobileBase`. It transforms reference-frame obstacles into the
localized body frame, checks a forward footprint corridor, and uses a stopping
envelope to pass, scale, or zero the requested twist. Obstacles in unresolved
frames block motion until perception transforms them. The guard is software
collision avoidance and does not replace native or hardware safety.

`OccupancyGrid2` provides framed free/occupied/unknown cells, while `Costmap2`
adds dynamic obstacle disks, conservative unknown handling, circular footprint
inflation, and a soft proximity cost. `AStarPlanner` applies deterministic
8-connected A* without diagonal corner cutting and converts its route into a
`Path` for `Navigation`. `Navigator` owns a framed goal, refreshes dynamic
obstacles, and checks the remaining path against the costmap before each control
step. It replans blocked routes and stops/retries if no traversable path exists.
`Navigation.follow(path)` remains the lower-level tracking API.

`robotkit.material.Forks` is another explicit view over `Robot`. Its named axis
configuration is resolved against `RobotDescription` once, then each lift,
tilt, or spread request is validated and submitted as one `JointTargets` batch.
Payload and load-limit values describe application policy and current knowledge;
they do not replace enforcement in the native runtime.

Perception values carry their own source and receive clock identities. The
initial LiDAR adapter maps valid range returns into obstacles in the sensor's
declared frame; pallet and docking-target values can be produced by higher
level perception algorithms. Safety and battery state are descriptive service
boundaries. Safety policy can report restrictions and a stopping envelope, but
hard stops and joint limits remain enforced in `RobotRuntime`.

## Skills

`robotkit.skill` composes navigation, perception, mechanism, and power views
without adding a robot subclass or capability registry. A skill is advanced by
the application loop through `start()`, `update(snapshot, dt)`, and
`cancel()`; it exposes terminal `status()` and `result()` values. The first
compositions are `GoTo`, `Dock`, `PickPallet`, `PlacePallet`, and `Charge`.
`SkillRunner` provides a one-skill-at-a-time robot-local update loop. It forwards
observations and elapsed time, rejects overlapping starts, handles cancellation,
and retains the terminal status and result; mission sequencing remains above
RobotKit.
Pick and place skills issue atomic fork batches, then wait for observed load
state changes. Charging waits for the configured battery fraction after the
docking approach succeeds. These lifecycle operations coordinate application
behavior; native `RobotRuntime` remains responsible for actuator limits and
hard safety.

The same skill objects work above `RemoteRobot`, `SimulatedRobot`, and
`ReplayRobot`. The forklift regression records the simulated approach, pickup,
second drive, placement, and charging sequence to MCAP, then replays the skills
against those observations and checks every emitted joint target batch.
Recording retains robot observations and commands so application skills can be
replayed against the same observation stream.

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
does not publish IMU. LiDAR casts 1–64 planar rays from the configured first
bearing across its angular coverage in the sensor frame. A full revolution does
not duplicate its endpoint; partial scans include both coverage boundaries. It
uses a configurable positive maximum range against oriented boxes at their
physics poses. It excludes all own links and includes other robots and
environment objects. Mesh queries are not implemented. Defaults are 8 rays,
10 m, a full revolution starting at +X, every shared tick, and no noise.
`updateRate` is Hz (zero = every tick),
quantized to available physics ticks without interpolating invented samples.
Each sensor has its own sequence and source/receive timestamps, held unchanged
between acquisitions. Seeded Gaussian noise is optional per sensor; LiDAR
clips noisy distances to `[0, maxRange]`. Reset restarts the schedule and PRNG.

Native ABI version 4 carries up to 8 sensor configurations and bounded sample
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
observers. A controller's `Welcome` advertises a three-second lease timeout;
`RobotClient` renews it with `ControlHeartbeat` every timeout/3. `robotd`
measures heartbeat arrival with its own monotonic clock, so the client and host
do not need synchronized clocks. Expiry emergency-stops the runtime, releases
the owner, and closes that connection. A replacement controller receives a
new session and must explicitly reset safety before motion can resume. World
composition and multi-robot discovery remain in `RobotWorld`.

Recording stores commands, snapshots, sensor-bearing world snapshots, faults,
and world events. `ReplayRobot` and `WorldBehaviorRunner` consume the same
public boundary, so behavior code can be exercised against simulated, remote,
or recorded state without backend conditionals. `worldd` is only a headless
composition of `RobotWorld` plus `Simulation`; it does not create a parallel
domain model.

Persistent recording is an adapter below the world boundary. Haxe owns the
versioned RobotKit payload contract; a small C ABI owns a bounded queue and the
MCAP reader/writer. Neither MCAP headers nor MCAP concepts appear in `Robot`,
`RobotWorld`, behavior contexts, or runtime observation contracts. The writer
uses one channel/schema per event kind and no compression. File order and the
recording ordinal—not unrelated source clocks—define deterministic replay.

The current format uses MCAP log time for the independent wall-clock recording
timestamp and publish time for the full-width event ordinal. Reader cursors are
incremental and schema-validating. Writer queues are bounded by payload bytes,
in-memory retention is optional, and a companion terminal-status record keeps
write failures and drop counts visible across process restarts.

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
