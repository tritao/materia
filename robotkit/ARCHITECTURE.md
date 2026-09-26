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

`HolonomicDrive` (M9) adds a third `DriveModel`: an omnidirectional ("kiwi")
base with three wheels at 120-degree intervals, each rolling tangentially.
`Twist2` now carries forward speed, body +Y lateral speed, and yaw rate.
`Navigator` and `GoTo` continue to produce zero lateral speed, while direct
callers can use the full holonomic map. `DriveModel.createOdometry()` is typed to return
`Null<DifferentialOdometry>` specifically (a pre-existing, differential-drive-
specific signature), so `HolonomicDrive.createOdometry()` returns `null`
(matching `AckermannDrive`) and `robotkit.mobile.HolonomicOdometry` /
`robotkit.localization.HolonomicOdometryLocalization` take the three wheel
joints and geometry directly instead, mirroring `WheelOdometryLocalization`.
The three wheel angles being 120 degrees apart make the inverse map decouple
exactly, so forward and lateral distance are weighted wheel sums and heading
change is their average divided by the base radius. `runtime.HolonomicDrivePlant` mirrors `DifferentialDrivePlant`:
it couples the three wheel joints to the base natively
(`Simulation.setOmniDrive`), and every tick decodes the full planar body
twist -- forward, lateral, and yaw rate -- from the wheel targets the robot
actually applied, so stops, rate clamps, and wheels driven directly all move
the chassis exactly as they would the real base. Differential and Ackermann
models reject non-zero lateral commands explicitly.

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
`Path` for `Navigation`. The costmap separates lethal cells (a robot centred
there would overlap an obstacle) from its blocked discretization margin, so a
start inside that margin plans an escape through non-lethal cells instead of
failing; otherwise a follower that cut a corner into the margin would stop and
never be able to replan. `Navigator` owns a framed goal, refreshes dynamic
obstacles, and checks the path ahead of the robot against the costmap before
each control step (a leading escape segment only against lethal cells). It
replans blocked routes and stops/retries if no traversable path exists.
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
compositions are `GoTo`, `FollowPath`, `Dock`, `PickPallet`, `PlacePallet`, and
`Charge`. `GoTo` owns goal-level planning and replanning through `Navigator`;
`FollowPath` tracks an explicitly supplied path and speed limits.
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
Direct parent/child pairs and pairs whose authored rest-pose geometries
overlap are excluded from self contact, including static-root pairs. Other
link pairs remain collision-enabled, so a folded arm can contact its own base.
The robot geometry remains the current simple box representation, not an
imported CAD collision model.

### Link rest poses (M8.5, F1)

`Simulation::add_robot` walks the compiled joint tree from the root at
`q = 0` and sets each link's scene node to its actual rest pose
(`world_T_child = world_T_parent . T(parent_frame_position, parent_frame_rotation)
. T(child_frame_position, child_frame_rotation)^-1`, the same composition
`robotkit.manipulation.KinematicChain`'s FK uses at zero joint values) before
creating that link's native body — not the identity-rotation placeholder
transform every link previously shared. `nksim_joint_desc`/`BackendJointDesc`
carry the joint frame's orientation relative to each body,
`rotation_a`/`rotation_b` (xyzw), appended after the original fields so an
old struct prefix stays valid (`nk_init_options`'s convention); a struct_size
or an all-zero (unnormalizable) value defaults to identity. The MuJoCo
backend places a non-fixed joint entirely from the child side —
`anchor_b`/`rotation_b` — and independently checks that `anchor_a`/`rotation_a`
describe the same physical pivot against both bodies' rest poses, refusing
(`NKSIM_ERROR_INVALID_STATE`) a joint description whose two sides disagree,
rather than silently trusting one side as before.

A regression surfaced while validating this fix: `robotkit_mujoco_tests`
(`robotkit/runtime/tests/mujoco.cpp`) already failed on the pre-M8.5 `HEAD`
(confirmed by stashing these changes and re-running it) — a single-hinge
model's IMU gyro and its joint's reported velocity disagree by a large,
non-shrinking margin from the very first tick. That model's root link is
`NKSIM_MOTION_KINEMATIC`, which MuJoCo represented as an ordinary
mass-bearing free joint pinned back to the scene node only once per outer
`step()` (via `refresh_kinematic_bodies`), not once per physics substep; the
root could pick up spurious free-joint velocity within a step's substeps
that leaked into a child's world angular velocity without appearing in the
child's own hinge `qvel`. This was a real bug, but a kinematic-root/substep-
timing issue independent of F1-F4's root causes (rest poses, actuator gains,
self-collision, and the default backend's kinematic placement); left unfixed
and reported here in M8.5 rather than silently expanding that milestone's
scope, and fixed ahead of M9 (below) since M9's holonomic base is exactly
this "kinematic root" case.

### MuJoCo kinematic-root velocity leak (pre-M9 fix)

`MujocoBackend::configure_body` gave every non-`STATIC` root body — both
`KINEMATIC` and `DYNAMIC` — a mass-bearing MuJoCo free joint. A `KINEMATIC`
body (a robot's own root/base link, or any body driven purely by
`World::refresh_kinematic_bodies` from its scene node) is never meant to be
an independent dynamical variable: it is externally scripted, exactly like a
`STATIC` body, just repositioned over time instead of fixed forever. Giving
it a free joint let real MuJoCo dynamics act on it between the once-per-outer-
step re-pin, so a driven child's reaction torque (through the shared mass
matrix — ordinary momentum coupling, not a separate bug) gave the
"kinematic" root real, nonzero velocity within a step's substeps, which then
leaked into a child's world-frame velocity reading (what an IMU measures)
without ever showing up in that child's own joint `qvel`. The fix treats
`KINEMATIC` exactly like `STATIC` in `configure_body`: no mass/inertia, no
free joint. A `KINEMATIC` body already had no dof-bearing joint's-worth of
special handling needed in `body_set_state` — the existing "no incoming
joint, no free joint" branch (previously reached only by `STATIC` bodies)
already sets `body_pos`/`body_quat` directly from the given state before
`mj_forward`, which is exactly the right zero-dof, externally-driven
placement for a `KINEMATIC` root too, so no other code path changed.
`kinematic_root_child_velocity_matches_joint_across_substeps`
(`simkit/sim_mujoco/tests/mujoco.cpp`) reproduces the leak directly (an
effort-mode torque on a hinge child of a `KINEMATIC` root, so the
reproduction is independent of the actuator's own mass-matrix addressing —
see below) and confirms the fix; `robotkit_mujoco_tests`'
IMU-vs-joint-velocity assertion now passes along with the rest of that
regression suite (12/12).

A second, previously undocumented native bug surfaced while writing this
fix's test with a position-mode target instead of effort mode: `data->M`
stores MuJoCo's mass matrix in a *sparse, per-dof-row* format where
`dof_Madr[dof]` is the address of the **start** of dof `dof`'s row (that
dof's own ancestor chain, then finally its own diagonal as the row's *last*
entry — see `mj_mulM`'s use of `dof_Madr[j+1] - dof_Madr[j]` as a row length
in `engine_derivative.c`), not the diagonal entry itself. F2's controller
(`m_ii = data->M[model->dof_Madr[dof]]`) is therefore only correct for a dof
with *no* ancestor dofs (row length 1) — true for every F2 test fixture
(a single joint directly on a `STATIC` base) but false for any joint with a
movable ancestor: a `KINEMATIC` base (M9's holonomic drive) or the second and
later joints of any multi-joint arm. With an ancestor present, F2's `m_ii`
silently reads an off-diagonal coupling term instead — often a tiny or zero
value — so the position/velocity controller could apply near-zero torque and
never move the joint at all. This is fixed together with F2's other known
limitation (no cross-joint compensation) by replacing the per-dof diagonal
lookup with full computed-torque control; see below.

### MuJoCo full computed-torque control (pre-M9 fix)

`MujocoBackend::apply_joint_targets` now computes `tau = M * qacc_desired +
qfrc_bias` via `mj_mulM` over the *whole* system's degrees of freedom,
replacing F2's single-dof diagonal lookup. For every position/velocity-mode
joint it fills one `qacc_desired` entry at that joint's own dof (the same
PD/velocity-error formula F2 used, just as a desired acceleration instead of
a pre-scaled torque) and leaves every other dof's entry zero, then calls
`mj_mulM(model, data, m_qacc, qacc_desired)` once per substep; each actuated
joint's torque is `m_qacc[dof] + qfrc_bias[dof]`, clamped to `max_force` as
before. Effort-mode joints are unchanged (`torque = target` directly, no
mass matrix involved). `mj_mulM` applies the *full* sparse mass matrix as an
operator (`M * vec`), so a nonzero desired acceleration at one dof correctly
distributes torque to every dof coupled to it through the articulated
system's true inertia — this is what "computed torque control" means, and
it fixes both known problems with the diagonal-only approach in one change:
it no longer depends on `dof_Madr[dof]` happening to address a diagonal
entry (see the kinematic-root section above), and it has real cross-joint
compensation, so the steady-state coupling error the M8.5 cross-backend
acceptance test noted (2-4mm at larger lever arms, ~0.6-0.7mm at the test's
own centimeter scale) is no longer an expected limitation of this
controller — `cross_backend_link_poses_agree_with_fk` still passes at its
existing tolerance and centimeter-scale anchors with room to spare. `ωn`,
`ζ`, and `kv` keep F2's defaults; only how their result reaches `ctrl`
changed.

### MuJoCo joint actuation (M8.5, F2, superseded above)

Each non-fixed joint has exactly one MuJoCo motor actuator (`mjs_setToMotor`),
not one each for position/velocity/effort: a MuJoCo position actuator's bias
(`-kp*q - kv*qdot`) applies even at `ctrl = 0`, so an idle position actuator
previously dragged a velocity- or effort-commanded joint back toward `q = 0`
and stalled it (and, symmetrically, an idle velocity actuator added damping
to a position-commanded joint). `MujocoBackend::apply_joint_targets` computes
the actual torque every substep instead:
`position: m_ii*(ωn²*(q*-q) - 2ζωn*qdot) + bias`,
`velocity: m_ii*kv*(qdot*-qdot) + bias`, `effort: target`, clamped to
`max_force` when positive. `m_ii` is the joint's own diagonal of MuJoCo's
sparse mass matrix (`data->M[model->dof_Madr[dof]]`), and `bias` is
`data->qfrc_bias` (gravity/Coriolis compensation), so the response no longer
depends on a fixed gain fighting a link's actual mass — a heavier arm no
longer sags under a fixed `kp`, and the same gains produce a comparable
response regardless of the joint's inertia. Defaults are `ωn = 2π*10 rad/s`,
`ζ = 1`, `kv = 50 s⁻¹`.

### MuJoCo self-collision (M8.5, F4)

After F1's rest-pose fix, every link sits at its real offset, so two
non-adjacent links of the same robot can genuinely overlap by construction
(a folded arm, for instance). `MujocoBackend::rebuild` now excludes only
direct parent/child pairs and pairs whose rest-pose geometries overlap. The
box path uses an oriented-box separating-axis test; sphere/capsule pairs use
their conservative rest-pose bounds. A folded non-adjacent link therefore
stops against the base instead of passing through it. `RobotRuntimeBlueprint`
has `selfCollision`, enabled by default; setting it false assigns the robot
to a collision category that still contacts ordinary environment geometry
while disabling contacts between its own links. This opt-out is for models
whose simple box approximation is too coarse.

### Default backend kinematic link placement (M8.5, F4)

The default (test) `PhysicsBackend` used to track a joint's position as a
bare number: a joint-connected `DYNAMIC` body never moved with it and
instead free-fell under gravity like an unconnected body.
`TestPhysicsBackend::recompute_articulated_poses` now recomputes every
joint-connected `DYNAMIC` body's world pose from its parent and current
joint position, in topological order:
`world_T_child = world_T_parent . T(anchor_a, rotation_a) . M(q) . T(anchor_b, rotation_b)^-1`
(`M(q)` built from the joint-frame axis recovered from `axis_a`/`rotation_a`,
the same composition `KinematicChain`'s FK and F1's rest-pose fix use). A
joint-connected `DYNAMIC` body no longer accumulates gravity/force in
`step()`; its linear/angular velocity is instead set by finite difference
from the recompute so IMU-style consumers on arm links still read something
physical. A `KINEMATIC`/`STATIC` body with an "incoming joint" (e.g. an
externally-scripted second link, as one existing test uses) is left alone —
only a `DYNAMIC` child's pose is derived this way. The recompute runs after
every substep in `step()`, after `set_joint_targets` (an instant
position-mode target moves the body right away, since this backend applies
targets exactly with no interpolation), and after `body_set_state` (a root's
new pose carries its whole subtree; a constrained `DYNAMIC` child can only
be reset to its declared rest pose, which zeroes its incoming joint,
mirroring the MuJoCo backend's own convention — never teleported
independently). It rejects a cyclic joint graph with
`NKSIM_ERROR_INVALID_ARGUMENT`, and clamps a joint's position to
`[lower_limit, upper_limit]` whenever `lower_limit < upper_limit`. Because a
kinematic cascade triggered by `body_set_state`/`set_joint_targets` can move
other bodies the caller never touched directly, `World::set_body_state`,
`World::reset_body`, `World::reset`, and `World::set_joint_targets` now
re-read every body's and joint's state back from the backend afterward, so
an immediate `nksim_body_get_state`/`nksim_joint_get_state` observes the
cascade rather than only the next `step()`.

### Cross-backend acceptance (M8.5)

`simkit/sim_mujoco/tests/mujoco.cpp`'s `cross_backend_link_poses_agree_with_fk`
is the one place both native backends are checked against each other and
against RobotKit's own FK formula in a single test binary: a 3-link arm
(non-zero offsets on both joints, a rotated joint frame on the second) at a
commanded configuration gives the same link world poses in the default
backend (exact, 1e-9) and MuJoCo (after settling, 1e-3), and teleporting the
root carries the whole arm in both. MuJoCo's per-joint PD controller (F2)
has no cross-joint compensation, so its steady-state tracking error grows
with lever-arm scale under active load; this test's anchors are a few
centimeters, not the meters a real link might use, specifically to keep
that expected, undiagnosed-bug error under the 1e-3 tolerance — a future
multi-DOF controller improvement should re-check this at larger scales.

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

## 3D spatial types

`robotkit.spatial` is the one place full rigid-body geometry lives above
`Robot`; `Pose2`/`FrameTree2`/`Navigator` are unchanged and remain the
planar stack used by `mobile`, `navigation`, and `localization`.

`Vec3` and `Quat` (x, y, z, w, per the frames-and-units rules above) are
immutable value types. `Quat`'s constructor always normalizes, so every
live `Quat` is unit length; there is no separate "unnormalized quaternion"
state to reason about. `Transform3` pairs a `Vec3` translation with a
`Quat` rotation and is the one 3D pose/transform type in RobotKit: a value
named `a_T_b` maps coordinates expressed in frame `b` into frame `a`, and
`compose` follows `a_T_c = (a_T_b).compose(b_T_c)`, matching
`world_T_sensor = world_T_link * link_T_sensor`. `toColumnMajorArray()`
exports the usual 4x4 column-major matrix; `Quat.toRotationMatrix()`
exports the 3x3 block the same way. `Transform3.fromPose2(pose, z)` and
`transform.toPose2()` are the only bridge to the planar stack: `toPose2()`
is a lossy yaw projection (it does not reject roll/pitch the way
`FrameTree2`'s planar frame import does), meant for driving `Navigator`
from an approximate 3D heading, not for round-tripping arbitrary
orientations.

`Twist3` (linear, angular) and `Wrench3` (force, torque) are plain spatial
vectors. `Transform3.transformTwist`/`transformWrench` apply the rigid-body
adjoint: if `this` is `a_T_b`, a twist or wrench known in `b` becomes the
equivalent value in `a` via `angular' = R * angular`,
`linear' = R * linear + p x angular'` (force/torque use the same block
structure). `Transform3.integrate(twist, dt)` composes a first-order
body-frame motion for `dt`; it is exact only as `dt -> 0`, which is what
the adjoint-consistency test in `SpatialTests` checks: transforming a body
twist into a rigidly-offset frame and then integrating it there agrees
with integrating the twist in its own frame and then transporting the
resulting pose, to first order in `dt`.

`FrameTree3` mirrors `FrameTree2` for full 3D poses: `FrameTransform3`
edges store `parent_T_child`, `add()` rejects a second parent or a cycle,
and `lookup(target, source)` returns `target_T_source` by walking both
frames to their common root. Unrelated frame graphs can share one tree
through a single registration edge — for example a BIM hierarchy
(`project -> building -> storey -> room -> wall`) and a robot's own frame
chain (`map -> base -> ... -> flange -> tcp`) joined by one
`project_T_map` edge, as milestone 6's CAD/BIM bridge will do.

### Joint-frame convention (read from `Joint`)

`FK` (milestone 2) reads `Joint.parentFramePosition`/`parentFrameRotation`,
`childFramePosition`/`childFrameRotation`, and `axis` directly; there is no
parallel joint description. These fields already have one authoritative
interpretation, used identically by `RobotFrameTree2` (planar) and by the
native runtime (`simulation.cpp` / MuJoCo backend):

- `parent_T_jointFrame = Transform3.fromArrays(parentFramePosition, parentFrameRotation)`
  and `child_T_jointFrame = Transform3.fromArrays(childFramePosition, childFrameRotation)`
  both place a "joint frame" relative to their respective link.
- `axis` is a unit vector expressed **in the joint frame** (i.e. after
  `parentFrameRotation`, before any joint motion) — this is exactly the
  vector `simulation.cpp` rotates by `parent_frame_rotation` to build
  `axis_a` (the joint axis in the parent body's frame) for the native
  physics joint.
- Joint motion at value `q` is a pure rotation about `axis` by `q`
  (revolute/continuous) or a pure translation along `axis` by `q`
  (prismatic), expressed in the joint frame; fixed joints contribute
  identity motion.
- `parent_T_child = parent_T_jointFrame.compose(motion(q)).compose(child_T_jointFrame.inverse())`.

This is the same composition `RobotFrameTree2.jointMotion`/`compose` chain
already implements for the planar frame tree; `robotkit.manipulation`'s
forward kinematics is the full-3D version of that same read, not a new
convention.

## Kinematic chains and IK

`robotkit.manipulation` sits above `robotkit.spatial` and below the future
`robotkit.tool`/`robotkit.process` layers (milestones 3-4); it does not
touch `Robot`, `RobotRuntime`, or the native runtime.

`KinematicChain` walks a `RobotModel`'s `Joint`s from a base `LinkId` to a
tip, which is either a link's own origin or a mounted `Frame`
(`ChainTip.Link`/`ChainTip.Frame`), using the joint-frame convention
documented above. Revolute and continuous joints become rotational degrees
of freedom, prismatic joints become translational ones, fixed joints fold
into constant transforms, and any other joint type (`Floating`, or an
unrecognized value) is a construction error — the same restriction
`RobotRuntimeCompiler` enforces. `forwardKinematics(q)` returns
`base_T_tip`; `allLinkTransforms(q)` returns `base_T_link` for the base
link and every link visited along the chain; `jacobian(q)` returns the
geometric Jacobian as 6 rows (linear x/y/z, then angular x/y/z) by `n`
degrees of freedom, expressed in the base frame — the same row order as
`Twist3`'s `(linear, angular)` fields.

`JointGroup` is an ordered, named joint/limit selection independent of any
one chain (`JointGroup.fromChain` is the common case). A joint whose
limits have `lower >= upper` — `JointLimits`'s own default, and the
existing convention for continuous joints — is treated as unlimited and is
never clamped.

`InverseKinematics.solve` is damped least squares (Levenberg-Marquardt
style) against the normal equations `(J^T J + λ^2 I) dq = J^T e`, with
joint limits clamped every iteration. The orientation error is the exact
axis-angle (log-map) rotation from the current tip orientation to the
target, not a small-angle linearization, so it remains well-defined for
large initial errors. It always returns an `IKResult`
(`converged`, `q`, `positionError`, `orientationError`, `iterations`); it
never throws for non-convergence. Like most redundant-looking wrists, this
chain family has more than one joint configuration reaching the same pose,
so a solver seeded far from the intended configuration can converge to a
different, still-valid, solution branch — callers that care which branch
they land in should seed close to their last known configuration.

`Manipulator` pairs a `KinematicChain` (whose tip must be a `Frame`, the
flange) with the compiled joint indices `toJointTargets(q)` needs: it
looks up each chain joint's position in `RobotModel.joints`, which is the
same index `RobotRuntimeCompiler` assigns as the runtime joint index, and
emits one `robotkit.world.JointTarget.position(...)` per degree of
freedom — the existing typed joint-command boundary, unchanged.

## Tools and TCP

`robotkit.tool` sits above `robotkit.manipulation`; it does not touch
`Robot`, `RobotRuntime`, or the native runtime. A `Tool` is a mounted end
effector: `id`, `flangeTTcp` (the tool center point's pose in the flange
frame, per the `a_T_b` convention — `flange_T_tcp` maps tool-tip coordinates
into the flange frame), a `ToolCollisionShape` (`NoCollision`, `Box`, or
`Cylinder`, since `model.CollisionApproximation` is a link-geometry
derivation policy, not a shape), and `mass`.

Capability control surfaces are typed interfaces, not
`Map<String, Dynamic>` commands, per haxeon's structural-typing rules:
`SurfaceTool` (enable/disable, standoff), `Sander` (speed, contact force),
`Sprayer` (flow, pressure), and `Gripper` (open/close, observed grasp
state). Every command method takes the caller's `timestampNs` explicitly —
simulated implementations never read a wall clock, keeping planners
deterministic. `Simulated*` classes implement each interface by recording
every commanded state change, with its timestamp, into a `history` array
(e.g. `SimulatedSprayer.history:Array<SprayerEvent>`) so tests and coverage
tracking can observe exactly what was commanded and when.

`Manipulator` carries the mounted tool's `flangeTTcp` (identity when no
tool is attached) and exposes it at the TCP level: `tcpPose(q)` is
`chain.forwardKinematics(q).compose(flangeTTcp)`, and
`solveIkForTcp(target, seed, ...)` converts a TCP-frame target to the
equivalent flange target (`target.compose(flangeTTcp.inverse())`) before
delegating to `InverseKinematics.solve`, so callers can work entirely in
tool-center-point coordinates without re-deriving the flange offset.

## Toolpaths and Cartesian trajectories

`robotkit.process` sits above `robotkit.tool`; it does not touch `Robot`,
`RobotRuntime`, or the native runtime. A `ToolpathPoint` is one TCP
waypoint (`work_T_tcp`, feed rate, `processOn`, and optional normal/standoff
a generator may attach) expressed in a named frame that a `Toolpath` (an
ordered list of points plus that `frameId`) carries as a whole.
`Toolpath.length()`/`processOnLength()` sum consecutive point distances; a
move's process state is its *departing* point's `processOn`, the same
convention `CartesianTrajectory` samples use.
`Toolpath.segmentByProcess()` splits into a leading `approach` run
(`processOn == false`), a `process` run from the first to the last
`processOn == true` point inclusive, and a trailing `retract` run.

`CartesianTrajectory.build(toolpath, maxAcceleration, sampleInterval,
?maxLinearStep, ?maxAngularStep)` gives each consecutive pair of points its
own independent symmetric trapezoidal (or triangular, when the distance is
too short to reach cruise speed) velocity profile toward the *arriving*
point's feed rate. The default spatial limits are 5 cm and 5 degrees. Each
segment uses the greatest of the time, linear-distance, and rotation-angle
sample counts, so `sampleInterval` remains an upper bound on the time between
samples without under-sampling long moves or pure reorientations. Position is
linearly interpolated along the straight line between the two points, and
rotation is slerped using the same normalized profile fraction, so translation
and rotation always reach a waypoint together. Every segment's final sample
lands exactly at that segment's closed-form duration; `duration()` is the last
sample's time.

`ToolpathExecutor.execute(manipulator, trajectory, base_T_work, seed, ...)`
samples the trajectory and solves `Manipulator.solveIkForTcp` for each
sample, seeded by the previous sample's solution (`base_T_work` brings the
trajectory's own frame into the chain's base frame; callers resolve that
transform, e.g. via `FrameTree3.lookup`, before calling). It rejects a
joint-space jump above `maxJointStep` between consecutive samples. Failure
is always an explicit `ToolpathExecutionResult` value
(`ToolpathExecutionFailure.Unreachable`/`Discontinuity`, carrying the
failing sample index), never an exception — matching `InverseKinematics`'s
own non-throwing convention. A successful result's `steps` pair each
sample's `JointTarget`s with its `processOn` flag; turning the physical
tool on/off from that flag is the caller's job (e.g. a future skill calling
`SurfaceTool.enable`/`disable`), keeping `robotkit.process` independent of
which capability interface a given tool implements.

## Work geometry

`robotkit.work` sits above `robotkit.process` (it builds `Toolpath` values)
but stays independent of `robotkit.tool`/`robotkit.manipulation`; it does
not touch `Robot`, `RobotRuntime`, or the native runtime, and geometry stays
planar (`Point2`/`Polygon2`, no curved surfaces).

A `WorkSurface` is a planar region to run a process over: `boundary` and
`exclusions` are `Polygon2` values (simple, counter-clockwise, positive
area) in the surface's own local plane, `+Z` outward. `frameId` is the
frame this surface is registered against (e.g. a BIM wall's frame or a
robot's world frame); `surfaceFrameId` names the surface's *own* local
plane frame, and `frame_T_surface` is the `frameId -> surfaceFrameId` edge
a caller registers into a `FrameTree3` via `WorkSurface.frameEdge()`.
`boundary`/`exclusions`, and every `Toolpath` a generator builds over the
surface, are expressed directly in `surfaceFrameId` — a generated
`Toolpath.frameId` is literally `surface.surfaceFrameId`, so `work_T_tcp`
in the plan's naming is exactly `surface_T_tcp`. `Provenance` (design
element id + `SourceKind`: `design`/`observed`/`work`) traces a surface
back to the design element it came from; M7 will add `observed`/`work`
surfaces derived from registration.

`Polygon2.scanlineIntervals(y)` returns the X-intervals where a horizontal
line at `y` is inside the polygon (even-odd rule); `contains(point)` is the
matching point-in-polygon test. `RasterToolpathGenerator.generate(surface,
toolWidth, overlap, standoff, feedRate, leadInOut)` builds a boustrophedon
`Toolpath`: rows are spaced `toolWidth * (1 - overlap)` apart, with the
first and last row placed exactly `toolWidth / 2` inside the boundary's Y
extent so the tool's own footprint radius (not just its centerline) can
still reach the top/bottom edges. Each exclusion is expanded by
`toolWidth / 2` before its scanline intervals are subtracted. The expanded
slice is built from the exclusion interior, an offset strip around every
edge, and a radius disk around every vertex, then the pieces are unioned.
This is the exact horizontal slice of the exclusion's Minkowski sum with a
disk, so rotated and concave exclusions receive the same tool clearance as
rectangular ones. The tool is off while transiting between rows and across
exclusion gaps within a row; a lead-in point (off, before the first process
point) and a lead-out point (off, after the last) bracket the whole path, so
`Toolpath.segmentByProcess()` recovers a proper approach/process/retract
split.

`CoverageMap(surface, cellSize)` grids the surface's boundary bounding box
and classifies every cell once, by its center, as `allowed` (inside the
boundary, no exclusion), `excluded` (inside the boundary and inside an
exclusion), or neither (outside the boundary). `markFootprint`/`markSweep`
mark cells within a radius of a point/segment as covered, regardless of
classification; `coverageFraction()` reports covered-allowed /
allowed-total, and `exclusionCoverageFraction()` reports covered-excluded /
excluded-total independent of cells that are simply outside the
boundary — so it is a direct test of whether the process ever touched an
excluded region, not of the raster's ordinary edge margin.

## CAD/BIM bridge (`robotkit/cadbridge`, separate project)

`robotkit/cadbridge` is its own haxeon project (`robotkit/cadbridge/haxeon.json`),
depending on `robotkit`, `cadkit`, and `bimkit`. It is the *only* place CAD/BIM
concepts meet RobotKit; `robotkit/haxeon.json` itself still depends only on
`nativekit`, per the plan's CAD-agnostic-core rule.

`cadbridge.FaceBridge.toWorkSurface(face, id, frameId, ?provenance,
?surfaceFrameId, ?scale)` converts any CadKit planar `Face` into a design
`WorkSurface`: it walks each wire's edge endpoints to preserve the authored
connected loop, normalizes the resulting loop to counter-clockwise, and picks
the largest-area wire as the boundary (`Polygon2`) and every other wire as an
exclusion — one exclusion per inner wire/opening. If an edge does not expose
two endpoints or the endpoint walk cannot close, the bridge raises an error
instead of guessing an order. `cadkit.Face`
(`cadkit/haxe/src/cadkit/Face.hx`) does not itself expose wire or vertex
enumeration, but `Face.cloneShape()` already returns a full `Shape` scoped
to just that face, and `Shape.subshapeCount`/`subshape` already walk any
`CadKit.ShapeKind`, including `Wire` and `Vertex` — reading a face's
boundary loops needed no CadKit change. The surface's local plane frame
(`frame_T_surface`) is built directly from the face's own `center()` and
`normal()`: `basisU = normal x up`, `basisV = normal x basisU`, so the
rotation's columns `(basisU, basisV, normal)` make the surface's local +Z
exactly the face's outward normal. `scale` converts the shape's own linear
units into meters; CadKit itself is unit-agnostic.

`cadbridge.WallBridge.wallToWorkSurface(bim, wallId, id, frameId,
?sideNormal)` finds a `BimSchema.Wall` element's side face (the planar face
whose normal is closest to `sideNormal`, default `+Y`) on its *cut* shape —
`bimkit.BimDocument.rebuildWall` already boolean-cuts a wall's body with
every hosted window/door, so that face's inner wires already are the
openings, and `FaceBridge` turns each into one exclusion automatically.
Provenance references the wall's BIM element id (`SourceKind.Design`).
BimKit's own convention is millimeter dimensions (`bimkit.BimSchema`
quantities), so `WallBridge` calls `FaceBridge` with `scale = 0.001`.

`cadbridge.BimFrameBridge.registerHierarchy(bim, tree)` walks a BIM
document's spatial aggregation edges (`BimSchema.Aggregates`: project ->
site -> building -> storey) and adds one `FrameTree3` edge per relationship,
named `"bim:" + elementId.value`. Project/site/building objects are generic
classified CadKit objects with no geometry of their own, so their edges are
identity; a storey with a base `Level` gets a Z-only translation from that
level's elevation (BimKit millimeters -> RobotKit meters).

`robotkit/cadbridge/tests` is its own nested haxeon project (entry
`tests.CadBridgeTests`), mirroring `robotkit/tests/haxeon.json`'s
`native.cmake` block (for RobotKit's own native runtime) and declaring
`cadkit`/`bimkit`/`projectkit`/`nativekit` alongside `robotkit`/`cadbridge`
as explicit dependencies — haxeon's dependency declarations are not
transitive for source resolution. CadKit's own native library is prebuilt
by its separate CMake/OCCT build (`cadkit/build/debug/core/libcadkit-core.so`),
not by haxeon, so running cadbridge tests requires that directory on
`LD_LIBRARY_PATH` (haxeon preserves and extends the caller's existing value).

## As-built registration

`robotkit.perception` (plus a `robotkit.work` addition) sits above
`robotkit.work`; it does not touch `Robot`, `RobotRuntime`, or the native
runtime. `PointCloud` is a set of `Vec3` points in one named frame with the
same source/receive clock provenance as `SensorFrame`
(`sourceTimestampNs`/`receivedTimestampNs`, `sourceClockId`/`receivedClockId`).
`PlaneEstimate` is a fitted plane `normal . x = offset` (in the cloud's own
frame) plus its inlier count and RMS residual.

`PlaneFit.fit` is total least squares: the normal is the smallest-eigenvalue
eigenvector of the 3x3 point covariance, found by `JacobiEigenSolver` — the
classic cyclic Jacobi eigenvalue algorithm, implemented directly (no external
linear-algebra library) and kept general over N even though `PlaneFit` only
ever calls it at N=3. `PlaneFit.fitRansac` is seeded RANSAC: it repeatedly
samples three points to build a candidate plane, keeps the candidate with the
most inliers within a threshold, then calls `fit` again over just that
consensus set for the final estimate — deterministic for a fixed seed via the
library's own `SeededRandom` (the same linear-congruential recurrence
`KinematicsTests`' IK fixture already used, promoted here so `PlaneFit` and
`SimulatedSurfaceScanner` share one seeded, wall-clock-free source of
randomness). `SeededRandom.nextGaussian` sums twelve uniforms (an
Irwin-Hall/CLT approximation) rather than Box-Muller, since
`haxeon/stdlib/Math.hx` has no `Math.log`.

`SimulatedSurfaceScanner.scan` samples a design `WorkSurface`'s boundary
(skipping exclusions, as a real scanner would see through an opening) from a
"true" wall offset from design by a small rigid `surface_T_trueSurface`
transform plus a smooth bow. The bow term is re-centered to exactly zero mean
over the actual sampled points *before* the rigid transform and noise are
applied, so a plane fit recovers the injected offset/rotation rather than a
bow-biased plane; the bow remains visible as a spatial pattern to a
`DeviationMap`. A generous RANSAC inlier threshold is required to recover the
injected offset/rotation to the plan's 1mm/0.05° tolerance: a tight threshold
can silently drop the bow's most extreme (and therefore most negative, after
demeaning) samples from the consensus set, reintroducing the bias the
demeaning was meant to remove.

`SurfaceRegistration.register(design, cloud, seed, ...)` fits a plane (seeded
RANSAC, oriented by the design's own `+Z`) and derives the *minimal* rigid
correction between the design plane (`z = 0` in the surface's own frame) and
the fitted plane: a rotation about `(0,0,1) x normal` by the angle between
them (identity when they already agree — this is exactly what recovers an
installation yaw error, i.e. a rotation of the wall about its own vertical
axis that tilts its normal), and a translation of `offset` along that normal
— purely out-of-plane. In-plane shift and rotation about the normal are not
observable from a plane fit alone and are left as identity; this is a
deliberate scope limit, not an oversight. A correction whose translation or
rotation magnitude exceeds configured limits is rejected
(`SurfaceRegistrationResult.accepted == false`, `registered == null`,
`rejectionReason` set) instead of silently applied. An accepted registration
produces a `work` `WorkSurface`: same boundary/exclusions (their in-plane
shape is unaffected by an out-of-plane correction) at
`design.frame_T_surface.compose(correction)`, with `provenance` retaining the
design element id under `SourceKind.Work`.

`robotkit.work.DeviationMap(surface, cellSize)` mirrors `CoverageMap`'s grid
(cells classified by the surface's boundary bounding box), but accumulates
the *mean* signed deviation reported for each cell instead of a covered flag.
A caller adds samples already expressed in the surface's own local plane
(`addPoints` for a raw point cloud, treating each point's `x, y` as location
and `z` as the deviation) so a smooth bow shows up as a spatial pattern
(higher near the bow's center, lower near the boundary) rather than
collapsing to one scalar.

## Base placement and reachability

`robotkit.manipulation` gains `ReachabilityChecker` and `WorkPatchPlanner`,
still above `Robot`/`RobotRuntime`/the native runtime, and still not
performing any joint base+arm optimization: base motion is the plan's
existing planar `Navigator`/`GoTo`, driven by a `Pose2` each planner produces
via `Transform3.toPose2`.

`ReachabilityChecker.check(manipulator, toolpath, base_T_work, seed, ...)`
solves IK for every point of a `Toolpath` from one candidate base placement
(folded into `base_T_work`, the same "manipulator base frame to toolpath
frame" convention `ToolpathExecutor` uses), seeded by continuation from the
previous point's *converged* solution — not a failed attempt's own wandering
final iterate, which can land far from any good configuration and would
otherwise poison every following point's seed. It reports the reachable
fraction and the index of the first unreachable point rather than aborting,
unlike `ToolpathExecutor`, which is used only after a placement has already
been chosen.

`WorkPatchPlanner.plan(design, map_T_surface, manipulator, maxPatchWidth,
...)` splits a `WorkSurface` into axis-aligned column patches no wider than
`maxPatchWidth` (mirroring `RasterToolpathGenerator`'s own axis-aligned
scope: a patch's boundary/exclusions are the design's own bounding-box
rectangles clipped to the column, exact for the rectangular walls this
milestone targets), builds each patch's raster `Toolpath`
(`RasterToolpathGenerator`), and searches a grid of candidate base
placements — positioned in front of the wall along the surface's outward
normal within a standoff distance band, at lateral offsets across the
patch's width, all projected into `map_T_surface`'s frame and then down to a
`Pose2` via a `Transform3` built purely from a yaw computed toward the wall
— to find one from which the whole patch is reachable. A candidate within a
`BaseObstacle`'s radius plus the search's `clearance` is skipped outright, so
an obstacle at the otherwise-best candidate forces the search onto a
different one. Each returned `WorkPatch` carries its own sub-`WorkSurface`,
`Toolpath`, and the chosen `basePose`; `WorkPatchPlanResult.fullyPlanned` is
false if any patch's best candidate fell short of full reachability. Base
motion between patches is left entirely to the existing `Navigator`/`GoTo`
against each patch's `basePose`.

## Simulated wall-finishing robot (M9)

`robotkit/tests/src/tests/WallFinishingScenarioTests.hx` is the first
end-to-end exercise of the full stack (`spatial` -> `manipulation` -> `tool`
-> `process` -> `work` -> `perception` -> `mobile`/`navigation` ->
`skill`-adjacent orchestration): a `RobotModel` fixture (an omnidirectional
base — `HolonomicDrive`, three wheels at 120 degrees — carrying a UR5-style
6R arm, the same published DH-equivalent offsets `KinematicsTests`/
`PlacementTests` already use, with a sprayer flange offset and a
base-mounted lidar-kind "scanner" sensor) drives a design `WorkSurface`
through scan -> registration -> `WorkPatchPlanner` -> per-patch
navigate/execute/coverage -> verification, records the run to MCAP via
`RecordingRobot`, and replays it against a `ReplayRobot`.

The "BIM wall" is a `WorkSurface` built directly in the test, not through
`robotkit/cadbridge`: `robotkit/tests/haxeon.json` depends only on
`nativekit`/`robotkit`, and importing `cadbridge` there would pull CadKit
into the CAD-agnostic core's own test project. Per the plan's own fallback,
`robotkit/cadbridge/tests` carries a separate end-to-end check instead
(`testBimWallToPatchPlanEndToEnd`): a `BimSchema.Wall` with a hosted opening,
through `WallBridge.wallToWorkSurface`, into `WorkPatchPlanner` against a UR5
fixture built directly in that test project (cadbridge cannot depend on
`robotkit/tests`' own fixtures) — it does not assert `fullyPlanned` the way
the hand-built, axis-aligned `PlacementTests` (M8) fixture does, since a real
`WallBridge`-derived `frame_T_surface` is not tuned to one particular
standoff search; each patch reaching more than half-reachable is enough to
demonstrate the BIM-to-patch-plan pipeline without overfitting the test to
one fixture's search parameters.

The scenario's arm holds its seed configuration (a submitted
`JointTargets` command) from the very first tick, throughout every patch's
navigation phase, not only during raster execution. Nothing else commands
the arm while the base drives between patches, and the default backend's
purely kinematic joint placement (M8.5 F4) never drifts an uncommanded
joint away from its rest value — but MuJoCo has real dynamics, so an
uncommanded joint free-falls under gravity while the base drives, eventually
exceeding the joint's compiled envelope. Holding a defined pose is also the
physically correct model for a real robot driving between patches, not a
workaround.

Coverage is derived from the *observed* TCP pose each tick
(`Simulation.linkPose` of the wrist-3 link, composed with `linkTFlange` and
`flangeTTcp`), never from commanded joint targets, per the plan. The same
observed-pose computation doubles as the required cross-check between
SimKit and RobotKit's own FK: `wrist3FromSim` (the simulator's own reported
link pose) must equal `basePoseNow . manipulator.tcpPose(reportedQ)` (base
pose composed with RobotKit's FK evaluated at the simulator's *own reported*
joint values) — this is a pure kinematics-consistency check between the
physics engine's body pose and RobotKit's analytic FK, independent of how
closely the controller is tracking its *commanded* target, so it holds to
near machine precision in both backends regardless of controller settling.
The default backend asserts it to `1e-6` (it applies position targets
exactly and instantly, per M8.5 F4); the MuJoCo backend, which has finite
settling time, keeps the same computation but only reports it (a loose
`2e-2` sanity bound guards against a gross regression, e.g. a stalled or
diverging joint, without asserting default-backend precision).

`runScenario(backend, physicsTimestep, physicsSubsteps, strictFkCrossCheck,
minCoverage)` parameterizes the whole scenario so the identical
scan/register/plan/navigate/execute/coverage/replay logic runs against
either backend; `testSimulatedWallFinishingScenario` calls it with
`backend = 0` (default, `strictFkCrossCheck = true`, `minCoverage = 0.99`),
and `testSimulatedWallFinishingScenarioMuJoCo` with the plan's own
`new Simulation(0.01, 2, 1)` (`strictFkCrossCheck = false`,
`minCoverage = 0.97`). A fixed navigation tick budget is scaled by the
timestep (`40.0 / timestep`) so both backends get the same *simulated-time*
budget to converge, not the same tick count. MuJoCo's per-joint
computed-torque controller (a real critically-damped second-order response,
time constant ~1/omega_n ~ 16ms) needs several time constants to close a
potentially multi-radian jump between a patch's seed-held arm pose and its
first reach pose — a jump the default backend closes in one tick since it
applies targets exactly — so the MuJoCo run gives that initial move 60
settling ticks (instead of 2) and each subsequent raster sample 6 ticks
(instead of 2) before reading back joint state; this is the class of
"controller re-tuned, not masked regression" adjustment M8.5 F2 already
established precedent for. Observed on this fixture: MuJoCo coverage
99.27% (default backend 99.23%), FK-consistency tracking error max
5.4e-8m / RMS 4.7e-8m (both backends, since it is a kinematics-consistency
check as above, not a controller-tracking metric), and joint-tracking error
(commanded vs. simulator-observed position) max ~0.9 degrees under MuJoCo
vs. exact under the default backend.

**Why a separate `robotkit/tests/mujoco` haxeon project.** `robotkit/tests`'
own native build has `NKSIM_BUILD_MUJOCO` compiled out (`robotd/native`'s own
`option(... OFF)`, so every ordinary Haxe test project's build time and
dependency footprint stays small), and haxeon's `NativeCMakeProvider` always
configures a package's `native.cmake` project with a fixed, hardcoded
argument list — there is no manifest field to pass an extra `-D` define per
package. `robotkit/robotd/native-mujoco/CMakeLists.txt` is a thin wrapper
that pre-seeds `NKSIM_BUILD_MUJOCO=ON` in its *own* isolated CMake cache
before `add_subdirectory`-including the real `robotd/native` project (CMake's
`option()` only sets a variable when it is not already cached, so the
wrapper's forced value wins without touching `robotd/native`'s own default
for any other consumer). `robotkit/tests/mujoco/haxeon.json` points its
`native.cmake.source` at that wrapper and its `entry` at the small
`tests.WallFinishingMuJoCoRunner`, which calls only
`WallFinishingScenarioTests.runMuJoCo()` — not `RobotWorldTests.main()`,
which is still the entry for the standard `robotkit/tests` project and must
never reach a `new Simulation(dt, substeps, 1)` call on a build where MuJoCo
support is compiled out (`RK_ERROR_UNSUPPORTED`, not a silent fallback, per
the "One simulation tick" section above).

## Construction skills (M10)

`robotkit.skill` gains five compositions over the M1-M9 layers, all run
through the existing `SkillRunner` and following `GoTo`/`PickPallet`'s
pattern of a `start()`/`update()`/`cancel()` lifecycle backed by
`SkillLifecycle`:

`ScanSurface` dwells for a configured number of `update()` calls (a
stationary scan pass takes real time on a real sensor), then captures one
`PointCloud` from a caller-supplied `scan:Void->PointCloud` closure —
keeping the skill independent of any one scanning technique (a real
accumulated LiDAR scan, `SimulatedSurfaceScanner.scan`, or a replayed cloud
all fit the same shape). The closure must be deterministic given the
observations already available to the caller, matching every other
simulated capability's no-wall-clock rule, so a replay reproduces it.

`RegisterSurface` runs `SurfaceRegistration.register` against an
already-captured cloud (typically `ScanSurface.cloud`); registration is a
closed-form computation, not a physical action, so it completes within
`start()` the way `GoTo` completes immediately when already at its goal.

`FinishSurface(surface, spec:FinishSpec)` drives M8's `WorkPatchPlanner`
through M4's `ToolpathExecutor`: plan patches once in `start()`, then for
each patch navigate the base (`Navigator`/`GoTo`, unchanged, per the plan's
base-motion boundary) and execute its raster, toggling a caller-supplied
`setProcessOn:(Bool, Int64)->Void` callback around each step's `processOn`
flag. `FinishSpec` is a pure-data anonymous typedef (per haxeon's
structural-typing rules) carrying every `RasterToolpathGenerator`/
`WorkPatchPlanner`/`CartesianTrajectory`/`ToolpathExecutor` parameter the
plan's raster geometry and tolerances need. `FinishSurface` itself never
depends on which capability interface the mounted tool implements — `Paint`
binds `setProcessOn` to `Sprayer.setFlow`/`setPressure`, and `Sand` binds it
to `Sander.setSpeed`/`setContactForce` (a contact-force setpoint while
sanding, per the plan), each a thin thirty-line wrapper delegating every
`Skill` method to an internal `FinishSurface`. Coverage is tracked from the
*planned* TCP pose (`Manipulator.tcpPose` at each executed step's joint
solution): unlike the M9 scenario test, a skill has no `Simulation` to
cross-check against and must work identically over a `RemoteRobot`,
`SimulatedRobot`, or `ReplayRobot`.

`Drill` (point operations at surface positions) was not built: nothing in
this milestone's acceptance needs it, and the plan marks it optional.
**`LayTile` is out of scope for this plan.** It would need: an inventory
model (tile stock, size, and orientation), a `Gripper`-based pick-and-place
sequence analogous to `PickPallet`/`PlacePallet` but for individual tiles
against a laid course, adhesive/mortar process state (a new capability
interface alongside `SurfaceTool`/`Sander`/`Sprayer`), and force control
during placement (seating a tile against a substrate without cracking it or
leaving a proud edge) that this codebase has no capability interface or
simulated contact-force model for yet. `robotkit.work.WorkPatchPlanner`'s
axis-aligned patch geometry would also need a per-tile course/coursing-offset
layer above the raster it already produces.

`robotkit/tests/src/tests/ConstructionSkillTests.hx` exercises `ScanSurface`,
`RegisterSurface`, `Paint`, and `Sand` through `SkillRunner` against an M9-style
simulated robot, then replays every one of them against a `ReplayRobot` of
the recording, mirroring the forklift skills' pattern
(`testForkliftSkillsOnSimulationAndReplay` replays each of its own skills
individually, not just one representative skill). `ScanSurface`/
`RegisterSurface` submit no `RobotCommand`, so replaying them exercises only
their own `SkillRunner` lifecycle against the `ReplayRobot`'s observations,
not the recorded command stream; `Paint` and `Sand` were recorded back to
back in one continuous MCAP stream during the live run, so their replays
share one `ReplayRobot` cursor, continuing straight from where `Paint`'s
replay leaves off into `Sand`'s recorded commands. Replay needs its own fresh
`MobileBase`/`Navigation`/`Navigator` over the `ReplayRobot` (not the live
run's, whose underlying `RecordingRobot` is closed): `SimulationTruthLocalization`
reads a live `Simulation`'s own owned base pose directly, which a `ReplayRobot`
does not have, so replay instead uses `HolonomicOdometryLocalization` (an
`odom`-to-`base` estimate from the three wheel joints' recorded positions, the
holonomic equivalent of `WheelOdometryLocalization`). The replayed run is not
asserted bit-identical to the live run's coverage: it drives its own
freshly built A* route and pure-pursuit tracking from the same recorded
observations, which can converge to a very slightly different final base
pose (still within `GoTo`'s own tolerance) with no nondeterminism in the
replayed observations themselves; both runs independently reaching strong
coverage is the meaningful check.

## Terrain height maps (M11)

`robotkit.work` gains a second grid family alongside `CoverageMap`/
`DeviationMap`, for the excavator milestone. Where those two grids classify
or accumulate samples at *cell centers* over a `WorkSurface`'s boundary,
`HeightMap` stores elevation at grid *vertices* (`columns x rows` points,
`cellSize` apart, from `(originX, originY)`) in a named frame, because a
vertex grid is what makes bilinear interpolation and per-cell trapezoidal
volume well defined without an extra half-cell offset to reason about.
Like `CoverageMap.covered`, a `HeightMap`'s elevation array is mutable state
(`lowerTo`/`setElevation`), not an immutable value type: it models terrain
that a `BucketSweep` physically changes over time, not a pose or transform.

`HeightMap.bilinearSample(x, y)` interpolates within the grid's extent and
throws outside it (the same strict-validation style as `WorkSurface`'s own
constructors) rather than clamping, so a caller's own bounds mistake is
visible immediately instead of silently reading an edge value.

`HeightMap.volumeBetween(existing, design, ?skipCell)` requires both maps to
share the same frame and grid geometry (`HeightMap.ensureSameGrid`) and sums,
over every `(columns - 1) x (rows - 1)` cell, the average of its four
corners' signed `existing - design` difference times the cell's plan area,
splitting the result into `cut` (existing above design) and `fill` (existing
below design) in a `VolumeResult`. `skipCell` (by grid cell, i.e. its
lower-left vertex indices) lets `EarthworkRegion.remainingVolume` omit any
cell touching an exclusion polygon without duplicating the summation. For a
trench whose vertical faces land exactly on grid columns and which spans a
grid's full extent in the other axis, this trapezoidal average reproduces
the trench's exact geometric volume with no discretization error — the two
"half-cut" boundary columns each contribute exactly half a full column's
volume, together equal to one full column — which is what
`TerrainTests.testVolumeOfKnownTrench` checks bit-for-bit rather than within
a tolerance.

`EarthworkRegion` pairs an `existing` and `design` `HeightMap` (validated to
share one grid) with exclusion polygons (`robotkit.work.Polygon2`, the same
type `WorkSurface` uses for its own exclusions) and a `gradeTolerance`.
`isAtGrade(col, row)` is true when a vertex's excluded, or its
`|existing - design|` delta is within tolerance; `gradeFraction()` and
`worstVertex()` (the largest-magnitude non-excluded, out-of-tolerance delta)
are the two queries `GradeRegion` (M12) needs to decide where to dig next and
when to stop.

`BucketSweep.apply(map, from, to, halfWidth, edgeHeight)` is deliberately not
a soil model: it lowers every grid vertex within `halfWidth` of the swept
segment (a capsule footprint) that is currently above `edgeHeight` down to
`edgeHeight`, and reports `removedVolume` as
`sum(oldElevation - edgeHeight) * cellSize^2` — a box/Voronoi area
approximation per vertex, with no fill-factor, spillage, or repose-angle
modeling. Sweeping twice at the same `edgeHeight` removes nothing further,
which is the closest thing to a physical invariant this approximation needs
to satisfy.

One haxeon-specific finding while writing `EarthworkRegion.remainingVolume`:
an anonymous function passed as an argument (e.g.
`function(col:Int, row:Int) { ... }`) cannot carry an explicit return-type
annotation the way a named function or method can — `function(col:Int,
row:Int):Bool { ... }` fails to parse (`E0002: Expected expression` at the
`:`), because `Parser.hx`'s anonymous-function-literal path
(`TokenKind.Function` in `parsePrimary`) goes straight from the closing
`)` of the argument list to the function body with no `:Type` production in
between (only `parseFunction`/`parseFunctionBody`, used for declarations and
methods, parse a return type). The fix is simply to omit the annotation and
let the return type be inferred, as every other lambda in this codebase
already does. A related, separately-discovered finding while writing
`TerrainTests`: field access on a `Null<T>` value — even one already guarded
by a prior `check(value != null, ...)` runtime-assertion call — is rejected
at compile time (`E1005: Field "..." requires an object`) unless the
narrowing comes from an actual `if`/`||`/`&&` control-flow construct
`FlowAnalysis.narrowedScope` tracks; a `check(...)` helper call, however
assertion-like, is invisible to that analysis. The existing codebase's own
`if (x == null) throw ...; use(x.field);` guard-clause idiom (seen throughout
`KinematicChain`, `WorkPatchPlanner`, etc.) is what actually narrows, so
`TerrainTests.testCellsAtGradeReported` uses that idiom instead.

## Simulated excavator (M12)

The excavator fixture (built in `robotkit/tests/src/tests/ExcavatorTests.hx`,
not the library, per the same "fixtures live in tests" convention M2's UR5
fixture established) is a tracked-base machine reduced to the joints that
actually matter for digging: `slew_joint` (revolute, `+Z`), then
`boom_joint`, `stick_joint`, `bucket_joint` (revolute, all `+Y`, i.e.
parallel to each other and perpendicular to the slew axis), each folded
straight onto the previous link with no rotation offset. The undercarriage
itself is a single fixed link with no drive joints -- `RobotDriveConfiguration`
has no "tracked" variant, and modeling track propulsion (or the machine
repositioning between passes) is out of scope for this milestone's bounded
skills, which operate from one stationary base pose. The chain's tip is a
`Frame` at the bucket pivot; a `Tool` (`flange_T_tcp`, mass, a `Box`
collision shape) carries the constant translation from that pivot out to the
bucket's cutting edge, giving `KinematicChain`/`Manipulator`/`InverseKinematics`
a 4-DOF chain reused completely unchanged from M2 -- no new manipulation code
was needed for this milestone.

**IK approach (the plan's "task-space weighting... or a closed-form planar
solver, your choice, log it"):** because `boom`/`stick`/`bucket` all turn
about *parallel* axes, `KinematicChain.evaluate`'s own rotation composition
(traced by hand against its source: each step's `motion` rotates about the
joint's local axis, and consecutive rotations about a shared axis commute and
add) means the chain's tip orientation is *always* exactly
`Rz(slew) * Ry(boomAngle + stickAngle + bucketAngle)` -- a two-parameter
family (`slew`, total pitch), regardless of how the three angles individually
split that pitch. `DigCyclePlanner.poseAt(x, y, z, pitch)` builds every
waypoint's orientation as `Rz(atan2(y, x)) * Ry(pitch)`, i.e. always exactly
on that manifold (`atan2(y, x)` is also the position's own required slew, so
position and orientation are never in conflict). With every target already
on the manifold, the existing generic 6-DOF damped-least-squares
`InverseKinematics.solve` converges to near-zero residual in *both* position
and orientation with **no code changes and no weighting scheme** -- simpler
than either alternative the plan offered, since there is no free orientation
DOF left over to weight or solve for separately once slew and pitch are
fixed. This is verified directly: `ExcavatorTests.testFourDofIkRecoversManifoldTargets`
warm-starts across a short sequence of `poseAt`-built targets and checks
both position (`< 1e-3` m) and orientation (`< 1e-2` rad) convergence.

**Joint limits are deliberately wide** (`boom` +/-2.2 rad, `stick`/`bucket`
+/-3.0 rad) compared with a real machine: an early, tighter set (+/-1.5 /
+/-2.8 / +/-2.8, closer to a real excavator's working envelope) made one
otherwise-reasonable trench waypoint (a modest reach, moderate depth,
`digPitch = -0.4`) genuinely infeasible -- not an IK convergence problem but
a real one, confirmed independently in Python by a closed-form 2R sub-solve
(`P2 = target - bucketLength * direction(pitch)`, then a standard two-link
IK for `boom`/`stick` reaching `P2`) that found the *closest* achievable
point under those limits still ~6.5cm off, with `boom` pinned at its limit.
Widening the limits (still well short of a full rotation on any joint) made
every cycle's waypoints exactly reachable. This fixture is for exercising the
planning/skill machinery, not certifying a real machine's working envelope, so
robustness of the solver took priority over tight real-world limits; a real
excavator's controller would enforce its own certified limits underneath
`RobotRuntime` regardless (see the safety note below).

**`DigCyclePlanner`** (`robotkit.work`, alongside `HeightMap`/`EarthworkRegion`/
`BucketSweep`, since it plays the same "produces a `process.Toolpath`" role
`RasterToolpathGenerator` already does from that package) builds one dig
cycle -- entry, cut, curl, lift, N dense swing waypoints, descend (still
curled), open -- as a `Toolpath` plus a `DigCyclePlan`'s sweep parameters
(`BucketSweep.apply`'s own arguments) for the caller to apply once execution
succeeds. Two things worth noting: (1) the swing phase is deliberately *dense*
(`swingSteps` intermediate `poseAt` waypoints, not just the two endpoints) --
`CartesianTrajectory`'s straight-line position lerp / rotation slerp between
two *far apart* manifold-consistent points does not itself stay on the
manifold at intermediate fractions (slerp interpolates the shortest quaternion
arc, which is not `Rz(slew(t)) * Ry(pitch(t))` for an arbitrary two
endpoints), so a wide, sparse swing can fail IK partway through; dense
waypoints keep every consecutive pair's implied slew delta small enough that
the interpolation error stays under tolerance. (2) the final "dump" is *two*
points (descend at `curlPitch`, then open to `dumpPitch` at the same
position), not one combined move, for the same reason -- changing position
and a large pitch amount simultaneously through a single lerp/slerp segment
risks the identical manifold-drift problem; splitting so each move changes
either position or pitch (not a large amount of both) sidesteps it without
adding any manifold-specific logic to the generic `CartesianTrajectory`/
`ToolpathExecutor` machinery.

**Skills** (`robotkit.skill`, following `FinishSurface`'s pattern of a
pure-data `*Spec` typedef plus a `start()`/`update()`/`cancel()` lifecycle):
`DigTrench(lineFrom, lineTo, width, depth, gradeTolerance, spec, seed)` and
`GradeRegion(region, spec, seed)` both loop bounded `DigCyclePlanner` cycles
-- `DigTrench` re-cutting the same line deeper each pass (bounded by
`spec.maxCutPerPass`) until every one of `spec.progressSamples + 1` points
along the line reads within `gradeTolerance` of the design elevation from
`heightMap.bilinearSample`; `GradeRegion` repeatedly targeting
`region.worstVertex()` (the largest-magnitude non-excluded, out-of-tolerance
vertex) until it returns null -- both applying `BucketSweep.apply` to the
height map only *after* a cycle's `Toolpath` executes successfully, so
progress is always read back from the terrain the way a real machine's only
feedback is the ground it actually moved, never the commanded cycle alone.
Both fail explicitly after `spec.maxCycles` (a bounded operation, not an
open-ended search, per the plan), and `GradeRegion` also fails explicitly if
its worst vertex needs *fill* rather than cut (out of scope: no soil
mechanics, no material-addition model). `DumpAt(pose)` is a single bounded
Cartesian move + release, for a caller that already holds a loaded bucket and
just needs to relocate a little and open it; unlike `DigTrench`/`GradeRegion`,
its target `pose` is not required to be `poseAt`-manifold-consistent, so it
inherits the same lerp/slerp manifold-drift limitation described above for
large combined moves -- its own test exercises a realistic "release in place"
move (small relocation, pitch-only change from a seed already at the dump
position) rather than an arbitrary cross-workspace jump, and this limitation
is the reason a caller needing a large reposition-and-dump should use
`DigTrench`/`GradeRegion`'s own built-in dump legs (which stay on the
manifold throughout) instead of a standalone `DumpAt`.

Both `beginNextCycle` implementations seed every cycle's `ToolpathExecutor`
call from the skill's own *constructor* seed, not the previous cycle's final
(dump) joint configuration: a new cycle's entry pose is usually a large
joint-space jump away from wherever the previous cycle's dump left the arm,
and warm-starting from that far, ever-drifting configuration hit the same
cold-start/local-optimum sensitivity M2's and M8's logs already document
(confirmed directly: the *same* seed-independent residual appeared whether
seeded from the previous dump or from a fresh warm-started seed, which is
also how a genuine infeasibility was distinguished from a convergence issue
while tuning the joint limits above). Re-approaching from a known-good seed
each cycle also matches how a real operator would reposition the boom/stick/
bucket before a new pass, rather than trying to make a single autonomous
controller responsible for tracking every intermediate configuration between
cycles.

`ExcavatorTests.testDigTrenchScenario` digs a 1.5m trench (`width = 0.8`m,
`depth = 0.5`m, `gradeTolerance = 0.03`m, `maxCutPerPass = 0.2`m) to grade in
3 bounded cycles, reporting `remainingDepthError`/`totalRemovedVolume` from
the height map after every cycle; `testGradeRegionScenario` grades a single
high spot on a 9x9 pad to `gradeFraction() == 1.0`. Both run on the default
simulator backend only: unlike M9, the plan's MuJoCo fallback applies here --
after M9's MuJoCo scenario needed a thin CMake wrapper
(`robotkit/robotd/native-mujoco`) specifically to reach a
`NKSIM_BUILD_MUJOCO=ON` native build, standing up a *second* such MuJoCo
Haxe project just to re-verify an already-generic (M2, unmodified)
4-DOF `KinematicChain`/`InverseKinematics` path -- whose interesting
behavior is entirely in Haxe-level planning/skill logic, not in anything
MuJoCo's own dynamics would newly exercise -- was not judged worth
duplicating that build-system surface area for; M9's own MuJoCo run already
covers this codebase's one MuJoCo-specific finding (holding an idle arm's
seed configuration against gravity), and this milestone adds no new
joint-actuation or rigid-body code for MuJoCo to stress differently.

**Safety boundary.** `DigTrench`/`GradeRegion`/`DumpAt` are application-level
autonomy running above `RobotRuntime`, exactly like every other skill in this
codebase (`ARCHITECTURE.md`'s "Runtime versus simulation" section already
draws this line for navigation/manipulation skills generally) -- they are
**not** the safety-rated layer for an excavator. A real machine's
certified controller, underneath `RobotRuntime` and outside this codebase's
scope, is what would enforce ISO 17757 (autonomous/semi-autonomous earthmoving
machinery safety) and ISO 19014 (functional safety for earth-moving
machinery) -- envelope limits, operator-presence/e-stop interlocks, and
collision/tip-over avoidance independent of whatever a `DigTrench` or
`GradeRegion` cycle happens to command. Nothing in this milestone (or any
earlier one) attempts to satisfy either standard; `robotd`'s existing
controller-lease/heartbeat/emergency-stop mechanism (M0) is a liveness
safeguard for the control channel, not a certified functional-safety system.


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
