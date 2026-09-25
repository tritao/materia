# RobotKit

RobotKit is the complete robotics layer for Materia. It owns robot models,
commands, control, runtime ownership, endpoint adapters, world orchestration,
and the protocol shared by `robotd` and editor clients.

The current increment contains deliberately small boundaries:

- `runtime`: engine-neutral values and validation, an owner-thread runtime,
  command mailbox, immutable snapshots, and the first framed serial endpoint;
- `haxe/robotkit/protocol`: versioned framing independent of any particular transport;
- `haxe`: Haxeon façades, protocol clients, and `RobotWorld` orchestration;
- `robotd`: one independently deployable logical robot host;
- `Simulation`: one shared SimKit-backed universe and clock for any number of
  simulated robots.

The semantic robot model and native-runtime compiler are reusable Haxe APIs
under `haxe/robotkit`; `robotd` supplies only process hosting and deployment
policy.
RobotKit receives only compiled runtime blueprints and bulk data at execution
boundaries. A standalone `RobotRuntime` can use the in-memory endpoint for
host bring-up, while `Simulation` owns the shared SimKit backend for live
multi-robot execution. The first physical boundary is a deliberately small
POSIX framed serial endpoint; its device protocol remains replaceable until a
specific controller is selected.

Build and test RobotKit independently:

```sh
cmake -S . -B build -GNinja -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build --output-on-failure
```

The canonical native CMake target is `RobotKit::runtime`.

`robotd/native` enables the SimKit simulation and builds the native dependency
graph consumed by the Haxeon host. The Haxe façade and wire protocol are under
`haxe/robotkit`;
its bindings deliberately submit one command batch and retrieve one snapshot
per tick.

Multi-robot simulation is coordinated by `rk_simulation`. One simulation owns
the SceneKit scene, SimKit world, host, and clock. Robot runtimes remain the
command/snapshot boundary, but they do not advance physics independently:
`Simulation` drains every robot's commands, advances the shared world once,
and publishes every robot state from the resulting snapshot. The Haxe
`robotkit.runtime.Simulation` façade exposes the same lifecycle while behavior
code continues to depend on robot-scoped submit/snapshot APIs.
Participating runtimes cannot be stepped individually; applications must call
`Simulation.step()` so the shared-world boundary remains explicit.

`Simulation` also owns explicit runtime-scene operations: reset, per-robot
reset, robot teleport, environment-object spawn/remove/teleport, and one
shared simulation clock. These edits are accepted while stopped. Once running,
physics is authoritative and editable-scene changes must go through the
simulation owner. Each simulated robot publishes transport-neutral joint
encoder, IMU, and LiDAR frames with the same source clock, frame IDs, and
sequences used by the remote path.

IMU measures mounted sensor-frame angular velocity and specific force from
physics body state; its first sample after reset/teleport primes the derivative.
Frames define mount translation/orientation; IDs and mounts survive compilation,
wire transport, and recording. Sensors configure Hz, LiDAR ray count (1–64),
range, first-ray bearing, angular coverage, and deterministic seeded Gaussian
noise. LiDAR queries box geometry and excludes own links. Runtime receipt
timestamps use the actual local monotonic clock, not the simulation tick hint.
Absolute world-command deadlines are rejected until clock negotiation and
runtime enforcement are available.

MuJoCo is opt-in and tested through the same runtime/sensor implementation:

```sh
cmake -S robotkit/robotd/native -B /tmp/materia-mujoco -DROBOTD_BUILD_TESTS=ON -DNKSIM_BUILD_MUJOCO=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/materia-mujoco -j 6
ctest --test-dir /tmp/materia-mujoco --output-on-failure
```

Select it with `new Simulation(0.01, 2, 1)`; backend `0` remains the test backend.

`robotkit.world.SimulatedRobot` adapts one simulation-owned runtime to the same
`Robot` interface used by `RemoteRobot`. `robotkit.world.SerialRobot` compiles
an authored `RobotModel`, opens a POSIX serial device, and owns its standalone
runtime. Both can be attached to `RobotWorld` and used through the same command
and snapshot interfaces. The serial protocol carries indexed target batches,
joint positions/velocities/efforts, and sensor samples in compiled model slot
order; see the serial protocol document before implementing device firmware.

The complete ownership and tick model is documented in
[`ARCHITECTURE.md`](ARCHITECTURE.md).

`RobotWorld` is a single-owner composition object. Its owner thread is checked
at the boundary; adapter callbacks only enqueue `RobotWorldEvent` values, and
the owner applies them with `pump()` while building a snapshot.
`WorldSnapshot` and `RobotSnapshot` own copied arrays and sensor frames, expose
no mutable maps, and distinguish backend/source time from the time the world
received an observation.

The ownership rule is intentionally simple:

```text
RobotWorld owns attached adapters
Simulation owns simulated runtimes and physics
robotd owns the deployed runtime and endpoint
```

`robotkit.worldd.WorldHost` is only a headless composition of those existing
objects. It does not introduce a second world model. `ReplayRobot`, recording,
and `WorldBehaviorRunner` use the same `Robot`/snapshot/command boundary for
offline debugging and behavior reuse.

World commands use `RobotCommand.JointTargets` to submit a heterogeneous batch
as one operation. `JointTargetMode` distinguishes position, velocity, and
effort values, so wheel pairs and steering-plus-drive setpoints share the same
transport-neutral API:

```haxe
world.submit("amr-17", JointTargets([
  JointTarget.velocity(leftWheel, leftSpeed),
  JointTarget.velocity(rightWheel, rightSpeed)
], null));
```

Adapters copy and validate a batch before submitting it. Repeated joint indices
are rejected. `RemoteRobot` sends one `JointTargets` protocol frame and
`robotd` queues one native runtime command, preserving the batch sequence.
`SimulatedRobot` submits the same target array through one runtime mailbox call;
`ReplayRobot` retains generated batches separately from the source recording.
The runtime applies configured position, velocity, and effort limits.

`robotkit.mobile` adds planar motion as a configured view over a normal `Robot`.
`MobileBase` applies `MotionLimits` and sends the selected `DriveModel`'s joint
targets as one batch. `DifferentialDrive` maps forward/yaw velocity to left and
right wheel rates; `AckermannDrive` maps them to steering position and drive
wheel rate. `Pose2`, `Twist2`, and `Footprint` are transport-neutral values.
`DifferentialOdometry` integrates wheel-position changes from `RobotSnapshot`
and resets its encoder baseline when the source clock identity changes.

Drive roles can be authored on `RobotModel` with stable joint IDs instead of
repeating runtime array positions and wheel geometry in application factories.
The compiler resolves those roles against the model's joint ordering and stores
the resolved configuration on `RobotRuntimeBlueprint`. It rejects missing or
reused roles, incompatible joint types, and invalid dimensions before runtime
creation. Factories also check the live robot description against the compiled
joint ordering before exposing a control view.

```haxe
model.mobileBase = new RobotMobileConfiguration(
  RobotDriveConfiguration.Differential(leftWheelId, rightWheelId, 0.1, 0.5),
  1.5, 1.2, 0.8, 1.5, 0.8, 0.55);
var blueprint = RobotRuntimeCompiler.compile(model);
var base = MobileBase.fromBlueprint(robot, blueprint);
base.command(new Twist2(0.6, 0.2), 0.02);
```

The optional duration applies acceleration limits relative to the previous
command. Joint indices follow the robot description, and the wrapped robot
continues to own status, snapshots, transport, and lifecycle.

`robotkit.localization` represents estimates separately from robot snapshots.
Each `LocalizationState` carries a pose, reference/body frame IDs, planar
covariance, quality, and both source and receive clock identities.
`WheelOdometryLocalization` derives an `odom` to `base` estimate from a
differential `MobileBase`; `SimulationTruthLocalization` projects a
simulation-owned robot pose into `map` to `base` for deterministic scenarios.
`FrameTree2` stores static parent-child transforms and exposes explicit
`target_T_source` lookup direction. `PoseFusionLocalization` combines wheel
odometry with transformed external pose observations using separate covariance
weights for planar position and heading. It reports an invalid estimate until
the odometry frame can be anchored to the requested reference frame.

`robotkit.navigation.Navigation` follows a framed `Path` using the latest
localization state and an application-supplied update duration. `Trajectory`
stores time-parameterized pose/twist samples, and `NavigationGoal` defines final
position and heading tolerances. The controller commands `MobileBase`; runtime
limits and native safety checks remain active underneath it.

`Path.project` returns monotonic arc-length progress, signed cross-track error,
and the active segment tangent. `Navigation` adapts lookahead to commanded
speed, limits speed by curvature and lateral acceleration, and slows according
to the remaining braking distance. Path waypoint yaw describes the robot body
heading; when it faces opposite the segment tangent, the follower drives that
segment in reverse.

`Trajectory.fromPath` converts a geometric path into timed pose and body-velocity
samples. It applies the mobile base's linear and angular speed and acceleration
limits, a lateral acceleration limit, and the drive model's steering curvature
limit. `Navigation.followTrajectory` tracks those samples with pose feedback;
`follow` remains available for online geometric path following.

```haxe
navigation.follow(new Path([startPose, stagingPose, goalPose], "odom"));
navigation.updateObservation(robot.snapshot(), 0.02);
```

`robotkit.material.Forks` maps configured lift, tilt, and spread joint names to
atomic position batches. `ForkState` reads their positions, velocities, efforts,
and source/receive clocks. `Payload`, `LoadState`, and `LoadLimits` represent
load knowledge and a configured mass, load-moment, and lift-height envelope;
runtime joint limits remain authoritative.

Fork axes can use model-owned joint limits and IDs. Compile the model once and
construct both views from its blueprint:

```haxe
model.forkMechanism = new RobotForkConfiguration(liftJointId,
  1000.0, 600.0, 1.8, tiltJointId, spreadJointId);
var blueprint = RobotRuntimeCompiler.compile(model);
var base = MobileBase.fromBlueprint(robot, blueprint);
var forks = Forks.fromBlueprint(robot, blueprint);
```

`robotkit.perception` provides timestamped `Detection`, `Obstacle`, `Pallet`,
and `DockingTarget` values. `LidarObstaclePerception.fromSensor()` uses the
compiled sensor's range and angular coverage to turn finite LiDAR returns into
planar obstacle observations in the sensor frame.
`robotkit.safety` exposes operator-facing phase, active restrictions, speed
limit, and stopping envelope values; the native runtime continues to enforce
hard safety. `LoadSafetyPolicy.refresh()` reduces `MobileBase` speed and
acceleration limits from observed payload mass and fork height, reports an
expanded robot-and-load footprint, and blocks new base commands when fork or
payload state exceeds its configured envelope. Unknown load state applies
conservative limits until the application confirms the forks are empty or
carrying a load. `robotkit.power` defines `BatteryState` and a `Power` view for
charge, voltage, current, temperature, energy, and clock provenance.

`robotkit.skill` composes these explicit views into task-sized operations. Each
`Skill` has `start`, `update`, `cancel`, `status`, and `result` methods. `GoTo`
follows a path, `Dock` approaches a detected target, `PickPallet` and
`PlacePallet` combine navigation with fork commands and load confirmation, and
`Charge` docks before waiting for a battery threshold. Skills run at the
application's update frequency and use the same `Robot` boundary, so the same
scenario can run with `SimulatedRobot`, `RemoteRobot`, or `ReplayRobot`.

```haxe
var base = new MobileBase(robot,
  new DifferentialDrive(leftWheelJoint, rightWheelJoint, wheelRadius, trackWidth),
  motionLimits);
var localization = new WheelOdometryLocalization(base);
var navigation = new Navigation(base, localization);
var forks = new Forks(robot, forkConfig);

var pick = new PickPallet(navigation, forks, pallet, payload, approachPose, 0.5);
pick.start();
// Call update(robot.snapshot(), dt) until the load sensor confirms pickup.
```

### Persistent recordings

`McapRobotRecording` can preserve an in-memory `RobotRecording` while enqueueing
versioned JSON payloads to a byte-bounded native writer thread. Pass
`retainInMemory = false` for file-only capture. Call `close()` to
drain the queue and write the MCAP footer. Queue overflow, I/O failure, and a
writer destroyed without a clean finish are explicit failures; `status()`
exposes accepted, written, queued-event, queued-byte, and dropped counts.
Terminal status is persisted beside the recording and can be inspected after
restart with `McapRecordingReader.status(path)`. Files are currently
uncompressed for straightforward inspection.

`RecordingRobot` decorates any `Robot` with a `RobotRecordingSink`. It records
each returned observation and each accepted `JointTargets` batch while
delegating status, transport, and lifecycle to the wrapped adapter. Both the
in-memory and MCAP writers implement the sink, so a control client can record a
simulation or remote robot without changing its skill code. Logging failures
are reported through `recordingError` without changing the wrapped robot's
command or observation results.

Every message carries a recording-wide 64-bit ordinal. It is the sole replay
ordering key: robot and sensor source timestamps retain their clock-domain IDs
and are never compared across domains. Wide sequences and timestamps are JSON
decimal strings, avoiding precision loss in generic JSON tools. Payload schemas
cover commands, robot snapshots, individual sensor frames, faults, world
snapshots, and world lifecycle events. Schema v2 records complete joint target
batches; readers also load v1 single-position commands as one-target batches.
Recorded commands are history only; loading a file never forwards them to a
live adapter.

Recording timestamps are captured separately from event ordinals and stored in
both MCAP time fields; the ordinal remains a full-width decimal string in the
payload instead of masquerading as a timestamp. The reader validates
the exact channel schema, channel/event type agreement, envelope ordinal and
timestamp, and payload contract. `McapRecordingReader.next()` is an incremental
cursor; the convenience `load()` method is the explicitly retaining variant.

```haxe
var writer = new McapRobotRecording("session.mcap", 16 * 1024 * 1024, false);
writer.recordSnapshot(robot.snapshot());
writer.close();

var recording = McapRecordingReader.load("session.mcap");
var replay = new ReplayRobot("offline", recording);
while (replay.advance()) { /* deterministic single-step playback */ }
```

The native dependency is pinned to MCAP C++ 2.1.3. To independently inspect a
fixture with the official Python MCAP implementation, retain the test file with
`ROBOTKIT_KEEP_MCAP=1` and open it using `mcap.reader.make_reader`; CI/native
tests also reject truncated files and unknown schemas. Run
`robotkit/tests/mcap-independent.sh` for the automated official-Python-reader
cross-check.

`robotkit/tests/world-tcp.sh` exercises the same `HoldJointBehavior` against a
local `SimulatedRobot` and a TCP-connected `RemoteRobot` hosted by `robotd`.
An atomic position/velocity/effort batch is checked before three successive
behavior targets test command counts, unchanged-snapshot suppression, settled
joint positions, and encoder/IMU/LiDAR identities, mounts, and values. The
fixture uses the deterministic simulation backend; independent clocks and
sample sequences are deliberately not compared for equality. Run with
`ROBOTKIT_TEST_SESSIONS=1` to check controller leases, observers, and reconnects.

Behavior hosting builds on that same boundary. `RobotBehaviorRunner` receives a
`RobotSnapshot`, gives a behavior a read-only `RobotContext`, and publishes the
latest expiring intent through `IntentBuffer`. `robotd` turns supported intents
into bulk runtime commands; behaviors never access native handles or MuJoCo
state directly. The initial `robotd --behavior=oscillate` behavior is an
integration probe for this path, not a permanent controller API.
