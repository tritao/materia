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
range, and deterministic seeded Gaussian noise. LiDAR queries box geometry and
excludes own links. Runtime receipt
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
`Robot` interface used by `RemoteRobot`. It does not own or dispose the
shared simulation, allowing one `RobotWorld` to contain local simulated robots
and remote physical robots without backend-specific orchestration.

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

Every message carries a recording-wide 64-bit ordinal. It is the sole replay
ordering key: robot and sensor source timestamps retain their clock-domain IDs
and are never compared across domains. Wide sequences and timestamps are JSON
decimal strings, avoiding precision loss in generic JSON tools. Payload schemas
cover commands, robot snapshots, individual sensor frames, faults, world
snapshots, and world lifecycle events. Recorded commands are history only;
loading a file never forwards them to a live adapter.

Recording timestamps are captured separately from event ordinals and stored as
MCAP log time; the ordinal is stored as MCAP publish time. The reader validates
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
Three successive targets check command counts, unchanged-snapshot suppression,
settled joint positions, and encoder/IMU/LiDAR identities, mounts, and values.
The fixture uses the deterministic simulation backend; independent clocks and
sample sequences are deliberately not compared for equality. Run with
`ROBOTKIT_TEST_SESSIONS=1` to check controller leases, observers, and reconnects.

Behavior hosting builds on that same boundary. `RobotBehaviorRunner` receives a
`RobotSnapshot`, gives a behavior a read-only `RobotContext`, and publishes the
latest expiring intent through `IntentBuffer`. `robotd` turns supported intents
into bulk runtime commands; behaviors never access native handles or MuJoCo
state directly. The initial `robotd --behavior=oscillate` behavior is an
integration probe for this path, not a permanent controller API.
