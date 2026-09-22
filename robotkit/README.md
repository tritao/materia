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

IMU measures base-frame angular velocity and specific force from physics body
state; its first sample after reset/teleport primes the derivative. LiDAR uses
eight planar rays against the current box geometry, excluding own links.
These measurements use SimKit's test backend; MuJoCo dynamics, arbitrary sensor
mounts, and configurable scan patterns remain future work. Runtime receipt
timestamps use the actual local monotonic clock, not the simulation tick hint.
Absolute world-command deadlines are rejected until clock negotiation and
runtime enforcement are available.

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

Behavior hosting builds on that same boundary. `RobotBehaviorRunner` receives a
`RobotSnapshot`, gives a behavior a read-only `RobotContext`, and publishes the
latest expiring intent through `IntentBuffer`. `robotd` turns supported intents
into bulk runtime commands; behaviors never access native handles or MuJoCo
state directly. The initial `robotd --behavior=oscillate` behavior is an
integration probe for this path, not a permanent controller API.
