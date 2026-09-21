# RobotKit

RobotKit is the complete robotics layer for Materia. It owns robot models,
commands, control, runtime ownership, endpoint adapters, world orchestration,
and the protocol shared by `robotd` and editor clients.

The first increment contains deliberately small boundaries:

- `runtime`: engine-neutral values and validation, an owner-thread runtime,
  command mailbox, and immutable snapshots;
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
multi-robot execution. MuJoCo model loading, controllers, NativeKit transport
adapters, and physical endpoints remain later increments.

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

`robotkit.world.SimulatedRobot` adapts one simulation-owned runtime to the same
`Robot` interface used by `RemoteRobot`. It does not own or dispose the
shared simulation, allowing one `RobotWorld` to contain local simulated robots
and remote physical robots without backend-specific orchestration.

The complete ownership and tick model is documented in
[`ARCHITECTURE.md`](ARCHITECTURE.md).

Behavior hosting builds on that same boundary. `RobotBehaviorRunner` receives a
`RobotSnapshot`, gives a behavior a read-only `RobotContext`, and publishes the
latest expiring intent through `IntentBuffer`. `robotd` turns supported intents
into bulk runtime commands; behaviors never access native handles or MuJoCo
state directly. The initial `robotd --behavior=oscillate` behavior is an
integration probe for this path, not a permanent controller API.
