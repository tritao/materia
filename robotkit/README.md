# RobotKit

RobotKit is the robotics layer for Materia. It owns robot models, commands,
control, runtime ownership, endpoint adapters, and the protocol shared by
`robotd` and editor clients.

The first increment contains four deliberately small boundaries:

- `core`: engine-neutral runtime values and validation;
- `runtime`: an owner-thread runtime, command mailbox, and immutable snapshots;
- `protocol`: versioned framing independent of any particular transport;
- `sim endpoint`: a SimKit-backed endpoint that owns the live simulation model
  when enabled by the host application.

The semantic robot/document model belongs to the Haxeon `robotd` application.
RobotKit receives only compiled runtime blueprints and bulk data at execution
boundaries. The runtime includes an in-memory endpoint for deterministic tests
and an optional SimKit endpoint for live simulation. MuJoCo model loading,
controllers, NativeKit transport adapters, and physical endpoints remain later
increments.

Build and test RobotKit independently:

```sh
cmake -S . -B build -GNinja -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build --output-on-failure
```

The canonical CMake targets are `RobotKit::core`, `RobotKit::runtime`, and
`RobotKit::protocol`.

`robotd/native` enables the SimKit endpoint and builds the native dependency
graph consumed by the Haxeon host. The Haxe façade is under `haxeon/robotkit`;
its bindings deliberately submit one command batch and retrieve one snapshot
per tick.

Behavior hosting builds on that same boundary. `RobotBehaviorRunner` receives a
`RobotSnapshot`, gives a behavior a read-only `RobotContext`, and publishes the
latest expiring intent through `IntentBuffer`. `robotd` turns supported intents
into bulk runtime commands; behaviors never access native handles or MuJoCo
state directly. The initial `robotd --behavior=oscillate` behavior is an
integration probe for this path, not a permanent controller API.
