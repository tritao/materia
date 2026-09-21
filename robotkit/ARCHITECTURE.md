# RobotKit architecture

RobotKit separates the logical robot API from the mechanism that produces a
robot's state. This is what lets the same world and behavior code work with a
remote physical robot, a local simulation, a replay source, or a future
hardware-in-the-loop adapter.

## Public object model

```text
RobotWorld
└── RobotInstance[]
    ├── RemoteRobot
    │   └── RobotClient ── RobotProtocol ── robotd / physical process
    └── SimulatedRobot
        └── RobotRuntime ──┐
                            ▼
                        Simulation
```

`RobotWorld` owns the logical `RobotInstance` map. It does not know whether a
robot is remote or simulated. It routes `RobotCommand`, collects
`RobotSnapshot`, tracks lifecycle changes, and builds `WorldSnapshot` values.

`RemoteRobot` owns a network client. `SimulatedRobot` is only an adapter over a
`RobotRuntime` supplied by an externally owned `Simulation`; it never creates,
steps, stops, or disposes that simulation.

## Runtime versus simulation

`RobotRuntime` is the command mailbox and state publication boundary for one
robot. `RobotRuntimeBlueprint` is the compiled execution description produced
from the editable robot model. `RobotRuntimeLayout` and
`RobotRuntimeSnapshot` describe the native execution ABI and are deliberately
kept below the `RobotInstance` API.

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
