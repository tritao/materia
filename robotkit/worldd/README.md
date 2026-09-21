# worldd

`WorldHost` is the headless composition point for RobotKit. It creates one
`RobotWorld`, one shared `Simulation`, attaches remote and simulated robots,
and exposes the resulting immutable snapshots. It deliberately adds no second
world model or fleet abstraction; a future protocol server should wrap this
composition rather than duplicate its ownership rules.

The package also has a small executable smoke host. It creates the same
composition without introducing another runtime model:

```sh
../../haxeon/scripts/haxeon run --project haxeon.json -- --robots=2 --ticks=10
```

Use `Simulation` for reset, robot teleport, and environment-object operations;
use `RobotWorld` for adapter ownership, discovery, health, observation events,
and transport-neutral commands/snapshots. A future Facility/Fleet/Mission
layer may compose `RobotWorld`, but must not move those concepts into
RobotKit.
