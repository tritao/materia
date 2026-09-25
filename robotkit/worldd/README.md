# worldd

`WorldHost` is the headless composition point for RobotKit. It creates one
`RobotWorld`, one shared `Simulation`, attaches remote and simulated robots,
and exposes the resulting immutable snapshots. It is a local simulation host,
not a multi-robot protocol service, and adds no second world model or fleet
abstraction.

The package also has a small executable smoke host. It creates the same
composition without introducing another runtime model:

```sh
../../haxeon/scripts/haxeon run --project haxeon.json -- --robots=2 --ticks=10
```

Use `Simulation` for reset, robot teleport, and environment-object operations;
use `RobotWorld` for adapter ownership, discovery, health, observation events,
and transport-neutral commands/snapshots. The separate `materia.automation`
package composes facility, fleet, task, and mission behavior over RobotKit;
those operational concepts remain outside RobotKit.
