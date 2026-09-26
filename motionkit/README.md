# MotionKit

MotionKit is the transport-neutral motion layer between mechanical design and
RobotKit execution. The bootstrap package currently provides:

- reusable Cartesian path points, line and planar-arc primitives, and ordered
  geometric paths;
- immutable timed joint trajectory samples;
- deterministic synchronized velocity/acceleration planning with jerk in the
  public limits API, plus tangent-aware line/arc lookahead with exact-stop and
  blend modes;
- a semantic `MotionSystem`/`MotionAxis` view over any RobotKit `Robot`;
- authored homing plus bounded timed jogging through the same logical-axis API;
- a MachineKit `LinearAxis` compiler that produces a two-link prismatic
  RobotModel and a runtime-ready MotionSystem blueprint.
- buffered trajectory execution with a native timestamped-chunk path when the
  RobotKit runtime advertises queue support, including bounded streaming for
  trajectories longer than one native chunk.

MachineKit dimensions are authored in millimetres. The compiler converts them
to RobotKit metres, places the logical zero at the axis's lower travel limit,
and retains the motor and lead-screw identity on the compiled actuator.

The first end-to-end path is intentionally small:

```haxe
var blueprint = MachineKitRobotCompiler.compileLinearAxis(axis, "x");
var runtime = simulation.addRobot(blueprint.runtime);
var robot = new SimulatedRobot("gantry-x", runtime, blueprint.model.name,
  [for (link in blueprint.model.links) link.name],
  [for (joint in blueprint.model.joints) joint.name]);
var machine = MotionSystem.fromBlueprint(robot, blueprint);

machine.home();
machine.moveAxes([new AxisTarget("x", 0.04)], new MotionOptions(0.08, 0.4));
while (machine.isMoving()) {
  machine.update();
  simulation.step(nextTimestamp);
}

machine.jog("x", 0.02, 1.0);
while (machine.isMoving()) {
  machine.update();
  simulation.step(nextTimestamp);
}
```

For a direct XYZ machine, the same system can plan and buffer a Cartesian
polyline:

```haxe
machine.movePath(GeometricPath.lines([
  new PathPoint(0.0, 0.0, 0.0),
  new PathPoint(0.1, 0.0, 0.0),
  new PathPoint(0.1, 0.1, 0.0)
]), PathPlanningOptions.blend(0.001));
```

The runtime interpolates timestamped trajectory chunks on its owner clock and
publishes queue depth and tagged timestamp progress through `RobotSnapshot`.
MotionKit refills long trajectories before the native window drains. A normal
hold slows the trajectory clock along the planned path, keeping every joint
within its acceleration limit even when the hold lands while the trajectory is
already speeding up or braking. Before stopping, a hold tops up the queued path
to at least v/a. If a stop still runs out of queued path, the runtime finishes
it on a straight, acceleration-limited ramp. A chunk that arrives while a stop
is running only extends that stop. Resume starts from the runtime-reported stop
tag and time. MotionKit retains a deterministic position-target fallback when a
backend does not support buffered chunks. CNC semantics and G-code remain
outside MotionKit.

Queued trajectories currently run back to back with a 1–2 control-cycle pause
between them: the next one is only sent once the runtime has drained the
previous one. Planned moves start and end at rest, so this costs throughput
rather than smoothness. Sending the next trajectory ahead of time is the next
buffered-execution increment.

Run the focused native-backed test with:

```sh
./haxeon/scripts/haxeon run --project motionkit/tests/haxeon.json
```
