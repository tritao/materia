# MotionKit

MotionKit is the transport-neutral motion layer between mechanical design and
RobotKit execution. The bootstrap package currently provides:

- reusable Cartesian path points, line and planar-arc primitives, and ordered
  geometric paths;
- immutable timed joint trajectory samples;
- a native polynomial trajectory evaluator with degree 0–5 segments,
  analytic derivatives and a Haxe wrapper;
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

The pure `motionkit` package contains paths, planners, trajectories and logical
axes. `motionkit-robot` provides `motionkit.robot.MotionSystem`,
`motionkit.robot.MotionSystemBlueprint` and
`motionkit.robot.MachineKitRobotCompiler` for RobotKit integration.

MachineKit dimensions are authored in millimetres. The compiler converts them
to RobotKit metres, places the logical zero at the axis's lower travel limit,
and retains the motor and lead-screw identity on the compiled actuator. The
actuator ratio follows the thread hand: positive rotation about +Z moves a
right-hand nut toward -Z.

The first end-to-end path is intentionally small:

```haxe
import motionkit.robot.MachineKitRobotCompiler;
import motionkit.robot.MotionSystem;

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

The runtime executes native trajectory plans on its owner clock and publishes
queue depth and tagged progress through `RobotSnapshot`. MotionKit refills
long paths in bounded plan chunks. Native HOLD and RESUME slow and restart the
path clock within joint acceleration limits. Non-final chunks declare that
more motion follows, so a late refill triggers controlled underflow braking.
The runtime rejects plans that violate joint limits and bounds queue depth.

`motionkit.trajectory.Trajectory.fromPositionSamples` builds degree-1 native
segments. Native velocity is each segment's chord slope; acceleration and
jerk are zero within that segment. The transitional sampled Haxe type retains
authored derivatives for the position-streaming fallback.

Every change of speed obeys the joint limits. A moving Ruckig axis move, jog,
or home can retarget from the runtime's committed state. Degree-1 path moves
still stop first, so `moveLinear` may return null while motion is in progress.
`movePath` needs the machine at rest, since a path must start where the machine
is. MotionKit retains its path-preserving position-target fallback, including
host-side re-timing, for backends without plan support. CNC semantics and
G-code remain outside MotionKit.

A `jog` issued while the same axis is already jogging changes speed or
direction without stopping. Queue backends replace the Ruckig plan beyond
the committed horizon; a late replacement is retried once before falling
back to stop-first. The non-queue fallback still uses `JogProfile`.

Axis moves are planned in logical axis units and mapped onto joints, so the
motors of a geared or dual-motor axis stay in proportion throughout a move.

Queued trajectories currently run back to back with a 1–2 control-cycle pause
between them: the next one is only sent once the runtime has drained the
previous one. Planned moves start and end at rest, so this costs throughput
rather than smoothness. Sending the next trajectory ahead of time is the next
buffered-execution increment.

Run the focused native-backed test with:

```sh
./haxeon/scripts/haxeon run --project motionkit/tests/haxeon.json
```
