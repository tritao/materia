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
- software homing to an authored coordinate plus bounded timed jogging through the same logical-axis API;
- a MachineKit `LinearAxis` compiler that produces a two-link prismatic
  RobotModel and a runtime-ready MotionSystem blueprint.
- buffered trajectory execution with native execution plans and a trajectory
  queue, including bounded streaming for trajectories longer than one chunk.

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

`home()` moves to the authored home coordinate using a planned trajectory. It
does not reference a physical limit switch or establish a hardware zero.

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

## What “validated” means

`ValidationReport.guarantees()` summarizes each check as `Proven`,
`Sampled(resolutionNs)`, `Unchecked`, or `Failed`. Joint position, velocity,
and acceleration bounds are checked against polynomial extrema over the full
trajectory when those limits are claimed. Jerk and continuity are reported
separately; either can be `Unchecked` if its limit was not supplied. An
unchecked check is not a safety guarantee.

Cartesian task-space deviation is checked at samples no more than 1 ms apart
for timed paths and manipulator programs. `Sampled` describes coverage at
those points, not a continuous bound between them. The report retains the
actual sampling resolution and any unresolved assumptions. Each execution
plan exposes the summary through `plan.guarantees()`; active manipulator
programs include it in `ManipulatorProgress.guarantees`.

`motionkit.trajectory.Trajectory.fromPositionSamples` builds degree-1 native
segments. Native velocity is each segment's chord slope; acceleration and
jerk are zero within that segment. Robots used with MotionKit must support
execution plans and the trajectory queue.

Every change of speed obeys the joint limits. A moving Ruckig axis move, jog,
or home can retarget from the runtime's committed state. Degree-1 path moves
still stop first, so `moveLinear` may return null while motion is in progress.
`movePath` needs the machine at rest, since a path must start where the machine
is. CNC semantics and G-code remain outside MotionKit.

A `jog` issued while the same axis is already jogging changes speed or
direction without stopping. The runtime replaces the Ruckig plan beyond
the committed horizon; a late replacement is retried once before falling
back to stop-first.

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

### Sensor homing

Physical home switches carry `homeAfter` prerequisite joint IDs. MachineKit authors these with
`assembly.homeAfter("x", ["z"])` after registering all switches. Assembly flattening scopes the
references with the owning machine; saved assemblies and RobotModel artifacts retain them.
Missing prerequisites, dependency cycles and disagreeing switches on one coordinate are rejected.
The shared robot queue executes the validated order serially; axis names do not imply clearance.

Homing searches for a switch, stops through the runtime owner, and plans a finite backoff to
release it. The approach starts only after the endpoint is at rest, all contacts are open and
release clearance is confirmed. It never substitutes a captured search edge for the final approach.
Captured-edge sources identify their capability even before the first crossing. Such sources
must supply a fresh crossing position and counter; digital-only sources use a speed derived from
switch repeatability and observation period. Capability changes and missing captures fault.

`HomingDynamics` includes command/observation latency, acceleration and the planner's jerk limit
when bounding switch approach speed. Runtime command admission and the owner period establish the
default latency; `MotionSystemBlueprint.homingLatencySeconds` can state a longer measured response
budget. Jerk defaults to acceleration divided by the owner period as a planning policy, rather
than a claimed hardware specification. Finite plans use the same acceleration and jerk limits.
Captured approaches reserve physical end-stop room, and independent motor holds additionally
require an authored racking tolerance. Stop, hold and calibration acknowledgements remain barriers.

Homing retains the runtime's finite coordinate search window. A cold start whose unknown counter
origin places the switch outside that window faults at the boundary; it does not expand travel
blindly. Full-stroke homing from arbitrary unknown origins requires a matching runtime/device
search-window contract. Displaced authored poses with known coordinates remain supported.
