# MotionKit

MotionKit is the transport-neutral motion layer between mechanical design and
RobotKit execution. The bootstrap package currently provides:

- reusable Cartesian path points, line primitives, and ordered geometric paths;
- immutable timed joint trajectory samples;
- a deterministic synchronized velocity/acceleration planner with jerk in the
  public limits API;
- a semantic `MotionSystem`/`MotionAxis` view over any RobotKit `Robot`;
- a MachineKit `LinearAxis` compiler that produces a two-link prismatic
  RobotModel and a runtime-ready MotionSystem blueprint.

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
```

The bootstrap submits one position batch per deterministic update. Buffered
trajectory commands, continuous-path lookahead, Cartesian paths, and CNC
semantics are intentionally subsequent increments.

Run the focused native-backed test with:

```sh
./haxeon/scripts/haxeon run --project motionkit/tests/haxeon.json
```
