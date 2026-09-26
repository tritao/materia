import haxe.Int64;
import machinekit.assembly.LinearAxis;
import motionkit.AxisTarget;
import motionkit.Feed;
import motionkit.MachineKitRobotCompiler;
import motionkit.MotionOptions;
import motionkit.MotionSystem;
import motionkit.Pose;
import motionkit.path.GeometricPath;
import motionkit.path.PathPoint;
import motionkit.planner.TrapezoidalPlanner;
import motionkit.trajectory.MotionLimits;
import robotkit.runtime.Simulation;
import robotkit.world.SimulatedRobot;

class MotionKitBootstrapTests {
  static var assertions:Int = 0;

  public static function main():Void {
    testGeometricPathPrimitives();
    testPlannerIsDeterministicAndBounded();
    testLinearAxisCompilesToRobotModel();
    testCompiledAxisRunsThroughSimulation();
    testCompiledXYZGantryRunsThroughSimulation();
    Sys.println('MotionKit bootstrap tests passed ($assertions assertions)');
  }

  static function testGeometricPathPrimitives():Void {
    var path = GeometricPath.lines([new PathPoint(0.0, 0.0, 0.0),
      new PathPoint(0.1, 0.0, 0.0), new PathPoint(0.1, 0.1, 0.0)]);
    near(path.totalLength, 0.2, "line path accumulates primitive lengths");
    near(path.pointAt(0.05).x, 0.05, "line path samples its first primitive");
    near(path.pointAt(0.15).y, 0.05, "line path samples its second primitive");
  }

  static function testPlannerIsDeterministicAndBounded():Void {
    var planner = new TrapezoidalPlanner(0.01);
    var limits = new MotionLimits(1.0, 2.0, 10.0);
    var first = planner.plan([0.0, 0.0], [1.0, -0.25], limits);
    var second = planner.plan([0.0, 0.0], [1.0, -0.25], limits);
    check(first.durationSeconds > 0.0, "planner produces a timed trajectory");
    check(first.samples.length == second.samples.length, "planner sample count is deterministic");
    for (i in 0...first.samples.length) {
      near(first.samples[i].timeSeconds, second.samples[i].timeSeconds,
        "planner sample time is deterministic");
      for (joint in 0...2) {
        near(first.samples[i].positions[joint], second.samples[i].positions[joint],
          "planner position is deterministic");
        check(Math.abs(first.samples[i].velocities[joint]) <= limits.maxVelocity + 1e-6,
          "planner respects velocity limit");
        check(Math.abs(first.samples[i].accelerations[joint]) <= limits.maxAcceleration + 1e-6,
          "planner respects acceleration limit");
      }
    }
    near(first.sample(0.0).positions[0], 0.0, "trajectory starts at the requested position");
    near(first.sample(first.durationSeconds).positions[0], 1.0,
      "trajectory ends at the requested position");
  }

  static function testLinearAxisCompilesToRobotModel():Void {
    var axis = new LinearAxis(23, 10, 80);
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(axis, "x", 0.1, 0.4);
    check(blueprint.model.links.length == 2, "linear axis compiles base and carriage links");
    check(blueprint.model.joints.length == 1, "linear axis compiles one prismatic joint");
    var joint = blueprint.model.joints[0];
    check(joint.id == "x" && joint.name == "x", "linear axis uses a stable joint ID");
    check(joint.type == robotkit.model.JointType.Prismatic, "linear axis is prismatic");
    near(joint.parentFramePosition[2], (axis.screwStart + axis.travelMin) * 0.001,
      "MachineKit travel origin is converted to metres");
    near(joint.limits.upper, 0.08, "MachineKit stroke becomes the logical upper limit");
    near(joint.limits.velocity, 0.1, "compiled actuator rate is retained");
    check(joint.drive != null && joint.drive.name.indexOf(axis.motor.designation) >= 0,
      "compiled actuator retains motor identity");
    check(joint.drive != null && joint.drive.name.indexOf("mm/rev") >= 0,
      "compiled actuator retains transmission identity");
  }

  static function testCompiledAxisRunsThroughSimulation():Void {
    var axis = new LinearAxis(23, 10, 80);
    var blueprint = MachineKitRobotCompiler.compileLinearAxis(axis, "x", 0.1, 0.4);
    var simulation = new Simulation(0.01);
    var runtime = simulation.addRobot(blueprint.runtime);
    var robot = new SimulatedRobot("single-axis", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var machine = MotionSystem.fromBlueprint(robot, blueprint);

    machine.home();
    check(machine.isMoving(), "home creates a motion command");
    machine.update();
    simulation.step(Int64.ofInt(0));
    check(!machine.isMoving(), "zero-distance home completes deterministically");

    machine.moveAxes([new AxisTarget("x", 0.04)], new MotionOptions(0.08, 0.4));
    var tick = 1;
    while (machine.isMoving()) {
      machine.update();
      simulation.step(Int64.ofInt(tick++));
      if (tick > 1000) throw "single-axis trajectory did not complete";
    }
    for (_ in 0...4) simulation.step(Int64.ofInt(tick++));
    var snapshot = robot.snapshot();
    near(snapshot.positions.get(0), 0.04, "simulated robot reaches the MotionKit axis target", 1e-5);
    simulation.dispose();
  }

  static function testCompiledXYZGantryRunsThroughSimulation():Void {
    var xAxis = new LinearAxis(23, 10, 80);
    var yAxis = new LinearAxis(23, 10, 60);
    var zAxis = new LinearAxis(23, 10, 40);
    var blueprint = MachineKitRobotCompiler.compileXYZGantry(xAxis, yAxis, zAxis, 0.1, 0.4);
    check(blueprint.model.links.length == 4, "XYZ gantry compiles one base and three carriages");
    check(blueprint.model.joints.length == 3, "XYZ gantry compiles three prismatic joints");
    for (i in 0...3) {
      var joint = blueprint.model.joints[i];
      check(joint.id == ["x", "y", "z"][i], "XYZ gantry uses stable joint IDs");
      check(joint.type == robotkit.model.JointType.Prismatic,
        "XYZ gantry joints are prismatic");
      near(joint.limits.velocity, 0.1, "XYZ gantry retains the actuator rate limit");
    }
    near(blueprint.model.joints[0].parentFramePosition[0],
      (xAxis.screwStart + xAxis.travelMin) * 0.001, "X carriage frame is compiled in metres");
    near(blueprint.model.joints[1].parentFramePosition[1],
      (yAxis.screwStart + yAxis.travelMin) * 0.001, "Y carriage frame is compiled in metres");
    near(blueprint.model.joints[2].parentFramePosition[2],
      (zAxis.screwStart + zAxis.travelMin) * 0.001, "Z carriage frame is compiled in metres");

    var simulation = new Simulation(0.01);
    var runtime = simulation.addRobot(blueprint.runtime);
    var robot = new SimulatedRobot("xyz-gantry", runtime, blueprint.model.name,
      [for (link in blueprint.model.links) link.name],
      [for (joint in blueprint.model.joints) joint.name]);
    var machine = MotionSystem.fromBlueprint(robot, blueprint);
    check(machine.axis("x") != null && machine.axis("y") != null && machine.axis("z") != null,
      "XYZ gantry exposes all logical axes");

    machine.home();
    runMotion(machine, simulation);
    var home = robot.snapshot();
    near(home.positions.get(0), 0.0, "XYZ gantry homes X");
    near(home.positions.get(1), 0.0, "XYZ gantry homes Y");
    near(home.positions.get(2), 0.0, "XYZ gantry homes Z");
    throws(function() machine.moveAxes([new AxisTarget("x", 0.081)]),
      "XYZ gantry rejects an out-of-range axis target");

    machine.moveAxes([
      new AxisTarget("x", 0.02), new AxisTarget("y", 0.01), new AxisTarget("z", 0.015)
    ], new MotionOptions(0.08, 0.4));
    runMotion(machine, simulation);
    var firstMove = robot.snapshot();
    near(firstMove.positions.get(0), 0.02, "XYZ gantry reaches X axis target", 1e-5);
    near(firstMove.positions.get(1), 0.01, "XYZ gantry reaches Y axis target", 1e-5);
    near(firstMove.positions.get(2), 0.015, "XYZ gantry reaches Z axis target", 1e-5);

    var linear = machine.moveLinear(Pose.xyz(0.03, 0.02, 0.025), Feed.mmPerSecond(50));
    var midpoint = linear.sample(linear.durationSeconds * 0.5);
    var xAlpha = (midpoint.positions[0] - 0.02) / 0.01;
    var yAlpha = (midpoint.positions[1] - 0.01) / 0.01;
    var zAlpha = (midpoint.positions[2] - 0.015) / 0.01;
    near(xAlpha, yAlpha, "Cartesian move preserves the X/Y line", 1e-5);
    near(xAlpha, zAlpha, "Cartesian move preserves the X/Z line", 1e-5);
    runMotion(machine, simulation);
    var linearMove = robot.snapshot();
    near(linearMove.positions.get(0), 0.03, "Cartesian move reaches X target", 1e-5);
    near(linearMove.positions.get(1), 0.02, "Cartesian move reaches Y target", 1e-5);
    near(linearMove.positions.get(2), 0.025, "Cartesian move reaches Z target", 1e-5);
    simulation.dispose();
  }

  static function runMotion(machine:MotionSystem, simulation:Simulation):Void {
    var tick = 0;
    while (machine.isMoving()) {
      machine.update();
      simulation.step(Int64.ofInt(tick++));
      if (tick > 2000) throw "MotionKit trajectory did not complete";
    }
    for (_ in 0...4) simulation.step(Int64.ofInt(tick++));
  }

  static function check(value:Bool, message:String):Void {
    if (!value) throw message;
    assertions++;
  }

  static function near(actual:Float, expected:Float, message:String,
      tolerance:Float = 1e-6):Void {
    check(Math.abs(actual - expected) <= tolerance * Math.max(1.0, Math.abs(expected)),
      '$message: $actual != $expected');
  }

  static function throws(action:Void -> Void, message:String):Void {
    var didThrow = false;
    try action() catch (_:Dynamic) didThrow = true;
    check(didThrow, message);
  }
}
