import haxe.Int64;
import machinekit.assembly.LinearAxis;
import motionkit.AxisTarget;
import motionkit.MachineKitRobotCompiler;
import motionkit.MotionOptions;
import motionkit.MotionSystem;
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

  static function check(value:Bool, message:String):Void {
    if (!value) throw message;
    assertions++;
  }

  static function near(actual:Float, expected:Float, message:String,
      tolerance:Float = 1e-6):Void {
    check(Math.abs(actual - expected) <= tolerance * Math.max(1.0, Math.abs(expected)),
      '$message: $actual != $expected');
  }
}
