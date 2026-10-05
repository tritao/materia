package tests;

import haxe.Int64;
import robotkit.model.Actuator;
import robotkit.model.ActuatorDrive.HydraulicDrive;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.model.Transmission;
import robotkit.profile.RobotProfile;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.RobotRuntimeCompiler;
import RobotKitRuntime;
import robotkit.runtime.SimulationHarness;
import robotkit.execution.ExecutionPlanSubmission;
import robotkit.execution.ProcessTimedEvent;
import robotkit.execution.ProcessEventValue;
import robotkit.execution.TrajectorySegment;

/** Process coils select measured-travel phases through the shared native cylinder binding. */
class HydraulicRuntimeTests {
  static var assertions:Int = 0;
  public static function main():Void {
    run(0);
    run(1);
    pressureLoss(0);
    pressureLoss(1);
    Sys.println('Hydraulic runtime tests passed ($assertions assertions)');
  }
  public static function pressureLoss(backend:Int):Int {
    var firstAssertion = assertions;
    var model = new RobotModel("hydraulic-no-pressure");
    var base = model.addLink(new Link("base"));
    var ram = model.addLink(new Link("ram"));
    ram.mass = 1;
    ram.inertiaTensor = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01];
    var joint = model.addJoint(new Joint("ram", JointType.Prismatic, base, ram));
    joint.axis = [1, 0, 0];
    joint.limits.lower = 0; joint.limits.upper = 0.1;
    var drive = new HydraulicDrive(0.16, 0.1, 0.1, 0, 0.08, 0.008, 0.07, 0.0685, "A", "B", 1);
    var actuator = new Actuator("cylinder", 0, drive.ratedSpeed,
      Transmission.SimpleTransmission(joint.id, 1, 0));
    actuator.drive = drive;
    model.addActuator(actuator);
    var blueprint = RobotRuntimeCompiler.compile(model, new RobotProfile());
    var harness = new SimulationHarness(0.001, 1, backend);
    try {
      var runtime = harness.simulation.addRobot(blueprint);
      var segment = new TrajectorySegment(Int64.ofInt(0), Int64.ofInt(1000000000), [[0.0]]);
      runtime.submitPlan(new ExecutionPlanSubmission(Int64.ofInt(1401), Int64.ofInt(blueprint.revision),
        Int64.ofInt(blueprint.calibrationRevision), RobotKitRuntimeConstants.RK_PLAN_CAPABILITY_TRAJECTORY_QUEUE |
        RobotKitRuntimeConstants.RK_PLAN_CAPABILITY_EVENTS, [0.0], [0.0], [0.0], [segment],
        null, null, null, null, null, true, [
          new ProcessTimedEvent(Int64.ofInt(1000000), "A", ProcessEventValue.Digital(true))
        ]), 1);
      step(harness, 1, 1001);
      near(position(runtime), 0, 1e-6, 'zero pressure cannot drive the ram backend $backend');
      near(runtime.snapshot().effort.get(0), 0, 1e-6, 'zero pressure supplies no effort backend $backend');
      harness.dispose();
    } catch (error:Dynamic) {
      harness.dispose();
      throw error;
    }
    return assertions - firstAssertion;
  }

  public static function run(backend:Int):Int {
    var firstAssertion = assertions;
    var model = new RobotModel("hydraulic-stroke");
    var base = model.addLink(new Link("base"));
    var ram = model.addLink(new Link("ram"));
    ram.mass = 1;
    ram.inertiaTensor = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01];
    var joint = model.addJoint(new Joint("ram", JointType.Prismatic, base, ram));
    joint.axis = [1, 0, 0];
    joint.limits.lower = 0; joint.limits.upper = 0.1;
    var drive = new HydraulicDrive(0.16, 0.1, 0.1, 10e6, 0.08, 0.008, 0.07, 0.0685, "A", "B", 1);
    var actuator = new Actuator("cylinder", drive.extensionForce(), drive.ratedSpeed,
      Transmission.SimpleTransmission(joint.id, 1, 0));
    actuator.drive = drive;
    model.addActuator(actuator);
    var blueprint = RobotRuntimeCompiler.compile(model, new RobotProfile());
    var harness = new SimulationHarness(0.001, 1, backend);
    try {
      var runtime = harness.simulation.addRobot(blueprint);
      var segment = new TrajectorySegment(Int64.ofInt(0), Int64.ofInt(2000000000), [[0.0]]);
      runtime.submitPlan(new ExecutionPlanSubmission(Int64.ofInt(1400), Int64.ofInt(blueprint.revision),
        Int64.ofInt(blueprint.calibrationRevision), RobotKitRuntimeConstants.RK_PLAN_CAPABILITY_TRAJECTORY_QUEUE |
        RobotKitRuntimeConstants.RK_PLAN_CAPABILITY_EVENTS, [0.0], [0.0], [0.0], [segment],
        null, null, null, null, null, true, [
          new ProcessTimedEvent(Int64.ofInt(1000000), "A", ProcessEventValue.Digital(true)),
          new ProcessTimedEvent(Int64.ofInt(1400000000), "A", ProcessEventValue.Digital(false)),
          new ProcessTimedEvent(Int64.ofInt(1600000000), "B", ProcessEventValue.Digital(true))
        ]), 1);
      step(harness, 1, 251);
      var first = position(runtime);
      step(harness, 251, 501);
      near(position(runtime) - first, 0.020, 0.002, 'approach rate backend $backend');
      step(harness, 501, 1101);
      var working = position(runtime);
      step(harness, 1101, 1301);
      near(position(runtime) - working, 0.0016, 0.0004, 'working rate backend $backend');
      step(harness, 1301, 1501);
      var stopped = position(runtime);
      step(harness, 1501, 1591);
      near(position(runtime), stopped, 0.0003, 'closed centre backend $backend');
      step(harness, 1591, 1701);
      var returning = position(runtime);
      step(harness, 1701, 1901);
      near(position(runtime) - returning, -0.014, 0.002, 'return rate backend $backend');
      runtime.submitStop(2, true);
      step(harness, 1901, 2001);
      var emergency = position(runtime);
      step(harness, 2001, 2101);
      near(position(runtime), emergency, 0.0003, 'safe coils close valve backend $backend');
      harness.dispose();
    } catch (error:Dynamic) {
      harness.dispose();
      throw error;
    }
    return assertions - firstAssertion;
  }
  static function position(runtime:RobotRuntime):Float return runtime.snapshot().q.get(0);
  static function step(harness:SimulationHarness, begin:Int, end:Int):Void
    for (tick in begin...end) harness.step(Int64.ofInt(tick * 1000000));
  static function near(actual:Float, expected:Float, tolerance:Float, message:String):Void {
    assertions++;
    if (!Math.isFinite(actual) || Math.abs(actual - expected) > tolerance)
      throw '$message: expected $expected, got $actual';
  }
}
