package tests;

import RobotKitRuntime;
import haxe.Int64;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.SimulationHarness;
import robotkit.tool.SuctionChannels;
import robotkit.tool.WeldChannels;
import robotkit.world.ExecutionPlanSubmission;
import robotkit.world.ProcessEventValue;
import robotkit.world.ProcessTimedEvent;
import robotkit.world.TrajectorySegment;

/**
 * The welder's channels in the real RobotKit runtime, declared with the same `WeldChannels.declarations` the app declares
 * them with: whatever stops the robot, the arc goes off and the wire stops; a suction tool on the same robot keeps its
 * vacuum through a commanded stop and an abort, as it does on a mobile base. The runtime enforces this on its own, so
 * it holds for every way of running the robot, not only the application's.
 */
class WeldChannelTests {
  static var assertions = 0;
  static inline var ARC = "torch.arc";
  static inline var WIRE = "torch.wire_speed";
  static inline var VOLTAGE = "torch.voltage";
  static inline var SUCTION = "tool.vacuum";
  static inline var TICK = 10000000;

  public static function run():Int {
    assertions = 0;
    testStopsTakeTheArcOff();
    testNothingChangesWithoutAStop();
    Sys.println('RobotKit weld channel tests passed ($assertions assertions)');
    return assertions;
  }

  /** A one-joint robot with the welder's channels and a suction tool's, and a plan that switches all of them on. */
  static function rig(plan:Int):{harness:SimulationHarness, runtime:RobotRuntime} {
    var model = new RobotModel("welder-channels");
    var base = model.addLink(new Link("base"));
    var tool = model.addLink(new Link("tool"));
    var joint = model.addJoint(new Joint("axis", JointType.Revolute, base, tool));
    joint.limits.lower = -1.0;
    joint.limits.upper = 1.0;
    var blueprint = RobotRuntimeCompiler.compile(model);
    for (declaration in WeldChannels.declarations(ARC, WIRE, VOLTAGE)) blueprint.channels.push(declaration);
    blueprint.channels.push(SuctionChannels.declaration(SUCTION));
    var harness = new SimulationHarness();
    var runtime = harness.simulation.addRobot(blueprint);
    var segment = new TrajectorySegment(Int64.ofInt(0), Int64.ofInt(500000000), [[0.0, 0.0]]);
    var at = Int64.ofInt(50000000);
    runtime.submitPlan(new ExecutionPlanSubmission(Int64.ofInt(plan), Int64.ofInt(blueprint.revision),
      Int64.ofInt(blueprint.calibrationRevision), RobotKitRuntimeConstants.RK_PLAN_CAPABILITY_TRAJECTORY_QUEUE,
      [0.0], [0.0], [0.0], [segment], null, null, null, null, null, true,
      [new ProcessTimedEvent(at, VOLTAGE, ProcessEventValue.Analog(24.0)),
       new ProcessTimedEvent(at, WIRE, ProcessEventValue.Analog(8.0)),
       new ProcessTimedEvent(at, ARC, ProcessEventValue.Digital(true)),
       new ProcessTimedEvent(at, SUCTION, ProcessEventValue.Digital(true))]), 1);
    for (tick in 1...11) harness.step(Int64.ofInt(tick * TICK));
    return {harness: harness, runtime: runtime};
  }

  static function digital(runtime:RobotRuntime, channel:String):Bool
    return switch runtime.channelValue(channel) {
      case Digital(on): on;
      case _: throw 'Channel "$channel" is not digital';
    };

  static function analog(runtime:RobotRuntime, channel:String):Float
    return switch runtime.channelValue(channel) {
      case Analog(value): value;
      case _: throw 'Channel "$channel" is not analog';
    };

  /** Whether the arc, wire, voltage and vacuum are all up, as the plan set them. */
  static function running(runtime:RobotRuntime):Bool
    return digital(runtime, ARC) && analog(runtime, WIRE) == 8.0 && analog(runtime, VOLTAGE) == 24.0 && digital(runtime, SUCTION);

  static function testStopsTakeTheArcOff():Void {
    // A commanded stop and an abort end the welding motion: the arc goes off and the wire stops. The voltage setpoint and
    // the suction tool keep what they had.
    for (case_ in [{label: "a commanded stop", abort: false, emergency: false},
        {label: "an abort", abort: true, emergency: false}]) {
      var rig = rig(900);
      check(running(rig.runtime), 'before ${case_.label} the arc burns, the wire feeds and the vacuum is on');
      if (case_.abort) rig.runtime.submitAbort(2);
      else rig.runtime.submitStop(2, false);
      for (tick in 11...90) rig.harness.step(Int64.ofInt(tick * TICK));
      check(!digital(rig.runtime, ARC), '${case_.label} takes the arc off');
      check(analog(rig.runtime, WIRE) == 0.0, '${case_.label} stops the wire');
      check(analog(rig.runtime, VOLTAGE) == 24.0, '${case_.label} keeps the voltage setpoint');
      check(digital(rig.runtime, SUCTION), '${case_.label} keeps a suction tool holding');
      rig.harness.dispose();
    }
    // An emergency stop takes every channel to its safe value, the setpoint and the vacuum too.
    var emergency = rig(901);
    emergency.runtime.submitStop(2, true);
    for (tick in 11...20) emergency.harness.step(Int64.ofInt(tick * TICK));
    check(!digital(emergency.runtime, ARC) && analog(emergency.runtime, WIRE) == 0.0, "an emergency stop takes the arc off and stops the wire");
    check(analog(emergency.runtime, VOLTAGE) == 0.0 && !digital(emergency.runtime, SUCTION), "an emergency stop takes every channel safe");
    emergency.harness.dispose();
    // A robot fault does the same: here a position target beyond the joint's limit, which the runtime refuses and latches.
    var faulted = rig(902);
    faulted.runtime.submitPositions([5.0], 2);
    for (tick in 11...20) faulted.harness.step(Int64.ofInt(tick * TICK));
    check(faulted.runtime.snapshot().safety == RobotKitRuntimeConstants.RK_SAFETY_FAULT, "the refused target faults the robot");
    check(!digital(faulted.runtime, ARC) && analog(faulted.runtime, WIRE) == 0.0, "a robot fault takes the arc off and stops the wire");
    check(analog(faulted.runtime, VOLTAGE) == 0.0 && !digital(faulted.runtime, SUCTION), "a robot fault takes every channel safe");
    faulted.harness.dispose();
  }

  /** The control: the channels hold their values while the robot runs and nothing stops it. */
  static function testNothingChangesWithoutAStop():Void {
    var rig = rig(903);
    for (tick in 11...40) rig.harness.step(Int64.ofInt(tick * TICK));
    check(running(rig.runtime), "without a stop the arc keeps burning");
    rig.harness.dispose();
  }

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'Assertion failed: $message';
  }
}
