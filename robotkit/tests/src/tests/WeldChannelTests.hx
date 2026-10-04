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
 * The welder's channels in the real RobotKit runtime, set up only by adding the tools (`RobotRuntimeBlueprint.addTool`):
 * the stop policy comes with the tool, not from the application that sets the robot up. Whatever stops the robot, the arc
 * goes off and the wire stops; a suction tool on the same robot keeps its vacuum through a commanded stop and an abort,
 * as it does on a mobile base. The runtime enforces this on its own, so it holds for every way of running the robot.
 * A setup that declared one of the tool's channels another way is refused.
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
    testSetupCannotOverrideThePolicy();
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
    var blueprint = RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile());
    // The robot is set up with its tools and nothing else: the stop policy comes with each tool.
    blueprint.addTool(new WeldChannels(ARC, WIRE, VOLTAGE));
    blueprint.addTool(new SuctionChannels(SUCTION));
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

  /** A setup that declared the arc to keep its output through a stop is refused by the torch, and one that agrees is accepted. */
  static function testSetupCannotOverrideThePolicy():Void {
    function blueprint():robotkit.runtime.RobotRuntimeBlueprint {
      var model = new RobotModel("policy");
      var base = model.addLink(new Link("base"));
      var tool = model.addLink(new Link("tool"));
      var joint = model.addJoint(new Joint("axis", JointType.Revolute, base, tool));
      joint.limits.lower = -1.0;
      joint.limits.upper = 1.0;
      return RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile());
    }
    var wrong = blueprint();
    wrong.channels.push(new robotkit.world.ProcessChannelDeclaration(ARC, ProcessEventValue.Digital(false), true));
    var refused = false;
    try wrong.addTool(new WeldChannels(ARC, WIRE, VOLTAGE)) catch (_:Dynamic) refused = true;
    check(refused, "an arc declared to keep its output on a stop is refused by the torch that needs it off");
    var wrongSafe = blueprint();
    wrongSafe.channels.push(new robotkit.world.ProcessChannelDeclaration(WIRE, ProcessEventValue.Analog(4.0), false));
    refused = false;
    try wrongSafe.addTool(new WeldChannels(ARC, WIRE, VOLTAGE)) catch (_:Dynamic) refused = true;
    check(refused, "a wire speed declared with another safe value is refused");
    var agreeing = blueprint();
    agreeing.channels.push(new robotkit.world.ProcessChannelDeclaration(ARC, ProcessEventValue.Digital(false), false));
    agreeing.addTool(new WeldChannels(ARC, WIRE, VOLTAGE));
    check(agreeing.channels.length == 3, "a declaration that agrees with the tool is not doubled");
    var plain = blueprint();
    plain.addTool(new WeldChannels(ARC, WIRE, VOLTAGE));
    var kept = [for (channel in plain.channels) if (channel.keepOnStop) channel.id];
    check(kept.join(",") == VOLTAGE, 'only the voltage setpoint keeps its output on a stop, got ${kept.join(",")}');
  }

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'Assertion failed: $message';
  }
}
