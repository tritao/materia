package humanoid;

import RobotKitRuntime;
import haxe.Int64;
import machinekit.assembly.LinearAxis;
import motionkit.robot.MachineKitRobotCompiler;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.model.RobotModelCodec;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.Simulation;
import robotkit.runtime.SimulationHarness;
import robotkit.runtime.SimulationSpace;

/**
 * Mixed-scene safety (robotkit/plans/MIXED_SCENE.md): a UR-class arm, an XYZ
 * CNC gantry and an imported floating-base humanoid (Unitree G1 from
 * fetch-g1.sh) share one MuJoCo session.
 *
 * 1. The arm and the gantry run a scripted job alone, and again with the G1
 *    dropped 10 m away, toppling and faulting on its joint stops while a
 *    policy-like client keeps commanding it. Every step must succeed and
 *    neither may fault. The gantry must match bit for bit; the arm, whose own
 *    links touch, to solver round-off.
 * 2. The G1 is dropped onto the gantry's bed and onto the arm's base. Its feet
 *    must land on them, as they land on the floor.
 *
 * Usage: haxeon run --project robotkit/tools/humanoid/haxeon.json -- mixed <g1 robot.json>
 */
class MixedScene {
  static inline var TIMESTEP = 0.01;
  static inline var SUBSTEPS = 5;
  static inline var TICKS = 300;
  static var failures = 0;

  static function arm():RobotRuntimeBlueprint {
    var model = new RobotModel("mixed-scene arm");
    var base = model.addLink(new Link("base", "link/base"));
    var names = ["shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link"];
    var links = [base].concat([for (name in names) model.addLink(new Link(name, 'link/$name'))]);
    var offsets = [[0.0, 0.0, 0.089159], [0.0, 0.13585, 0.0], [0.0, -0.1197, 0.425],
      [0.0, 0.0, 0.39225], [0.0, 0.10915, 0.0], [0.0, 0.0, 0.09465]];
    var axes = [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [0.0, 1.0, 0.0],
      [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]];
    for (i in 0...6) {
      var joint = model.addJoint(new Joint('joint$i', JointType.Revolute, links[i], links[i + 1], 'joint/$i'));
      joint.parentFramePosition = offsets[i];
      joint.axis = axes[i];
      joint.limits.lower = -2.0 * Math.PI;
      joint.limits.upper = 2.0 * Math.PI;
      joint.limits.velocity = null;
      joint.limits.maxAcceleration = 0.3;
    }
    return RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile());
  }

  static function gantry():RobotRuntimeBlueprint {
    return MachineKitRobotCompiler.compileXYZGantry(new LinearAxis(23, 10, 200),
      new LinearAxis(23, 10, 200), new LinearAxis(23, 10, 200), 0.02, 0.08).runtime;
  }

  /** The scripted job: positions for the arm's and the gantry's joints at a tick. */
  static function armTargets(tick:Int):Array<Float> {
    var t = tick * TIMESTEP;
    return [0.4 * Math.sin(t), -1.0 + 0.3 * Math.sin(1.7 * t), 1.3 + 0.2 * Math.sin(0.7 * t),
      -0.3, 0.5 + 0.2 * Math.sin(2.0 * t), 0.0];
  }

  static function gantryTargets(blueprint:RobotRuntimeBlueprint, tick:Int):Array<Float> {
    var t = tick * TIMESTEP;
    return [for (i in 0...3) {
      var joint = blueprint.joints[i];
      var mid = 0.5 * (joint.lowerLimit + joint.upperLimit), span = 0.3 * (joint.upperLimit - joint.lowerLimit);
      mid + span * Math.sin((1.1 + 0.6 * i) * t);
    }];
  }

  static function robotsTrace(simulation:Simulation, robots:Array<RobotRuntime>, links:Array<Int>):Array<Float> {
    var values:Array<Float> = [];
    for (index in 0...robots.length) {
      var snapshot = robots[index].snapshot();
      for (i in 0...snapshot.q.length) {
        values.push(snapshot.q.get(i));
        values.push(snapshot.dq.get(i));
        values.push(snapshot.effort.get(i));
      }
      for (link in 0...links[index]) {
        var pose = simulation.linkPose(index, link);
        values = values.concat(pose.position).concat(pose.rotation);
      }
    }
    return values;
  }

  static function check(value:Bool, message:String):Void {
    if (!value) {
      failures++;
      Sys.println('FAIL: $message');
    }
  }

  /** Runs the job; with `humanoid`, the G1 falls 10 m away and is commanded every tick. */
  static function job(humanoid:Null<RobotModel>):{trace:Array<Float>, failedSteps:Int,
      safeties:Array<Int>, humanoidSafety:Int} {
    var harness = new SimulationHarness(TIMESTEP, SUBSTEPS, SimulationSpace.MUJOCO);
    var simulation = harness.simulation;
    var armBlueprint = arm(), gantryBlueprint = gantry();
    var armRuntime = simulation.addRobotAtPose(armBlueprint, [0.0, 0.0, 0.5], [0.0, 0.0, 0.0, 1.0]);
    var gantryRuntime = simulation.addRobotAtPose(gantryBlueprint, [3.0, 0.0, 0.5], [0.0, 0.0, 0.0, 1.0]);
    var present = [armRuntime, gantryRuntime];
    var counts = [7, 4];
    var g1:Null<RobotRuntime> = null;
    var g1Zero:Array<Float> = [];
    if (humanoid != null) {
      // The compiler's default of no observed-limit tolerance: a G1 pressed onto
      // a joint stop faults, as it does in an application scene.
      var blueprint = RobotRuntimeCompiler.compile(humanoid, new robotkit.profile.RobotProfile());
      // Dropped tilted 0.6 rad forward, so that it topples and faults.
      g1 = simulation.addRobotAtPose(blueprint, [10.0, 0.0, 0.8], [0.0, Math.sin(0.3), 0.0, Math.cos(0.3)]);
      g1Zero = [for (_ in humanoid.joints) 0.0];
    }
    harness.spawnPlane();
    var failed = 0;
    var trace:Array<Float> = [];
    var sequence = 1;
    for (tick in 0...TICKS) {
      armRuntime.submitPositions(armTargets(tick), sequence);
      gantryRuntime.submitPositions(gantryTargets(gantryBlueprint, tick), sequence);
      // A policy-like client: a fresh batch every tick, also after the fall.
      if (g1 != null) try g1.submitPositions(g1Zero, sequence) catch (error:Dynamic) {}
      sequence++;
      try harness.step(Int64.ofInt(tick)) catch (error:Dynamic) failed++;
      trace = trace.concat(robotsTrace(simulation, present, counts));
    }
    var result = {trace: trace, failedSteps: failed,
      safeties: [for (runtime in present) runtime.snapshot().safety],
      humanoidSafety: g1 == null ? -1 : g1.snapshot().safety};
    harness.dispose();
    return result;
  }

  /** Drops the G1 onto a robot link; returns the G1's base height after `seconds`. */
  static function dropOnto(humanoid:RobotModel, name:String, surfaceTop:Float, seconds:Float):Float {
    var harness = new SimulationHarness(TIMESTEP, SUBSTEPS, SimulationSpace.MUJOCO);
    var simulation = harness.simulation;
    var machine:Null<RobotRuntime> = null;
    var blueprint:RobotRuntimeBlueprint;
    if (name == "gantry bed") {
      blueprint = gantry();
    } else {
      blueprint = arm();
    }
    var boxes:Array<Null<Array<Float>>> = [for (index in 0...blueprint.links.length)
      index == 0 ? [0.4, 0.4, 0.05] : null];
    // The bed or base link is centred on the robot's origin, 0.5 m up: top at 0.55.
    machine = simulation.addRobotAtPose(blueprint, [3.0, 0.0, 0.5], [0.0, 0.0, 0.0, 1.0], null, boxes);
    var g1Blueprint = RobotRuntimeCompiler.compile(humanoid, new robotkit.profile.RobotProfile());
    g1Blueprint.observedLimitTolerance = 0.05;
    // G1's authored base height puts its feet on the floor: 0.793 m up.
    var g1 = simulation.addRobotAtPose(g1Blueprint, [3.0, 0.0, surfaceTop + 0.793 + 0.02],
      [0.0, 0.0, 0.0, 1.0]);
    harness.spawnPlane();
    g1.submitPositions([for (_ in humanoid.joints) 0.0], 1);
    var ticks = Std.int(Math.round(seconds / TIMESTEP));
    for (tick in 0...ticks) harness.step(Int64.ofInt(tick));
    var height = simulation.robotPose(1).position[2];
    harness.dispose();
    return height;
  }

  public static function run(args:Array<String>):Void {
    if (args.length < 1) {
      Sys.println("usage: mixed <g1 robot.json>");
      Sys.exit(2);
    }
    var g1 = RobotModelCodec.decode(sys.io.File.getBytes(args[0]));
    if (!g1.floatingBase) throw "mixed needs a floating-base robot";

    var alone = job(null);
    check(alone.failedSteps == 0, 'the arm and the gantry alone fail ${alone.failedSteps} steps');
    var mixed = job(g1);
    check(alone.trace.length == mixed.trace.length, "the traces differ in length");
    // Both robots share one MuJoCo solve with the G1, so their trajectories
    // match to solver round-off (see MIXED_SCENE.md).
    var armWorst = 0.0, gantryWorst = 0.0, armLength = 6 * 3 + 7 * 7, record = armLength + 3 * 3 + 4 * 7;
    var armTravel = 0.0;
    for (tick in 0...TICKS)
      armTravel = Math.max(armTravel, Math.abs(mixed.trace[tick * record] - mixed.trace[0]));
    check(armTravel > 0.05, 'the mixed-scene arm stayed frozen (joint travel $armTravel rad)');
    for (i in 0...alone.trace.length) {
      var difference = Math.abs(alone.trace[i] - mixed.trace[i]);
      if (i % record < armLength) armWorst = Math.max(armWorst, difference);
      else gantryWorst = Math.max(gantryWorst, difference);
    }
    Sys.println('G1 toppled (safety ${mixed.humanoidSafety}); failed steps ${mixed.failedSteps} of $TICKS; '
      + 'arm and gantry safety ${mixed.safeties}; trace difference: arm $armWorst, gantry $gantryWorst');
    check(mixed.humanoidSafety == RobotKitRuntimeConstants.RK_SAFETY_FAULT, "the G1 should have faulted");
    check(mixed.failedSteps == 0, 'a G1 fault failed ${mixed.failedSteps} session steps');
    check(mixed.safeties[0] != RobotKitRuntimeConstants.RK_SAFETY_FAULT &&
      mixed.safeties[1] != RobotKitRuntimeConstants.RK_SAFETY_FAULT, "the arm or the gantry faulted");
    check(gantryWorst < 1e-9, 'the gantry trajectory changed by $gantryWorst with the G1 in the session');
    check(armWorst < 1e-9, 'the arm trajectory changed by $armWorst with the G1 in the session');

    for (name in ["gantry bed", "arm base"]) {
      var height = dropOnto(g1, name, 0.55, 0.5);
      Sys.println('G1 dropped onto the $name: base height ${height} after 0.5 s (0.55 + 0.79 = 1.34 on it, 0.79 through it)');
      check(height > 1.2, 'the G1 fell through the $name (height $height)');
    }
    if (failures > 0) {
      Sys.println('FAIL: $failures mixed-scene check(s)');
      Sys.exit(1);
    }
    Sys.println("PASS: arm, CNC gantry and G1 share one session without disturbing each other");
  }
}
