package humanoid;

import haxe.Int64;
import robotkit.model.RobotModel;
import robotkit.model.RobotModelCodec;
import robotkit.policy.OnnxPolicy;
import robotkit.policy.PolicyController;
import robotkit.policy.PolicySession;
import robotkit.policy.PolicySpec;
import robotkit.policy.VelocityReference.VelocityCommand;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.Simulation;
import robotkit.runtime.SimulationHarness;
import robotkit.runtime.SimulationSpace;
import robotkit.world.McapRobotRecording;
import robotkit.world.RobotCommand;
import robotkit.world.RobotRecording;
import robotkit.world.RobotRecordingEvent;
import robotkit.world.RecordingRobot;
import robotkit.world.Robot;
import robotkit.world.SimulatedRobot;

/** A velocity command from `from` seconds on, until the next segment. */
typedef CommandSegment = {from:Float, vx:Float, vy:Float, wz:Float};

/** A horizontal push on the base: `force` newtons from `at` for `duration` seconds. */
typedef Push = {at:Float, force:Array<Float>, duration:Float};

typedef WalkScenario = {
  /** The RobotModel JSON, and the policy spec beside its ONNX file. */
  model:String,
  spec:String,
  seconds:Float,
  commands:Array<CommandSegment>,
  pushes:Array<Push>,
  /** Write an MCAP recording of the run here, or null. */
  record:Null<String>,
  /** Debug only: feed the policy the simulator's own gravity and gyro instead of the IMU estimate. */
  truthObservations:Bool,
  /** Seconds between progress lines; zero for none. */
  report:Float,
  /** Seconds the robot holds its default pose, policy off, before the run starts. */
  warmup:Float
};

/** The simulation, robot and policy of a scenario, before any control. */
typedef WalkRig = {
  model:RobotModel,
  spec:PolicySpec,
  policy:OnnxPolicy,
  harness:SimulationHarness,
  simulation:Simulation,
  runtime:RobotRuntime,
  simulated:SimulatedRobot
};

/** One sample per control tick. */
typedef WalkSample = {t:Float, x:Float, y:Float, z:Float, yaw:Float, tilt:Float, vx:Float, vy:Float, wz:Float};

typedef WalkResult = {
  samples:Array<WalkSample>,
  fell:Bool,
  fallTime:Float,
  worstTilt:Float,
  lowest:Float,
  /** Largest gap between the estimated and true down vector, rad (measured only for reporting). */
  worstEstimateError:Float,
  /** Mean of (estimated - true) down vector over the run after 2 s, per axis: the filter's bias. */
  meanEstimateError:Array<Float>,
  /** Policy targets clamped into a joint's travel, out of all those sent. */
  clampedTargets:Int,
  recording:Null<String>
};

/**
 * Runs a policy-driven humanoid in the MuJoCo simulation through the ordinary
 * runtime: RobotModel to blueprint to Simulation, a PolicySession closing the
 * loop over a Robot, joint servo targets evaluated in every physics substep.
 * The physics step is the model's own (2 ms for G1) inside a 20 ms control tick.
 */
class WalkRun {
  /** The solver settings of Unitree's G1 scene, which robotkit_mjcf_import reports. */
  static inline final INTEGRATOR_EULER = 1;
  static inline final PHYSICS_STEP = 0.002;

  static function build(scenario:WalkScenario):WalkRig {
    var model = RobotModelCodec.decode(sys.io.File.getBytes(scenario.model));
    var spec = PolicySpec.load(scenario.spec);
    var specDirectory = haxe.io.Path.directory(scenario.spec);
    var policy = OnnxPolicy.load(haxe.io.Path.join([specDirectory, spec.model]));
    PolicyController.prepareModel(model, spec);
    var blueprint = RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile());
    // A compliant stop always gives a little; the policy leans on its stops.
    blueprint.observedLimitTolerance = 0.05;

    // One simulation tick is one physics step of 2 ms, the IMU's period; the policy runs every 10th.
    var harness = new SimulationHarness(PHYSICS_STEP, 1, SimulationSpace.MUJOCO,
      {integrator: INTEGRATOR_EULER, solverIterations: 100, lineSearchIterations: 50});
    var simulation = harness.simulation;
    var runtime = simulation.addRobotAtPose(blueprint, [0.0, 0.0, 0.793], [0.0, 0.0, 0.0, 1.0]);
    harness.spawnPlane();
    var simulated = new SimulatedRobot("g1", runtime, model.name, [for (link in model.links) link.name],
      [for (joint in model.joints) joint.name]);
    return {model: model, spec: spec, policy: policy, harness: harness, simulation: simulation, runtime: runtime, simulated: simulated};
  }

  public static function run(scenario:WalkScenario):WalkResult {
    var rig = build(scenario);
    var model = rig.model, spec = rig.spec, policy = rig.policy, simulation = rig.simulation, simulated = rig.simulated;
    var writer:Null<McapRobotRecording> = scenario.record == null ? null : new McapRobotRecording(scenario.record, 16 * 1024 * 1024, false);
    var robot:Robot = writer == null ? simulated : new RecordingRobot(simulated, writer);

    var controller = new PolicyController(spec, policy, model);
    if (scenario.truthObservations)
      controller.debugTruth = function() {
        var pose = simulation.robotPose(0), velocity = simulation.robotBaseVelocity(0);
        return {down: rotateInverse(pose.rotation, [0.0, 0.0, -1.0]), angularVelocity: rotateInverse(pose.rotation, velocity.angular)};
      };
    var session = new PolicySession(robot, controller, null, PHYSICS_STEP);
    session.start(scenario.warmup);

    var result:WalkResult = {samples: [], fell: false, fallTime: 0.0, worstTilt: 0.0, lowest: 9.0, worstEstimateError: 0.0, meanEstimateError: [0.0, 0.0, 0.0], clampedTargets: 0,
      recording: scenario.record};
    var errSum = [0.0, 0.0, 0.0], errCount = 0;
    var ticks = Std.int(Math.round(scenario.seconds / PHYSICS_STEP));
    var nextReport = scenario.report;
    for (tick in 0...ticks) {
      var t = tick * PHYSICS_STEP;
      rig.harness.step(Int64.ofInt(tick));
      // The command is a cyclic reference: refreshed every tick with a deadline a few ticks ahead.
      var segment = scenario.commands[0];
      for (candidate in scenario.commands) if (candidate.from <= t + 1e-9) segment = candidate;
      session.commandFor({vx: segment.vx, vy: segment.vy, wz: segment.wz}, 5.0 * spec.controlPeriod);
      for (push in scenario.pushes)
        if (t + 1e-9 >= push.at && t < push.at + push.duration - 1e-9) simulation.applyRobotForce(0, push.force);
      session.update();

      var pose = simulation.robotPose(0), velocity = simulation.robotBaseVelocity(0);
      var up = rotate(pose.rotation, [0.0, 0.0, 1.0]);
      var tilt = Math.acos(Math.max(-1.0, Math.min(1.0, up[2])));
      var local = rotateInverse(pose.rotation, velocity.linear);
      var q = pose.rotation;
      var yaw = Math.atan2(2.0 * (q[3] * q[2] + q[0] * q[1]), 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]));
      result.samples.push({t: (tick + 1) * PHYSICS_STEP, x: pose.position[0], y: pose.position[1], z: pose.position[2],
        yaw: yaw, tilt: tilt, vx: local[0], vy: local[1], wz: rotateInverse(pose.rotation, velocity.angular)[2]});
      result.worstTilt = Math.max(result.worstTilt, tilt);
      if (t > 0.5) result.lowest = Math.min(result.lowest, pose.position[2]);
      if (controller.ready()) {
        var truth = rotateInverse(pose.rotation, [0.0, 0.0, -1.0]), estimate = controller.estimatedDown();
        var dot = truth[0] * estimate[0] + truth[1] * estimate[1] + truth[2] * estimate[2];
        result.worstEstimateError = Math.max(result.worstEstimateError, Math.acos(Math.max(-1.0, Math.min(1.0, dot))));
        if (t > 2.0) { for (a in 0...3) errSum[a] += estimate[a] - truth[a]; errCount++; }
      }
      if (scenario.report > 0.0 && t >= nextReport) {
        nextReport += scenario.report;
        Sys.println('t=${round(t, 2)}: x=${round(pose.position[0], 3)} y=${round(pose.position[1], 3)} z=${round(pose.position[2], 3)} '
          + 'tilt=${round(tilt, 3)} v=(${round(local[0], 2)}, ${round(local[1], 2)}) wz=${round(result.samples[result.samples.length - 1].wz, 2)}');
      }
      if (pose.position[2] < 0.35 || tilt > 1.0) {
        result.fell = true;
        result.fallTime = t;
        break;
      }
    }
    result.clampedTargets = controller.clampedTargets;
    if (errCount > 0) result.meanEstimateError = [for (v in errSum) round(v / errCount, 4)];
    if (writer != null) writer.close();
    robot.close();
    rig.harness.dispose();
    policy.dispose();
    return result;
  }

  /**
   * Feeds a recording's joint targets back into a fresh simulation, open loop
   * and with no policy, stepping once per recorded snapshot, and returns the
   * largest gaps to the recorded joint positions (rad) and to the recorded
   * snapshot times (s).
   */
  public static function replay(scenario:WalkScenario, recording:RobotRecording):{steps:Int, worstJoint:Float, worstTime:Float} {
    var rig = build(scenario);
    var steps = 0, worstJoint = 0.0, worstTime = 0.0, sequence = 1;
    for (event in recording.events)
      switch event {
        case Command(JointTargets(targets, _)):
          rig.runtime.submitTargets(targets, sequence++);
        case RobotSnapshot(recorded):
          rig.harness.step(Int64.ofInt(steps++));
          var now = rig.runtime.snapshot();
          for (joint in 0...recorded.positions.length)
            worstJoint = Math.max(worstJoint, Math.abs(now.q.get(joint) - recorded.positions.get(joint)));
          worstTime = Math.max(worstTime, Math.abs(Int64.toInt(now.sourceTimestampNs - recorded.sourceTimestampNs)) * 1e-9);
        case _:
      }
    rig.harness.dispose();
    rig.policy.dispose();
    return {steps: steps, worstJoint: worstJoint, worstTime: worstTime};
  }

  /** Mean base velocity in the base frame over [from, to] seconds. */
  public static function meanVelocity(result:WalkResult, from:Float, to:Float):VelocityCommand {
    var vx = 0.0, vy = 0.0, wz = 0.0, count = 0;
    for (sample in result.samples)
      if (sample.t >= from && sample.t <= to) {
        vx += sample.vx; vy += sample.vy; wz += sample.wz; count++;
      }
    return count == 0 ? {vx: 0.0, vy: 0.0, wz: 0.0} : {vx: vx / count, vy: vy / count, wz: wz / count};
  }

  public static function sampleAt(result:WalkResult, t:Float):WalkSample {
    var best = result.samples[0];
    for (sample in result.samples) if (Math.abs(sample.t - t) < Math.abs(best.t - t)) best = sample;
    return best;
  }

  static function round(value:Float, digits:Int):Float {
    var scale = Math.pow(10.0, digits);
    return Math.round(value * scale) / scale;
  }

  /** Unit xyzw quaternion rotation of v. */
  public static function rotate(q:Array<Float>, v:Array<Float>):Array<Float> {
    var x = q[0], y = q[1], z = q[2], w = q[3];
    var tx = 2.0 * (y * v[2] - z * v[1]), ty = 2.0 * (z * v[0] - x * v[2]), tz = 2.0 * (x * v[1] - y * v[0]);
    return [v[0] + w * tx + y * tz - z * ty, v[1] + w * ty + z * tx - x * tz, v[2] + w * tz + x * ty - y * tx];
  }

  public static function rotateInverse(q:Array<Float>, v:Array<Float>):Array<Float>
    return rotate([-q[0], -q[1], -q[2], q[3]], v);
}
