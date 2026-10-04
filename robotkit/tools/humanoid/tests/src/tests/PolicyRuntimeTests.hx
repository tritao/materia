package tests;

import haxe.Int64;
import robotkit.model.RobotModelCodec;
import robotkit.policy.GravityEstimator;
import robotkit.policy.ObservationBuilder;
import robotkit.policy.PolicyController;
import robotkit.policy.PolicySpec;
import robotkit.policy.VelocityReference;
import robotkit.runtime.RobotRuntimeCompiler;

/** The parts of the policy runtime that need no physics: spec, observation, command reference, gravity filter. */
class PolicyRuntimeTests {
  public static function run():Void {
    specParsesAndValidates();
    observationLayoutMatchesTraining();
    historyStacksOldestFirst();
    velocityReferenceRampsExpiresAndRejects();
    gravityEstimatorFollowsGyroAndIgnoresShocks();
    controllerChecksSpecAgainstModelAndNetwork();
    Sys.println("Policy runtime tests passed");
  }

  static function ns(seconds:Float):Int64 return Int64.fromFloat(seconds * 1e9 + 0.5);
  static function near(a:Float, b:Float, tolerance:Float):Bool return Math.abs(a - b) <= tolerance;

  static function specPath():String return Files.robotkit("tools/humanoid/policies/unitree-g1/policy.json");
  static function g1Spec():PolicySpec return PolicySpec.load(specPath());

  static function expectThrow(action:() -> Void, what:String):Void {
    var threw = false;
    try action() catch (_:Dynamic) threw = true;
    if (!threw) throw 'expected $what to be rejected';
  }

  static function specParsesAndValidates():Void {
    var spec = g1Spec();
    if (spec.joints.length != 12 || spec.observationSize() != 47 || spec.controlPeriod != 0.02)
      throw "the G1 spec did not parse to 12 joints and a 47-value observation";
    if (spec.kp[3] != 150.0 || spec.kd[3] != 4.0 || spec.defaultPose[3] != 0.3 || spec.actionScale != 0.25)
      throw "the G1 spec gains or default pose are wrong";
    if (spec.source.get("licence").indexOf("BSD-3-Clause") < 0) throw "the spec must name the policy's licence";
    var text = sys.io.File.getContent(specPath());
    expectThrow(() -> PolicySpec.parse(StringTools.replace(text, "\"version\": 1", "\"version\": 2")), "an unknown version");
    expectThrow(() -> PolicySpec.parse(StringTools.replace(text, "\"jointVelocity\"", "\"jointJerk\"")), "an unknown term");
    expectThrow(() -> PolicySpec.parse(StringTools.replace(text, "\"controlPeriod\": 0.02", "\"controlPeriod\": 0")), "a zero period");
    expectThrow(() -> PolicySpec.parse(StringTools.replace(text, "\"actionScale\"", "\"actionScaleX\"")), "a missing field");
  }

  /** The layout of unitree_rl_gym's deploy_mujoco.py, term by term. */
  static function observationLayoutMatchesTraining():Void {
    var spec = g1Spec();
    var builder = new ObservationBuilder(spec);
    var q = [for (i in 0...12) spec.defaultPose[i] + 0.01 * (i + 1)];
    var dq = [for (i in 0...12) 0.1 * (i + 1)];
    var action = [for (i in 0...12) 0.5 - 0.1 * i];
    var obs = builder.build({
      q: q, dq: dq, angularVelocity: [1.0, -2.0, 4.0], down: [0.0, 0.1, -0.99],
      command: {vx: 0.5, vy: -0.25, wz: 1.0}, lastAction: action, gaitTime: 0.2
    });
    if (obs.length != 47) throw 'observation length ${obs.length}';
    var expect = function(index:Int, value:Float, what:String) {
      if (!near(obs[index], value, 1e-12)) throw '$what: obs[$index] = ${obs[index]}, expected $value';
    };
    expect(0, 0.25, "gyro x scale"); expect(1, -0.5, "gyro y"); expect(2, 1.0, "gyro z");
    expect(3, 0.0, "gravity x"); expect(4, 0.1, "gravity y"); expect(5, -0.99, "gravity z");
    expect(6, 1.0, "command vx * 2"); expect(7, -0.5, "command vy * 2"); expect(8, -0.25, "command wz * -0.25 (the policy turns clockwise for a positive input)");
    expect(9, 0.01, "q - default 0"); expect(20, 0.12, "q - default 11");
    expect(21, 0.1 * 1 * 0.05, "dq 0"); expect(32, 0.1 * 12 * 0.05, "dq 11");
    expect(33, 0.5, "last action 0"); expect(44, 0.5 - 1.1, "last action 11");
    expect(45, Math.sin(2.0 * Math.PI * 0.25), "sin phase (0.2 s of a 0.8 s period)");
    expect(46, Math.cos(2.0 * Math.PI * 0.25), "cos phase");
    // The clock wraps at the period.
    var wrapped = new ObservationBuilder(spec).build({
      q: q, dq: dq, angularVelocity: [0.0, 0.0, 0.0], down: [0.0, 0.0, -1.0],
      command: {vx: 0.0, vy: 0.0, wz: 0.0}, lastAction: action, gaitTime: 0.8 * 3 + 0.2
    });
    if (!near(wrapped[45], Math.sin(2.0 * Math.PI * 0.25), 1e-9)) throw "the gait clock did not wrap at the period";
  }

  static function historyStacksOldestFirst():Void {
    var text = sys.io.File.getContent(specPath());
    var spec = PolicySpec.parse(StringTools.replace(text, "\"historyLength\": 1", "\"historyLength\": 3"));
    var builder = new ObservationBuilder(spec);
    var frame = function(gyro:Float) return builder.build({
      q: spec.defaultPose, dq: [for (_ in 0...12) 0.0], angularVelocity: [gyro, 0.0, 0.0], down: [0.0, 0.0, -1.0],
      command: {vx: 0.0, vy: 0.0, wz: 0.0}, lastAction: [for (_ in 0...12) 0.0], gaitTime: 0.02
    });
    var first = frame(4.0);
    if (first.length != 141 || first[0] != 1.0 || first[47] != 1.0 || first[94] != 1.0)
      throw "the first observation must repeat until the history fills";
    frame(8.0);
    var third = frame(12.0);
    if (third[0] != 1.0 || third[47] != 2.0 || third[94] != 3.0) throw "history is not oldest first";
    var fourth = frame(16.0);
    if (fourth[0] != 2.0 || fourth[94] != 4.0) throw "the oldest observation did not drop out";
  }

  static function velocityReferenceRampsExpiresAndRejects():Void {
    var reference = new VelocityReference([1.0, 0.5, 1.0], [1.0, 1.0, 2.0]);
    if (reference.sample(ns(0.0)).vx != 0.0) throw "a reference with no command must read zero";
    if (reference.submit({vx: 0.5, vy: 0.0, wz: 0.0}, 1, ns(1.0), ns(0.0)) != null) throw "a fresh command was rejected";
    // 1 m/s^2: half a second to reach 0.5 m/s.
    var vx = 0.0;
    for (i in 1...11) vx = reference.sample(ns(0.02 * i)).vx;
    if (!near(vx, 0.2, 1e-9)) throw 'ramp after 0.2 s: $vx';
    for (i in 11...26) vx = reference.sample(ns(0.02 * i)).vx;
    if (!near(vx, 0.5, 1e-9)) throw 'the reference did not reach its target: $vx';
    // Clamped to the limits.
    if (reference.submit({vx: 5.0, vy: -9.0, wz: 0.0}, 2, ns(2.0), ns(0.5)) != null) throw "an over-limit command must be clamped, not rejected";
    var target = reference.targetCommand();
    if (target.vx != 1.0 || target.vy != -0.5) throw 'limits not applied: ${target.vx}, ${target.vy}';
    // Stale, non-finite and already-expired commands.
    if (reference.submit({vx: 0.1, vy: 0.0, wz: 0.0}, 2, ns(3.0), ns(0.5)) != Stale) throw "a repeated sequence must be stale";
    if (reference.submit({vx: 0.1, vy: 0.0, wz: 0.0}, 1, ns(3.0), ns(0.5)) != Stale) throw "an older sequence must be stale";
    if (reference.submit({vx: Math.NaN, vy: 0.0, wz: 0.0}, 3, ns(3.0), ns(0.5)) != NotFinite) throw "NaN must be rejected";
    if (reference.submit({vx: 0.1, vy: 0.0, wz: 0.0}, 3, ns(0.4), ns(0.5)) != Expired) throw "a past deadline must be rejected";
    if (reference.targetCommand().vx != 1.0) throw "rejected commands must not change the target";
    // The deadline passes: the target drops to zero and the value brakes within the acceleration limit.
    var previous = reference.sample(ns(1.99)).vx;
    var braked = reference.sample(ns(2.0));
    if (!reference.expired || reference.targetCommand().vx != 0.0) throw "the command did not expire at its deadline";
    if (braked.vx >= previous || previous - braked.vx > 1.0 * 0.011 + 1e-9) throw "expiry must brake, within the limit, not jump";
    for (i in 1...80) braked = reference.sample(ns(2.0 + 0.02 * i));
    if (braked.vx != 0.0 || braked.vy != 0.0) throw 'the reference did not brake to zero: ${braked.vx}, ${braked.vy}, ${braked.wz}';
    // A new command after expiry is accepted.
    if (reference.submit({vx: 0.3, vy: 0.0, wz: 0.0}, 4, ns(5.0), ns(3.6)) != null) throw "a command after expiry was rejected";
  }

  static function gravityEstimatorFollowsGyroAndIgnoresShocks():Void {
    var g = 9.81;
    // Rotating about x at 0.5 rad/s: the body tilts, and the accelerometer sees the tilted gravity.
    var estimator = new GravityEstimator();
    var down = estimator.update([0.0, 0.0, 0.0], [0.0, 0.0, g], 0.0);
    if (!near(down[2], -1.0, 1e-12)) throw "at rest the down vector is -z";
    var angle = 0.0, dt = 0.02, worst = 0.0;
    for (step in 0...100) {
      angle += 0.5 * dt;
      // Body tilted by `angle` about x: world down (0,0,-1) in the body frame is (0, -sin, -cos)
      // (R^T applied to it), and the specific force is its negative times g.
      var force = [0.0, g * Math.sin(angle), g * Math.cos(angle)];
      down = estimator.update([0.5, 0.0, 0.0], force, dt);
      var error = Math.acos(Math.max(-1.0, Math.min(1.0, down[1] * -Math.sin(angle) + down[2] * -Math.cos(angle))));
      worst = Math.max(worst, error);
    }
    if (worst > 0.01) throw 'estimator lagged the true tilt by $worst rad';
    // An impulse the accelerometer alone would mistake for tilt: 3 g of push must not tilt the estimate.
    var before = estimator.update([0.5, 0.0, 0.0], [0.0, g * Math.sin(angle), g * Math.cos(angle)], dt);
    var shocked = estimator.update([0.5, 0.0, 0.0], [3.0 * g, g * Math.sin(angle), g * Math.cos(angle)], dt);
    var drift = Math.acos(Math.max(-1.0, Math.min(1.0, before[0] * shocked[0] + before[1] * shocked[1] + before[2] * shocked[2])));
    if (drift > 0.02) throw 'a 3 g shock tilted the estimate by $drift rad';
    var length = Math.sqrt(shocked[0] * shocked[0] + shocked[1] * shocked[1] + shocked[2] * shocked[2]);
    if (!near(length, 1.0, 1e-9)) throw "the estimate must stay a unit vector";
    // A robot that starts leaning 0.3 rad (the first, moving samples are not trusted), then stands still,
    // is found within a second; a moving one is not pulled towards a gravity the accelerometer misreads.
    var still = new GravityEstimator();
    still.update([0.5, 0.0, 0.0], [0.0, g * Math.sin(0.3), g * Math.cos(0.3)], 0.0); // moving: not trusted, starts upright
    var leaning = [0.0, g * Math.sin(0.3), g * Math.cos(0.3)];
    var found = [0.0, 0.0, -1.0];
    for (step in 0...200) found = still.update([0.0, 0.0, 0.0], leaning, 0.002 * 5.0);
    if (!near(found[1], -Math.sin(0.3), 0.01)) throw 'a still robot was not found leaning: ${found[1]}';
    var moving = new GravityEstimator();
    moving.update([0.0, 0.5, 0.0], [0.0, g * Math.sin(0.3), g * Math.cos(0.3)], 0.0);
    var unmoved = [0.0, 0.0, -1.0];
    for (step in 0...200) unmoved = moving.update([0.0, Std.int(step / 25) % 2 == 0 ? 0.5 : -0.5, 0.0], leaning, 0.002 * 5.0); // swings out and back
    if (!near(unmoved[1], 0.0, 0.05)) throw 'a turning robot was pulled towards a misread gravity';
  }

  static function controllerChecksSpecAgainstModelAndNetwork():Void {
    var spec = g1Spec();
    var model = RobotModelCodec.decode(sys.io.File.getBytes(Files.robotkit("tests/fixtures/humanoid/unitree-g1-12dof.robot.json")));
    var policy = robotkit.policy.OnnxPolicy.load(Files.robotkit("tools/humanoid/policies/unitree-g1/policy.onnx"));
    expectThrow(() -> new PolicyController(spec, policy, model), "a model with no IMU");
    PolicyController.prepareModel(model, spec);
    PolicyController.prepareModel(model, spec); // idempotent
    if (model.sensors.length != 1 || model.frames.length != 1) throw "prepareModel must add the IMU once";
    var controller = new PolicyController(spec, policy, model);
    var stand = controller.standTargets();
    if (stand.length != 12 || stand[3].stiffness != 150.0 || stand[3].target != 0.3) throw "stand targets are not the default pose";
    RobotRuntimeCompiler.compile(model, new robotkit.profile.RobotProfile()); // the prepared model compiles
    // A spec naming a joint the model lacks, and a network that does not fit.
    var text = sys.io.File.getContent(specPath());
    var wrongJoint = PolicySpec.parse(StringTools.replace(text, "joint/left_knee_joint", "joint/no_such_joint"));
    expectThrow(() -> new PolicyController(wrongJoint, policy, model), "a spec joint the model lacks");
    var wrongTensor = PolicySpec.parse(StringTools.replace(text, "\"observation\": \"obs\"", "\"observation\": \"observations\""));
    expectThrow(() -> new PolicyController(wrongTensor, policy, model), "a spec tensor the network lacks");
    var wrongHistory = PolicySpec.parse(StringTools.replace(text, "\"historyLength\": 1", "\"historyLength\": 2"));
    expectThrow(() -> new PolicyController(wrongHistory, policy, model), "a history the network was not trained with");
    policy.dispose();
  }
}
