package tests;

import haxe.Int64;
import robotkit.model.RobotModel;
import robotkit.model.Link;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Frame;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.manipulation.ChainTip;
import robotkit.manipulation.KinematicChain;
import robotkit.manipulation.JointGroup;
import robotkit.manipulation.InverseKinematics;
import robotkit.manipulation.IKResult;
import robotkit.manipulation.Manipulator;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.Simulation;
import robotkit.world.SimulatedRobot;
import robotkit.world.RobotCommand;

/** M2 acceptance tests for robotkit.manipulation: KinematicChain, Jacobian, IK, Manipulator. */
class KinematicsTests {
  static var assertions = 0;

  public static function run():Int {
    assertions = 0;
    testPlanar2RClosedForm();
    testUR5StyleZeroPose();
    testJacobianNumericVsAnalytic();
    testIKRecoversReachableTargets();
    testIKReportsNonConvergence();
    testToJointTargetsThroughSimulatedRobot();
    Sys.println('RobotKit kinematics tests passed ($assertions assertions)');
    return assertions;
  }

  static function testPlanar2RClosedForm():Void {
    var fixture = build2RFixture(1.0, 0.7);
    var samples = [[0.0, 0.0], [0.4, -0.9], [-1.2, 0.6], [Math.PI * 0.5, Math.PI * 0.25]];
    for (sample in samples) {
      var fk = fixture.chain.forwardKinematics(sample);
      var q1 = sample[0], q2 = sample[1];
      var expectedX = 1.0 * Math.cos(q1) + 0.7 * Math.cos(q1 + q2);
      var expectedY = 1.0 * Math.sin(q1) + 0.7 * Math.sin(q1 + q2);
      check(approx(fk.translation.x, expectedX, 1e-9) && approx(fk.translation.y, expectedY, 1e-9) &&
        approx(fk.translation.z, 0.0, 1e-9),
        "2R planar arm FK matches the closed-form position");
      var rpy = fk.rotation.toRollPitchYaw();
      check(approx(Pose2WrapDiff(rpy.yaw, q1 + q2), 0.0, 1e-9),
        "2R planar arm FK yaw matches q1 + q2");
    }
  }

  static function testUR5StyleZeroPose():Void {
    var fixture = buildUR5Fixture();
    var zero = [for (_ in 0...6) 0.0];
    var fk = fixture.chain.forwardKinematics(zero);
    var expected = Vec3.zero();
    for (offset in fixture.zeroPoseOffsets) expected = expected.add(offset);
    check(approx(fk.translation.x, expected.x, 1e-9) && approx(fk.translation.y, expected.y, 1e-9) &&
      approx(fk.translation.z, expected.z, 1e-9),
      "UR5-style 6R arm FK at zero pose matches the summed published link offsets");
    check(fk.rotation.angularDistance(Quat.identity()) < 1e-9,
      "UR5-style 6R arm has no net rotation at zero pose");
    check(fixture.chain.dofCount() == 6, "UR5-style fixture exposes six degrees of freedom");
  }

  static function testJacobianNumericVsAnalytic():Void {
    var fixture = buildUR5Fixture();
    var q = [0.3, -0.5, 0.8, -0.2, 0.6, -0.4];
    var analytic = fixture.chain.jacobian(q);
    var eps = 1e-6;
    for (i in 0...6) {
      var plus = q.copy(); plus[i] += eps;
      var minus = q.copy(); minus[i] -= eps;
      var fkPlus = fixture.chain.forwardKinematics(plus);
      var fkMinus = fixture.chain.forwardKinematics(minus);
      var linear = fkPlus.translation.sub(fkMinus.translation).scale(1.0 / (2.0 * eps));
      var angular = logMap(fkMinus.rotation, fkPlus.rotation).scale(1.0 / (2.0 * eps));
      check(approx(analytic[0][i], linear.x, 1e-6) && approx(analytic[1][i], linear.y, 1e-6) &&
        approx(analytic[2][i], linear.z, 1e-6),
        'Analytic Jacobian linear column $i matches the numeric central difference');
      check(approx(analytic[3][i], angular.x, 1e-6) && approx(analytic[4][i], angular.y, 1e-6) &&
        approx(analytic[5][i], angular.z, 1e-6),
        'Analytic Jacobian angular column $i matches the numeric central difference');
    }
  }

  static function testIKRecoversReachableTargets():Void {
    var fixture = buildUR5Fixture();
    var group = JointGroup.fromChain(fixture.chain);
    var rng = new Rng(2026);
    for (trial in 0...6) {
      var qTrue = [for (_ in 0...6) (rng.next() - 0.5) * 2.0];
      var target = fixture.chain.forwardKinematics(qTrue);
      // Warm-started near the true configuration, like a real IK caller re-solving
      // from its last known joint state; a fixed cold-start seed can land in a
      // different (still valid) solution branch of this redundant-looking wrist.
      var seed = [for (i in 0...6) qTrue[i] + (rng.next() - 0.5) * 0.2];
      var result = InverseKinematics.solve(fixture.chain, group, target, seed, 1e-4, 1e-3, 100, 0.02);
      check(result.converged, 'IK converges for seeded reachable target $trial');
      check(result.positionError < 1e-4, 'IK position error is within tolerance for target $trial');
      check(result.orientationError < 1e-3, 'IK orientation error is within tolerance for target $trial');
      var achieved = fixture.chain.forwardKinematics(result.q);
      check(approx(achieved.translation.x, target.translation.x, 1e-3) &&
        approx(achieved.translation.y, target.translation.y, 1e-3) &&
        approx(achieved.translation.z, target.translation.z, 1e-3),
        'IK solution forward-kinematics matches the requested target position for trial $trial');
    }
  }

  static function testIKReportsNonConvergence():Void {
    var fixture = buildUR5Fixture();
    var group = JointGroup.fromChain(fixture.chain);
    var unreachable = new Transform3(new Vec3(100.0, 100.0, 100.0), Quat.identity());
    var seed = [for (_ in 0...6) 0.0];
    var result = InverseKinematics.solve(fixture.chain, group, unreachable, seed, 1e-4, 1e-3, 25, 0.02);
    check(!result.converged, "IK reports non-convergence for an unreachable target instead of throwing");
    check(result.iterations == 25, "IK non-convergence result reports the iteration budget it used");
    check(result.q.length == 6, "IK non-convergence result still returns a full joint vector");
  }

  static function testToJointTargetsThroughSimulatedRobot():Void {
    var fixture = buildUR5Fixture();
    var manipulator = new Manipulator(fixture.model, fixture.chain);
    var blueprint = RobotRuntimeCompiler.compile(fixture.model);
    var simulation = new Simulation(0.02);
    var runtime = simulation.addRobot(blueprint);
    var robot = new SimulatedRobot("ur5-fixture", runtime, fixture.model.name,
      [for (link in fixture.model.links) link.name], [for (joint in fixture.model.joints) joint.name]);

    var q = [0.2, -0.3, 0.5, -0.1, 0.4, -0.2];
    var targets = manipulator.toJointTargets(q);
    check(targets.length == 6, "toJointTargets emits one target per chain joint");
    robot.submit(RobotCommand.JointTargets(targets, null));
    simulation.step(Int64.ofInt(0));
    simulation.step(Int64.ofInt(1));
    var snapshot = robot.snapshot();
    for (i in 0...6)
      check(approx(snapshot.positions.get(i), q[i], 1e-6),
        'Joint $i reaches its commanded target through RobotCommand and SimulatedRobot');
    simulation.dispose();
  }

  // -- fixtures --------------------------------------------------------

  static function build2RFixture(l1:Float, l2:Float):{model:RobotModel, chain:KinematicChain} {
    var model = new RobotModel("2r-arm");
    var base = model.addLink(new Link("base"));
    var link1 = model.addLink(new Link("link1"));
    var link2 = model.addLink(new Link("link2"));
    var joint1 = model.addJoint(new Joint("joint1", JointType.Revolute, base, link1));
    joint1.axis = [0.0, 0.0, 1.0];
    var joint2 = model.addJoint(new Joint("joint2", JointType.Revolute, link1, link2));
    joint2.parentFramePosition = [l1, 0.0, 0.0];
    joint2.axis = [0.0, 0.0, 1.0];
    var tip = model.addFrame(new Frame("tip", link2));
    tip.position = [l2, 0.0, 0.0];
    var chain = new KinematicChain(model, base.id, ChainTip.Frame(tip.id));
    return { model: model, chain: chain };
  }

  /** A 6R arm laid out with the axis pattern and published DH-equivalent offsets of a UR5. */
  static function buildUR5Fixture():{model:RobotModel, chain:KinematicChain, zeroPoseOffsets:Array<Vec3>} {
    var d1 = 0.089159, shoulderOffset = 0.13585, elbowOffset = -0.1197,
      a2 = 0.425, a3 = 0.39225, d4 = 0.10915, d5 = 0.09465, d6 = 0.0823;
    var model = new RobotModel("ur5-fixture");
    var linkNames = ["base_link", "shoulder_link", "upper_arm_link", "forearm_link",
      "wrist_1_link", "wrist_2_link", "wrist_3_link"];
    var links = [for (name in linkNames) model.addLink(new Link(name))];
    var offsets = [
      new Vec3(0.0, 0.0, d1),
      new Vec3(0.0, shoulderOffset, 0.0),
      new Vec3(0.0, elbowOffset, a2),
      new Vec3(0.0, 0.0, a3),
      new Vec3(0.0, d4, 0.0),
      new Vec3(0.0, 0.0, d5)
    ];
    var axes = [
      [0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [0.0, 1.0, 0.0],
      [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]
    ];
    var jointNames = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"];
    for (i in 0...6) {
      var joint = model.addJoint(new Joint(jointNames[i], JointType.Revolute, links[i], links[i + 1]));
      joint.parentFramePosition = offsets[i].toArray();
      joint.axis = axes[i];
      joint.limits.lower = -2.0 * Math.PI;
      joint.limits.upper = 2.0 * Math.PI;
      joint.limits.velocity = 0.0;
    }
    var flangeOffset = new Vec3(0.0, d6, 0.0);
    var flange = model.addFrame(new Frame("flange", links[6]));
    flange.position = flangeOffset.toArray();
    var chain = new KinematicChain(model, links[0].id, ChainTip.Frame(flange.id));
    return { model: model, chain: chain, zeroPoseOffsets: offsets.concat([flangeOffset]) };
  }

  // -- small local helpers ---------------------------------------------

  static function logMap(from:Quat, to:Quat):Vec3 {
    var error = to.multiply(from.conjugate());
    if (error.w < 0.0) error = new Quat(-error.x, -error.y, -error.z, -error.w);
    var sinHalf = Math.sqrt(error.x * error.x + error.y * error.y + error.z * error.z);
    if (sinHalf < 1e-9) return Vec3.zero();
    var angle = 2.0 * Math.atan2(sinHalf, error.w);
    return new Vec3(error.x / sinHalf, error.y / sinHalf, error.z / sinHalf).scale(angle);
  }

  static function Pose2WrapDiff(a:Float, b:Float):Float {
    var tau = Math.PI * 2.0;
    var wrapped = ((a - b) + Math.PI) % tau;
    if (wrapped < 0.0) wrapped += tau;
    return wrapped - Math.PI;
  }

  static function approx(a:Float, b:Float, tolerance:Float):Bool return Math.abs(a - b) <= tolerance;

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'assertion failed: $message';
  }
}

/** Deterministic linear congruential generator; used only to seed IK test targets. */
private class Rng {
  var state:Int;

  public function new(seed:Int) {
    state = seed;
  }

  /** Returns a value in [0, 1). */
  public function next():Float {
    state = (state * 1103515245 + 12345) & 0x7fffffff;
    return state / 2147483647.0;
  }
}
