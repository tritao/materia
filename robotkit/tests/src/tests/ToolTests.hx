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
import robotkit.manipulation.Manipulator;
import robotkit.tool.Tool;
import robotkit.tool.ToolCollisionShape;
import robotkit.tool.SimulatedSprayer;

/** M3 acceptance tests for robotkit.tool: Tool/TCP offsets and simulated capability state. */
class ToolTests {
  static var assertions = 0;

  public static function run():Int {
    assertions = 0;
    testTcpOffsetUnderRotation();
    testIkToTcpTarget();
    testSimulatedSprayerStateHistory();
    Sys.println('RobotKit tool tests passed ($assertions assertions)');
    return assertions;
  }

  static function testTcpOffsetUnderRotation():Void {
    // Single revolute joint about +Z; a tool offset of 0.2m along local +X
    // at the flange should land at +Y once the arm has rotated 90 degrees.
    var fixture = buildSingleJointFixture();
    var tool = new Tool("probe", "probe", new Transform3(new Vec3(0.2, 0.0, 0.0), Quat.identity()),
      ToolCollisionShape.NoCollision, 0.1);
    var manipulator = new Manipulator(fixture.model, fixture.chain, tool.flangeTTcp);

    var zero = manipulator.tcpPose([0.0]);
    check(approx(zero.translation.x, 1.2, 1e-9) && approx(zero.translation.y, 0.0, 1e-9),
      "TCP pose at zero joint angle offsets straight out along the flange's +X");

    var rotated = manipulator.tcpPose([Math.PI * 0.5]);
    check(approx(rotated.translation.x, 0.0, 1e-9) && approx(rotated.translation.y, 1.2, 1e-9),
      "TCP pose rotates the tool offset with the flange, landing at +Y after a 90deg turn");
  }

  static function testIkToTcpTarget():Void {
    var fixture = buildUR5Fixture();
    var tool = new Tool("sprayer", "sprayer", new Transform3(new Vec3(0.0, 0.0, 0.08), Quat.identity()));
    var manipulator = new Manipulator(fixture.model, fixture.chain, tool.flangeTTcp);

    var qTrue = [0.3, -0.6, 0.9, -0.4, 0.5, -0.2];
    var target = manipulator.tcpPose(qTrue);
    var seed = [for (i in 0...6) qTrue[i] + 0.05];
    var result = manipulator.solveIkForTcp(target, seed, 1e-4, 1e-3, 100, 0.02);
    check(result.converged, "IK to a TCP target converges");
    var achieved = manipulator.tcpPose(result.q);
    check(approx(achieved.translation.x, target.translation.x, 1e-3) &&
      approx(achieved.translation.y, target.translation.y, 1e-3) &&
      approx(achieved.translation.z, target.translation.z, 1e-3),
      "IK solution's TCP pose matches the requested TCP target position");
    check(achieved.rotation.angularDistance(target.rotation) < 1e-3,
      "IK solution's TCP pose matches the requested TCP target orientation");
  }

  static function testSimulatedSprayerStateHistory():Void {
    var sprayer = new SimulatedSprayer();
    sprayer.setFlow(0.5, Int64.ofInt(1000));
    sprayer.setPressure(3.0, Int64.ofInt(1500));
    sprayer.setFlow(0.8, Int64.ofInt(2000));

    check(sprayer.history.length == 3, "Simulated sprayer records one event per command");
    check(approx(sprayer.history[0].flow, 0.5, 1e-9) && Int64.toInt(sprayer.history[0].timestampNs) == 1000,
      "First recorded event captures the commanded flow and its timestamp");
    check(approx(sprayer.history[1].flow, 0.5, 1e-9) && approx(sprayer.history[1].pressure, 3.0, 1e-9) &&
      Int64.toInt(sprayer.history[1].timestampNs) == 1500,
      "Pressure command records the flow unchanged alongside the new pressure");
    check(approx(sprayer.history[2].flow, 0.8, 1e-9) && Int64.toInt(sprayer.history[2].timestampNs) == 2000,
      "Later flow command overwrites the recorded flow and advances the timestamp");
    check(approx(sprayer.flow(), 0.8, 1e-9) && approx(sprayer.pressure(), 3.0, 1e-9),
      "Simulated sprayer exposes its latest commanded state");
  }

  // -- fixtures --------------------------------------------------------

  static function buildSingleJointFixture():{model:RobotModel, chain:KinematicChain} {
    var model = new RobotModel("single-joint-arm");
    var base = model.addLink(new Link("base"));
    var link1 = model.addLink(new Link("link1"));
    var joint1 = model.addJoint(new Joint("joint1", JointType.Revolute, base, link1));
    joint1.axis = [0.0, 0.0, 1.0];
    var flange = model.addFrame(new Frame("flange", link1));
    flange.position = [1.0, 0.0, 0.0];
    var chain = new KinematicChain(model, base.id, ChainTip.Frame(flange.id));
    return { model: model, chain: chain };
  }

  /** A 6R arm laid out with the axis pattern and published DH-equivalent offsets of a UR5. */
  static function buildUR5Fixture():{model:RobotModel, chain:KinematicChain} {
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
    return { model: model, chain: chain };
  }

  static function approx(a:Float, b:Float, tolerance:Float):Bool return Math.abs(a - b) <= tolerance;

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'assertion failed: $message';
  }
}
