package tests;

import robotkit.manipulation.IkOptions;
import haxe.Int64;
import robotkit.model.RobotModel;
import robotkit.model.Link;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Frame;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.spatial.Inertia3;
import robotkit.manipulation.Manipulator;
import robotkit.manipulation.PayloadChecker;
import robotkit.manipulation.RobotPayloadLimit;
import robotkit.manipulation.ReachLoadBand;
import robotkit.manipulation.ReachLoadChart;
import robotkit.tool.Tool;
import robotkit.tool.MassProperties;
import robotkit.tool.WorkpieceLoad;
import robotkit.tool.ToolCollisionShape;
import robotkit.tool.SimulatedSprayer;
import robotkit.tool.SuctionGrip;
import robotkit.tool.SuctionMotionSample;
import robotkit.tool.SuctionCapacityChecker;

/** M3 acceptance tests for robotkit.tool: Tool/TCP offsets and simulated capability state. */
class ToolTests {
  static var assertions = 0;

  public static function run():Int {
    assertions = 0;
    testTcpOffsetUnderRotation();
    testIkToTcpTarget();
    testSimulatedSprayerStateHistory();
    testWorkpieceMassRollup();
    testPayloadChartAcrossJointMotion();
    testSuctionCapacityAcrossMotion();
    Sys.println('RobotKit tool tests passed ($assertions assertions)');
    return assertions;
  }

  static function testTcpOffsetUnderRotation():Void {
    // Single revolute joint about +Z; a tool offset of 0.2m along local +X
    // at the flange should land at +Y once the arm has rotated 90 degrees.
    var fixture = buildSingleJointFixture();
    var tool = new Tool("probe", "probe", new Transform3(new Vec3(0.2, 0.0, 0.0), Quat.identity()),
      ToolCollisionShape.NoCollision, 0.1);
    var manipulator = fixture.arm.withTool(tool.flangeTTcp);

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
    var manipulator = fixture.arm.withTool(tool.flangeTTcp);

    var qTrue = [0.3, -0.6, 0.9, -0.4, 0.5, -0.2];
    var target = manipulator.tcpPose(qTrue);
    var seed = [for (i in 0...6) qTrue[i] + 0.05];
    var result = manipulator.solve(target, seed, new IkOptions(1e-4, 1e-3, 100, 0.02));
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

  static function testWorkpieceMassRollup():Void {
    var tool = new MassProperties(2.0, new Vec3(0.1, 0, 0),
      new Inertia3(0.02, 0, 0, 0.03, 0, 0.04));
    var part = new WorkpieceLoad("pick-a", new MassProperties(1.0,
      new Vec3(0.1, 0, 0), new Inertia3(0.01, 0, 0, 0.02, 0, 0.01)),
      new Transform3(new Vec3(0, 0.2, 0),
        Quat.fromAxisAngle(new Vec3(0, 0, 1), Math.PI * 0.5)));
    var combined = tool.combined(part.atFlange());
    check(approx(combined.massKg, 3.0, 1e-12) &&
      approx(combined.centerOfMass.x, 1.0 / 15.0, 1e-12) &&
      approx(combined.centerOfMass.y, 0.1, 1e-12),
      "Pick-specific workpiece pose changes the combined flange centre of mass");
    check(combined.inertia != null && approx(combined.inertia.zz, 7.0 / 60.0, 1e-12),
      "Combined centroidal inertia uses the parallel-axis theorem");
    var rotated = new Inertia3(1, 0, 0, 2, 0, 3)
      .rotated(Quat.fromAxisAngle(new Vec3(1, 0, 0), Math.PI * 0.5));
    check(approx(rotated.xx, 1, 1e-12) && approx(rotated.yy, 3, 1e-12) &&
      approx(rotated.zz, 2, 1e-12), "Inertia rotates with the workpiece frame");
  }

  static function testPayloadChartAcrossJointMotion():Void {
    var model = new RobotModel("payload-slide");
    var base = model.addLink(new Link("base"));
    var carriage = model.addLink(new Link("carriage"));
    var joint = model.addJoint(new Joint("slide", JointType.Prismatic, base, carriage));
    joint.axis = [1.0, 0.0, 0.0];
    joint.limits.lower = 0.0;
    joint.limits.upper = 3.0;
    var flange = model.addFrame(new Frame("flange", carriage));
    var manipulator = new Manipulator(model, base.id, flange.id);
    var tool = new Tool("pickup", "pickup", Transform3.identity(),
      ToolCollisionShape.NoCollision, 1.0,
      new MassProperties(1.0, new Vec3(0.1, 0, 0), Inertia3.zero()));
    var part = new WorkpieceLoad("part-a", new MassProperties(2.0,
      new Vec3(0.1, 0, 0), Inertia3.zero()), Transform3.identity());
    var chart = new ReachLoadChart([
      new ReachLoadBand(0.5, new RobotPayloadLimit(4.0, 4.0)),
      new ReachLoadBand(2.0, new RobotPayloadLimit(4.0, 2.0))
    ]);
    var bare = PayloadChecker.checkPath(manipulator, tool, null, chart, [[0.2], [1.2]], 0.1);
    check(bare.safe, "Tool alone fits the load chart through the move");
    var loaded = PayloadChecker.checkPath(manipulator, tool, part, chart,
      [[0.2], [1.2]], 0.1);
    check(!loaded.safe && loaded.firstFailureSegment == 1 &&
      loaded.segmentFraction > 0 && loaded.segmentFraction < 1 &&
      loaded.reason != null && loaded.reason.indexOf("moment") >= 0,
      "Workpiece load violates the chart between two sampled waypoints");
    check(loaded.checkedSamples > 2, "Payload checker samples the joint path between waypoints");
    var outside = PayloadChecker.checkPath(manipulator, tool, null, chart, [[2.0]]);
    check(outside.safe, "Last chart band includes its reach boundary");
    var beyond = PayloadChecker.checkPath(manipulator, tool, null, chart, [[2.1]]);
    check(!beyond.safe && beyond.reason != null && beyond.reason.indexOf("outside") >= 0,
      "Robot poses beyond the load chart are rejected");
    var missingCentre = false;
    try PayloadChecker.checkPath(manipulator,
      new Tool("mass-only", "mass-only", Transform3.identity(), NoCollision, 1.0),
      null, chart, [[0.2]])
    catch (error:Dynamic) missingCentre = Std.string(error).indexOf("centre of mass") >= 0;
    check(missingCentre, "Payload check rejects legacy tools with mass but no centre of mass");
  }

  static function testSuctionCapacityAcrossMotion():Void {
    var part = new WorkpieceLoad("panel", new MassProperties(2.0,
      Vec3.zero(), Inertia3.zero()), Transform3.identity());
    var down = new Transform3(Vec3.zero(),
      Quat.fromAxisAngle(new Vec3(1, 0, 0), Math.PI));
    var area = Math.PI * 0.02 * 0.02;
    var grip = new SuctionGrip(down, area, 60, 0.5, 2.0);
    var staticSample = new SuctionMotionSample(Transform3.identity(), Vec3.zero());
    var staticResult = SuctionCapacityChecker.checkPath(part, grip, [staticSample]);
    check(staticResult.safe && approx(staticResult.normalCapacityN, 60e3 * area, 1e-9),
      "Cup capacity uses minimum vacuum at the sealed area");
    var moving = SuctionCapacityChecker.checkPath(part, grip, [staticSample,
      new SuctionMotionSample(Transform3.identity(), new Vec3(4, 0, 0))]);
    check(moving.safe && moving.worstShearMarginN > 0,
      "Moderate sideways acceleration stays within the friction margin");
    var slipping = SuctionCapacityChecker.checkPath(part, grip, [staticSample,
      new SuctionMotionSample(Transform3.identity(), new Vec3(5, 0, 0))]);
    check(!slipping.safe && slipping.firstFailureIndex == 1 &&
      slipping.reason != null && slipping.reason.indexOf("Tangential") >= 0,
      "Path reports the first sideways slip risk separately from suction tension");
    var lifting = SuctionCapacityChecker.checkPath(part, grip, [
      new SuctionMotionSample(Transform3.identity(), new Vec3(0, 0, 10))]);
    check(!lifting.safe && lifting.reason != null && lifting.reason.indexOf("Normal") >= 0,
      "Upward acceleration can exceed normal suction capacity");
    var weakVacuum = SuctionCapacityChecker.checkPath(part,
      new SuctionGrip(down, area, 10, 0.5, 2.0), [staticSample]);
    check(!weakVacuum.safe && weakVacuum.worstNormalMarginN < 0,
      "Low cup vacuum fails even with no commanded acceleration");
    var sidewaysCup = new Transform3(Vec3.zero(),
      Quat.fromAxisAngle(new Vec3(0, 1, 0), Math.PI * 0.5));
    var vertical = SuctionCapacityChecker.checkPath(part,
      new SuctionGrip(sidewaysCup, area, 60, 0.5, 2.0), [staticSample]);
    check(!vertical.safe && vertical.reason != null && vertical.reason.indexOf("Tangential") >= 0,
      "A vertical cup must hold the workpiece through friction");
    var offsetPart = new WorkpieceLoad("offset", new MassProperties(2.0,
      new Vec3(0.1, 0, 0), Inertia3.zero()), Transform3.identity());
    var unknownMoment = SuctionCapacityChecker.checkPath(offsetPart, grip, [staticSample]);
    check(!unknownMoment.safe && unknownMoment.reason != null &&
      unknownMoment.reason.indexOf("unverified") >= 0,
      "An offset workpiece cannot pass without a cup moment rating");
    var ratedMoment = SuctionCapacityChecker.checkPath(offsetPart,
      new SuctionGrip(down, area, 60, 0.5, 2.0, 5.0), [staticSample]);
    check(ratedMoment.safe && ratedMoment.worstMomentMarginNm != null &&
      ratedMoment.worstMomentMarginNm > 0,
      "A rated cup can carry the offset moment within its safety margin");
  }

  // -- fixtures --------------------------------------------------------

  static function buildSingleJointFixture():{model:RobotModel, arm:Manipulator} {
    var model = new RobotModel("single-joint-arm");
    var base = model.addLink(new Link("base"));
    var link1 = model.addLink(new Link("link1"));
    var joint1 = model.addJoint(new Joint("joint1", JointType.Revolute, base, link1));
    joint1.axis = [0.0, 0.0, 1.0];
    var flange = model.addFrame(new Frame("flange", link1));
    flange.position = [1.0, 0.0, 0.0];
    var arm = new Manipulator(model, base.id, flange.id);
    return { model: model, arm: arm };
  }

  /** A 6R arm laid out with the axis pattern and published DH-equivalent offsets of a UR5. */
  static function buildUR5Fixture():{model:RobotModel, arm:Manipulator} {
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
      joint.limits.velocity = null;
    }
    var flangeOffset = new Vec3(0.0, d6, 0.0);
    var flange = model.addFrame(new Frame("flange", links[6]));
    flange.position = flangeOffset.toArray();
    var arm = new Manipulator(model, links[0].id, flange.id);
    return { model: model, arm: arm };
  }

  static function approx(a:Float, b:Float, tolerance:Float):Bool return Math.abs(a - b) <= tolerance;

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'assertion failed: $message';
  }
}
