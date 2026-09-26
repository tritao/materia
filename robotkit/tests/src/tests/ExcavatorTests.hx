package tests;

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
import robotkit.manipulation.InverseKinematics;
import robotkit.manipulation.JointGroup;
import robotkit.manipulation.Manipulator;
import robotkit.tool.Tool;
import robotkit.tool.ToolCollisionShape;
import robotkit.work.DigCyclePlanner;
import robotkit.work.Point2;

/**
 * M12 acceptance tests for the simulated excavator: the 4-DOF
 * slew/boom/stick/bucket kinematic chain (reused unchanged from M2) and
 * `DigCyclePlanner`. `DigTrench`/`GradeRegion`/`DumpAt` skill and scenario
 * tests are added in a follow-up commit alongside those skills.
 */
class ExcavatorTests {
  static var assertions = 0;

  public static function run():Int {
    assertions = 0;
    testZeroPoseFK();
    testFourDofIkRecoversManifoldTargets();
    testDigCyclePlannerStages();
    Sys.println('RobotKit excavator tests passed ($assertions assertions)');
    return assertions;
  }

  // -- M12 kinematics -----------------------------------------------------

  static function testZeroPoseFK():Void {
    var fixture = buildExcavatorFixture();
    var zero = [0.0, 0.0, 0.0, 0.0];
    var tcp = fixture.manipulator.tcpPose(zero);
    check(approx(tcp.translation.x, 6.0, 1e-9) && approx(tcp.translation.y, 0.0, 1e-9) &&
      approx(tcp.translation.z, 0.9, 1e-9),
      "Excavator zero-pose TCP position matches the summed fixture offsets");
    check(tcp.rotation.angularDistance(Quat.identity()) < 1e-9,
      "Excavator zero-pose TCP has no net rotation");
    check(fixture.chain.dofCount() == 4, "Excavator fixture exposes four degrees of freedom");
  }

  static function testFourDofIkRecoversManifoldTargets():Void {
    var fixture = buildExcavatorFixture();
    var group = JointGroup.fromChain(fixture.chain);
    var seed = [0.0, 0.3, -1.0, -1.0];
    var samples = [
      { x: 3.2, y: 0.0, z: -0.4, pitch: -0.4 },
      { x: 3.6, y: 0.4, z: 0.1, pitch: -1.6 },
      { x: 2.6, y: -0.3, z: 0.6, pitch: 0.5 }
    ];
    var current = seed;
    for (sample in samples) {
      var target = DigCyclePlanner.poseAt(sample.x, sample.y, sample.z, sample.pitch);
      var result = InverseKinematics.solve(fixture.chain, group, target, current, 1e-4, 1e-3, 300, 0.02);
      check(result.converged, 'Excavator IK converges for manifold target ($sample.x, $sample.y, $sample.z, pitch=$sample.pitch)');
      var achieved = fixture.chain.forwardKinematics(result.q);
      check(approx(achieved.translation.x, target.translation.x, 1e-3) &&
        approx(achieved.translation.y, target.translation.y, 1e-3) &&
        approx(achieved.translation.z, target.translation.z, 1e-3),
        "Excavator IK solution FK matches the requested position");
      check(achieved.rotation.angularDistance(target.rotation) < 1e-2,
        "Excavator IK solution FK matches the requested orientation");
      current = result.q;
    }
  }

  static function testDigCyclePlannerStages():Void {
    var entry = new Point2(3.0, 0.0);
    var exit = new Point2(4.0, 0.0);
    var dump = new Point2(1.5, 3.0);
    var plan = DigCyclePlanner.planCycle("excavator", entry, exit, 0.0, 0.3,
      0.6, dump, 0.4, 0.4, -0.4, -1.8, 0.6, 0.4, 8);
    var points = plan.toolpath.points;
    check(points.length == 14,
      "Dig cycle toolpath has entry + cut + curl + lift + 8 swing steps + descend + open");
    check(!points[0].processOn, "Entry point is not engaged with the ground");
    check(points[1].processOn, "Cut point is engaged with the ground");
    check(points[2].processOn, "Curl point is engaged with the ground");
    check(!points[3].processOn, "Lift point is not engaged with the ground");
    check(approx(points[1].work_T_tcp.translation.x, exit.x, 1e-9) &&
      approx(points[1].work_T_tcp.translation.z, -0.3, 1e-9),
      "Cut point reaches the exit position at the target depth");
    var descend = points[points.length - 2];
    var open = points[points.length - 1];
    check(approx(descend.work_T_tcp.translation.x, dump.x, 1e-9) &&
      approx(descend.work_T_tcp.translation.y, dump.y, 1e-9) &&
      approx(descend.work_T_tcp.translation.z, 0.4, 1e-9),
      "Descend point reaches the dump position and elevation, still curled");
    check(approx(open.work_T_tcp.translation.x, dump.x, 1e-9) &&
      approx(open.work_T_tcp.translation.y, dump.y, 1e-9) &&
      approx(open.work_T_tcp.translation.z, 0.4, 1e-9),
      "Open point stays at the dump position and elevation");
    check(descend.work_T_tcp.rotation.angularDistance(open.work_T_tcp.rotation) > 0.1,
      "Open point's pitch differs from the descend point's curled pitch");
    check(plan.sweepFrom == entry && plan.sweepTo == exit, "Dig cycle plan carries the sweep endpoints");
    check(approx(plan.sweepHalfWidth, 0.4, 1e-9), "Dig cycle plan carries the sweep half-width");
    check(approx(plan.sweepEdgeHeight, -0.3, 1e-9), "Dig cycle plan carries the cut depth as the sweep edge height");
  }

  // -- fixture --------------------------------------------------------

  static function buildExcavatorFixture():{model:RobotModel, chain:KinematicChain, manipulator:Manipulator, tool:Tool} {
    var model = new RobotModel("excavator-fixture");
    var undercarriage = model.addLink(new Link("undercarriage"));
    var upperStructure = model.addLink(new Link("upper_structure"));
    var boomLink = model.addLink(new Link("boom_link"));
    var stickLink = model.addLink(new Link("stick_link"));
    var bucketLink = model.addLink(new Link("bucket_link"));

    var slew = model.addJoint(new Joint("slew_joint", JointType.Revolute, undercarriage, upperStructure));
    slew.axis = [0.0, 0.0, 1.0];
    slew.limits.lower = -Math.PI * 2.0;
    slew.limits.upper = Math.PI * 2.0;

    var boom = model.addJoint(new Joint("boom_joint", JointType.Revolute, upperStructure, boomLink));
    boom.parentFramePosition = [0.3, 0.0, 1.0];
    boom.axis = [0.0, 1.0, 0.0];
    boom.limits.lower = -2.2;
    boom.limits.upper = 2.2;

    var stick = model.addJoint(new Joint("stick_joint", JointType.Revolute, boomLink, stickLink));
    stick.parentFramePosition = [3.0, 0.0, 0.0];
    stick.axis = [0.0, 1.0, 0.0];
    stick.limits.lower = -3.0;
    stick.limits.upper = 3.0;

    var bucket = model.addJoint(new Joint("bucket_joint", JointType.Revolute, stickLink, bucketLink));
    bucket.parentFramePosition = [1.8, 0.0, 0.0];
    bucket.axis = [0.0, 1.0, 0.0];
    bucket.limits.lower = -3.0;
    bucket.limits.upper = 3.0;

    var flange = model.addFrame(new Frame("bucket_flange", bucketLink));

    var chain = new KinematicChain(model, undercarriage.id, ChainTip.Frame(flange.id));
    var tool = new Tool("bucket", "bucket", new Transform3(new Vec3(0.9, 0.0, -0.1), Quat.identity()),
      ToolCollisionShape.Box(new Vec3(0.5, 0.6, 0.4)), 400.0);
    var manipulator = new Manipulator(model, chain, tool.flangeTTcp);
    return { model: model, chain: chain, manipulator: manipulator, tool: tool };
  }

  // -- small local helpers ---------------------------------------------

  static function approx(a:Float, b:Float, tolerance:Float):Bool return Math.abs(a - b) <= tolerance;

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'assertion failed: $message';
  }
}
