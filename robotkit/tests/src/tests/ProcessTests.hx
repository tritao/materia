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
import robotkit.manipulation.Manipulator;
import robotkit.process.ToolpathPoint;
import robotkit.process.Toolpath;
import robotkit.process.CartesianTrajectory;
import robotkit.process.ToolpathExecutor;
import robotkit.process.ToolpathExecutionFailure;

/** M4 acceptance tests for robotkit.process: Toolpath, CartesianTrajectory, ToolpathExecutor. */
class ProcessTests {
  static var assertions = 0;

  public static function run():Int {
    assertions = 0;
    testToolpathLengthAndSegmenting();
    testTrapezoidalTimingSingleSegment();
    testTrapezoidalTimingTriangleSegment();
    testTrapezoidalTimingMultiSegment();
    testAdaptiveCartesianSampling();
    testExecutorOnSmallRasterYieldsContinuousJoints();
    testExecutorReportsUnreachableIndex();
    Sys.println('RobotKit process tests passed ($assertions assertions)');
    return assertions;
  }

  static function testToolpathLengthAndSegmenting():Void {
    var identity = Quat.identity();
    var p0 = new ToolpathPoint(new Transform3(new Vec3(0.0, 0.0, 0.0), identity), 0.1, false);
    var p1 = new ToolpathPoint(new Transform3(new Vec3(1.0, 0.0, 0.0), identity), 0.1, true);
    var p2 = new ToolpathPoint(new Transform3(new Vec3(1.0, 2.0, 0.0), identity), 0.1, true);
    var p3 = new ToolpathPoint(new Transform3(new Vec3(1.0, 2.0, 0.0), identity), 0.1, false);
    var toolpath = new Toolpath("work", [p0, p1, p2, p3]);

    check(approx(toolpath.length(), 3.0, 1e-9), "Toolpath length sums consecutive point distances");
    check(approx(toolpath.processOnLength(), 2.0, 1e-9),
      "Toolpath process-on length only counts moves departing a processOn point");

    var segments = toolpath.segmentByProcess();
    check(segments.approach.points.length == 1, "Approach segment holds the leading processOn==false run");
    check(segments.process.points.length == 2, "Process segment spans first..last processOn==true point inclusive");
    check(segments.retract.points.length == 1, "Retract segment holds the trailing processOn==false run");
  }

  static function testTrapezoidalTimingSingleSegment():Void {
    var identity = Quat.identity();
    var p0 = new ToolpathPoint(new Transform3(Vec3.zero(), identity), 0.5, true);
    var p1 = new ToolpathPoint(new Transform3(new Vec3(1.0, 0.0, 0.0), identity), 0.5, true);
    var toolpath = new Toolpath("work", [p0, p1]);
    var trajectory = CartesianTrajectory.build(toolpath, 1.0, 0.1);
    var expected = trapezoidDuration(1.0, 0.5, 1.0);
    check(approx(trajectory.duration(), expected, 1e-9), "Single-segment trapezoid duration matches the closed form");
    check(trajectory.samples[0].time == 0.0, "First sample starts at time zero");
    check(approx(trajectory.samples[trajectory.samples.length - 1].time, expected, 1e-9),
      "Final sample lands exactly at the segment's closed-form duration");
    var last = trajectory.samples[trajectory.samples.length - 1];
    check(approx(last.work_T_tcp.translation.x, 1.0, 1e-9), "Final sample reaches the toolpath's last point");
  }

  static function testTrapezoidalTimingTriangleSegment():Void {
    var identity = Quat.identity();
    var p0 = new ToolpathPoint(new Transform3(Vec3.zero(), identity), 1.0, true);
    var p1 = new ToolpathPoint(new Transform3(new Vec3(0.05, 0.0, 0.0), identity), 1.0, true);
    var toolpath = new Toolpath("work", [p0, p1]);
    var trajectory = CartesianTrajectory.build(toolpath, 1.0, 0.1);
    var expected = trapezoidDuration(0.05, 1.0, 1.0);
    check(approx(trajectory.duration(), expected, 1e-9), "Triangular (too-short-to-cruise) profile duration matches the closed form");
  }

  static function testTrapezoidalTimingMultiSegment():Void {
    var identity = Quat.identity();
    var p0 = new ToolpathPoint(new Transform3(Vec3.zero(), identity), 0.5, true);
    var p1 = new ToolpathPoint(new Transform3(new Vec3(1.0, 0.0, 0.0), identity), 0.5, true);
    var p2 = new ToolpathPoint(new Transform3(new Vec3(1.0, 0.6, 0.0), identity), 0.3, true);
    var toolpath = new Toolpath("work", [p0, p1, p2]);
    var trajectory = CartesianTrajectory.build(toolpath, 1.0, 0.1);
    var expectedFirst = trapezoidDuration(1.0, 0.5, 1.0);
    var expectedSecond = trapezoidDuration(0.6, 0.3, 1.0);
    check(approx(trajectory.duration(), expectedFirst + expectedSecond, 1e-9),
      "Multi-segment trajectory duration is the sum of each segment's closed-form duration");
    var segmentIndices = [for (sample in trajectory.samples) sample.segmentIndex];
    check(segmentIndices[0] == 0 && segmentIndices[segmentIndices.length - 1] == 1,
      "Samples carry the departing toolpath point's segment index");
  }

  static function testAdaptiveCartesianSampling():Void {
    var identity = Quat.identity();
    var longPath = new Toolpath("work", [
      new ToolpathPoint(new Transform3(Vec3.zero(), identity), 1.0, true),
      new ToolpathPoint(new Transform3(new Vec3(20.0, 0.0, 0.0), identity), 1.0, true)
    ]);
    var linear = CartesianTrajectory.build(longPath, 1.0, 0.1, 0.05, Math.PI / 36.0);
    check(linear.samples.length == 401,
      '20m segment uses one sample per 5cm step (got ${linear.samples.length})');
    var largestTimeGap = 0.0;
    for (i in 1...linear.samples.length)
      largestTimeGap = Math.max(largestTimeGap, linear.samples[i].time - linear.samples[i - 1].time);
    check(largestTimeGap <= 0.1 + 1e-12,
      'spatially dense samples still honor the time interval upper bound (got $largestTimeGap)');

    var reorientation = new Toolpath("work", [
      new ToolpathPoint(new Transform3(Vec3.zero(), identity), 1.0, true),
      new ToolpathPoint(new Transform3(Vec3.zero(), Quat.fromAxisAngle(new Vec3(0.0, 0.0, 1.0), Math.PI * 0.5)), 1.0, true)
    ]);
    var angular = CartesianTrajectory.build(reorientation, 1.0, 10.0, 1.0, Math.PI / 36.0);
    check(angular.samples.length >= 19,
      '90-degree reorientation uses at least one sample per 5 degrees (got ${angular.samples.length})');
    check(angular.samples[angular.samples.length - 1].work_T_tcp.rotation.angularDistance(
      reorientation.points[1].work_T_tcp.rotation) < 1e-9,
      "adaptive angular samples reach the requested final orientation");
  }

  static function testExecutorOnSmallRasterYieldsContinuousJoints():Void {
    var fixture = buildUR5Fixture();
    var manipulator = new Manipulator(fixture.model, fixture.chain);
    var referenceQ = [0.3, -0.9, 1.2, -0.3, 0.5, 0.0];
    var referencePose = manipulator.tcpPose(referenceQ);

    var offsets = [[0.0, 0.0], [0.02, 0.0], [0.02, 0.02], [0.0, 0.02], [-0.02, 0.02], [-0.02, 0.0]];
    var points = [for (offset in offsets) new ToolpathPoint(
      new Transform3(referencePose.translation.add(new Vec3(offset[0], offset[1], 0.0)), referencePose.rotation),
      0.05, true)];
    var toolpath = new Toolpath("base", points);
    var trajectory = CartesianTrajectory.build(toolpath, 0.5, 0.05);
    var result = ToolpathExecutor.execute(manipulator, trajectory, Transform3.identity(), referenceQ, 0.5);

    check(result.success, "Executor succeeds over a small in-reach raster");
    check(result.steps.length == trajectory.samples.length, "Executor emits one step per trajectory sample");
    var maxJointDelta = 0.0;
    for (i in 1...result.steps.length) {
      for (joint in 0...6) {
        var delta = Math.abs(result.steps[i].q[joint] - result.steps[i - 1].q[joint]);
        if (delta > maxJointDelta) maxJointDelta = delta;
      }
    }
    check(maxJointDelta < 0.3, "Consecutive joint solutions across the raster stay continuous (no large jumps)");
    check(result.steps[0].targets.length == 6, "Each step emits one joint target per degree of freedom");
  }

  static function testExecutorReportsUnreachableIndex():Void {
    var fixture = buildUR5Fixture();
    var manipulator = new Manipulator(fixture.model, fixture.chain);
    var referenceQ = [0.3, -0.9, 1.2, -0.3, 0.5, 0.0];
    var referencePose = manipulator.tcpPose(referenceQ);

    var reachable = new ToolpathPoint(referencePose, 5.0, true);
    var unreachable = new ToolpathPoint(new Transform3(new Vec3(100.0, 100.0, 100.0), referencePose.rotation), 5.0, true);
    var toolpath = new Toolpath("base", [reachable, unreachable]);
    // A long sample interval and large spatial limits collapse each segment to
    // a single final sample, so this two-point path yields exactly two samples.
    var trajectory = CartesianTrajectory.build(toolpath, 10.0, 1000.0, 1000.0, Math.PI);
    check(trajectory.samples.length == 2, "Coarse sampling collapses the far segment to exactly two samples");

    var result = ToolpathExecutor.execute(manipulator, trajectory, Transform3.identity(), referenceQ, 0.5);
    check(!result.success, "Executor reports failure for an unreachable point instead of throwing");
    check(result.steps.length == 1, "Executor keeps the steps produced before the unreachable point");
    switch result.failure {
      case ToolpathExecutionFailure.Unreachable(sampleIndex, ik):
        check(sampleIndex == 1, "Unreachable failure reports the index of the far point");
        check(!ik.converged, "Unreachable failure carries the non-converged IK result");
      case other:
        check(false, 'Expected an Unreachable failure, got $other');
    }
  }

  // -- fixtures and helpers ---------------------------------------------

  static function trapezoidDuration(distance:Float, targetVelocity:Float, maxAcceleration:Float):Float {
    var accelTimeToTarget = targetVelocity / maxAcceleration;
    var accelDistanceToTarget = 0.5 * maxAcceleration * accelTimeToTarget * accelTimeToTarget;
    if (2.0 * accelDistanceToTarget <= distance) {
      var cruiseTime = (distance - 2.0 * accelDistanceToTarget) / targetVelocity;
      return 2.0 * accelTimeToTarget + cruiseTime;
    }
    var peakVelocity = Math.sqrt(maxAcceleration * distance);
    return 2.0 * (peakVelocity / maxAcceleration);
  }

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
