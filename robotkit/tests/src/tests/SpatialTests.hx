package tests;

import robotkit.mobile.Pose2;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.spatial.Twist3;
import robotkit.spatial.FrameTree3;
import robotkit.spatial.FrameTransform3;

/** M1 acceptance tests for robotkit.spatial: Vec3, Quat, Transform3, FrameTree3. */
class SpatialTests {
  static var assertions = 0;

  public static function run():Int {
    assertions = 0;
    testVec3Basics();
    testQuatRoundTrips();
    testQuatSlerp();
    testTransformInverseIdentity();
    testCompositionAssociativity();
    testPose2Bridge();
    testAdjointConsistency();
    testFrameTree3TwoBranches();
    Sys.println('RobotKit spatial tests passed ($assertions assertions)');
    return assertions;
  }

  static function testVec3Basics():Void {
    var a = new Vec3(1.0, 2.0, 3.0);
    var b = new Vec3(4.0, -1.0, 0.5);
    var cross = a.cross(b);
    check(approx(cross.x, 2.0 * 0.5 - 3.0 * -1.0, 1e-9) &&
      approx(cross.y, 3.0 * 4.0 - 1.0 * 0.5, 1e-9) &&
      approx(cross.z, 1.0 * -1.0 - 2.0 * 4.0, 1e-9),
      "Vec3 cross product matches the determinant formula");
    check(approx(a.dot(a), a.norm() * a.norm(), 1e-9), "Vec3 dot with itself equals squared norm");
    var unit = a.normalized();
    check(approx(unit.norm(), 1.0, 1e-9), "Vec3 normalized has unit length");
    throws(function() Vec3.zero().normalized(), "Vec3 normalization rejects a zero vector");
  }

  static function testQuatRoundTrips():Void {
    var axis = new Vec3(0.0, 0.0, 1.0);
    var q = Quat.fromAxisAngle(axis, Math.PI * 0.5);
    var rotated = q.rotate(new Vec3(1.0, 0.0, 0.0));
    check(approxVec(rotated, new Vec3(0.0, 1.0, 0.0), 1e-9),
      "90 degree yaw quaternion rotates +X into +Y");

    var samples = [
      [0.2, -0.4, 0.1], [0.0, 0.0, 0.0], [1.1, 0.3, -0.7],
      [-0.5, 1.2, 0.6], [0.05, -0.02, 1.4]
    ];
    for (sample in samples) {
      var original = Quat.fromRollPitchYaw(sample[0], sample[1], sample[2]);
      var matrix = original.toRotationMatrix();
      var fromMatrix = Quat.fromRotationMatrix(matrix);
      check(original.angularDistance(fromMatrix) < 1e-6,
        "Quat -> rotation matrix -> Quat preserves the rotation");

      var rpy = original.toRollPitchYaw();
      var rebuilt = Quat.fromRollPitchYaw(rpy.roll, rpy.pitch, rpy.yaw);
      check(original.angularDistance(rebuilt) < 1e-6,
        "Quat -> roll/pitch/yaw -> Quat preserves the rotation");
    }

    var identity = Quat.identity();
    check(approx(identity.angularDistance(identity), 0.0, 1e-12),
      "Identity quaternion has zero angular distance from itself");
  }

  static function testQuatSlerp():Void {
    var start = Quat.identity();
    var end = Quat.fromAxisAngle(new Vec3(0.0, 0.0, 1.0), Math.PI * 0.5);
    var mid = start.slerp(end, 0.5);
    var expected = Quat.fromAxisAngle(new Vec3(0.0, 0.0, 1.0), Math.PI * 0.25);
    check(mid.angularDistance(expected) < 1e-6, "Slerp halfway matches a half-angle quaternion");
    check(start.slerp(end, 0.0).angularDistance(start) < 1e-9, "Slerp at t=0 returns the start");
    check(start.slerp(end, 1.0).angularDistance(end) < 1e-9, "Slerp at t=1 returns the end");
  }

  static function testTransformInverseIdentity():Void {
    var samples = sampleTransforms();
    for (t in samples) {
      var roundTrip = t.compose(t.inverse());
      check(approxVec(roundTrip.translation, Vec3.zero(), 1e-9) &&
        roundTrip.rotation.angularDistance(Quat.identity()) < 1e-9,
        "Transform3 composed with its inverse is the identity");
      var otherWay = t.inverse().compose(t);
      check(approxVec(otherWay.translation, Vec3.zero(), 1e-9) &&
        otherWay.rotation.angularDistance(Quat.identity()) < 1e-9,
        "Transform3 inverse composed with the transform is the identity");
    }
  }

  static function testCompositionAssociativity():Void {
    var samples = sampleTransforms();
    var a = samples[0], b = samples[1], c = samples[2];
    var left = a.compose(b).compose(c);
    var right = a.compose(b.compose(c));
    check(approxVec(left.translation, right.translation, 1e-9) &&
      left.rotation.angularDistance(right.rotation) < 1e-9,
      "Transform3 composition is associative");
  }

  static function testPose2Bridge():Void {
    var poses = [new Pose2(1.5, -2.25, 0.4), new Pose2(0.0, 0.0, 0.0),
      new Pose2(-3.1, 4.0, Math.PI * 0.9)];
    for (pose in poses) {
      var t = Transform3.fromPose2(pose, 0.6);
      check(approx(t.translation.z, 0.6, 1e-12), "fromPose2 places the transform at the requested height");
      var back = t.toPose2();
      check(approx(back.x, pose.x, 1e-9) && approx(back.y, pose.y, 1e-9) &&
        approx(Pose2.wrapAngle(back.yaw - pose.yaw), 0.0, 1e-9),
        "Transform3 <-> Pose2 round-trips position and yaw");
    }
  }

  // this = a_T_b (fixed rigid offset). A body twist V_b integrated in b, then
  // reinterpreted in a via a_T_b, should match V_a = a_T_b.transformTwist(V_b)
  // integrated directly in a, to first order in dt. See ARCHITECTURE.md.
  static function testAdjointConsistency():Void {
    var offset = new Transform3(new Vec3(0.3, -0.1, 0.2),
      Quat.fromRollPitchYaw(0.1, 0.2, -0.3));
    var twistB = new Twist3(new Vec3(0.5, -0.2, 0.1), new Vec3(0.1, 0.4, -0.3));
    var twistA = offset.transformTwist(twistB);

    var worldTB0 = new Transform3(new Vec3(2.0, 1.0, 0.5),
      Quat.fromRollPitchYaw(-0.2, 0.05, 0.4));
    var worldTA0 = worldTB0.compose(offset.inverse());

    var dt = 1e-6;
    var worldTBDt = worldTB0.integrate(twistB, dt);
    var worldTADirect = worldTBDt.compose(offset.inverse());
    var worldTAViaTwistA = worldTA0.integrate(twistA, dt);

    var tolerance = 1e-9;
    check(approxVec(worldTADirect.translation, worldTAViaTwistA.translation, tolerance),
      "Adjoint-transformed twist integrates to the same translation as transforming then integrating");
    check(worldTADirect.rotation.angularDistance(worldTAViaTwistA.rotation) < tolerance,
      "Adjoint-transformed twist integrates to the same rotation as transforming then integrating");
  }

  static function testFrameTree3TwoBranches():Void {
    var tree = new FrameTree3();
    tree.add(new FrameTransform3("project", "building",
      new Transform3(new Vec3(1.0, 0.0, 0.0), Quat.identity())));
    tree.add(new FrameTransform3("building", "storey",
      new Transform3(new Vec3(0.0, 1.0, 0.0), Quat.identity())));
    tree.add(new FrameTransform3("storey", "room",
      new Transform3(new Vec3(0.0, 0.0, 1.0), Quat.identity())));
    tree.add(new FrameTransform3("room", "wall",
      new Transform3(new Vec3(2.0, 0.0, 0.0), Quat.fromAxisAngle(new Vec3(0, 0, 1), Math.PI * 0.5))));

    var projectTMap = new Transform3(new Vec3(5.0, 5.0, 0.0),
      Quat.fromAxisAngle(new Vec3(0.0, 0.0, 1.0), Math.PI * 0.5));
    tree.add(new FrameTransform3("project", "map", projectTMap));
    tree.add(new FrameTransform3("map", "base",
      new Transform3(new Vec3(0.0, 0.0, 0.5), Quat.identity())));
    tree.add(new FrameTransform3("base", "arm_base",
      new Transform3(new Vec3(0.0, 0.0, 0.3), Quat.identity())));
    tree.add(new FrameTransform3("arm_base", "flange",
      new Transform3(new Vec3(0.5, 0.0, 0.0), Quat.fromAxisAngle(new Vec3(0, 1, 0), 0.3))));
    tree.add(new FrameTransform3("flange", "tcp",
      new Transform3(new Vec3(0.0, 0.0, 0.1), Quat.identity())));

    check(tree.count() == 9, "Frame tree retains every registered edge");

    // Independently hand-compose the same two chains as a reference, exercising
    // the tree's cross-branch traversal against known-good Transform3 math.
    var projectTWall = new Transform3(new Vec3(1, 0, 0), Quat.identity())
      .compose(new Transform3(new Vec3(0, 1, 0), Quat.identity()))
      .compose(new Transform3(new Vec3(0, 0, 1), Quat.identity()))
      .compose(new Transform3(new Vec3(2, 0, 0), Quat.fromAxisAngle(new Vec3(0, 0, 1), Math.PI * 0.5)));
    var projectTTcp = projectTMap
      .compose(new Transform3(new Vec3(0, 0, 0.5), Quat.identity()))
      .compose(new Transform3(new Vec3(0, 0, 0.3), Quat.identity()))
      .compose(new Transform3(new Vec3(0.5, 0, 0), Quat.fromAxisAngle(new Vec3(0, 1, 0), 0.3)))
      .compose(new Transform3(new Vec3(0, 0, 0.1), Quat.identity()));
    var expectedWallTTcp = projectTWall.inverse().compose(projectTTcp);

    var wallTTcp = tree.lookup("wall", "tcp");
    check(approxVec(wallTTcp.translation, expectedWallTTcp.translation, 1e-9) &&
      wallTTcp.rotation.angularDistance(expectedWallTTcp.rotation) < 1e-9,
      "FrameTree3 crosses the registration edge from the robot branch to the BIM branch");

    var tcpTWall = tree.lookup("tcp", "wall");
    var roundTrip = wallTTcp.compose(tcpTWall);
    check(approxVec(roundTrip.translation, Vec3.zero(), 1e-9) &&
      roundTrip.rotation.angularDistance(Quat.identity()) < 1e-9,
      "FrameTree3 lookups in opposite directions are inverses");

    throws(function() tree.lookup("wall", "nowhere"), "FrameTree3 rejects an unknown frame");
    throws(function() tree.add(new FrameTransform3("tcp", "wall", Transform3.identity())),
      "FrameTree3 rejects an edge that would create a cycle");
  }

  static function sampleTransforms():Array<Transform3> {
    return [
      new Transform3(new Vec3(1.0, 2.0, -0.5), Quat.fromRollPitchYaw(0.3, -0.1, 0.7)),
      new Transform3(new Vec3(-2.0, 0.4, 1.1), Quat.fromRollPitchYaw(-0.6, 0.2, -0.4)),
      new Transform3(new Vec3(0.2, -0.3, 0.05), Quat.fromRollPitchYaw(0.05, 0.9, 1.2)),
      new Transform3(Vec3.zero(), Quat.identity())
    ];
  }

  static function approx(a:Float, b:Float, tolerance:Float):Bool return Math.abs(a - b) <= tolerance;

  static function approxVec(a:Vec3, b:Vec3, tolerance:Float):Bool
    return approx(a.x, b.x, tolerance) && approx(a.y, b.y, tolerance) && approx(a.z, b.z, tolerance);

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'assertion failed: $message';
  }

  static function throws(action:Void -> Void, message:String):Void {
    var didThrow = false;
    try action() catch (_:Dynamic) didThrow = true;
    check(didThrow, message);
  }
}
