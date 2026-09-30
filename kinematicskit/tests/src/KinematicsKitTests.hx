import kinematicskit.ClosureKind;
import kinematicskit.JointKind;
import kinematicskit.KinematicModel;
import kinematicskit.KinematicModelBuilder;
import kinematicskit.KinematicSnapshot;
import kinematicskit.KinematicState;
import kinematicskit.Transform;
import kinematicskit.Vector3;

class KinematicsKitTests {
  static var assertions = 0;

  public static function main():Void {
    testTransformAlgebra();
    testPlanar2R();
    testPrismaticAndRootPose();
    testBranchingForest();
    testCouplings();
    testJacobianMatchesFiniteDifferences();
    testCompileErrors();
    testClosuresAreNotTreeEdges();
    Sys.println('KinematicsKit tests passed ($assertions assertions)');
  }

  static function testTransformAlgebra():Void {
    var a = new Transform(0.3, -1.2, 2.0, 0.0, 0.0, 0.0, 1.0).compose(Transform.axisAngle(0.0, 0.6, 0.8, 0.9));
    var b = new Transform(-0.5, 0.25, 1.0, 0.0, 0.0, 0.0, 1.0).compose(Transform.axisAngle(1.0, 0.0, 0.0, -1.3));
    var identity = a.compose(a.inverse());
    check(near(identity.x, 0.0, 1e-12) && near(identity.y, 0.0, 1e-12) && near(identity.z, 0.0, 1e-12) &&
      near(Math.abs(identity.qw), 1.0, 1e-12), "a · a⁻¹ is the identity");
    var p = a.compose(b).transformPoint(0.1, 0.2, 0.3);
    var inner = b.transformPoint(0.1, 0.2, 0.3);
    var q = a.transformPoint(inner.x, inner.y, inner.z);
    check(near(p.x, q.x, 1e-12) && near(p.y, q.y, 1e-12) && near(p.z, q.z, 1e-12),
      "(a · b)(p) = a(b(p))");
  }

  static function testPlanar2R():Void {
    var builder = new KinematicModelBuilder();
    var base = builder.addBody("base");
    var link1 = builder.addBody("link1");
    var link2 = builder.addBody("link2");
    var z = new Vector3(0.0, 0.0, 1.0);
    builder.addJoint("j1", JointKind.Revolute, base, link1, Transform.identity(), Transform.identity(), z);
    builder.addJoint("j2", JointKind.Revolute, link1, link2, Transform.translation(1.0, 0.0, 0.0),
      Transform.identity(), z);
    var tip = builder.addFrame("tip", link2, Transform.translation(0.7, 0.0, 0.0));
    var model = builder.build();
    check(model.dofCount() == 2, "planar 2R has two DOFs");
    var state = new KinematicState(model);
    var snapshot = new KinematicSnapshot(model);
    for (sample in [[0.0, 0.0], [0.4, -0.9], [-1.2, 0.6], [Math.PI * 0.5, Math.PI * 0.25]]) {
      state.q[0] = sample[0];
      state.q[1] = sample[1];
      snapshot.evaluate(state);
      var pose = snapshot.framePose(tip);
      check(near(pose.x, Math.cos(sample[0]) + 0.7 * Math.cos(sample[0] + sample[1]), 1e-12) &&
        near(pose.y, Math.sin(sample[0]) + 0.7 * Math.sin(sample[0] + sample[1]), 1e-12) &&
        near(pose.z, 0.0, 1e-12), "planar 2R tip matches the closed form");
      check(near(2.0 * Math.atan2(pose.qz, pose.qw), sample[0] + sample[1], 1e-12),
        "planar 2R tip yaw is q1 + q2");
    }
  }

  static function testPrismaticAndRootPose():Void {
    var builder = new KinematicModelBuilder();
    var base = builder.addBody("base");
    var carriage = builder.addBody("carriage");
    builder.setRootPose(base, Transform.translation(10.0, 0.0, 0.0));
    builder.addJoint("slide", JointKind.Prismatic, base, carriage, Transform.axisAngle(0.0, 0.0, 1.0, Math.PI * 0.5),
      Transform.translation(0.0, 0.0, -1.0), new Vector3(2.0, 0.0, 0.0), -5.0, 5.0);
    var model = builder.build();
    check(model.dofLower[0] == -5.0 && model.dofUpper[0] == 5.0, "prismatic limits become the DOF range");
    var state = new KinematicState(model, [3.0]);
    var pose = KinematicSnapshot.of(state).bodyPose(carriage);
    // The joint frame is turned 90° about Z, so its X axis points along world Y.
    check(near(pose.x, 10.0, 1e-12) && near(pose.y, 3.0, 1e-12) && near(pose.z, -1.0, 1e-12),
      "prismatic motion follows the normalized axis in the joint frame, after the root pose");
    state.setRootPose(base, Transform.identity());
    pose = KinematicSnapshot.of(state).bodyPose(carriage);
    check(near(pose.x, 0.0, 1e-12) && near(pose.y, 3.0, 1e-12), "a state root pose overrides the model's");
    rejects(() -> state.setRootPose(carriage, Transform.identity()), "only roots take root poses");
  }

  static function testBranchingForest():Void {
    var builder = new KinematicModelBuilder();
    var torso = builder.addBody("torso");
    var left = builder.addBody("left");
    var right = builder.addBody("right");
    var table = builder.addBody("table");
    var lid = builder.addBody("lid");
    var z = new Vector3(0.0, 0.0, 1.0);
    builder.addJoint("left_shoulder", JointKind.Revolute, torso, left, Transform.translation(0.0, 0.5, 0.0),
      Transform.identity(), z);
    builder.addJoint("right_shoulder", JointKind.Revolute, torso, right, Transform.translation(0.0, -0.5, 0.0),
      Transform.identity(), z);
    builder.setRootPose(table, Transform.translation(3.0, 0.0, 0.0));
    builder.addJoint("hinge", JointKind.Revolute, table, lid, Transform.identity(), Transform.identity(),
      new Vector3(1.0, 0.0, 0.0));
    var leftHand = builder.addFrame("left_hand", left, Transform.translation(1.0, 0.0, 0.0));
    var rightHand = builder.addFrame("right_hand", right, Transform.translation(1.0, 0.0, 0.0));
    var model = builder.build();
    check(model.dofCount() == 3, "forest exposes every tree DOF");
    var snapshot = KinematicSnapshot.of(new KinematicState(model, [Math.PI * 0.5, -Math.PI * 0.5, 0.2]));
    var l = snapshot.framePose(leftHand), r = snapshot.framePose(rightHand);
    check(near(l.x, 0.0, 1e-12) && near(l.y, 1.5, 1e-12), "left branch tip");
    check(near(r.x, 0.0, 1e-12) && near(r.y, -1.5, 1e-12), "right branch tip");
    check(near(snapshot.bodyPose(lid).x, 3.0, 1e-12), "second root is placed by its own pose");
    var jacobian = snapshot.frameJacobian(leftHand);
    check(jacobian[1] == 0.0 && jacobian[2] == 0.0, "a branch tip does not move with the other branch's DOFs");
  }

  static function testCouplings():Void {
    var builder = new KinematicModelBuilder();
    var base = builder.addBody("base");
    var gearA = builder.addBody("gear_a");
    var gearB = builder.addBody("gear_b");
    var gearC = builder.addBody("gear_c");
    var z = new Vector3(0.0, 0.0, 1.0);
    builder.addJoint("a", JointKind.Revolute, base, gearA, Transform.identity(), Transform.identity(), z, -2.0, 2.0);
    builder.addJoint("b", JointKind.Revolute, base, gearB, Transform.translation(1.0, 0.0, 0.0),
      Transform.identity(), z, -1.0, 3.0);
    builder.addJoint("c", JointKind.Revolute, base, gearC, Transform.translation(2.0, 0.0, 0.0),
      Transform.identity(), z);
    // Chain c <- b <- a, declared target-first to exercise ordering.
    builder.couple("c", "b", 2.0, 0.5);
    builder.couple("b", "a", -0.5, 0.0);
    var model = builder.build();
    check(model.dofCount() == 1 && model.dofId(0) == "a", "coupled joints share their source's DOF");
    check(model.dofIndex("b") == -1, "a coupled joint has no DOF of its own");
    // b = -0.5 a in [-1, 3] gives a in [-6, 2]; with a's own [-2, 2] the range is [-2, 2].
    check(near(model.dofLower[0], -2.0, 1e-12) && near(model.dofUpper[0], 2.0, 1e-12),
      "coupled limits narrow the source range only where tighter");
    var snapshot = KinematicSnapshot.of(new KinematicState(model, [1.0]));
    check(near(snapshot.jointValue(model.jointIndex("b")), -0.5, 1e-15), "b follows a");
    check(near(snapshot.jointValue(model.jointIndex("c")), -0.5, 1e-15), "c follows b: 2 * -0.5 + 0.5");
    check(model.jointScale[model.jointIndex("c")] == -1.0, "c's Jacobian scale is the ratio product");

    var tight = new KinematicModelBuilder();
    var tb = tight.addBody("base"), t1 = tight.addBody("one"), t2 = tight.addBody("two");
    tight.addJoint("p", JointKind.Revolute, tb, t1, Transform.identity(), Transform.identity(), z, -2.0, 2.0);
    tight.addJoint("q", JointKind.Revolute, tb, t2, Transform.identity(), Transform.identity(), z, 0.0, 1.0);
    tight.couple("q", "p", 1.0, 0.0);
    var tightModel = tight.build();
    check(tightModel.dofLower[0] == 0.0 && tightModel.dofUpper[0] == 1.0, "tighter coupled limits win");
  }

  static function testJacobianMatchesFiniteDifferences():Void {
    var rng = new Rng(7);
    var builder = new KinematicModelBuilder();
    var bodies = [builder.addBody("b0")];
    var kinds = [JointKind.Revolute, JointKind.Prismatic, JointKind.Fixed, JointKind.Revolute,
      JointKind.Revolute, JointKind.Prismatic, JointKind.Revolute];
    for (i in 0...kinds.length) {
      var body = builder.addBody('b${i + 1}');
      // Mostly a chain, with one side branch hanging off b2.
      var parent = i == 5 ? bodies[2] : bodies[bodies.length - 1];
      builder.addJoint('j$i', kinds[i], parent, body, randomTransform(rng), randomTransform(rng),
        new Vector3(rng.signed(), rng.signed(), rng.signed() + 0.1));
      bodies.push(body);
    }
    builder.couple("j4", "j3", 0.75, 0.1);
    var tip = builder.addFrame("tip", bodies[bodies.length - 1], randomTransform(rng));
    var side = builder.addFrame("side", bodies[6], randomTransform(rng));
    var model = builder.build();
    var n = model.dofCount();
    check(n == 5, "fixture has five DOFs (one fixed, one coupled)");
    var q = [for (_ in 0...n) rng.signed()];
    var snapshot = KinematicSnapshot.of(new KinematicState(model, q));
    var plus = new KinematicSnapshot(model), minus = new KinematicSnapshot(model);
    var eps = 1e-6;
    for (frame in [tip, side]) {
      var jacobian = snapshot.frameJacobian(frame);
      for (dof in 0...n) {
        var qp = q.copy(); qp[dof] += eps;
        var qm = q.copy(); qm[dof] -= eps;
        plus.evaluate(new KinematicState(model, qp));
        minus.evaluate(new KinematicState(model, qm));
        var a = minus.framePose(frame), b = plus.framePose(frame);
        var w = logMap(a, b);
        var numeric = [(b.x - a.x) / (2 * eps), (b.y - a.y) / (2 * eps), (b.z - a.z) / (2 * eps),
          w.x / (2 * eps), w.y / (2 * eps), w.z / (2 * eps)];
        for (row in 0...6)
          check(near(jacobian[row * n + dof], numeric[row], 1e-6),
            'Jacobian row $row, DOF $dof of frame ${model.frameIds[frame]} matches central differences');
      }
    }
    // A point Jacobian at an offset point is the frame Jacobian shifted by ω × r.
    var pose = snapshot.framePose(tip);
    var px = pose.x + 0.3, py = pose.y - 0.2, pz = pose.z + 0.4;
    var atPoint = snapshot.pointJacobian(model.frameBody[tip], px, py, pz);
    var atFrame = snapshot.frameJacobian(tip);
    for (dof in 0...n) {
      var wx = atFrame[3 * n + dof], wy = atFrame[4 * n + dof], wz = atFrame[5 * n + dof];
      check(near(atPoint[dof], atFrame[dof] + wy * 0.4 - wz * -0.2, 1e-12) &&
        near(atPoint[n + dof], atFrame[n + dof] + wz * 0.3 - wx * 0.4, 1e-12) &&
        near(atPoint[2 * n + dof], atFrame[2 * n + dof] + wx * -0.2 - wy * 0.3, 1e-12),
        'point Jacobian column $dof is the shifted frame Jacobian');
    }
    var reused = [for (_ in 0...100) 9.0];
    snapshot.frameJacobian(tip, reused);
    check(reused.length == 6 * n && reused[0] == atFrame[0], "a supplied output array is resized and filled");
  }

  static function testCompileErrors():Void {
    var z = new Vector3(0.0, 0.0, 1.0);
    rejects(() -> {
      var b = new KinematicModelBuilder();
      var a = b.addBody("a"), c = b.addBody("c"), d = b.addBody("d");
      b.addJoint("one", JointKind.Fixed, a, d, Transform.identity(), Transform.identity());
      b.addJoint("two", JointKind.Fixed, c, d, Transform.identity(), Transform.identity());
      b.build();
    }, "a body with two parent joints is rejected");
    rejects(() -> {
      var b = new KinematicModelBuilder();
      var a = b.addBody("a"), c = b.addBody("c");
      b.addJoint("one", JointKind.Fixed, a, c, Transform.identity(), Transform.identity());
      b.addJoint("two", JointKind.Fixed, c, a, Transform.identity(), Transform.identity());
      b.build();
    }, "a cycle is rejected");
    rejects(() -> {
      var b = new KinematicModelBuilder();
      var a = b.addBody("a"), c = b.addBody("c");
      b.addJoint("one", JointKind.Revolute, a, c, Transform.identity(), Transform.identity(), new Vector3(0, 0, 0));
    }, "a zero axis is rejected");
    rejects(() -> {
      var b = new KinematicModelBuilder();
      var a = b.addBody("a"), c = b.addBody("c"), d = b.addBody("d");
      b.addJoint("one", JointKind.Revolute, a, c, Transform.identity(), Transform.identity(), z);
      b.addJoint("two", JointKind.Revolute, a, d, Transform.identity(), Transform.identity(), z);
      b.couple("two", "one", 1.0, 0.0);
      b.couple("two", "one", 2.0, 0.0);
      b.build();
    }, "a joint driven by two couplings is rejected");
    rejects(() -> {
      var b = new KinematicModelBuilder();
      var a = b.addBody("a"), c = b.addBody("c"), d = b.addBody("d");
      b.addJoint("one", JointKind.Revolute, a, c, Transform.identity(), Transform.identity(), z);
      b.addJoint("two", JointKind.Revolute, a, d, Transform.identity(), Transform.identity(), z);
      b.couple("two", "one", 1.0, 0.0);
      b.couple("one", "two", 1.0, 0.0);
      b.build();
    }, "a coupling cycle is rejected");
    rejects(() -> {
      var b = new KinematicModelBuilder();
      var a = b.addBody("a");
      b.addBody("a");
    }, "duplicate body IDs are rejected");
    rejects(() -> {
      var b = new KinematicModelBuilder();
      var a = b.addBody("a"), c = b.addBody("c");
      b.addJoint("one", JointKind.Fixed, a, c, new Transform(0, 0, 0, 0, 0, 0, 2), Transform.identity());
    }, "a non-unit quaternion is rejected");
  }

  static function testClosuresAreNotTreeEdges():Void {
    var builder = new KinematicModelBuilder();
    var ground = builder.addBody("ground");
    var crank = builder.addBody("crank");
    var z = new Vector3(0.0, 0.0, 1.0);
    builder.addJoint("pivot", JointKind.Revolute, ground, crank, Transform.identity(), Transform.identity(), z);
    var pin = builder.addFrame("pin", crank, Transform.translation(1.0, 0.0, 0.0));
    var anchor = builder.addFrame("anchor", ground, Transform.translation(1.0, 0.0, 0.0));
    builder.addClosure("loop", ClosureKind.Revolute, pin, anchor, z, 1e-3);
    var model = builder.build();
    check(model.closureCount() == 1 && model.closureIndex("loop") == 0, "closures are recorded");
    check(model.bodyParentJoint[ground] == -1, "a closure does not make its body a child");
    rejects(() -> {
      var b = new KinematicModelBuilder();
      var a = b.addBody("a");
      var f = b.addFrame("f", a, Transform.identity());
      b.addClosure("bad", ClosureKind.Revolute, f, f);
    }, "a revolute closure needs an axis");
  }

  // -- helpers ---------------------------------------------------------

  static function randomTransform(rng:Rng):Transform {
    var ax = rng.signed(), ay = rng.signed(), az = rng.signed() + 0.1;
    var norm = Math.sqrt(ax * ax + ay * ay + az * az);
    return new Transform(rng.signed(), rng.signed(), rng.signed(), 0.0, 0.0, 0.0, 1.0)
      .compose(Transform.axisAngle(ax / norm, ay / norm, az / norm, rng.signed() * 2.0));
  }

  /** Rotation vector of `to · from⁻¹`, in world coordinates. */
  static function logMap(from:Transform, to:Transform):Vector3 {
    var delta = to.compose(new Transform(0, 0, 0, -from.qx, -from.qy, -from.qz, from.qw));
    var x = delta.qx, y = delta.qy, z = delta.qz, w = delta.qw;
    if (w < 0.0) { x = -x; y = -y; z = -z; w = -w; }
    var s = Math.sqrt(x * x + y * y + z * z);
    if (s < 1e-15) return new Vector3(2 * x, 2 * y, 2 * z);
    var angle = 2.0 * Math.atan2(s, w);
    return new Vector3(x / s * angle, y / s * angle, z / s * angle);
  }

  static function near(a:Float, b:Float, tolerance:Float):Bool return Math.abs(a - b) <= tolerance;

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'assertion failed: $message';
  }

  static function rejects(action:Void->Void, message:String):Void {
    var threw = false;
    try action() catch (_:Dynamic) threw = true;
    check(threw, message);
  }
}

/** Deterministic linear congruential generator. */
private class Rng {
  var state:Int;

  public function new(seed:Int) state = seed;

  /** A value in [-1, 1). */
  public function signed():Float {
    state = (state * 1103515245 + 12345) & 0x7fffffff;
    return state / 2147483647.0 * 2.0 - 1.0;
  }
}
