import kinematicskit.ClosureKind;
import kinematicskit.ClosureTask;
import kinematicskit.DampedLeastSquares;
import kinematicskit.FrameOrientation;
import kinematicskit.FrameTask;
import kinematicskit.JacobianLayout;
import kinematicskit.LookAtTask;
import kinematicskit.SolverWorkspace;
import kinematicskit.SwivelTask;
import kinematicskit.RootDampingTask;
import kinematicskit.RootMotion;
import kinematicskit.SolverSupport;
import kinematicskit.KinematicProblem;
import kinematicskit.KinematicStatus;
import kinematicskit.LevenbergMarquardt;
import kinematicskit.PostureTask;
import kinematicskit.PrioritizedSolver;
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
    testMultiLeaderCouplings();
    testJacobianMatchesFiniteDifferences();
    testCompileErrors();
    testClosuresAreNotTreeEdges();
    testDampedLeastSquaresReachesAndReports();
    testMaskedPositionAndAxisTasks();
    testPostureResolvesRedundancy();
    testClosureJacobiansMatchFiniteDifferences();
    testFourBarClosure();
    testDualArmTargets();
    testCameraLooksAtPart();
    testLargeAssemblyUsesActiveColumns();
    testNoAllocationPerIteration();
    testSwivelJacobian();
    testRelativeFrameTask();
    testPrioritizedSolver();
    testRootJacobianMatchesFiniteDifferences();
    testMobileManipulatorDrivesWhenTheArmCannotReach();
    testFloatingBodyIsPlacedExactly();
    testDampedLeastSquaresChecksItsLastStep();
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

  static function testMultiLeaderCouplings():Void {
    // A CoreXY head: slides x and y; belt joints a = x + y, b = x - y, and a joint following a.
    var builder = new KinematicModelBuilder();
    var base = builder.addBody("base"), bx = builder.addBody("bx"), head = builder.addBody("head");
    var ba = builder.addBody("ba"), bb = builder.addBody("bb"), bc = builder.addBody("bc");
    var z = new Vector3(0.0, 0.0, 1.0);
    builder.addJoint("x", JointKind.Prismatic, base, bx, Transform.identity(), Transform.identity(), new Vector3(1.0, 0.0, 0.0), -1.0, 1.0);
    builder.addJoint("y", JointKind.Prismatic, bx, head, Transform.identity(), Transform.identity(), new Vector3(0.0, 1.0, 0.0), -1.0, 1.0);
    builder.addJoint("a", JointKind.Revolute, base, ba, Transform.identity(), Transform.identity(), z);
    builder.addJoint("b", JointKind.Revolute, base, bb, Transform.identity(), Transform.identity(), z);
    builder.addJoint("c", JointKind.Revolute, base, bc, Transform.identity(), Transform.identity(), z);
    var tip = builder.addFrame("tip", ba, Transform.translation(1.0, 0.0, 0.0));
    builder.couple("a", "x", 1.0, 0.0);
    builder.couple("a", "y", 1.0, 0.25);
    builder.couple("b", "x", 1.0, 0.0);
    builder.couple("b", "y", -1.0, 0.0);
    builder.couple("c", "a", 2.0, 0.0);
    var model = builder.build();
    check(model.dofCount() == 2, "the leaders are the only DOFs");
    var snapshot = KinematicSnapshot.of(new KinematicState(model, [0.5, 0.25]));
    check(near(snapshot.jointValue(model.jointIndex("a")), 1.0, 1e-15), "a = x + y + offset");
    check(near(snapshot.jointValue(model.jointIndex("b")), 0.25, 1e-15), "b = x - y");
    check(near(snapshot.jointValue(model.jointIndex("c")), 2.0, 1e-15), "c follows the combined a");
    var ia = model.jointIndex("a");
    check(model.jointTermStart[ia + 1] - model.jointTermStart[ia] == 2, "a sums two terms");
    check(model.dofIndex("a") == -1, "a combined joint has no DOF of its own");
    var jacobian = snapshot.frameJacobian(tip);
    // Rotating about z by a = x + y moves the tip along y: d(tipY)/dx = d(tipY)/dy = cos(a).
    check(near(jacobian[model.dofCount() + 0], Math.cos(1.0), 1e-12) && near(jacobian[model.dofCount() + 1], Math.cos(1.0), 1e-12),
      "a combined joint's Jacobian sums its terms");
    var threw = false;
    var twice = new KinematicModelBuilder();
    var tb = twice.addBody("base"), t1 = twice.addBody("one"), t2 = twice.addBody("two");
    twice.addJoint("p", JointKind.Revolute, tb, t1, Transform.identity(), Transform.identity(), z);
    twice.addJoint("q", JointKind.Revolute, tb, t2, Transform.identity(), Transform.identity(), z);
    twice.couple("p", "q", 1.0, 0.0);
    twice.couple("q", "p", 1.0, 0.0);
    try twice.build() catch (error:Dynamic) threw = true;
    check(threw, "terms that form a cycle are rejected");
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

  static function planarArm(links:Array<Float>):KinematicModel {
    var builder = new KinematicModelBuilder();
    var previous = builder.addBody("base");
    var z = new Vector3(0.0, 0.0, 1.0);
    var reach = 0.0;
    for (i in 0...links.length) {
      var body = builder.addBody('link$i');
      builder.addJoint('j$i', JointKind.Revolute, previous, body, Transform.translation(reach, 0.0, 0.0),
        Transform.identity(), z, -3.0, 3.0);
      reach = links[i];
      previous = body;
    }
    builder.addFrame("tip", previous, Transform.translation(reach, 0.0, 0.0));
    return builder.build();
  }

  static function testDampedLeastSquaresReachesAndReports():Void {
    var model = planarArm([1.0, 0.7]);
    var tip = model.frameIndex("tip");
    var truth = KinematicSnapshot.of(new KinematicState(model, [0.5, -0.8])).framePose(tip);
    var problem = new KinematicProblem(model)
      .add(FrameTask.atFrame(model, tip, truth, 1e-6, 1e-6));
    var solution = DampedLeastSquares.solve(problem, new KinematicState(model, [0.3, -0.4]), 200, 0.01);
    check(solution.status == KinematicStatus.Converged && solution.tasks[0].positionError <= 1e-6,
      "DLS reaches a reachable planar pose");
    check(solution.rank == 2 && solution.freeDofs == 0, "a planar pose task has full rank over two DOFs");

    var far = Transform.translation(10.0, 0.0, 0.0);
    var unreachable = DampedLeastSquares.solve(new KinematicProblem(model)
      .add(FrameTask.atFrame(model, tip, far, 1e-6, 1e-6, null, FrameTask.AXIS_X | FrameTask.AXIS_Y, FrameOrientation.Free)),
      new KinematicState(model, [0.3, 0.2]), 25, 0.02);
    check(unreachable.status == KinematicStatus.IterationLimit && unreachable.iterations == 25,
      "an unreachable target reports the iteration limit");
    check(unreachable.unsatisfied()[0] == "tip", "the solution names the unmet task");

    var seed = new KinematicState(model, [0.3, -0.4]);
    DampedLeastSquares.solve(problem, seed);
    check(seed.q[0] == 0.3 && seed.q[1] == -0.4, "solvers leave the seed untouched");
  }

  static function testMaskedPositionAndAxisTasks():Void {
    // Three planar links: a position-only target leaves one DOF free.
    var model = planarArm([1.0, 0.8, 0.5]);
    var tip = model.frameIndex("tip");
    var target = new Transform(1.2, 0.9, 0.0, 0.0, 0.0, 0.0, 1.0);
    var solution = LevenbergMarquardt.solve(new KinematicProblem(model)
      .add(FrameTask.atFrame(model, tip, target, 1e-8, 1e-8, null, FrameTask.AXIS_X | FrameTask.AXIS_Y,
        FrameOrientation.Free)), new KinematicState(model, [0.2, 0.3, 0.1]));
    var pose = KinematicSnapshot.of(solution.state).framePose(tip);
    check(solution.converged() && near(pose.x, 1.2, 1e-7) && near(pose.y, 0.9, 1e-7),
      "a position-only task converges");
    check(solution.rank == 2 && solution.freeDofs == 1, "the unconstrained orientation shows up as one free DOF");

    // A spatial wrist: align the tool's Z with a target Z and ignore roll about it.
    var builder = new KinematicModelBuilder();
    var base = builder.addBody("base");
    var bodies = [base];
    var axes = [new Vector3(0, 0, 1), new Vector3(0, 1, 0), new Vector3(0, 1, 0), new Vector3(1, 0, 0),
      new Vector3(0, 1, 0), new Vector3(1, 0, 0)];
    for (i in 0...6) {
      var body = builder.addBody('w$i');
      builder.addJoint('w$i', JointKind.Revolute, bodies[i], body,
        Transform.translation(i == 1 ? 0.0 : 0.3, 0.0, i == 0 ? 0.4 : 0.0), Transform.identity(), axes[i]);
      bodies.push(body);
    }
    var tool = builder.addFrame("tool", bodies[6], Transform.translation(0.1, 0.0, 0.0));
    var arm = builder.build();
    var goal = KinematicSnapshot.of(new KinematicState(arm, [0.4, -0.3, 0.6, 0.2, -0.5, 1.1])).framePose(tool);
    var rolled = goal.compose(Transform.axisAngle(0.0, 0.0, 1.0, 1.3));
    var aligned = LevenbergMarquardt.solve(new KinematicProblem(arm)
      .add(FrameTask.atFrame(arm, tool, rolled, 1e-8, 1e-8, null, FrameTask.ALL_AXES, FrameOrientation.Axis(0, 0, 1))),
      new KinematicState(arm, [0.3, -0.2, 0.5, 0.0, -0.4, 0.2]));
    var reached = KinematicSnapshot.of(aligned.state).framePose(tool);
    var z = reached.transformVector(0, 0, 1), goalZ = rolled.transformVector(0, 0, 1);
    check(aligned.converged() && near(z.x * goalZ.x + z.y * goalZ.y + z.z * goalZ.z, 1.0, 1e-12),
      "an axis task aligns the tool Z");
    check(aligned.freeDofs == 1, "roll about the aligned axis is left free");
  }

  static function testPostureResolvesRedundancy():Void {
    var model = planarArm([1.0, 0.8, 0.5]);
    var tip = model.frameIndex("tip");
    var target = new Transform(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    function solveNear(posture:Array<Float>):Array<Float> {
      var problem = new KinematicProblem(model)
        .add(FrameTask.atFrame(model, tip, target, 1e-6, 1e-6, null, FrameTask.AXIS_X | FrameTask.AXIS_Y,
          FrameOrientation.Free))
        .add(new PostureTask(model, posture, 1e-3));
      var solution = DampedLeastSquares.solve(problem, new KinematicState(model, posture), 500, 0.01);
      check(solution.converged(), "a soft posture task does not block convergence");
      return solution.state.q;
    }
    var elbowUp = solveNear([0.2, 1.2, 0.8]);
    var elbowDown = solveNear([1.4, -1.2, -0.3]);
    check(elbowUp[1] > 0.0 && elbowDown[1] < 0.0, "the posture preference picks the elbow branch");
  }

  static function testClosureJacobiansMatchFiniteDifferences():Void {
    var rng = new Rng(3);
    for (kind in [ClosureKind.Fixed, ClosureKind.Revolute, ClosureKind.Prismatic]) {
      var builder = new KinematicModelBuilder();
      var root = builder.addBody("root");
      var left = builder.addBody("left"), leftTip = builder.addBody("left_tip");
      var right = builder.addBody("right");
      builder.addJoint("l1", JointKind.Revolute, root, left, randomTransform(rng), randomTransform(rng), new Vector3(0, 0, 1));
      builder.addJoint("l2", JointKind.Prismatic, left, leftTip, randomTransform(rng), randomTransform(rng), new Vector3(1, 0, 0));
      builder.addJoint("r1", JointKind.Revolute, root, right, randomTransform(rng), randomTransform(rng), new Vector3(0, 1, 0));
      var a = builder.addFrame("a", leftTip, randomTransform(rng));
      var b = builder.addFrame("b", right, randomTransform(rng));
      builder.addClosure("loop", kind, a, b, new Vector3(0.2, -0.4, 1.0));
      var model = builder.build();
      var task = new ClosureTask(model, 0, 1.0, 1.0);
      var rows = task.rowCount(), n = model.dofCount();
      var q = [rng.signed(), rng.signed(), rng.signed()];
      var residual = [for (_ in 0...rows) 0.0], jacobian = [for (_ in 0...rows * n) 0.0];
      var snapshot = new KinematicSnapshot(model);
      var state = new KinematicState(model, q);
      var layout = JacobianLayout.all(model);
      snapshot.evaluate(state);
      task.evaluate(state, snapshot, layout, residual, jacobian, 0);
      var plus = residual.copy(), minus = residual.copy(), scratch = jacobian.copy();
      // Rows whose Jacobian is exact away from closure: all six for Fixed, the three position rows for Revolute.
      var exactRows = kind == ClosureKind.Fixed ? 6 : kind == ClosureKind.Revolute ? 3 : 0;
      for (dof in 0...n) {
        var eps = 1e-6;
        var qp = q.copy(); qp[dof] += eps;
        var qm = q.copy(); qm[dof] -= eps;
        var sp = new KinematicState(model, qp), sm = new KinematicState(model, qm);
        snapshot.evaluate(sp); task.evaluate(sp, snapshot, layout, plus, scratch, 0);
        snapshot.evaluate(sm); task.evaluate(sm, snapshot, layout, minus, scratch, 0);
        for (row in 0...exactRows)
          check(near(-(plus[row] - minus[row]) / (2 * eps), jacobian[row * n + dof], 1e-6),
            'closure ${Std.string(kind)} row $row DOF $dof Jacobian matches central differences');
      }
    }
  }

  /**
   * Crank-rocker four-bar: ground pivots 2 apart, crank 1, coupler 2.2, rocker 1.5.
   * The crank is driven; coupler and rocker angles are solved.
   */
  static function fourBar(?rockerLower:Float, ?rockerUpper:Float):KinematicModel {
    var builder = new KinematicModelBuilder();
    var ground = builder.addBody("ground");
    var crank = builder.addBody("crank"), coupler = builder.addBody("coupler"), rocker = builder.addBody("rocker");
    var z = new Vector3(0.0, 0.0, 1.0);
    builder.addJoint("crank", JointKind.Revolute, ground, crank, Transform.identity(), Transform.identity(), z);
    builder.addJoint("coupler", JointKind.Revolute, crank, coupler, Transform.translation(1.0, 0.0, 0.0),
      Transform.identity(), z);
    builder.addJoint("rocker", JointKind.Revolute, ground, rocker, Transform.translation(2.0, 0.0, 0.0),
      Transform.identity(), z, rockerLower, rockerUpper);
    var couplerEnd = builder.addFrame("coupler_end", coupler, Transform.translation(2.2, 0.0, 0.0));
    var rockerEnd = builder.addFrame("rocker_end", rocker, Transform.translation(1.5, 0.0, 0.0));
    builder.addClosure("pin", ClosureKind.Revolute, couplerEnd, rockerEnd, z);
    return builder.build();
  }

  static function testFourBarClosure():Void {
    var model = fourBar();
    function solve(model:KinematicModel, crank:Float, seed:Array<Float>) {
      var problem = new KinematicProblem(model)
        .setActiveDofs([model.dofIndex("coupler"), model.dofIndex("rocker")])
        .add(new ClosureTask(model, 0, 1e-9, 1e-9));
      return LevenbergMarquardt.solve(problem, new KinematicState(model, [crank, seed[0], seed[1]]));
    }
    var closed = solve(model, 0.6, [0.2, 1.4]);
    check(closed.converged() && closed.freeDofs == 0 && closed.state.q[0] == 0.6,
      "the four-bar closes with the crank held at its seed");
    var snapshot = KinematicSnapshot.of(closed.state);
    var a = snapshot.framePose(model.frameIndex("coupler_end")), b = snapshot.framePose(model.frameIndex("rocker_end"));
    check(near(a.x, b.x, 1e-8) && near(a.y, b.y, 1e-8), "the coupler and rocker ends meet");

    // A 5.0 coupler cannot close: the residual stops at a stationary configuration.
    var builder = new KinematicModelBuilder();
    var ground = builder.addBody("ground"), link = builder.addBody("link");
    var z = new Vector3(0.0, 0.0, 1.0);
    builder.addJoint("swing", JointKind.Revolute, ground, link, Transform.identity(), Transform.identity(), z);
    var end = builder.addFrame("end", link, Transform.translation(1.0, 0.0, 0.0));
    var anchor = builder.addFrame("anchor", ground, Transform.translation(5.0, 0.0, 0.0));
    builder.addClosure("pin", ClosureKind.Revolute, end, anchor, z);
    var impossible = builder.build();
    var conflicting = LevenbergMarquardt.solve(new KinematicProblem(impossible).add(new ClosureTask(impossible, 0, 1e-9, 1e-9)),
      new KinematicState(impossible, [0.3]));
    check(conflicting.status == KinematicStatus.Conflicting && conflicting.unsatisfied()[0] == "pin",
      "an unclosable loop reports Conflicting and names the closure");

    // Limit the rocker below the angle it needs.
    var limited = fourBar(1.8, 2.0);
    var blocked = solve(limited, 0.6, [0.2, 1.9]);
    var needs = closed.state.q[2];
    check(needs < 1.8, 'fixture sanity: the closed rocker angle $needs is below the limit');
    check(blocked.status == KinematicStatus.LimitBlocked && blocked.limitHits.indexOf(limited.dofIndex("rocker")) >= 0,
      "a limit in the way reports LimitBlocked and the limited DOF");
  }

  /** Torso yaw, then two 3-joint planar-ish arms offset ±0.3 along Y, with hand frames. */
  static function dualArm():KinematicModel {
    var builder = new KinematicModelBuilder();
    var base = builder.addBody("base"), torso = builder.addBody("torso");
    var z = new Vector3(0, 0, 1), y = new Vector3(0, 1, 0);
    builder.addJoint("torso", JointKind.Revolute, base, torso, Transform.translation(0, 0, 1.0), Transform.identity(), z,
      -1.5, 1.5);
    for (side in ["left", "right"]) {
      var previous = torso;
      var offsets = [Transform.translation(0, side == "left" ? 0.3 : -0.3, 0.2), Transform.translation(0.3, 0, 0),
        Transform.translation(0.25, 0, 0)];
      var axes = [z, y, y];
      for (i in 0...3) {
        var body = builder.addBody('${side}_$i');
        builder.addJoint('${side}_$i', JointKind.Revolute, previous, body, offsets[i], Transform.identity(), axes[i], -2.5, 2.5);
        previous = body;
      }
      builder.addFrame('${side}_hand', previous, Transform.translation(0.2, 0, 0));
    }
    return builder.build();
  }

  static function testDualArmTargets():Void {
    var model = dualArm();
    var truth = new KinematicState(model, [0.2, 0.4, -0.5, 0.9, -0.3, 0.6, -0.8]);
    var snapshot = KinematicSnapshot.of(truth);
    var left = model.frameIndex("left_hand"), right = model.frameIndex("right_hand");
    var leftGoal = snapshot.framePose(left), rightGoal = snapshot.framePose(right);
    var position = FrameTask.ALL_AXES;
    var seed = new KinematicState(model, [0.1, 0.3, -0.3, 0.7, -0.2, 0.4, -0.6]);

    // Both hands in one solve, torso shared.
    var both = LevenbergMarquardt.solve(new KinematicProblem(model)
      .add(FrameTask.atFrame(model, left, leftGoal, 1e-8, 1e-8, null, position, FrameOrientation.Free))
      .add(FrameTask.atFrame(model, right, rightGoal, 1e-8, 1e-8, null, position, FrameOrientation.Free)), seed);
    check(both.converged() && both.tasks[0].satisfied && both.tasks[1].satisfied, "one solve reaches both hand targets");

    // The arms alone, torso held: the right arm's solve never touches the left arm or torso.
    var rightOnly = LevenbergMarquardt.solve(new KinematicProblem(model)
      .setActiveJoints(["right_0", "right_1", "right_2"])
      .add(FrameTask.atFrame(model, right, rightGoal, 1e-8, 1e-8, null, position, FrameOrientation.Free)),
      new KinematicState(model, [0.2, 0.4, -0.5, 0.9, -0.2, 0.4, -0.6]));
    check(rightOnly.converged() && rightOnly.state.q[0] == 0.2 && rightOnly.state.q[1] == 0.4,
      "an arm solved alone leaves the torso and the other arm at the seed");
    check(rightOnly.freeDofs == 0, "three joints fully used by a position target");
    rejects(() -> new KinematicProblem(model).setActiveJoints(["nope"]), "unknown joint IDs are rejected");
  }

  static function testCameraLooksAtPart():Void {
    var model = dualArm();
    var left = model.frameIndex("left_hand");
    var part = new Vector3(0.6, 0.8, 0.9);
    // Keep the hand at a height of 1.2 while its X axis looks at the part.
    var hold = FrameTask.atFrame(model, left, Transform.translation(0, 0, 1.2), 1e-8, 1e-8, null, FrameTask.AXIS_Z,
      FrameOrientation.Free);
    var look = LookAtTask.atFrame(model, left, new Vector3(1, 0, 0), part, 1e-8);
    var solution = LevenbergMarquardt.solve(new KinematicProblem(model)
      .setActiveJoints(["torso", "left_0", "left_1", "left_2"]).add(hold).add(look),
      new KinematicState(model, [0.1, 0.3, -0.3, 0.7, 0.0, 0.0, 0.0]));
    var pose = KinematicSnapshot.of(solution.state).framePose(left);
    var axis = pose.transformVector(1, 0, 0);
    var lx = part.x - pose.x, ly = part.y - pose.y, lz = part.z - pose.z;
    var distance = Math.sqrt(lx * lx + ly * ly + lz * lz);
    check(solution.converged() && near(pose.z, 1.2, 1e-7), "the hand holds its height");
    check(near((axis.x * lx + axis.y * ly + axis.z * lz) / distance, 1.0, 1e-12), "the hand's X axis points at the part");
  }

  static function testLargeAssemblyUsesActiveColumns():Void {
    // A four-bar next to 200 unrelated hinged plates.
    var builder = new KinematicModelBuilder();
    var ground = builder.addBody("ground");
    var z = new Vector3(0.0, 0.0, 1.0);
    for (i in 0...200) {
      var plate = builder.addBody('plate$i');
      builder.addJoint('plate$i', JointKind.Revolute, ground, plate, Transform.translation(10.0 + i, 0.0, 0.0),
        Transform.identity(), z);
    }
    var crank = builder.addBody("crank"), coupler = builder.addBody("coupler"), rocker = builder.addBody("rocker");
    builder.addJoint("crank", JointKind.Revolute, ground, crank, Transform.identity(), Transform.identity(), z);
    builder.addJoint("coupler", JointKind.Revolute, crank, coupler, Transform.translation(1.0, 0.0, 0.0), Transform.identity(), z);
    builder.addJoint("rocker", JointKind.Revolute, ground, rocker, Transform.translation(2.0, 0.0, 0.0), Transform.identity(), z);
    var a = builder.addFrame("coupler_end", coupler, Transform.translation(2.2, 0.0, 0.0));
    var b = builder.addFrame("rocker_end", rocker, Transform.translation(1.5, 0.0, 0.0));
    builder.addClosure("pin", ClosureKind.Revolute, a, b, z);
    var large = builder.build();
    var seed = [for (_ in 0...large.dofCount()) 0.0];
    seed[large.dofIndex("crank")] = 0.6; seed[large.dofIndex("coupler")] = 0.2; seed[large.dofIndex("rocker")] = 1.4;
    var workspace = new SolverWorkspace();
    var solution = LevenbergMarquardt.solve(new KinematicProblem(large).setActiveJoints(["coupler", "rocker"])
      .add(new ClosureTask(large, 0, 1e-9, 1e-9)), new KinematicState(large, seed), 100, 1e-3, 1e-8, 1.0, workspace);
    check(solution.converged() && solution.freeDofs == 0, "the linkage closes inside a 203-DOF model");
    check(workspace.jacobian.length == 5 * 2, 'the Jacobian holds 5 rows x 2 active columns (${workspace.jacobian.length})');
    var small = fourBar();
    var reference = LevenbergMarquardt.solve(new KinematicProblem(small).setActiveJoints(["coupler", "rocker"])
      .add(new ClosureTask(small, 0, 1e-9, 1e-9)), new KinematicState(small, [0.6, 0.2, 1.4]));
    check(solution.state.q[large.dofIndex("rocker")] == reference.state.q[2] &&
      solution.iterations == reference.iterations, "unrelated joints do not change the answer");
  }

  static function testNoAllocationPerIteration():Void {
    // DLS towards an unreachable point runs its whole budget.
    var arm = planarArm([1.0, 0.8, 0.5]);
    var dlsProblem = new KinematicProblem(arm)
      .add(FrameTask.atFrame(arm, arm.frameIndex("tip"), Transform.translation(10.0, 0.0, 0.0), 1e-9, 1e-9))
      .add(new PostureTask(arm, [0.0, 0.0, 0.0], 1e-3));
    var seed = new KinematicState(arm, [0.2, 0.3, 0.1]);
    var workspace = new SolverWorkspace();
    var perIteration = bytesPerIteration(maxIterations -> DampedLeastSquares.solve(dlsProblem, seed, maxIterations,
      0.02, 1e-8, workspace).iterations);
    check(perIteration == 0.0, 'DLS allocates nothing per iteration ($perIteration bytes)');

    // LM on a four-bar whose limit stops it short of closing also runs its whole budget.
    var limited = fourBar(1.8, 2.0);
    var lmProblem = new KinematicProblem(limited).setActiveJoints(["coupler", "rocker"])
      .add(new ClosureTask(limited, 0, 1e-9, 1e-9));
    var lmSeed = new KinematicState(limited, [0.6, 0.2, 1.9]);
    perIteration = bytesPerIteration(maxIterations -> LevenbergMarquardt.solve(lmProblem, lmSeed, maxIterations, 1e-3,
      1e-8, 1.0, workspace).iterations);
    check(perIteration == 0.0, 'LM allocates nothing per iteration ($perIteration bytes)');
  }

  /** Bytes allocated by a 60-iteration run beyond a 10-iteration run, per extra iteration. */
  static function bytesPerIteration(run:Int->Int):Float {
    run(10); run(60); // warm up (JIT, lazily sized buffers)
    var before = hl.Gc.totalAllocated();
    var shortIterations = run(10);
    var middle = hl.Gc.totalAllocated();
    var longIterations = run(60);
    var after = hl.Gc.totalAllocated();
    if (longIterations <= shortIterations) throw 'fixture did not run longer ($shortIterations vs $longIterations)';
    return ((after - middle) - (middle - before)) / (longIterations - shortIterations);
  }

  /** A 7-axis arm with alternating Z/Y axes: spherical shoulder at 0.34, elbow 0.4 above, spherical wrist 0.4 above. */
  public static function sevenAxisArm():KinematicModel {
    var builder = new KinematicModelBuilder();
    var previous = builder.addBody("base");
    var z = new Vector3(0, 0, 1), y = new Vector3(0, 1, 0);
    var offsets = [0.34, 0.0, 0.4, 0.0, 0.4, 0.0, 0.0];
    for (i in 0...7) {
      var body = builder.addBody('link$i');
      builder.addJoint('a$i', JointKind.Revolute, previous, body, Transform.translation(0, 0, i == 0 ? 0.0 : offsets[i - 1]),
        Transform.identity(), i % 2 == 0 ? z : y, -2.9, 2.9);
      previous = body;
    }
    builder.addFrame("flange", previous, Transform.translation(0, 0, 0.126));
    return builder.build();
  }

  static function testSwivelJacobian():Void {
    var model = sevenAxisArm();
    var origin = new Vector3(0, 0, 0);
    // Shoulder at link1's origin, elbow at link3's, wrist at link5's.
    var task = new SwivelTask(model, model.bodyIndex("link1"), origin, model.bodyIndex("link3"), origin,
      model.bodyIndex("link5"), origin, 0.0, 1e-9, new Vector3(1, 0, 0));
    var q = [0.3, 0.6, 0.5, -1.2, 0.3, 0.8, 0.2];
    var layout = JacobianLayout.all(model);
    var snapshot = KinematicSnapshot.of(new KinematicState(model, q));
    var residual = [0.0], jacobian = [for (_ in 0...7) 0.0];
    task.evaluate(new KinematicState(model, q), snapshot, layout, residual, jacobian, 0);
    var eps = 1e-6;
    for (dof in 0...7) {
      var qp = q.copy(); qp[dof] += eps;
      var qm = q.copy(); qm[dof] -= eps;
      var numeric = (task.angle(KinematicSnapshot.of(new KinematicState(model, qp))) -
        task.angle(KinematicSnapshot.of(new KinematicState(model, qm)))) / (2 * eps);
      check(near(jacobian[dof], numeric, 1e-6), 'swivel Jacobian column $dof matches central differences (${jacobian[dof]} vs $numeric)');
    }
    // The wrist joints (a4..a6) turn the hand, not the elbow: their columns vanish.
    check(Math.abs(jacobian[4]) < 1e-9 && Math.abs(jacobian[5]) < 1e-9 && Math.abs(jacobian[6]) < 1e-9,
      "joints beyond the wrist point do not change the swivel");
  }

  /** A 3-joint arm and a turntable on one base: the workcell of a robot with a positioner. */
  static function cellWithPositioner():KinematicModel {
    var builder = new KinematicModelBuilder();
    var base = builder.addBody("base");
    var z = new Vector3(0, 0, 1), y = new Vector3(0, 1, 0);
    var shoulder = builder.addBody("shoulder"), upper = builder.addBody("upper"), fore = builder.addBody("fore");
    builder.addJoint("a0", JointKind.Revolute, base, shoulder, Transform.translation(0, 0, 0.4), Transform.identity(), z, -3, 3);
    builder.addJoint("a1", JointKind.Revolute, shoulder, upper, Transform.identity(), Transform.identity(), y, -3, 3);
    builder.addJoint("a2", JointKind.Revolute, upper, fore, Transform.translation(0.5, 0, 0), Transform.identity(), y, -3, 3);
    builder.addFrame("tool", fore, Transform.translation(0.4, 0, 0));
    var table = builder.addBody("table");
    builder.addJoint("p0", JointKind.Revolute, base, table, Transform.translation(0.7, 0.2, 0.1), Transform.identity(), z, -3, 3);
    builder.addFrame("work", table, Transform.translation(0.1, 0.05, 0.05));
    return builder.build();
  }

  static function testRelativeFrameTask():Void {
    var model = cellWithPositioner();
    var tool = model.frameIndex("tool"), work = model.frameIndex("work");
    var q = [0.3, -0.4, 0.9, 0.5];
    var goal = new Transform(0.02, -0.03, 0.08, 0, 0, 0, 1);
    function task():FrameTask
      return FrameTask.atFrame(model, tool, goal, 1e-9, 1e-9, null, FrameTask.ALL_AXES, FrameOrientation.Free)
        .relativeTo(model, model.frameBody[work], model.frameTransform(work));
    var probe = task(), layout = JacobianLayout.all(model);
    var residual = [0.0, 0.0, 0.0], jacobian = [for (_ in 0...12) 0.0];
    probe.evaluate(new KinematicState(model, q), KinematicSnapshot.of(new KinematicState(model, q)), layout, residual,
      jacobian, 0);
    // The residual is the world-frame gap to the moving target; its derivative is minus the rows.
    function gap(values:Array<Float>):Array<Float> {
      var r = [0.0, 0.0, 0.0], j = [for (_ in 0...12) 0.0];
      task().evaluate(new KinematicState(model, values), KinematicSnapshot.of(new KinematicState(model, values)), layout,
        r, j, 0);
      return r;
    }
    var eps = 1e-6;
    for (dof in 0...4) {
      var qp = q.copy(); qp[dof] += eps;
      var qm = q.copy(); qm[dof] -= eps;
      var plus = gap(qp), minus = gap(qm);
      for (row in 0...3) {
        var numeric = -(plus[row] - minus[row]) / (2 * eps);
        check(near(jacobian[row * 4 + dof], numeric, 1e-6),
          'relative frame rows match central differences (row $row, dof $dof: ${jacobian[row * 4 + dof]} vs $numeric)');
      }
    }
    // Solved with both sides free, the tool lands on the workpiece target and the turntable helps.
    var solve = task();
    var problem = new KinematicProblem(model).add(solve);
    var solution = DampedLeastSquares.solve(problem, new KinematicState(model, q), 200);
    check(solution.converged(), 'the relative target is reached (${solution.status})');
    var reached = solve.relativePose(KinematicSnapshot.of(solution.state));
    check(Math.abs(reached.x - goal.x) < 1e-8 && Math.abs(reached.y - goal.y) < 1e-8 && Math.abs(reached.z - goal.z) < 1e-8,
      'the tool sits at the target in the work frame (${reached.x}, ${reached.y}, ${reached.z})');
    check(Math.abs(solution.state.q[3] - q[3]) > 1e-4, "the positioner moves as well as the arm");
  }

  static function testPrioritizedSolver():Void {
    var model = sevenAxisArm();
    var flange = model.frameIndex("flange");
    var reach = [0.3, 0.6, 0.5, -1.2, 0.3, 0.8, 0.2];
    var goal = KinematicSnapshot.of(new KinematicState(model, reach)).framePose(flange);
    var seed = new KinematicState(model, [0.25, 0.55, 0.45, -1.15, 0.25, 0.75, 0.15]);
    // The preferred posture is another solution of the same tool pose, at a different swivel: reachable only by
    // moving along the arm's self-motion, which the tool target leaves free.
    var origin = new Vector3(0, 0, 0);
    var swivel = new SwivelTask(model, model.bodyIndex("link1"), origin, model.bodyIndex("link3"), origin,
      model.bodyIndex("link5"), origin, 0.0, 1e-9, new Vector3(1, 0, 0));
    swivel.target = swivel.angle(KinematicSnapshot.of(new KinematicState(model, reach))) + 0.6;
    var other = DampedLeastSquares.solve(new KinematicProblem(model).add(FrameTask.atFrame(model, flange, goal, 1e-10, 1e-10))
      .add(swivel), new KinematicState(model, reach), 400);
    check(other.converged(), "the same tool pose solves at another swivel");
    var preferred = other.state.q.copy();
    function distance(q:Array<Float>):Float {
      var sum = 0.0;
      for (i in 0...7) sum += Math.pow(q[i] - preferred[i], 2);
      return Math.sqrt(sum);
    }
    function frameError(state:KinematicState):Float {
      var pose = KinematicSnapshot.of(state).framePose(flange);
      return Math.sqrt(Math.pow(pose.x - goal.x, 2) + Math.pow(pose.y - goal.y, 2) + Math.pow(pose.z - goal.z, 2));
    }
    var plain = DampedLeastSquares.solve(new KinematicProblem(model).add(FrameTask.atFrame(model, flange, goal, 1e-9, 1e-9)),
      seed, 200);
    var problem = new KinematicProblem(model).add(FrameTask.atFrame(model, flange, goal, 1e-9, 1e-9))
      .add(new PostureTask(model, preferred, 0.1));
    var solution = PrioritizedSolver.solve(problem, seed, 400);
    check(solution.converged(), 'the prioritized solve meets the tool target (${solution.status})');
    check(frameError(solution.state) < 1e-9, 'the posture does not pull the tool off (${frameError(solution.state)} m)');
    check(distance(solution.state.q) < 1e-3 && distance(plain.state.q) > 0.1,
      'the arm slides along its self-motion to the preferred posture (${distance(solution.state.q)}; without it ${distance(plain.state.q)})');
    // A posture beyond a limit: the joint stops on the limit, the tool is still exact.
    var limited = new KinematicProblem(model).add(FrameTask.atFrame(model, flange, goal, 1e-9, 1e-9))
      .add(new PostureTask(model, [1.5, 0.6, 0.5, -1.2, 0.3, 0.8, 0.2], 0.1));
    limited.setLimits(0, -2.9, 0.4);
    var bounded = PrioritizedSolver.solve(limited, seed, 400);
    check(bounded.converged() && frameError(bounded.state) < 1e-9, 'limits and the tool target hold together (${bounded.status})');
    check(bounded.state.q[0] <= 0.4 + 1e-12, 'the joint stays inside its limit (${bounded.state.q[0]})');
  }

  /** A cart (root) carrying a planar 3-link arm on vertical hinges, 1.0 + 0.8 + 0.5 m, with a tool frame. */
  static function mobileArm():KinematicModel {
    var builder = new KinematicModelBuilder();
    var cart = builder.addBody("cart");
    var previous = cart;
    var z = new Vector3(0, 0, 1);
    var reach = 0.0;
    for (i in 0...3) {
      var body = builder.addBody('m$i');
      builder.addJoint('m$i', JointKind.Revolute, previous, body, Transform.translation(reach, 0, i == 0 ? 0.5 : 0.0),
        Transform.identity(), z, -2.8, 2.8);
      reach = [1.0, 0.8, 0.5][i];
      previous = body;
    }
    builder.addFrame("tool", previous, Transform.translation(reach, 0, 0));
    return builder.build();
  }

  static function testRootJacobianMatchesFiniteDifferences():Void {
    var model = mobileArm();
    var cart = model.bodyIndex("cart"), tool = model.frameIndex("tool");
    for (mode in [RootMotion.Planar, RootMotion.Floating]) {
      var problem = new KinematicProblem(model).setRootMotion(cart, mode);
      var layout = problem.layout();
      var state = new KinematicState(model, [0.4, -0.7, 0.3]);
      state.setRootPose(cart, new Transform(0.3, -0.2, 0.1, 0, 0, 0, 1).compose(Transform.axisAngle(0.6, 0.0, 0.8, 0.7)));
      var snapshot = KinematicSnapshot.of(state);
      var pose = snapshot.framePose(tool);
      var jacobian = [for (_ in 0...6 * layout.width) 0.0];
      snapshot.pointJacobianColumns(model.frameBody[tool], pose.x, pose.y, pose.z, layout, jacobian);
      var eps = 1e-6, worst = 0.0;
      for (column in layout.dofs.length...layout.width) {
        var plus = state.copy(), minus = state.copy();
        var nudge = [for (c in 0...layout.width) c == column ? eps : 0.0];
        SolverSupport.applyStep(problem, plus, nudge, 1.0);
        SolverSupport.applyStep(problem, minus, nudge, -1.0);
        var a = KinematicSnapshot.of(minus).framePose(tool), b = KinematicSnapshot.of(plus).framePose(tool);
        var w = logMap(a, b);
        var numeric = [(b.x - a.x) / (2 * eps), (b.y - a.y) / (2 * eps), (b.z - a.z) / (2 * eps), w.x / (2 * eps),
          w.y / (2 * eps), w.z / (2 * eps)];
        for (row in 0...6) worst = Math.max(worst, Math.abs(jacobian[row * layout.width + column] - numeric[row]));
      }
      check(worst < 1e-6, 'root columns (${mode == RootMotion.Planar ? "planar" : "floating"}) match central differences ($worst)');
    }
  }

  static function testMobileManipulatorDrivesWhenTheArmCannotReach():Void {
    var model = mobileArm();
    var cart = model.bodyIndex("cart"), tool = model.frameIndex("tool");
    var seed = new KinematicState(model, [0.3, 0.4, -0.2]);
    var far = Transform.translation(5.0, 1.0, 0.5);
    function task() return FrameTask.atFrame(model, tool, far, 1e-7, 1e-7, null, FrameTask.ALL_AXES, FrameOrientation.Free);
    var armOnly = LevenbergMarquardt.solve(new KinematicProblem(model).add(task()), seed);
    check(!armOnly.converged(), "a 2.3 m arm cannot reach 5 m on its own");
    var driving = LevenbergMarquardt.solve(new KinematicProblem(model).setRootMotion(cart, RootMotion.Planar).add(task()), seed);
    var reached = KinematicSnapshot.of(driving.state).framePose(tool);
    check(driving.converged() && Math.abs(reached.x - 5.0) < 1e-6 && Math.abs(reached.y - 1.0) < 1e-6,
      "with a planar base the tool reaches it");
    var base = driving.state.rootPose(cart);
    check(Math.abs(base.z) < 1e-12 && Math.abs(base.qx) < 1e-12 && Math.abs(base.qy) < 1e-12,
      "a planar base stays on the floor and upright");
    check(seed.rootPose(cart).x == 0.0, "the seed's root pose is untouched");

    // Within the arm's reach: damping the base keeps it (nearly) still and lets the arm do the work.
    var near = Transform.translation(1.6, 0.9, 0.5);
    function baseTravel(damping:Float):Float {
      var problem = new KinematicProblem(model).setRootMotion(cart, RootMotion.Planar)
        .add(FrameTask.atFrame(model, tool, near, 1e-7, 1e-7, null, FrameTask.ALL_AXES, FrameOrientation.Free));
      if (damping > 0.0) problem.add(new RootDampingTask(model, cart, damping));
      var solution = DampedLeastSquares.solve(problem, seed, 400, 0.02);
      check(solution.converged(), 'the near target is reached (base damping $damping)');
      var p = solution.state.rootPose(cart);
      return Math.sqrt(p.x * p.x + p.y * p.y);
    }
    var free = baseTravel(0.0), damped = baseTravel(1.0);
    check(damped < 0.1 * free, 'root damping makes the arm do the work (base moved $damped m vs $free m)');
  }

  static function testFloatingBodyIsPlacedExactly():Void {
    // A free box with three corner frames; targets are those corners under an unknown pose.
    var builder = new KinematicModelBuilder();
    var box = builder.addBody("box");
    var corners = [new Vector3(0.2, 0.0, 0.0), new Vector3(0.0, 0.3, 0.0), new Vector3(0.0, 0.0, 0.4)];
    for (i in 0...3) builder.addFrame('corner$i', box, Transform.translation(corners[i].x, corners[i].y, corners[i].z));
    var model = builder.build();
    var truth = new Transform(1.5, -0.7, 2.0, 0, 0, 0, 1).compose(Transform.axisAngle(0.0, 0.6, 0.8, 2.5));
    var problem = new KinematicProblem(model).setRootMotion(box, RootMotion.Floating);
    for (i in 0...3) {
      var target = truth.compose(Transform.translation(corners[i].x, corners[i].y, corners[i].z));
      problem.add(FrameTask.atFrame(model, model.frameIndex('corner$i'), target, 1e-10, 1e-10, null, FrameTask.ALL_AXES,
        FrameOrientation.Free));
    }
    var solution = LevenbergMarquardt.solve(problem, new KinematicState(model, []), 200);
    var placed = solution.state.rootPose(box);
    var dot = Math.abs(placed.qx * truth.qx + placed.qy * truth.qy + placed.qz * truth.qz + placed.qw * truth.qw);
    check(solution.converged() && Math.abs(placed.x - truth.x) < 1e-9 && Math.abs(placed.y - truth.y) < 1e-9 &&
      Math.abs(placed.z - truth.z) < 1e-9 && Math.abs(dot - 1.0) < 1e-12,
      'a floating body is placed exactly from three points under a 2.5 rad rotation (${solution.status})');
    check(solution.rank == 6 && solution.freeDofs == 0, "three non-collinear points fix all six DOFs");
  }

  static function testDampedLeastSquaresChecksItsLastStep():Void {
    var model = planarArm([1.0, 0.7]);
    var tip = model.frameIndex("tip");
    var goal = KinematicSnapshot.of(new KinematicState(model, [0.9, -0.4])).framePose(tip);
    var problem = new KinematicProblem(model).add(FrameTask.atFrame(model, tip, goal, 1e-8, 1e-8));
    var seed = new KinematicState(model, [0.3, 0.2]);
    // How many steps it needs, then exactly that budget: the last step lands inside the tolerance.
    var needed = DampedLeastSquares.solve(problem, seed, 200, 0.02).iterations;
    check(needed > 2, 'fixture needs several steps ($needed)');
    var exact = DampedLeastSquares.solve(problem, seed, needed, 0.02);
    check(exact.status == KinematicStatus.Converged && exact.iterations == needed,
      'a solve whose last step converges reports it (${exact.status} after ${exact.iterations})');
    var short = DampedLeastSquares.solve(problem, seed, needed - 1, 0.02);
    check(short.status == KinematicStatus.IterationLimit, "one step fewer is still the iteration limit");
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
