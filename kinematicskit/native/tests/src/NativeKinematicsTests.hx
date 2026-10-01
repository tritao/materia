import kinematicskit.JacobianLayout;
import kinematicskit.JointKind;
import kinematicskit.KinematicModel;
import kinematicskit.KinematicModelBuilder;
import kinematicskit.KinematicSnapshot;
import kinematicskit.KinematicState;
import kinematicskit.Transform;
import kinematicskit.Vector3;
import kinematicskit.FrameOrientation;
import kinematicskit.FrameTask;
import kinematicskit.KinematicProblem;
import kinematicskit.LinearAlgebra;
import kinematicskit.PostureTask;
import kinematicskit.SolverWorkspace;
import kinematicskit.StepLimits;
import kinematicskit.SwivelTask;
import kinematicskit.RootMotion;
import kinematicskit.RootDampingTask;
import kinematicskit.SolverSupport;
import kinematicskit.native.DifferentialIk;
import kinematicskit.native.NativeKinematics;
import kinematicskit.native.NativeQpStep;

class NativeKinematicsTests {
  static var assertions = 0;

  public static function main():Void {
    testForwardAndJacobianParity();
    testRejectsBadInput();
    testUnboundedQpMatchesDampedStep();
    testBoundedQpMeetsOptimalityConditions();
    testWarmStartHelps();
    testDifferentialIkReachesWithinLimits();
    testRedundantArmFollowsPosture();
    testSwivelPicksTheSevenAxisConfiguration();
    testMinkOracle();
    testMobileBaseTracking();
    Sys.println('KinematicsKit native tests passed ($assertions assertions)');
  }

  static function testForwardAndJacobianParity():Void {
    var worstPose = 0.0, worstJacobian = 0.0;
    for (seed in 1...6) {
      var rng = new Rng(seed);
      var model = randomForest(rng);
      var native = new NativeKinematics(model);
      var snapshot = new KinematicSnapshot(model);
      var n = model.dofCount();
      for (_ in 0...20) {
        var state = new KinematicState(model, [for (_ in 0...n) rng.signed() * 2.0]);
        snapshot.evaluate(state);
        var poses = native.forward(state.q);
        for (body in 0...model.bodyCount()) {
          var expected = snapshot.bodyPose(body);
          var values = [expected.x, expected.y, expected.z, expected.qx, expected.qy, expected.qz, expected.qw];
          for (k in 0...7) worstPose = Math.max(worstPose, Math.abs(poses[body * 7 + k] - values[k]));
        }
        // A subset of columns, in a shuffled order, at a point off each body.
        var columns = [for (dof in 0...n) if (rng.signed() > -0.4) dof];
        columns.reverse();
        if (columns.length == 0) columns = [0];
        var layout = new JacobianLayout(model, columns);
        var expectedJacobian = [for (_ in 0...6 * columns.length) 0.0];
        for (body in 0...model.bodyCount()) {
          var point = [rng.signed(), rng.signed(), rng.signed()];
          snapshot.pointJacobianColumns(body, point[0], point[1], point[2], layout, expectedJacobian);
          var got = native.pointJacobian(state.q, body, point, columns);
          for (i in 0...expectedJacobian.length) worstJacobian = Math.max(worstJacobian, Math.abs(got[i] - expectedJacobian[i]));
        }
      }
      native.dispose();
    }
    check(worstPose == 0.0, 'native forward kinematics is bit-identical to the Haxe snapshot (worst $worstPose)');
    check(worstJacobian == 0.0, 'native Jacobians are bit-identical to the Haxe snapshot (worst $worstJacobian)');
  }

  static function testRejectsBadInput():Void {
    var model = randomForest(new Rng(9));
    var native = new NativeKinematics(model);
    rejects(() -> native.forward([0.0]), "a wrong DOF count is rejected");
    rejects(() -> native.forward([for (_ in 0...model.dofCount()) Math.NaN]), "non-finite joint values are rejected");
    rejects(() -> native.pointJacobian([for (_ in 0...model.dofCount()) 0.0], model.bodyCount(), [0.0, 0.0, 0.0], [0]),
      "an out-of-range body is rejected");
    rejects(() -> native.pointJacobian([for (_ in 0...model.dofCount()) 0.0], 0, [0.0, 0.0, 0.0], [0, 0]),
      "a repeated column is rejected");
    native.dispose();
    rejects(() -> native.forward([for (_ in 0...model.dofCount()) 0.0]), "a disposed model is rejected");
    var ints = NativeKinematics.packInts(model), reals = NativeKinematics.packReals(model);
    ints[0] = 2;
    check(KinematicsKitNative.kk_model_create(ints, reals).status == KinematicsKitNativeConstants.KK_ERROR_INVALID_ARGUMENT,
      "an unknown packed format is rejected");
  }

  static function randomSystem(rng:Rng, rows:Int, width:Int):{jacobian:Array<Float>, residual:Array<Float>} {
    return {jacobian: [for (_ in 0...rows * width) rng.signed()], residual: [for (_ in 0...rows) rng.signed() * 2.0]};
  }

  static function testUnboundedQpMatchesDampedStep():Void {
    var rng = new Rng(21);
    var qp = new NativeQpStep(7);
    var worst = 0.0;
    for (_ in 0...20) {
      var system = randomSystem(rng, 6, 7);
      var free = [for (_ in 0...7) Math.POSITIVE_INFINITY];
      var result = qp.solve(system.jacobian, system.residual, [for (v in free) -v], free, 0.05);
      check(result.solved(), "an unbounded QP step solves");
      var expected = LinearAlgebra.dampedStep(system.jacobian, 6, 7, [for (i in 0...7) i], system.residual, 0.05);
      for (i in 0...7) worst = Math.max(worst, Math.abs(result.step[i] - expected[i]));
    }
    qp.dispose();
    // The solver's 1e-9 tolerance bounds its optimality residual; the step itself can be off by up to
    // about 1/λ² (400 here) times that.
    check(worst < 1e-6, 'with no bound active the QP is the damped least-squares step (worst $worst)');
  }

  static function testBoundedQpMeetsOptimalityConditions():Void {
    var rng = new Rng(22);
    var qp = new NativeQpStep(7);
    var damping = 0.05;
    var boundsHit = 0;
    for (_ in 0...20) {
      var system = randomSystem(rng, 6, 7);
      var lower = [for (_ in 0...7) -0.2 - 0.3 * (rng.signed() + 1.0)];
      var upper = [for (_ in 0...7) 0.2 + 0.3 * (rng.signed() + 1.0)];
      var result = qp.solve(system.jacobian, system.residual, lower, upper, damping);
      check(result.solved(), "a bounded QP step solves");
      var x = result.step;
      // Gradient of ½xᵀ(JᵀJ + λ²I)x − (Jᵀe)ᵀx.
      for (i in 0...7) {
        var gradient = damping * damping * x[i];
        for (k in 0...6) {
          var jx = 0.0;
          for (j in 0...7) jx += system.jacobian[k * 7 + j] * x[j];
          gradient += system.jacobian[k * 7 + i] * (jx - system.residual[k]);
        }
        check(x[i] >= lower[i] - 1e-9 && x[i] <= upper[i] + 1e-9, "the step respects its bounds");
        // ProxQP meets complementarity to its tolerance, so an active variable can sit ~1e-6 inside its bound.
        if (x[i] <= lower[i] + 1e-5) { boundsHit++; check(gradient >= -1e-6, 'a variable at its lower bound would not improve by rising ($gradient)'); }
        else if (x[i] >= upper[i] - 1e-5) { boundsHit++; check(gradient <= 1e-6, 'a variable at its upper bound would not improve by falling ($gradient)'); }
        else check(Math.abs(gradient) <= 1e-6, 'a free variable is stationary (gradient $gradient, x ${x[i]} in [${lower[i]}, ${upper[i]}], iterations ${result.iterations})');
      }
    }
    qp.dispose();
    check(boundsHit > 10, 'the fixture actually exercises active bounds ($boundsHit)');
  }

  static function testWarmStartHelps():Void {
    var rng = new Rng(23);
    var system = randomSystem(rng, 6, 7);
    var lower = [for (_ in 0...7) -0.3], upper = [for (_ in 0...7) 0.3];
    var qp = new NativeQpStep(7);
    var first = qp.solve(system.jacobian, system.residual, lower, upper, 0.05);
    var again = qp.solve(system.jacobian, system.residual, lower, upper, 0.05);
    qp.dispose();
    check(first.solved() && again.solved() && again.iterations <= first.iterations,
      'a repeated solve warm-starts (${first.iterations} then ${again.iterations} iterations)');
  }

  /** A 7-axis arm with alternating Z/Y axes (the layout of common collaborative arms), limits ±2.9 rad. */
  static function sevenAxisArm():KinematicModel {
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

  /** Integrates differential IK steps; returns the final state and whether every step stayed legal. */
  static function track(problem:KinematicProblem, start:Array<Float>, velocityLimits:Array<Float>, steps:Int):{q:Array<Float>, legal:Bool, why:String} {
    var model = problem.model;
    var state = new KinematicState(model, start);
    var qp = new NativeQpStep(problem.layout().width);
    var workspace = new SolverWorkspace();
    var dt = 0.01, legal = true, why = "";
    for (_ in 0...steps) {
      var step = DifferentialIk.step(problem, state, dt, qp, StepLimits.ofVelocity(velocityLimits), 0.5, 1e-3, workspace);
      if (!(step.status == KinematicsKitNativeConstants.KK_QP_SOLVED)) { legal = false; why = 'status ${step.status}'; }
      for (column in 0...problem.layout().width) {
        var dof = problem.layout().dofs[column];
        if (Math.abs(step.velocity[column]) > velocityLimits[column] + 1e-9) { legal = false; why = 'velocity ${step.velocity[column]}'; }
        state.q[dof] += step.velocity[column] * dt;
        if (state.q[dof] < problem.lower[dof] - 1e-12 || state.q[dof] > problem.upper[dof] + 1e-12) { legal = false; why = 'q ${state.q[dof]}'; }
      }
    }
    qp.dispose();
    return {q: state.q, legal: legal, why: why};
  }

  static function testDifferentialIkReachesWithinLimits():Void {
    var model = sevenAxisArm();
    var flange = model.frameIndex("flange");
    var goal = KinematicSnapshot.of(new KinematicState(model, [0.4, 0.7, -0.3, -1.1, 0.5, 0.9, 0.2])).framePose(flange);
    var task = FrameTask.atFrame(model, flange, goal, 1e-6, 1e-6);
    var problem = new KinematicProblem(model).add(task).add(new PostureTask(model, [for (_ in 0...7) 0.0], 1e-3));
    var speeds = [for (_ in 0...7) 1.5];
    var run = track(problem, [0.1, 0.3, 0.0, -0.6, 0.0, 0.4, 0.0], speeds, 600);
    var reached = KinematicSnapshot.of(new KinematicState(model, run.q)).framePose(flange);
    check(run.legal, 'every step solves, respects the velocity limits and stays inside the joint range (${run.why})');
    check(Math.abs(reached.x - goal.x) < 1e-5 && Math.abs(reached.y - goal.y) < 1e-5 && Math.abs(reached.z - goal.z) < 1e-5,
      "differential IK converges to the target");

    // A target only reachable past a joint's limit: the joint stops exactly on it.
    var tight = new KinematicProblem(model).add(FrameTask.atFrame(model, flange, goal, 1e-6, 1e-6));
    tight.setLimits(3, -0.8, 0.8);
    var held = track(tight, [0.1, 0.3, 0.0, -0.6, 0.0, 0.4, 0.0], speeds, 600);
    check(held.legal && Math.abs(held.q[3] - (-0.8)) < 1e-9, 'a limit in the way holds exactly (${held.q[3]})');

    // With limit gain 0.5 the joint covers at most half its remaining distance per step: it slows into the
    // stop and never quite touches it.
    var state = new KinematicState(model, [0.1, 0.3, 0.0, -0.6, 0.0, 0.4, 0.0]);
    var qp = new NativeQpStep(7);
    var gaps:Array<Float> = [];
    for (_ in 0...60) {
      var step = DifferentialIk.step(tight, state, 0.01, qp, StepLimits.ofVelocity(speeds, 0.5), 0.5, 1e-3);
      for (column in 0...7) state.q[tight.layout().dofs[column]] += step.velocity[column] * 0.01;
      gaps.push(state.q[3] - (-0.8));
    }
    qp.dispose();
    var halving = true;
    for (i in 1...gaps.length) if (gaps[i] < 0.5 * gaps[i - 1] - 1e-12 || gaps[i] <= 0.0) halving = false;
    check(halving && gaps[gaps.length - 1] < 1e-3, 'a limit gain of 0.5 approaches the stop gradually (last gap ${gaps[gaps.length - 1]})');
  }

  /**
   * A planar 3-link arm with a position-only target has one redundant DOF. A posture preference on the
   * base joint alone slides along that self-motion, so the target is still met exactly.
   * (A 7-axis arm's swivel is a poor fixture for this: at generic poses its self-motion moves the base
   * joints ~9x more than any single "elbow" joint, so a one-joint preference crawls; a swivel-angle task
   * is the right tool there.)
   */
  static function testRedundantArmFollowsPosture():Void {
    var builder = new KinematicModelBuilder();
    var previous = builder.addBody("base");
    var reach = 0.0;
    var z = new Vector3(0, 0, 1);
    for (i in 0...3) {
      var body = builder.addBody('p$i');
      builder.addJoint('p$i', JointKind.Revolute, previous, body, Transform.translation(reach, 0, 0), Transform.identity(), z,
        -3.0, 3.0);
      reach = [1.0, 0.8, 0.5][i];
      previous = body;
    }
    var tip = builder.addFrame("tip", previous, Transform.translation(reach, 0, 0));
    var model = builder.build();
    var target = Transform.translation(1.0, 1.0, 0.0);
    function reachPreferring(base:Float):Array<Float> {
      var problem = new KinematicProblem(model)
        .add(FrameTask.atFrame(model, tip, target, 1e-9, 1e-9, null, FrameTask.AXIS_X | FrameTask.AXIS_Y,
          FrameOrientation.Free))
        .add(new PostureTask(model, [base, 0.0, 0.0], 0.05, [1, 0, 0]));
      var run = track(problem, [0.6, 0.4, 0.3], [for (_ in 0...3) Math.POSITIVE_INFINITY], 400);
      check(run.legal, 'redundant tracking stays legal (${run.why})');
      var reached = KinematicSnapshot.of(new KinematicState(model, run.q)).framePose(tip);
      check(Math.abs(reached.x - 1.0) < 1e-6 && Math.abs(reached.y - 1.0) < 1e-6,
        'the target is reached whatever the preference (${reached.x}, ${reached.y})');
      return run.q;
    }
    var high = reachPreferring(0.9), low = reachPreferring(0.3);
    check(Math.abs(high[0] - 0.9) < 1e-4 && Math.abs(low[0] - 0.3) < 1e-4,
      'the preference picks the base angle along the self-motion (${high[0]} vs ${low[0]})');
  }

  static function testSwivelPicksTheSevenAxisConfiguration():Void {
    var model = sevenAxisArm();
    var flange = model.frameIndex("flange");
    var start = [0.3, 0.6, 0.5, -1.2, 0.3, 0.8, 0.2];
    var goal = KinematicSnapshot.of(new KinematicState(model, start)).framePose(flange);
    var origin = new Vector3(0, 0, 0);
    function reachAt(swivel:Float):Array<Float> {
      var task = new SwivelTask(model, model.bodyIndex("link1"), origin, model.bodyIndex("link3"), origin,
        model.bodyIndex("link5"), origin, swivel, 1e-9, new Vector3(1, 0, 0));
      var problem = new KinematicProblem(model).add(FrameTask.atFrame(model, flange, goal, 1e-9, 1e-9)).add(task);
      var run = track(problem, start, [for (_ in 0...7) Math.POSITIVE_INFINITY], 300);
      var snapshot = KinematicSnapshot.of(new KinematicState(model, run.q));
      var reached = snapshot.framePose(flange);
      check(run.legal, 'swivel tracking stays legal (${run.why})');
      check(Math.abs(reached.x - goal.x) < 1e-6 && Math.abs(reached.y - goal.y) < 1e-6 && Math.abs(reached.z - goal.z) < 1e-6 &&
        Math.abs(Math.abs(reached.qw * goal.qw + reached.qx * goal.qx + reached.qy * goal.qy + reached.qz * goal.qz) - 1.0) < 1e-9,
        'the 6D tool target is met at swivel $swivel');
      check(Math.abs(task.angle(snapshot) - swivel) < 1e-6, 'the arm swivels to $swivel (${task.angle(snapshot)})');
      return run.q;
    }
    var one = reachAt(0.4), other = reachAt(-0.6);
    var differs = 0.0;
    for (i in 0...7) differs = Math.max(differs, Math.abs(one[i] - other[i]));
    check(differs > 0.3, 'different swivels are different arm configurations for the same tool pose ($differs)');
  }

  /**
   * The same differential-IK step solved by mink (MuJoCo kinematics, DAQP) and by us (our kinematics,
   * ProxQP). Needs `KK_MINK_PYTHON`: a Python with `mink` and `mujoco` (see mink_oracle.py); skipped otherwise.
   */
  static function testMinkOracle():Void {
    var python = Sys.getEnv("KK_MINK_PYTHON");
    if (python == null || python == "" || !sys.FileSystem.exists(python)) {
      Sys.println("mink oracle skipped: set KK_MINK_PYTHON to a Python with mink and mujoco");
      return;
    }
    // The script lives next to this suite; the working directory is wherever `haxeon run` started.
    var script = [for (candidate in ["mink_oracle.py", "tests/mink_oracle.py", "native/tests/mink_oracle.py",
      "kinematicskit/native/tests/mink_oracle.py"]) if (sys.FileSystem.exists(candidate)) candidate][0];
    if (script == null) throw "mink oracle script not found from the working directory";
    var dir = haxe.io.Path.directory(script);
    dir = (dir == "" ? "." : dir) + "/build/oracle";
    var model = sevenAxisArm();
    var flange = model.frameIndex("flange");
    var q = [0.3, 0.6, 0.5, -1.2, 0.3, 0.8, 0.2];
    var here = KinematicSnapshot.of(new KinematicState(model, q)).framePose(flange);
    // mink's frame error is the SE(3) logarithm, whose translational part couples with any rotation error,
    // so the position-only cases keep the current orientation (there both formulations coincide).
    var shifted = new Transform(here.x + 0.04, here.y - 0.03, here.z + 0.02, here.qx, here.qy, here.qz, here.qw);
    var turned = shifted.compose(Transform.axisAngle(0.6, 0.0, 0.8, 0.05));
    var nudged = shifted.compose(Transform.axisAngle(0.6, 0.0, 0.8, 0.005));
    var dt = 0.01, lambda = 1e-3, gain = 0.8;
    var cases:Array<{name:String, target:Transform, orientation:Bool, velocity:Null<Array<Float>>, tolerance:Float}> = [
      {name: "position target", target: shifted, orientation: false, velocity: null, tolerance: 1e-6},
      {name: "position target, speed-limited", target: shifted, orientation: false, velocity: [for (_ in 0...7) 0.5],
        tolerance: 1e-6},
      // With a rotation error, our world-frame position rows and first-order orientation Jacobian differ from
      // mink's SE(3) logarithm and its exact Jacobian: to first order in the 0.05 rad rotation times the offset.
      {name: "pose target, 0.05 rad", target: turned, orientation: true, velocity: null, tolerance: 5e-2},
      {name: "pose target, 0.005 rad", target: nudged, orientation: true, velocity: null, tolerance: 5e-3}
    ];
    var poseDifferences:Array<Float> = [];
    for (c in cases) {
      var nearby = c.target;
      var task = FrameTask.atFrame(model, flange, nearby, 1e-9, 1e-9, null, FrameTask.ALL_AXES,
        c.orientation ? FrameOrientation.Full : FrameOrientation.Free);
      var posture = [0.0, 0.5, 0.0, -1.0, 0.0, 0.5, 0.0];
      var problem = new KinematicProblem(model).add(task).add(new PostureTask(model, posture, 0.05));
      var qp = new NativeQpStep(7);
      var ours = DifferentialIk.step(problem, new KinematicState(model, q), dt, qp, StepLimits.ofVelocity(c.velocity), gain, lambda).velocity;
      qp.dispose();
      var request = {mjcf: Mjcf.write(model), q: q, dt: dt, damping: lambda * lambda, site: "flange",
        target_position: [nearby.x, nearby.y, nearby.z], target_wxyz: [nearby.qw, nearby.qx, nearby.qy, nearby.qz],
        position_cost: 1.0, orientation_cost: c.orientation ? 1.0 : 0.0, gain: gain, posture_cost: 0.05,
        posture_target: posture, velocity_limits: c.velocity, joint_names: [for (i in 0...7) 'a$i']};
      sys.FileSystem.createDirectory(dir);
      sys.io.File.saveContent('$dir/request.json', haxe.Json.stringify(request));
      var exit = Sys.command(python, [script, '$dir/request.json', '$dir/answer.json']);
      check(exit == 0, 'the mink oracle ran for the ${c.name}');
      var answer:Dynamic = haxe.Json.parse(sys.io.File.getContent('$dir/answer.json'));
      var theirs:Array<Float> = Reflect.field(answer, "velocity");
      var worst = 0.0, size = 0.0;
      for (i in 0...7) { worst = Math.max(worst, Math.abs(ours[i] - theirs[i])); size = Math.max(size, Math.abs(theirs[i])); }
      Sys.println('mink oracle, ${c.name}: worst difference $worst rad/s (largest velocity $size)');
      check(worst <= c.tolerance * Math.max(1.0, size), 'our step agrees with mink for the ${c.name}');
      if (c.orientation) poseDifferences.push(worst / size);
    }
    // The pose-target difference is the SE(3)-log coupling, first order in the rotation error: 10x less
    // rotation, about 10x less difference.
    var ratio = poseDifferences[0] / poseDifferences[1];
    check(ratio > 7.0 && ratio < 13.0, 'the pose difference scales with the rotation error (ratio $ratio)');
  }

  /** A cart driving (planar base, speed-limited) while its arm tracks a target 4 m away. */
  static function testMobileBaseTracking():Void {
    var builder = new KinematicModelBuilder();
    var cart = builder.addBody("cart");
    var previous = cart, reach = 0.0;
    var z = new Vector3(0, 0, 1);
    for (i in 0...3) {
      var body = builder.addBody('m$i');
      builder.addJoint('m$i', JointKind.Revolute, previous, body, Transform.translation(reach, 0, i == 0 ? 0.5 : 0.0),
        Transform.identity(), z, -2.8, 2.8);
      reach = [1.0, 0.8, 0.5][i];
      previous = body;
    }
    var tool = builder.addFrame("tool", previous, Transform.translation(reach, 0, 0));
    var model = builder.build();
    var target = Transform.translation(4.0, 1.5, 0.5);
    var problem = new KinematicProblem(model).setRootMotion(cart, RootMotion.Planar)
      .add(FrameTask.atFrame(model, tool, target, 1e-6, 1e-6, null, FrameTask.ALL_AXES, FrameOrientation.Free))
      .add(new RootDampingTask(model, cart, 0.3));
    var width = problem.layout().width;
    check(width == 6, 'three arm joints plus a planar base give six columns ($width)');
    // Arm joints up to 1.5 rad/s; the base up to 0.5 m/s and 0.5 rad/s.
    var limits = [1.5, 1.5, 1.5, 0.5, 0.5, 0.5];
    var state = new KinematicState(model, [0.3, 0.4, -0.2]);
    var qp = new NativeQpStep(width);
    var dt = 0.02, legal = true;
    for (_ in 0...1000) {
      var step = DifferentialIk.step(problem, state, dt, qp, StepLimits.ofVelocity(limits), 0.5, 1e-3);
      for (c in 0...width) if (Math.abs(step.velocity[c]) > limits[c] + 1e-9) legal = false;
      SolverSupport.applyStep(problem, state, step.velocity, dt);
      for (dof in 0...3) if (state.q[dof] < -2.8 - 1e-12 || state.q[dof] > 2.8 + 1e-12) legal = false;
    }
    qp.dispose();
    var reached = KinematicSnapshot.of(state).framePose(tool);
    check(legal, "base and joint velocities stay within their limits, joints within their range");
    check(Math.abs(reached.x - 4.0) < 1e-5 && Math.abs(reached.y - 1.5) < 1e-5,
      'the base drives until the arm reaches the target (${reached.x}, ${reached.y})');
    var base = state.rootPose(cart);
    check(Math.sqrt(base.x * base.x + base.y * base.y) > 1.2, "the base actually drove");
  }

  /** Two roots; a chain with revolute, prismatic, fixed and coupled joints; a side branch. */
  static function randomForest(rng:Rng):KinematicModel {
    var builder = new KinematicModelBuilder();
    var bodies = [builder.addBody("root")];
    var kinds = [JointKind.Revolute, JointKind.Prismatic, JointKind.Fixed, JointKind.Revolute, JointKind.Revolute,
      JointKind.Prismatic, JointKind.Revolute];
    for (i in 0...kinds.length) {
      var body = builder.addBody('b$i');
      var parent = i == 5 ? bodies[2] : bodies[bodies.length - 1];
      builder.addJoint('j$i', kinds[i], parent, body, randomTransform(rng), randomTransform(rng),
        new Vector3(rng.signed(), rng.signed(), rng.signed() + 0.1));
      bodies.push(body);
    }
    builder.couple("j4", "j3", 0.75, 0.1);
    var other = builder.addBody("other_root");
    builder.setRootPose(other, randomTransform(rng));
    var arm = builder.addBody("other_arm");
    builder.addJoint("k0", JointKind.Revolute, other, arm, randomTransform(rng), randomTransform(rng), new Vector3(0, 0, 1));
    builder.addFrame("tip", bodies[bodies.length - 1], randomTransform(rng));
    return builder.build();
  }

  static function randomTransform(rng:Rng):Transform {
    var ax = rng.signed(), ay = rng.signed(), az = rng.signed() + 0.1;
    var norm = Math.sqrt(ax * ax + ay * ay + az * az);
    return new Transform(rng.signed(), rng.signed(), rng.signed(), 0.0, 0.0, 0.0, 1.0)
      .compose(Transform.axisAngle(ax / norm, ay / norm, az / norm, rng.signed() * 2.0));
  }

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

private class Rng {
  var state:Int;
  public function new(seed:Int) state = seed;
  public function signed():Float {
    state = (state * 1103515245 + 12345) & 0x7fffffff;
    return state / 2147483647.0 * 2.0 - 1.0;
  }
}
