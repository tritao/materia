import haxe.Int64;
import kinematicskit.LinearAlgebra;
import robotkit.manipulation.IkOptions;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.PathRequest;
import motionkit.kinematics.Pose3;
import motionkit.kinematics.Twist6;
import motionkit.path.OrientationPolicy;
import motionkit.path.PoseLine;
import motionkit.path.PosePath;
import motionkit.path.PoseMath;
import motionkit.path.PoseWaypoint;
import motionkit.program.Blend;
import motionkit.program.MotionOp;
import motionkit.program.MotionProgram;
import motionkit.robot.ManipulatorKinematics;
import motionkit.robot.ProgramCompiler;
import motionkit.robot.StartTolerances;
import motionkit.robot.ToolFreedom;
import motionkit.trajectory.ValidationLimits;
import robotkit.manipulation.KinematicGroup;
import robotkit.model.Frame;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;

/** Tool freedom reaches pose IK, differential IK, path search and checked plans. */
class ToolFreedomTests extends MotionKitTestSupport {
  public function new() { super(); }

  public function testFreeSpinPath():Void {
    var solver = cartesian(true);
    var seed = [0.0, 0.0, 0.0, 0.0];
    var target = new Pose3(0.2, 0.1, 0.1, 0, 0, Math.sin(0.5), Math.cos(0.5));
    var tolerance = new IkTolerance(1e-5, 1e-5, 200, 0.01);
    check(solver.solvePose(target, seed, tolerance, OrientationPolicy.Interpolated) == null,
      "XYZ+C cannot follow spin beyond its C travel");
    var free = solver.solvePose(target, seed, tolerance, OrientationPolicy.FreeAboutTool);
    check(free != null, "The same XYZ+C target is reachable with free spin");
    near(free[3], 0, "Free spin does not consume C travel", 1e-12);
    near(ToolFreedom.orientationError(solver.forward(free), target, OrientationPolicy.FreeAboutTool), 0,
      "The constrained tool axis has zero orientation error", 1e-12);
    var twist = new Twist6(1, 0, 0, 0, 0, 8);
    var rate = solver.solveDifferential(seed, twist, null, OrientationPolicy.FreeAboutTool);
    check(rate != null, "Differential IK drops the free spin row");
    near(rate[0], 1, "Differential IK retains the position row", 1e-8);
    near(rate[3], 0, "Differential IK leaves unneeded C still", 1e-12);
    solver.preferredOrientation = target;
    var preferred = solver.solvePose(new Pose3(target.x, target.y, target.z), seed, tolerance,
      OrientationPolicy.FreeAboutTool);
    check(preferred != null, "An unreachable full orientation preference falls back to hard tool-axis rows");
    near(preferred[3], 0.2, "Soft orientation preference reaches the closest permitted C orientation", 1e-5);
    var forked:ManipulatorKinematics = cast solver.fork();
    check(forked.preferredOrientation == solver.preferredOrientation, "Worker retains the immutable orientation preference");
    solver.preferredOrientation = null;
    solver.preferTargetOrientation = true;
    var authored = solver.solvePose(target, seed, tolerance, OrientationPolicy.FreeAboutTool);
    check(authored != null, "A target orientation preference leaves unreachable spin free");
    near(authored[3], 0.2, "An explicit target preference selects the closest permitted spin", 1e-5);
    var targetFork:ManipulatorKinematics = cast solver.fork();
    check(targetFork.preferTargetOrientation, "Worker retains the target orientation preference");
    solver.preferTargetOrientation = false;
    var compiler = compilerFor(solver);
    var path = new PosePath("work", [new PoseLine(new PoseWaypoint(solver.forward(seed), 1e-4, 1e-4),
      new PoseWaypoint(target, 1e-4, 1e-4), OrientationPolicy.FreeAboutTool, 0.1, 0.1)]);
    var compiled = compiler.compile(new MotionProgram([MotionOp.FollowPath(path, "work", 0.1, [])]), seed,
      Int64.ofInt(20));
    var plan = compiled.blocks[0].plans[0];
    for (i in 0...101) {
      var q = plan.evaluate(plan.durationSeconds*i/100).positions;
      check(Math.abs(q[3]) < 1e-10, "Compiled free-spin path holds C within its travel");
      near(ToolFreedom.orientationError(solver.forward(q), target, OrientationPolicy.FreeAboutTool), 0,
        "Compiled path preserves the tool axis", 1e-12);
    }
    compiled.dispose();
    var line = compiler.compile(new MotionProgram([MotionOp.MoveL(target, "work", 0.1, Blend.ExactStop,
      OrientationPolicy.FreeAboutTool)]), seed, Int64.ofInt(21));
    check(line.blocks[0].plans.length == 1, "MoveL carries its explicit freedom into planning");
    line.dispose();
    var middle = new PoseWaypoint(new Pose3(0.1, 0.05, 0.05, 0, 0, Math.sin(0.2), Math.cos(0.2)),
      1e-4, 1e-4);
    var segmented = new PosePath("work", [
      new PoseLine(new PoseWaypoint(solver.forward(seed), 1e-4, 1e-4), middle,
        OrientationPolicy.FreeAboutTool, 0.1, 0.1),
      new PoseLine(middle, new PoseWaypoint(target, 1e-4, 1e-4),
        OrientationPolicy.FreeAboutTool, 0.1, 0.1)
    ]);
    var joined = compiler.compile(new MotionProgram([MotionOp.FollowPath(segmented, "work", 0.1, [])]), seed,
      Int64.ofInt(22));
    check(joined.blocks[0].plans.length == 1, "Continuous path tolerates a change in the free spin rate at its junction");
    joined.dispose();
    var fullMiddle = new PoseWaypoint(new Pose3(0.1, 0.05, 0.05, 0, 0, Math.sin(0.01), Math.cos(0.01)),
      1e-4, 1e-4);
    var fullEnd = new PoseWaypoint(new Pose3(0.2, 0.1, 0.1, 0, 0, Math.sin(0.05), Math.cos(0.05)),
      1e-4, 1e-4);
    var hardSpin = new PosePath("work", [
      new PoseLine(new PoseWaypoint(solver.forward(seed), 1e-4, 1e-4), fullMiddle,
        OrientationPolicy.Interpolated, 0.1, 0.1),
      new PoseLine(fullMiddle, fullEnd, OrientationPolicy.Interpolated, 0.1, 0.1)
    ]);
    var stopped = compiler.compile(new MotionProgram([MotionOp.FollowPath(hardSpin, "work", 0.1, [])]), seed,
      Int64.ofInt(23));
    check(stopped.blocks[0].plans.length == 2, "Hard spin rate discontinuity still creates an exact stop at its junction");
    stopped.dispose();
  }

  public function testUnreachableTilt():Void {
    var solver = cartesian(false);
    var seed = [0.0, 0.0, 0.0];
    var target = new Pose3(0.2, 0.1, 0.1, Math.sin(0.2), 0, 0, Math.cos(0.2));
    var compiler = compilerFor(solver);
    check(solver.solvePose(target, seed, new IkTolerance(), OrientationPolicy.Fixed) == null,
      "XYZ rejects a tilted fixed-orientation target");
    var diagnostic = "";
    try compiler.compile(new MotionProgram([MotionOp.MoveL(target, "work", 0.1, Blend.ExactStop,
      OrientationPolicy.Fixed)]), seed, Int64.ofInt(30))
    catch (error:Dynamic) diagnostic = Std.string(error);
    check(diagnostic.indexOf("orientation residual") >= 0,
      "A fixed-orientation MoveL cannot silently discard a tilted endpoint");
    var path = new PosePath("work", [new PoseLine(new PoseWaypoint(target, 1e-4, 1e-4),
      new PoseWaypoint(new Pose3(0.3, 0.1, 0.1, target.qx, target.qy, target.qz, target.qw), 1e-4, 1e-4),
      OrientationPolicy.Fixed, 0.1, 0.1)]);
    diagnostic = "";
    try compiler.compile(new MotionProgram([MotionOp.FollowPath(path, "work", 0.1, [])]), seed, Int64.ofInt(31))
    catch (error:Dynamic) diagnostic = Std.string(error);
    check(diagnostic.indexOf("orientation residual") >= 0 && diagnostic.indexOf("position residual") >= 0,
      'Unreachable fixed tilt names both hard residuals: $diagnostic');
    check(solver.solveDifferential(seed, new Twist6(1, 0, 0, 0.4, 0, 0), null,
      OrientationPolicy.Fixed) == null, "XYZ rejects an impossible hard angular rate");
    check(solver.lastFailure.indexOf("orientation velocity residual") >= 0,
      "Impossible differential IK names its angular residual");
    check(solver.solvePose(target, seed, new IkTolerance(), OrientationPolicy.Free) != null,
      "Position-only XYZ ignores the same tilt");
    check(solver.solvePose(target, seed, new IkTolerance(), OrientationPolicy.FreeAboutTool) == null,
      "Free spin retains tilt as a hard constraint");
  }

  public function testConeAndRedundancy():Void {
    var actual = new Pose3(0, 0, 0);
    var spun = new Pose3(0, 0, 0, Math.sin(0.5), 0, 0, Math.cos(0.5));
    near(ToolFreedom.orientationError(actual, spun, OrientationPolicy.Cone([0, 0, 2], 0.2)), 0,
      "Cone axis is in the path frame, independently of waypoint orientation", 1e-12);
    var cone = ToolFreedom.of(actual, OrientationPolicy.Cone([1, 0, 0], 0.3), 0.01);
    var axis = ToolFreedom.toolAxis(cone.target);
    near(axis[0], 1, "Cone IK target aligns local tool Z with its path-frame axis", 1e-12);
    near(cone.orientationTolerance, 0.3, "Cone aperture becomes the IK orientation tolerance", 1e-12);
    var solver = cartesian(true, true);
    var start = [0.0, 0.0, 0.0, 0.0, 0.0];
    var poses:Array<Pose3> = [], distances:Array<Float> = [], policies:Array<OrientationPolicy> = [];
    for (i in 0...11) {
      var t = i/10;
      poses.push(new Pose3(0.2*t, 0.1*t, 0, 0, 0, Math.sin(t*0.5), Math.cos(t*0.5)));
      distances.push(0.224*t);
      policies.push(OrientationPolicy.FreeAboutTool);
    }
    var request = new PathRequest(distances, poses, start, new IkTolerance(),
      [for (_ in start) 0.5], [for (_ in start) 1.0], 8, policies);
    solver.preferredOrientation = poses[poses.length - 1];
    var result = solver.solvePathWithRates(request);
    check(result.configurations.length == poses.length, "External-axis path search preserves aligned freedom samples");
    for (i in 0...poses.length) {
      var q = result.configurations[i];
      check(q != null, "External-axis lattice and refinement solve the reduced tool task");
      near(ToolFreedom.orientationError(solver.forward(q), poses[i], policies[i]), 0,
        "External-axis route leaves impossible spin free", 1e-12);
    }
    var last = result.configurations[result.configurations.length - 1];
    near(last[4], 0.2, "External-axis lattice and refinement retain the soft orientation preference", 1e-5);
  }

  public function testFullOrientationIdentity():Void {
    var fixture = buildContractArmFixture();
    var solver = new ManipulatorKinematics(fixture.arm);
    var seed = [0.2, -0.3, 0.4, 0.1, 0.2, -0.1];
    var target = solver.forward([0.25, -0.35, 0.45, 0.15, 0.25, -0.15]);
    var tolerance = new IkTolerance();
    var original = solver.solvePose(target, seed, tolerance, null);
    var legacy = fixture.arm.solve(new Transform3(new Vec3(target.x, target.y, target.z),
      new Quat(target.qx, target.qy, target.qz, target.qw)), seed,
      new IkOptions(tolerance.position, tolerance.orientation, tolerance.maxIterations, tolerance.damping));
    check(legacy.converged, "The G0 full-task solve reaches the comparison target");
    for (i in 0...6) check(original[i] == legacy.q[i], "Full pose IK retains G0 solver bits");
    var fixed = solver.solvePose(target, seed, tolerance, OrientationPolicy.Fixed);
    var interpolated = solver.solvePose(target, seed, tolerance, OrientationPolicy.Interpolated);
    check(original != null && fixed != null && interpolated != null, "Full 6R variants solve the same target");
    for (i in 0...6) check(original[i] == fixed[i] && original[i] == interpolated[i],
      "Fixed and Interpolated preserve the original full IK bits");
    var twist = new Twist6(0.02, -0.01, 0.03, 0.01, 0.02, -0.03);
    var rates = solver.solveDifferential(seed, twist, null, null);
    var legacyRates = LinearAlgebra.dampedStep(fixture.arm.tcpJacobian(seed), 6, 6,
      [0, 1, 2, 3, 4, 5], twist.toArray(), solver.differentialDamping);
    for (i in 0...6) check(rates[i] == legacyRates[i], "Full differential IK retains G0 solver bits");
    var fixedRates = solver.solveDifferential(seed, twist, null, OrientationPolicy.Fixed);
    var interpolatedRates = solver.solveDifferential(seed, twist, null, OrientationPolicy.Interpolated);
    for (i in 0...6) check(rates[i] == fixedRates[i] && rates[i] == interpolatedRates[i],
      "Fixed and Interpolated preserve the original full differential IK bits");
    var redundant = buildSevenAxisArmFixture();
    var seven = new ManipulatorKinematics(redundant.arm);
    var sevenSeed = [0.2, -0.4, 0.3, 0.6, -0.2, 0.4, 0.1];
    var sevenTarget = seven.forward([0.22, -0.42, 0.32, 0.62, -0.22, 0.42, 0.12]);
    var swivel = redundant.arm.swivelAngle(sevenSeed);
    var oldSeven = redundant.arm.solve(new Transform3(new Vec3(sevenTarget.x, sevenTarget.y, sevenTarget.z),
      new Quat(sevenTarget.qx, sevenTarget.qy, sevenTarget.qz, sevenTarget.qw)), sevenSeed,
      new IkOptions(tolerance.position, tolerance.orientation, tolerance.maxIterations, tolerance.damping)
        .atSwivel(swivel, false));
    var newSeven = seven.solvePose(sevenTarget, sevenSeed, tolerance, OrientationPolicy.Interpolated);
    check(oldSeven.converged && newSeven != null, "The G0 swivel-preserving solve reaches the comparison target");
    for (i in 0...7) check(newSeven[i] == oldSeven.q[i], "Full redundant pose IK retains G0 solver bits");
  }

  public function testCaNearCableLimit():Void {
    var model = new RobotModel("xyz-ca-bent-torch");
    var base = model.addLink(new Link("base")), parent = base;
    for (i in 0...5) {
      var child = model.addLink(new Link('ca-link-$i'));
      var joint = model.addJoint(new Joint('ca-joint-$i', i < 3 ? JointType.Prismatic : JointType.Revolute, parent, child));
      joint.axis = i == 0 ? [0.0, 1.0, 0.0] : i == 1 ? [1.0, 0.0, 0.0] : i == 2 ? [0.0, 0.0, -1.0]
        : i == 3 ? [0.0, 0.0, 1.0] : [1.0, 0.0, 0.0];
      joint.limits.lower = i < 3 ? 0 : i == 3 ? -Math.PI : -Math.PI/2;
      joint.limits.upper = i == 0 ? 1 : i == 1 ? 1.2 : i == 2 ? 0.6 : i == 3 ? Math.PI : Math.PI/2;
      if (i == 3) {
        joint.parentFramePosition = [0.0, -0.367, 1.4025];
        joint.parentFrameRotation = [Math.cos(Math.PI/8), Math.sin(Math.PI/8), 0.0, 0.0];
      }
      if (i == 4) joint.parentFramePosition = [-0.074, 0.0, 0.122];
      parent = child;
    }
    var flange = model.addFrame(new Frame("ca-flange", parent));
    flange.position = [0.074, 0.0, 0.070];
    var clock = Quat.fromAxisAngle(new Vec3(0, 0, 1), Math.PI/4);
    var offset = new Transform3(clock.rotate(new Vec3(0.1365, 0, 0.3105)),
      clock.multiply(Quat.fromAxisAngle(new Vec3(0, 1, 0), Math.PI/4)));
    var solver = new ManipulatorKinematics(new KinematicGroup(model, base.id, flange.id, null, offset, null,
      ["ca-joint-0", "ca-joint-1", "ca-joint-2"]));
    solver.preferTargetOrientation = true;
    var goal = [0.9081, 0.5779, 0.0922, -2.91185, -0.02117];
    var target = solver.forward(goal), tolerance = new IkTolerance(5e-5, 1e-3, 300, 0.03);
    var candidates = solver.sampleCandidates(target, 12, tolerance, OrientationPolicy.FreeAboutTool);
    check(candidates.length > 0, "Bent torch CA sampling reaches the seam near the C cable limit");
    for (q in candidates) {
      check(PoseMath.distance(solver.forward(q), target) <= tolerance.position, "CA candidate reaches the authored wire tip");
      near(ToolFreedom.orientationError(solver.forward(q), target, OrientationPolicy.FreeAboutTool), 0,
        "CA candidate preserves the wire direction", tolerance.orientation);
    }
    var distances:Array<Float> = [], poses:Array<Pose3> = [], policies:Array<OrientationPolicy> = [];
    for (i in 0...11) {
      var q = goal.copy();
      q[1] += i * 0.002;
      q[3] += i * 0.01;
      distances.push(i * 0.005);
      poses.push(solver.forward(q));
      policies.push(OrientationPolicy.FreeAboutTool);
    }
    var route = solver.solvePathWithRates(new PathRequest(distances, poses, goal, tolerance,
      [for (_ in goal) 0.2], [for (_ in goal) 1.0], 12, policies));
    check(route.redundancyRates == null, "A fully constrained CA path has no external-axis redundancy curve");
    for (i in 0...poses.length) {
      var q = route.configurations[i];
      check(q != null, "The fully constrained CA path stays on its entry branch");
      check(PoseMath.distance(solver.forward(q), poses[i]) <= tolerance.position, "CA path follows the authored tip positions");
      near(ToolFreedom.orientationError(solver.forward(q), poses[i], policies[i]), 0,
        "CA path follows the authored wire axes", tolerance.orientation);
      near(q[3], goal[3] + i * 0.01, "CA path has no C branch flip", 1e-3);
    }
  }

  function compilerFor(solver:ManipulatorKinematics):ProgramCompiler {
    var count = solver.jointCount();
    var limits = new ValidationLimits(count, Int64.ofInt(1), Int64.ofInt(0));
    for (i in 0...count) {
      limits.position(i, -1, 1);
      limits.velocity(i, 1);
      limits.acceleration(i, 2);
      limits.jerk(i, 20);
    }
    return new ProgramCompiler(solver, limits, "work", [for (_ in 0...count) 1.0],
      [for (_ in 0...count) 2.0], [for (_ in 0...count) 20.0],
      StartTolerances.uniform(count, 0.001, 0.001, 0.001), null, 0.01, 0.5, 1e-4, 1e-4,
      new IkTolerance(1e-5, 1e-5, 200, 0.01));
  }

  function cartesian(rotary:Bool, ?track:Bool = false):ManipulatorKinematics {
    var model = new RobotModel("tool-freedom-xyz");
    var base = model.addLink(new Link("base"));
    var parent = base;
    var count = (rotary ? 4 : 3) + (track ? 1 : 0);
    for (i in 0...count) {
      var child = model.addLink(new Link('link-$i'));
      var coordinate = i - (track ? 1 : 0);
      var rotation = coordinate == 3;
      var joint = model.addJoint(new Joint('joint-$i', rotation ? JointType.Revolute : JointType.Prismatic, parent, child));
      joint.axis = rotation ? [0.0, 0.0, 1.0] : [for (k in 0...3) k == (coordinate < 0 ? 0 : coordinate) ? 1.0 : 0.0];
      joint.limits.lower = rotation ? -0.2 : -1.0;
      joint.limits.upper = rotation ? 0.2 : 1.0;
      parent = child;
    }
    var flange = model.addFrame(new Frame("flange", parent));
    return new ManipulatorKinematics(new KinematicGroup(model, base.id, flange.id, null, null, null,
      track ? ["joint-0"] : null));
  }
}
