package robotkit.manipulation;

import kinematicskit.DampedLeastSquares;
import kinematicskit.LevenbergMarquardt;
import kinematicskit.RootDampingTask;
import kinematicskit.RootMotion;
import kinematicskit.FrameTask;
import kinematicskit.JacobianLayout;
import kinematicskit.KinematicModel;
import kinematicskit.KinematicProblem;
import kinematicskit.KinematicSnapshot;
import kinematicskit.KinematicState;
import kinematicskit.KinematicStatus;
import kinematicskit.SolverWorkspace;
import kinematicskit.SwivelTask;
import kinematicskit.Transform;
import kinematicskit.Vector3;
import robotkit.kinematics.RobotKinematics;
import robotkit.model.Frame;
import robotkit.model.FrameId;
import robotkit.model.Joint;
import robotkit.model.JointId;
import robotkit.model.LinkId;
import robotkit.model.RobotModel;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.world.JointTarget;

/**
 * An arm of a robot: the joints from `baseLink` to the link carrying the
 * `flangeFrame`, its mounted tool (`flangeTTcp`, identity without one), and
 * the runtime joint indices `toJointTargets` needs to drive a compiled
 * robot. Kinematics run on the whole robot compiled by `RobotKinematics`,
 * so joint couplings apply; poses, Jacobians and IK targets are expressed
 * in the base link's frame.
 *
 * The arm's joint values `q` are one per arm DOF, in base-to-flange order:
 * each movable joint on the path that drives its own DOF, or the leader of
 * a coupled one. Robot DOFs off the arm stay at their defaults.
 *
 * A redundant arm has a `swivel` naming its extra motion: given, or for a
 * 7-DOF arm by default through the pivots of its 2nd, 4th and 6th joints.
 */
class Manipulator {
  public final robot:RobotModel;
  /** The whole robot, compiled. */
  public final model:KinematicModel;
  public final baseLink:LinkId;
  public final flangeLink:LinkId;
  public final flangeFrame:FrameId;
  public final flangeTTcp:Transform3;
  /** The arm DOFs' driving joints and their limits, in `q` order. */
  public final group:JointGroup;
  /** How the swivel (elbow) angle of a redundant arm is measured; null when none is defined. */
  public final swivel:Null<ArmSwivel>;

  final path:Array<Joint>;
  final dofs:Array<Int>;
  final jointModelIndices:Array<Int>;
  final layout:JacobianLayout;
  final baseBody:Int;
  final flangeBody:Int;
  final flangeFrameIndex:Int;
  final baseIsIdentity:Bool;
  final state:KinematicState;
  final snapshot:KinematicSnapshot;
  final workspace = new SolverWorkspace();
  /** Swivel bodies (shoulder, elbow, wrist), their points, and the reference in world coordinates. */
  final swivelBodies:Array<Int> = [];
  final swivelPoints:Array<Vector3> = [];
  var swivelReference:Null<Vector3> = null;
  var swivelProbe:Null<SwivelTask> = null;

  public function new(robot:RobotModel, baseLink:LinkId, flangeFrame:FrameId, ?flangeTTcp:Transform3,
      ?compiled:KinematicModel, ?swivel:ArmSwivel) {
    if (robot == null) throw "Manipulator requires a robot model";
    var frame:Null<Frame> = null;
    for (candidate in robot.frames) if (candidate != null && candidate.id == flangeFrame) { frame = candidate; break; }
    if (frame == null || frame.link == null) throw 'Manipulator flange frame "$flangeFrame" is not part of the model';
    this.robot = robot;
    this.model = compiled == null ? RobotKinematics.compile(robot) : compiled;
    this.baseLink = baseLink;
    this.flangeLink = frame.link.id;
    this.flangeFrame = flangeFrame;
    this.flangeTTcp = flangeTTcp == null ? Transform3.identity() : flangeTTcp;
    baseBody = model.bodyIndex(baseLink);
    if (baseBody < 0) throw 'Manipulator base link "$baseLink" is not part of the model';
    flangeBody = model.bodyIndex(flangeLink);
    flangeFrameIndex = model.frameIndex(flangeFrame);
    path = walk(robot, baseLink, flangeLink);

    dofs = [];
    for (joint in path) {
      var dof = model.jointDof[model.jointIndex(joint.id)];
      if (dof >= 0 && dofs.indexOf(dof) < 0) dofs.push(dof);
    }
    if (dofs.length == 0) throw 'Manipulator from "$baseLink" to "$flangeFrame" has no movable joints';
    layout = new JacobianLayout(model, dofs);
    var drivers:Array<Joint> = [for (dof in dofs) findJoint(robot, model.dofId(dof))];
    group = new JointGroup([for (joint in drivers) joint.id], [for (joint in drivers) joint.limits]);
    jointModelIndices = [for (joint in drivers) robot.joints.indexOf(joint)];
    var root = model.bodyParentJoint[baseBody] < 0 ? model.bodyRootPoses[baseBody] : null;
    baseIsIdentity = root != null && root.x == 0.0 && root.y == 0.0 && root.z == 0.0 && root.qx == 0.0 &&
      root.qy == 0.0 && root.qz == 0.0 && root.qw == 1.0;
    state = new KinematicState(model);
    snapshot = new KinematicSnapshot(model);
    this.swivel = swivel != null ? swivel : drivers.length == 7 ? ArmSwivel.throughJoints(drivers[1], drivers[3], drivers[5]) : null;
    if (this.swivel != null) {
      var definition:ArmSwivel = this.swivel;
      for (link in [definition.shoulderLink, definition.elbowLink, definition.wristLink]) {
        var body = model.bodyIndex(link);
        if (body < 0) throw 'Arm swivel link "$link" is not part of the model';
        swivelBodies.push(body);
      }
      for (point in [definition.shoulder, definition.elbow, definition.wrist])
        swivelPoints.push(new Vector3(point.x, point.y, point.z));
      // The reference is given in the base frame; the base does not move with the arm.
      snapshot.evaluate(state);
      var r = definition.reference;
      var world = snapshot.bodyPose(baseBody).compose(Transform.translation(r.x, r.y, r.z));
      var origin = snapshot.bodyPose(baseBody);
      swivelReference = new Vector3(world.x - origin.x, world.y - origin.y, world.z - origin.z);
      swivelProbe = swivelTask(0.0, 1.0, false);
    }
  }

  /** The same arm carrying a different tool; shares the compiled model. */
  public function withTool(flangeTTcp:Transform3):Manipulator
    return new Manipulator(robot, baseLink, flangeFrame, flangeTTcp, model, swivel);

  /** True when the arm has more DOFs than a tool pose fixes and a swivel names the extra one. */
  public function redundant():Bool return swivel != null && dofs.length > 6;

  /**
   * The swivel angle at `q` in radians (see `ArmSwivel`), or NaN without a
   * swivel or where it is undefined (the elbow straight, or the
   * shoulder-wrist line along the reference).
   */
  public function swivelAngle(q:Array<Float>):Float {
    if (swivelProbe == null) return Math.NaN;
    evaluate(q);
    var probe:SwivelTask = swivelProbe;
    try return probe.angle(snapshot) catch (_:Dynamic) return Math.NaN;
  }

  /**
   * IK for a tool-centre-point target that also sets the swivel: exactly
   * (within `swivelTolerance`), or with `soft` as a preference the tool
   * target wins over. Needs a swivel. Reports non-convergence, never throws,
   * also where the swivel is undefined along the way.
   */
  public function solveIkAtSwivel(target:Transform3, seed:Array<Float>, swivelAngle:Float, ?soft:Bool = false,
      ?positionTolerance:Float = 1e-4, ?orientationTolerance:Float = 1e-3, ?swivelTolerance:Float = 1e-4,
      ?maxIterations:Int = 100, ?damping:Float = 0.02):IKResult {
    if (target == null) throw "Inverse kinematics requires a target";
    if (swivel == null) throw "Solving at a swivel angle needs an arm swivel";
    if (!Math.isFinite(swivelAngle)) throw "Swivel angle must be finite";
    var n = dofs.length;
    var start = seed == null ? [for (_ in 0...n) 0.0] : seed;
    if (start.length != n) throw 'Joint group requires $n values, got ${start.length}';
    var problem = limitedProblem();
    evaluate(start);
    var goal = RobotKinematics.toTransform(target);
    if (!baseIsIdentity) goal = snapshot.bodyPose(baseBody).compose(goal);
    problem.add(FrameTask.atFrame(model, flangeFrameIndex, goal, positionTolerance, orientationTolerance,
      RobotKinematics.toTransform(flangeTTcp)));
    problem.add(swivelTask(swivelAngle, swivelTolerance, soft));
    try {
      var solution = DampedLeastSquares.solve(problem, state, maxIterations, damping, 1e-8, workspace);
      var tip = solution.tasks[0];
      return new IKResult(solution.status == KinematicStatus.Converged, [for (dof in dofs) solution.state.q[dof]],
        tip.positionError, tip.orientationError, solution.iterations, solution.status);
    } catch (_:Dynamic) {
      return new IKResult(false, start.copy(), Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, 0,
        KinematicStatus.NumericalFailure);
    }
  }

  function swivelTask(target:Float, tolerance:Float, soft:Bool):SwivelTask
    return new SwivelTask(model, swivelBodies[0], swivelPoints[0], swivelBodies[1], swivelPoints[1], swivelBodies[2],
      swivelPoints[2], target, tolerance, swivelReference, soft, "swivel");

  /** A problem over the arm's DOFs within the group's limits. */
  function limitedProblem():KinematicProblem {
    var problem = new KinematicProblem(model).setActiveDofs(dofs);
    for (i in 0...dofs.length) {
      var limits = group.limitsOf(i);
      if (limits.lower < limits.upper) problem.setLimits(dofs[i], limits.lower, limits.upper);
      else problem.setLimits(dofs[i], Math.NEGATIVE_INFINITY, Math.POSITIVE_INFINITY);
    }
    return problem;
  }

  public function dofCount():Int return dofs.length;

  public function jointIds():Array<JointId> return group.jointIds.copy();

  /** The runtime (compiled `RobotModel.joints`) index of each arm DOF's driving joint, in `q` order. */
  public function jointIndices():Array<Int> return jointModelIndices.copy();

  /** Every joint from the base link to the flange link (fixed ones included), base first. */
  public function pathJoints():Array<Joint> return path.copy();

  /** `base_T_flange`. */
  public function forwardKinematics(q:Array<Float>):Transform3 {
    evaluate(q);
    return RobotKinematics.toTransform3(inBase(snapshot.framePose(flangeFrameIndex)));
  }

  /** `base_T_tcp`: the flange pose composed with the mounted tool's `flange_T_tcp`. */
  public function tcpPose(q:Array<Float>):Transform3 return forwardKinematics(q).compose(flangeTTcp);

  /** Geometric Jacobian at the flange, 6 x n (rows 0..2 linear, 3..5 angular), in the base frame. */
  public function jacobian(q:Array<Float>):Array<Array<Float>> {
    var flat = pointJacobian(q, Vec3.zero());
    var n = dofs.length;
    return [for (row in 0...6) [for (column in 0...n) flat[row * n + column]]];
  }

  /** As `jacobian`, flat row-major, for the point `flangeTPoint` on the flange (e.g. the TCP). */
  public function pointJacobian(q:Array<Float>, flangeTPoint:Vec3):Array<Float> {
    evaluate(q);
    var flange = snapshot.framePose(flangeFrameIndex);
    var point = flange.transformPoint(flangeTPoint.x, flangeTPoint.y, flangeTPoint.z);
    var n = dofs.length;
    var flat = [for (_ in 0...6 * n) 0.0];
    snapshot.pointJacobianColumns(flangeBody, point.x, point.y, point.z, layout, flat);
    if (!baseIsIdentity) {
      var base = snapshot.bodyPose(baseBody);
      var inverse = new Transform(0.0, 0.0, 0.0, -base.qx, -base.qy, -base.qz, base.qw);
      for (block in 0...2) for (c in 0...n) {
        var v = inverse.transformVector(flat[(3 * block) * n + c], flat[(3 * block + 1) * n + c], flat[(3 * block + 2) * n + c]);
        flat[(3 * block) * n + c] = v.x; flat[(3 * block + 1) * n + c] = v.y; flat[(3 * block + 2) * n + c] = v.z;
      }
    }
    return flat;
  }

  /** The Jacobian of the tool centre point: linear rows at the TCP, in the base frame. */
  public function tcpJacobian(q:Array<Float>):Array<Float> return pointJacobian(q, flangeTTcp.translation);

  /**
   * Damped-least-squares IK for a flange target in the base frame, within the
   * group's limits (`lower >= upper` means unlimited). Non-convergence is
   * reported, never thrown.
   */
  public function solveIk(target:Transform3, seed:Array<Float>, ?positionTolerance:Float = 1e-4,
      ?orientationTolerance:Float = 1e-3, ?maxIterations:Int = 100, ?damping:Float = 0.02):IKResult {
    if (target == null) throw "Inverse kinematics requires a target";
    var n = dofs.length;
    var start = seed == null ? [for (_ in 0...n) 0.0] : seed;
    if (start.length != n) throw 'Joint group requires $n values, got ${start.length}';
    var problem = new KinematicProblem(model).setActiveDofs(dofs);
    for (i in 0...n) {
      var limits = group.limitsOf(i);
      if (limits.lower < limits.upper) problem.setLimits(dofs[i], limits.lower, limits.upper);
      else problem.setLimits(dofs[i], Math.NEGATIVE_INFINITY, Math.POSITIVE_INFINITY);
    }
    evaluate(start);
    var goal = RobotKinematics.toTransform(target);
    if (!baseIsIdentity) goal = snapshot.bodyPose(baseBody).compose(goal);
    problem.add(FrameTask.atFrame(model, flangeFrameIndex, goal, positionTolerance, orientationTolerance));
    var solution = DampedLeastSquares.solve(problem, state, maxIterations, damping, 1e-8, workspace);
    var tip = solution.tasks[0];
    return new IKResult(solution.status == KinematicStatus.Converged, [for (dof in dofs) solution.state.q[dof]],
      tip.positionError, tip.orientationError, solution.iterations, solution.status);
  }

  /**
   * IK for a target at the tool centre point, by converting it to the
   * equivalent flange target (`target.compose(flangeTTcp.inverse())`).
   */
  public function solveIkForTcp(target:Transform3, seed:Array<Float>, ?positionTolerance:Float = 1e-4,
      ?orientationTolerance:Float = 1e-3, ?maxIterations:Int = 100, ?damping:Float = 0.02):IKResult {
    if (target == null) throw "TCP inverse kinematics requires a target";
    return solveIk(target.compose(flangeTTcp.inverse()), seed, positionTolerance, orientationTolerance,
      maxIterations, damping);
  }

  /**
   * How the robot's base may move in `solveIkWithBase`: `Floating` for a
   * `floatingBase` robot, `Planar` (x, y, yaw on the floor) for a
   * `mobileBase` one, `Fixed` otherwise. Planar treats the base as able to
   * reach any floor pose, which is right for deciding where to stand, not
   * for instantaneous motion of a differential drive.
   */
  public function baseMotion():RootMotion
    return robot.floatingBase ? RootMotion.Floating : robot.mobileBase != null ? RootMotion.Planar : RootMotion.Fixed;

  /**
   * IK for a tool-centre-point target in the world frame, moving the base
   * as well as the arm when the robot has a movable base (`baseMotion`):
   * where to put the base and how to set the arm so the tool reaches the
   * target. `rootPose` is the robot root's current world pose; the result's
   * `rootPose` is where the solve moved it. `baseCost` (> 0) makes the arm
   * do what it can before the base moves. Levenberg-Marquardt (a reaching
   * solve; see KINEMATICS.md KK-D11), tolerances at the TCP. On a
   * fixed-base robot this is `solveIkForTcp` expressed in the world frame.
   */
  public function solveIkWithBase(target:Transform3, seed:Array<Float>, rootPose:Transform3,
      ?positionTolerance:Float = 1e-4, ?orientationTolerance:Float = 1e-3, ?maxIterations:Int = 200,
      ?baseCost:Float = 0.1):IKResult {
    if (target == null || rootPose == null) throw "Inverse kinematics with a base requires a target and a root pose";
    if (!(baseCost >= 0.0)) throw "Base cost must be non-negative";
    var n = dofs.length;
    var start = seed == null ? [for (_ in 0...n) 0.0] : seed;
    if (start.length != n) throw 'Joint group requires $n values, got ${start.length}';
    var root = model.bodyRoot[baseBody];
    var problem = new KinematicProblem(model).setActiveDofs(dofs);
    for (i in 0...n) {
      var limits = group.limitsOf(i);
      if (limits.lower < limits.upper) problem.setLimits(dofs[i], limits.lower, limits.upper);
      else problem.setLimits(dofs[i], Math.NEGATIVE_INFINITY, Math.POSITIVE_INFINITY);
    }
    // The tool task first, so the result reports it.
    problem.add(FrameTask.atFrame(model, flangeFrameIndex, RobotKinematics.toTransform(target), positionTolerance,
      orientationTolerance, RobotKinematics.toTransform(flangeTTcp)));
    var motion = baseMotion();
    if (motion != RootMotion.Fixed) {
      problem.setRootMotion(root, motion);
      if (baseCost > 0.0) problem.add(new RootDampingTask(model, root, baseCost));
    }
    var seedState = new KinematicState(model);
    for (i in 0...n) seedState.q[dofs[i]] = start[i];
    seedState.setRootPose(root, RobotKinematics.toTransform(rootPose));
    problem.clamp(seedState.q);
    var solution = LevenbergMarquardt.solve(problem, seedState, maxIterations, 1e-3, 1e-8, 1.0, workspace);
    var tip = solution.tasks[0];
    return new IKResult(solution.status == KinematicStatus.Converged, [for (dof in dofs) solution.state.q[dof]],
      tip.positionError, tip.orientationError, solution.iterations, solution.status,
      RobotKinematics.toTransform3(solution.state.rootPose(root)));
  }

  /** Position targets for every arm DOF's driving joint, indexed by its compiled RobotModel position. */
  public function toJointTargets(q:Array<Float>):Array<JointTarget> {
    if (q == null || q.length != jointModelIndices.length)
      throw 'Manipulator requires ${jointModelIndices.length} joint values, got ${q == null ? 0 : q.length}';
    return [for (i in 0...q.length) JointTarget.position(jointModelIndices[i], q[i])];
  }

  function evaluate(q:Array<Float>):Void {
    if (q == null || q.length != dofs.length)
      throw 'Manipulator requires ${dofs.length} joint values, got ${q == null ? 0 : q.length}';
    for (i in 0...dofs.length) state.q[dofs[i]] = q[i];
    snapshot.evaluate(state);
  }

  function inBase(world:Transform):Transform
    return baseIsIdentity ? world : snapshot.bodyPose(baseBody).inverse().compose(world);

  /** The joints from `baseLink` to `tipLink`, base first; throws when the tip is not below the base. */
  public static function walk(robot:RobotModel, baseLink:LinkId, tipLink:LinkId):Array<Joint> {
    var joints:Array<Joint> = [];
    var current = tipLink;
    var guard = 0;
    while (current != baseLink) {
      if (guard++ > robot.links.length) throw "Manipulator walk did not terminate; check the model topology";
      var incoming:Null<Joint> = null;
      for (joint in robot.joints) if (joint != null && joint.child != null && joint.child.id == current) {
        incoming = joint;
        break;
      }
      if (incoming == null) throw 'Manipulator has no path from base link "$baseLink" to link "$current"';
      joints.push(incoming);
      current = incoming.parent.id;
    }
    joints.reverse();
    return joints;
  }

  static function findJoint(robot:RobotModel, id:JointId):Joint {
    for (joint in robot.joints) if (joint != null && joint.id == id) return joint;
    throw 'Manipulator joint "$id" is not part of the model';
  }
}
