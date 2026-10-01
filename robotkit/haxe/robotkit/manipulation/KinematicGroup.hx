package robotkit.manipulation;

import kinematicskit.DampedLeastSquares;
import kinematicskit.DofDampingTask;
import kinematicskit.FrameTask;
import kinematicskit.JacobianLayout;
import kinematicskit.KinematicModel;
import kinematicskit.KinematicProblem;
import kinematicskit.KinematicSnapshot;
import kinematicskit.KinematicState;
import kinematicskit.KinematicStatus;
import kinematicskit.LevenbergMarquardt;
import kinematicskit.PostureTask;
import kinematicskit.PrioritizedSolver;
import kinematicskit.RootDampingTask;
import kinematicskit.RootMotion;
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
 * The joints that move a tool, solved together: those from `rootLink` to
 * the link carrying the `flangeFrame` (a rail under an arm, then the arm),
 * and with a `workFrame`, those from `rootLink` to it (a positioner holding
 * the workpiece). Kinematics run on the whole robot compiled by
 * `RobotKinematics`, so joint couplings apply; robot DOFs outside the group
 * stay at their defaults.
 *
 * Every pose, Jacobian and IK target is expressed in the group's reference
 * frame: the work frame when there is one (so a path on the workpiece stays
 * on it however the positioner turns it), else the root link's frame.
 * `Manipulator` is the common case: a fixed arm, poses in its base frame.
 *
 * `q` holds the flange path's DOFs (base first), then the work path's that
 * the flange path does not share; each is a movable joint driving its own
 * DOF or the leader of a coupled one. External axes (the work path's
 * joints, plus any named `externalAxes`) are damped in every solve, so they
 * move for what the arm cannot do alone; the damping shapes the steps,
 * never the answer.
 *
 * A redundant arm has a `swivel` naming its extra motion: given, or for a
 * 7-DOF arm (external axes aside) by default through the pivots of its 2nd,
 * 4th and 6th joints.
 *
 * One solve entry point, `solve`, takes its settings as `IkOptions`
 * (tracking or reaching, TCP or flange target, swivel, posture, moving base).
 * The group is all synchronous state, as everything that commands it runs
 * on one robot runtime: a plan over its joints executes on one clock.
 */
class KinematicGroup {
  public final robot:RobotModel;
  /** The whole robot, compiled. */
  public final model:KinematicModel;
  public final rootLink:LinkId;
  public final flangeLink:LinkId;
  public final flangeFrame:FrameId;
  /** Frame poses are expressed in; null for the root link's frame. */
  public final workFrame:Null<FrameId>;
  public final flangeTTcp:Transform3;
  /** The group DOFs' driving joints and their limits, in `q` order. */
  public final group:JointGroup;
  /** True for each group DOF that is an external axis (damped), in `q` order. */
  public final external:Array<Bool>;
  /** How the swivel (elbow) angle of a redundant arm is measured; null when none is defined. */
  public final swivel:Null<ArmSwivel>;
  /** Weight of an external axis's step damping, in the units of the tool task (1 m weighs like 1 rad). */
  public final externalWeight:Float;

  final path:Array<Joint>;
  final dofs:Array<Int>;
  final jointModelIndices:Array<Int>;
  final layout:JacobianLayout;
  final flangeBody:Int;
  final flangeFrameIndex:Int;
  final rootBody:Int;
  /** Reference frame: a body and the frame's pose on it. */
  final referenceBody:Int;
  final referenceOffset:Transform;
  final damping:Array<Float>;
  final state:KinematicState;
  final snapshot:KinematicSnapshot;
  final workspace = new SolverWorkspace();
  final swivelBodies:Array<Int> = [];
  final swivelPoints:Array<Vector3> = [];
  var swivelReference:Null<Vector3> = null;
  var swivelProbe:Null<SwivelTask> = null;

  /**
   * `externalAxes` names further joints on the flange path to treat as
   * external (a rail under the arm); the work path's joints always are.
   */
  public function new(robot:RobotModel, rootLink:LinkId, flangeFrame:FrameId, ?workFrame:FrameId,
      ?flangeTTcp:Transform3, ?swivel:ArmSwivel, ?externalAxes:Array<JointId>, ?externalWeight:Float = 0.3,
      ?compiled:KinematicModel) {
    if (robot == null) throw "A kinematic group requires a robot model";
    if (!(externalWeight >= 0.0) || !Math.isFinite(externalWeight)) throw "External axis weight must be non-negative";
    var flange = frameOf(robot, flangeFrame);
    this.robot = robot;
    this.model = compiled == null ? RobotKinematics.compile(robot) : compiled;
    this.rootLink = rootLink;
    this.flangeLink = flange.link.id;
    this.flangeFrame = flangeFrame;
    this.workFrame = workFrame;
    this.flangeTTcp = flangeTTcp == null ? Transform3.identity() : flangeTTcp;
    this.externalWeight = externalWeight;
    rootBody = model.bodyIndex(rootLink);
    if (rootBody < 0) throw 'Kinematic group root link "$rootLink" is not part of the model';
    flangeBody = model.bodyIndex(flangeLink);
    flangeFrameIndex = model.frameIndex(flangeFrame);
    path = walk(robot, rootLink, flangeLink);

    dofs = [];
    external = [];
    var named = externalAxes == null ? [] : externalAxes;
    for (joint in path) addDof(joint, named.indexOf(joint.id) >= 0);
    for (id in named) {
      var found = false;
      for (joint in path) if (joint.id == id) found = true;
      if (!found) throw 'External axis "$id" is not on the path to the flange';
    }
    if (workFrame != null) {
      var work = frameOf(robot, workFrame);
      for (joint in walk(robot, rootLink, work.link.id)) addDof(joint, true);
      var index = model.frameIndex(workFrame);
      referenceBody = model.frameBody[index];
      referenceOffset = model.frameTransform(index);
    } else {
      referenceBody = rootBody;
      referenceOffset = Transform.identity();
    }
    if (dofs.length == 0) throw 'Nothing moves the tool of "$flangeFrame" from "$rootLink"';
    layout = new JacobianLayout(model, dofs);
    var drivers:Array<Joint> = [for (dof in dofs) findJoint(robot, model.dofId(dof))];
    group = new JointGroup([for (joint in drivers) joint.id], [for (joint in drivers) joint.limits]);
    jointModelIndices = [for (joint in drivers) robot.joints.indexOf(joint)];
    damping = [for (_ in 0...model.dofCount()) 0.0];
    for (i in 0...dofs.length) if (external[i]) damping[dofs[i]] = externalWeight;
    state = new KinematicState(model);
    snapshot = new KinematicSnapshot(model);

    var arm = [for (i in 0...drivers.length) if (!external[i]) drivers[i]];
    this.swivel = swivel != null ? swivel : arm.length == 7 ? ArmSwivel.throughJoints(arm[1], arm[3], arm[5]) : null;
    if (this.swivel != null) {
      var definition:ArmSwivel = this.swivel;
      for (link in [definition.shoulderLink, definition.elbowLink, definition.wristLink]) {
        var body = model.bodyIndex(link);
        if (body < 0) throw 'Arm swivel link "$link" is not part of the model';
        swivelBodies.push(body);
      }
      for (point in [definition.shoulder, definition.elbow, definition.wrist])
        swivelPoints.push(new Vector3(point.x, point.y, point.z));
      // The reference is given in the root link's frame, which the arm does not move.
      snapshot.evaluate(state);
      var r = definition.reference;
      var root = snapshot.bodyPose(rootBody);
      var tip = root.compose(Transform.translation(r.x, r.y, r.z));
      swivelReference = new Vector3(tip.x - root.x, tip.y - root.y, tip.z - root.z);
      swivelProbe = swivelTask(0.0, 1.0, false);
    }
  }

  public function dofCount():Int return dofs.length;

  public function jointIds():Array<JointId> return group.jointIds.copy();

  /** The runtime (compiled `RobotModel.joints`) index of each group DOF's driving joint, in `q` order. */
  public function jointIndices():Array<Int> return jointModelIndices.copy();

  /** Every joint from the root link to the flange link (fixed ones included), root first. */
  public function pathJoints():Array<Joint> return path.copy();

  /** True when an arm has more DOFs than a tool pose fixes and a swivel names the extra one. */
  public function redundant():Bool {
    var arm = 0;
    for (value in external) if (!value) arm++;
    return swivel != null && arm > 6;
  }

  /**
   * How the robot's base may move in a solve with `IkOptions.rootPose`:
   * `Floating` for a `floatingBase` robot, `Planar` (x, y, yaw on the floor)
   * for a `mobileBase` one, `Fixed` otherwise. Planar treats the base as able
   * to reach any floor pose, which is right for deciding where to stand, not
   * for instantaneous motion of a differential drive.
   */
  public function baseMotion():RootMotion
    return robot.floatingBase ? RootMotion.Floating : robot.mobileBase != null ? RootMotion.Planar : RootMotion.Fixed;

  /** The flange's pose in the reference frame. */
  public function forwardKinematics(q:Array<Float>):Transform3 {
    evaluate(q);
    return RobotKinematics.toTransform3(referencePose().inverse().compose(snapshot.framePose(flangeFrameIndex)));
  }

  /** The tool centre point's pose in the reference frame (the flange composed with `flange_T_tcp`). */
  public function tcpPose(q:Array<Float>):Transform3 return forwardKinematics(q).compose(flangeTTcp);

  /** The work frame's pose in the root link's frame (the root link's own pose without a work frame). */
  public function workPose(q:Array<Float>):Transform3 {
    evaluate(q);
    return RobotKinematics.toTransform3(snapshot.bodyPose(rootBody).inverse().compose(referencePose()));
  }

  /** Geometric Jacobian at the flange, 6 x n (rows 0..2 linear, 3..5 angular), in the reference frame. */
  public function jacobian(q:Array<Float>):Array<Array<Float>> {
    var flat = pointJacobian(q, Vec3.zero());
    var n = dofs.length;
    return [for (row in 0...6) [for (column in 0...n) flat[row * n + column]]];
  }

  /**
   * As `jacobian`, flat row-major, for the point `flangeTPoint` on the
   * flange (e.g. the TCP): its velocity relative to the reference frame
   * (which moves with a positioner), expressed in it.
   */
  public function pointJacobian(q:Array<Float>, flangeTPoint:Vec3):Array<Float> {
    evaluate(q);
    var point = snapshot.framePose(flangeFrameIndex).transformPoint(flangeTPoint.x, flangeTPoint.y, flangeTPoint.z);
    var n = dofs.length;
    var flat = [for (_ in 0...6 * n) 0.0];
    snapshot.pointJacobianColumns(flangeBody, point.x, point.y, point.z, layout, flat);
    var reference = referencePose();
    if (workFrame != null) {
      // The reference body's own motion at that point.
      var moving = [for (_ in 0...6 * n) 0.0];
      snapshot.pointJacobianColumns(referenceBody, point.x, point.y, point.z, layout, moving);
      for (i in 0...6 * n) flat[i] -= moving[i];
    }
    var inverse = new Transform(0.0, 0.0, 0.0, -reference.qx, -reference.qy, -reference.qz, reference.qw);
    for (block in 0...2) for (c in 0...n) {
      var v = inverse.transformVector(flat[(3 * block) * n + c], flat[(3 * block + 1) * n + c],
        flat[(3 * block + 2) * n + c]);
      flat[(3 * block) * n + c] = v.x;
      flat[(3 * block + 1) * n + c] = v.y;
      flat[(3 * block + 2) * n + c] = v.z;
    }
    return flat;
  }

  /** The Jacobian of the tool centre point: linear rows at the TCP, in the reference frame. */
  public function tcpJacobian(q:Array<Float>):Array<Float> return pointJacobian(q, flangeTTcp.translation);

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
   * Inverse kinematics: joint values that put the tool centre point (or the
   * flange, `options.atFlange`) at `target`, in the reference frame, or with
   * `options.rootPose` in the world while the base moves too. Within the
   * group's limits (`lower >= upper` means unlimited). Non-convergence is
   * reported, never thrown, also where a swivel is undefined along the way.
   */
  public function solve(target:Transform3, seed:Array<Float>, ?options:IkOptions):IKResult {
    if (target == null) throw "Inverse kinematics requires a target";
    var o = options == null ? new IkOptions() : options;
    var n = dofs.length;
    var start = seed == null ? [for (_ in 0...n) 0.0] : seed;
    if (start.length != n) throw 'Kinematic group requires $n values, got ${start.length}';
    if (o.swivel != null && swivel == null) throw "Solving at a swivel angle needs an arm swivel";
    if (o.swivel != null && !Math.isFinite(o.swivel)) throw "Swivel angle must be finite";
    if (o.rootPose != null) return solveWithBase(target, start, o);
    for (i in 0...n) state.q[dofs[i]] = start[i];
    snapshot.evaluate(state);
    var problem = limitedProblem();
    problem.add(toolTask(RobotKinematics.toTransform(target), o));
    if (hasExternal()) problem.add(new DofDampingTask(model, damping));
    if (o.swivel != null) problem.add(swivelTask(o.swivel, o.swivelTolerance, !o.swivelExact));
    var method = o.method;
    if (o.posture != null) {
      var posture:Array<Float> = o.posture;
      if (posture.length != n) throw 'Preferred posture needs $n values';
      // The arm is drawn towards its posture; the external axes take up the rest.
      var targets = [for (_ in 0...model.dofCount()) 0.0], weights = [for (_ in 0...model.dofCount()) 0.0];
      for (i in 0...n) if (!external[i]) {
        targets[dofs[i]] = posture[i];
        weights[dofs[i]] = 1.0;
      }
      problem.add(new PostureTask(model, targets, o.postureWeight, weights));
      method = IkMethod.Prioritized;
    }
    try {
      var solution = run(problem, state, o, method);
      var tip = solution.tasks[0];
      return new IKResult(solution.status == KinematicStatus.Converged, [for (dof in dofs) solution.state.q[dof]],
        tip.positionError, tip.orientationError, solution.iterations, solution.status);
    } catch (_:Dynamic) {
      return new IKResult(false, start.copy(), Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, 0,
        KinematicStatus.NumericalFailure);
    }
  }

  /** Position targets for every group DOF's driving joint, indexed by its compiled RobotModel position. */
  public function toJointTargets(q:Array<Float>):Array<JointTarget> {
    if (q == null || q.length != jointModelIndices.length)
      throw 'Kinematic group requires ${jointModelIndices.length} joint values, got ${q == null ? 0 : q.length}';
    return [for (i in 0...q.length) JointTarget.position(jointModelIndices[i], q[i])];
  }

  function run(problem:KinematicProblem, seed:KinematicState, o:IkOptions, method:IkMethod):kinematicskit.KinematicSolution
    return switch method {
      case Reaching: LevenbergMarquardt.solve(problem, seed, o.maxIterations, o.damping > 0.0 ? o.damping : 1e-3, 1e-8,
          1.0, workspace);
      case Prioritized: PrioritizedSolver.solve(problem, seed, o.maxIterations, o.damping > 0.0 ? o.damping : 0.02,
          1e-8, workspace);
      default: DampedLeastSquares.solve(problem, seed, o.maxIterations, o.damping, 1e-8, workspace);
    };

  /**
   * The base moves as well as the joints (`baseMotion`); the target is in
   * the world frame and the result says where the root went. A reaching
   * solve (KINEMATICS.md KK-D11), tolerances at the TCP or flange.
   */
  function solveWithBase(target:Transform3, start:Array<Float>, o:IkOptions):IKResult {
    if (!(o.baseCost >= 0.0)) throw "Base cost must be non-negative";
    var root = model.bodyRoot[rootBody];
    var problem = limitedProblem();
    // The tool task first, so the result reports it.
    problem.add(FrameTask.atFrame(model, flangeFrameIndex, RobotKinematics.toTransform(target), o.positionTolerance,
      o.orientationTolerance, o.atFlange ? null : RobotKinematics.toTransform(flangeTTcp)));
    if (hasExternal()) problem.add(new DofDampingTask(model, damping));
    var motion = baseMotion();
    if (motion != RootMotion.Fixed) {
      problem.setRootMotion(root, motion);
      if (o.baseCost > 0.0) problem.add(new RootDampingTask(model, root, o.baseCost));
    }
    var seedState = new KinematicState(model);
    for (i in 0...dofs.length) seedState.q[dofs[i]] = start[i];
    seedState.setRootPose(root, RobotKinematics.toTransform(o.rootPose));
    problem.clamp(seedState.q);
    var solution = LevenbergMarquardt.solve(problem, seedState, o.maxIterations, 1e-3, 1e-8, 1.0, workspace);
    var tip = solution.tasks[0];
    return new IKResult(solution.status == KinematicStatus.Converged, [for (dof in dofs) solution.state.q[dof]],
      tip.positionError, tip.orientationError, solution.iterations, solution.status,
      RobotKinematics.toTransform3(solution.state.rootPose(root)));
  }

  /**
   * The tool frame at `target` in the reference frame. A work frame moves
   * with the solve, so the task follows it; the root link does not, so its
   * pose (from the current snapshot) turns the target into a world target.
   */
  function toolTask(target:Transform, o:IkOptions):FrameTask {
    var offset = o.atFlange ? null : RobotKinematics.toTransform(flangeTTcp);
    if (workFrame == null)
      return FrameTask.atFrame(model, flangeFrameIndex, referencePose().compose(target), o.positionTolerance,
        o.orientationTolerance, offset);
    return FrameTask.atFrame(model, flangeFrameIndex, target, o.positionTolerance, o.orientationTolerance, offset)
      .relativeTo(model, referenceBody, referenceOffset);
  }

  function hasExternal():Bool {
    for (value in external) if (value) return true;
    return false;
  }

  function swivelTask(target:Float, tolerance:Float, soft:Bool):SwivelTask
    return new SwivelTask(model, swivelBodies[0], swivelPoints[0], swivelBodies[1], swivelPoints[1], swivelBodies[2],
      swivelPoints[2], target, tolerance, swivelReference, soft, "swivel");

  /** A problem over the group's DOFs within their limits. */
  function limitedProblem():KinematicProblem {
    var problem = new KinematicProblem(model).setActiveDofs(dofs);
    for (i in 0...dofs.length) {
      var limits = group.limitsOf(i);
      if (limits.lower < limits.upper) problem.setLimits(dofs[i], limits.lower, limits.upper);
      else problem.setLimits(dofs[i], Math.NEGATIVE_INFINITY, Math.POSITIVE_INFINITY);
    }
    return problem;
  }

  function referencePose():Transform return snapshot.bodyPose(referenceBody).compose(referenceOffset);

  function addDof(joint:Joint, isExternal:Bool):Void {
    var dof = model.jointDof[model.jointIndex(joint.id)];
    if (dof < 0) return;
    var existing = dofs.indexOf(dof);
    if (existing >= 0) {
      if (isExternal) external[existing] = true;
      return;
    }
    dofs.push(dof);
    external.push(isExternal);
  }

  function evaluate(q:Array<Float>):Void {
    if (q == null || q.length != dofs.length)
      throw 'Kinematic group requires ${dofs.length} joint values, got ${q == null ? 0 : q.length}';
    for (i in 0...dofs.length) state.q[dofs[i]] = q[i];
    snapshot.evaluate(state);
  }

  /** The joints from `baseLink` to `tipLink`, base first; throws when the tip is not below the base. */
  public static function walk(robot:RobotModel, baseLink:LinkId, tipLink:LinkId):Array<Joint> {
    var joints:Array<Joint> = [];
    var current = tipLink;
    var guard = 0;
    while (current != baseLink) {
      if (guard++ > robot.links.length) throw "Kinematic group walk did not terminate; check the model topology";
      var incoming:Null<Joint> = null;
      for (joint in robot.joints) if (joint != null && joint.child != null && joint.child.id == current) {
        incoming = joint;
        break;
      }
      if (incoming == null) throw 'No path from link "$baseLink" to link "$current"';
      joints.push(incoming);
      current = incoming.parent.id;
    }
    joints.reverse();
    return joints;
  }

  static function frameOf(robot:RobotModel, id:FrameId):Frame {
    for (candidate in robot.frames) if (candidate != null && candidate.id == id && candidate.link != null) return candidate;
    throw 'Frame "$id" is not part of the model';
  }

  static function findJoint(robot:RobotModel, id:JointId):Joint {
    for (joint in robot.joints) if (joint != null && joint.id == id) return joint;
    throw 'Joint "$id" is not part of the model';
  }
}
