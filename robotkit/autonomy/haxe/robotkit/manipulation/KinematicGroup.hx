package robotkit.manipulation;

import kinematicskit.DampedLeastSquares;
import kinematicskit.DofDampingTask;
import kinematicskit.FrameTask;
import kinematicskit.FrameVelocityTask;
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
import robotkit.core.JointTarget;
import sys.thread.Tls;

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
 * The group itself never changes once built; what an evaluation writes
 * (the model state, its snapshot, solver scratch) is `KinematicGroupData`.
 * Each query takes the data to work in, by default this thread's own, so a
 * group can be shared between threads (a planner and the frame loop).
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

  public final profile:robotkit.profile.RobotProfile;
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
  final swivelBodies:Array<Int> = [];
  final swivelPoints:Array<Vector3> = [];
  var swivelReference:Null<Vector3> = null;

  /**
   * `externalAxes` names further joints on the flange path to treat as
   * external (a rail under the arm); the work path's joints always are.
   */
  public function new(robot:RobotModel, rootLink:LinkId, flangeFrame:FrameId, ?workFrame:FrameId,
      ?flangeTTcp:Transform3, ?swivel:ArmSwivel, ?externalAxes:Array<JointId>, ?externalWeight:Float = 0.3,
      ?compiled:KinematicModel, ?profile:robotkit.profile.RobotProfile) {
    if (robot == null) throw "A kinematic group requires a robot model";
    this.profile = profile == null ? new robotkit.profile.RobotProfile() : profile;
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
    var named = externalAxes == null ? ExternalAxes.derive(robot, rootLink, flangeFrame) : externalAxes;
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
      var initial = new KinematicSnapshot(model);
      initial.evaluate(new KinematicState(model));
      var r = definition.reference;
      var root = initial.bodyPose(rootBody);
      var tip = root.compose(Transform.translation(r.x, r.y, r.z));
      swivelReference = new Vector3(tip.x - root.x, tip.y - root.y, tip.z - root.z);
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
    return robot.floatingBase ? RootMotion.Floating : profile.mobileBase != null ? RootMotion.Planar : RootMotion.Fixed;

  /** Each thread's working data for the groups it evaluated most recently, the latest first. */
  static final threadData = new Tls<Array<KinematicGroupData>>();
  static inline var THREAD_CACHED_GROUPS = 8;

  /**
   * This thread's working data for the group: what every query uses unless given its own. Kept for
   * the few groups the thread used last; one evicted and needed again starts afresh, which no
   * query notices, as each writes the state it reads.
   */
  public function threadLocalData():KinematicGroupData {
    var cache = threadData.value;
    if (cache == null) {
      cache = [];
      threadData.value = cache;
    }
    for (i in 0...cache.length) {
      var found = cache[i];
      if (found.group != this) continue;
      if (i > 0) {
        cache.splice(i, 1);
        cache.unshift(found);
      }
      return found;
    }
    var created = newData();
    cache.unshift(created);
    if (cache.length > THREAD_CACHED_GROUPS) cache.pop();
    return created;
  }

  /** Fresh working data for the group, for a caller that keeps its own (a planner, a hot loop). */
  public function newData():KinematicGroupData
    return new KinematicGroupData(this, swivel == null ? null : swivelTask(0.0, 1.0, false));

  /** The flange's pose in the reference frame. */
  public function forwardKinematics(q:Array<Float>, ?data:KinematicGroupData):Transform3 {
    var d = evaluate(q, data);
    return RobotKinematics.toTransform3(referencePose(d).inverse().compose(d.snapshot.framePose(flangeFrameIndex)));
  }

  /**
   * The poses of the bodies of `links` in the reference frame at joint values `q`, in one evaluation: for placing what
   * the links carry (collision hulls) as the arm moves.
   */
  public function linkPoses(q:Array<Float>, links:Array<LinkId>, ?data:KinematicGroupData):Array<Transform3> {
    var d = evaluate(q, data);
    var inverse = referencePose(d).inverse();
    return [for (link in links) {
      var body = model.bodyIndex(link);
      if (body < 0) throw 'Link "$link" is not part of the model';
      RobotKinematics.toTransform3(inverse.compose(d.snapshot.bodyPose(body)));
    }];
  }

  /** Whether the link moves when one of the group's joints does (it is carried by such a joint, however far out). */
  public function moves(link:LinkId):Bool {
    for (joint in walk(robot, rootLink, link)) {
      var index = model.jointIndex(joint.id);
      if(index>=0)for(term in model.jointTermStart[index]...model.jointTermStart[index+1])
        if(dofs.indexOf(model.jointTermDof[term])>=0)return true;
    }
    return false;
  }

  /** The tool centre point's pose in the reference frame (the flange composed with `flange_T_tcp`). */
  public function tcpPose(q:Array<Float>, ?data:KinematicGroupData):Transform3
    return forwardKinematics(q, data).compose(flangeTTcp);

  /** The work frame's pose in the root link's frame (the root link's own pose without a work frame). */
  public function workPose(q:Array<Float>, ?data:KinematicGroupData):Transform3 {
    var d = evaluate(q, data);
    return RobotKinematics.toTransform3(d.snapshot.bodyPose(rootBody).inverse().compose(referencePose(d)));
  }

  /** Geometric Jacobian at the flange, 6 x n (rows 0..2 linear, 3..5 angular), in the reference frame. */
  public function jacobian(q:Array<Float>, ?data:KinematicGroupData):Array<Array<Float>> {
    var flat = pointJacobian(q, Vec3.zero(), data);
    var n = dofs.length;
    return [for (row in 0...6) [for (column in 0...n) flat[row * n + column]]];
  }

  /**
   * As `jacobian`, flat row-major, for the point `flangeTPoint` on the
   * flange (e.g. the TCP): its velocity relative to the reference frame
   * (which moves with a positioner), expressed in it.
   */
  public function pointJacobian(q:Array<Float>, flangeTPoint:Vec3, ?data:KinematicGroupData):Array<Float> {
    var d = evaluate(q, data);
    var snapshot = d.snapshot;
    var point = snapshot.framePose(flangeFrameIndex).transformPoint(flangeTPoint.x, flangeTPoint.y, flangeTPoint.z);
    var n = dofs.length;
    var flat = [for (_ in 0...6 * n) 0.0];
    snapshot.pointJacobianColumns(flangeBody, point.x, point.y, point.z, layout, flat);
    var reference = referencePose(d);
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
  public function tcpJacobian(q:Array<Float>, ?data:KinematicGroupData):Array<Float>
    return pointJacobian(q, flangeTTcp.translation, data);

  /**
   * The swivel angle at `q` in radians (see `ArmSwivel`), or NaN without a
   * swivel or where it is undefined (the elbow straight, or the
   * shoulder-wrist line along the reference).
   */
  public function swivelAngle(q:Array<Float>, ?data:KinematicGroupData):Float {
    if (swivel == null) return Math.NaN;
    var d = evaluate(q, data);
    var probe:SwivelTask = d.swivelProbe;
    try return probe.angle(d.snapshot) catch (_:Dynamic) return Math.NaN;
  }

  /**
   * The swivel angle's gradient at `q`, one value per group DOF (dψ = G · dq), or null without a
   * swivel or where the angle is undefined.
   */
  public function swivelJacobian(q:Array<Float>, ?data:KinematicGroupData):Null<Array<Float>> {
    if (swivel == null) return null;
    var d = evaluate(q, data);
    var probe:SwivelTask = d.swivelProbe;
    var width = layout.width;
    var residual = [0.0], row = [for (_ in 0...width) 0.0];
    try probe.evaluate(d.state, d.snapshot, layout, residual, row, 0) catch (_:Dynamic) return null;
    var gradient = [for (i in 0...dofs.length) row[i]];
    for (value in gradient) if (!Math.isFinite(value)) return null;
    return gradient;
  }

  /** Diagnostic count for this thread (or explicit context), without sharing mutable counters between planners. */
  public function numericSolveCount(?data:KinematicGroupData):Int return dataFor(data).numericSolves;

  /**
   * Inverse kinematics: joint values that put the tool centre point (or the
   * flange, `options.atFlange`) at `target`, in the reference frame, or with
   * `options.rootPose` in the world while the base moves too. Within the
   * group's limits (`lower >= upper` means unlimited). Non-convergence is
   * reported, never thrown, also where a swivel is undefined along the way.
   */
  public function solve(target:Transform3, seed:Array<Float>, ?options:IkOptions, ?data:KinematicGroupData):IKResult {
    if (target == null) throw "Inverse kinematics requires a target";
    var o = options == null ? new IkOptions() : options;
    var n = dofs.length;
    var start = seed == null ? [for (_ in 0...n) 0.0] : seed;
    if (start.length != n) throw 'Kinematic group requires $n values, got ${start.length}';
    if (o.swivel != null && swivel == null) throw "Solving at a swivel angle needs an arm swivel";
    if (o.swivel != null && !Math.isFinite(o.swivel)) throw "Swivel angle must be finite";
    var d = dataFor(data);
    d.numericSolves++;
    if (o.rootPose != null) return solveWithBase(target, start, o, d);
    var state = d.state;
    for (i in 0...n) state.q[dofs[i]] = start[i];
    // Held DOFs sit at their values and leave the solve.
    var excluded = [for (_ in 0...n) false];
    if (o.held != null) {
      var held:Array<Int> = o.held, values:Array<Float> = o.heldValues;
      for (k in 0...held.length) {
        if (held[k] < 0 || held[k] >= n || !Math.isFinite(values[k])) throw "Held DOFs must be group DOFs with finite values";
        excluded[held[k]] = true;
        state.q[dofs[held[k]]] = values[k];
      }
    }
    d.snapshot.evaluate(state);
    var problem = limitedProblem(excluded);
    problem.add(toolTask(RobotKinematics.toTransform(target), o, d));
    if (hasExternal()) problem.add(new DofDampingTask(model, damping));
    if (o.swivel != null) problem.add(swivelTask(o.swivel, o.swivelTolerance, !o.swivelExact));
    var method = o.method;
    if (o.orientationPreference != null) {
      problem.add(orientationPreferenceTask(target, o, d));
      method = IkMethod.Prioritized;
    }
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
      var solution = run(problem, state, o, method, d.workspace);
      var tip = solution.tasks[0];
      return new IKResult(solution.status == KinematicStatus.Converged, [for (dof in dofs) solution.state.q[dof]],
        tip.positionError, tip.orientationError, solution.iterations, solution.status);
    } catch (_:Dynamic) {
      return new IKResult(false, start.copy(), Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, 0,
        KinematicStatus.NumericalFailure);
    }
  }

  /**
   * A problem over the group's DOFs within their limits, columns in `q`
   * order: for differential steps (`kinematicskit.native.DifferentialIk`) and
   * other solves the group's own `solve` does not cover.
   */
  public function problem():KinematicProblem return limitedProblem();

  /** A model state holding `q` on the group's DOFs (the rest at their defaults). */
  public function stateOf(q:Array<Float>):KinematicState {
    if (q == null || q.length != dofs.length)
      throw 'Kinematic group requires ${dofs.length} joint values, got ${q == null ? 0 : q.length}';
    var result = new KinematicState(model);
    for (i in 0...dofs.length) result.q[dofs[i]] = q[i];
    return result;
  }

  /** The tool centre point's twist task, in the reference frame (`FrameVelocityTask.setTwist` sets it). */
  public function toolVelocityTask():FrameVelocityTask {
    var task = FrameVelocityTask.atFrame(model, flangeFrameIndex, RobotKinematics.toTransform(flangeTTcp));
    return workFrame == null ? task.relativeTo(model, rootBody) : task.relativeTo(model, referenceBody, referenceOffset);
  }

  /** Position targets for every group DOF's driving joint, indexed by its compiled RobotModel position. */
  public function toJointTargets(q:Array<Float>):Array<JointTarget> {
    if (q == null || q.length != jointModelIndices.length)
      throw 'Kinematic group requires ${jointModelIndices.length} joint values, got ${q == null ? 0 : q.length}';
    return [for (i in 0...q.length) JointTarget.position(jointModelIndices[i], q[i])];
  }

  function run(problem:KinematicProblem, seed:KinematicState, o:IkOptions, method:IkMethod,
      workspace:SolverWorkspace):kinematicskit.KinematicSolution
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
  function solveWithBase(target:Transform3, start:Array<Float>, o:IkOptions, d:KinematicGroupData):IKResult {
    if (!(o.baseCost >= 0.0)) throw "Base cost must be non-negative";
    var root = model.bodyRoot[rootBody];
    var problem = limitedProblem();
    // The tool task first, so the result reports it.
    problem.add(FrameTask.atFrame(model, flangeFrameIndex, RobotKinematics.toTransform(target), o.positionTolerance,
      o.orientationTolerance, o.atFlange ? null : RobotKinematics.toTransform(flangeTTcp),
      o.positionAxes, o.orientation));
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
    if (o.orientationPreference != null) problem.add(orientationPreferenceTask(target, o, d, true));
    var solution = o.orientationPreference == null
      ? LevenbergMarquardt.solve(problem, seedState, o.maxIterations, 1e-3, 1e-8, 1.0, d.workspace)
      : PrioritizedSolver.solve(problem, seedState, o.maxIterations, o.damping > 0 ? o.damping : 0.02, 1e-8, d.workspace);
    var tip = solution.tasks[0];
    return new IKResult(solution.status == KinematicStatus.Converged, [for (dof in dofs) solution.state.q[dof]],
      tip.positionError, tip.orientationError, solution.iterations, solution.status,
      RobotKinematics.toTransform3(solution.state.rootPose(root)));
  }

  /**
   * The tool frame at `target` in the reference frame. A work frame moves
   * with the solve, so the task follows it; the root link does not, so its
   * pose (from the snapshot in `d`) turns the target into a world target.
   */
  function toolTask(target:Transform, o:IkOptions, d:KinematicGroupData):FrameTask {
    var offset = o.atFlange ? null : RobotKinematics.toTransform(flangeTTcp);
    if (workFrame == null)
      return FrameTask.atFrame(model, flangeFrameIndex, referencePose(d).compose(target), o.positionTolerance,
        o.orientationTolerance, offset, o.positionAxes, o.orientation);
    return FrameTask.atFrame(model, flangeFrameIndex, target, o.positionTolerance, o.orientationTolerance, offset,
      o.positionAxes, o.orientation)
      .relativeTo(model, referenceBody, referenceOffset);
  }

  function orientationPreferenceTask(target:Transform3, o:IkOptions, d:KinematicGroupData,
      ?world:Bool = false):FrameTask {
    var preferred = new Transform3(target.translation, o.orientationPreference);
    var soft = o.copy().freedom(kinematicskit.FrameOrientation.Full, 0);
    var task = world ? FrameTask.atFrame(model, flangeFrameIndex, RobotKinematics.toTransform(preferred),
      o.positionTolerance, o.orientationTolerance, o.atFlange ? null : RobotKinematics.toTransform(flangeTTcp),
      0, kinematicskit.FrameOrientation.Full) : toolTask(RobotKinematics.toTransform(preferred), soft, d);
    task.orientationWeight = o.orientationPreferenceWeight;
    return task.asPreference();
  }

  function hasExternal():Bool {
    for (value in external) if (value) return true;
    return false;
  }

  function swivelTask(target:Float, tolerance:Float, soft:Bool):SwivelTask
    return new SwivelTask(model, swivelBodies[0], swivelPoints[0], swivelBodies[1], swivelPoints[1], swivelBodies[2],
      swivelPoints[2], target, tolerance, swivelReference, soft, "swivel");

  /** A problem over the group's DOFs (but `excluded` ones) within their limits. */
  function limitedProblem(?excluded:Array<Bool>):KinematicProblem {
    var problem = new KinematicProblem(model)
      .setActiveDofs(excluded == null ? dofs : [for (i in 0...dofs.length) if (!excluded[i]) dofs[i]]);
    for (i in 0...dofs.length) {
      var limits = group.limitsOf(i);
      if (limits.lower < limits.upper) problem.setLimits(dofs[i], limits.lower, limits.upper);
      else problem.setLimits(dofs[i], Math.NEGATIVE_INFINITY, Math.POSITIVE_INFINITY);
    }
    return problem;
  }

  function referencePose(d:KinematicGroupData):Transform
    return d.snapshot.bodyPose(referenceBody).compose(referenceOffset);

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

  function dataFor(data:Null<KinematicGroupData>):KinematicGroupData {
    if (data == null) return threadLocalData();
    if (data.group != this) throw "Kinematic group data belongs to another group";
    return data;
  }

  /** `q` evaluated in `data` (this thread's by default), which it returns. */
  function evaluate(q:Array<Float>, data:Null<KinematicGroupData>):KinematicGroupData {
    if (q == null || q.length != dofs.length)
      throw 'Kinematic group requires ${dofs.length} joint values, got ${q == null ? 0 : q.length}';
    var d = dataFor(data);
    for (i in 0...dofs.length) d.state.q[dofs[i]] = q[i];
    d.snapshot.evaluate(d.state);
    return d;
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
