package robotkit.manipulation;

import kinematicskit.DampedLeastSquares;
import kinematicskit.DofDampingTask;
import kinematicskit.FrameTask;
import kinematicskit.JacobianLayout;
import kinematicskit.KinematicModel;
import kinematicskit.KinematicProblem;
import kinematicskit.KinematicSnapshot;
import kinematicskit.KinematicState;
import kinematicskit.PostureTask;
import kinematicskit.KinematicStatus;
import kinematicskit.SolverWorkspace;
import robotkit.kinematics.RobotKinematics;
import robotkit.model.Frame;
import robotkit.model.FrameId;
import robotkit.model.Joint;
import robotkit.model.JointId;
import robotkit.model.LinkId;
import robotkit.model.RobotModel;
import robotkit.spatial.Transform3;
import robotkit.world.JointTarget;

/**
 * An arm and the external axes that move with it, solved as one group:
 * the joints from `rootLink` to the flange (a rail under the arm, then the
 * arm) and from `rootLink` to the work frame (a positioner carrying the
 * workpiece). Tool poses are expressed in the work frame, so a path on the
 * workpiece stays on it however the positioner turns it.
 *
 * All the group's joints belong to one robot model and so to one runtime:
 * a plan over them executes on one clock, which is what keeps the arm and
 * its external axes synchronized.
 *
 * `q` holds the flange path's DOFs (base first), then the work path's
 * (those not shared with the flange path). Solves prefer arm motion: each
 * external axis (the work path's joints, plus any named `externalAxes`)
 * carries a `DofDampingTask` weight, so it moves for what the arm cannot do
 * alone. That weight shapes the steps, never the answer: tool targets are
 * met exactly.
 *
 * With a `preferredPosture`, the arm is also drawn towards it (a soft pull
 * on the non-external DOFs), so the external axes bring the work to a
 * comfortable arm rather than the arm stretching after the work: a
 * positioner turns the workpiece round as the tool goes round it. The pull
 * shapes the configuration only; a second pass without it meets the tool
 * target exactly.
 */
class CoordinatedGroup {
  public final robot:RobotModel;
  public final model:KinematicModel;
  public final rootLink:LinkId;
  public final flangeFrame:FrameId;
  public final workFrame:FrameId;
  public final flangeTTcp:Transform3;
  /** The group DOFs' driving joints and their limits, in `q` order. */
  public final group:JointGroup;
  /** True for each group DOF that is an external axis (damped), in `q` order. */
  public final external:Array<Bool>;
  /** Arm posture the solves are drawn towards (`q` order; external entries ignored); null for none. */
  public var preferredPosture:Null<Array<Float>> = null;
  /** Strength of the pull towards `preferredPosture`, per radian (or metre) away. */
  public var postureWeight:Float = 0.05;

  final dofs:Array<Int>;
  final jointModelIndices:Array<Int>;
  final flangeFrameIndex:Int;
  final workFrameIndex:Int;
  final damping:Array<Float>;
  final state:KinematicState;
  final snapshot:KinematicSnapshot;
  final layout:JacobianLayout;
  final workspace = new SolverWorkspace();

  /**
   * `externalAxes` names further joints on the flange path to treat as
   * external (a rail under the arm); the work path's joints always are.
   * `externalWeight` is their step damping, in the units of the tool task
   * (1 m of tool motion weighs like 1 rad).
   */
  public function new(robot:RobotModel, rootLink:LinkId, flangeFrame:FrameId, workFrame:FrameId,
      ?flangeTTcp:Transform3, ?externalAxes:Array<JointId>, ?externalWeight:Float = 0.3, ?compiled:KinematicModel) {
    if (robot == null) throw "Coordinated group requires a robot model";
    if (!(externalWeight >= 0.0) || !Math.isFinite(externalWeight)) throw "External axis weight must be non-negative";
    var flange = frameOf(robot, flangeFrame), work = frameOf(robot, workFrame);
    this.robot = robot;
    this.model = compiled == null ? RobotKinematics.compile(robot) : compiled;
    this.rootLink = rootLink;
    this.flangeFrame = flangeFrame;
    this.workFrame = workFrame;
    this.flangeTTcp = flangeTTcp == null ? Transform3.identity() : flangeTTcp;
    if (model.bodyIndex(rootLink) < 0) throw 'Coordinated group root link "$rootLink" is not part of the model';
    flangeFrameIndex = model.frameIndex(flangeFrame);
    workFrameIndex = model.frameIndex(workFrame);

    var flangePath = Manipulator.walk(robot, rootLink, flange.link.id);
    var workPath = Manipulator.walk(robot, rootLink, work.link.id);
    dofs = [];
    external = [];
    var named = externalAxes == null ? [] : externalAxes;
    for (joint in flangePath) addDof(joint, named.indexOf(joint.id) >= 0);
    for (joint in workPath) addDof(joint, true);
    for (id in named) {
      var found = false;
      for (joint in flangePath) if (joint.id == id) found = true;
      if (!found) throw 'External axis "$id" is not on the path to the flange';
    }
    if (dofs.length == 0) throw "Coordinated group has no movable joints";
    var drivers:Array<Joint> = [for (dof in dofs) findJoint(robot, model.dofId(dof))];
    group = new JointGroup([for (joint in drivers) joint.id], [for (joint in drivers) joint.limits]);
    jointModelIndices = [for (joint in drivers) robot.joints.indexOf(joint)];
    damping = [for (_ in 0...model.dofCount()) 0.0];
    for (i in 0...dofs.length) if (external[i]) damping[dofs[i]] = externalWeight;
    state = new KinematicState(model);
    snapshot = new KinematicSnapshot(model);
    layout = new JacobianLayout(model, dofs);
  }

  public function dofCount():Int return dofs.length;

  public function jointIds():Array<JointId> return group.jointIds.copy();

  /** The runtime (compiled `RobotModel.joints`) index of each group DOF's driving joint, in `q` order. */
  public function jointIndices():Array<Int> return jointModelIndices.copy();

  /** The work frame's pose in the root link's frame. */
  public function workPose(q:Array<Float>):Transform3 {
    evaluate(q);
    return RobotKinematics.toTransform3(inRoot(snapshot.framePose(workFrameIndex)));
  }

  /** `work_T_tcp`: the tool centre point in the work frame. */
  public function toolInWork(q:Array<Float>):Transform3 {
    evaluate(q);
    var tcp = snapshot.framePose(flangeFrameIndex).compose(RobotKinematics.toTransform(flangeTTcp));
    return RobotKinematics.toTransform3(snapshot.framePose(workFrameIndex).inverse().compose(tcp));
  }

  /**
   * IK for a tool-centre-point target in the work frame (`work_T_tcp`), over
   * every group joint within its limits, external axes damped. Reports
   * non-convergence, never throws.
   */
  public function solveIk(target:Transform3, seed:Array<Float>, ?positionTolerance:Float = 1e-4,
      ?orientationTolerance:Float = 1e-3, ?maxIterations:Int = 200, ?dampingFactor:Float = 0.02):IKResult {
    if (target == null) throw "Coordinated inverse kinematics requires a target";
    var n = dofs.length;
    var start = seed == null ? [for (_ in 0...n) 0.0] : seed;
    if (start.length != n) throw 'Coordinated group requires $n values, got ${start.length}';
    var problem = new KinematicProblem(model).setActiveDofs(dofs);
    for (i in 0...n) {
      var limits = group.limitsOf(i);
      if (limits.lower < limits.upper) problem.setLimits(dofs[i], limits.lower, limits.upper);
      else problem.setLimits(dofs[i], Math.NEGATIVE_INFINITY, Math.POSITIVE_INFINITY);
    }
    problem.add(toolTask(RobotKinematics.toTransform(target), positionTolerance, orientationTolerance));
    problem.add(new DofDampingTask(model, damping));
    for (i in 0...n) state.q[dofs[i]] = start[i];
    var seedState = state;
    if (preferredPosture != null) {
      var posture:Array<Float> = preferredPosture;
      if (posture.length != n) throw 'Preferred posture needs $n values';
      // First draw the arm towards its posture; the external axes take up the rest.
      var targets = [for (_ in 0...model.dofCount()) 0.0], weights = [for (_ in 0...model.dofCount()) 0.0];
      for (i in 0...n) if (!external[i]) {
        targets[dofs[i]] = posture[i];
        weights[dofs[i]] = 1.0;
      }
      var drawn = new KinematicProblem(model).setActiveDofs(dofs);
      for (i in 0...n) drawn.setLimits(dofs[i], problem.lower[dofs[i]], problem.upper[dofs[i]]);
      drawn.add(toolTask(RobotKinematics.toTransform(target), positionTolerance, orientationTolerance));
      drawn.add(new DofDampingTask(model, damping));
      drawn.add(new PostureTask(model, targets, postureWeight, weights));
      seedState = DampedLeastSquares.solve(drawn, state, maxIterations, dampingFactor, 1e-8, workspace).state;
    }
    var solution = DampedLeastSquares.solve(problem, seedState, maxIterations, dampingFactor, 1e-8, workspace);
    var tip = solution.tasks[0];
    return new IKResult(solution.status == KinematicStatus.Converged, [for (dof in dofs) solution.state.q[dof]],
      tip.positionError, tip.orientationError, solution.iterations, solution.status);
  }

  /**
   * The tool's velocity relative to the workpiece per unit of each group
   * DOF: 6 x n rows (linear then angular) in the work frame, at `q`.
   */
  public function relativeJacobian(q:Array<Float>):Array<Float> {
    evaluate(q);
    var here = snapshot.framePose(workFrameIndex).inverse()
      .compose(snapshot.framePose(flangeFrameIndex).compose(RobotKinematics.toTransform(flangeTTcp)));
    var task = toolTask(here, 1.0, 1.0);
    var n = dofs.length;
    var residual = [for (_ in 0...6) 0.0], rows = [for (_ in 0...6 * n) 0.0];
    task.evaluate(state, snapshot, layout, residual, rows, 0);
    // Rows are in world coordinates; express them in the work frame.
    var work = snapshot.framePose(workFrameIndex);
    var inverse = new kinematicskit.Transform(0, 0, 0, -work.qx, -work.qy, -work.qz, work.qw);
    for (block in 0...2) for (c in 0...n) {
      var v = inverse.compose(kinematicskit.Transform.translation(rows[(3 * block) * n + c], rows[(3 * block + 1) * n + c],
        rows[(3 * block + 2) * n + c]));
      rows[(3 * block) * n + c] = v.x;
      rows[(3 * block + 1) * n + c] = v.y;
      rows[(3 * block + 2) * n + c] = v.z;
    }
    return rows;
  }

  /** Position targets for every group DOF's driving joint, indexed by its compiled RobotModel position. */
  public function toJointTargets(q:Array<Float>):Array<JointTarget> {
    if (q == null || q.length != jointModelIndices.length)
      throw 'Coordinated group requires ${jointModelIndices.length} joint values, got ${q == null ? 0 : q.length}';
    return [for (i in 0...q.length) JointTarget.position(jointModelIndices[i], q[i])];
  }

  function toolTask(target:kinematicskit.Transform, positionTolerance:Float, orientationTolerance:Float):FrameTask
    return FrameTask.atFrame(model, flangeFrameIndex, target, positionTolerance, orientationTolerance,
      RobotKinematics.toTransform(flangeTTcp)).relativeTo(model, model.frameBody[workFrameIndex],
      model.frameTransform(workFrameIndex));

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
      throw 'Coordinated group requires ${dofs.length} joint values, got ${q == null ? 0 : q.length}';
    for (i in 0...dofs.length) state.q[dofs[i]] = q[i];
    snapshot.evaluate(state);
  }

  function inRoot(world:kinematicskit.Transform):kinematicskit.Transform
    return snapshot.bodyPose(model.bodyIndex(rootLink)).inverse().compose(world);

  static function frameOf(robot:RobotModel, id:FrameId):Frame {
    for (candidate in robot.frames) if (candidate != null && candidate.id == id && candidate.link != null) return candidate;
    throw 'Coordinated group frame "$id" is not part of the model';
  }

  static function findJoint(robot:RobotModel, id:JointId):Joint {
    for (joint in robot.joints) if (joint != null && joint.id == id) return joint;
    throw 'Coordinated group joint "$id" is not part of the model';
  }
}
