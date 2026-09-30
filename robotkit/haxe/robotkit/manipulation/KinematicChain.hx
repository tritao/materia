package robotkit.manipulation;

import kinematicskit.FrameTask;
import kinematicskit.KinematicModel;
import kinematicskit.KinematicSnapshot;
import kinematicskit.KinematicState;
import robotkit.kinematics.RobotKinematics;
import robotkit.model.RobotModel;
import robotkit.model.LinkId;
import robotkit.model.Frame;
import robotkit.model.FrameId;
import robotkit.model.Joint;
import robotkit.model.JointId;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;

/**
 * A serial kinematic chain from a base link to a tip (a link's own origin,
 * or a mounted Frame): a view over the `kinematicskit` model that
 * `RobotKinematics.path` compiles, walking `Joint`s exactly as
 * `RobotFrameTree2`/the native runtime interpret them (see
 * ARCHITECTURE.md, "Joint-frame convention"). Revolute, continuous, and
 * prismatic joints become degrees of freedom; fixed joints fold into
 * constant transforms; any other joint type is a construction error.
 */
class KinematicChain {
  public final baseLink:LinkId;
  public final tipLink:LinkId;
  public final tipFrame:Null<FrameId>;
  /** The compiled path, with `baseLink` as its identity root. */
  public final model:KinematicModel;

  final dofJoints:Array<Joint> = [];
  final tipBody:Int;
  final tipFrameIndex:Int;
  final state:KinematicState;
  final snapshot:KinematicSnapshot;
  /** Bodies in path order, base first (the order `allLinkTransforms` reports). */
  final pathBodies:Array<Int>;

  public function new(model:RobotModel, baseLinkId:LinkId, tip:ChainTip) {
    if (model == null) throw "Kinematic chain requires a robot model";
    this.baseLink = baseLinkId;
    var tipLinkId:LinkId;
    switch tip {
      case ChainTip.Link(id):
        tipLinkId = id;
        this.tipFrame = null;
      case ChainTip.Frame(id):
        var frame:Null<Frame> = null;
        for (candidate in model.frames) if (candidate != null && candidate.id == id) { frame = candidate; break; }
        if (frame == null || frame.link == null)
          throw 'Kinematic chain tip frame "$id" is not part of the model';
        tipLinkId = frame.link.id;
        this.tipFrame = id;
    }
    this.tipLink = tipLinkId;
    this.model = RobotKinematics.path(model, baseLinkId, tipLinkId);
    this.tipBody = this.model.bodyIndex(tipLinkId);
    this.tipFrameIndex = tipFrame == null ? -1 : this.model.frameIndex(tipFrame);
    for (dof in 0...this.model.dofCount()) {
      var id = this.model.dofId(dof);
      for (joint in model.joints) if (joint != null && joint.id == id) { dofJoints.push(joint); break; }
    }
    pathBodies = [for (body in this.model.bodyOrder) body];
    state = new KinematicState(this.model);
    snapshot = new KinematicSnapshot(this.model);
  }

  public function dofCount():Int return dofJoints.length;

  public function dofJointIds():Array<JointId> return [for (joint in dofJoints) joint.id];

  public function dofJointsCopy():Array<Joint> return dofJoints.copy();

  public function forwardKinematics(q:Array<Float>):Transform3 {
    evaluate(q);
    return RobotKinematics.toTransform3(tipFrameIndex >= 0 ? snapshot.framePose(tipFrameIndex) : snapshot.bodyPose(tipBody));
  }

  /** `base_T_link` for the base link and every link reached while walking to the tip (base link first). */
  public function allLinkTransforms(q:Array<Float>):Array<Transform3> {
    evaluate(q);
    return [for (body in pathBodies) RobotKinematics.toTransform3(snapshot.bodyPose(body))];
  }

  /** Geometric Jacobian, 6xn (rows 0..2 linear, rows 3..5 angular), expressed in the base frame. */
  public function jacobian(q:Array<Float>):Array<Array<Float>> {
    evaluate(q);
    var n = dofJoints.length;
    var flat = tipFrameIndex >= 0 ? snapshot.frameJacobian(tipFrameIndex) : snapshot.bodyJacobian(tipBody);
    return [for (row in 0...6) [for (column in 0...n) flat[row * n + column]]];
  }

  /** A task that moves the chain tip (its frame, or its link origin) to `target` in the base frame. */
  public function tipTask(target:Transform3, positionTolerance:Float, orientationTolerance:Float):FrameTask {
    var goal = RobotKinematics.toTransform(target);
    return tipFrameIndex >= 0
      ? FrameTask.atFrame(model, tipFrameIndex, goal, positionTolerance, orientationTolerance)
      : new FrameTask(model, tipBody, null, goal, positionTolerance, orientationTolerance);
  }

  /**
   * Geometric Jacobian (6 x n, row-major) of the point `tipTPoint` fixed in
   * the tip frame, e.g. a tool centre point: linear rows are that point's
   * velocity, angular rows the tip's angular velocity, in the base frame.
   */
  public function pointJacobian(q:Array<Float>, tipTPoint:Vec3):Array<Float> {
    evaluate(q);
    var tip = tipFrameIndex >= 0 ? snapshot.framePose(tipFrameIndex) : snapshot.bodyPose(tipBody);
    var point = tip.transformPoint(tipTPoint.x, tipTPoint.y, tipTPoint.z);
    return snapshot.pointJacobian(tipBody, point.x, point.y, point.z);
  }

  function evaluate(q:Array<Float>):Void {
    if (q == null || q.length != dofJoints.length)
      throw 'Kinematic chain requires ${dofJoints.length} joint values, got ${q == null ? 0 : q.length}';
    for (i in 0...q.length) state.q[i] = q[i];
    snapshot.evaluate(state);
  }
}
