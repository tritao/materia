package kinematicskit;

/** DOF values (in `KinematicModel.dofJoint` order) plus optional root-pose overrides. */
class KinematicState {
  public final model:KinematicModel;
  public final q:Array<Float>;
  final rootPoses:Array<Null<Transform>>;

  /** Starts every DOF at its driving joint's default value and every root at its model pose. */
  public function new(model:KinematicModel, ?q:Array<Float>) {
    if (model == null) throw "Kinematic state requires a model";
    this.model = model;
    if (q == null) this.q = [for (dof in 0...model.dofCount()) model.jointDefault[model.dofJoint[dof]]];
    else {
      if (q.length != model.dofCount())
        throw 'Kinematic state requires ${model.dofCount()} values, got ${q.length}';
      this.q = q.copy();
    }
    rootPoses = [for (_ in 0...model.bodyCount()) null];
  }

  public function setRootPose(body:Int, pose:Transform):Void {
    if (body < 0 || body >= model.bodyCount() || model.bodyParentJoint[body] >= 0)
      throw 'Kinematic body $body is not a root';
    rootPoses[body] = Transform.checked(pose, "Kinematic root pose");
  }

  /** The root's pose in this state: its override, or the model's pose. */
  public function rootPose(body:Int):Transform {
    var pose = rootPoses[body];
    return pose == null ? model.bodyRootPoses[body] : pose;
  }

  public function copy():KinematicState {
    var result = new KinematicState(model, q);
    for (body in 0...rootPoses.length) result.rootPoses[body] = rootPoses[body];
    return result;
  }
}
