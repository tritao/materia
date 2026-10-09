package kinematicskit;

/**
 * A configuration-independent motion bound for a model's bodies
 * (COLLISION.md CL-D5, CL7): over any motion in which DOF k travels at most
 * `travel[k]`, no point of body b within `radius` of its frame moves further
 * than Σ_k (distance[b][k] + angular[b][k]·radius)·travel[k].
 *
 * For a revolute joint j between the root and b, the point's distance from
 * j's axis is at most the sum of the offsets from j to b (every joint on the
 * way, fixed or movable, with a prismatic joint's largest travel) plus the
 * point's radius about b's frame, whatever the configuration; a prismatic
 * joint moves it at most its own travel. Coupled joints weigh each DOF by
 * the absolute scale of its term.
 */
class MotionReach {
  public final model:KinematicModel;
  /** Per body, per DOF: the offsets' share (row-major, bodies x DOFs). */
  public final distance:Array<Float>;
  /** Per body, per DOF: the radius's share (revolute joints' absolute scales). */
  public final angular:Array<Float>;

  public function new(model:KinematicModel) {
    this.model = model;
    var n = model.dofCount();
    distance = [for (_ in 0...model.bodyCount() * n) 0.0];
    angular = [for (_ in 0...model.bodyCount() * n) 0.0];
    for (body in 0...model.bodyCount()) {
      // The joints from the body up to its root, nearest first, with the distance from each joint to the body.
      var reach = 0.0;
      var current = body;
      while (model.bodyParentJoint[current] >= 0) {
        var joint = model.bodyParentJoint[current];
        reach += length(model.jointJointTChild, joint);
        if (model.jointKind[joint] == JointKind.Revolute) {
          for (term in model.jointTermStart[joint]...model.jointTermStart[joint + 1]) {
            var column = body * n + model.jointTermDof[term], scale = Math.abs(model.jointTermScale[term]);
            distance[column] += scale * reach;
            angular[column] += scale;
          }
        } else if (model.jointKind[joint] == JointKind.Prismatic) {
          for (term in model.jointTermStart[joint]...model.jointTermStart[joint + 1])
            distance[body * n + model.jointTermDof[term]] += Math.abs(model.jointTermScale[term]);
          reach += Math.max(Math.abs(model.jointLower[joint]), Math.abs(model.jointUpper[joint]));
        }
        reach += length(model.jointParentTJoint, joint);
        current = model.jointParent[joint];
      }
    }
  }

  /** The bound for a body's points within `radius` of its frame, over per-DOF `travel`. */
  public function bound(body:Int, radius:Float, travel:Array<Float>):Float {
    var n = model.dofCount(), sum = 0.0;
    for (dof in 0...n) {
      var t = travel[dof];
      if (t == 0) continue;
      sum += (distance[body * n + dof] + angular[body * n + dof] * radius) * t;
    }
    return sum;
  }

  static function length(transforms:Array<Float>, joint:Int):Float {
    var o = joint * 7;
    return Math.sqrt(transforms[o] * transforms[o] + transforms[o + 1] * transforms[o + 1] + transforms[o + 2] * transforms[o + 2]);
  }
}
