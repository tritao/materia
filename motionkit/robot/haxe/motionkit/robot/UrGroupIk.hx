package motionkit.robot;

import motionkit.robot.AnalyticIk.AnalyticBranch;
import motionkit.kinematics.Pose3;
import motionkit.path.OrientationPolicy;
import robotkit.manipulation.KinematicGroup;
import robotkit.manipulation.Manipulator;
import robotkit.model.LinkId;
import robotkit.spatial.Transform3;
import robotkit.spatial.Quat;
import robotkit.spatial.Vec3;

/** An UR6R arm with its upstream and workpiece axes held at a lattice cell. */
class UrGroupIk implements AnalyticIk {
  public final group:KinematicGroup;
  public final arm:UrAnalyticIk;
  public final armIndices:Array<Int>;
  public final armRoot:LinkId;

  public function new(group:KinematicGroup, geometryTolerance:Float = 1e-6) {
    this.group = group;
    armIndices = [for (index in 0...group.group.count()) if (!group.external[index]) index];
    if (armIndices.length != 6) throw "UR6R group requires six arm joints after separating external axes";
    var first = group.group.jointIds[armIndices[0]];
    var root:Null<LinkId> = null;
    for (joint in group.pathJoints()) if (joint.id == first) { root = joint.parent.id; break; }
    if (root == null) throw "UR6R group cannot locate its arm root";
    armRoot = cast root;
    var chain = new Manipulator(group.robot, armRoot, group.flangeFrame, group.flangeTTcp,
      group.model, null, group.profile);
    if (chain.group.count() != 6) throw "UR6R external joints must be upstream of the six-joint arm";
    arm = new UrAnalyticIk(chain, geometryTolerance);
    for (index in 0...6) if (chain.group.jointIds[index] != group.group.jointIds[armIndices[index]])
      throw "UR6R group arm ordering does not match the model chain";
  }

  public function family():String return "UR6R";
  public function jointCount():Int return group.group.count();
  public function armBase(q:Array<Float>):Transform3 return group.linkPoses(q, [armRoot])[0];
  public function forward(q:Array<Float>):Pose3 {
    if (q == null || q.length != jointCount()) throw "UR6R group FK needs complete joints";
    var local = arm.forward([for (index in armIndices) q[index]]);
    var pose = armBase(q).compose(new Transform3(new Vec3(local.x, local.y, local.z),
      new Quat(local.qx, local.qy, local.qz, local.qw)));
    var p = pose.translation, r = pose.rotation;
    return new Pose3(p.x, p.y, p.z, r.x, r.y, r.z, r.w);
  }

  public function branches(target:Pose3, seed:Array<Float>, ?freedom:OrientationPolicy):Array<AnalyticBranch> {
    if (target == null || seed == null || seed.length != jointCount())
      throw "UR6R group branch enumeration needs a target and complete lattice cell";
    for (index in 0...seed.length) if (group.external[index]) {
      var bounds = group.group.limitsOf(index);
      if (bounds.lower < bounds.upper && (seed[index] < bounds.lower - 1e-9 || seed[index] > bounds.upper + 1e-9)) return [];
    }
    var requested = new Transform3(new Vec3(target.x, target.y, target.z),
      new Quat(target.qx, target.qy, target.qz, target.qw));
    var local = armBase(seed).inverse().compose(requested), p = local.translation, r = local.rotation;
    var pose = new Pose3(p.x, p.y, p.z, r.x, r.y, r.z, r.w);
    var armSeed = [for (index in armIndices) seed[index]];
    var answers:Array<AnalyticBranch> = [];
    for (branch in arm.branches(pose, armSeed, freedom)) {
      var q = seed.copy();
      for (index in 0...6) q[armIndices[index]] = branch.q[index];
      answers.push(new AnalyticBranch(q, branch.branch, branch.singular,branch.singularityKnown,branch.configuration));
    }
    return answers;
  }
}
