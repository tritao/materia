package motionkit.robot;

import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.Pose3;
import motionkit.path.OrientationPolicy;
import motionkit.robot.AnalyticIk.AnalyticBranch;
import robotkit.manipulation.KinematicGroup;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;

/** Explicit fallback for geometry without a supported analytic family.
 * Numeric branch IDs identify seeds, not persistent geometric branches.
 */
class NumericBranchIk implements AnalyticIk {
  public final group:KinematicGroup;
  public final diagnostic:String;
  final tolerance:IkTolerance;
  final fixedSeeds:Array<Array<Float>>;

  public function new(group:KinematicGroup, reason:String, ?tolerance:IkTolerance) {
    if (group == null || reason == null || StringTools.trim(reason).length == 0)
      throw "Numeric fallback requires a group and an unsupported-family diagnostic";
    this.group = group;
    diagnostic = "Unsupported analytic family: " + reason + "; using multi-seed numeric IK";
    this.tolerance = tolerance == null ? new IkTolerance() : tolerance;
    var centre = [for (i in 0...jointCount()) {
      var limits = group.group.limitsOf(i);
      Math.isFinite(limits.lower) && Math.isFinite(limits.upper) && limits.lower < limits.upper
        ? (limits.lower + limits.upper) * 0.5 : 0.0;
    }];
    fixedSeeds = [centre];
    for (i in 0...jointCount()) if (!group.external[i]) for (fraction in [0.2,0.8]) {
      var q = centre.copy(), limits = group.group.limitsOf(i);
      q[i] = Math.isFinite(limits.lower) && Math.isFinite(limits.upper) && limits.lower < limits.upper
        ? limits.lower + fraction * (limits.upper-limits.lower) : (fraction < 0.5 ? -Math.PI : Math.PI);
      fixedSeeds.push(q);
    }
  }
  public function family():String return "numeric-fallback";
  public function jointCount():Int return group.group.count();
  public function forward(q:Array<Float>):Pose3 {
    var t = group.tcpPose(q), p = t.translation, r = t.rotation;
    return new Pose3(p.x,p.y,p.z,r.x,r.y,r.z,r.w);
  }
  public function branches(target:Pose3, seed:Array<Float>, ?freedom:OrientationPolicy):Array<AnalyticBranch>
    return branchesFromNeighbours(target,[seed],seed,freedom);

  /** Neighbour seeds precede the deterministic fixed set; external coordinates stay held. */
  public function branchesFromNeighbours(target:Pose3, neighbours:Array<Array<Float>>, cell:Array<Float>,
      ?freedom:OrientationPolicy,includeFixedSeeds:Bool=true,?heldJoints:Array<Int>):Array<AnalyticBranch> {
    if (target == null || cell == null || cell.length != jointCount() || neighbours == null)
      throw "Numeric fallback requires a target, neighbours and complete lattice cell";
    var held = [for (i in 0...jointCount()) if (group.external[i]) i];
    if(heldJoints!=null)for(j in heldJoints){
      if(j<0 || j>=jointCount())throw "Numeric held joint lies outside the group";
      if(held.indexOf(j)<0)held.push(j);
    }
    for (i in 0...jointCount()) {
      if (!Math.isFinite(cell[i])) throw "Numeric fallback seed must be finite";
      if (group.external[i]) {
        var bounds = group.group.limitsOf(i);
        if (cell[i] < bounds.lower || cell[i] > bounds.upper) return [];
      }
    }
    var answers:Array<AnalyticBranch> = [], seeds = includeFixedSeeds ? neighbours.concat(fixedSeeds) : neighbours;
    var task = ToolFreedom.of(target,freedom,tolerance.orientation);
    for (index in 0...seeds.length) {
      if (seeds[index] == null || seeds[index].length != jointCount()) throw "Numeric fallback neighbour is incomplete";
      var q = seeds[index].copy();
      for (i in held) q[i] = cell[i];
      var options = task.options(tolerance);
      options.held = held; options.heldValues = [for (i in held) cell[i]];
      var solved = group.solve(new Transform3(new Vec3(task.target.x,task.target.y,task.target.z),
        new Quat(task.target.qx,task.target.qy,task.target.qz,task.target.qw)),q,options);
      if (!solved.converged) continue;
      var duplicate = false;
      for (answer in answers) {
        var distance = 0.0;
        for (i in 0...jointCount()) distance += Math.pow(answer.q[i]-solved.q[i],2);
        if (Math.sqrt(distance) < tolerance.candidateSeparation) duplicate = true;
      }
      if (!duplicate) answers.push(new AnalyticBranch(solved.q,index,false,false));
    }
    return answers;
  }
}
