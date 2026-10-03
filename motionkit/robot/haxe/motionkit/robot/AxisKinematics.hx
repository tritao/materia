package motionkit.robot;

import motionkit.axis.MotionAxis;
import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.KinematicsSolver;
import motionkit.kinematics.PathRequest;
import motionkit.kinematics.Pose3;
import motionkit.kinematics.Twist6;

/** Exact Cartesian kinematics for an XYZ machine's logical axis mapping. */
class AxisKinematics implements KinematicsSolver {
  public final blueprint:MotionSystemBlueprint;
  public final x:MotionAxis;
  public final y:MotionAxis;
  public final z:MotionAxis;
  final home:Array<Float>;

  public function new(blueprint:MotionSystemBlueprint,
      ?xAxisId:String = "x", ?yAxisId:String = "y", ?zAxisId:String = "z") {
    if (blueprint == null) throw "Axis kinematics needs a motion-system blueprint";
    this.blueprint = blueprint;
    var names = [for (joint in blueprint.model.joints) joint.name];
    var axes = [for (entry in blueprint.axes) new MotionAxis(entry, names)];
    function required(id:String):MotionAxis {
      for (axis in axes) if (axis.id == id) return axis;
      throw 'Axis kinematics requires logical axis "$id"';
    }
    x = required(xAxisId); y = required(yAxisId); z = required(zAxisId);
    var used = new Map<Int, Bool>();
    for (axis in [x, y, z]) for (joint in axis.jointIndices) {
      if (used.exists(joint))
        throw 'Axis kinematics maps joint $joint to more than one coordinate';
      used.set(joint, true);
    }
    home = [for (_ in names) 0.0];
    x.writeLogicalPosition(home, x.homePosition);
    y.writeLogicalPosition(home, y.homePosition);
    z.writeLogicalPosition(home, z.homePosition);
  }

  public function jointCount():Int return blueprint.model.joints.length;

  public function forward(q:Array<Float>):Pose3 {
    requireJoints(q);
    return new Pose3(x.logicalPosition(q), y.logicalPosition(q), z.logicalPosition(q));
  }

  public function solvePose(target:Pose3, seed:Array<Float>,
      tolerance:IkTolerance):Null<Array<Float>> {
    if (target == null || tolerance == null) throw "Axis IK needs a pose and tolerance";
    requireJoints(seed);
    if (2.0 * Math.acos(Math.min(1.0, Math.abs(target.qw))) > tolerance.orientation)
      return null;
    var logical = [target.x, target.y, target.z];
    var axes = [x, y, z];
    var result = seed.copy();
    for (index in 0...3) {
      var axis = axes[index];
      if (logical[index] < axis.lowerLimit - tolerance.position ||
          logical[index] > axis.upperLimit + tolerance.position) return null;
      axis.writeLogicalPosition(result, logical[index]);
    }
    for (index in 0...result.length) {
      var bounds = blueprint.model.joints[index].limits;
      if (bounds.lower < bounds.upper &&
          (result[index] < bounds.lower - tolerance.position ||
           result[index] > bounds.upper + tolerance.position)) return null;
    }
    // A follower is the sum of its couplings' terms.
    var sums = new Map<String, Float>();
    for (coupling in blueprint.model.couplings) {
      var leader = -1, follower = -1;
      for (index in 0...blueprint.model.joints.length) {
        if (blueprint.model.joints[index].id == coupling.leader) leader = index;
        if (blueprint.model.joints[index].id == coupling.follower) follower = index;
      }
      if (leader < 0 || follower < 0) continue;
      var sum = sums.get(coupling.follower);
      sums.set(coupling.follower, (sum == null ? 0.0 : sum) + coupling.ratio * result[leader] + coupling.offset);
    }
    for (index in 0...blueprint.model.joints.length) {
      var sum = sums.get(blueprint.model.joints[index].id);
      if (sum != null && Math.abs(result[index] - sum) > tolerance.position) return null;
    }
    return result;
  }

  /** Logical axes have one solution per pose: the path follows point by point. */
  public function solvePath(request:PathRequest):Array<Null<Array<Float>>> return request.followPointByPoint(this);

  public function sampleCandidates(target:Pose3, maxCount:Int,
      tolerance:IkTolerance):Array<Array<Float>> {
    if (maxCount < 0) throw "Axis IK candidate count must be non-negative";
    if (maxCount == 0) return [];
    var solution = solvePose(target, home, tolerance);
    return solution == null ? [] : [solution];
  }

  /** Nothing here changes once built. */
  public function fork():KinematicsSolver return this;

  public function solveDifferential(q:Array<Float>, twist:Twist6, ?redundancyRate:Array<Float>):Null<Array<Float>> {
    requireJoints(q);
    if (twist == null) throw "Axis differential IK needs a tool twist";
    if (Math.abs(twist.angularX) > 1e-12 || Math.abs(twist.angularY) > 1e-12 ||
        Math.abs(twist.angularZ) > 1e-12) return null;
    var result = [for (_ in 0...jointCount()) 0.0];
    x.writeLogicalDelta(result, twist.linearX);
    y.writeLogicalDelta(result, twist.linearY);
    z.writeLogicalDelta(result, twist.linearZ);
    return result;
  }

  function requireJoints(q:Array<Float>):Void {
    if (q == null || q.length != jointCount())
      throw 'Axis kinematics needs ${jointCount()} joint positions';
    for (value in q) if (!Math.isFinite(value)) throw "Axis joints must be finite";
  }
}
