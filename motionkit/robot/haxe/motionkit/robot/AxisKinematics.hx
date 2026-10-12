package motionkit.robot;

import motionkit.path.OrientationPolicy;

import motionkit.axis.MotionAxis;
import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.AffineJointMap;
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
  /**
    The joints no axis owns that follow other joints (a CoreXY motor's pulley follows both axes): each is the sum
    of its couplings' terms, placed after the joints it follows.
  **/
  final derived:Array<Int> = [];
  final terms:Array<Array<{leader:Int, ratio:Float, offset:Float}>> = [];
  /** Every coupling between joints of the model, by index, for `solvePose`'s check that followers keep their ratio. */
  final couplingLeaders:Array<Int> = [];
  final couplingFollowers:Array<Int> = [];
  final couplingRatios:Array<Float> = [];
  final couplingOffsets:Array<Float> = [];

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
    var ids = [for (joint in blueprint.model.joints) joint.id];
    for (_ in ids) terms.push([]);
    for (coupling in blueprint.model.couplings) {
      var leader = ids.indexOf(coupling.leader), follower = ids.indexOf(coupling.follower);
      if (leader >= 0 && follower >= 0) terms[follower].push({leader: leader, ratio: coupling.ratio, offset: coupling.offset});
    }
    for (index in 0...ids.length) if (terms[index].length > 0 && !used.exists(index)) derived.push(index);
    // Each derived joint after the derived joints it follows, so one pass completes them all.
    var placed:Array<Int> = [];
    while (placed.length < derived.length) {
      var before = placed.length;
      for (index in derived) {
        if (placed.indexOf(index) >= 0) continue;
        var ready = true;
        for (term in terms[index]) if (derived.indexOf(term.leader) >= 0 && placed.indexOf(term.leader) < 0) ready = false;
        if (ready) placed.push(index);
      }
      if (placed.length == before) { for (index in derived) if (placed.indexOf(index) < 0) placed.push(index); break; }
    }
    derived.splice(0, derived.length);
    for (index in placed) derived.push(index);
    for (coupling in blueprint.model.couplings) {
      var leader = ids.indexOf(coupling.leader), follower = ids.indexOf(coupling.follower);
      if (leader >= 0 && follower >= 0) {
        couplingLeaders.push(leader); couplingFollowers.push(follower);
        couplingRatios.push(coupling.ratio); couplingOffsets.push(coupling.offset);
      }
    }
    home = [for (_ in names) 0.0];
    x.writeLogicalPosition(home, x.homePosition);
    y.writeLogicalPosition(home, y.homePosition);
    z.writeLogicalPosition(home, z.homePosition);
  }

  public function jointCount():Int return blueprint.model.joints.length;

  var affine:Null<AffineJointMap> = null;

  /**
    The joints are exactly affine in the logical axes: each axis scales and offsets its joints and the followers sum
    their terms. Read off by placing the tool at the origin and a metre along each axis, limits aside (a joint no axis
    owns and nothing drives, an extruder, stays zero).
  **/
  public function affineMap():Null<AffineJointMap> {
    if (affine != null) return affine;
    var count = jointCount();
    function placed(position:Array<Float>):Array<Float> {
      var joints = [for (_ in 0...count) 0.0];
      for (index in 0...3) [x, y, z][index].writeLogicalPosition(joints, position[index]);
      completeDerived(joints, true);
      return joints;
    }
    var origin = placed([0.0, 0.0, 0.0]);
    var columns = [for (axis in 0...3) {
      var along = [0.0, 0.0, 0.0];
      along[axis] = 1.0;
      var moved = placed(along);
      [for (joint in 0...count) moved[joint] - origin[joint]];
    }];
    return affine = new AffineJointMap(origin, columns);
  }

  public function forward(q:Array<Float>):Pose3 {
    requireJoints(q);
    return new Pose3(x.logicalPosition(q), y.logicalPosition(q), z.logicalPosition(q));
  }

  public function solvePose(target:Pose3, seed:Array<Float>,
      tolerance:IkTolerance, ?freedom:OrientationPolicy):Null<Array<Float>> {
    if (target == null || tolerance == null) throw "Axis IK needs a pose and tolerance";
    requireJoints(seed);
    if (ToolFreedom.orientationError(new Pose3(0, 0, 0), target, freedom == null ? OrientationPolicy.Interpolated : freedom) > tolerance.orientation)
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
    completeDerived(result, true);
    // A follower is the sum of its couplings' terms.
    var sums = [for (_ in 0...result.length) 0.0];
    var summed = [for (_ in 0...result.length) false];
    for (coupling in 0...couplingLeaders.length) {
      var follower = couplingFollowers[coupling];
      sums[follower] += couplingRatios[coupling] * result[couplingLeaders[coupling]] + couplingOffsets[coupling];
      summed[follower] = true;
    }
    for (index in 0...result.length)
      if (summed[index] && Math.abs(result[index] - sums[index]) > tolerance.position) return null;
    return result;
  }

  /** Logical axes have one solution per pose: the path follows point by point. */
  public function solvePath(request:PathRequest):Array<Null<Array<Float>>> return request.followPointByPoint(this);

  public function sampleCandidates(target:Pose3, maxCount:Int,
      tolerance:IkTolerance, ?freedom:OrientationPolicy):Array<Array<Float>> {
    if (maxCount < 0) throw "Axis IK candidate count must be non-negative";
    if (maxCount == 0) return [];
    var solution = solvePose(target, home, tolerance, freedom);
    return solution == null ? [] : [solution];
  }

  /** Nothing here changes once built. */
  public function fork():KinematicsSolver return this;

  public function solveDifferential(q:Array<Float>, twist:Twist6, ?redundancyRate:Array<Float>, ?freedom:OrientationPolicy):Null<Array<Float>> {
    requireJoints(q);
    if (twist == null) throw "Axis differential IK needs a tool twist";
    var asked = twist.toArray();
    switch freedom {
      case Cone(_, _): for (i in 3...6) asked[i] = 0.0;
      default:
    }
    for (row in ToolFreedom.twistRows(forward(q), freedom)) {
      var angular = 0.0;
      for (axis in 3...6) angular += row[axis]*asked[axis];
      if (Math.abs(angular) > 1e-12) return null;
    }
    var result = [for (_ in 0...jointCount()) 0.0];
    x.writeLogicalDelta(result, twist.linearX);
    y.writeLogicalDelta(result, twist.linearY);
    z.writeLogicalDelta(result, twist.linearZ);
    completeDerived(result, false);
    return result;
  }

  /**
    Sets every joint no axis owns that follows others to the sum of its terms: positions with the terms' offsets, rates
    (`positions` false) without. They are held in dependency order, so one pass puts each after what it follows. A driven joint (an extruder) is set
    first, and its followers (the motor behind its gears) come out here.
  **/
  public function completeDerived(q:Array<Float>, positions:Bool):Void {
    for (index in derived) {
      var sum = 0.0;
      for (term in terms[index]) sum += term.ratio * q[term.leader] + (positions ? term.offset : 0.0);
      q[index] = sum;
    }
  }

  function requireJoints(q:Array<Float>):Void {
    if (q == null || q.length != jointCount())
      throw 'Axis kinematics needs ${jointCount()} joint positions';
    for (value in q) if (!Math.isFinite(value)) throw "Axis joints must be finite";
  }
}
