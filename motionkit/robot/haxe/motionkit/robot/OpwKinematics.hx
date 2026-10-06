package motionkit.robot;

import motionkit.path.OrientationPolicy;
import motionkit.robot.AnalyticIk.AnalyticBranch;

import TrajectoryCore;
import MotionKitNative;
import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.KinematicsSolver;
import motionkit.kinematics.PathRequest;
import motionkit.kinematics.Pose3;
import motionkit.kinematics.Twist6;
import robotkit.manipulation.Manipulator;
import robotkit.model.Joint;
import robotkit.model.Frame;
import robotkit.model.JointType;
import robotkit.model.RobotModel;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;

/** Analytic 6R IK for a parallel-base, spherical-wrist RobotModel. */
class OpwKinematics implements KinematicsSolver implements AnalyticIk {
  public final parameters:OpwParameters;
  public final manipulator:Manipulator;
  public final geometryTolerance:Float;
  final base:Transform3;
  final flangeTTcp:Transform3;
  final native:mk_opw_parameters;
  final differential:ManipulatorKinematics;

  public function new(model:RobotModel, manipulator:Manipulator,
      ?geometryTolerance:Float = 1e-6) {
    if (model == null || manipulator == null) throw "OPW requires a model and manipulator";
    if (!Math.isFinite(geometryTolerance) || geometryTolerance <= 0.0)
      throw "OPW geometric tolerance must be positive and finite";
    this.manipulator = manipulator;
    this.geometryTolerance = geometryTolerance;
    differential = new ManipulatorKinematics(manipulator);
    var ordered = orderedJoints(model, manipulator);
    var current = Transform3.identity();
    var joints:Array<Joint> = [];
    var origins:Array<Vec3> = [];
    var axes:Array<Vec3> = [];
    for (joint in ordered) {
      var frame = current.compose(Transform3.fromArrays(joint.parentFramePosition,
        joint.parentFrameRotation));
      switch joint.type {
        case JointType.Revolute, JointType.Continuous:
          joints.push(joint);
          origins.push(frame.translation);
          axes.push(frame.transformVector(Vec3.fromArray(joint.axis)).normalized());
        case JointType.Fixed:
        case other: throw 'OPW joint ${joint.id} must be revolute or fixed, got $other';
      }
      current = frame.compose(Transform3.fromArrays(joint.childFramePosition,
        joint.childFrameRotation).inverse());
    }
    if (joints.length != 6) throw 'OPW requires six revolute joints, got ${joints.length}';
    var zero = [for (_ in 0...6) 0.0];
    var canonical = canonicalParameters(model, manipulator, joints, geometryTolerance);
    if (canonical != null) {
      base = Transform3.identity();
      parameters = canonical;
    } else {
    // Assembly models use the placed pose as q=0. Recover a canonical
    // straight-arm reference from axis lines, without assembly zero metadata.
    var reference = [for (_ in 0...6) 0.0];
    var geometry = geometryAt(ordered, reference);
    if (Math.abs(geometry.axes[3].dot(geometry.axes[2])) > geometryTolerance)
      throw 'OPW joint ${joints[3].id} is not perpendicular to the elbow axis';
    var targetZ = geometry.axes[0];
    reference[1] = alignmentAngle(geometry.origins[2].sub(geometry.origins[1]), targetZ, geometry.axes[1]);
    geometry = geometryAt(ordered, reference);
    reference[2] = alignmentAngle(geometry.axes[3], targetZ, geometry.axes[2]);
    geometry = geometryAt(ordered, reference);
    reference[3] = alignmentAngle(geometry.axes[4], geometry.axes[1], geometry.axes[3]);
    geometry = geometryAt(ordered, reference);
    reference[4] = alignmentAngle(geometry.axes[5], targetZ, geometry.axes[4]);
    geometry = geometryAt(ordered, reference);
    origins = geometry.origins;
    axes = geometry.axes;
    var z = axes[0];
    var y = axes[1];
    if (Math.abs(z.dot(y)) > geometryTolerance)
      throw 'OPW joint ${joints[1].id} is not perpendicular to the base axis';
    var x = y.cross(z).normalized();
    y = z.cross(x).normalized();
    base = new Transform3(origins[0], Quat.fromRotationMatrix([
      x.x, x.y, x.z, y.x, y.y, y.z, z.x, z.y, z.z]));
    var signs:Array<Int> = [];
    for (index in 0...6) {
      var expected = index == 0 || index >= 3 && index % 2 == 1 ? z : y;
      // The OPW axis sequence at zero is Z, Y, Y, Z, Y, Z.
      if (index == 5) expected = z;
      var alignment = axes[index].dot(expected);
      if (Math.abs(Math.abs(alignment) - 1.0) > geometryTolerance)
        throw 'OPW joint ${joints[index].id} violates the parallel-base/spherical-wrist axis pattern';
      signs.push(alignment < 0.0 ? -1 : 1);
    }
    var local = [for (origin in origins) base.inverse().transformPoint(origin)];
    var p2 = local[1], p3 = local[2].sub(p2);
    if (Math.abs(p3.x) > geometryTolerance)
      throw 'OPW joint ${joints[2].id} is not on the upper-arm axis';
    // Axis origins need not coincide: housings can be displaced along
    // their own axes. The three wrist axis lines must intersect instead.
    var wrist = new Vec3(local[3].x, local[3].y, local[4].z);
    if (Math.abs(local[4].x - wrist.x) > geometryTolerance ||
        Math.abs(local[5].x - wrist.x) > geometryTolerance ||
        Math.abs(local[5].y - wrist.y) > geometryTolerance)
      throw 'OPW wrist axis lines ${joints[3].id}, ${joints[4].id}, ${joints[5].id} do not intersect';
    var p4 = wrist.sub(local[2]);
    // c4 is the wrist-center-to-flange distance along the zero-pose tool Z.
    var tcpAtZero = base.inverse().compose(manipulator.tcpPose(zero));
    var flangeAtZero = base.inverse().compose(manipulator.forwardKinematics(reference));
    var c4 = flangeAtZero.translation.sub(wrist).dot(zeroAxis());
    if (c4 < -geometryTolerance) throw 'OPW flange lies behind wrist joint ${joints[5].id}';
    parameters = new OpwParameters(p2.x, p4.x, wrist.y, p2.z, p3.z, p4.z,
      Math.max(0.0, c4), [for (index in 0...6) reference[index] * signs[index]], signs);
    }
    native = parameters.toNative();
    var tcpAtZero = base.inverse().compose(manipulator.tcpPose(zero));
    flangeTTcp = opwForward(zero).inverse().compose(tcpAtZero);
    for (index in 0...6) {
      var probe = zero.copy();
      probe[index] = 0.37;
      var opw = forwardTransform(probe);
      var reference = manipulator.tcpPose(probe);
      if (opw.translation.sub(reference.translation).norm() > geometryTolerance ||
          opw.rotation.angularDistance(reference.rotation) > geometryTolerance)
        throw 'OPW joint ${joints[index].id} fails forward-geometry agreement';
    }
  }

  static function zeroAxis():Vec3 return new Vec3(0.0, 0.0, 1.0);

  static function alignmentAngle(from:Vec3, to:Vec3, axis:Vec3):Float {
    var a = from.sub(axis.scale(from.dot(axis)));
    var b = to.sub(axis.scale(to.dot(axis)));
    if (a.norm() < 1e-12 || b.norm() < 1e-12) throw "OPW canonical reference is degenerate";
    return Math.atan2(axis.dot(a.cross(b)), a.dot(b));
  }

  static function geometryAt(path:Array<Joint>, q:Array<Float>):{origins:Array<Vec3>, axes:Array<Vec3>} {
    var origins:Array<Vec3> = [], axes:Array<Vec3> = [];
    var at = Transform3.identity(), index = 0;
    for (joint in path) {
      var frame = at.compose(Transform3.fromArrays(joint.parentFramePosition, joint.parentFrameRotation));
      if (joint.type != JointType.Fixed) {
        var axis = Vec3.fromArray(joint.axis).normalized();
        origins.push(frame.translation);
        axes.push(frame.transformVector(axis).normalized());
        frame = frame.compose(new Transform3(new Vec3(0, 0, 0), Quat.fromAxisAngle(axis, q[index++])));
      }
      at = frame.compose(Transform3.fromArrays(joint.childFramePosition, joint.childFrameRotation).inverse());
    }
    return {origins: origins, axes: axes};
  }

  static function canonicalParameters(model:RobotModel, manipulator:Manipulator,
      joints:Array<Joint>, tolerance:Float):Null<OpwParameters> {
    var positions = [for (joint in joints) Vec3.fromArray(joint.parentFramePosition)];
    if (positions[0].norm() > tolerance ||
        Math.abs(positions[2].x) > tolerance || Math.abs(positions[2].y) > tolerance ||
        Math.abs(positions[3].y) > tolerance ||
        positions[4].norm() > tolerance || positions[5].norm() > tolerance)
      return null;
    var signs:Array<Int> = [];
    var offsets:Array<Float> = [];
    for (index in 0...6) {
      var expected = index == 0 || index == 3 || index == 5
        ? new Vec3(0.0, 0.0, 1.0) : new Vec3(0.0, 1.0, 0.0);
      var axis = Vec3.fromArray(joints[index].axis).normalized();
      var alignment = axis.dot(expected);
      if (Math.abs(Math.abs(alignment) - 1.0) > tolerance) return null;
      signs.push(alignment < 0.0 ? -1 : 1);
      var child = Transform3.fromArrays(joints[index].childFramePosition,
        joints[index].childFrameRotation);
      if (child.translation.norm() > tolerance ||
          child.rotation.angularDistance(Quat.identity()) > tolerance) return null;
      var q = Quat.fromArray(joints[index].parentFrameRotation);
      var vector = new Vec3(q.x, q.y, q.z);
      var offAxis = vector.sub(expected.scale(vector.dot(expected)));
      if (offAxis.norm() > tolerance) return null;
      offsets.push(-2.0 * Math.atan2(vector.dot(expected), q.w));
    }
    var flange:Null<Frame> = null;
    for (frame in model.frames)
      if (frame.id == manipulator.flangeFrame) { flange = frame; break; }
    if (flange == null) return null;
    var tip = Vec3.fromArray(flange.position);
    if (Math.abs(tip.x) > tolerance || Math.abs(tip.y) > tolerance ||
        tip.z < -tolerance) return null;
    return new OpwParameters(positions[1].x, positions[3].x, positions[1].y,
      positions[1].z, positions[2].z, positions[3].z, Math.max(0.0, tip.z),
      offsets, signs);
  }

  /** Nothing here changes once built, and the arm is safe to share. */
  public function fork():KinematicsSolver return this;

  public function jointCount():Int return 6;
  public function family():String return "OPW";

  public function branches(target:Pose3, seed:Array<Float>, ?freedom:OrientationPolicy):Array<AnalyticBranch> {
    if (!ToolFreedom.isFull(freedom)) throw "OPW tool freedom must be sampled before analytic branch enumeration";
    if (seed == null || seed.length != 6) throw "OPW branch enumeration needs six seed joints";
    return branchCandidates(target, new IkTolerance(1e-6, 1e-6), seed);
  }

  public function forward(q:Array<Float>):Pose3 {
    var result = forwardTransform(q);
    return new Pose3(result.translation.x, result.translation.y, result.translation.z,
      result.rotation.x, result.rotation.y, result.rotation.z, result.rotation.w);
  }

  public function solvePose(target:Pose3, seed:Array<Float>,
      tolerance:IkTolerance, ?freedom:OrientationPolicy):Null<Array<Float>> {
    if (!ToolFreedom.isFull(freedom)) return differential.solvePose(target, seed, tolerance, freedom);
    if (seed == null || seed.length != 6) throw "OPW pose solving needs six seed joints";
    var candidates = candidates(target, tolerance, seed);
    if (candidates.length == 0) return null;
    candidates.sort((a, b) -> compareDistance(a, b, seed));
    return candidates[0];
  }

  public function sampleCandidates(target:Pose3, maxCount:Int,
      tolerance:IkTolerance, ?freedom:OrientationPolicy):Array<Array<Float>> {
    if (!ToolFreedom.isFull(freedom)) return differential.sampleCandidates(target, maxCount, tolerance, freedom);
    if (maxCount < 0) throw "OPW candidate count must be non-negative";
    if (maxCount == 0) return [];
    var result = candidates(target, tolerance, null);
    result.sort((a, b) -> compareDistance(a, b, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]));
    return result.slice(0, maxCount);
  }

  public function solveDifferential(q:Array<Float>, twist:Twist6,
      ?redundancyRate:Array<Float>, ?freedom:OrientationPolicy):Null<Array<Float>>
    return differential.solveDifferential(q, twist, redundancyRate, freedom);

  /**
   * The path across the arm's analytic branches: every sample's OPW
   * solutions and the cheapest continuous route through them, in one native
   * call (`mk_select_opw_configurations`).
   */
  public function solvePath(request:PathRequest):Array<Null<Array<Float>>> {
    for (freedom in request.freedoms) if (!ToolFreedom.isFull(freedom)) return differential.solvePath(request);
    var lower:Array<Float> = [], upper:Array<Float> = [];
    for (joint in 0...jointCount()) {
      var bounds = manipulator.group.limitsOf(joint);
      lower.push(bounds.lower < bounds.upper ? bounds.lower : -1e6);
      upper.push(bounds.lower < bounds.upper ? bounds.upper : 1e6);
    }
    var selector = new PathConfigurationSelector(this, lower, upper, request.maxJump, request.velocity, null, 1,
      request.maxCandidates);
    var samples = [for (index in 0...request.poses.length) nativePathSample(request.distances[index],
      request.poses[index])];
    var selected = MotionKitNative.mk_select_opw_configurations(selector.nativeRequest(), native, samples,
      request.startQ);
    var result:Array<Null<Array<Float>>> = [];
    for (q in selector.readResult(selected.status, selected.out_sequence)) result.push(q);
    return result;
  }

  public function nativeParameters():mk_opw_parameters return native;

  /** Transform a requested TCP pose to the OPW flange frame for bulk IK. */
  public function nativePathSample(distance:Float, target:Pose3):mk_opw_path_sample {
    if (target == null || !Math.isFinite(distance))
      throw "OPW path sample needs a finite distance and pose";
    var local = localFlangePose(target);
    var result = new mk_opw_path_sample();
    result.set_struct_size(mk_opw_path_sample.size());
    result.set_distance(distance);
    var position = local.translation.toArray();
    for (index in 0...3) result.set_position(index, position[index]);
    var quaternion = [local.rotation.x, local.rotation.y, local.rotation.z, local.rotation.w];
    for (index in 0...4) result.set_quaternion(index, quaternion[index]);
    return result;
  }

  function localFlangePose(target:Pose3):Transform3 {
    var requested = new Transform3(new Vec3(target.x, target.y, target.z),
      new Quat(target.qx, target.qy, target.qz, target.qw));
    return base.inverse().compose(requested).compose(flangeTTcp.inverse());
  }

  function candidates(target:Pose3, tolerance:IkTolerance,
      seed:Null<Array<Float>>):Array<Array<Float>>
    return [for (candidate in branchCandidates(target, tolerance, seed)) candidate.q];

  function branchCandidates(target:Pose3, tolerance:IkTolerance,
      seed:Null<Array<Float>>):Array<AnalyticBranch> {
    if (target == null || tolerance == null) throw "OPW candidate sampling needs pose and tolerance";
    var requested = new Transform3(new Vec3(target.x, target.y, target.z),
      new Quat(target.qx, target.qy, target.qz, target.qw));
    var local = localFlangePose(target);
    var nativePose = new mk_opw_pose();
    nativePose.set_struct_size(mk_opw_pose.size());
    for (index in 0...3) nativePose.set_position(index, local.translation.toArray()[index]);
    var quaternion = [local.rotation.x, local.rotation.y, local.rotation.z, local.rotation.w];
    for (index in 0...4) nativePose.set_quaternion(index, quaternion[index]);
    var result = MotionKitNative.mk_opw_inverse(native, nativePose, 8);
    if (result.status != TrajectoryCoreConstants.MK_OK)
      throw 'OPW inverse failed with MotionKit error ${result.status}';
    var candidates:Array<AnalyticBranch> = [];
    var slot = 0;
    for (solution in result.out_solutions) {
      var branch = slot++;
      if (solution.get_valid() == 0) continue;
      var raw = [for (joint in 0...6) solution.get_joints(joint)];
      var wrapped:Array<Array<Float>> = [raw];
      for (joint in 0...6) {
        var next:Array<Array<Float>> = [];
        var bounds = manipulator.group.limitsOf(joint);
        for (candidate in wrapped) {
          var first = -1, last = 1;
          if (bounds.lower < bounds.upper) {
            var lo = Math.ceil((bounds.lower - candidate[joint] - 1e-9) / (2 * Math.PI));
            var hi = Math.floor((bounds.upper - candidate[joint] + 1e-9) / (2 * Math.PI));
            if (!Math.isFinite(lo) || !Math.isFinite(hi) || hi - lo > 64)
              throw "OPW periodic lifts require a finite rotary planning range";
            first = Std.int(lo); last = Std.int(hi);
          } else if (seed != null) {
            var nearest = Math.round((seed[joint] - candidate[joint]) / (2 * Math.PI));
            first += nearest; last += nearest;
          }
          for (shift in first...last + 1) {
            var variant = candidate.copy();
            variant[joint] += shift * 2.0 * Math.PI;
            if (withinJoint(joint, variant[joint])) next.push(variant);
          }
        }
        wrapped = next;
      }
      for (candidate in wrapped) {
        var achieved = forwardTransform(candidate);
        if (achieved.translation.sub(requested.translation).norm() > tolerance.position ||
            achieved.rotation.angularDistance(requested.rotation) > tolerance.orientation)
          continue;
        var duplicate = false;
        for (existing in candidates) {
          var squared = 0.0;
          for (joint in 0...6) squared += Math.pow(existing.q[joint] - candidate[joint], 2);
          if (squared < tolerance.candidateSeparation * tolerance.candidateSeparation)
            duplicate = true;
        }
        if (!duplicate) candidates.push(new AnalyticBranch(candidate, branch, solution.get_singular() != 0));
      }
    }
    return candidates;
  }

  function withinJoint(index:Int, value:Float):Bool {
    var limits = manipulator.group.limitsOf(index);
    return limits.lower >= limits.upper ||
      value >= limits.lower - 1e-12 && value <= limits.upper + 1e-12;
  }

  static function compareDistance(a:Array<Float>, b:Array<Float>, seed:Array<Float>):Int {
    var da = 0.0, db = 0.0;
    for (joint in 0...6) {
      da += Math.pow(a[joint] - seed[joint], 2);
      db += Math.pow(b[joint] - seed[joint], 2);
    }
    return da < db ? -1 : da > db ? 1 : 0;
  }

  function forwardTransform(q:Array<Float>):Transform3
    return base.compose(opwForward(q)).compose(flangeTTcp);

  function opwForward(q:Array<Float>):Transform3 {
    if (q == null || q.length != 6) throw "OPW forward needs six joint values";
    var result = MotionKitNative.mk_opw_forward(native, q);
    if (result.status != TrajectoryCoreConstants.MK_OK)
      throw 'OPW forward failed with MotionKit error ${result.status}';
    var pose = result.out_pose;
    return new Transform3(new Vec3(pose.get_position(0), pose.get_position(1),
      pose.get_position(2)), new Quat(pose.get_quaternion(0),
      pose.get_quaternion(1), pose.get_quaternion(2), pose.get_quaternion(3)));
  }

  static function orderedJoints(model:RobotModel, manipulator:Manipulator):Array<Joint>
    return manipulator.pathJoints();

}

/** Parameters inferred from physical joint origins, axis directions, and TCP. */
class OpwParameters {
  public final a1:Float;
  public final a2:Float;
  public final b:Float;
  public final c1:Float;
  public final c2:Float;
  public final c3:Float;
  public final c4:Float;
  public final offsets:Array<Float>;
  public final signCorrections:Array<Int>;

  public function new(a1:Float, a2:Float, b:Float, c1:Float, c2:Float,
      c3:Float, c4:Float, offsets:Array<Float>, signs:Array<Int>) {
    this.a1 = a1; this.a2 = a2; this.b = b; this.c1 = c1;
    this.c2 = c2; this.c3 = c3; this.c4 = c4;
    this.offsets = offsets.copy();
    signCorrections = signs.copy();
  }

  public function toNative():mk_opw_parameters {
    var result = new mk_opw_parameters();
    result.set_struct_size(mk_opw_parameters.size());
    result.set_a1(a1); result.set_a2(a2); result.set_b(b);
    result.set_c1(c1); result.set_c2(c2);
    result.set_c3(c3); result.set_c4(c4);
    for (index in 0...6) {
      result.set_offsets(index, offsets[index]);
      result.set_sign_corrections(index, signCorrections[index]);
    }
    return result;
  }
}
