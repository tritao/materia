package motionkit.robot;

import MotionKitNative;
import TrajectoryCore;
import motionkit.robot.AnalyticIk.AnalyticBranch;
import motionkit.kinematics.Pose3;
import motionkit.path.OrientationPolicy;
import robotkit.manipulation.KinematicGroup;
import robotkit.model.JointType;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;

/** Model-derived XYZ and C/A heads, evaluated in the group's fixed reference. */
class CartesianAnalyticIk implements AnalyticIk {
  public final group:KinematicGroup;
  final native:mk_analytic_cartesian_model;

  public function new(group:KinematicGroup) {
    if (group == null || group.workFrame != null)
      throw "Cartesian analytic IK requires a group with a fixed reference";
    this.group = group;
    var count = group.group.count();
    if (count < 3 || count > 5) throw "Cartesian analytic IK requires XYZ, XYZ+C or XYZ+C+A";
    native = new mk_analytic_cartesian_model();
    native.set_struct_size(mk_analytic_cartesian_model.size());
    native.set_joint_count(count);
    var at = Transform3.identity(), index = 0;
    for (joint in group.pathJoints()) {
      var frame = at.compose(Transform3.fromArrays(joint.parentFramePosition, joint.parentFrameRotation));
      if (joint.type != JointType.Fixed) {
        if (index >= count || joint.id != group.group.jointIds[index])
          throw "Cartesian analytic IK requires independent joints in chain order";
        var axis = frame.transformVector(Vec3.fromArray(joint.axis)).normalized().toArray();
        if (index < 3) {
          if (joint.type != JointType.Prismatic) throw "Cartesian analytic IK requires three leading prismatic joints";
          for (i in 0...3) native.set_translation_axes(index * 3 + i, axis[i]);
        } else {
          if (joint.type != JointType.Revolute && joint.type != JointType.Continuous)
            throw "Cartesian analytic head requires revolute C/A joints";
          var origin = frame.translation.toArray();
          for (i in 0...3) {
            native.set_rotary_axes((index - 3) * 3 + i, axis[i]);
            native.set_rotary_origins((index - 3) * 3 + i, origin[i]);
          }
        }
        index++;
      }
      at = frame.compose(Transform3.fromArrays(joint.childFramePosition, joint.childFrameRotation).inverse());
    }
    if (index != count) throw "Cartesian analytic joint count does not match the model";
    var home = group.tcpPose([for (_ in 0...count) 0.0]);
    var position = home.translation.toArray(), r = home.rotation;
    var rotation = [r.x, r.y, r.z, r.w];
    for (i in 0...3) native.set_home_position(i, position[i]);
    for (i in 0...4) native.set_home_quaternion(i, rotation[i]);
    // Verify extraction against compiled FK, including fixed links and the tool.
    for (probe in 0...count + 1) {
      var q = [for (i in 0...count) probe == i ? 0.37 : 0.0];
      var actual = group.tcpPose(q), predicted = forward(q);
      var p = new Vec3(predicted.x, predicted.y, predicted.z);
      var qr = new robotkit.spatial.Quat(predicted.qx, predicted.qy, predicted.qz, predicted.qw);
      if (p.sub(actual.translation).norm() > 1e-7 || qr.angularDistance(actual.rotation) > 1e-7)
        throw "Cartesian analytic extraction fails model FK agreement";
    }
  }

  public function family():String return switch jointCount() { case 3: "XYZ"; case 4: "XYZ+C"; default: "XYZ+C+A"; }
  public function jointCount():Int return group.group.count();
  /** Immutable descriptor for native lattice sampling. */
  public function nativeModel():mk_analytic_cartesian_model return native;

  public function forward(q:Array<Float>):Pose3 {
    if (q == null || q.length != jointCount()) throw "Cartesian analytic FK needs complete joints";
    var result = MotionKitNative.mk_analytic_cartesian_forward(native, q);
    if (result.status != TrajectoryCoreConstants.MK_OK)
      throw 'Cartesian analytic FK failed with MotionKit error ${result.status}';
    var p = result.out_pose;
    return new Pose3(p.get_position(0), p.get_position(1), p.get_position(2),
      p.get_quaternion(0), p.get_quaternion(1), p.get_quaternion(2), p.get_quaternion(3));
  }

  public function branches(target:Pose3, seed:Array<Float>, ?freedom:OrientationPolicy):Array<AnalyticBranch> {
    if (target == null || seed == null || seed.length != jointCount())
      throw "Cartesian analytic IK needs a target and complete seed";
    var axisOnly = switch freedom { case null | Fixed | Interpolated: false; case FreeAboutTool: true;
      default: throw "Cartesian analytic IK does not enumerate this orientation freedom"; };
    var pose = new mk_opw_pose(); pose.set_struct_size(mk_opw_pose.size());
    var xyz = [target.x, target.y, target.z], quat = [target.qx, target.qy, target.qz, target.qw];
    for (i in 0...3) pose.set_position(i, xyz[i]);
    for (i in 0...4) pose.set_quaternion(i, quat[i]);
    var result = MotionKitNative.mk_analytic_cartesian_inverse(native, pose, axisOnly ? 1 : 0,
      jointCount() > 3 ? seed[3] : 0.0, 2);
    if (result.status != TrajectoryCoreConstants.MK_OK)
      throw 'Cartesian analytic IK failed with MotionKit error ${result.status}';
    var answers:Array<AnalyticBranch> = [];
    for (i in 0...result.out_count) {
      var raw = result.out_solutions[i];
      var lifted = [[for (joint in 0...jointCount()) raw.get_joints(joint)]];
      for (joint in 0...jointCount()) {
        var bounds = group.group.limitsOf(joint);
        var next:Array<Array<Float>> = [];
        for (q in lifted) {
          if (joint < 3) {
            if (q[joint] >= bounds.lower - 1e-9 && q[joint] <= bounds.upper + 1e-9) next.push(q);
          } else {
            var first = Math.ceil((bounds.lower - q[joint] - 1e-9) / (2 * Math.PI));
            var last = Math.floor((bounds.upper - q[joint] + 1e-9) / (2 * Math.PI));
            if (!Math.isFinite(first) || !Math.isFinite(last) || last - first > 64)
              throw "Cartesian analytic wraps require a finite rotary planning range";
            for (wrap in Std.int(first)...Std.int(last) + 1) {
              var value = q.copy(); value[joint] += wrap * 2 * Math.PI; next.push(value);
            }
          }
        }
        lifted = next;
      }
      for (q in lifted) answers.push(new AnalyticBranch(q, raw.get_branch(), raw.get_singular() != 0));
    }
    return answers;
  }
}
