package motionkit.path;

import TrajectoryCore;
import MotionKitNative;
import motionkit.planner.JointPathSamples;
import motionkit.planner.PathTimingLimits;
import motionkit.trajectory.Trajectory;

/** Native C2 joint path. Owns the path handle and lowers against a time law. */
class NativeJointPath {
  final owner:Ownedmk_path_handle;
  var disposed:Bool = false;

  public function new(samples:JointPathSamples) {
    if (samples.jointCount > TrajectoryCoreConstants.MK_MAX_JOINTS)
      throw "Native joint path exceeds the joint limit";
    var native:Array<mk_path_sample> = [];
    for (index in 0...samples.s.length) {
      var sample = new mk_path_sample();
      sample.set_struct_size(mk_path_sample.size());
      sample.set_s(samples.s[index]);
      sample.set_joint_count(samples.jointCount);
      for (joint in 0...samples.jointCount) {
        sample.set_position(joint, samples.q[index][joint]);
        sample.set_first(joint, samples.qPrime[index][joint]);
        sample.set_second(joint, samples.qDoublePrime[index][joint]);
        sample.set_second_before(joint, samples.qDoublePrimeBefore[index][joint]);
      }
      native.push(sample);
    }
    var created = MotionKitNative.mk_path_create(native);
    check(created.status, "path.create");
    owner = created.out_path;
  }

  public function lower(law:PathTimeLaw, tolerance:Float):Trajectory {
    if (disposed) throw "Native joint path has been disposed";
    if (law == null) throw "Native joint path needs a time law";
    var created = MotionKitNative.mk_path_lower(owner.borrow(), law.borrow(), tolerance);
    check(created.status, "path.lower");
    return new Trajectory(created.out_trajectory);
  }

  public function time(limits:PathTimingLimits,
      loweringTolerance:Float):{law:PathTimeLaw, trajectory:Trajectory} {
    if (disposed) throw "Native joint path has been disposed";
    var created = MotionKitNative.mk_time_path(owner.borrow(), limits.maxVelocity,
      limits.maxAcceleration, limits.speedCaps, limits.startPathSpeed,
      limits.endPathSpeed, loweringTolerance,limits.requireFeasiblePath?1:0);
    if(created.status!=TrajectoryCoreConstants.MK_OK && limits.requireFeasiblePath && created.out_report.get_derivative()!=0)
      throw 'Refined process timing requires an adjustment: joint ${created.out_report.get_joint()}, derivative ${created.out_report.get_derivative()}, ${created.out_report.get_value()} > ${created.out_report.get_limit()}, sustainable feed scale ${created.out_report.get_sustainable_scale()}';
    check(created.status, "path.time");
    return {law: new PathTimeLaw([], created.out_law),
      trajectory: new Trajectory(created.out_trajectory)};
  }

  public function dispose():Void {
    if (disposed) return;
    disposed = true;
    owner.close();
  }

  static function check(status:Int, operation:String):Void {
    if (status != TrajectoryCoreConstants.MK_OK)
      throw '$operation failed with MotionKit error $status';
  }
}
