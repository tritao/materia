package motionkit.path;

import MotionKitNative;
import motionkit.planner.JointPathSamples;
import motionkit.trajectory.Trajectory;

/** Native C2 joint path. Owns the path handle and lowers against a time law. */
class NativeJointPath {
  final owner:Ownedmk_path_handle;
  var disposed:Bool = false;

  public function new(samples:JointPathSamples) {
    if (samples.jointCount > MotionKitNativeConstants.MK_MAX_JOINTS)
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

  public function dispose():Void {
    if (disposed) return;
    disposed = true;
    owner.close();
  }

  static function check(status:Int, operation:String):Void {
    if (status != MotionKitNativeConstants.MK_OK)
      throw '$operation failed with MotionKit error $status';
  }
}
