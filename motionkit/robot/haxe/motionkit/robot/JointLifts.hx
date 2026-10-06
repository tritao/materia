package motionkit.robot;

import MotionKitNative;
import TrajectoryCore;
import robotkit.manipulation.KinematicGroup;

/** Native finite-limit filtering and complete periodic lifts of a geometric branch. */
class JointLifts {
  final request:mk_joint_lift_request;
  final count:Int;
  public function new(group:KinematicGroup, periodic:Array<Bool>) {
    if (group == null || periodic == null || periodic.length != group.group.count())
      throw "Joint lifts require one periodic flag per group joint";
    count = group.group.count();
    request = new mk_joint_lift_request();request.set_struct_size(mk_joint_lift_request.size());request.set_joint_count(count);
    for (i in 0...count) {
      var limits = group.group.limitsOf(i);
      request.set_lower(i,limits.lower);request.set_upper(i,limits.upper);request.set_periodic(i,periodic[i] ? 1 : 0);
    }
  }
  public function enumerate(q:Array<Float>):Array<JointLift> {
    if (q == null || q.length != count) throw "Joint lifts require a complete geometric configuration";
    var size = MotionKitNative.mk_joint_lift_count(request,q);
    if (size.status != TrajectoryCoreConstants.MK_OK)
      throw "Joint lifts require finite planning limits and representable counts and wrap indices";
    if (size.out_count == 0) return [];
    var result = MotionKitNative.mk_enumerate_joint_lifts(request,q,size.out_count);
    if (result.status != TrajectoryCoreConstants.MK_OK) throw 'Native joint lift enumeration failed: ${result.status}';
    return [for (i in 0...result.out_count) {
      var value = result.out_lifts[i];
      new JointLift([for (j in 0...count) value.get_joints(j)],[for (j in 0...count) value.get_wraps(j)]);
    }];
  }
}
class JointLift {
  public final q:Array<Float>;
  public final wraps:Array<Int>;
  public function new(q:Array<Float>,wraps:Array<Int>) { this.q=q;this.wraps=wraps; }
}
