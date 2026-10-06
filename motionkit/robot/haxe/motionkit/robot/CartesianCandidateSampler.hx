package motionkit.robot;

import MotionKitNative;
import TrajectoryCore;
import motionkit.kinematics.Pose3;
import motionkit.path.OrientationPolicy;
import robotkit.manipulation.KinematicGroup;

/** One native Cartesian orientation/branch/lift call, with model-derived limits. */
class CartesianCandidateSampler {
  public final analytic:CartesianAnalyticIk;
  final group:KinematicGroup;
  final model:mk_serial_cell_model;
  final external:mk_external_lattice;
  final limits:mk_joint_lift_request;

  public function new(group:KinematicGroup) {
    this.group=group;
    analytic=new CartesianAnalyticIk(group);
    if (group.external.indexOf(true)>=0) throw "Cartesian candidate export requires a standalone Cartesian group";
    var count=group.group.count();
    model=new mk_serial_cell_model();model.set_struct_size(mk_serial_cell_model.size());
    model.set_joint_count(count);model.set_arm_joint_count(count);model.set_external_count(0);
    model.set_base_quaternion(3,1);model.set_work_quaternion(3,1);model.set_tool_quaternion(3,1);
    external=new mk_external_lattice();external.set_struct_size(mk_external_lattice.size());
    external.set_joint_count(count);external.set_axis_count(0);
    limits=new mk_joint_lift_request();limits.set_struct_size(mk_joint_lift_request.size());limits.set_joint_count(count);
    for (i in 0...count) {
      model.set_arm_joint_indices(i,i);
      var bound=group.group.limitsOf(i);
      limits.set_lower(i,bound.lower);limits.set_upper(i,bound.upper);limits.set_periodic(i,i>=3 ? 1 : 0);
    }
  }
  public function sample(target:Pose3,seed:Array<Float>,freedom:OrientationPolicy,
      rollCount:Int=12,tiltRings:Int=3,azimuthCount:Int=8):Array<LatticeCandidate> {
    if (target==null || seed==null || seed.length!=group.group.count())
      throw "Cartesian candidate sampling requires a target and complete seed";
    var orientation=OrientationLattice.describe(freedom,rollCount,tiltRings,azimuthCount);
    var centre=ToolFreedom.of(target,freedom,0.0).target;
    var pose=new mk_opw_pose();pose.set_struct_size(mk_opw_pose.size());
    var p=[centre.x,centre.y,centre.z],r=[centre.qx,centre.qy,centre.qz,centre.qw];
    for (i in 0...3)pose.set_position(i,p[i]);for (i in 0...4)pose.set_quaternion(i,r[i]);
    var size=MotionKitNative.mk_cartesian_candidate_count(analytic.nativeModel(),model,external,orientation,limits,pose,seed);
    if (size.status!=TrajectoryCoreConstants.MK_OK)throw 'Native Cartesian candidate count failed: ${size.status}';
    if (size.out_count==0)return [];
    var result=MotionKitNative.mk_sample_cartesian_candidates(analytic.nativeModel(),model,external,orientation,limits,pose,seed,size.out_count);
    if (result.status!=TrajectoryCoreConstants.MK_OK)throw 'Native Cartesian candidate sampling failed: ${result.status}';
    return [for (i in 0...result.out_count) {
      var c=result.out_candidates[i];
      new LatticeCandidate([for (j in 0...seed.length)c.get_joints(j)],[for (j in 0...seed.length)c.get_wraps(j)],
        [],c.get_roll_index(),c.get_tilt_index(),c.get_azimuth_index(),c.get_branch(),c.get_singular());
    }];
  }
}
class LatticeCandidate {
  public final q:Array<Float>;
  public final wraps:Array<Int>;
  public final external:Array<Int>;
  public final roll:Int;
  public final tilt:Int;
  public final azimuth:Int;
  public final branch:Int;
  public final singular:Int;
  public function new(q:Array<Float>,wraps:Array<Int>,external:Array<Int>,roll:Int,tilt:Int,azimuth:Int,branch:Int,singular:Int) {
    this.q=q;this.wraps=wraps;this.external=external;this.roll=roll;this.tilt=tilt;this.azimuth=azimuth;
    this.branch=branch;this.singular=singular;
  }
}
