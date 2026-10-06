package motionkit.robot;

import MotionKitNative;
import TrajectoryCore;
import motionkit.kinematics.Pose3;
import motionkit.path.OrientationPolicy;
import motionkit.robot.CartesianCandidateSampler.LatticeCandidate;
import motionkit.robot.ExternalAxisGrid.ExternalAxisRange;
import robotkit.manipulation.KinematicGroup;

/** Combined native serial-arm sampling, including model-derived external transforms. */
class SerialCandidateSampler {
  final group:KinematicGroup;
  final model:SerialCellModel;
  final ur:Null<UrAnalyticIk>;
  final opw:Null<OpwKinematics>;
  final limits:mk_joint_lift_request;
  public function new(group:KinematicGroup) {
    this.group=group;
    var backend=BranchIk.of(group);
    if(Std.isOfType(backend,UrAnalyticIk)) {
      ur=cast backend;opw=null;model=new SerialCellModel(group,ur.nativeBase(),ur.nativeTool());
    } else if(Std.isOfType(backend,OpwKinematics)) {
      opw=cast backend;ur=null;model=new SerialCellModel(group,opw.nativeBase(),opw.nativeTool());
    } else throw 'Serial native sampling requires UR or OPW geometry, got ${backend.family()}';
    limits=new mk_joint_lift_request();limits.set_struct_size(mk_joint_lift_request.size());limits.set_joint_count(group.group.count());
    for(i in 0...group.group.count()) {
      var bound=group.group.limitsOf(i);
      limits.set_lower(i,bound.lower);limits.set_upper(i,bound.upper);limits.set_periodic(i,group.external[i] ? 0 : 1);
    }
  }
  public function sample(target:Pose3,seed:Array<Float>,freedom:OrientationPolicy,ranges:Array<ExternalAxisRange>,
      rollCount:Int=12,tiltRings:Int=3,azimuthCount:Int=8):Array<LatticeCandidate> {
    if(target==null)throw "Serial candidate sampling requires a target";
    var external=ExternalAxisGrid.describe(group,seed,ranges);
    var orientation=OrientationLattice.describe(freedom,rollCount,tiltRings,azimuthCount);
    var centre=ToolFreedom.of(target,freedom,0.0).target;
    var pose=new mk_opw_pose();pose.set_struct_size(mk_opw_pose.size());
    var p=[centre.x,centre.y,centre.z],r=[centre.qx,centre.qy,centre.qz,centre.qw];
    for(i in 0...3)pose.set_position(i,p[i]);for(i in 0...4)pose.set_quaternion(i,r[i]);
    if(ur!=null) {
      var size=MotionKitNative.mk_ur_candidate_count(ur.nativeModel(),model.native,external,orientation,limits,pose,seed);
      if(size.status!=TrajectoryCoreConstants.MK_OK)throw 'Native UR candidate count failed: ${size.status}';
      if(size.out_count==0)return [];
      var result=MotionKitNative.mk_sample_ur_candidates(ur.nativeModel(),model.native,external,orientation,limits,pose,seed,size.out_count);
      if(result.status!=TrajectoryCoreConstants.MK_OK)throw 'Native UR candidate sampling failed: ${result.status}';
      return decode(result.out_candidates,result.out_count);
    }
    var size=MotionKitNative.mk_opw_candidate_count(opw.nativeModel(),model.native,external,orientation,limits,pose,seed);
    if(size.status!=TrajectoryCoreConstants.MK_OK)throw 'Native OPW candidate count failed: ${size.status}';
    if(size.out_count==0)return [];
    var result=MotionKitNative.mk_sample_opw_candidates(opw.nativeModel(),model.native,external,orientation,limits,pose,seed,size.out_count);
    if(result.status!=TrajectoryCoreConstants.MK_OK)throw 'Native OPW candidate sampling failed: ${result.status}';
    return decode(result.out_candidates,result.out_count);
  }
  function decode(records:Array<mk_lattice_candidate>,count:Int):Array<LatticeCandidate>
    return [for(i in 0...count) {
      var c=records[i];
      new LatticeCandidate([for(j in 0...group.group.count())c.get_joints(j)],
        [for(j in 0...group.group.count())c.get_wraps(j)],
        [for(j in 0...model.externalIndices.length)c.get_external_coordinates(j)],
        c.get_roll_index(),c.get_tilt_index(),c.get_azimuth_index(),c.get_branch(),c.get_singular());
    }];
}
