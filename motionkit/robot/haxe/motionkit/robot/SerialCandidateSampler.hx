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
      rollCount:Int=12,tiltRings:Int=3,azimuthCount:Int=8,?maxJump:Array<Float>,
      ?jointBounds:{lower:Array<Float>,upper:Array<Float>}):Array<LatticeCandidate> {
    if(target==null)throw "Serial candidate sampling requires a target";
    var liftLimits=limits;
    if(maxJump!=null || jointBounds!=null){
      if(seed==null || seed.length!=group.group.count() || maxJump!=null && maxJump.length!=seed.length ||
          jointBounds!=null && (jointBounds.lower==null || jointBounds.upper==null ||
            jointBounds.lower.length!=seed.length || jointBounds.upper.length!=seed.length))
        throw "Serial jump bounds must match complete joints";
      liftLimits=new mk_joint_lift_request();liftLimits.set_struct_size(mk_joint_lift_request.size());
      liftLimits.set_joint_count(seed.length);
      for(j in 0...seed.length){
        if(maxJump!=null && (!Math.isFinite(maxJump[j]) || maxJump[j]<=0))
          throw "Serial jump bounds must be finite and positive";
        var bound=group.group.limitsOf(j);
        var lower=bound.lower,upper=bound.upper;
        if(maxJump!=null){lower=Math.max(lower,seed[j]-maxJump[j]-1e-12);upper=Math.min(upper,seed[j]+maxJump[j]+1e-12);}
        if(jointBounds!=null){
          if(!Math.isFinite(jointBounds.lower[j]) || !Math.isFinite(jointBounds.upper[j]) ||
              jointBounds.lower[j]>jointBounds.upper[j])throw "Serial joint bounds must be finite ordered intervals";
          lower=Math.max(lower,jointBounds.lower[j]);upper=Math.min(upper,jointBounds.upper[j]);
        }
        // External cells are authored by ExternalAxisGrid. Its full range
        // must remain inside these native limits; forward pruning handles
        // cell-to-cell jumps after sampling without reindexing the grid.
        if(group.external[j]){lower=bound.lower;upper=bound.upper;}
        if(lower>upper)return [];
        liftLimits.set_lower(j,lower);
        liftLimits.set_upper(j,upper);
        liftLimits.set_periodic(j,group.external[j] ? 0 : 1);
      }
    }
    var external=ExternalAxisGrid.describe(group,seed,ranges);
    var orientation=OrientationLattice.describe(freedom,rollCount,tiltRings,azimuthCount);
    var centre=OrientationLattice.centre(target,freedom);
    var pose=new mk_opw_pose();pose.set_struct_size(mk_opw_pose.size());
    var p=[centre.x,centre.y,centre.z],r=[centre.qx,centre.qy,centre.qz,centre.qw];
    for(i in 0...3)pose.set_position(i,p[i]);for(i in 0...4)pose.set_quaternion(i,r[i]);
    if(ur!=null) {
      var size=MotionKitNative.mk_ur_candidate_count(ur.nativeModel(),model.native,external,orientation,liftLimits,pose,seed);
      if(size.status!=TrajectoryCoreConstants.MK_OK)throw 'Native UR candidate count failed: ${size.status}';
      if(size.out_count==0)return [];
      var lengths=CartesianCandidateSampler.compactLengths(size.out_count,seed.length,model.externalIndices.length);
      var result=MotionKitNative.mk_sample_ur_candidates_compact(ur.nativeModel(),model.native,external,orientation,liftLimits,pose,seed,lengths.joints,lengths.coordinates);
      if(result.status!=TrajectoryCoreConstants.MK_OK)throw 'Native UR candidate sampling failed: ${result.status}';
      return CartesianCandidateSampler.decodeCompact(result.out_joints,result.out_wraps,result.out_coordinates,
        result.out_count,seed.length,model.externalIndices.length);
    }
    var size=MotionKitNative.mk_opw_candidate_count(opw.nativeModel(),model.native,external,orientation,liftLimits,pose,seed);
    if(size.status!=TrajectoryCoreConstants.MK_OK)throw 'Native OPW candidate count failed: ${size.status}';
    if(size.out_count==0)return [];
    var lengths=CartesianCandidateSampler.compactLengths(size.out_count,seed.length,model.externalIndices.length);
    var result=MotionKitNative.mk_sample_opw_candidates_compact(opw.nativeModel(),model.native,external,orientation,liftLimits,pose,seed,lengths.joints,lengths.coordinates);
    if(result.status!=TrajectoryCoreConstants.MK_OK)throw 'Native OPW candidate sampling failed: ${result.status}';
    return CartesianCandidateSampler.decodeCompact(result.out_joints,result.out_wraps,result.out_coordinates,
        result.out_count,seed.length,model.externalIndices.length);
  }
}
