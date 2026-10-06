package motionkit.robot;

import motionkit.kinematics.SixAxisConfiguration;

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
  final eaik:EaikAnalyticIk;
  final limits:mk_joint_lift_request;
  public function new(group:KinematicGroup) {
    this.group=group;
    eaik=new EaikAnalyticIk(group);
    model=new SerialCellModel(group,robotkit.spatial.Transform3.identity(),group.flangeTTcp);
    limits=new mk_joint_lift_request();limits.set_struct_size(mk_joint_lift_request.size());limits.set_joint_count(group.group.count());
    for(i in 0...group.group.count()) {
      var bound=group.group.limitsOf(i);
      limits.set_lower(i,bound.lower);limits.set_upper(i,bound.upper);limits.set_periodic(i,group.external[i] ? 0 : 1);
    }
  }
  function labelled(candidates:Array<LatticeCandidate>):Array<LatticeCandidate> {
    for(candidate in candidates)candidate.labelArm(model.armIndices);
    return candidates;
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
    var pose=new mk_analytic_pose();pose.set_struct_size(mk_analytic_pose.size());
    var p=[centre.x,centre.y,centre.z],r=[centre.qx,centre.qy,centre.qz,centre.qw];
    for(i in 0...3)pose.set_position(i,p[i]);for(i in 0...4)pose.set_quaternion(i,r[i]);
    var result=MotionKitNative.mk_create_eaik_candidate_batch(eaik.chain.solver,eaik.configuration.native,
      model.native,external,orientation,liftLimits,pose,seed);
    if(result.status!=TrajectoryCoreConstants.MK_OK)throw 'Native EAIK candidate generation failed: ${result.status}';
    return labelled(NativeCandidateBatch.decode(result.out_batch,result.out_count,seed.length,model.externalIndices.length));
  }
}
