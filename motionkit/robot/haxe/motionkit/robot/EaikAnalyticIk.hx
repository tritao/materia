package motionkit.robot;
import MotionKitNative;
import TrajectoryCore;
import motionkit.robot.AnalyticIk.AnalyticBranch;
import motionkit.kinematics.Pose3;
import motionkit.kinematics.SixAxisConfiguration;
import motionkit.path.OrientationPolicy;
import robotkit.manipulation.KinematicGroup;

/** Exact compiled-chain inverse with physical geometric labels and periodic turns. */
class EaikAnalyticIk implements AnalyticIk {
  public final chain:EaikChain;
  public final configuration:EaikConfiguration;
  final lifts:JointLifts;
  public function new(group:KinematicGroup) {
    chain=new EaikChain(group);
    try configuration=new EaikConfiguration(chain) catch(error:Dynamic){chain.dispose();throw error;}
    lifts=new JointLifts(group,[for(i in 0...group.group.count())!group.external[i]]);
  }
  public function family():String return "EAIK";
  public function jointCount():Int return chain.group.group.count();
  public function forward(q:Array<Float>):Pose3 {
    var t=chain.group.tcpPose(q),p=t.translation,r=t.rotation;
    return new Pose3(p.x,p.y,p.z,r.x,r.y,r.z,r.w);
  }
  public function branches(target:Pose3,seed:Array<Float>,?freedom:OrientationPolicy):Array<AnalyticBranch> {
    if(!ToolFreedom.isFull(freedom))throw "EAIK requires a sampled fixed tool orientation";
    var result=MotionKitNative.mk_eaik_inverse_labelled(chain.solver,configuration.native,chain.localTarget(target,seed),[for(j in chain.armIndices)seed[j]],32);
    if(result.status!=TrajectoryCoreConstants.MK_OK)throw 'EAIK inverse failed: ${result.status}';
    var answers:Array<AnalyticBranch> = [];
    for(i in 0...result.out_count){
      var raw=result.out_solutions[i],q=seed.copy(),arm=[for(j in 0...6)raw.get_joints(j)];
      var branch=raw.get_branch();
      for(j in 0...6)q[chain.armIndices[j]]=arm[j];
      for(lift in lifts.enumerate(q))answers.push(new AnalyticBranch(lift.q,branch,raw.get_singular()!=0,true,
        SixAxisConfiguration.of("EAIK",branch,[for(j in chain.armIndices)lift.q[j]])));
    }
    answers.sort((a,b)->a.branch-b.branch);
    return answers;
  }
}
