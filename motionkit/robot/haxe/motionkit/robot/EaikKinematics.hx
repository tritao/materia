package motionkit.robot;
import motionkit.robot.AnalyticIk.AnalyticBranch;
import motionkit.kinematics.KinematicsSolver;
import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.PathRequest;
import motionkit.kinematics.Pose3;
import motionkit.kinematics.Twist6;
import motionkit.path.OrientationPolicy;
import robotkit.manipulation.KinematicGroup;

/** Compiled 6R exact inverse; the model supplies FK and differential kinematics. */
class EaikKinematics implements KinematicsSolver implements AnalyticIk {
  public final manipulator:KinematicGroup;
  public final analytic:EaikAnalyticIk;
  final differential:ManipulatorKinematics;
  public function new(group:KinematicGroup) {
    manipulator=group;analytic=new EaikAnalyticIk(group);differential=new ManipulatorKinematics(group);
  }
  public function fork():KinematicsSolver return new EaikKinematics(manipulator);
  public function family():String return analytic.family();
  public function jointCount():Int return analytic.jointCount();
  public function forward(q:Array<Float>):Pose3 return analytic.forward(q);
  public function branches(target:Pose3,seed:Array<Float>,?freedom:OrientationPolicy):Array<AnalyticBranch>
    return analytic.branches(target,seed,freedom);
  static function distance(q:Array<Float>,seed:Array<Float>):Float {
    var sum=0.0;for(i in 0...q.length)sum+=(q[i]-seed[i])*(q[i]-seed[i]);return sum;
  }
  public function solvePose(target:Pose3,seed:Array<Float>,tolerance:IkTolerance,?freedom:OrientationPolicy):Null<Array<Float>> {
    if(!ToolFreedom.isFull(freedom))return differential.solvePose(target,seed,tolerance,freedom);
    var candidates=branches(target,seed,freedom);
    candidates.sort((a,b)->{var delta=distance(a.q,seed)-distance(b.q,seed);return delta<0 ? -1 : delta>0 ? 1 : a.branch-b.branch;});
    return candidates.length==0 ? null : candidates[0].q;
  }
  public function sampleCandidates(target:Pose3,maxCount:Int,tolerance:IkTolerance,?freedom:OrientationPolicy):Array<Array<Float>> {
    if(maxCount<0)throw "EAIK candidate count must be non-negative";
    if(maxCount==0)return [];
    if(!ToolFreedom.isFull(freedom))return differential.sampleCandidates(target,maxCount,tolerance,freedom);
    var seed=[for(_ in 0...jointCount())0.0];
    var candidates=branches(target,seed,freedom);
    candidates.sort((a,b)->{var delta=distance(a.q,seed)-distance(b.q,seed);return delta<0 ? -1 : delta>0 ? 1 : a.branch-b.branch;});
    return [for(branch in candidates.slice(0,maxCount))branch.q];
  }
  public function solveDifferential(q:Array<Float>,twist:Twist6,?redundancyRate:Array<Float>,?freedom:OrientationPolicy):Null<Array<Float>>
    return differential.solveDifferential(q,twist,redundancyRate,freedom);
  public function solvePath(request:PathRequest):Array<Null<Array<Float>>> {
    var problem=new CandidateProblem(manipulator,request),selected=StructuredLadder.search(problem);
    if(selected.diagnostic!=null)throw 'EAIK path selection failed at sample ${selected.failedSample}, distance ${selected.failedDistance}: ${selected.diagnostic}';
    return [for(candidate in selected.candidates)candidate.q.copy()];
  }
}
