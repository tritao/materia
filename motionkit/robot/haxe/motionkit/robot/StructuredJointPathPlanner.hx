package motionkit.robot;

import motionkit.kinematics.PathRequest;
import motionkit.path.PosePath;
import motionkit.path.PoseMath;
import motionkit.planner.JointPathSamples;
import motionkit.robot.CandidateProblem.CandidateSamplingOptions;
import motionkit.robot.StructuredLadder.CoarseSearchOptions;
import robotkit.manipulation.KinematicGroup;
import robotkit.manipulation.ArmClearance;

/** Native ladder selection followed by analytic geometric refinement.
 * Unsupported-family fallback and edge exclusion remain separate stages. */
class StructuredJointPathPlanner implements JointPathPlanner {
  public final group:KinematicGroup;
  final sampling:Null<CandidateSamplingOptions>;
  final coarse:Null<CoarseSearchOptions>;
  final clearance:Null<ArmClearance>;
  final collisionRounds:Int;
  final contact:Bool;
  public function new(group:KinematicGroup,?sampling:CandidateSamplingOptions,?coarse:CoarseSearchOptions,
      ?clearance:ArmClearance,collisionRounds:Int=8,contact:Bool=false) {
    if(group==null || collisionRounds<1)throw "Joint path planner requires a compiled group and positive collision round budget";
    this.group=group;this.sampling=sampling;this.coarse=coarse;
    this.clearance=clearance;this.collisionRounds=collisionRounds;this.contact=contact;
  }
  public function plan(path:PosePath,request:PathRequest):JointPathSamples {
    if(path==null || request==null || request.distances.length<2 || request.distances[0]!=0 ||
        request.distances[request.distances.length-1]!=path.length())
      throw "Joint path request must span its complete authored path";
    // Refine the same geometric task that was searched. A caller may use
    // coarse samples, but cannot substitute a different pose path afterwards.
    var provider=new PosePathRefinement(path);
    for(i in 0...request.distances.length){var task=provider.at(request.distances[i]);
      if(!(ToolFreedom.isFull(task.freedom) && ToolFreedom.isFull(request.freedoms[i])) &&
          !Type.enumEq(task.freedom,request.freedoms[i]))
        throw 'Joint path request differs from authored freedom at sample $i';
      if(PoseMath.distance(task.pose,request.poses[i])>request.tolerance.position ||
          ToolFreedom.orientationError(task.pose,request.poses[i],task.freedom)>request.tolerance.orientation)
        throw 'Joint path request differs from authored geometry at sample $i';
    }
    var problem=new CandidateProblem(group,request,sampling);
    if(problem.diagnostic!=null)throw 'Joint path family requires fallback: ${problem.diagnostic}';
    var selected=clearance==null ? StructuredLadder.search(problem,null,0,null,coarse)
      : LazyCollisionLadder.select(problem,clearance,collisionRounds,contact,coarse);
    if(selected.diagnostic!=null)throw 'Joint path selection failed at distance ${selected.failedDistance}: ${selected.diagnostic}';
    var refined=new AnalyticPathRefiner(group,problem,selected).refinePath(request.distances,provider.at);
    // Spline refinement can leave the discrete joint interpolation. Check
    // the resulting samples and sweeps before releasing it to timing.
    if(clearance!=null)for(i in 0...refined.q.length){
      var failure=clearance.violation(refined.q[i],contact);
      if(failure==null && i>0)failure=clearance.sweep(refined.q[i-1],refined.q[i],contact);
      if(failure!=null)throw 'Refined path collision at distance ${refined.s[i]} (${failure.a}, ${failure.b})';
    }
    return refined;
  }
}
