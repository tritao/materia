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
 * Unsupported families use explicitly diagnosed numeric continuation. */
class StructuredJointPathPlanner implements JointPathPlanner {
  public final group:KinematicGroup;
  public var fallbackDiagnostic(default,null):Null<String> = null;
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
  public function withSolver(solver:motionkit.kinematics.KinematicsSolver):JointPathPlanner {
    if(!Std.isOfType(solver,ManipulatorKinematics))throw "Structured planner worker requires compiled group kinematics";
    var adapter:ManipulatorKinematics=cast solver;
    return new StructuredJointPathPlanner(adapter.manipulator,sampling,coarse,
      clearance==null ? null : clearance.withGroup(adapter.manipulator),collisionRounds,contact);
  }
  static function sameFreedom(a:motionkit.path.OrientationPolicy,b:motionkit.path.OrientationPolicy):Bool {
    return switch a {
      case Fixed | Interpolated:ToolFreedom.isFull(b);
      case FreeAboutTool:switch b {case FreeAboutTool:true;default:false;};
      case Free:switch b {case Free:true;default:false;};
      case Cone(axis,angle):switch b {
        case Cone(other,half):
          var same=axis!=null && other!=null && axis.length==other.length && angle==half;
          if(same)for(i in 0...axis.length)if(axis[i]!=other[i])same=false;
          same;
        default:false;
      };
    };
  }
  public function plan(path:PosePath,request:PathRequest):JointPathSamples {
    if(path==null || request==null || request.distances.length<2 || request.distances[0]!=0 ||
        request.distances[request.distances.length-1]!=path.length())
      throw "Joint path request must span its complete authored path";
    // Refine the same geometric task that was searched. A caller may use
    // coarse samples, but cannot substitute a different pose path afterwards.
    var provider=new PosePathRefinement(path);
    for(i in 0...request.distances.length){var task=provider.at(request.distances[i]);
      if(!sameFreedom(task.freedom,request.freedoms[i]))
        throw 'Joint path request differs from authored freedom at sample $i';
      if(PoseMath.distance(task.pose,request.poses[i])>request.tolerance.position ||
          ToolFreedom.orientationError(task.pose,request.poses[i],task.freedom)>request.tolerance.orientation)
        throw 'Joint path request differs from authored geometry at sample $i';
    }
    var problem=new CandidateProblem(group,request,sampling);
    fallbackDiagnostic=problem.diagnostic;
    var refined:Null<JointPathSamples> = null;
    var world=clearance;
    var selected=world==null ? StructuredLadder.search(problem,null,0,null,coarse)
      : LazyCollisionLadder.selectWithChecks(problem,q -> world.violation(q,contact),collisionRounds,
        (from,to) -> world.sweep(from,to,contact),coarse,route -> {
          var curve=new AnalyticPathRefiner(group,problem,route).refinePath(request.distances,provider.at);
          for(i in 0...curve.q.length){
            var failure=world.violation(curve.q[i],contact);
            if(failure!=null)return new motionkit.robot.LazyCollisionLadder.RefinedCollision(i,false,failure);
            if(i>0){failure=world.sweep(curve.q[i-1],curve.q[i],contact);
              if(failure!=null)return new motionkit.robot.LazyCollisionLadder.RefinedCollision(i,true,failure);}
          }
          refined=curve;return null;
        });
    if(selected.diagnostic!=null)throw 'Joint path selection failed at distance ${selected.failedDistance}: ${selected.diagnostic}';
    return refined==null ? new AnalyticPathRefiner(group,problem,selected).refinePath(request.distances,provider.at) : refined;
  }
}
