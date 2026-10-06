package motionkit.robot;

import motionkit.robot.StructuredLadder.LadderSelection;
import motionkit.robot.StructuredLadder.CoarseSearchOptions;
import motionkit.robot.CartesianCandidateSampler.LatticeCandidate;
import robotkit.manipulation.ArmClearance;
import robotkit.manipulation.ArmClearance.ClearanceViolation;

/** Check only winning routes and disable colliding sample candidates.
 * Sweep edge exclusion and closest-clearance reporting remain separate work. */
class LazyCollisionLadder {
  public static function select(problem:CandidateProblem,clearance:ArmClearance,rounds:Int=8,
      contact:Bool=false,?coarse:CoarseSearchOptions):LadderSelection {
    if(clearance==null)throw "Lazy collision selection requires a clearance world";
    return selectWithChecks(problem,q -> clearance.violation(q,contact),rounds,
      (from,to) -> clearance.sweep(from,to,contact),coarse);
  }
  public static function selectWithChecks(problem:CandidateProblem,
      check:Array<Float>->Null<ClearanceViolation>,rounds:Int=8,
      ?sweep:(Array<Float>,Array<Float>)->Null<ClearanceViolation>,?coarse:CoarseSearchOptions):LadderSelection {
    if(problem==null || check==null || rounds<1)throw "Lazy collision selection requires a problem, checker and positive round budget";
    var blocked=[for(_ in problem.samples)new haxe.ds.ObjectMap<LatticeCandidate,Bool>()];
    var last:Null<ClearanceViolation> = null,lastSample=-1;
    for(round in 0...rounds){
      var route=StructuredLadder.search(problem,null,0,(sample,candidate) ->
        blocked[sample].exists(candidate) ? Math.POSITIVE_INFINITY : 0.0,coarse);
      if(route.diagnostic!=null){
        if(last!=null)throw 'Collision blocks sample $lastSample (${last.a}, ${last.b}): ${route.diagnostic}';
        return route;
      }
      var rejected=false;
      for(i in 0...route.candidates.length){var c=route.candidates[i],failure=check(c.q);
        if(failure!=null){blocked[i].set(c,true);last=failure;lastSample=i;rejected=true;}}
      if(rejected)continue;
      if(sweep!=null)for(i in 1...route.candidates.length){
        var failure=sweep(route.candidates[i-1].q,route.candidates[i].q);
        if(failure!=null)throw 'Collision blocks edge ${i-1} to $i (${failure.a}, ${failure.b}); edge exclusion is required';
      }
      return route;
    }
    if(last!=null)throw 'Collision selection exhausted $rounds rounds at sample $lastSample (${last.a}, ${last.b})';
    throw 'Collision selection exhausted $rounds rounds';
  }
}
