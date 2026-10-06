package motionkit.robot;

import motionkit.robot.StructuredLadder.LadderSelection;
import motionkit.robot.StructuredLadder.BlockedLadderEdge;
import motionkit.robot.StructuredLadder.CoarseSearchOptions;
import motionkit.robot.CartesianCandidateSampler.LatticeCandidate;
import robotkit.manipulation.ArmClearance;
import robotkit.manipulation.ArmClearance.ClearanceViolation;

/** Check only winning routes and disable colliding sample candidates.
 * Failed sweeps exclude transitions; final sweeps report sampled clearance. */
class LazyCollisionLadder {
  public static function select(problem:CandidateProblem,clearance:ArmClearance,rounds:Int=8,
      contact:Bool=false,?coarse:CoarseSearchOptions):LadderSelection {
    if(clearance==null)throw "Lazy collision selection requires a clearance world";
    var selected=selectWithChecks(problem,q -> clearance.violation(q,contact),rounds,
      (from,to) -> clearance.sweep(from,to,contact),coarse);
    if(selected.diagnostic!=null)return selected;
    var closest:Null<ClearanceViolation> = null;
    for(i in 0...selected.candidates.length){
      var found=i==0 ? clearance.closest(selected.candidates[i].q,contact)
        : clearance.closestSweep(selected.candidates[i-1].q,selected.candidates[i].q,contact);
      if(found!=null && (closest==null || found.distance<closest.distance))closest=found;
    }
    return new LadderSelection(selected.candidates,selected.cost,-1,0,null,closest);
  }
  public static function selectWithChecks(problem:CandidateProblem,
      check:Array<Float>->Null<ClearanceViolation>,rounds:Int=8,
      ?sweep:(Array<Float>,Array<Float>)->Null<ClearanceViolation>,?coarse:CoarseSearchOptions,
      ?refinedCheck:LadderSelection->Null<RefinedCollision>,
      ?stateCost:(Int,LatticeCandidate)->Float):LadderSelection {
    if(problem==null || check==null || rounds<1)throw "Lazy collision selection requires a problem, checker and positive round budget";
    var blocked=[for(_ in problem.samples)new haxe.ds.ObjectMap<LatticeCandidate,Bool>()];
    var edges:Array<BlockedLadderEdge> = [];
    var last:Null<ClearanceViolation> = null,lastSample=-1;
    for(round in 0...rounds){
      var route=StructuredLadder.search(problem,null,0,(sample,candidate) ->
        blocked[sample].exists(candidate) ? Math.POSITIVE_INFINITY :
          stateCost == null ? 0.0 : stateCost(sample,candidate),coarse,edges);
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
        if(failure!=null){
          var from=problem.samples[i-1].candidates.indexOf(route.candidates[i-1]);
          var to=problem.samples[i].candidates.indexOf(route.candidates[i]);
          edges.push(new BlockedLadderEdge(i,from,to));last=failure;lastSample=i;rejected=true;
        }
      }
      if(rejected)continue;
      if(refinedCheck!=null){var failure=refinedCheck(route);
        if(failure!=null){var i=failure.sample;
          if(i<0 || i>=route.candidates.length || (failure.edge && i==0))throw "Refined collision must address a route sample or incoming edge";
          if(failure.edge)edges.push(new BlockedLadderEdge(i,
            problem.samples[i-1].candidates.indexOf(route.candidates[i-1]),problem.samples[i].candidates.indexOf(route.candidates[i])));
          else blocked[i].set(route.candidates[i],true);
          last=failure.failure;lastSample=i;continue;
        }
      }
      return route;
    }
    if(last!=null)throw 'Collision selection exhausted $rounds rounds at sample $lastSample (${last.a}, ${last.b})';
    throw 'Collision selection exhausted $rounds rounds';
  }
}

/** A refined sample failure or the sweep arriving at that sample. */
class RefinedCollision {
  public final sample:Int;
  public final edge:Bool;
  public final failure:ClearanceViolation;
  public function new(sample:Int,edge:Bool,failure:ClearanceViolation){
    if(failure==null)throw "Refined collision requires a blocking pair";
    this.sample=sample;this.edge=edge;this.failure=failure;
  }
}
