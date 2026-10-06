package motionkit.robot;

import MotionKitNative;
import TrajectoryCore;
import motionkit.robot.CartesianCandidateSampler.LatticeCandidate;

/** Native structured selection over all candidates retained by CandidateProblem. */
class StructuredLadder {
  public static function search(problem:CandidateProblem,?weights:Array<Float>,rollWeight:Float=0,
      ?stateCost:(Int,LatticeCandidate)->Float):LadderSelection {
    if(problem==null || problem.samples.length==0)throw "Ladder search requires candidate layers";
    var path=problem.request,n=path.startQ.length;
    if(weights!=null && weights.length!=n)throw "Ladder weight count must match joints";
    var request=new mk_ladder_request();request.set_struct_size(mk_ladder_request.size());
    request.set_joint_count(n);request.set_external_count(problem.externalJoints.length);
    request.set_roll_count(problem.rollCount);request.set_tilt_count(problem.tiltCount);request.set_azimuth_count(problem.azimuthCount);
    request.set_roll_weight(rollWeight);
    for(j in 0...n){request.set_max_jump(j,path.maxJump[j]);request.set_velocity(j,path.velocity[j]);
      request.set_weights(j,weights==null ? 1.0 : weights[j]);request.set_start_joints(j,path.startQ[j]);}
    var samples:Array<mk_configuration_sample> = [],candidates:Array<mk_lattice_candidate> = [],costs:Array<Float> = [];
    var originals:Array<LatticeCandidate> = [];
    for(i in 0...problem.samples.length){var layer=problem.samples[i];
      var sample=new mk_configuration_sample();sample.set_struct_size(mk_configuration_sample.size());sample.set_distance(layer.distance);
      sample.set_first_candidate(candidates.length);sample.set_candidate_count(layer.candidates.length);samples.push(sample);
      for(c in layer.candidates){
        if(c.q.length!=n || c.wraps.length!=n || c.external.length!=problem.externalJoints.length)
          throw "Ladder candidate dimensions must match the problem";
        var record=new mk_lattice_candidate();record.set_struct_size(mk_lattice_candidate.size());
        for(j in 0...n){record.set_joints(j,c.q[j]);record.set_wraps(j,c.wraps[j]);}
        for(j in 0...c.external.length)record.set_external_coordinates(j,c.external[j]);
        record.set_roll_index(c.roll);record.set_tilt_index(c.tilt);record.set_azimuth_index(c.azimuth);
        record.set_branch(c.branch);record.set_singular(c.singular);
        candidates.push(record);originals.push(c);costs.push(stateCost==null ? 0.0 : stateCost(i,c));
      }
    }
    var result=MotionKitNative.mk_search_ladder(request,samples,candidates,costs);
    if(result.status==TrajectoryCoreConstants.MK_ERROR_GENERATION){var report=result.out_result;
      return new LadderSelection([],report.get_cost(),report.get_failed_sample(),report.get_failed_distance(),
        report.get_failure_kind()==1 ? "no candidates (unreachable or joint limits)" : "no legal edges (jump or disabled states)");}
    if(result.status!=TrajectoryCoreConstants.MK_OK)throw 'Native structured ladder failed: ${result.status}';
    return new LadderSelection([for(index in result.out_indices)originals[index]],result.out_result.get_cost());
  }
}
class LadderSelection {
  public final candidates:Array<LatticeCandidate>;
  public final cost:Float;
  public final failedSample:Int;
  public final failedDistance:Float;
  public final diagnostic:Null<String>;
  public function new(candidates:Array<LatticeCandidate>,cost:Float,failedSample:Int=-1,failedDistance:Float=0,?diagnostic:String){
    this.candidates=candidates;this.cost=cost;this.failedSample=failedSample;this.failedDistance=failedDistance;this.diagnostic=diagnostic;
  }
  public function joints():Array<Array<Float>> return [for(candidate in candidates)candidate.q.copy()];
}
