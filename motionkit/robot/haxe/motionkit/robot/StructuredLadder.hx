package motionkit.robot;

import MotionKitNative;
import TrajectoryCore;
import motionkit.robot.CartesianCandidateSampler.LatticeCandidate;

/** Native structured selection over all candidates retained by CandidateProblem. */
class StructuredLadder {
  /** Cheapest complete routes with distinct entry configurations, including current-state cost. */
  public static function kBestStarts(problem:CandidateProblem,count:Int,
      ?weights:Array<Float>,rollWeight:Float=0,?stateCost:(Int,LatticeCandidate)->Float,
      ?coarse:CoarseSearchOptions,?blockedEdges:Array<BlockedLadderEdge>):Array<LadderSelection> {
    if (problem == null || count < 1) throw "Alternative starts require a problem and positive count";
    var routes:Array<LadderSelection> = [];
    var excluded = new haxe.ds.ObjectMap<LatticeCandidate,Bool>();
    for (_ in 0...count) {
      var route = search(problem,weights,rollWeight,(sample,candidate) ->
        sample == 0 && excluded.exists(candidate) ? Math.POSITIVE_INFINITY :
          stateCost == null ? 0.0 : stateCost(sample,candidate),coarse,blockedEdges);
      if (route.diagnostic != null) break;
      routes.push(route);
      // Different IK records can describe the same physical entry (wrap/branch degeneracy).
      var chosen = route.candidates[0];
      for (candidate in problem.samples[0].candidates) {
        var same = true;
        for (joint in 0...chosen.q.length)
          if (Math.abs(candidate.q[joint] - chosen.q[joint]) > 1e-7) same = false;
        if (same) excluded.set(candidate,true);
      }
    }
    return routes;
  }

  public static function search(problem:CandidateProblem,?weights:Array<Float>,rollWeight:Float=0,
      ?stateCost:(Int,LatticeCandidate)->Float,?coarse:CoarseSearchOptions,?blockedEdges:Array<BlockedLadderEdge>,maxPacketBytes:Int=268435456):LadderSelection {
    if(maxPacketBytes<mk_lattice_candidate.size() || maxPacketBytes>268435456)
      throw "Ladder packet budget must hold a candidate and stay within the FFI safety limit";
    return searchRegion(problem,weights,rollWeight,stateCost,coarse,blockedEdges,candidate->true,maxPacketBytes);
  }

  // A gap wider than the jump bound in any joint disconnects the graph.
  // Search those components separately when fixed-capacity ABI records would
  // exceed the FFI packet limit. Every state/edge and its cost is retained.
  static function searchRegion(problem:CandidateProblem,weights:Null<Array<Float>>,rollWeight:Float,
      stateCost:Null<(Int,LatticeCandidate)->Float>,coarse:Null<CoarseSearchOptions>,
      blockedEdges:Null<Array<BlockedLadderEdge>>,allowed:LatticeCandidate->Bool,maxPacketBytes:Int):LadderSelection {
    if(problem==null || problem.samples.length==0)throw "Ladder search requires candidate layers";
    var count=0;
    for(layer in problem.samples)for(candidate in layer.candidates)if(allowed(candidate))count++;
    if(count>Std.int(maxPacketBytes/mk_lattice_candidate.size())) {
      for(joint in 0...problem.request.startQ.length){
        // Values inside a jump-width bin cannot be separated by a legal
        // disconnection. Retain its extremes, rather than sorting one value
        // for every periodic lift at every path sample.
        var bins=new Map<Int,{lower:Float,upper:Float}>(),compact=true;
        var width=problem.request.maxJump[joint];
        for(layer in problem.samples)for(candidate in layer.candidates)if(allowed(candidate) && compact){
          var value=candidate.q[joint],scaled=value/width;
          if(!Math.isFinite(scaled) || scaled < -2147483648.0 || scaled >= 2147483647.0){compact=false;continue;}
          var index=Math.floor(scaled),bin=bins.get(index);
          if(bin==null)bins.set(index,{lower:value,upper:value});
          else {bin.lower=Math.min(bin.lower,value);bin.upper=Math.max(bin.upper,value);}
        }
        // Verify the span rather than assuming floating division has exact
        // bin boundaries. Large coordinates retain the exhaustive fallback.
        for(bin in bins)if(bin.upper-bin.lower>width+1e-12)compact=false;
        var values:Array<Float> = [];
        if(compact)for(bin in bins){values.push(bin.lower);values.push(bin.upper);}
        else for(layer in problem.samples)for(candidate in layer.candidates)if(allowed(candidate))values.push(candidate.q[joint]);
        values.sort((a:Float,b:Float)->a<b?-1:a>b?1:0);
        var boundaries:Array<Float> = [];
        for(i in 1...values.length)if(values[i]-values[i-1]>problem.request.maxJump[joint]+1e-12)
          boundaries.push((values[i]+values[i-1])/2);
        if(boundaries.length==0)continue;
        var best:Null<LadderSelection> = null,failure:Null<LadderSelection> = null;
        for(part in 0...boundaries.length+1){
          var lower=part==0?Math.NEGATIVE_INFINITY:boundaries[part-1];
          var upper=part==boundaries.length?Math.POSITIVE_INFINITY:boundaries[part];
          var axis=joint;
          var result=searchRegion(problem,weights,rollWeight,stateCost,coarse,blockedEdges,
            candidate->allowed(candidate) && candidate.q[axis]>=lower && candidate.q[axis]<upper,maxPacketBytes);
          if(result.diagnostic!=null){if(failure==null || result.failedSample>failure.failedSample)failure=result;continue;}
          var replace=best==null || result.cost<best.cost;
          if(best!=null && result.cost==best.cost){
            // Native DP breaks ties by the lowest final index, then predecessor.
            for(reverse in 0...result.candidates.length){var sample=result.candidates.length-1-reverse;
              var a=problem.samples[sample].candidates.indexOf(result.candidates[sample]);
              var b=problem.samples[sample].candidates.indexOf(best.candidates[sample]);
              if(a!=b){replace=a<b;break;}}
          }
          if(replace)best=result;
        }
        return best==null?failure:best;
      }
      throw "Connected ladder exceeds the FFI packet limit; native streaming candidate search is required";
    }
    var profiling=Sys.getEnv("PROCESS_PATH_PROFILE")=="1",packingStarted=profiling?Sys.time():0.0;
    var path=problem.request,n=path.startQ.length;
    if(weights!=null && weights.length!=n)throw "Ladder weight count must match joints";
    var request=new mk_ladder_request();request.set_struct_size(mk_ladder_request.size());
    request.set_joint_count(n);request.set_external_count(problem.externalJoints.length);
    request.set_roll_count(problem.rollCount);request.set_tilt_count(problem.tiltCount);request.set_azimuth_count(problem.azimuthCount);
    request.set_roll_weight(rollWeight);
    if(coarse!=null){request.set_coarse_sample_stride(coarse.sampleStride);request.set_coarse_lattice_stride(coarse.latticeStride);
      request.set_corridor_radius(coarse.radius);request.set_corridor_widenings(coarse.widenings);}
    for(j in 0...n){request.set_max_jump(j,path.maxJump[j]);request.set_velocity(j,path.velocity[j]);
      request.set_weights(j,weights==null ? 1.0 : weights[j]);request.set_start_joints(j,path.startQ[j]);}
    var samples:Array<mk_configuration_sample> = [],candidates:Array<mk_lattice_candidate> = [],costs:Array<Float> = [];
    var originals:Array<LatticeCandidate> = [];
    var localMaps:Array<Array<Int>> = [];
    for(i in 0...problem.samples.length){var layer=problem.samples[i];
      var sample=new mk_configuration_sample();sample.set_struct_size(mk_configuration_sample.size());sample.set_distance(layer.distance);
      sample.set_first_candidate(candidates.length);samples.push(sample);
      var localMap:Array<Int> = [],localCount=0;localMaps.push(localMap);
      for(c in layer.candidates){
        if(!allowed(c)){localMap.push(-1);continue;}
        localMap.push(localCount++);
        if(c.q.length!=n || c.wraps.length!=n || c.external.length!=problem.externalJoints.length)
          throw "Ladder candidate dimensions must match the problem";
        var record=new mk_lattice_candidate();record.set_struct_size(mk_lattice_candidate.size());
        for(j in 0...n){record.set_joints(j,c.q[j]);record.set_wraps(j,c.wraps[j]);}
        for(j in 0...c.external.length)record.set_external_coordinates(j,c.external[j]);
        record.set_roll_index(c.roll);record.set_tilt_index(c.tilt);record.set_azimuth_index(c.azimuth);
        record.set_branch(c.branch);record.set_singular(c.singular);
        candidates.push(record);originals.push(c);
      }
      sample.set_candidate_count(localCount);
    }
    var exclusions:Array<mk_ladder_edge> = [];
    if(blockedEdges!=null)for(edge in blockedEdges){
      if(edge==null || edge.sample<1 || edge.sample>=problem.samples.length || edge.from<0 || edge.to<0 ||
          edge.from>=problem.samples[edge.sample-1].candidates.length || edge.to>=problem.samples[edge.sample].candidates.length)
        throw "Blocked ladder edge must address adjacent candidate layers";
      var from=localMaps[edge.sample-1][edge.from],to=localMaps[edge.sample][edge.to];
      if(from<0 || to<0)continue;
      var record=new mk_ladder_edge();record.set_struct_size(mk_ladder_edge.size());
      record.set_sample(edge.sample);record.set_from_candidate(from);record.set_to_candidate(to);exclusions.push(record);
    }
    var packingSeconds=profiling?Sys.time()-packingStarted:0.0;
    var costStarted=profiling?Sys.time():0.0;
    for(i in 0...samples.length){var sample=samples[i];
      for(index in sample.get_first_candidate()...sample.get_first_candidate()+sample.get_candidate_count())
        costs.push(stateCost==null?0.0:stateCost(i,originals[index]));
    }
    var costSeconds=profiling?Sys.time()-costStarted:0.0;
    var nativeStarted=profiling?Sys.time():0.0;
    var result=MotionKitNative.mk_search_ladder_filtered(request,samples,candidates,costs,exclusions);
    if(profiling)Sys.println("PROCESS_PATH_LADDER_PROFILE "+haxe.Json.stringify({
      samples:samples.length,candidates:candidates.length,packetBytes:candidates.length*mk_lattice_candidate.size(),
      packingSeconds:packingSeconds,stateCostSeconds:costSeconds,nativeCallSeconds:Sys.time()-nativeStarted}));
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
  public final closestClearance:Null<robotkit.manipulation.ArmClearance.ClearanceViolation>;
  public function new(candidates:Array<LatticeCandidate>,cost:Float,failedSample:Int=-1,failedDistance:Float=0,?diagnostic:String,?closestClearance:robotkit.manipulation.ArmClearance.ClearanceViolation){
    this.candidates=candidates;this.cost=cost;this.failedSample=failedSample;this.failedDistance=failedDistance;this.diagnostic=diagnostic;this.closestClearance=closestClearance;
  }
  public function joints():Array<Array<Float>> return [for(candidate in candidates)candidate.q.copy()];
}

/** Approximate corridor optimization; disconnected corridors widen or revert to full search. */
class CoarseSearchOptions {
  public final sampleStride:Int;
  public final latticeStride:Int;
  public final radius:Int;
  public final widenings:Int;
  public function new(sampleStride:Int=10,latticeStride:Int=4,radius:Int=2,widenings:Int=2){
    if(sampleStride<1 || latticeStride<1 || radius<1 || widenings<0 || widenings>16)throw "Invalid coarse ladder resolution";
    this.sampleStride=sampleStride;this.latticeStride=latticeStride;this.radius=radius;this.widenings=widenings;
  }
}

/** Candidate indices are local to adjacent source layers. */
class BlockedLadderEdge {
  public final sample:Int;
  public final from:Int;
  public final to:Int;
  public function new(sample:Int,from:Int,to:Int){this.sample=sample;this.from=from;this.to=to;}
}
