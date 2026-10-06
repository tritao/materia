package motionkit.robot;

import motionkit.kinematics.PathRequest;
import motionkit.kinematics.Pose3;
import motionkit.path.PoseMath;
import motionkit.path.OrientationPolicy;
import motionkit.robot.CartesianCandidateSampler.LatticeCandidate;
import motionkit.robot.ExternalAxisGrid.ExternalAxisRange;
import robotkit.manipulation.KinematicGroup;

/** Candidate ladder construction; discrete selection and timing are separate. */
class CandidateProblem {
  public final request:PathRequest;
  public final family:String;
  public final diagnostic:Null<String>;
  public final samples:Array<CandidateLayer>;
  public final externalJoints:Array<Int>;
  public final pinnedStart:Bool;
  public final rollCount:Int;
  public final tiltCount:Int;
  public final azimuthCount:Int;

  public function new(group:KinematicGroup,request:PathRequest,?options:CandidateSamplingOptions) {
    if(group==null || request==null || request.startQ.length!=group.group.count())
      throw "Candidate problem requires a group and complete path request";
    var settings=options==null ? new CandidateSamplingOptions() : options;
    this.request=request;this.pinnedStart=settings.pinStart;
    rollCount=settings.rollCount;tiltCount=settings.tiltRings+1;azimuthCount=settings.azimuthCount;
    for(i in 0...request.distances.length) {
      if(!Math.isFinite(request.distances[i]) || request.distances[i]<0 ||
          (i>0 && request.distances[i]<=request.distances[i-1]) || request.poses[i]==null)
        throw "Candidate path samples require increasing finite distances and poses";
    }
    for(i in 0...request.startQ.length)
      if(!Math.isFinite(request.startQ[i]) || !Math.isFinite(request.maxJump[i]) || request.maxJump[i]<=0 ||
          !Math.isFinite(request.velocity[i]) || request.velocity[i]<=0)
        throw "Candidate path requires finite joints and positive jump/speed limits";
    var backend=BranchIk.of(group,request.tolerance);
    family=backend.family();
    externalJoints=Std.isOfType(backend,CartesianAnalyticIk) ? [] : [for(i in 0...group.group.count())if(group.external[i])i];
    var fallback:Null<NumericBranchIk> = Std.isOfType(backend,NumericBranchIk) ? cast backend : null;
    diagnostic=fallback==null ? null : fallback.diagnostic;
    var cart:Null<CartesianCandidateSampler> = Std.isOfType(backend,CartesianAnalyticIk) ? new CartesianCandidateSampler(group) : null;
    var serial:Null<SerialCandidateSampler> = cart==null && fallback==null ? new SerialCandidateSampler(group) : null;
    samples=[];
    var neighbours=[request.startQ.copy()];
    for(index in 0...request.poses.length) {
      var target=request.poses[index],freedom=request.freedoms[index];
      var rule = settings.externalRule;
      var ranges=rule==null ? settings.externalRanges : rule(index,request.distances[index],target);
      if(ranges==null)throw "External rule must return ranges for every sample";
      var samplingTarget=target,samplingFreedom=freedom;
      if(index==0 && settings.pinStart) {
        samplingTarget=backend.forward(request.startQ);
        if(PoseMath.distance(samplingTarget,target)>request.tolerance.position ||
            ToolFreedom.orientationError(samplingTarget,target,freedom)>request.tolerance.orientation)
          throw "Pinned start does not satisfy the first task sample";
        // The start's exact spin must not depend on a discretized cone/roll grid.
        samplingFreedom=OrientationPolicy.Fixed;
      }
      var candidates:Array<LatticeCandidate>;
      if(cart!=null) {
        if(ranges.length>0)throw "Standalone Cartesian candidate group has no external ranges";
        candidates=cart.sample(samplingTarget,request.startQ,samplingFreedom,settings.rollCount,settings.tiltRings,settings.azimuthCount);
      } else if(serial!=null) {
        candidates=serial.sample(samplingTarget,request.startQ,samplingFreedom,ranges,settings.rollCount,settings.tiltRings,settings.azimuthCount);
      } else {
        candidates=[];
        // Unsupported geometry uses the explicitly diagnosed numeric fallback.
        for(cell in ExternalAxisGrid.sample(group,request.startQ,ranges))
          for(branch in fallback.branchesFromNeighbours(samplingTarget,neighbours,cell.q,samplingFreedom))
            candidates.push(new LatticeCandidate(branch.q,[for(_ in branch.q)0],cell.coordinates,0,0,0,branch.branch,0,false));
      }
      if(index==0 && settings.pinStart) {
        var selected:Array<LatticeCandidate> = [];
        for(candidate in candidates) {
          var same=true;
          for(joint in 0...request.startQ.length)
            if(Math.abs(candidate.q[joint]-request.startQ[joint])>1e-7)same=false;
          if(same && selected.length==0)selected.push(new LatticeCandidate(request.startQ.copy(),candidate.wraps.copy(),
            candidate.external.copy(),candidate.roll,candidate.tilt,candidate.azimuth,candidate.branch,candidate.singular,candidate.singularityKnown));
        }
        candidates=selected;
      }
      samples.push(new CandidateLayer(request.distances[index],target,freedom,candidates));
      neighbours=[for(candidate in candidates)candidate.q.copy()];
      if(neighbours.length==0)neighbours=[request.startQ.copy()];
    }
  }
}
class CandidateSamplingOptions {
  public final rollCount:Int;
  public final tiltRings:Int;
  public final azimuthCount:Int;
  public final pinStart:Bool;
  public final externalRanges:Array<ExternalAxisRange>;
  public final externalRule:Null<(Int,Float,Pose3)->Array<ExternalAxisRange>>;
  public function new(rollCount:Int=12,tiltRings:Int=3,azimuthCount:Int=8,pinStart:Bool=true,
      ?externalRanges:Array<ExternalAxisRange>,?externalRule:(Int,Float,Pose3)->Array<ExternalAxisRange>) {
    if(rollCount<=0 || tiltRings<=0 || azimuthCount<=0)throw "Candidate lattice resolutions must be positive";
    this.rollCount=rollCount;this.tiltRings=tiltRings;this.azimuthCount=azimuthCount;this.pinStart=pinStart;
    this.externalRanges=externalRanges==null ? [] : externalRanges.copy();this.externalRule=externalRule;
  }
}
class CandidateLayer {
  public final distance:Float;
  public final target:Pose3;
  public final freedom:OrientationPolicy;
  public final candidates:Array<LatticeCandidate>;
  public function new(distance:Float,target:Pose3,freedom:OrientationPolicy,candidates:Array<LatticeCandidate>) {
    this.distance=distance;this.target=target;this.freedom=freedom;this.candidates=candidates;
  }
}
