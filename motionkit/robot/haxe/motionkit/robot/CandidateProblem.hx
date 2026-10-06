package motionkit.robot;

import motionkit.kinematics.SixAxisConfiguration;

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
  /** Hard branch/lift constraint, also applied to refinement re-solves. */
  public final configuration:Null<SixAxisConfiguration>;
  public final rollCount:Int;
  public final tiltCount:Int;
  public final azimuthCount:Int;

  /** Conservative forward reachability bounds. A state outside any joint's
   * predecessor envelope cannot belong to a legal complete route. */
  public function pruneUnreachableBounds():Void {
    for(index in 1...samples.length){
      var previous=samples[index-1].candidates;
      if(previous.length==0)continue;
      var lower=previous[0].q.copy(),upper=lower.copy();
      for(candidate in previous)for(joint in 0...lower.length){
        lower[joint]=Math.min(lower[joint],candidate.q[joint]);
        upper[joint]=Math.max(upper[joint],candidate.q[joint]);
      }
      var layer=samples[index].candidates,retained=0;
      for(candidate in layer){
        var legal=true;
        for(joint in 0...lower.length)
          if(candidate.q[joint]<lower[joint]-request.maxJump[joint]-1e-12 ||
              candidate.q[joint]>upper[joint]+request.maxJump[joint]+1e-12)legal=false;
        if(legal)layer[retained++]=candidate;
      }
      layer.resize(retained);
    }
  }

  public function new(group:KinematicGroup,request:PathRequest,?options:CandidateSamplingOptions,
      boundedConstruction:Bool=false) {
    if(group==null || request==null || request.startQ.length!=group.group.count())
      throw "Candidate problem requires a group and complete path request";
    var settings=options==null ? new CandidateSamplingOptions() : options;
    this.request=request;this.pinnedStart=settings.pinStart;
    this.configuration=settings.configuration;
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
    var profile=Sys.getEnv("PROCESS_PATH_PROFILE")=="1",began=profile?Sys.time():0.0,lastReport=began;
    var queriesBefore=profile?group.numericSolveCount():0,totalCandidates=0;
    if(profile)Sys.println("PROCESS_PATH_BUILD_START "+haxe.Json.stringify({family:family,
      diagnostic:diagnostic,samples:request.poses.length,externalAxes:externalJoints.length,pinnedStart:pinnedStart,
      configuration:settings.configuration==null ? null : cast(settings.configuration,SixAxisConfiguration).label()}));
    samples=[];
    var initialPose=backend.forward(request.startQ);
    var neighbours=[request.startQ.copy()];
    for(index in 0...request.poses.length) {
      var target=request.poses[index],freedom=request.freedoms[index];
      var rule = settings.externalRule;
      var ranges=rule==null ? settings.externalRanges : rule(index,request.distances[index],target);
      if(ranges==null)throw "External rule must return ranges for every sample";
      var samplingTarget=target,samplingFreedom=freedom;
      switch freedom {
        case Free:
          // Keep the unconstrained rotation lattice in one frame throughout
          // the path. Authored rotations remain available for state costs.
          samplingTarget=new Pose3(target.x,target.y,target.z,
            initialPose.qx,initialPose.qy,initialPose.qz,initialPose.qw);
        default:
      }
      var gridTarget=samplingTarget;
      if(index==0 && settings.pinStart) {
        samplingTarget=initialPose;
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
        var bounds:Null<{lower:Array<Float>,upper:Array<Float>}> = null;
        var pinnedJump=index==0 && settings.pinStart && boundedConstruction ? [for(_ in request.startQ)1e-7] : null;
        if(boundedConstruction && index>0 && samples[index-1].candidates.length>0){
          var previous=samples[index-1].candidates;
          var lower=previous[0].q.copy(),upper=lower.copy();
          for(candidate in previous)for(j in 0...lower.length){
            lower[j]=Math.min(lower[j],candidate.q[j]);upper[j]=Math.max(upper[j],candidate.q[j]);
          }
          for(j in 0...lower.length){lower[j]-=request.maxJump[j]+1e-12;upper[j]+=request.maxJump[j]+1e-12;}
          bounds={lower:lower,upper:upper};
        }
        candidates=serial.sample(samplingTarget,request.startQ,samplingFreedom,ranges,
          settings.rollCount,settings.tiltRings,settings.azimuthCount,pinnedJump,bounds);
      } else {
        candidates=[];
        // Reduced tasks use the same explicit orientation lattice as analytic
        // families. A reduced numeric solve per neighbour would invent new free
        // spins at every layer instead of sampling that declared finite lattice.
        var orientations=OrientationLattice.sample(samplingTarget,samplingFreedom,
          settings.rollCount,settings.tiltRings,settings.azimuthCount);
        for(cell in ExternalAxisGrid.sample(group,request.startQ,ranges))for(orientation in orientations)
          for(branch in fallback.branchesFromNeighbours(orientation.pose,neighbours,cell.q,OrientationPolicy.Fixed))
            candidates.push(new LatticeCandidate(branch.q,[for(_ in branch.q)0],cell.coordinates,
              orientation.roll,orientation.tilt,orientation.azimuth,branch.branch,0,false));
      }
      var configurationPin=settings.configuration;
      if(configurationPin!=null){
        var pin:SixAxisConfiguration=cast configurationPin;
        if(cart!=null || fallback!=null)throw "Configuration pin requires a labelled six-axis geometric backend";
        candidates=[for(candidate in candidates)if(pin.accepts(candidate.configuration))candidate];
      }
      if(index==0 && settings.pinStart) {
        var selected:Array<LatticeCandidate> = [];
        var initialCell:Null<motionkit.robot.OrientationLattice.OrientationCell> = null;
        if(!ToolFreedom.isFull(freedom)){
          var nearest=Math.POSITIVE_INFINITY;
          for(cell in OrientationLattice.sample(gridTarget,freedom,settings.rollCount,settings.tiltRings,settings.azimuthCount)){
            var angle=PoseMath.angle(samplingTarget,cell.pose);
            if(angle<nearest-1e-12){nearest=angle;initialCell=cell;}
          }
        }
        for(candidate in candidates) {
          var same=true;
          for(joint in 0...request.startQ.length)
            if(Math.abs(candidate.q[joint]-request.startQ[joint])>1e-7)same=false;
          if(same && selected.length==0)selected.push(new LatticeCandidate(request.startQ.copy(),candidate.wraps.copy(),
            candidate.external.copy(),initialCell==null ? candidate.roll : initialCell.roll,
            initialCell==null ? candidate.tilt : initialCell.tilt,
            initialCell==null ? candidate.azimuth : initialCell.azimuth,candidate.branch,candidate.singular,candidate.singularityKnown,candidate.configuration));
        }
        candidates=selected;
      }
      samples.push(new CandidateLayer(request.distances[index],target,freedom,candidates));
      if(profile){
        totalCandidates+=candidates.length;
        var now=Sys.time();
        if(now-lastReport>=1.0 || index==request.poses.length-1){
          Sys.println("PROCESS_PATH_BUILD_PROGRESS "+haxe.Json.stringify({family:family,
            completedSamples:index+1,samples:request.poses.length,candidates:totalCandidates,
            numericIkSolves:group.numericSolveCount()-queriesBefore,seconds:now-began}));
          lastReport=now;
        }
      }
      // Only the explicit numerical fallback consumes predecessor seeds.
      // Analytic sampling uses its declared complete native lattice instead.
      if(fallback!=null){
        neighbours=[for(candidate in candidates)candidate.q.copy()];
        if(neighbours.length==0)neighbours=[request.startQ.copy()];
      }
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
  public final configuration:Null<SixAxisConfiguration>;
  public function new(rollCount:Int=12,tiltRings:Int=3,azimuthCount:Int=8,pinStart:Bool=true,
      ?externalRanges:Array<ExternalAxisRange>,?externalRule:(Int,Float,Pose3)->Array<ExternalAxisRange>,
      ?configuration:SixAxisConfiguration) {
    if(rollCount<=0 || tiltRings<=0 || azimuthCount<=0)throw "Candidate lattice resolutions must be positive";
    this.rollCount=rollCount;this.tiltRings=tiltRings;this.azimuthCount=azimuthCount;this.pinStart=pinStart;
    this.externalRanges=externalRanges==null ? [] : externalRanges.copy();this.externalRule=externalRule;this.configuration=configuration;
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
