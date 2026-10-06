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
 * Unsupported families use explicitly diagnosed numeric continuation.
 * contactPose is a pure worker-safe policy over TCP poses in the task frame;
 * it overrides contact at every clearance sample, including swept interiors. */
class StructuredJointPathPlanner implements JointPathPlanner {
  public final group:KinematicGroup;
  /** Cost of the last successfully selected complete discrete route. */
  public var selectionCost(default,null):Null<Float> = null;
  public var fallbackDiagnostic(default,null):Null<String> = null;
  final sampling:Null<CandidateSamplingOptions>;
  final coarse:Null<CoarseSearchOptions>;
  final clearance:Null<ArmClearance>;
  final collisionRounds:Int;
  final contact:Bool;
  final contactPose:Null<motionkit.kinematics.Pose3->Bool>;
  final retreat:Null<Array<Float>>;
  final weights:Null<Array<Float>>;
  final rollWeight:Float;
  final preferenceSource:Null<ManipulatorKinematics>;
  final stateCost:Null<(Int,motionkit.robot.CartesianCandidateSampler.LatticeCandidate)->Float>;
  public function new(group:KinematicGroup,?sampling:CandidateSamplingOptions,?coarse:CoarseSearchOptions,
      ?clearance:ArmClearance,collisionRounds:Int=8,contact:Bool=false,
      ?stateCost:(Int,motionkit.robot.CartesianCandidateSampler.LatticeCandidate)->Float,
      ?retreat:Array<Float>,?weights:Array<Float>,rollWeight:Float=0,?preferenceSource:ManipulatorKinematics,?contactPose:motionkit.kinematics.Pose3->Bool) {
    if(group==null || collisionRounds<1)throw "Joint path planner requires a compiled group and positive collision round budget";
    if (!Math.isFinite(rollWeight) || rollWeight < 0)
      throw "Roll motion cost must be finite and nonnegative";
    if (weights != null) {
      if (weights.length != group.group.count()) throw "Joint motion weights must match compiled joints";
      for (weight in weights) if (!Math.isFinite(weight) || weight < 0)
        throw "Joint motion weights must be finite and nonnegative";
    }
    if (preferenceSource != null && preferenceSource.manipulator != group)
      throw "Planner preferences must belong to its compiled group";
    this.preferenceSource = preferenceSource;
    this.contactPose = contactPose;
    this.weights = weights == null ? null : weights.copy();
    this.rollWeight = rollWeight;
    if (retreat != null) {
      if (retreat.length != group.group.count()) throw "Retreat target must match compiled joints";
      for (joint in 0...retreat.length) {
        var limits = group.group.limitsOf(joint);
        if (!Math.isFinite(retreat[joint]) || limits.lower < limits.upper &&
            (retreat[joint] < limits.lower || retreat[joint] > limits.upper))
          throw 'Retreat target violates joint $joint planning limits';
      }
    }
    this.retreat = retreat == null ? null : retreat.copy();
    this.group=group;this.sampling=sampling;this.coarse=coarse;
    this.clearance=clearance;this.collisionRounds=collisionRounds;this.contact=contact;this.stateCost=stateCost;
  }
  public function withConfiguration(configuration:motionkit.kinematics.SixAxisConfiguration):JointPathPlanner {
    if(configuration==null)throw "Configuration pin is required";
    var original=sampling==null ? new CandidateSamplingOptions() : sampling;
    if(original.configuration!=null){
      var existing:motionkit.kinematics.SixAxisConfiguration=cast original.configuration;
      var geometric=new motionkit.kinematics.SixAxisConfiguration(configuration.shoulder,configuration.elbow,configuration.wrist);
      if(!geometric.accepts(existing) || existing.turns!=null && configuration.turns!=null && !existing.accepts(configuration))
        throw "Conflicting planner and program configuration pins";
      if(configuration.turns==null)configuration=existing;
    }
    var settings=new CandidateSamplingOptions(original.rollCount,original.tiltRings,original.azimuthCount,
      original.pinStart,original.externalRanges,original.externalRule,configuration);
    return new StructuredJointPathPlanner(group,settings,coarse,clearance,collisionRounds,contact,stateCost,
      retreat,weights,rollWeight,preferenceSource,contactPose);
  }
  public function withSolver(solver:motionkit.kinematics.KinematicsSolver):JointPathPlanner {
    var workerGroup:KinematicGroup;
    if (Std.isOfType(solver, ManipulatorKinematics)) {
      var adapter:ManipulatorKinematics = cast solver;
      workerGroup = adapter.manipulator;
    } else if (Std.isOfType(solver, OpwKinematics)) {
      var adapter:OpwKinematics = cast solver;
      workerGroup = adapter.manipulator;
    } else throw "Structured planner worker requires compiled group kinematics";
    return new StructuredJointPathPlanner(workerGroup, sampling, coarse,
      clearance == null ? null : clearance.withGroup(workerGroup), collisionRounds, contact, stateCost, retreat, weights, rollWeight,
      preferenceSource == null ? null : Std.isOfType(solver,ManipulatorKinematics) ? cast solver :
        throw "Planner preference worker requires manipulator kinematics",contactPose);
  }
  public static function sameFreedom(a:motionkit.path.OrientationPolicy,b:motionkit.path.OrientationPolicy):Bool {
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
  public function retreatTarget():Null<Array<Float>> return retreat == null ? null : retreat.copy();
  public function allowsFreeStart():Bool return sampling != null && !sampling.pinStart;
  function contactAt(q:Array<Float>):Bool {
    if (contactPose == null) return contact;
    var fk = group.tcpPose(q);
    return contactPose(new motionkit.kinematics.Pose3(fk.translation.x,fk.translation.y,fk.translation.z,
      fk.rotation.x,fk.rotation.y,fk.rotation.z,fk.rotation.w));
  }
  public function checkMotion(trajectory:motionkit.trajectory.Trajectory):Null<ArmClearance.ClearanceViolation>
    return clearance == null ? null : TrajectoryClearance.violation(clearance,trajectory,contact,0.01,contactPose == null ? null : contactAt);
  public function plan(path:PosePath,request:PathRequest,?pinStart:Bool,
      ?entryCheck:(Array<Float>,Array<Float>)->Null<ArmClearance.ClearanceViolation>,
      ?exitCheck:Array<Float>->Null<ArmClearance.ClearanceViolation>):JointPathSamples {
    return planSections([path],request,pinStart,entryCheck,exitCheck)[0];
  }

  /** One ladder over all timing sections; stop derivatives remain per section. */
  public function planSections(paths:Array<PosePath>,request:PathRequest,?pinStart:Bool,
      ?entryCheck:(Array<Float>,Array<Float>)->Null<ArmClearance.ClearanceViolation>,
      ?exitCheck:Array<Float>->Null<ArmClearance.ClearanceViolation>):Array<JointPathSamples> {
    selectionCost=null;
    if(paths==null || paths.length==0 || request==null || request.distances.length<2 || request.distances[0]!=0)
      throw "Joint path sections require a complete sampled request";
    var offsets:Array<Float> = [],providers:Array<PosePathRefinement> = [],total=0.0;
    for(path in paths){
      if(path==null)throw "Missing authored path section";
      if(path.frameId!=paths[0].frameId)throw "Joint path sections must share a task frame";
      offsets.push(total);providers.push(new PosePathRefinement(path));total+=path.length();
    }
    if(request.distances[request.distances.length-1]!=total)
      throw "Joint path request must span all authored sections";
    // Require every stop as a ladder knot. At a stop, the outgoing task is
    // sampled; the incoming task must describe the same hard geometry.
    for(section in 1...paths.length){
      if(request.distances.indexOf(offsets[section])<0)throw "Timing stop must be a sampled route knot";
      var incoming=paths[section-1].primitives[paths[section-1].primitives.length-1];
      var outgoing=paths[section].primitives[0];
      if(!sameFreedom(incoming.orientationPolicy(),outgoing.orientationPolicy()) ||
          PoseMath.distance(incoming.endWaypoint().pose,outgoing.startWaypoint().pose)>request.tolerance.position ||
          ToolFreedom.orientationError(incoming.endWaypoint().pose,outgoing.startWaypoint().pose,
            outgoing.orientationPolicy())>request.tolerance.orientation)
        throw "Timing sections must share their hard endpoint task";
    }
    for(i in 0...request.distances.length){
      var section=paths.length-1;
      for(k in 1...paths.length)if(request.distances[i]<offsets[k]){section=k-1;break;}
      var local=Math.max(0.0,Math.min(paths[section].length(),request.distances[i]-offsets[section]));
      var task=providers[section].at(local);
      if(!sameFreedom(task.freedom,request.freedoms[i]))
        throw 'Joint path request differs from authored freedom at sample $i';
      if(PoseMath.distance(task.pose,request.poses[i])>request.tolerance.position ||
          ToolFreedom.orientationError(task.pose,request.poses[i],task.freedom)>request.tolerance.orientation)
        throw 'Joint path request differs from authored geometry at sample $i';
    }
    var settings = sampling;
    if (pinStart != null) {
      var original = settings == null ? new CandidateSamplingOptions() : settings;
      settings = new CandidateSamplingOptions(original.rollCount,original.tiltRings,original.azimuthCount,
        pinStart,original.externalRanges,original.externalRule,original.configuration);
    }
    var profile=Sys.getEnv("PROCESS_PATH_PROFILE")=="1";
    var buildStarted=profile ? Sys.time() : 0.0;
    var problem=new CandidateProblem(group,request,settings,true);
    problem.pruneUnreachableBounds();
    var buildSeconds=profile ? Sys.time()-buildStarted : 0.0;
    var refinementSeconds=0.0,refinementAttempts=0;
    fallbackDiagnostic=problem.diagnostic;
    var refined:Null<Array<JointPathSamples>> = null;
    function refine(route:motionkit.robot.StructuredLadder.LadderSelection):Array<JointPathSamples> {
      var refiner=new AnalyticPathRefiner(group,problem,route),curves:Array<JointPathSamples> = [];
      for(section in 0...paths.length){
        var offset=offsets[section],end=section+1<paths.length?offsets[section+1]:request.distances[request.distances.length-1];
        var distances=[for(distance in request.distances)if(distance>=offset && distance<=end)distance];
        var provider=providers[section],length=paths[section].length();
        var curve=refiner.refineSection(distances,distance->provider.at(Math.max(0.0,Math.min(length,distance-offset))));
        var local=[for(distance in curve.s)Math.max(0.0,Math.min(length,distance-offset))];
        local[local.length-1]=length;
        curves.push(new JointPathSamples(local,curve.q,curve.qPrime,curve.qDoublePrime,curve.qDoublePrimeBefore));
      }
      return curves;
    }
    var world=clearance;
    var preferences = preferenceSource == null ? null : new JointPathPreferences(preferenceSource);
    var cost = preferences == null ? stateCost :
      (sample:Int,candidate:motionkit.robot.CartesianCandidateSampler.LatticeCandidate) ->
        (stateCost == null ? 0.0 : stateCost(sample,candidate)) +
          preferences.cost(group,request.poses[sample],candidate.q);
    if (exitCheck != null && retreat != null) {
      var destination:Array<Float> = retreat.copy();
      var precedingCost = cost;
      cost = (sample:Int,candidate:motionkit.robot.CartesianCandidateSampler.LatticeCandidate) -> {
        var value = precedingCost == null ? 0.0 : precedingCost(sample,candidate);
        if (sample == problem.samples.length - 1)
          for (joint in 0...destination.length) value += (weights == null ? 1.0 : weights[joint])*Math.abs(destination[joint]-candidate.q[joint])/request.velocity[joint];
        return value;
      };
    }
    var searchStarted=profile ? Sys.time() : 0.0;
    var selected=world==null ? StructuredLadder.search(problem,weights,rollWeight,cost,coarse)
      : LazyCollisionLadder.selectWithChecks(problem,q -> world.violation(q,contactAt(q)),collisionRounds,
        (from,to) -> world.sweep(from,to,contact,0.02,null,contactPose == null ? null : contactAt),coarse,route -> {
          var refinementStarted=profile ? Sys.time() : 0.0;
          var curves=refine(route);
          if(profile){refinementSeconds+=Sys.time()-refinementStarted;refinementAttempts++;}
          for(section in 0...curves.length){
            var curve=curves[section],base=request.distances.indexOf(offsets[section]);
            for(i in 0...curve.q.length){
              var sample=base+i;
              var failure=world.violation(curve.q[i],contactAt(curve.q[i]));
              if(failure!=null)return new motionkit.robot.LazyCollisionLadder.RefinedCollision(sample,false,failure);
              if(i>0){failure=world.sweep(curve.q[i-1],curve.q[i],contact,0.02,null,contactPose == null ? null : contactAt);
                if(failure!=null)return new motionkit.robot.LazyCollisionLadder.RefinedCollision(sample,true,failure);}
            }
          }
          refined=curves;return null;
        },cost, problem.pinnedStart ? null : entryCheck != null ? entryCheck : (from,to) -> world.sweep(from,to,contact,0.02,null,contactPose == null ? null : contactAt),exitCheck,weights,rollWeight);
    if(selected.diagnostic!=null)throw 'Joint path selection failed at distance ${selected.failedDistance}: ${selected.diagnostic}';
    var searchAndChecksSeconds=profile ? Sys.time()-searchStarted-refinementSeconds : 0.0;
    if(refined==null){
      var refinementStarted=profile ? Sys.time() : 0.0;
      refined=refine(selected);
      if(profile){refinementSeconds+=Sys.time()-refinementStarted;refinementAttempts++;}
    }
    if(profile){
      var candidates=0;for(layer in problem.samples)candidates+=layer.candidates.length;
      Sys.println("PROCESS_PATH_PROFILE "+haxe.Json.stringify({family:problem.family,
        samples:problem.samples.length,candidates:candidates,buildSeconds:buildSeconds,
        searchAndChecksSeconds:searchAndChecksSeconds,refinementSeconds:refinementSeconds,
        refinementAttempts:refinementAttempts}));
    }
    selectionCost=selected.cost;
    return refined;
  }
}
