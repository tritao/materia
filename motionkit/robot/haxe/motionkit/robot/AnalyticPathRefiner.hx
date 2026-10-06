package motionkit.robot;

import motionkit.kinematics.Pose3;
import motionkit.path.OrientationPolicy;
import motionkit.path.PoseMath;
import motionkit.robot.StructuredLadder.LadderSelection;
import motionkit.robot.CartesianCandidateSampler.LatticeCandidate;
import motionkit.robot.ExternalAxisGrid.ExternalAxisRange;
import robotkit.manipulation.KinematicGroup;
import robotkit.spatial.Quat;
import robotkit.spatial.Vec3;

/** Analytic refinement on one geometric branch. Branch-transition
 * segmentation is handled separately. */
class AnalyticPathRefiner {
  final group:KinematicGroup;
  final problem:CandidateProblem;
  final selection:LadderSelection;
  final sampler:Null<SerialCandidateSampler>;
  final cartesian:Null<CartesianCandidateSampler>;
  final branch:Int;
  final external:Array<RedundancySpline>;
  final roll:RedundancySpline;
  final swingX:RedundancySpline;
  final swingY:RedundancySpline;
  public function new(group:KinematicGroup,problem:CandidateProblem,selection:LadderSelection) {
    if(group==null || problem==null || selection==null || selection.diagnostic!=null ||
        problem.samples.length<2 || selection.candidates.length!=problem.samples.length)
      throw "Analytic refinement requires a complete selected path with at least two samples";
    var isCartesian=problem.family=="XYZ" || problem.family=="XYZ+C" || problem.family=="XYZ+C+A";
    if(!isCartesian && problem.family!="UR6R" && problem.family!="OPW")throw "Refinement requires a supported analytic family";
    this.group=group;this.problem=problem;this.selection=selection;
    sampler=isCartesian ? null : new SerialCandidateSampler(group);
    cartesian=isCartesian ? new CartesianCandidateSampler(group) : null;
    branch=selection.candidates[0].branch;
    var distances=[for(layer in problem.samples)layer.distance];
    for(c in selection.candidates)if(c.branch!=branch)throw "Refinement must split the route at geometric branch transitions";
    external=[for(j in problem.externalJoints)new RedundancySpline(distances,[for(c in selection.candidates)c.q[j]])];
    var rolls:Array<Float> = [],xs:Array<Float> = [],ys:Array<Float> = [];
    for(i in 0...selection.candidates.length){
      var actual=group.tcpPose(selection.candidates[i].q),layer=problem.samples[i];
      var centre=OrientationLattice.centre(layer.target,layer.freedom);
      var reference=new Quat(centre.qx,centre.qy,centre.qz,centre.qw);
      var relative=reference.conjugate().multiply(actual.rotation),length=Math.sqrt(relative.z*relative.z+relative.w*relative.w);
      if(length<1e-10)throw "Refinement roll is undefined at an antipodal tool axis";
      var spin=2*Math.atan2(relative.z,relative.w),twist=Quat.fromAxisAngle(new Vec3(0,0,1),spin);
      var swing=relative.multiply(twist.conjugate()),axis=swing.rotate(new Vec3(0,0,1));
      var tilt=Math.acos(Math.max(-1,Math.min(1,axis.z))),azimuth=Math.atan2(axis.y,axis.x);
      rolls.push(spin);xs.push(tilt*Math.cos(azimuth));ys.push(tilt*Math.sin(azimuth));
    }
    roll=new RedundancySpline(distances,rolls,2*Math.PI);
    swingX=new RedundancySpline(distances,xs);swingY=new RedundancySpline(distances,ys);
  }
  public function externalState(index:Int,distance:Float):motionkit.robot.RedundancySpline.SplineSample {
    if(index<0 || index>=external.length)throw "Unknown refinement external axis";
    return external[index].evaluate(distance);
  }
  /** Task rates must describe the refined TCP pose, including its smoothed
   * orientation. External redundancy rates come directly from the spline. */
  public function derivatives(distance:Float,q:Array<Float>,taskVelocity:Array<Float>,taskAcceleration:Array<Float>):motionkit.robot.PathDifferential.JointDerivatives {
    if(q==null || q.length!=group.group.count())throw "Refined derivatives require the complete configuration";
    var known=[for(_ in q)false],first=[for(_ in q)0.0],second=[for(_ in q)0.0];
    for(i in 0...external.length){var j=problem.externalJoints[i],state=external[i].evaluate(distance);
      if(!Math.isFinite(q[j]) || Math.abs(q[j]-state.value)>1e-8)throw "Differential configuration differs from its redundancy spline";
      known[j]=true;first[j]=state.first;second[j]=state.second;}
    return PathDifferential.solve(group,q,taskVelocity,taskAcceleration,known,first,second);
  }
  public function orientationMotion(distance:Float,target:Pose3,freedom:OrientationPolicy,
      centreOmega:Array<Float>,centreAlpha:Array<Float>):motionkit.robot.OrientationDifferential.OrientationMotion {
    var centre=OrientationLattice.centre(target,freedom);
    return OrientationDifferential.refine(new Quat(centre.qx,centre.qy,centre.qz,centre.qw),centreOmega,centreAlpha,
      swingX.evaluate(distance),swingY.evaluate(distance),roll.evaluate(distance));
  }
  /** The supplied six-dimensional rates describe OrientationLattice.centre,
   * including any changing cone axis. Spline swing/roll rates are added here. */
  public function refinedDerivatives(distance:Float,q:Array<Float>,target:Pose3,freedom:OrientationPolicy,
      centreVelocity:Array<Float>,centreAcceleration:Array<Float>):motionkit.robot.PathDifferential.JointDerivatives {
    if(centreVelocity==null || centreAcceleration==null || centreVelocity.length!=6 || centreAcceleration.length!=6)
      throw "Refined task derivatives need six-dimensional centre rates";
    var motion=orientationMotion(distance,target,freedom,centreVelocity.slice(3),centreAcceleration.slice(3));
    return derivatives(distance,q,centreVelocity.slice(0,3).concat(motion.velocity),centreAcceleration.slice(0,3).concat(motion.acceleration));
  }
  /** Produce the complete refined path for timing, preserving curvature on
   * both sides of a geometric knot. The provider supplies exact task data. */
  public function refinePath(distances:Array<Float>,task:Float->RefinementTarget):motionkit.planner.JointPathSamples {
    if(distances==null || distances.length<2 || task==null ||
        distances[0]!=problem.samples[0].distance || distances[distances.length-1]!=problem.samples[problem.samples.length-1].distance)
      throw "Refined path must cover the complete selected path range";
    for(i in 0...distances.length)if(!Math.isFinite(distances[i]) || i>0 && distances[i]<=distances[i-1])
      throw "Refined path distances must increase finitely";
    var positions:Array<Array<Float>> = [],first:Array<Array<Float>> = [],second:Array<Array<Float>> = [],before:Array<Array<Float>> = [];
    var previous=selection.candidates[0].q.copy();
    for(i in 0...distances.length){var distance=distances[i],target=task(distance);
      if(target==null)throw 'Missing refinement task at distance $distance';
      var candidate=sample(distance,target.pose,target.freedom,previous),q=candidate.q.copy();
      if(i==0){var same=true;for(j in 0...q.length)if(Math.abs(q[j]-previous[j])>1e-7)same=false;
        if(problem.pinnedStart && !same)throw "Refinement changed the pinned initial configuration";
        if(same)q=previous.copy();}
      var rates=refinedDerivatives(distance,q,target.pose,target.freedom,target.velocity,target.acceleration);
      var sameCurvature=true;for(row in 0...6)if(target.acceleration[row]!=target.accelerationBefore[row])sameCurvature=false;
      var incoming=sameCurvature ? rates : refinedDerivatives(distance,q,target.pose,target.freedom,target.velocity,target.accelerationBefore);
      positions.push(q);first.push(rates.first);second.push(rates.second);before.push(incoming.second);previous=q;
    }
    return new motionkit.planner.JointPathSamples(distances,positions,first,second,before);
  }
  public function sample(distance:Float,target:Pose3,freedom:OrientationPolicy,?seed:Array<Float>):LatticeCandidate {
    if(target==null || freedom==null)throw "Refinement needs a target and task freedom";
    var q=seed==null ? selection.candidates[0].q.copy() : seed.copy();
    if(q.length!=group.group.count())throw "Refinement seed must contain the complete group";
    var previous=q.copy();
    var ranges:Array<ExternalAxisRange> = [];
    for(i in 0...external.length){var j=problem.externalJoints[i],value=external[i].evaluate(distance).value;
      var limits=group.group.limitsOf(j);
      if(value<limits.lower || value>limits.upper)throw 'Refined external axis exceeds limits at distance $distance';
      q[j]=value;ranges.push(new ExternalAxisRange(j,value,value,1));}
    var rotation=orientationMotion(distance,target,freedom,[0.0,0.0,0.0],[0.0,0.0,0.0]).rotation;
    var refined=new Pose3(target.x,target.y,target.z,rotation.x,rotation.y,rotation.z,rotation.w);
    if(ToolFreedom.orientationError(refined,target,freedom)>problem.request.tolerance.orientation)
      throw 'Refined orientation exceeds task freedom at distance $distance';
    var best:Null<LatticeCandidate> = null,bestDistance=Math.POSITIVE_INFINITY;
    var candidates=cartesian!=null ? cartesian.sample(refined,q,OrientationPolicy.Fixed,1,1,1)
      : sampler.sample(refined,q,OrientationPolicy.Fixed,ranges,1,1,1);
    for(c in candidates)if(c.branch==branch){
      var d=0.0,legal=true;for(j in 0...q.length){d+=Math.abs(c.q[j]-q[j]);
        if(Math.abs(c.q[j]-previous[j])>problem.request.maxJump[j]+1e-12)legal=false;}
      if(legal && d<bestDistance){best=c;bestDistance=d;}}
    if(best==null)throw 'Selected analytic branch is unreachable or exceeds joint jumps at distance $distance';
    var actual=group.tcpPose(best.q),p=actual.translation,r=actual.rotation;
    var checked=new Pose3(p.x,p.y,p.z,r.x,r.y,r.z,r.w);
    if(PoseMath.distance(checked,target)>problem.request.tolerance.position ||
        ToolFreedom.orientationError(checked,target,freedom)>problem.request.tolerance.orientation)
      throw 'Refined analytic pose exceeds task tolerance at distance $distance';
    return best;
  }
}

/** Centre rates are six-dimensional spatial derivatives: linear then angular. */
class RefinementTarget {
  public final pose:Pose3;
  public final freedom:OrientationPolicy;
  public final velocity:Array<Float>;
  public final acceleration:Array<Float>;
  public final accelerationBefore:Array<Float>;
  public function new(pose:Pose3,freedom:OrientationPolicy,velocity:Array<Float>,acceleration:Array<Float>,?accelerationBefore:Array<Float>){
    if(pose==null || freedom==null || velocity==null || acceleration==null || velocity.length!=6 || acceleration.length!=6 ||
        accelerationBefore!=null && accelerationBefore.length!=6)throw "Refinement targets require a pose and spatial derivatives";
    this.pose=pose;this.freedom=freedom;this.velocity=velocity.copy();this.acceleration=acceleration.copy();
    this.accelerationBefore=accelerationBefore==null ? acceleration.copy() : accelerationBefore.copy();
    for(values in [this.velocity,this.acceleration,this.accelerationBefore])for(value in values)
      if(!Math.isFinite(value))throw "Refinement task derivatives must be finite";
  }
}
