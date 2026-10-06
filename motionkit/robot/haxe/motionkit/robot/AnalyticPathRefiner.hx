package motionkit.robot;

import motionkit.kinematics.SixAxisConfiguration;

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
  final numeric:Null<NumericBranchIk>;
  public final diagnostic:Null<String>;
  final branch:Int;
  final external:Array<RedundancySpline>;
  final prescribedJoints:Array<Int>;
  final internalJoints:Array<Int>;
  final roll:RedundancySpline;
  final swingX:RedundancySpline;
  final swingY:RedundancySpline;
  final freeCentre:Quat;
  public function new(group:KinematicGroup,problem:CandidateProblem,selection:LadderSelection) {
    if(group==null || problem==null || selection==null || selection.diagnostic!=null ||
        problem.samples.length<2 || selection.candidates.length!=problem.samples.length)
      throw "Analytic refinement requires a complete selected path with at least two samples";
    var isCartesian=problem.family=="XYZ" || problem.family=="XYZ+C" || problem.family=="XYZ+C+A";
    var isNumeric=problem.family=="numeric-fallback";
    if(!isNumeric && !isCartesian && problem.family!="UR6R" && problem.family!="OPW")throw "Refinement requires a supported analytic family";
    this.group=group;this.problem=problem;this.selection=selection;
    freeCentre=group.tcpPose(selection.candidates[0].q).rotation;
    sampler=isCartesian || isNumeric ? null : new SerialCandidateSampler(group);
    numeric=isNumeric ? new NumericBranchIk(group,problem.diagnostic,problem.request.tolerance) : null;
    diagnostic=problem.diagnostic;
    cartesian=isCartesian ? new CartesianCandidateSampler(group) : null;
    branch=selection.candidates[0].branch;
    var distances=[for(layer in problem.samples)layer.distance];
    for(c in selection.candidates)if(!isNumeric && c.branch!=branch)throw "Refinement must split the route at geometric branch transitions";
    internalJoints=[];
    if(isNumeric){
      var arm=[for(j in 0...group.group.count())if(!group.external[j])j];
      if(arm.length>6){
        var independent:Array<Int> = [],jacobian=group.tcpJacobian(selection.candidates[0].q);
        for(j in arm){var trial=independent.concat([j]);
          var columns=[for(row in 0...6)for(column in trial)jacobian[row*group.group.count()+column]];
          if(kinematicskit.LinearAlgebra.rank(columns,6,trial.length,1e-10)>independent.length)independent.push(j);
          if(independent.length==6)break;
        }
        if(independent.length!=6)throw "Numeric refinement needs a regular six-dimensional task chart";
        for(j in arm)if(independent.indexOf(j)<0)internalJoints.push(j);
        for(c in selection.candidates){var matrix=group.tcpJacobian(c.q);
          var columns=[for(row in 0...6)for(j in independent)matrix[row*group.group.count()+j]];
          if(kinematicskit.LinearAlgebra.rank(columns,6,6,1e-10)!=6)
            throw "Numeric refinement must split at an internal redundancy chart transition";
        }
      }
    }
    prescribedJoints=problem.externalJoints.concat(internalJoints);
    external=[for(j in prescribedJoints)new RedundancySpline(distances,[for(c in selection.candidates)c.q[j]])];
    var rolls:Array<Float> = [],xs:Array<Float> = [],ys:Array<Float> = [];
    for(i in 0...selection.candidates.length){
      var actual=group.tcpPose(selection.candidates[i].q),layer=problem.samples[i];
      // Full orientation has no redundancy to smooth. Fitting IK residuals
      // would turn numerical pose error into artificial spline curvature.
      if(ToolFreedom.isFull(layer.freedom)){rolls.push(0);xs.push(0);ys.push(0);continue;}
      var centre=OrientationLattice.centre(layer.target,layer.freedom);
      var reference=switch layer.freedom {case Free:freeCentre;default:new Quat(centre.qx,centre.qy,centre.qz,centre.qw);};
      var relative=reference.conjugate().multiply(actual.rotation),length=Math.sqrt(relative.z*relative.z+relative.w*relative.w);
      if(length<1e-10)throw "Refinement roll is undefined at an antipodal tool axis";
      var spin=2*Math.atan2(relative.z,relative.w),twist=Quat.fromAxisAngle(new Vec3(0,0,1),spin);
      var swing=relative.multiply(twist.conjugate()),axis=swing.rotate(new Vec3(0,0,1));
      var tilt=Math.atan2(Math.sqrt(axis.x*axis.x+axis.y*axis.y),axis.z),azimuth=Math.atan2(axis.y,axis.x);
      rolls.push(spin);xs.push(tilt*Math.cos(azimuth));ys.push(tilt*Math.sin(azimuth));
    }
    roll=new RedundancySpline(distances,rolls,2*Math.PI);
    swingX=new RedundancySpline(distances,xs);swingY=new RedundancySpline(distances,ys);
  }
  public function externalState(index:Int,distance:Float):motionkit.robot.RedundancySpline.SplineSample {
    if(index<0 || index>=problem.externalJoints.length)throw "Unknown refinement external axis";
    return external[index].evaluate(distance);
  }
  /** Task rates must describe the refined TCP pose, including its smoothed
   * orientation. External redundancy rates come directly from the spline. */
  public function derivatives(distance:Float,q:Array<Float>,taskVelocity:Array<Float>,taskAcceleration:Array<Float>):motionkit.robot.PathDifferential.JointDerivatives {
    if(q==null || q.length!=group.group.count())throw "Refined derivatives require the complete configuration";
    var known=[for(_ in q)false],first=[for(_ in q)0.0],second=[for(_ in q)0.0];
    for(i in 0...external.length){var j=prescribedJoints[i],state=external[i].evaluate(distance);
      if(!Math.isFinite(q[j]) || Math.abs(q[j]-state.value)>1e-8)throw "Differential configuration differs from its redundancy spline";
      known[j]=true;first[j]=state.first;second[j]=state.second;}
    return PathDifferential.solve(group,q,taskVelocity,taskAcceleration,known,first,second);
  }
  public function orientationMotion(distance:Float,target:Pose3,freedom:OrientationPolicy,
      centreOmega:Array<Float>,centreAlpha:Array<Float>):motionkit.robot.OrientationDifferential.OrientationMotion {
    var centre=OrientationLattice.centre(target,freedom);
    switch freedom {
      case Free:
        // Authored rotation is unconstrained. Its movement must not inject
        // angular rates or put the selected start at an artificial pole.
        return OrientationDifferential.refine(freeCentre,[0.0,0.0,0.0],[0.0,0.0,0.0],
          swingX.evaluate(distance),swingY.evaluate(distance),roll.evaluate(distance));
      default:
    }
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
    return refineRange(distances,task,0);
  }

  /** Refine a timing section of the globally selected route. Distances and
   * task coordinates remain global, so redundancy splines are shared across
   * stops. The section task supplies its own one-sided endpoint derivatives. */
  public function refineSection(distances:Array<Float>,task:Float->RefinementTarget):motionkit.planner.JointPathSamples {
    if(distances==null || distances.length<2 || task==null)
      throw "Refinement section requires at least two samples and its task";
    var first=-1,last=-1;
    for(i in 0...problem.samples.length){
      if(problem.samples[i].distance==distances[0])first=i;
      if(problem.samples[i].distance==distances[distances.length-1])last=i;
    }
    if(first<0 || last<=first)throw "Refinement section endpoints must be selected route knots";
    return refineRange(distances,task,first);
  }

  function refineRange(distances:Array<Float>,task:Float->RefinementTarget,firstSample:Int):motionkit.planner.JointPathSamples {
    for(i in 0...distances.length)if(!Math.isFinite(distances[i]) || i>0 && distances[i]<=distances[i-1])
      throw "Refined path distances must increase finitely";
    var positions:Array<Array<Float>> = [],first:Array<Array<Float>> = [],second:Array<Array<Float>> = [],before:Array<Array<Float>> = [];
    var previous=selection.candidates[firstSample].q.copy();
    for(i in 0...distances.length){var distance=distances[i],target=task(distance);
      if(target==null)throw 'Missing refinement task at distance $distance';
      var candidate=sample(distance,target.pose,target.freedom,previous),q=candidate.q.copy();
      if(i==0){var same=true;for(j in 0...q.length)if(Math.abs(q[j]-previous[j])>1e-7)same=false;
        if(firstSample==0 && problem.pinnedStart && !same)throw "Refinement changed the pinned initial configuration";
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
    for(i in 0...external.length){var j=prescribedJoints[i],value=external[i].evaluate(distance).value;
      var limits=group.group.limitsOf(j);
      if(value<limits.lower || value>limits.upper)throw 'Refined external axis exceeds limits at distance $distance';
      q[j]=value;if(group.external[j])ranges.push(new ExternalAxisRange(j,value,value,1));}
    var rotation=orientationMotion(distance,target,freedom,[0.0,0.0,0.0],[0.0,0.0,0.0]).rotation;
    var refined=new Pose3(target.x,target.y,target.z,rotation.x,rotation.y,rotation.z,rotation.w);
    if(ToolFreedom.orientationError(refined,target,freedom)>problem.request.tolerance.orientation)
      throw 'Refined orientation exceeds task freedom at distance $distance';
    var best:Null<LatticeCandidate> = null,bestDistance=Math.POSITIVE_INFINITY;
    var candidates:Array<LatticeCandidate>;
    if(numeric!=null){
      // Numeric IDs identify seeds rather than geometric branches. Continue
      // the selected physical lift with one seed and the same jump guards.
      candidates=[for(c in numeric.branchesFromNeighbours(refined,[q],q,OrientationPolicy.Fixed,false,internalJoints))
        new LatticeCandidate(c.q,[for(_ in c.q)0],[for(_ in problem.externalJoints)0],0,0,0,c.branch,0,false)];
    }else candidates=cartesian!=null ? cartesian.sample(refined,q,OrientationPolicy.Fixed,1,1,1)
      : sampler.sample(refined,q,OrientationPolicy.Fixed,ranges,1,1,1,problem.request.maxJump);
    var configuration=problem.configuration;
    for(c in candidates)if((numeric!=null || c.branch==branch) &&
        (configuration==null || cast(configuration,SixAxisConfiguration).accepts(c.configuration))){
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
