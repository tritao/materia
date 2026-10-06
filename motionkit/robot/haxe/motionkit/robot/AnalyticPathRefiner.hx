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
  var external:Array<RedundancyCurve>;
  final prescribedJoints:Array<Int>;
  final internalJoints:Array<Int>;
  var roll:RedundancyCurve;
  var swingX:RedundancyCurve;
  var swingY:RedundancyCurve;
  final freeCentre:Quat;
  var constrained:Bool=false;
  public function new(group:KinematicGroup,problem:CandidateProblem,selection:LadderSelection) {
    if(group==null || problem==null || selection==null || selection.diagnostic!=null ||
        problem.samples.length<2 || selection.candidates.length!=problem.samples.length)
      throw "Analytic refinement requires a complete selected path with at least two samples";
    var isCartesian=problem.family=="XYZ" || problem.family=="XYZ+C" || problem.family=="XYZ+C+A";
    var isNumeric=problem.family=="numeric-fallback";
    if(!isNumeric && !isCartesian && problem.family!="EAIK")throw "Refinement requires a supported analytic family";
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
  /** Fit one global banded QP so stopped sections share identical positions.
   * The exact branch is re-solved and its linear bounds updated at most three times. */
  public function constrain(tasks:Array<RefinementTarget>,sections:Array<{first:Int,last:Int}>):Void {
    var drives=problem.request.drives;
    if(drives==null || constrained)return;
    var distances=[for(layer in problem.samples)layer.distance],n=distances.length;
    if(tasks==null || tasks.length!=n)throw "Drive refinement tasks must align with the route";
    var orientation=problem.family=="EAIK" || numeric!=null,tilt=false;
    for(target in tasks)switch target.freedom {case Free | Cone(_,_):tilt=true;default:}
    var orientationCount=orientation?(tilt?3:1):0;
    var d=prescribedJoints.length+orientationCount;
    if(d==0){
      for(i in 0...n){
        // A unique Cartesian curve cannot reshape its air motion. Its air
        // cap is enforced by physical timing, which may slow that motion.
        // The prescribed weld feed must still be feasible geometrically.
        if(!drives.fixedFeed[i])continue;
        var task=tasks[i],q=selection.candidates[i].q;
        var rates=refinedDerivatives(distances[i],q,task.pose,task.freedom,task.velocity,task.acceleration),feed=drives.feed[i];
        for(j in 0...q.length){var v=Math.abs(rates.first[j])*feed;
          var a=Math.abs(rates.second[j])*feed*feed+Math.abs(rates.first[j])*drives.feedGradient[i]*feed;
          if(v>problem.request.velocity[j] || a>drives.acceleration[j]){
            var sustainable=feed*Math.min(v==0?1e30:problem.request.velocity[j]/v,a==0?1e30:Math.sqrt(drives.acceleration[j]/a));
            throw 'Process joint $j cannot sustain planned feed $feed; sustainable feed at most $sustainable';
          }
        }
      }
      constrained=true;return;
    }
    var route=[for(j in 0...d)[for(i in 0...n)j<prescribedJoints.length ? external[j].evaluate(distances[i]).value :
      j==prescribedJoints.length ? roll.evaluate(distances[i]).value :
      j==prescribedJoints.length+1 ? swingX.evaluate(distances[i]).value : swingY.evaluate(distances[i]).value]];
    var seed=[for(row in route)row.copy()];
    var lower:Array<Float> = [],upper:Array<Float> = [],velocity:Array<Float> = [],acceleration:Array<Float> = [];
    for(j in prescribedJoints){var limit=group.group.limitsOf(j);
      lower.push(limit.lower);upper.push(limit.upper);velocity.push(problem.request.velocity[j]);acceleration.push(drives.acceleration[j]);}
    if(orientation)for(j in 0...orientationCount){
      // Roll/swing are task charts, not new drives. Their physical velocity
      // and acceleration constraints come from the linearized arm rows below.
      lower.push(j==0?-1e6:-Math.PI);upper.push(j==0?1e6:Math.PI);velocity.push(1e6);acceleration.push(1e6);
    }
    var seededFirst=-1,seededLast=-1,seededMargin=0.0;
    // A straight translation parallel to a physical prismatic external axis
    // admits a constant arm pose. Choose the best legal posture on the pinned
    // branch, then give the same optimizer that seed and its posture floor.
    var longest=0.0;
    for(section in sections){var a=section.first,b=section.last,length=distances[b]-distances[a];
      if(length<=longest || !orientation)continue;
      var from=tasks[a].pose,to=tasks[b].pose,delta=new Vec3(to.x-from.x,to.y-from.y,to.z-from.z),straight=true;
      for(i in a...b+1){var t=(distances[i]-distances[a])/length,p=tasks[i].pose;
        if(new Vec3(p.x-from.x,p.y-from.y,p.z-from.z).sub(delta.scale(t)).norm()>1e-8 ||
            PoseMath.angle(from,p)>1e-8)straight=false;}
      if(!straight)continue;
      for(k in 0...problem.externalJoints.length){var joint=problem.externalJoints[k],prismatic=false;
        for(record in group.pathJoints())if(record.id==group.group.jointIds[joint])
          switch record.type {case Prismatic:prismatic=true;default:}
        if(!prismatic)continue;
        var best:Null<LatticeCandidate> = null,bestMargin=-1.0,bestAxis:Null<Vec3> = null;
        for(candidate in problem.samples[a].candidates){
          if(candidate.branch!=branch || problem.configuration!=null && !cast(problem.configuration,SixAxisConfiguration).accepts(candidate.configuration))continue;
          var jac=group.tcpJacobian(candidate.q),count=group.group.count(),axis=new Vec3(jac[joint],jac[count+joint],jac[2*count+joint]);
          if(axis.norm()<1e-8 || axis.cross(delta).norm()>1e-8)continue;
          var end=candidate.q.copy();end[joint]+=delta.dot(axis)/axis.dot(axis);
          var limit=group.group.limitsOf(joint);if(end[joint]<limit.lower || end[joint]>limit.upper)continue;
          var endPose=group.tcpPose(end),r=endPose.rotation,p=endPose.translation;
          if(PoseMath.distance(new Pose3(p.x,p.y,p.z,r.x,r.y,r.z,r.w),to)>problem.request.tolerance.position)continue;
          var margin=armMargin(candidate.q);
          if(margin>bestMargin){best=candidate;bestMargin=margin;bestAxis=axis;}
        }
        if(best==null)continue;
        var chosen:LatticeCandidate=cast best,axis:Vec3=cast bestAxis;
        for(i in 0...n){var p=tasks[i].pose;
          var t=Math.max(0.0,Math.min(1.0,(distances[i]-distances[a])/length));
          seed[k][i]=chosen.q[joint]+delta.scale(t).dot(axis)/axis.dot(axis);
        }
        var actual=group.tcpPose(chosen.q),centre=OrientationLattice.centre(tasks[a].pose,tasks[a].freedom);
        var reference=switch tasks[a].freedom {case Free:freeCentre;default:new Quat(centre.qx,centre.qy,centre.qz,centre.qw);};
        var relative=reference.conjugate().multiply(actual.rotation),spin=2*Math.atan2(relative.z,relative.w);
        spin+=2*Math.PI*Math.round((route[prescribedJoints.length][a]-spin)/(2*Math.PI));
        for(i in a...b+1){seed[prescribedJoints.length][i]=spin;if(tilt){seed[prescribedJoints.length+1][i]=0;seed[prescribedJoints.length+2][i]=0;}}
        seededFirst=a;seededLast=b;seededMargin=Math.max(0.0,bestMargin-0.0001);longest=length;
      }
    }
    var fullDistances=distances,fullTasks=tasks;
    var controls=[0],stops=[for(section in sections)section.first].concat([for(section in sections)section.last]);
    for(i in 1...n)if(i==n-1 || stops.indexOf(i)>=0 || distances[i]-distances[controls[controls.length-1]]>=0.025)controls.push(i);
    distances=[for(i in controls)fullDistances[i]];tasks=[for(i in controls)fullTasks[i]];
    route=[for(row in route)[for(i in controls)row[i]]];seed=[for(row in seed)[for(i in controls)row[i]]];n=controls.length;
    var controlFeed:Array<Float> = [],controlGradient:Array<Float> = [];
    for(i in 0...n){var last=i+1<n?controls[i+1]:controls[i]+1,v=0.0,g=0.0;
      for(k in controls[i]...last){v=Math.max(v,drives.feed[k]);g=Math.max(g,drives.feedGradient[k]);}
      controlFeed.push(v);controlGradient.push(g);
    }
    var controlDrives=new motionkit.kinematics.PathDriveLimits(drives.acceleration,controlFeed,controlGradient);
    if(Sys.getEnv("PROCESS_PATH_PROFILE")=="1")Sys.println("PROCESS_PATH_REFINEMENT_SEED "+haxe.Json.stringify({
      first:seededFirst,last:seededLast,marginFloor:seededMargin,coordinates:d,controlKnots:n,exactKnots:fullDistances.length}));
    function install(curves:Array<RedundancyCurve>):Void {
      external=curves.slice(0,prescribedJoints.length);
      if(orientation){roll=curves[prescribedJoints.length];if(tilt){swingX=curves[prescribedJoints.length+1];swingY=curves[prescribedJoints.length+2];}}
    }
    install([for(j in 0...d)new RedundancySpline(distances,seed[j])]);
    for(round in 0...3){
      var bounds:Array<MotionKitNative.mk_refinement_bound> = [],gradient=[for(_ in 0...d)[for(_ in 0...n)0.0]];
      var curves=external.concat(orientation?(tilt?[roll,swingX,swingY]:[roll]):[]);
      for(i in 0...n){var target=tasks[i],q=sample(distances[i],target.pose,target.freedom,selection.candidates[controls[i]].q,true).q;
        var values=[for(curve in curves)curve.evaluate(distances[i])];
        var known=[for(_ in q)false];for(j in prescribedJoints)known[j]=true;
        var columns:Array<Array<Float>> = [];
        for(c in 0...d){
          var first=[for(_ in q)0.0],taskFirst=[for(_ in 0...6)0.0];
          if(c<prescribedJoints.length)first[prescribedJoints[c]]=1;
          else {
            var centre=OrientationLattice.centre(target.pose,target.freedom);
            var reference=switch target.freedom {case Free:freeCentre;default:new Quat(centre.qx,centre.qy,centre.qz,centre.qw);};
            var offset=prescribedJoints.length;
            var motion=OrientationDifferential.refine(reference,[0.0,0,0],[0.0,0,0],
              new motionkit.robot.RedundancySpline.SplineSample(swingX.evaluate(distances[i]).value,c==offset+1?1:0,0),
              new motionkit.robot.RedundancySpline.SplineSample(swingY.evaluate(distances[i]).value,c==offset+2?1:0,0),
              new motionkit.robot.RedundancySpline.SplineSample(values[offset].value,c==offset?1:0,0));
            for(axis in 0...3)taskFirst[axis+3]=motion.velocity[axis];
          }
          columns.push(PathDifferential.solve(group,q,taskFirst,[0.0,0,0,0,0,0],known,first,[for(_ in q)0.0]).first);
        }
        var actual=refinedDerivatives(distances[i],q,target.pose,target.freedom,target.velocity,target.acceleration);
        var feed=controlDrives.feed[i];
        for(j in 0...q.length)if(!group.external[j] && prescribedJoints.indexOf(j)<0){
          var coefficients=[for(column in columns)column[j]],bias=q[j],firstBias=actual.first[j],secondBias=actual.second[j];
          for(c in 0...d){bias-=coefficients[c]*values[c].value;firstBias-=coefficients[c]*values[c].first;secondBias-=coefficients[c]*values[c].second;}
          var limit=group.group.limitsOf(j),margin=controls[i]>=seededFirst && controls[i]<=seededLast?seededMargin:0.0;
          if(limit.lower<limit.upper){
            bounds.push(DriveAwareRefinement.bound(i,coefficients,limit.lower+margin-bias,limit.upper-margin-bias));
            var centre=(limit.lower+limit.upper)/2;
            for(c in 0...d)gradient[c][i]+=0.1*(q[j]-centre)*coefficients[c];
          }
          var vmax=problem.request.velocity[j]*0.8/feed;
          bounds.push(DriveAwareRefinement.bound(i,coefficients,-vmax-firstBias,vmax-firstBias,1));
          var available=drives.acceleration[j]*0.8-problem.request.velocity[j]*controlDrives.feedGradient[i];
          if(available<=0)throw 'Drive-aware refinement joint $j cannot sustain feed $feed';
          var amax=available/(feed*feed);
          bounds.push(DriveAwareRefinement.bound(i,coefficients,-amax-secondBias,amax-secondBias,2));
        }
        if(orientation)for(c in prescribedJoints.length...d){
          var fixed=ToolFreedom.isFull(target.freedom) || c>prescribedJoints.length &&
            switch target.freedom {case FreeAboutTool:true;default:false;};
          var unit=[for(_ in 0...d)0.0];unit[c]=1;
          if(fixed){bounds.push(DriveAwareRefinement.bound(i,unit,0,0));bounds.push(DriveAwareRefinement.bound(i,unit,0,0,1));}
          else if(c>prescribedJoints.length)switch target.freedom {
            case Cone(_,angle):var box=angle/Math.sqrt(2);bounds.push(DriveAwareRefinement.bound(i,unit,-box,box));
            default:
          }
        }
        if(problem.pinnedStart && i==0)for(c in 0...d){var unit=[for(_ in 0...d)0.0];unit[c]=1;
          bounds.push(DriveAwareRefinement.bound(i,unit,route[c][0],route[c][0]));}
      }
      install(DriveAwareRefinement.fit(distances,route,seed,lower,upper,velocity,acceleration,controlDrives,bounds,gradient));
      var valid=true,lastDiagnostic="";
      for(i in 0...fullDistances.length){var target=fullTasks[i],q=sample(fullDistances[i],target.pose,target.freedom,selection.candidates[i].q,true).q;
        var rates=refinedDerivatives(fullDistances[i],q,target.pose,target.freedom,target.velocity,target.acceleration),feed=drives.feed[i];
        if(i>=seededFirst && i<=seededLast && armMargin(q)<seededMargin-1e-6){valid=false;lastDiagnostic='posture at ${fullDistances[i]}';}
        for(j in 0...q.length){var v=Math.abs(rates.first[j])*feed;
          var a=Math.abs(rates.second[j])*feed*feed+Math.abs(rates.first[j])*drives.feedGradient[i]*feed;
          if(v>problem.request.velocity[j]*(1+1e-6) || a>drives.acceleration[j]*(1+1e-6)){
            valid=false;var sustainable=feed*Math.min(v==0?1e30:problem.request.velocity[j]/v,a==0?1e30:Math.sqrt(drives.acceleration[j]/a));
            lastDiagnostic='joint $j at ${fullDistances[i]}, feed $feed, sustainable feed at most $sustainable';
          }
        }
      }
      if(valid){constrained=true;return;}
      if(round==2)throw 'Drive-aware exact recheck failed: $lastDiagnostic';
    }
  }
  function armMargin(q:Array<Float>):Float {
    var margin=Math.POSITIVE_INFINITY;
    for(j in 0...q.length)if(!group.external[j]){var limit=group.group.limitsOf(j);
      if(limit.lower<limit.upper)margin=Math.min(margin,Math.min(q[j]-limit.lower,limit.upper-q[j]));}
    return margin;
  }
  public function externalState(index:Int,distance:Float):motionkit.robot.RedundancySpline.SplineSample {
    if(index<0 || index>=problem.externalJoints.length)throw "Unknown refinement external axis";
    return external[index].evaluate(distance);
  }
  /** Task rates must describe the refined TCP pose, including its smoothed
   * orientation. External redundancy rates come directly from the spline. */
  public function derivatives(distance:Float,q:Array<Float>,taskVelocity:Array<Float>,taskAcceleration:Array<Float>,freeToolAxis:Bool=false):motionkit.robot.PathDifferential.JointDerivatives {
    if(q==null || q.length!=group.group.count())throw "Refined derivatives require the complete configuration";
    var known=[for(_ in q)false],first=[for(_ in q)0.0],second=[for(_ in q)0.0];
    for(i in 0...external.length){var j=prescribedJoints[i],state=external[i].evaluate(distance);
      if(!Math.isFinite(q[j]) || Math.abs(q[j]-state.value)>1e-8)throw "Differential configuration differs from its redundancy spline";
      known[j]=true;first[j]=state.first;second[j]=state.second;}
    return PathDifferential.solve(group,q,taskVelocity,taskAcceleration,known,first,second,1e-6,freeToolAxis);
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
    return derivatives(distance,q,centreVelocity.slice(0,3).concat(motion.velocity),centreAcceleration.slice(0,3).concat(motion.acceleration),
      problem.family=="XYZ+C+A" && !ToolFreedom.isFull(freedom));
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
      var candidate=sample(distance,target.pose,target.freedom,previous,i==0 && !problem.pinnedStart),q=candidate.q.copy();
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
  public function sample(distance:Float,target:Pose3,freedom:OrientationPolicy,?seed:Array<Float>,allowRelocation:Bool=false):LatticeCandidate {
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
      : sampler.sample(refined,q,OrientationPolicy.Fixed,ranges,1,1,1,allowRelocation?null:problem.request.maxJump);
    var configuration=problem.configuration;
    for(c in candidates)if((numeric!=null || c.branch==branch) &&
        (configuration==null || cast(configuration,SixAxisConfiguration).accepts(c.configuration))){
      var d=0.0,legal=true;for(j in 0...q.length){d+=Math.abs(c.q[j]-q[j]);
        if(!allowRelocation && Math.abs(c.q[j]-previous[j])>problem.request.maxJump[j]+1e-12)legal=false;}
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
