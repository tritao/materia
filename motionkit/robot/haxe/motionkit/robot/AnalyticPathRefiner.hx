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

/** Serial-arm refinement on one geometric branch. Differential joint output
 * and branch-transition segmentation are handled separately. */
class AnalyticPathRefiner {
  final group:KinematicGroup;
  final problem:CandidateProblem;
  final selection:LadderSelection;
  final sampler:SerialCandidateSampler;
  final branch:Int;
  final external:Array<RedundancySpline>;
  final roll:RedundancySpline;
  final swingX:RedundancySpline;
  final swingY:RedundancySpline;
  public function new(group:KinematicGroup,problem:CandidateProblem,selection:LadderSelection) {
    if(group==null || problem==null || selection==null || selection.diagnostic!=null ||
        problem.samples.length<2 || selection.candidates.length!=problem.samples.length)
      throw "Analytic refinement requires a complete selected path with at least two samples";
    if(problem.family!="UR6R" && problem.family!="OPW")throw "Serial refinement requires a supported analytic arm";
    this.group=group;this.problem=problem;this.selection=selection;
    sampler=new SerialCandidateSampler(group);branch=selection.candidates[0].branch;
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
    var centre=OrientationLattice.centre(target,freedom),rotation=new Quat(centre.qx,centre.qy,centre.qz,centre.qw);
    var x=swingX.evaluate(distance).value,y=swingY.evaluate(distance).value;
    var tilt=Math.sqrt(x*x+y*y),azimuth=Math.atan2(y,x);
    rotation=rotation.multiply(Quat.fromAxisAngle(new Vec3(0,0,1),azimuth))
      .multiply(Quat.fromAxisAngle(new Vec3(0,1,0),tilt))
      .multiply(Quat.fromAxisAngle(new Vec3(0,0,1),-azimuth+roll.evaluate(distance).value));
    var refined=new Pose3(target.x,target.y,target.z,rotation.x,rotation.y,rotation.z,rotation.w);
    if(ToolFreedom.orientationError(refined,target,freedom)>problem.request.tolerance.orientation)
      throw 'Refined orientation exceeds task freedom at distance $distance';
    var best:Null<LatticeCandidate> = null,bestDistance=Math.POSITIVE_INFINITY;
    for(c in sampler.sample(refined,q,OrientationPolicy.Fixed,ranges,1,1,1))if(c.branch==branch){
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
