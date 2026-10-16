package motionkit.robot;

import motionkit.robot.ProgramCompiler.PendingMotion;
import motionkit.planner.PathTimingBackend;
import motionkit.planner.ToppraPathTiming;
import motionkit.planner.JointPathPolynomial;
import motionkit.path.PosePositionBound;

/** Spend only the Cartesian path allowance not already used by geometry and native lowering. */
class PathMergeBudget {
  public static function derive(solver:AxisKinematics,timing:PathTimingBackend,pending:PendingMotion,
      seconds:Float,projection:Null<CouplingProjection>):Null<MergeErrorBudget> {
    var path=pending.path,curve=pending.geometryCurve;
    if(path==null || curve==null || pending.authoredPolyline!=null || !Std.isOfType(timing,ToppraPathTiming))return null;
    var used=0.0,allowed=Math.POSITIVE_INFINITY;
    if(path.authoredGeometry!=null){
      if(path.geometryDeviationBound==null)return null;
      used=path.geometryDeviationBound;allowed=path.blendTolerance;
    } else for(primitive in path.primitives)
      allowed=Math.min(allowed,Math.min(primitive.startWaypoint().positionTolerance,primitive.endWaypoint().positionTolerance));
    var interpolation=0.0,at=0;
    for(span in 0...curve.s.length-1){
      while(at+1<path.primitives.length && curve.s[span]>=path.startOf(at)+path.primitives[at].length())at++;
      var primitive=path.primitives[at],length=primitive.length();
      if(!Std.isOfType(primitive,PosePositionBound))return null;
      var local=curve.s[span]-path.startOf(at),end=curve.s[span+1]-path.startOf(at);
      if(local < -1e-12 || end>length+1e-12)return null;
      local=Math.max(0,local);end=Math.min(length,end);
      var control=JointPathPolynomial.bernstein(JointPathPolynomial.coefficients(curve,span));
      var cartesian=[for(axis in [solver.x,solver.y,solver.z])
        [for(value in control[axis.jointIndices[0]])(value-axis.jointOffset(0))/axis.jointScale(0)]];
      var bound=(cast primitive:PosePositionBound).positionErrorBound(cartesian,local,end);
      if(!Math.isFinite(bound) || bound<0)return null;
      interpolation=Math.max(interpolation,bound);
    }
    var nativeTiming:ToppraPathTiming=cast timing;
    var loweringSquared=0.0;
    for(axis in [solver.x,solver.y,solver.z]){
      var error=nativeTiming.loweringTolerance/Math.abs(axis.jointScale(0));loweringSquared+=error*error;
    }
    var lowering=Math.sqrt(loweringSquared);
    var remaining=allowed-used-interpolation-lowering;
    remaining-=512*2.220446049250313e-16*(allowed+used+interpolation+lowering+path.length());
    if(!Math.isFinite(remaining) || remaining<=0)return null;
    var count=solver.jointCount(),position=[for(_ in 0...count)0.0],owned=[for(_ in 0...count)false];
    // An inscribed XYZ box has Euclidean radius `remaining` in the tool frame.
    for(axis in [solver.x,solver.y,solver.z])for(slot in 0...axis.jointIndices.length){
      var joint=axis.jointIndices[slot];
      position[joint]=remaining/Math.sqrt(3.0)*Math.abs(axis.jointScale(slot));owned[joint]=true;
    }
    // The distance-driven joint receives the same additional path-phase allowance,
    // expressed in its own units through the actual ramp profile (including overshoot).
    var drive=path.drive;
    if(drive!=null){
      position[drive.joint]=drive.maximumRate()*remaining;owned[drive.joint]=true;
    }
    if(projection!=null)for(follower in projection.followers){
      var sum=0.0;for(term in projection.termsOf(follower))sum+=Math.abs(term.ratio)*position[term.leader];
      position[follower]=owned[follower] ? Math.min(position[follower],sum) : sum;
    }
    return new MergeErrorBudget(position,seconds);
  }
}
