package motionkit.robot;

import motionkit.kinematics.Pose3;
import motionkit.path.PosePath;
import motionkit.path.PosePrimitive;
import motionkit.path.PoseMath;
import motionkit.planner.JointPathSamples;
import motionkit.planner.JointPathPolynomial;
import robotkit.manipulation.KinematicGroup;
import robotkit.manipulation.ClearanceMotionEnvelope;

/** Continuous task bounds for the actual quintic, including native lowering.
 * The linearized FK residual is a Bernstein polynomial. Rigid-chain Taylor
 * remainders, authored arc curvature and lowering displacement bound the rest.
 * Unsupported primitives or an inconclusive bound retain dense checking. */
class JointCurveTask {
  final arm:KinematicGroup;
  final path:PosePath;
  final curve:JointPathSamples;
  final envelope:ClearanceMotionEnvelope;
  final lowering:Array<Float>;
  public var positionBound(default,null):Float = 0.0;
  public var orientationBound(default,null):Float = 0.0;
  public var positionTolerance(default,null):Float = Math.POSITIVE_INFINITY;
  public var queries(default,null):Int = 0;

  function new(arm:KinematicGroup,path:PosePath,curve:JointPathSamples,tolerance:Float) {
    this.arm=arm;this.path=path;this.curve=curve;
    var lower=curve.q[0].copy(),upper=lower.copy();
    for(span in 0...curve.s.length-1){
      var controls=JointPathPolynomial.bernstein(JointPathPolynomial.coefficients(curve,span));
      for(j in 0...lower.length)for(value in controls[j]){lower[j]=Math.min(lower[j],value);upper[j]=Math.max(upper[j],value);}
    }
    lowering=[for(_ in lower)1.01*tolerance];
    for(j in 0...lower.length){lower[j]-=lowering[j];upper[j]+=lowering[j];}
    envelope=new ClearanceMotionEnvelope(arm,lower,upper,[],[]);
  }

  public static function prove(arm:KinematicGroup,path:PosePath,curve:JointPathSamples,
      tolerance:Float):Null<JointCurveTask> {
    if(!Math.isFinite(tolerance) || tolerance<=0 || curve.jointCount!=arm.group.count() ||
        path.authoredGeometry!=null || curve.s[0]!=0 ||
        curve.end()!=path.length())return null;
    var allowed=path.primitives[0].startWaypoint().positionTolerance;
    for(primitive in path.primitives){
      if(!Std.isOfType(primitive,motionkit.path.PoseLine) && !Std.isOfType(primitive,motionkit.path.PoseArc))return null;
      if(primitive.startWaypoint().positionTolerance!=allowed || primitive.endWaypoint().positionTolerance!=allowed)return null;
    }
    var proof=new JointCurveTask(arm,path,curve,tolerance);
    return proof.check() ? proof : null;
  }

  function check():Bool {
    var primitive=0,offset=0.0;
    for(span in 0...curve.s.length-1){
      var start=curve.s[span],end=curve.s[span+1];
      while(primitive<path.primitives.length-1 && start>=offset+path.primitives[primitive].length()){
        offset+=path.primitives[primitive].length();primitive++;
      }
      var authored=path.primitives[primitive],length=authored.length();
      if(start<offset || end>offset+length)return false;
      var control=JointPathPolynomial.bernstein(JointPathPolynomial.coefficients(curve,span));
      if(!piece(authored,control,start-offset,Math.min(length,end-offset),0))return false;
    }
    return true;
  }

  function piece(primitive:PosePrimitive,control:Array<Array<Float>>,start:Float,end:Float,depth:Int):Bool {
    var halves=JointPathPolynomial.split(control),q=[for(row in halves[0])row[5]],mid=(start+end)*0.5,width=end-start;
    var desired=primitive.waypointAt(mid),rates=primitive.derivativesAt(mid),pose=arm.tcpPose(q);
    var actual=new Pose3(pose.translation.x,pose.translation.y,pose.translation.z,
      pose.rotation.x,pose.rotation.y,pose.rotation.z,pose.rotation.w);
    var jacobian=arm.tcpJacobian(q),n=q.length;
    var errors=[for(j in 0...n){var error=0.0;for(value in control[j])error=Math.max(error,Math.abs(value-q[j]));error;}];
    var residual=0.0,numericScale=1.0+Math.abs(actual.x)+Math.abs(actual.y)+Math.abs(actual.z)+
      Math.abs(desired.pose.x)+Math.abs(desired.pose.y)+Math.abs(desired.pose.z);
    for(axis in 0...3)for(j in 0...n){var magnitude=Math.abs(q[j]);for(value in control[j])magnitude=Math.max(magnitude,Math.abs(value));
      numericScale+=Math.abs(jacobian[axis*n+j])*magnitude;}
    var roundoff=1e-9+1e-12*numericScale;
    for(k in 0...6){
      var sum=0.0;
      for(axis in 0...3){
        var component=axis==0 ? actual.x-desired.pose.x : axis==1 ? actual.y-desired.pose.y : actual.z-desired.pose.z;
        for(j in 0...n)component+=jacobian[axis*n+j]*(control[j][k]-q[j]);
        component-=rates.linear[axis]*width*(k/5.0-0.5);sum+=component*component;
      }
      residual=Math.max(residual,Math.sqrt(sum));
    }
    var curvature=Std.isOfType(primitive,motionkit.path.PoseArc) ?
      Math.sqrt(rates.linearSecond[0]*rates.linearSecond[0]+rates.linearSecond[1]*rates.linearSecond[1]+rates.linearSecond[2]*rates.linearSecond[2]) : 0.0;
    // Reserve endpoint-grid roundoff plus scale-dependent FK/Jacobian
    // arithmetic error. Large numerical scales fall back to dense checks.
    var position=residual+envelope.tcpLinearizationRemainder(errors)+curvature*width*width/8+
      envelope.tcp(lowering)+roundoff;
    var policy=primitive.orientationPolicy(),angularTravel=switch policy {
      case Fixed | Free | Cone(_, _):0.0;
      default:PoseMath.angle(primitive.startWaypoint().pose,primitive.endWaypoint().pose)*width/primitive.length();
    };
    // Twice the mean angular rate bounds both slerp and normalized lerp.
    var orientation=switch policy {case Free:0.0;default:
      ToolFreedom.orientationError(actual,desired.pose,policy)+
      envelope.angularBound([for(j in 0...n)errors[j]+lowering[j]])+angularTravel+roundoff;
    };
    var posTolerance=Math.min(primitive.startWaypoint().positionTolerance,primitive.endWaypoint().positionTolerance),
      rotTolerance=Math.min(primitive.startWaypoint().orientationTolerance,primitive.endWaypoint().orientationTolerance);
    queries++;
    if(position<=posTolerance && orientation<=rotTolerance){
      positionBound=Math.max(positionBound,position);orientationBound=Math.max(orientationBound,orientation);
      positionTolerance=Math.min(positionTolerance,posTolerance);return true;
    }
    if(depth>=8)return false;
    return piece(primitive,halves[0],start,mid,depth+1) && piece(primitive,halves[1],mid,end,depth+1);
  }
}
