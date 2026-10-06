package motionkit.robot;
import MotionKitNative;
import TrajectoryCore;
import robotkit.manipulation.KinematicGroup;

/** Differential q'(s), q''(s) at an analytically solved configuration.
 * J comes from compiled kinematics. J' is its centred directional difference
 * along q'; joint derivatives themselves are solved from the task relation. */
class PathDifferential {
  public static function solve(group:KinematicGroup,q:Array<Float>,velocity:Array<Float>,acceleration:Array<Float>,
      known:Array<Bool>,knownFirst:Array<Float>,knownSecond:Array<Float>,differenceStep:Float=1e-6,freeToolAxis:Bool=false):JointDerivatives {
    if(group==null || q==null || q.length!=group.group.count() || velocity==null || velocity.length!=6 ||
        acceleration==null || acceleration.length!=6 || known==null || knownFirst==null || knownSecond==null ||
        known.length!=q.length || knownFirst.length!=q.length || knownSecond.length!=q.length ||
        !Math.isFinite(differenceStep) || differenceStep<=0)throw "Differential path data must match the compiled group";
    for(value in q)if(!Math.isFinite(value))throw "Differential configuration must be finite";
    var n=q.length,request=new mk_differential_request();request.set_struct_size(mk_differential_request.size());
    request.set_joint_count(n);request.set_rank_tolerance(1e-10);request.set_linear_tolerance(1e-8);request.set_angular_tolerance(1e-8);
    for(j in 0...n){request.set_known(j,known[j]?1:0);request.set_known_first(j,knownFirst[j]);request.set_known_second(j,0);}
    for(row in 0...6){request.set_task_velocity(row,velocity[row]);request.set_task_acceleration(row,0);}
    var originalJacobian=group.tcpJacobian(q);
    var pose=new motionkit.kinematics.Pose3();
    var rows:Array<Array<Float>> = [];
    if(freeToolAxis){
      var transform=group.tcpPose(q),rotation=transform.rotation;
      pose=new motionkit.kinematics.Pose3(transform.translation.x,transform.translation.y,transform.translation.z,
        rotation.x,rotation.y,rotation.z,rotation.w);
      rows=ToolFreedom.twistRows(pose,motionkit.path.OrientationPolicy.FreeAboutTool);
      rows.push([0.0,0,0,0,0,0]);
    }
    function projectVector(vector:Array<Float>):Array<Float> {
      if(!freeToolAxis)return vector;
      var result=[for(_ in 0...6)0.0];
      for(row in 0...6)for(axis in 0...6)result[row]+=rows[row][axis]*vector[axis];
      return result;
    }
    function projectMatrix(matrix:Array<Float>):Array<Float> {
      if(!freeToolAxis)return matrix;
      var result=[for(_ in 0...6*n)0.0];
      for(row in 0...6)for(j in 0...n)for(axis in 0...6)result[row*n+j]+=rows[row][axis]*matrix[axis*n+j];
      return result;
    }
    var jacobian=projectMatrix(originalJacobian),projectedVelocity=projectVector(velocity);
    for(row in 0...6)request.set_task_velocity(row,projectedVelocity[row]);
    var first=MotionKitNative.mk_path_differential(request,jacobian,[for(_ in jacobian)0.0],n);
    if(first.status!=TrajectoryCoreConstants.MK_OK)throw 'Differential velocity failed: ${first.status}, rank ${first.out_report.get_rank()}';
    var scale=1.0;for(value in first.out_first)scale=Math.max(scale,Math.abs(value));
    var step=differenceStep/scale;
    var before=[for(j in 0...n)q[j]-step*first.out_first[j]],after=[for(j in 0...n)q[j]+step*first.out_first[j]];
    var minus=group.tcpJacobian(before),plus=group.tcpJacobian(after);
    var change=projectMatrix([for(i in 0...originalJacobian.length)(plus[i]-minus[i])/(2*step)]);
    var taskAcceleration=freeToolAxis ? acceleration.copy() : acceleration;
    if(freeToolAxis){
      // z'' = alpha × z + omega × (omega × z). The physical head's
      // uncommanded spin changes the second term, even though z' agrees.
      var axis=ToolFreedom.toolAxis(pose),actualOmega=[0.0,0,0],desiredSpin=0.0,actualSpin=0.0;
      for(i in 0...3){
        for(j in 0...n)actualOmega[i]+=originalJacobian[(i+3)*n+j]*first.out_first[j];
        desiredSpin+=axis[i]*velocity[i+3];actualSpin+=axis[i]*actualOmega[i];
      }
      var cross=[axis[1]*actualOmega[2]-axis[2]*actualOmega[1],
        axis[2]*actualOmega[0]-axis[0]*actualOmega[2],axis[0]*actualOmega[1]-axis[1]*actualOmega[0]];
      for(i in 0...3)taskAcceleration[i+3]+=(desiredSpin-actualSpin)*cross[i];
    }
    var projectedAcceleration=projectVector(taskAcceleration);
    for(j in 0...n)request.set_known_second(j,knownSecond[j]);
    for(row in 0...6)request.set_task_acceleration(row,projectedAcceleration[row]);
    var result=MotionKitNative.mk_path_differential(request,jacobian,change,n);
    if(result.status!=TrajectoryCoreConstants.MK_OK)throw 'Differential acceleration failed: ${result.status}, rank ${result.out_report.get_rank()}';
    return new JointDerivatives(result.out_first,result.out_second);
  }
}
class JointDerivatives {
  public final first:Array<Float>;
  public final second:Array<Float>;
  public function new(first:Array<Float>,second:Array<Float>){this.first=first.copy();this.second=second.copy();}
}
