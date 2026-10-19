import robotkit.model.RobotModel;
import robotkit.runtime.RobotRuntimeIdentity;
import robotkit.kinematics.RobotKinematics;
import kinematicskit.*;

/** Gravity feedforward from the simulator's own mass/CoG model; no gains or tolerances are relaxed. */
class TendingElectricGravity {
  final robot:RobotModel;
  final identity:RobotRuntimeIdentity;
  final model:KinematicModel;
  final state:KinematicState;
  final snapshot:KinematicSnapshot;
  final runtimeDofs:Array<Int>;
  public function new(robot:RobotModel,identity:RobotRuntimeIdentity) {
    this.robot=robot;this.identity=identity;model=RobotKinematics.compile(robot);
    state=new KinematicState(model);snapshot=new KinematicSnapshot(model);
    runtimeDofs=[for(i in 0...model.dofCount())identity.jointIndex(robot.joints[model.dofJoint[i]].id)];
  }
  function evaluate(q:Array<Float>):Void {
    for(i in 0...runtimeDofs.length)state.q[i]=q[runtimeDofs[i]];snapshot.evaluate(state);
  }
  public function mount(frame:materia.assembly.AssemblyRecord.AssemblyFrame):Void {
    state.setRootPose(model.bodyIndex(robot.links[0].id),new Transform(frame.x*0.001,frame.y*0.001,frame.z*0.001,frame.qx,frame.qy,frame.qz,frame.qw));
  }
  public function torques(q:Array<Float>,?payloads:Array<{link:Int,point:Array<Float>,mass:Float}>):Array<Float> {
    evaluate(q);var result=[for(_ in q)0.0],n=model.dofCount();
    for(link in robot.links) {
      var body=model.bodyIndex(link.id),pose=snapshot.bodyPose(body),c=link.centerOfMass;
      var point=pose.transformPoint(c[0],c[1],c[2]),jac=snapshot.pointJacobian(body,point.x,point.y,point.z);
      for(i in 0...n)result[runtimeDofs[i]]+=link.mass*9.81*jac[2*n+i];
    }
    if(payloads!=null)for(payload in payloads) {
      var jac=snapshot.pointJacobian(model.bodyIndex(robot.links[payload.link].id),payload.point[0],payload.point[1],payload.point[2]);
      for(i in 0...n)result[runtimeDofs[i]]+=payload.mass*9.81*jac[2*n+i];
    }
    return result;
  }
  function potential(q:Array<Float>):Float {
    evaluate(q);var result=0.0;
    for(link in robot.links) {
      var c=link.centerOfMass,point=snapshot.bodyPose(model.bodyIndex(link.id)).transformPoint(c[0],c[1],c[2]);
      result+=link.mass*9.81*point.z;
    }
    return result;
  }
  /** Independent potential-energy derivative catches sign, frame and DOF-order errors. */
  public function check(q:Array<Float>,joints:Array<Int>):Int {
    var analytic=torques(q),epsilon=0.00001;
    for(j in joints) {
      var plus=q.copy(),minus=q.copy();plus[j]+=epsilon;minus[j]-=epsilon;
      var derivative=(potential(plus)-potential(minus))/(2*epsilon);
      if(!Math.isFinite(derivative) || Math.abs(derivative-analytic[j])>Math.max(0.00001,Math.abs(derivative)*0.000001))
        throw "Gravity feedforward differs from the potential-energy derivative";
    }
    return joints.length;
  }
}
