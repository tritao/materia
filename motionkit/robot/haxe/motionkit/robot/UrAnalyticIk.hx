package motionkit.robot;

import MotionKitNative;
import TrajectoryCore;
import motionkit.kinematics.Pose3;
import motionkit.path.OrientationPolicy;
import motionkit.robot.AnalyticIk.AnalyticBranch;
import robotkit.manipulation.KinematicGroup;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.spatial.Transform3;
import robotkit.spatial.Quat;
import robotkit.spatial.Vec3;

/** UR parallel-axis 6R geometry extracted from model joint axes and frames. */
class UrAnalyticIk implements AnalyticIk {
  public final manipulator:KinematicGroup;
  final groupBackend:Null<UrGroupIk>;
  final native:mk_ur_parameters;
  final base:Transform3;
  final flangeTTcp:Transform3;

  public function new(manipulator:KinematicGroup, tolerance:Float = 1e-6) {
    if (manipulator == null || !Math.isFinite(tolerance) || tolerance <= 0)
      throw "UR analytic extraction requires a group and positive finite tolerance";
    this.manipulator = manipulator;
    if (manipulator.workFrame != null || manipulator.external.indexOf(true) >= 0) {
      groupBackend = new UrGroupIk(manipulator,tolerance);
      native = groupBackend.arm.native;
      base = groupBackend.arm.base;
      flangeTTcp = groupBackend.arm.flangeTTcp;
      return;
    }
    groupBackend = null;
    if (manipulator.group.count() != 6) throw "UR analytic extraction requires six arm joints";
    var path = manipulator.pathJoints();
    var drivers = [for (j in path) if (j.type != JointType.Fixed) j];
    if (drivers.length != 6) throw "UR analytic extraction requires six revolute joints";
    for (i in 0...drivers.length)
      if ((drivers[i].type != JointType.Revolute && drivers[i].type != JointType.Continuous) || drivers[i].id != manipulator.group.jointIds[i])
        throw "UR analytic extraction requires independent revolute joints in chain order";
    var reference = [for (_ in 0...6) 0.0];
    var geometry = geometryAt(path, reference);
    var z = geometry.axes[0], y = geometry.axes[1].scale(-1);
    if (Math.abs(z.dot(y)) > tolerance) throw "UR shoulder axis must be perpendicular to the base axis";
    var x = y.cross(z).normalized(); y = z.cross(x).normalized();
    reference[1] = alignment(geometry.origins[2].sub(geometry.origins[1]), x.scale(-1), geometry.axes[1]);
    geometry = geometryAt(path, reference);
    reference[2] = alignment(geometry.origins[3].sub(geometry.origins[2]), x.scale(-1), geometry.axes[2]);
    geometry = geometryAt(path, reference);
    reference[3] = alignment(geometry.axes[4], z.scale(-1), geometry.axes[3]);
    geometry = geometryAt(path, reference);
    reference[4] = alignment(geometry.axes[5], y.scale(-1), geometry.axes[4]);
    geometry = geometryAt(path, reference);
    base = new Transform3(geometry.origins[0], Quat.fromRotationMatrix([
      x.x,x.y,x.z,y.x,y.y,y.z,z.x,z.y,z.z]));
    var inverseBase = base.inverse();
    var local = [for (p in geometry.origins) inverseBase.transformPoint(p)];
    native = new mk_ur_parameters(); native.set_struct_size(mk_ur_parameters.size());
    var expected = [z,y.scale(-1),y.scale(-1),y.scale(-1),z.scale(-1),y.scale(-1)];
    for (i in 0...6) {
      var dot = geometry.axes[i].dot(expected[i]);
      if (Math.abs(Math.abs(dot)-1) > tolerance) throw 'UR joint ${drivers[i].id} violates the parallel-axis pattern';
      var sign = dot < 0 ? -1 : 1;
      native.set_sign_corrections(i,sign); native.set_offsets(i,reference[i]*sign);
    }
    native.set_a2(local[2].x-local[1].x); native.set_a3(local[3].x-local[2].x);
    native.set_d1(local[1].z); native.set_d4(-local[4].y);
    native.set_d5(local[1].z-local[5].z);
    var flange = inverseBase.compose(manipulator.forwardKinematics(reference));
    var d6 = -flange.translation.y + local[5].y;
    if (Math.abs(d6) < tolerance) throw "UR flange offset is degenerate";
    native.set_d6(d6);
    var zero = [for (_ in 0...6) 0.0];
    flangeTTcp = nativeForward(zero).inverse().compose(inverseBase.compose(manipulator.tcpPose(zero)));
    for (probe in 0...7) {
      var q = [for (i in 0...6) probe == i ? 0.37 : 0.0];
      var actual = manipulator.tcpPose(q), predicted = forwardTransform(q);
      if (predicted.translation.sub(actual.translation).norm() > tolerance || predicted.rotation.angularDistance(actual.rotation) > tolerance)
        throw "UR analytic extraction fails model FK agreement";
    }
  }
  static function alignment(from:Vec3,to:Vec3,axis:Vec3):Float {
    var a=from.sub(axis.scale(from.dot(axis))), b=to.sub(axis.scale(to.dot(axis)));
    if(a.norm()<1e-12 || b.norm()<1e-12) throw "UR canonical reference is degenerate";
    return Math.atan2(axis.dot(a.cross(b)),a.dot(b));
  }
  static function geometryAt(path:Array<Joint>,q:Array<Float>):{origins:Array<Vec3>,axes:Array<Vec3>} {
    var origins:Array<Vec3> = [],axes:Array<Vec3> = [],at=Transform3.identity(),index=0;
    for(joint in path) {
      var frame=at.compose(Transform3.fromArrays(joint.parentFramePosition,joint.parentFrameRotation));
      if(joint.type!=JointType.Fixed) {
        var axis=Vec3.fromArray(joint.axis).normalized(); origins.push(frame.translation);axes.push(frame.transformVector(axis).normalized());
        frame=frame.compose(new Transform3(new Vec3(0,0,0),Quat.fromAxisAngle(axis,q[index++])));
      }
      at=frame.compose(Transform3.fromArrays(joint.childFramePosition,joint.childFrameRotation).inverse());
    }
    return {origins:origins,axes:axes};
  }
  public function family():String return "UR6R";
  public function jointCount():Int return manipulator.group.count();
  public function nativeModel():mk_ur_parameters return native;
  function nativeForward(q:Array<Float>):Transform3 {
    var result=MotionKitNative.mk_analytic_ur_forward(native,q);
    if(result.status!=TrajectoryCoreConstants.MK_OK) throw 'UR analytic FK failed: ${result.status}';
    var p=result.out_pose;
    return new Transform3(new Vec3(p.get_position(0),p.get_position(1),p.get_position(2)),new Quat(p.get_quaternion(0),p.get_quaternion(1),p.get_quaternion(2),p.get_quaternion(3)));
  }
  function forwardTransform(q:Array<Float>):Transform3 return base.compose(nativeForward(q)).compose(flangeTTcp);
  public function forward(q:Array<Float>):Pose3 {
    if (q == null || q.length != jointCount()) throw "UR analytic FK requires complete joints";
    if (groupBackend != null) return groupBackend.forward(q);
    var t=forwardTransform(q),p=t.translation,r=t.rotation;
    return new Pose3(p.x,p.y,p.z,r.x,r.y,r.z,r.w);
  }
  public function branches(target:Pose3,seed:Array<Float>,?freedom:OrientationPolicy):Array<AnalyticBranch> {
    if (groupBackend != null) return groupBackend.branches(target,seed,freedom);
    if(seed==null || seed.length!=6 || target==null) throw "UR analytic IK requires a complete seed and target";
    switch freedom {case null | Fixed | Interpolated: case _: throw "UR analytic IK requires a fixed orientation sample";}
    var t=base.inverse().compose(new Transform3(new Vec3(target.x,target.y,target.z),new Quat(target.qx,target.qy,target.qz,target.qw))).compose(flangeTTcp.inverse());
    var pose=new mk_opw_pose();pose.set_struct_size(mk_opw_pose.size());
    var p=t.translation.toArray(),r=t.rotation,quat=[r.x,r.y,r.z,r.w];
    for(i in 0...3)pose.set_position(i,p[i]);for(i in 0...4)pose.set_quaternion(i,quat[i]);
    var result=MotionKitNative.mk_analytic_ur_inverse(native,pose,seed[5],8);
    if(result.status!=TrajectoryCoreConstants.MK_OK)throw 'UR analytic inverse failed: ${result.status}';
    var answers:Array<AnalyticBranch> = [];
    for(i in 0...result.out_count) {
      var raw=result.out_solutions[i],lifted=[[for(j in 0...6)raw.get_joints(j)]];
      for(j in 0...6) {
        var bounds=manipulator.group.limitsOf(j),next:Array<Array<Float>> = [];
        for(q in lifted) {
          var first=Math.ceil((bounds.lower-q[j]-1e-9)/(2*Math.PI)),last=Math.floor((bounds.upper-q[j]+1e-9)/(2*Math.PI));
          if(!Math.isFinite(first)||!Math.isFinite(last)||last-first>64)throw "UR analytic wraps require a finite planning range";
          for(wrap in Std.int(first)...Std.int(last)+1) {var copy=q.copy();copy[j]+=wrap*2*Math.PI;next.push(copy);}
        }
        lifted=next;
      }
      for(q in lifted)answers.push(new AnalyticBranch(q,raw.get_branch(),raw.get_singular()!=0));
    }
    return answers;
  }
}
