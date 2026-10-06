package motionkit.robot;

import MotionKitNative;
import TrajectoryCore;
import motionkit.kinematics.Pose3;
import motionkit.robot.OpwKinematics.OpwParameters;
import robotkit.manipulation.KinematicGroup;
import robotkit.manipulation.Manipulator;
import robotkit.model.JointType;
import robotkit.model.LinkId;
import robotkit.spatial.Transform3;
import robotkit.spatial.Quat;
import robotkit.spatial.Vec3;

/** Model-derived OPW axis pattern with a displacement along the middle wrist
 * axis. Forward geometry only: no inverse family is advertised by this class. */
@:access(motionkit.robot.OpwKinematics)
class OffsetWristGeometry {
  public final parameters:OpwParameters;
  public final offset:Float;
  public final armIndices:Array<Int>;
  public final armRoot:LinkId;
  final group:KinematicGroup;
  final base:Transform3;
  final tool:Transform3;
  final native:MotionKitNative.mk_opw_parameters;

  public function new(group:KinematicGroup,tolerance:Float=1e-6){
    if(group==null || !Math.isFinite(tolerance) || tolerance<=0)throw "Offset wrist geometry requires a group and positive tolerance";
    this.group=group;
    armIndices=[for(i in 0...group.group.count())if(!group.external[i])i];
    if(armIndices.length!=6)throw "Offset wrist geometry requires six internal joints";
    var root:Null<LinkId> = null;
    for(joint in group.pathJoints())if(joint.id==group.group.jointIds[armIndices[0]]){root=joint.parent.id;break;}
    if(root==null)throw "Offset wrist geometry cannot locate arm root";
    armRoot=cast root;
    var arm=new Manipulator(group.robot,armRoot,group.flangeFrame,group.flangeTTcp,group.model,null,group.profile);
    if(arm.group.count()!=6)throw "Offset wrist external axes must precede the arm";
    var path=arm.pathJoints(),joints=[for(joint in path)if(joint.type!=JointType.Fixed)joint];
    for(joint in joints)if(joint.type!=JointType.Revolute && joint.type!=JointType.Continuous)
      throw "Offset wrist internal joints must be rotary";
    var reference=[for(_ in 0...6)0.0],g=OpwKinematics.geometryAt(path,reference);
    if(Math.abs(g.axes[3].dot(g.axes[2]))>tolerance)throw "Offset wrist forearm axis is not perpendicular to elbow";
    var z=g.axes[0];
    reference[1]=OpwKinematics.alignmentAngle(g.origins[2].sub(g.origins[1]),z,g.axes[1]);
    g=OpwKinematics.geometryAt(path,reference);
    reference[2]=OpwKinematics.alignmentAngle(g.axes[3],z,g.axes[2]);
    g=OpwKinematics.geometryAt(path,reference);
    reference[3]=OpwKinematics.alignmentAngle(g.axes[4],g.axes[1],g.axes[3]);
    g=OpwKinematics.geometryAt(path,reference);
    reference[4]=OpwKinematics.alignmentAngle(g.axes[5],z,g.axes[4]);
    g=OpwKinematics.geometryAt(path,reference);
    z=g.axes[0];var y=g.axes[1];
    if(Math.abs(z.dot(y))>tolerance)throw "Offset wrist base and shoulder axes must be perpendicular";
    var x=y.cross(z).normalized();y=z.cross(x).normalized();
    base=new Transform3(g.origins[0],Quat.fromRotationMatrix([x.x,x.y,x.z,y.x,y.y,y.z,z.x,z.y,z.z]));
    var signs:Array<Int> = [];
    for(i in 0...6){
      var expected=i==0 || i==3 || i==5 ? z : y,alignment=g.axes[i].dot(expected);
      if(Math.abs(Math.abs(alignment)-1)>tolerance)throw "Offset wrist axis pattern is unsupported";
      signs.push(alignment<0?-1:1);
    }
    var local=[for(origin in g.origins)base.inverse().transformPoint(origin)];
    var p2=local[1],p3=local[2].sub(p2),wrist=new Vec3(local[3].x,local[3].y,local[4].z);
    if(Math.abs(p3.x)>tolerance || Math.abs(local[4].x-wrist.x)>tolerance || Math.abs(local[5].x-wrist.x)>tolerance)
      throw "Offset wrist displacement must lie along its middle axis";
    offset=local[5].y-wrist.y;
    var p4=wrist.sub(local[2]),flange=base.inverse().compose(arm.forwardKinematics(reference));
    var c4=flange.translation.sub(wrist).z;
    if(c4 < -tolerance)throw "Offset wrist flange is behind wrist";
    parameters=new OpwParameters(p2.x,p4.x,wrist.y,p2.z,p3.z,p4.z,Math.max(0,c4),
      [for(i in 0...6)reference[i]*signs[i]],signs);
    native=parameters.toNative();
    var zero=[for(_ in 0...6)0.0];
    tool=localForward(zero).inverse().compose(base.inverse().compose(arm.tcpPose(zero)));
    for(i in 0...6){
      var probe=zero.copy();probe[i]=0.37;
      var actual=base.compose(localForward(probe)).compose(tool),expected=arm.tcpPose(probe);
      if(actual.translation.sub(expected.translation).norm()>tolerance || actual.rotation.angularDistance(expected.rotation)>tolerance)
        throw 'Offset wrist forward geometry mismatch at joint $i';
    }
  }

  function localForward(q:Array<Float>):Transform3 {
    var result=MotionKitNative.mk_opw_forward(native,q);
    if(result.status!=TrajectoryCoreConstants.MK_OK)throw "Offset wrist OPW forward failed";
    var pose=result.out_pose,r=new Quat(pose.get_quaternion(0),pose.get_quaternion(1),pose.get_quaternion(2),pose.get_quaternion(3));
    var theta=q[5]*parameters.signCorrections[5]-parameters.offsets[5];
    var middle=r.multiply(Quat.fromAxisAngle(new Vec3(0,0,1),-theta)).rotate(new Vec3(0,1,0));
    return new Transform3(new Vec3(pose.get_position(0),pose.get_position(1),pose.get_position(2)).add(middle.scale(offset)),r);
  }

  public function forward(q:Array<Float>):Pose3 {
    if(q==null || q.length!=group.group.count())throw "Offset wrist forward requires complete joints";
    for(value in q)if(!Math.isFinite(value))throw "Offset wrist joints must be finite";
    var pose=group.linkPoses(q,[armRoot])[0].compose(base).compose(localForward([for(i in armIndices)q[i]])).compose(tool);
    var p=pose.translation,r=pose.rotation;
    return new Pose3(p.x,p.y,p.z,r.x,r.y,r.z,r.w);
  }
}
