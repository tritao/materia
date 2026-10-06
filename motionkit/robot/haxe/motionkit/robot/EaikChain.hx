package motionkit.robot;

import MotionKitNative;
import TrajectoryCore;
import robotkit.manipulation.KinematicGroup;
import robotkit.manipulation.Manipulator;
import robotkit.model.LinkId;
import robotkit.model.JointType;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import motionkit.kinematics.Pose3;

/** Immutable compiled 6R geometry, independent of recipe or arm dimensions. */
class EaikChain {
  public final group:KinematicGroup;
  public final armIndices:Array<Int>;
  public final armRoot:LinkId;
  public final chain:Manipulator;
  public final descriptor:mk_eaik_model;
  final owner:Ownedmk_eaik_handle;
  public var solver(get,never):mk_eaik_handle;
  function get_solver():mk_eaik_handle return owner.borrow();
  public function dispose():Void owner.close();
  public function new(group:KinematicGroup) {
    if(group==null)throw "EAIK requires a compiled kinematic group";
    this.group=group;
    armIndices=[for(i in 0...group.group.count())if(!group.external[i])i];
    if(armIndices.length!=6)throw "EAIK requires six independent arm joints";
    var root:Null<LinkId> = null;
    for(joint in group.pathJoints())if(joint.id==group.group.jointIds[armIndices[0]]){root=joint.parent.id;break;}
    if(root==null)throw "EAIK cannot locate the arm root";
    armRoot=cast root;
    chain=new Manipulator(group.robot,armRoot,group.flangeFrame,group.flangeTTcp,group.model,null,group.profile);
    if(chain.group.count()!=6)throw "EAIK external joints must lie upstream of the arm or on the workpiece";
    var at=Transform3.identity(),origins:Array<Vec3> = [],axes:Array<Vec3> = [];
    for(joint in chain.pathJoints()){
      var frame=at.compose(Transform3.fromArrays(joint.parentFramePosition,joint.parentFrameRotation));
      switch joint.type {
        case Revolute | Continuous:
          var i=axes.length;
          if(i>=6 || joint.id!=group.group.jointIds[armIndices[i]])throw "EAIK arm ordering differs from the compiled chain";
          axes.push(frame.transformVector(Vec3.fromArray(joint.axis)).normalized());origins.push(frame.translation);
        case Fixed:
        case _:throw "EAIK arm joints must be independent revolute joints";
      }
      at=frame.compose(Transform3.fromArrays(joint.childFramePosition,joint.childFrameRotation).inverse());
    }
    if(axes.length!=6)throw "EAIK needs six compiled revolute axes";
    var zero=chain.forwardKinematics([for(_ in 0...6)0.0]);
    var p=[origins[0]];
    for(i in 1...6)p.push(origins[i].sub(origins[i-1]));
    p.push(zero.translation.sub(origins[5]));
    descriptor=new mk_eaik_model();descriptor.set_struct_size(mk_eaik_model.size());
    for(i in 0...6){var v=axes[i].toArray();for(j in 0...3)descriptor.set_axes(3*i+j,v[j]);}
    for(i in 0...7){var v=p[i].toArray();for(j in 0...3)descriptor.set_displacements(3*i+j,v[j]);}
    var r=zero.rotation.toRotationMatrix();for(i in 0...9)descriptor.set_terminal_rotation(i,r[i]);
    var made=MotionKitNative.mk_eaik_create(descriptor);
    if(made.status!=TrajectoryCoreConstants.MK_OK)throw 'EAIK has no supported decomposition for this compiled 6R chain (${made.status})';
    owner=made.out_solver;
  }
  public function localTarget(target:Pose3,seed:Array<Float>):mk_analytic_pose {
    if(seed==null || seed.length!=group.group.count() || target==null)throw "EAIK target requires a complete lattice cell";
    var requested=new Transform3(new Vec3(target.x,target.y,target.z),new Quat(target.qx,target.qy,target.qz,target.qw));
    var local=group.linkPoses(seed,[armRoot])[0].inverse().compose(requested).compose(chain.flangeTTcp.inverse());
    var record=new mk_analytic_pose();record.set_struct_size(mk_analytic_pose.size());
    var p=local.translation.toArray(),r=local.rotation,q=[r.x,r.y,r.z,r.w];
    for(i in 0...3)record.set_position(i,p[i]);for(i in 0...4)record.set_quaternion(i,q[i]);
    return record;
  }
}
