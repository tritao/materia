package motionkit.robot;

import MotionKitNative;
import robotkit.manipulation.KinematicGroup;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.LinkId;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;

/** Exports external space screws in the group's root frame, once per model. */
class SerialCellModel {
  public final native:mk_serial_cell_model;
  public final armIndices:Array<Int>;
  public final externalIndices:Array<Int>;
  public final armRoot:LinkId;
  final group:KinematicGroup;

  public function new(group:KinematicGroup,base:Transform3,tool:Transform3) {
    if (group==null || base==null || tool==null) throw "Serial cell export requires group and analytic frame conventions";
    this.group=group;
    armIndices=[for(i in 0...group.group.count())if(!group.external[i])i];
    externalIndices=[for(i in 0...group.group.count())if(group.external[i])i];
    if(armIndices.length!=6)throw "Serial cell export requires six arm joints";
    var root:Null<LinkId> = null;
    for(joint in group.pathJoints())if(joint.id==group.group.jointIds[armIndices[0]]){root=joint.parent.id;break;}
    if(root==null)throw "Serial cell export cannot locate its arm root";
    armRoot=cast root;
    native=new mk_serial_cell_model();native.set_struct_size(mk_serial_cell_model.size());
    native.set_joint_count(group.group.count());native.set_arm_joint_count(6);native.set_external_count(externalIndices.length);
    for(i in 0...6)native.set_arm_joint_indices(i,armIndices[i]);
    var entries=new Map<Int,{axis:Vec3,origin:Vec3,scope:Int,kind:Int}>();
    function collect(path:Array<Joint>,scope:Int):Void {
      var at=Transform3.identity();
      for(joint in path) {
        var frame=at.compose(Transform3.fromArrays(joint.parentFramePosition,joint.parentFrameRotation));
        if(joint.type!=JointType.Fixed) {
          var index=group.group.jointIds.indexOf(joint.id);
          if(index<0 || !group.external[index])throw 'External cell path has unsupported joint ${joint.id}';
          var kind=switch joint.type {case JointType.Prismatic:0;case JointType.Revolute | JointType.Continuous:1;default:throw "External cell joint must be scalar";};
          var axis=frame.transformVector(Vec3.fromArray(joint.axis)).normalized(),prior=entries.get(index);
          if(prior!=null) {
            if(prior.kind!=kind || prior.axis.sub(axis).norm()>1e-8 || prior.origin.sub(frame.translation).norm()>1e-8)
              throw "Shared external joint has inconsistent root geometry";
            prior.scope=2;
          } else entries.set(index,{axis:axis,origin:frame.translation,scope:scope,kind:kind});
        }
        at=frame.compose(Transform3.fromArrays(joint.childFramePosition,joint.childFrameRotation).inverse());
      }
    }
    collect(KinematicGroup.walk(group.robot,group.rootLink,armRoot),0);
    if(group.workFrame!=null) {
      var link:Null<LinkId> = null;
      for(frame in group.robot.frames)if(frame.id==group.workFrame){link=frame.link.id;break;}
      if(link==null)throw "Serial cell export cannot locate work frame";
      collect(KinematicGroup.walk(group.robot,group.rootLink,cast link),1);
    }
    for(i in 0...externalIndices.length) {
      var joint=externalIndices[i],entry=entries.get(joint);
      if(entry==null)throw "Serial cell export has an external joint outside base/work paths";
      native.set_external_joint_indices(i,joint);native.set_external_scopes(i,entry.scope);native.set_external_kinds(i,entry.kind);
      var axis=entry.axis.toArray(),origin=entry.origin.toArray();
      for(k in 0...3){native.set_external_axes(3*i+k,axis[k]);native.set_external_origins(3*i+k,origin[k]);}
    }
    var zero=[for(_ in 0...group.group.count())0.0],work=group.workPose(zero);
    var armBase=work.compose(group.linkPoses(zero,[armRoot])[0]).compose(base);
    var bp=armBase.translation.toArray(),br=armBase.rotation,wp=work.translation.toArray(),wr=work.rotation;
    var tp=tool.translation.toArray(),tr=tool.rotation;
    var bq=[br.x,br.y,br.z,br.w],wq=[wr.x,wr.y,wr.z,wr.w],tq=[tr.x,tr.y,tr.z,tr.w];
    for(i in 0...3){native.set_base_position(i,bp[i]);native.set_work_position(i,wp[i]);native.set_tool_position(i,tp[i]);}
    for(i in 0...4){native.set_base_quaternion(i,bq[i]);native.set_work_quaternion(i,wq[i]);native.set_tool_quaternion(i,tq[i]);}
  }
}
