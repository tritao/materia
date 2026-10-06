package robotkit.manipulation;

import kinematicskit.JointKind;

/** Triangle-inequality motion bounds on compiled link geometry, including
 * affine multi-source followers and joints outside the planning group. */
class ClearanceMotionBounds {
  final arm:KinematicGroup;
  final values:Array<Float>;
  final errors:Array<Float>;
  public function new(arm:KinematicGroup,q:Array<Float>,error:Array<Float>) {
    if(q==null || error==null || q.length!=arm.group.count() || error.length!=q.length)
      throw "Motion bounds require one finite neighborhood per group coordinate";
    this.arm=arm;
    var model=arm.model,dofs=[for(id in arm.group.jointIds)model.dofIndex(id)];
    var state=arm.stateOf(q);
    for(i in 0...error.length)if(!Math.isFinite(q[i]) || !Math.isFinite(error[i]) || error[i]<0)
      throw "Motion neighborhoods must be finite and nonnegative";
    values=[];errors=[];
    for(j in 0...model.jointCount()){
      var value=model.jointConstant[j],bound=0.0;
      for(t in model.jointTermStart[j]...model.jointTermStart[j+1]){
        var d=model.jointTermDof[t],scale=model.jointTermScale[t],i=dofs.indexOf(d);
        value+=scale*state.q[d];
        if(i>=0)bound+=Math.abs(scale)*error[i];
      }
      values.push(value);errors.push(bound);
    }
  }
  static function norm(values:Array<Float>,at:Int):Float
    return Math.sqrt(values[at]*values[at]+values[at+1]*values[at+1]+values[at+2]*values[at+2]);

  /** Every point within radius of this link origin moves by at most this much. */
  public function point(link:String,radius:Float):Float {
    if(!Math.isFinite(radius) || radius<0)throw "Motion radius must be finite and nonnegative";
    var m=arm.model,body=m.bodyIndex(link),bound=0.0;
    if(body<0)throw "Motion bound link is not in the compiled model";
    while(m.bodyParentJoint[body]>=0){
      var j=m.bodyParentJoint[body];
      radius+=norm(m.jointJointTChild,7*j);
      switch m.jointKind[j] {
        case Revolute:bound+=radius*errors[j];
        case Prismatic:bound+=errors[j];radius+=Math.abs(values[j])+errors[j];
        case Fixed:
      }
      radius+=norm(m.jointParentTJoint,7*j);
      body=m.jointParent[j];
    }
    return bound;
  }
  function angular(link:String):Float {
    var m=arm.model,body=m.bodyIndex(link),bound=0.0;
    while(m.bodyParentJoint[body]>=0){var j=m.bodyParentJoint[body];
      if(m.jointKind[j]==JointKind.Revolute)bound+=errors[j];
      body=m.jointParent[j];
    }
    return bound;
  }
  /** Includes translation of the moving work frame and rotation of its axes. */
  public function tcp():Float {
    var m=arm.model,frame=m.frameIndex(arm.flangeFrame),link=m.bodyIds[m.frameBody[frame]];
    var tipRadius=norm(m.frameOffset,7*frame)+arm.flangeTTcp.translation.norm();
    var flange=point(link,tipRadius);
    if(arm.workFrame==null)return flange;
    var reference=m.frameIndex(arm.workFrame),workLink=m.bodyIds[m.frameBody[reference]];
    var work=point(workLink,norm(m.frameOffset,7*reference));
    // The reference rotation acts on the flange-reference difference.
    var q=[for(id in arm.group.jointIds) values[m.jointIndex(id)]];
    return flange+work+angular(workLink)*(arm.tcpPose(q).translation.norm()+flange+work);
  }
}
