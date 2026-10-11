package kinematicskit;

import kinematicskit.JointKind;

/** Triangle-inequality motion bounds on compiled link geometry, including
 * affine multi-source followers and joints outside the planning group. */
class ClearanceMotionBounds {
  final model:KinematicModel;
  final state:KinematicState;
  final flangeFrame:Int;
  final tcpOffset:Transform;
  final workFrame:Int;
  final values:Array<Float>;
  final errors:Array<Float>;
  public function new(model:KinematicModel,dofs:Array<Int>,state:KinematicState,error:Array<Float>,
      flangeFrame:Int,tcpOffset:Transform,workFrame:Int = -1) {
    if(model==null || state==null || state.model!=model || error==null || error.length!=dofs.length)
      throw "Motion bounds require one finite neighborhood per coordinate and a state of their model";
    this.model=model;this.state=state.copy();this.flangeFrame=flangeFrame;
    this.tcpOffset=Transform.checked(tcpOffset,"Motion bounds TCP offset");this.workFrame=workFrame;
    if(flangeFrame<0 || flangeFrame>=model.frameCount() || workFrame < -1 || workFrame>=model.frameCount())throw "Invalid motion bound frame";
    for(i in 0...error.length) {
      if(dofs[i]<0 || dofs[i]>=model.dofCount() || dofs.indexOf(dofs[i])!=i)throw "Invalid motion bound DOFs";
      if(!Math.isFinite(state.q[dofs[i]]) || !Math.isFinite(error[i]) || error[i]<0)
        throw "Motion neighborhoods must be finite and nonnegative";
    }
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
  public function point(link:Int,radius:Float):Float {
    if(!Math.isFinite(radius) || radius<0)throw "Motion radius must be finite and nonnegative";
    var m=model,body=link,bound=0.0;
    if(body<0 || body>=m.bodyCount())throw "Motion bound link is not in the compiled model";
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
  function angular(link:Int):Float {
    var m=model,body=link,bound=0.0;
    while(m.bodyParentJoint[body]>=0){var j=m.bodyParentJoint[body];
      if(m.jointKind[j]==JointKind.Revolute)bound+=errors[j];
      body=m.jointParent[j];
    }
    return bound;
  }
  /** Includes translation of the moving work frame and rotation of its axes. */
  public function tcp():Float {
    var m=model,frame=flangeFrame,link=m.frameBody[frame];
    var tipRadius=norm(m.frameOffset,7*frame)+Math.sqrt(tcpOffset.x*tcpOffset.x+tcpOffset.y*tcpOffset.y+tcpOffset.z*tcpOffset.z);
    var flange=point(link,tipRadius);
    if(workFrame<0)return flange;
    var reference=workFrame,workLink=m.frameBody[reference];
    var work=point(workLink,norm(m.frameOffset,7*reference));
    // The reference rotation acts on the flange-reference difference.
    var snapshot=KinematicSnapshot.of(state);
    var pose=snapshot.framePose(reference).inverse().compose(snapshot.framePose(frame)).compose(tcpOffset);
    return flange+work+angular(workLink)*(Math.sqrt(pose.x*pose.x+pose.y*pose.y+pose.z*pose.z)+flange+work);
  }
}
