package robotkit.manipulation;

import kinematicskit.JointKind;
import robotkit.spatial.Vec3;
import robotkit.spatial.Transform3;

/** Fixed reach coefficients over a finite joint box. The box must include
 * the entire geometry and lowering neighborhood. All affine follower terms
 * contribute; joints outside the group keep their compiled held values. */
class ClearanceMotionEnvelope {
  final arm:KinematicGroup;
  final travel:Array<Float>;
  final gains:Array<Array<Float>>;
  final hulls:Array<Array<Float>>;
  final tip:Array<Float>;
  public function new(arm:KinematicGroup,lower:Array<Float>,upper:Array<Float>,links:Array<String>,corners:Array<Array<Float>>) {
    this.arm=arm;
    var n=arm.group.count(),m=arm.model;
    if(lower.length!=n || upper.length!=n || links.length!=corners.length)throw "Invalid clearance envelope dimensions";
    for(i in 0...n)if(!Math.isFinite(lower[i]) || !Math.isFinite(upper[i]) || lower[i]>upper[i])throw "Invalid clearance envelope joint box";
    var mid=[for(i in 0...n)lower[i]+(upper[i]-lower[i])*0.5],state=arm.stateOf(mid),
      dofs=[for(id in arm.group.jointIds)m.dofIndex(id)];
    travel=[];gains=[];
    for(j in 0...m.jointCount()){
      var lo=m.jointConstant[j],hi=lo,gain=[for(_ in 0...n)0.0];
      for(t in m.jointTermStart[j]...m.jointTermStart[j+1]){
        var d=m.jointTermDof[t],scale=m.jointTermScale[t],i=dofs.indexOf(d);
        if(i<0){lo+=scale*state.q[d];hi+=scale*state.q[d];}
        else {lo+=scale*(scale>=0 ? lower[i] : upper[i]);hi+=scale*(scale>=0 ? upper[i] : lower[i]);gain[i]+=Math.abs(scale);}
      }
      travel.push(Math.max(Math.abs(lo),Math.abs(hi)));gains.push(gain);
    }
    hulls=[for(i in 0...links.length)hull(links[i],corners[i])];
    var f=m.frameIndex(arm.flangeFrame),flange=point(m.bodyIds[m.frameBody[f]],norm(m.frameOffset,7*f)+arm.flangeTTcp.translation.norm());
    tip=flange.coefficients;
    if(arm.workFrame!=null){
      var w=m.frameIndex(arm.workFrame),link=m.bodyIds[m.frameBody[w]],work=point(link,norm(m.frameOffset,7*w)),angles=angular(link);
      for(i in 0...n)tip[i]+=work.coefficients[i]+angles[i]*(flange.reach+work.reach);
    }
  }
  static function norm(v:Array<Float>,i:Int):Float return Math.sqrt(v[i]*v[i]+v[i+1]*v[i+1]+v[i+2]*v[i+2]);
  function point(link:String,radius:Float):{coefficients:Array<Float>,reach:Float} {
    if(!Math.isFinite(radius) || radius<0)throw "Invalid clearance envelope radius";
    var m=arm.model,body=m.bodyIndex(link),coefficients=[for(_ in 0...arm.group.count())0.0];
    if(body<0)throw "Clearance envelope link is outside the model";
    while(m.bodyParentJoint[body]>=0){
      var j=m.bodyParentJoint[body];radius+=norm(m.jointJointTChild,7*j);
      switch m.jointKind[j]{
        case Revolute:for(i in 0...coefficients.length)coefficients[i]+=radius*gains[j][i];
        case Prismatic:for(i in 0...coefficients.length)coefficients[i]+=gains[j][i];radius+=travel[j];
        case Fixed:
      }
      radius+=norm(m.jointParentTJoint,7*j);body=m.jointParent[j];
    }
    return {coefficients:coefficients,reach:radius};
  }
  /** For a hull rigidly attached below its first rotary joint, rotation
   * moves each point by its perpendicular lever arm. Axial shaft length
   * contributes no lever. Upstream joints retain the full reach bound. */
  function hull(link:String,corners:Array<Float>):Array<Float> {
    var points=[for(i in 0...Std.int(corners.length/3))new Vec3(corners[3*i],corners[3*i+1],corners[3*i+2])],radius=0.0;
    for(p in points)radius=Math.max(radius,p.norm());
    var result=point(link,radius).coefficients,m=arm.model,body=m.bodyIndex(link);
    while(m.bodyParentJoint[body]>=0){
      var j=m.bodyParentJoint[body],at=7*j;
      radius+=norm(m.jointJointTChild,at);
      var child=Transform3.fromArrays(m.jointJointTChild.slice(at,at+3),m.jointJointTChild.slice(at+3,at+7));
      points=[for(p in points)child.transformPoint(p)];
      switch m.jointKind[j]{
        case Revolute:
          var axis=new Vec3(m.jointAxis[3*j],m.jointAxis[3*j+1],m.jointAxis[3*j+2]).normalized(),lever=0.0;
          for(p in points)lever=Math.max(lever,p.cross(axis).norm());
          var reduction=Math.max(0.0,radius-lever);
          for(i in 0...result.length)result[i]=Math.max(0.0,result[i]-reduction*gains[j][i]);
          return result;
        case Prismatic:return result; // The descendant hull has variable travel.
        case Fixed:
      }
      var parent=Transform3.fromArrays(m.jointParentTJoint.slice(at,at+3),m.jointParentTJoint.slice(at+3,at+7));
      points=[for(p in points)parent.transformPoint(p)];
      radius+=norm(m.jointParentTJoint,at);body=m.jointParent[j];
    }
    return result;
  }
  function angular(link:String):Array<Float> {
    var m=arm.model,body=m.bodyIndex(link),result=[for(_ in 0...arm.group.count())0.0];
    while(m.bodyParentJoint[body]>=0){var j=m.bodyParentJoint[body];
      if(m.jointKind[j]==JointKind.Revolute)for(i in 0...result.length)result[i]+=gains[j][i];
      body=m.jointParent[j];}
    return result;
  }
  function validate(errors:Array<Float>):Void {
    if(errors.length!=arm.group.count())throw "Invalid clearance envelope error dimensions";
    for(e in errors)if(!Math.isFinite(e) || e<0)throw "Invalid clearance envelope error";
  }
  static function dot(a:Array<Float>,b:Array<Float>):Float {var value=0.0;for(i in 0...a.length)value+=a[i]*b[i];return value;}
  public function bounds(errors:Array<Float>):Array<Float> {validate(errors);return [for(row in hulls)dot(row,errors)];}
  public function tcp(errors:Array<Float>):Float {validate(errors);return dot(tip,errors);}
}
