package kinematicskit;

import kinematicskit.JointKind;

/** Fixed reach coefficients over a finite joint box. The box must include
 * the entire geometry and lowering neighborhood. All affine follower terms
 * contribute; joints outside the group keep their compiled held values. */
class ClearanceMotionEnvelope {
  final model:KinematicModel;
  final count:Int;
  final travel:Array<Float>;
  final gains:Array<Array<Float>>;
  final hulls:Array<Array<Float>>;
  final tip:Array<Float>;
  final tipAngular:Array<Float>;
  final tipPrismatic:Array<Float>;
  final taylorAngles:Array<Float>;
  final tipReach:Float;
  public function new(model:KinematicModel, dofs:Array<Int>, state:KinematicState,
      lower:Array<Float>, upper:Array<Float>, bodies:Array<Int>, corners:Array<Array<Float>>,
      flangeFrame:Int, tcpOffset:Transform, workFrame:Int = -1) {
    if (model == null || state == null || state.model != model) throw "Motion envelope requires a state of its model";
    this.model=model;this.count=dofs.length;
    var n=count,m=model;
    if(lower.length!=n || upper.length!=n || bodies.length!=corners.length)throw "Invalid clearance envelope dimensions";
    for(i in 0...n) {
      if(dofs[i]<0 || dofs[i]>=m.dofCount() || dofs.indexOf(dofs[i])!=i)throw "Invalid clearance envelope DOFs";
      if(!Math.isFinite(lower[i]) || !Math.isFinite(upper[i]) || lower[i]>upper[i])throw "Invalid clearance envelope joint box";
    }
    if(flangeFrame<0 || flangeFrame>=m.frameCount() || workFrame < -1 || workFrame>=m.frameCount())throw "Invalid motion envelope frame";
    tcpOffset=Transform.checked(tcpOffset,"Motion envelope TCP offset");
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
    hulls=[for(i in 0...bodies.length)hull(bodies[i],corners[i])];
    var f=flangeFrame,flange=point(m.frameBody[f],norm(m.frameOffset,7*f)+Math.sqrt(tcpOffset.x*tcpOffset.x+tcpOffset.y*tcpOffset.y+tcpOffset.z*tcpOffset.z));
    tip=flange.coefficients;
    var flangeLink=m.frameBody[f],reach=flange.reach;
    tipAngular=angular(flangeLink);taylorAngles=tipAngular.copy();tipPrismatic=prismatic(flangeLink);
    if(workFrame>=0){
      var w=workFrame,link=m.frameBody[w],work=point(link,norm(m.frameOffset,7*w)),angles=angular(link);
      var workPrismatic=prismatic(link);
      for(i in 0...n){
        tip[i]+=work.coefficients[i]+angles[i]*(flange.reach+work.reach);
        tipAngular[i]+=angles[i];taylorAngles[i]+=2*angles[i];tipPrismatic[i]+=workPrismatic[i];
      }
      reach+=work.reach;
    }
    tipReach=reach;
  }
  static function transform(v:Array<Float>,i:Int):Transform
    return new Transform(v[i],v[i+1],v[i+2],v[i+3],v[i+4],v[i+5],v[i+6]);
  static function length(v:Vector3):Float return Math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z);
  static function unit(v:Vector3):Vector3 {var n=length(v);return new Vector3(v.x/n,v.y/n,v.z/n);}
  static function crossLength(a:Vector3,b:Vector3):Float {
    var x=a.y*b.z-a.z*b.y,y=a.z*b.x-a.x*b.z,z=a.x*b.y-a.y*b.x;
    return Math.sqrt(x*x+y*y+z*z);
  }
  static function norm(v:Array<Float>,i:Int):Float return Math.sqrt(v[i]*v[i]+v[i+1]*v[i+1]+v[i+2]*v[i+2]);
  function point(link:Int,radius:Float):{coefficients:Array<Float>,reach:Float} {
    if(!Math.isFinite(radius) || radius<0)throw "Invalid clearance envelope radius";
    var m=model,body=link,coefficients=[for(_ in 0...count)0.0];
    if(body<0 || body>=m.bodyCount())throw "Clearance envelope link is outside the model";
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
  function hull(link:Int,corners:Array<Float>):Array<Float> {
    var points=[for(i in 0...Std.int(corners.length/3))new Vector3(corners[3*i],corners[3*i+1],corners[3*i+2])],radius=0.0;
    for(p in points)radius=Math.max(radius,length(p));
    var result=point(link,radius).coefficients,m=model,body=link;
    while(m.bodyParentJoint[body]>=0){
      var j=m.bodyParentJoint[body],at=7*j;
      radius+=norm(m.jointJointTChild,at);
      var child=transform(m.jointJointTChild,at);
      points=[for(p in points)child.transformPoint(p.x,p.y,p.z)];
      switch m.jointKind[j]{
        case Revolute:
          var axis=unit(new Vector3(m.jointAxis[3*j],m.jointAxis[3*j+1],m.jointAxis[3*j+2])),lever=0.0;
          for(p in points)lever=Math.max(lever,crossLength(p,axis));
          var reduction=Math.max(0.0,radius-lever);
          for(i in 0...result.length)result[i]=Math.max(0.0,result[i]-reduction*gains[j][i]);
          return result;
        case Prismatic:return result; // The descendant hull has variable travel.
        case Fixed:
      }
      var parent=transform(m.jointParentTJoint,at);
      points=[for(p in points)parent.transformPoint(p.x,p.y,p.z)];
      radius+=norm(m.jointParentTJoint,at);body=m.jointParent[j];
    }
    return result;
  }
  function angular(link:Int):Array<Float> {
    var m=model,body=link,result=[for(_ in 0...count)0.0];
    while(m.bodyParentJoint[body]>=0){var j=m.bodyParentJoint[body];
      if(m.jointKind[j]==JointKind.Revolute)for(i in 0...result.length)result[i]+=gains[j][i];
      body=m.jointParent[j];}
    return result;
  }
  function prismatic(link:Int):Array<Float> {
    var m=model,body=link,result=[for(_ in 0...count)0.0];
    while(m.bodyParentJoint[body]>=0){var j=m.bodyParentJoint[body];
      if(m.jointKind[j]==JointKind.Prismatic)for(i in 0...result.length)result[i]+=gains[j][i];
      body=m.jointParent[j];}
    return result;
  }
  /** Bounds the remainder after the TCP Jacobian's linear term. Each rigid
   * offset contributes |v| A²/2, each affine prismatic travel contributes
   * |t|max A²/2 + |delta t| A. For a moving reference, a work-side offset can
   * rotate on both sides, hence two work-chain angular sums. */
  public function tcpLinearizationRemainder(errors:Array<Float>):Float {
    validate(errors);var angle=dot(taylorAngles,errors),travelError=dot(tipPrismatic,errors);
    return tipReach*angle*angle/2+travelError*angle;
  }
  public function angularBound(errors:Array<Float>):Float {validate(errors);return dot(tipAngular,errors);}
  function validate(errors:Array<Float>):Void {
    if(errors.length!=count)throw "Invalid clearance envelope error dimensions";
    for(e in errors)if(!Math.isFinite(e) || e<0)throw "Invalid clearance envelope error";
  }
  static function dot(a:Array<Float>,b:Array<Float>):Float {var value=0.0;for(i in 0...a.length)value+=a[i]*b[i];return value;}
  public function bounds(errors:Array<Float>):Array<Float> {validate(errors);return [for(row in hulls)dot(row,errors)];}
  public function tcp(errors:Array<Float>):Float {validate(errors);return dot(tip,errors);}
}
