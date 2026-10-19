package machinekit.hobbing;

import machinekit.assembly.MachineAssembly;
import machinekit.assembly.Diagnostics;
import machinekit.structural.CastInsert;
import machinekit.structural.MineralCasting;
import materia.assembly.AssemblyFrames as F;

/** Geometry/tolerance checks only. No structural, modal or process adequacy claim. */
class FoundationIntegrity {
  public static function check(foundation:MineralFoundation,?measured:Map<String,materia.assembly.AssemblyRecord.AssemblyFrame>):Diagnostics {
    var out=new Diagnostics(),poses=measured==null?foundation.solvedPoses():measured,bodyPose=poses.get("casting");
    if(bodyPose==null)throw "missing casting pose";
    for(row in FoundationLayout.inserts(foundation.kind)){
      var actual=poses.get(row.id),expected=F.compose(bodyPose,row.pose);
      if(actual==null)throw "missing insert pose";
      var part=foundation.components().filter(m->m.id==row.id)[0].component;
      var insert:machinekit.structural.CastInsert=cast part,tolerance=insert.parameterNumber("positionTolerance");
      var distance=Math.sqrt(Math.pow(actual.x-expected.x,2)+Math.pow(actual.y-expected.y,2)+Math.pow(actual.z-expected.z,2));
      var normal=F.transformVector(actual,0,0,1),nominal=F.transformVector(expected,0,0,1);
      var tilt=Math.sqrt(Math.pow(normal.x-nominal.x,2)+Math.pow(normal.y-nominal.y,2)+Math.pow(normal.z-nominal.z,2));
      if(distance+tilt*Math.max(row.width,row.depth)>tolerance)
        out.error("foundation.insert-plane",row.id,'Insert "${row.id}" is outside its position/face-parallelism tolerance');
    }
    var left=poses.get("railLeft"),right=poses.get("railRight");
    if(left==null||right==null)throw "missing paired rail inserts";
    var inserts=foundation.components().filter(m->m.id=="railLeft"||m.id=="railRight");
    var first:machinekit.structural.CastInsert=cast inserts[0].component,second:machinekit.structural.CastInsert=cast inserts[1].component;
    var a=F.transformPoint(left,0,0,20),b=F.transformPoint(right,0,0,20),normal=F.transformVector(left,0,0,1);
    var offset=Math.abs(normal.x*(b.x-a.x)+normal.y*(b.y-a.y)+normal.z*(b.z-a.z));
    var otherNormal=F.transformVector(right,0,0,1);
    var tilt=Math.sqrt(Math.pow(normal.x-otherNormal.x,2)+Math.pow(normal.y-otherNormal.y,2)+Math.pow(normal.z-otherNormal.z,2));
    if(offset+tilt*Math.max(first.parameterNumber("depth"),second.parameterNumber("depth"))>Math.max(first.parameterNumber("positionTolerance"),second.parameterNumber("positionTolerance")))
      out.error("foundation.insert-coplanarity","railLeft/railRight","Rail insert finished faces exceed their coplanarity tolerance");
    return out;
  }
  /** Explicit precision mount list. All foundation-side endpoints must be steel inserts. */
  public static function checkMounts(assembly:MachineAssembly,mounts:Array<{name:String,parent:String}>):Diagnostics {
    var out=new Diagnostics(),members=assembly.components(),joints=assembly.definition().joints;
    for(m in mounts){var found=members.filter(p->p.id==m.parent),joint=joints.filter(j->j.id==m.name);
      if(found.length!=1||!Std.isOfType(found[0].component,CastInsert)||joint.length!=1||joint[0].parent!=m.parent)
        out.error("foundation.precision-mount",m.name,'Precision mount "${m.name}" must actually attach to cast-in steel insert "${m.parent}"');}
    for(j in joints){var parent=members.filter(p->p.id==j.parent),child=members.filter(p->p.id==j.child);
      if(parent.length==1&&child.length==1&&Std.isOfType(parent[0].component,MineralCasting)
        &&(Std.isOfType(child[0].component,machinekit.motion.LinearRail)||Std.isOfType(child[0].component,TableHousing)))
        out.error("foundation.precision-mount",j.id,'Precision mount "${j.id}" attaches directly to the casting body');}
    return out;
  }
}
