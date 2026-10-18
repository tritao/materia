package machinekit.structural;

import machinekit.assembly.MachineAssembly;
import machinekit.assembly.Diagnostics;
import materia.assembly.AssemblyFrames as F;

/** Geometry/tolerance checks only. No structural, modal or process adequacy claim. */
class FoundationIntegrity {
  public static function check(foundation:MineralFoundation,?measured:Map<String,materia.assembly.AssemblyRecord.AssemblyFrame>):Diagnostics {
    var out=new Diagnostics(),poses=measured==null?foundation.solvedPoses():measured,bodyPose=poses.get("casting");
    if(bodyPose==null)throw "missing casting pose";
    for(row in foundation.insertLocations()){
      var actual=poses.get(row.id),expected=F.compose(bodyPose,row.pose);
      if(actual==null)throw "missing insert pose";
      var part=foundation.components().filter(m->m.id==row.id)[0].component;
      var insert:CastInsert=cast part,tolerance=insert.parameterNumber("positionTolerance");
      var distance=Math.sqrt(Math.pow(actual.x-expected.x,2)+Math.pow(actual.y-expected.y,2)+Math.pow(actual.z-expected.z,2));
      var normal=F.transformVector(actual,0,0,1),nominal=F.transformVector(expected,0,0,1);
      var tilt=Math.sqrt(Math.pow(normal.x-nominal.x,2)+Math.pow(normal.y-nominal.y,2)+Math.pow(normal.z-nominal.z,2));
      if(distance+tilt*Math.max(row.part.parameterNumber("width"),row.part.parameterNumber("depth"))>tolerance)
        out.error("foundation.insert-plane",row.id,'Insert "${row.id}" is outside its position/face-parallelism tolerance');
    }
    return out;
  }
  /** Explicit precision mount list. All foundation-side endpoints must be steel inserts. */
  public static function checkMounts(assembly:MachineAssembly,mounts:Array<{name:String,parent:String}>):Diagnostics {
    var out=new Diagnostics(),members=assembly.components();
    for(m in mounts){var found=members.filter(p->p.id==m.parent);
      if(found.length!=1||!Std.isOfType(found[0].component,CastInsert))out.error("foundation.precision-mount",m.name,'Precision mount "${m.name}" must land on a cast-in steel insert, got "${m.parent}"');}
    return out;
  }
}
