import machinekit.assembly.PosedParts;
import machinekit.assembly.MachineAssembly;
import machinekit.component.MachineKitComponents;
import machinekit.hobbing.HobberSaddle;
import materia.assembly.AssemblyFrames as F;

/** Physical load paths and recipe reconstruction, independent of scene collision exclusions. */
class HobberDesignChecks {
  public static function run(machine:FlexsplineHobber):Int {
    var cache=new PosedParts(),poses=machine.solvedPoses(),members=machine.components(),count=0;
    try {
      var seat=F.compose(poses.get("spindle/arbor"),machine.spindle.arbor.connector("hobCentre").frame);
      var cutter=F.compose(poses.get("hob"),machine.hob.connector("centre").frame);
      count++;if(Math.abs(seat.x-cutter.x)+Math.abs(seat.y-cutter.y)+Math.abs(seat.z-cutter.z)>1e-7)throw "arbor hob seat differs from actual cutter datum";
      for(mount in machine.mountContacts){
        var first=members.filter(m->m.id==mount.parent)[0],second=members.filter(m->m.id==mount.child)[0];
        var a=cache.posed(first.component,poses.get(first.id)),b=cache.posed(second.component,poses.get(second.id));
        var gap=0.0;try gap=a.distance(b)catch(e:Dynamic){a.close();b.close();throw e;}
        a.close();b.close();count++;if(gap>0.000001)throw 'unsupported mount ${first.id}/${second.id}: $gap mm gap';
      }
      for(m in members)if(Std.isOfType(m.component,HobberSaddle)){
        count++;if(cache.local(m.component).solidCount()!=1)throw 'disconnected designed weldment ${m.id}';
      }
      for(m in members)if(Std.isOfType(m.component,machinekit.motion.LinearRailBlock)){
        var axis=m.id.substr(5,1),railId=axis=="Y"?"railY":"rail"+axis+(m.id.indexOf("Left")>=0?"Left":"Right");
        var rail:machinekit.motion.LinearRail=cast members.filter(p->p.id==railId)[0].component;
        var block:machinekit.motion.LinearRailBlock=cast m.component;
        var relative=F.compose(F.inverse(poses.get(railId)),poses.get(m.id)),travel=machine.travel.filter(t->t.id==axis)[0];
        count++;if(Math.abs(relative.x)+Math.abs(relative.y)>1e-7)throw 'block off guide datum ${m.id}';
        var lo=relative.z+travel.lower-travel.initial-block.spec.blockLength/2-rail.spec.railEndMargin;
        var hi=rail.length-(relative.z+travel.upper-travel.initial+block.spec.blockLength/2)-rail.spec.railEndMargin;
        count++;if(lo<5||hi<5)throw 'guide lacks declared 5 mm overtravel ${m.id}: $lo/$hi';
      }
      var rebuilt=MachineAssembly.decode(machine.encode(),MachineKitComponents.standard()),restored=rebuilt.solvedPoses();
      count++;if(rebuilt.components().length!=members.length)throw "redesigned machine lost members in persistence";
      for(m in members){var original:materia.assembly.AssemblyRecord.AssemblyFrame=poses.get(m.id);
        var copy:materia.assembly.AssemblyRecord.AssemblyFrame=restored.get(m.id);count++;
        if(Math.abs(original.x-copy.x)+Math.abs(original.y-copy.y)+Math.abs(original.z-copy.z)>1e-7)throw 'restored mount moved ${m.id}';
        for(axis in 0...3){var u=F.transformVector(original,axis==0?1:0,axis==1?1:0,axis==2?1:0),v=F.transformVector(copy,axis==0?1:0,axis==1?1:0,axis==2?1:0);
          count++;if(Math.abs(u.x-v.x)+Math.abs(u.y-v.y)+Math.abs(u.z-v.z)>1e-7)throw 'restored mount rotated ${m.id}';}
      }
    }catch(error:Dynamic){cache.close();throw error;}
    cache.close();return count;
  }
}
