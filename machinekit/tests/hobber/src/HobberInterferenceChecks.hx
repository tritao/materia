import machinekit.assembly.PosedParts;
import gearkit.GearSpec;

/** Exact solid audit: includes fixed and as-designed contacts hidden by scene collision rules. */
class HobberInterferenceChecks {
  public static function run():Void {
    var output=Sys.getEnv("HOBBER_INTERFERENCE_EVIDENCE");if(output==null)throw "HOBBER_INTERFERENCE_EVIDENCE required";
    var reference=Sys.args().indexOf("reference")>=0;
    var machine=reference?new FlexsplineHobber():new FlexsplineHobber(new GearSpec(1,40,4,20*Math.PI/180),4);
    var members=machine.components(),poses=machine.solvedPoses(),definition=machine.definition(),cache=new PosedParts();
    var boxes=[for(m in members)cache.box(m.component,poses.get(m.id))],overlaps:Array<Dynamic> =[],checked=0,began=Sys.time();
    function save():Void sys.io.File.saveContent(output+"/overlaps.json",haxe.Json.stringify({reference:reference,
      scope:"exact CAD common volume at assembled initial pose; includes same-body and designed contacts; not a swept-motion clearance proof",
      members:members.length,checked:checked,thresholdMm3:0.001,seconds:Sys.time()-began,overlaps:overlaps}));
    try {
      for(a in 0...members.length)for(b in a+1...members.length){
        if(PosedParts.apart(boxes[a],boxes[b],0))continue;
        var first=members[a],second=members[b];checked++;
        var volume=cache.overlap(first.component,poses.get(first.id),second.component,poses.get(second.id));
        if(volume>0.001){var row={a:first.id,b:second.id,volumeMm3:volume,
          aDesignation:first.component.designation,bDesignation:second.component.designation,
          directJoints:[for(j in definition.joints)if((j.parent==first.id&&j.child==second.id)||(j.child==first.id&&j.parent==second.id))j.id]};
          overlaps.push(row);Sys.println("HOBBER_SOLID_OVERLAP "+haxe.Json.stringify(row));Sys.stdout().flush();save();}
        if(checked%25==0){save();Sys.println('HOBBER_INTERFERENCE_PROGRESS checked=$checked positives=${overlaps.length} seconds=${Sys.time()-began}');Sys.stdout().flush();}
      }
      save();Sys.println('HOBBER_INTERFERENCE_COMPLETE checked=$checked positives=${overlaps.length} seconds=${Sys.time()-began}');
    }catch(error:Dynamic){cache.close();throw error;}
    cache.close();
  }
}
