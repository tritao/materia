import machinekit.assembly.PosedParts;
import materia.assembly.AssemblyFrames as F;

/** Geometric rack support and mounting checks; generic device mounting holes remain unspecified. */
class HobberElectricalChecks {
  public static function run(machine:FlexsplineHobber):Void {
    var members=machine.components(),poses=machine.solvedPoses(),cache=new PosedParts();
    try {
      var rack=[for(m in members)if(m.id=="electricalPanel")m][0],rackPose=poses.get(rack.id);
      var rackBox=cache.box(rack.component,rackPose);
      if(Math.abs(rackBox.minZ)>1e-6)throw "Electrical stand is not on the machine floor datum";
      var panel=HobberElectricalLayout.panelPose(),count=0;
      var devices=[for(m in members)if(m.id=="powerSupply"||StringTools.endsWith(m.id,"-driver"))m];
      for(m in devices){
        var pose=poses.get(m.id),solid=cache.posed(m.component,pose),work=PosedParts.moved(solid,F.inverse(FlexsplineHobber.MACHINE_FROM_WORK));solid.close();
        var b=PosedParts.boxOf(work);work.close();
        if(Math.abs(b.maxY-(panel.y-10))>1e-6)throw '${m.id} mounting face does not meet the backplate';
        if(b.minX<panel.x-210||b.maxX>panel.x+210||b.minZ<panel.z+20||b.maxZ>panel.z+850)
          throw '${m.id} extends beyond the supported backplate area';
        var overlap=cache.overlap(m.component,pose,rack.component,rackPose);
        if(overlap>0.001)throw '${m.id} penetrates the backplate/stand by $overlap mm3';
        for(other in devices)if(other.id!=m.id && cache.overlap(m.component,pose,other.component,poses.get(other.id))>0.001)
          throw 'Electrical items overlap: ${m.id} and ${other.id}';
        count++;
      }
      if(count!=7)throw "Electrical layout must mount one supply and six drives";
      cache.close();Sys.println("Electrical rack: floor support and seven flush, non-overlapping device mounts passed");
    }catch(error:Dynamic){cache.close();throw error;}
  }
}
