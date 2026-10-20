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
      var canonical=F.inverse(FlexsplineHobber.MACHINE_FROM_WORK),centre=F.compose(canonical,cutter);
      for(id in ["railYLeft","railYRight"]){
        var frame=F.compose(canonical,poses.get(id)),normal=F.transformVector(frame,0,1,0),axis=F.transformVector(frame,0,0,1);
        count++;if(Math.abs(frame.x-centre.x-70)>1e-8||Math.abs(normal.x+1)+Math.abs(normal.y)+Math.abs(normal.z)>1e-8||Math.abs(axis.y+1)+Math.abs(axis.x)+Math.abs(axis.z)>1e-8)throw "Y guides must lie on the radial vertical face, 70 mm behind hob";
      }
      var face=F.compose(canonical,F.compose(poses.get("swivel/bearing"),F.translation(0,0,machine.swivel.bearing.parameterNumber("height"))));
      var mount=F.compose(canonical,poses.get("swivel/housing"));
      count++;if(Math.abs(face.x-centre.x-114)>1e-8||Math.abs(mount.x-centre.x-135)>1e-8||face.x>=mount.x)throw "swivel bearing must be in front of the Z saddle";
      var screw=F.compose(canonical,poses.get("Xscrew"));
      count++;if(Math.abs(screw.y)>1e-8||Math.abs(screw.z-180)>1e-8)throw "X screw must lie centrally in recessed bed channel";
      var zScrew=F.compose(canonical,poses.get("Zscrew")),zl=F.compose(canonical,poses.get("railZLeft")),zr=F.compose(canonical,poses.get("railZRight"));
      var zd=F.transformVector(zScrew,0,0,1);
      count++;if(Math.abs(zScrew.y-(zl.y+zr.y)/2)>1e-8||Math.abs(zScrew.x-zl.x-19)>1e-8||Math.abs(zd.z+1)>1e-8)throw "Z screw must lie centrally behind the guide plane and point down";
      var zFixed=F.compose(canonical,poses.get("Zfixed")),zFloating=F.compose(canonical,poses.get("Zfloating"));
      count++;if(zFixed.z<=zFloating.z)throw "Z fixed bearing must be at the top";
      var motor=F.compose(canonical,poses.get("Xmotor"));
      count++;if(motor.x<screw.x)throw "X motor must be at rear end of central drive";
      for(m in members)if(Std.isOfType(m.component,HobberSaddle)){
        count++;if(["A-yoke","Y-saddle","X-bridge","Z-saddle"].indexOf(m.component.values().token("design"))>=0)throw "retired overhead saddle installed";
      }
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
        var axis=m.id.substr(5,1),railId="rail"+axis+(m.id.indexOf("Left")>=0?"Left":"Right");
        var rail:machinekit.motion.LinearRail=cast members.filter(p->p.id==railId)[0].component;
        var block:machinekit.motion.LinearRailBlock=cast m.component;
        count++;if(!Std.isOfType(block,machinekit.motion.SupplierRailBlock))throw "hobber must use full supplier carriage geometry";
        count++;if(axis=="X"&&!Std.isOfType(rail,machinekit.motion.HiwinHgr25Rail))throw "X must use matching supplier rail profile";
        var relative=F.compose(F.inverse(poses.get(railId)),poses.get(m.id)),travel=machine.travel.filter(t->t.id==axis)[0];
        count++;if(Math.abs(relative.x)+Math.abs(relative.y)>1e-7)throw 'block off guide datum ${m.id}';
        var lo=relative.z+travel.lower-travel.initial-block.spec.blockLength/2-rail.spec.railEndMargin;
        var hi=rail.length-(relative.z+travel.upper-travel.initial+block.spec.blockLength/2)-rail.spec.railEndMargin;
        count++;if(lo<5||hi<5)throw 'guide lacks declared 5 mm overtravel ${m.id}: $lo/$hi';
      }
      // Verify physical nut/carrier travel, including reserve beyond both soft stops.
      // Use the full mechanism lower bound even when this fixture imposes a higher cutting stop.
      var xTravel=machine.travel.filter(t->t.id=="X")[0];
      var screwPart:machinekit.motion.BallScrew=cast members.filter(m->m.id=="Xscrew")[0].component;
      var nutPart:machinekit.motion.BallNut=cast members.filter(m->m.id=="Xnut")[0].component;
      for(x in [machinekit.hobbing.HobberXLayout.MIN-5,xTravel.upper+5]){
        // Translation vector only; keep the mounted part's existing orientation.
        var delta=F.transformVector(FlexsplineHobber.MACHINE_FROM_WORK,x-xTravel.initial,0,0);
        var nutPose=F.compose(F.translation(delta.x,delta.y,delta.z),poses.get("Xnut"));
        var relative=F.compose(F.inverse(poses.get("Xscrew")),nutPose);
        var n0=F.compose(relative,F.translation(0,0,0)),n1=F.compose(relative,F.translation(0,0,nutPart.bodyLength+nutPart.flangeThickness));
        count++;if(Math.min(n0.z,n1.z)<screwPart.inputJournalLength+10-1e-7||Math.max(n0.z,n1.z)>screwPart.totalLength-machinekit.motion.BallScrew.JOURNAL_LENGTH-10+1e-7)
          throw 'X nut lacks 10 mm race allowance at reserve travel $x';
        for(id in ["Xnut","Xbridge"]){
          var moving=members.filter(m->m.id==id)[0],pose=F.compose(F.translation(delta.x,delta.y,delta.z),poses.get(id));
          for(supportId in ["Xfixed","Xfloating"]){var fixed=members.filter(m->m.id==supportId)[0];
            var a=cache.posed(moving.component,pose),b=cache.posed(fixed.component,poses.get(supportId));
            var gap=a.distance(b);a.close();b.close();count++;
            if(gap<10-1e-7)throw 'X carrier/support clearance below 10 mm at reserve travel $x: $id/$supportId $gap';
          }
        }
      }
      // A point contact can pass while a rail seat overhangs a well. Check its full backing volume.
      var foundations:Array<{kind:String, foundation:machinekit.structural.MineralFoundation}> = [
        {kind:"bed", foundation:machine.bed}, {kind:"column", foundation:machine.column}];
      for(item in foundations){
        var foundation=item.foundation;
        var localPoses=foundation.solvedPoses(),body=cache.posed(foundation.body,localPoses.get("casting"));
        try {
          if(item.kind=="bed"){
            var drive=foundation.insertLocations().filter(p->p.id=="driveSeat")[0];
            var raw=cadkit.modeling.Part.box(drive.part.parameterNumber("width"),drive.part.parameterNumber("depth"),50);
            var slab=PosedParts.moved(raw,F.compose(drive.pose,F.translation(0,0,-50)));raw.close();
            var missing=slab.subtract(body),volume=missing.volume();missing.close();slab.close();
            count++;if(volume>0.001)throw 'X drive insert lacks 50 mm full-footprint casting backing: $volume mm3';
            for(probe in HobberDriveSupportChecks.ribProbes(body)){count++;if(probe.missingMm3>0.001)throw 'Missing casting load path ${probe.label}: ${probe.missingMm3} mm3';}
          }
          if(item.kind=="column"){
            var drive=foundation.insertLocations().filter(p->p.id=="driveSeat")[0];
            var raw=cadkit.modeling.Part.box(drive.part.parameterNumber("width"),drive.part.parameterNumber("depth"),125);
            var slab=PosedParts.moved(raw,F.compose(drive.pose,F.translation(0,0,-125)));raw.close();
            var missing=slab.subtract(body),volume=missing.volume();missing.close();slab.close();
            count++;if(volume>0.001)throw 'Central Z drive seat lacks 125 mm full casting backing: $volume mm3';
          }
          for(row in foundation.insertLocations())if(row.id=="railLeft"||row.id=="railRight"){
            var backing=item.kind=="bed"?row.pose.z:10.0;
            var raw=cadkit.modeling.Part.box(row.part.parameterNumber("width"),row.part.parameterNumber("depth"),backing);
            var slab=PosedParts.moved(raw,F.compose(localPoses.get(row.id),F.translation(0,0,-backing)));raw.close();
            var unsupported=slab.subtract(body),missing=unsupported.volume();unsupported.close();slab.close();
            count++;if(missing>0.001)throw 'rail insert ${item.kind}/${row.id} lacks full casting backing: $missing mm3';
          }
        }catch(e:Dynamic){body.close();throw e;}body.close();
      }
      // Tray drops in from above with bed guides and table fitted, before the moving column/head.
      var tray=members.filter(m->m.id=="chipTray")[0].component;
      for(lift in [0.0,25,50,100,200,400]){
        var trayPose=F.compose(F.translation(0,0,lift),poses.get("chipTray"));
        for(m in members)if(StringTools.startsWith(m.id,"bed/")||StringTools.startsWith(m.id,"railX")||StringTools.startsWith(m.id,"table/")||m.id=="XdriveFrame"||m.id=="Xfixed"||m.id=="Xfloating"||m.id=="Xscrew"){
          var overlap=cache.overlap(tray,trayPose,m.component,poses.get(m.id));
          count++;if(overlap>0.001)throw 'chip tray cannot be installed vertically past ${m.id} at lift $lift mm: $overlap mm3';
        }
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
