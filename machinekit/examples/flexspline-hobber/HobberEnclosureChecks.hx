import machinekit.assembly.PosedParts;
import machinekit.assembly.MachineAssembly;
import machinekit.enclosure.EnclosureCutPanel;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;

class HobberEnclosureChecks {
  public static function main():Void {
    HobberElectricalChecks.run(new FlexsplineHobber());
    var fitted=HobberEnclosurePreview.assembled(),assembly=fitted.assembly,shell=fitted.shell;
    var guardReport=HobberEnclosureGuardChecks.run(assembly,shell.layout.stroke);
    if(assembly.validate().length!=0)throw "Enclosed hobber assembly does not validate";
    var model=new cadkit.modeling.AssemblyModel("mm");assembly.addTo(model,"");var state=model.initialState();
    var parts=assembly.components(),machine=[for(p in parts)if(StringTools.startsWith(p.id,"machine/"))p],panels=[for(p in parts)if(StringTools.startsWith(p.id,"enclosure"))p];
    var cache=new PosedParts(),checked=0,samples=0;
    var verifiedPairs:Map<String,Bool> = [];
    try {
      // All four outriggers must meet an underside extrusion, independently of the tray.
      var mounts=[for(p in panels)if(StringTools.startsWith(p.id,"enclosureBaseMount"))p];
      if(mounts.length!=4)throw "Expected four independent casting-to-frame supports";
      for(m in mounts) {
        var box=cache.box(m.component,state.worldPose(m.id)),contact=false;
        for(p in panels)if(StringTools.startsWith(p.id,"enclosure/frame/side0")) {
          var beam=cache.box(p.component,state.worldPose(p.id));
          if(Math.abs(box.maxZ-beam.minZ)<0.001&&Math.min(box.maxX,beam.maxX)-Math.max(box.minX,beam.minX)>1&&
              Math.min(box.maxY,beam.maxY)-Math.max(box.minY,beam.minY)>1)contact=true;
        }
        if(!contact)throw "Frame support does not meet lower extrusion: "+m.id+" "+haxe.Json.stringify(box)+" beams "+haxe.Json.stringify([for(p in panels)if(StringTools.startsWith(p.id,"enclosure/frame/side0"))cache.box(p.component,state.worldPose(p.id))]);
        for(p in panels)if(p.id!=m.id) {
          var pose=state.worldPose(p.id);
          if(!PosedParts.apart(box,cache.box(p.component,pose),0)&&cache.overlap(m.component,state.worldPose(m.id),p.component,pose)>0.001)
            throw 'Frame mount ${m.id} interferes with ${p.id}';
        }
      }
      var tray=[for(p in panels)if(["enclosure/trayLeft","enclosure/trayRight","enclosure/trayFront","enclosure/trayRearLeft","enclosure/trayRearRight","enclosure/trayRearCentre"].indexOf(p.id)>=0)p];
      if(tray.length!=6)throw "Tray must have six separately removable blanks";
      for(p in tray) {
        var panel:machinekit.enclosure.EnclosurePanel=cast p.component,box=cache.box(p.component,state.worldPose(p.id));
        if(panel.construction.id!="compact-laminate-13"||Math.abs(box.minZ-(shell.layout.input.floorZ-2))>0.001||Math.abs(box.maxZ-(shell.layout.input.floorZ+11))>0.001)
          throw "Incorrect laminate tray stack: "+p.id;
      }
      var trayBox=cache.box(tray[0].component,state.worldPose(tray[0].id));
      var supports=[for(p in panels)if(StringTools.startsWith(p.id,"enclosure/tray")&&StringTools.endsWith(p.id,"Support"))p];
      if(supports.length!=4)throw "Expected four separate tray support angles";
      for(p in supports) {
        var box=cache.box(p.component,state.worldPose(p.id));
        if(Math.abs(box.maxZ+machinekit.enclosure.TraySeat.GASKET-trayBox.minZ)>0.001)throw "Tray is not seated on "+p.id;
        for(other in panels)if(other.id!=p.id) {
          var pose=state.worldPose(other.id);
          if(!PosedParts.apart(box,cache.box(other.component,pose),0)&&cache.overlap(p.component,state.worldPose(p.id),other.component,pose)>0.001)
            throw 'Tray support ${p.id} interferes with ${other.id}';
        }
      }
      var clamps=[for(p in panels)if(StringTools.startsWith(p.id,"enclosureTrayClamp"))p];
      if(clamps.length!=12)throw "Expected ten tray clamps and two rear-cover clamps";
      for(p in clamps) {
        var contact=false;
        for(tile in panels)if(tray.indexOf(tile)>=0||tile.id=="enclosure/trayRearServiceBridge") {
          var pose=state.worldPose(tile.id),lift=materia.assembly.AssemblyFrames.compose(materia.assembly.AssemblyFrames.translation(0,0,-0.1),state.worldPose(p.id));
          if(cache.overlap(p.component,lift,tile.component,pose)>1)contact=true;
          if(cache.overlap(p.component,state.worldPose(p.id),tile.component,pose)>0.001)throw "Clamp penetrates tray: "+p.id;
        }
        if(!contact)throw "Clamp toe misses all tray pieces: "+p.id;
      }
      var trayParts=[for(p in panels)if(StringTools.startsWith(p.id,"enclosure/tray")||StringTools.startsWith(p.id,"enclosureTray"))p];
      for(i in 0...trayParts.length)for(j in i+1...trayParts.length) {
        var p=trayParts[i],q=trayParts[j],pp=state.worldPose(p.id),qp=state.worldPose(q.id);
        if(!PosedParts.apart(cache.box(p.component,pp),cache.box(q.component,qp),0)&&cache.overlap(p.component,pp,q.component,qp)>0.001)
          throw 'Tray construction interference: ${p.id} / ${q.id}';
      }
      // Undo clamps/cover and cut the joint sealant before lifting one blank; frame stays assembled.
      for(tile in tray)for(step in 1...6) {
        var pose=materia.assembly.AssemblyFrames.compose(materia.assembly.AssemblyFrames.translation(0,0,step*5),state.worldPose(tile.id));
        for(other in parts)if(other.id!=tile.id&&!StringTools.startsWith(other.id,"enclosureTray")&&
            !StringTools.startsWith(other.id,"enclosure/trayBridge")&&other.id!="enclosure/trayRearServiceBridge") {
          var otherPose=state.worldPose(other.id);
          if(!PosedParts.apart(cache.box(tile.component,pose),cache.box(other.component,otherPose),0)&&cache.overlap(tile.component,pose,other.component,otherPose)>0.001)
            throw 'Tray removal obstructed: ${tile.id} / ${other.id} at lift '+step*5;
        }
      }
      var source=new FlexsplineHobber(),ranges=[for(t in source.travel)[t.lower,t.upper]];
      for(x in ranges[0])for(y in ranges[1])for(z in ranges[2])for(a in [-Math.PI/12,0.0,Math.PI/12]) {
        state.setJoint("machine/X",x);state.setJoint("machine/Y",y);state.setJoint("machine/Z",z);state.setJoint("machine/"+FlexsplineHobber.A,a);
        for(door in [0.0,shell.layout.stroke]) {
          state.setJoint("enclosure/door/slide",door);state.forwardKinematics();
          for(p in panels)for(m in machine) {
            var pp=state.worldPose(p.id),mp=state.worldPose(m.id);
            if(PosedParts.apart(cache.box(p.component,pp),cache.box(m.component,mp),0))continue;
            // Static contacts recur at every axis sample: verify each exact posed pair once.
            var key=p.id+"|"+m.id+"|"+haxe.Json.stringify(pp)+"|"+haxe.Json.stringify(mp);
            if(verifiedPairs.exists(key))continue;
            checked++;var volume=cache.overlap(p.component,pp,m.component,mp);
            if(volume>0.001)throw 'Enclosure collision ${p.id} / ${m.id}: $volume mm3 at X$x Y$y Z$z A$a door$door; machine bounds '+haxe.Json.stringify(cache.box(m.component,mp));
            verifiedPairs.set(key,true);
          }
          samples++;
        }
      }
      // Check both leaves and their suspension against stationary shell/track/shoes.
      var leaves=[for(p in panels)if(p.id=="enclosure/door/leaf"||p.id=="enclosure/door/leftLeaf")p],doorSweepPairs=0;
      if(leaves.length!=2)throw "Expected two opposed door leaves";
      var moving=[for(p in panels)if(StringTools.startsWith(p.id,"enclosure/door/right")||StringTools.startsWith(p.id,"enclosure/door/left")||p.id=="enclosure/door/leaf")p];
      var movingIds=[for(p in moving)p.id];
      for(step in 0...21) {
        state.setJoint("enclosure/door/slide",shell.layout.stroke*step/20);state.forwardKinematics();
        if(Math.abs(state.joint("enclosure/door/leftSlide")+state.joint("enclosure/door/slide"))>1e-8)throw "Door leaves lost synchronization";
        for(prefix in ["right","left"]) {
          var channel=[for(p in moving)if(p.id=="enclosure/door/"+prefix+"LowerChannel")p][0];
          var box=cache.box(channel.component,state.worldPose(channel.id)),engaged=false;
          for(p in panels)if(StringTools.startsWith(p.id,"enclosure/door/guide")) {
            var shoe=cache.box(p.component,state.worldPose(p.id)),centre=(shoe.minX+shoe.maxX)/2;
            if(centre-3>=box.minX&&centre+3<=box.maxX)engaged=true;
          }
          if(!engaged)throw 'Lower guide disengaged from $prefix leaf at sample $step';
        }
        for(leaf in moving) {
          var pose=state.worldPose(leaf.id);
          for(p in panels)if(movingIds.indexOf(p.id)<0) {
            var other=state.worldPose(p.id);
            if(PosedParts.apart(cache.box(leaf.component,pose),cache.box(p.component,other),0))continue;
            doorSweepPairs++;
            if(cache.overlap(leaf.component,pose,p.component,other)>0.001)throw 'Door travel ${leaf.id} interferes with ${p.id} at sample $step';
          }
        }
      }
      state.setJoint("enclosure/door/slide",shell.layout.stroke);state.forwardKinematics();
      var b=shell.layout.input.loading,raw=Part.box(b.maxX-b.minX,b.maxY-b.minY,b.maxZ-b.minZ);
      var corridor=raw.translated(new Vector((b.maxX+b.minX)/2,(b.maxY+b.minY)/2,b.minZ));raw.close();
      for(p in panels){var posed=cache.posed(p.component,state.worldPose(p.id)),common=corridor.intersect(posed),volume=common.volume();common.close();posed.close();if(volume>0.001)throw 'Loading reservation obstructed by ${p.id}';}
      corridor.close();
      for(access in shell.serviceAccess){
        var b=access.bounds,raw=Part.box(b.maxX-b.minX,b.maxY-b.minY,b.maxZ-b.minZ),space=raw.translated(new Vector((b.maxX+b.minX)/2,(b.maxY+b.minY)/2,b.minZ));raw.close();
        for(p in panels)if(p.id!="enclosure/"+access.panelId){
          var solid=cache.posed(p.component,state.worldPose(p.id)),common=space.intersect(solid),volume=common.volume();common.close();solid.close();
          if(volume>0.001)throw 'Service opening ${access.panelId} obstructed by ${p.id}';
        }
        space.close();
      }
      var port=new EnclosureCutPanel(600,800,"steel-2",0,0,0,400,100),geometry=port.geometry();
      var expected=(600*800-Math.PI*2500)*2;
      if(Math.abs(geometry.volume()-expected)>0.01)throw "Mist hole not cut through panel";geometry.close();
      if(Math.abs(port.massProperties().mass-(expected*1e-9*7850))>1e-9)throw "Cut panel mass ignored hole";
      var proxy:machinekit.component.CollisionHullFacet=machinekit.component.CollisionHullFacet.of(port);
      if(proxy==null||proxy.hulls.length!=4)throw "Cut panel proxy filled its opening";
      var rebuilt=MachineAssembly.decode(assembly.encode());
      if(rebuilt.components().length!=parts.length)throw "Enclosure save/reopen lost components";
      cache.close();
      Sys.println('Hobber enclosure passed: $samples machine/door samples, $checked exact near-pair checks, loading corridor, port, mass and save/reopen');
      // Export only after acceptance, using the checked fit rather than sizing against a different model.
      var scene=HobberEnclosurePreview.runtimeScene(assembly,fitted.machine,shell.layout.stroke),root=Sys.getCwd();
      while(!sys.FileSystem.exists(root+"/machinekit/examples/flexspline-hobber/FlexsplineHobber.hx"))root=haxe.io.Path.directory(root);
      var out=root+"/machinekit/examples/flexspline-hobber/build/enclosure";sys.FileSystem.createDirectory(haxe.io.Path.directory(out));sys.FileSystem.createDirectory(out);
      sys.io.File.saveBytes(out+"/hobber.mtrg",materia.project.SceneArtifact.encode(scene));
      var cuts=["part,width_mm,depth_mm,thickness_mm,construction"];
      for(p in tray){var panel:machinekit.enclosure.EnclosurePanel=cast p.component;cuts.push([p.id,Std.string(panel.width),Std.string(panel.height),Std.string(panel.construction.thicknessMm()),panel.construction.id].join(","));}
      sys.io.File.saveContent(out+"/tray-cut-list.csv",cuts.join("\n")+"\n");
      sys.io.File.saveContent(out+"/guard-checks.json",haxe.Json.stringify(guardReport,null,"  "));
      sys.io.File.saveContent(out+"/layout.json",haxe.Json.stringify(fitted.report,null,"  "));
      sys.io.File.saveContent(out+"/checks.json",haxe.Json.stringify({samples:samples,exactNearPairs:checked,doorTravelSamples:21,doorSweepPairs:doorSweepPairs,lowerGuidesEngaged:true,baseSupports:4,traySupports:4,trayPieces:6,trayClamps:12,trayLiftClearanceMm:25,trayConstructionClear:true,trayStackMm:[4,2,13,11],frameSupportContacts:true,traySupportContacts:true,loadingCorridorClear:true,saveReopen:true},null,"  "));
    }catch(error:Dynamic){cache.close();throw error;}
  }
}
