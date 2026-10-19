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
    var parts=assembly.components(),machine=[for(p in parts)if(StringTools.startsWith(p.id,"machine/"))p],panels=[for(p in parts)if(StringTools.startsWith(p.id,"enclosure/"))p];
    var cache=new PosedParts(),checked=0,samples=0;
    try {
      var source=new FlexsplineHobber(),ranges=[for(t in source.travel)[t.lower,t.upper]];
      for(x in ranges[0])for(y in ranges[1])for(z in ranges[2])for(a in [-Math.PI/12,0.0,Math.PI/12]) {
        state.setJoint("machine/X",x);state.setJoint("machine/Y",y);state.setJoint("machine/Z",z);state.setJoint("machine/"+FlexsplineHobber.A,a);
        for(door in [0.0,shell.layout.stroke]) {
          state.setJoint("enclosure/door/slide",door);state.forwardKinematics();
          for(p in panels)for(m in machine) {
            var pp=state.worldPose(p.id),mp=state.worldPose(m.id);
            if(PosedParts.apart(cache.box(p.component,pp),cache.box(m.component,mp),0))continue;
            checked++;var volume=cache.overlap(p.component,pp,m.component,mp);
            if(volume>0.001)throw 'Enclosure collision ${p.id} / ${m.id}: $volume mm3 at X$x Y$y Z$z A$a door$door';
          }
          samples++;
        }
      }
      // Check the moving leaf against its own stationary shell throughout the stroke.
      var leaf=[for(p in panels)if(p.id=="enclosure/door/leaf")p][0],doorSweepPairs=0;
      for(step in 0...21) {
        state.setJoint("enclosure/door/slide",shell.layout.stroke*step/20);state.forwardKinematics();
        var pose=state.worldPose(leaf.id);
        for(p in panels)if(p.id!=leaf.id) {
          var other=state.worldPose(p.id);
          if(PosedParts.apart(cache.box(leaf.component,pose),cache.box(p.component,other),0))continue;
          doorSweepPairs++;
          if(cache.overlap(leaf.component,pose,p.component,other)>0.001)throw 'Door travel interferes with ${p.id} at sample $step';
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
      var scene=HobberEnclosurePreview.defaultView(machinekit.assembly.AssemblyPreview.scene(assembly,"hobber-enclosed")),root=Sys.getCwd();
      while(!sys.FileSystem.exists(root+"/machinekit/examples/flexspline-hobber/FlexsplineHobber.hx"))root=haxe.io.Path.directory(root);
      var out=root+"/machinekit/examples/flexspline-hobber/build/enclosure";sys.FileSystem.createDirectory(haxe.io.Path.directory(out));sys.FileSystem.createDirectory(out);
      sys.io.File.saveBytes(out+"/hobber.mtrg",materia.project.SceneArtifact.encode(scene));
      sys.io.File.saveContent(out+"/guard-checks.json",haxe.Json.stringify(guardReport,null,"  "));
      sys.io.File.saveContent(out+"/layout.json",haxe.Json.stringify(fitted.report,null,"  "));
      sys.io.File.saveContent(out+"/checks.json",haxe.Json.stringify({samples:samples,exactNearPairs:checked,doorTravelSamples:21,doorSweepPairs:doorSweepPairs,loadingCorridorClear:true,saveReopen:true},null,"  "));
    }catch(error:Dynamic){cache.close();throw error;}
  }
}
