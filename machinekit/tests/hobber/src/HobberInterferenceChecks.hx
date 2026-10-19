import machinekit.assembly.PosedParts;
import cadkit.modeling.AssemblyModel;
import gearkit.GearSpec;

/** Exact solids, including fixed neighbours and contacts suppressed in the interactive scene. */
class HobberInterferenceChecks {
  public static function run():Void {
    var output=Sys.getEnv("HOBBER_INTERFERENCE_EVIDENCE");if(output==null)throw "HOBBER_INTERFERENCE_EVIDENCE required";
    var railSupport=Sys.args().indexOf("rail-support")>=0;
    var castingOnly=Sys.args().indexOf("bed-casting")>=0;
    var reference=Sys.args().indexOf("reference")>=0,sweep=Sys.args().indexOf("sweep")>=0;
    var machine=reference?new FlexsplineHobber():new FlexsplineHobber(new GearSpec(1,40,4,20*Math.PI/180),4);
    var model=new AssemblyModel("mm");machine.addTo(model,"");var state=model.initialState();state.forwardKinematics();
    var members=machine.components(),definition=machine.definition(),cache=new PosedParts();
    var relativeVolumes:Map<String,Float> = [];
    function changed(id:String):Bool return id=="bed/casting"||id=="bed/railLeft"||id=="bed/railRight"||id=="Xbridge"||StringTools.startsWith(id,"railX")||StringTools.startsWith(id,"blockX");
    var groups:Map<String,String> = [],rigidVolumes:Map<String,Float> = [];
    for(m in members)groups.set(m.id,m.id);
    for(j in definition.joints)if(j.type==materia.assembly.AssemblyDefinition.AssemblyJointType.Fixed){
      var from=groups.get(j.child),to=groups.get(j.parent);
      for(m in members)if(groups.get(m.id)==from)groups.set(m.id,to);
    }
    var overlaps:Array<Dynamic> =[],checked=0,began=Sys.time(),completed=0,unexpected=0;
    var samples=[{x:machine.travel[0].initial,y:0.0,z:0.0,a:0.0}];
    var setup=FlexsplineHobberPreview.inspectionState(machine);
    samples.push({x:setup.joint("X"),y:setup.joint("Y"),z:setup.joint("Z"),a:setup.joint(FlexsplineHobber.A)});
    var job=FlexsplineHobberCam.job(machine);
    for(pass in job.passes)for(z in [pass.zStart,pass.zEnd])samples.push({x:pass.centreDistance,y:pass.shift,z:z,a:-machine.hob.design.leadAngle});
    if(sweep)for(x in [machine.travel[0].lower,(machine.travel[0].lower+machine.travel[0].upper)/2,machine.travel[0].upper])
      for(y in [-10.0,0,10])for(z in [-10.0,90,190])for(a in [-Math.PI/12,0.0,Math.PI/12])samples.push({x:x,y:y,z:z,a:a});
    function save():Void sys.io.File.saveContent(output+"/overlaps.json",haxe.Json.stringify({reference:reference,
      scope:railSupport?"changed bed casting, X rail seats, rails, blocks and widened carriage against all parts at full travel grid; unchanged pairs retain prior audit evidence":castingOnly?"changed bed casting against every other part at the full travel grid; unchanged pairs retain prior audit evidence":sweep?"conservative hob envelope; exact CAD common volumes on 3x3x3x3 travel grid plus initial, CAM inspection and pass endpoints; sampled clearance, not continuous swept-volume proof":"exact CAD common volume at initial, CAM inspection and pass endpoints",
      samples:samples,completed:completed,members:members.length,checked:checked,thresholdMm3:0.001,
      unexpected:unexpected,seconds:Sys.time()-began,overlaps:overlaps}));
    try {
      for(index in 0...samples.length){
        var s=samples[index];state.setJoint("X",s.x);state.setJoint("Y",s.y);state.setJoint("Z",s.z);state.setJoint(FlexsplineHobber.A,s.a);state.forwardKinematics();
        var poses=[for(m in members)state.worldPose(m.id)],boxes=[for(i in 0...members.length)cache.box(members[i].component,poses[i])];
        for(a in 0...members.length)for(b in a+1...members.length){
          if(railSupport&&!changed(members[a].id)&&!changed(members[b].id))continue;
          if(castingOnly&&members[a].id!="bed/casting"&&members[b].id!="bed/casting")continue;
          if(PosedParts.apart(boxes[a],boxes[b],0))continue;
          var first=members[a],second=members[b];checked++;
          var key=first.id+":"+second.id,rigid=groups.get(first.id)==groups.get(second.id);
          // Identical relative poses reuse the exact solid result; no rounding or tolerance relaxation.
          var relative=materia.assembly.AssemblyFrames.compose(materia.assembly.AssemblyFrames.inverse(poses[a]),poses[b]);
          var relativeKey=key+":"+haxe.Json.stringify(relative);
          var prior=rigid?rigidVolumes.get(key):relativeVolumes.get(relativeKey);
          var volume:Float=prior==null?cache.overlap(first.component,poses[a],second.component,poses[b]):prior;
          if(prior==null){if(rigid)rigidVolumes.set(key,volume);else relativeVolumes.set(relativeKey,volume);}
          if(volume>0.001){
            var cutting=(first.id=="blank"&&second.id=="hob")||(first.id=="hob"&&second.id=="blank");
            if(!cutting)unexpected++;
            var row={sample:index,a:first.id,b:second.id,volumeMm3:volume,cutting:cutting,
              directJoints:[for(j in definition.joints)if((j.parent==first.id&&j.child==second.id)||(j.child==first.id&&j.parent==second.id))j.id]};
            overlaps.push(row);if(!cutting)Sys.println("HOBBER_SOLID_OVERLAP "+haxe.Json.stringify(row));
          }
        }
        completed++;save();Sys.println('HOBBER_INTERFERENCE_PROGRESS samples=$completed/${samples.length} unexpected=$unexpected seconds=${Sys.time()-began}');Sys.stdout().flush();
      }
      save();Sys.println('HOBBER_INTERFERENCE_COMPLETE checked=$checked unexpected=$unexpected seconds=${Sys.time()-began}');
    }catch(error:Dynamic){cache.close();throw error;}
    cache.close();if(unexpected>0)throw 'hobber has $unexpected unintended solid intersections';
  }
}
