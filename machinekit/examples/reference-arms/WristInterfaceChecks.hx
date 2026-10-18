import ReferenceWristInterfaces.WristProfile;
import ReferenceWristInterfaces.WristUse;
import materia.project.SceneArtifact;
import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyFrames;

class WristInterfaceChecks {
  static function check(ok:Bool,message:String):Void {if(!ok) throw message;}
  static function reject(fn:Void->Void,label:String):Void {
    var caught=false;try fn() catch(_:Dynamic) caught=true;check(caught,"Expected rejection: "+label);
  }
  static function profile(profiles:Array<WristProfile>,id:String):WristProfile {
    for(p in profiles) if(p.id==id) return p;throw "Missing profile "+id;
  }
  static function load(power:String = "tool",signal:String = "tool"):WristUse return {
    powerPort:power,signalPort:signal,voltage:24,powerMode:"single-pin",signalMode:"rs485",
    continuousA:0.4,peakA:1.2,peakDurationS:0.75,peakDutyFraction:0.1,averageWindowA:0.48,
    capacitanceUf:100,requestedSignals:[]
  };
  public static function main():Void {
    var root=ReferenceAssetLocation.DIRECTORY;
    var ur=new ReferenceRobot(root+"/ur5e"),fr=new ReferenceRobot(root+"/fairino5_v6");
    check(ur.wristInterfaces.length==2 && fr.wristInterfaces.length==1,"Prepared manufacturer interface inventory");
    var male=profile(ur.wristInterfaces,"ur5e-m8-male"),female=profile(ur.wristInterfaces,"ur5e-m8-female"),fair=fr.wristInterfaces[0];
    check(ReferenceWristInterfaces.ur5eForSerial(ur.wristInterfaces,"20245501102").id==male.id,"UR5e last pre-cutover serial");
    check(ReferenceWristInterfaces.ur5eForSerial(ur.wristInterfaces,"20245501103").id==female.id,"UR5e first female serial");
    reject(()->ReferenceWristInterfaces.ur5eForSerial(ur.wristInterfaces,"unknown"),"unknown robot serial");
    check(ReferenceWristInterfaces.port(male,"tool").sex=="male" && ReferenceWristInterfaces.port(female,"tool").sex=="female","UR connector revision sex");
    check(ReferenceWristInterfaces.pin(male,"tool","RS485+")==1 && ReferenceWristInterfaces.pin(female,"tool","RS485+")==2,"UR revision-specific RS485 pin numbering");
    check(ReferenceWristInterfaces.pin(male,"tool","TI0")==4 && ReferenceWristInterfaces.pin(female,"tool","TI0")==6,"UR digital input remapping");
    check(ReferenceWristInterfaces.pin(male,"tool","POWER")==5 && ReferenceWristInterfaces.pin(female,"tool","GND")==8,"UR dedicated power pins");
    var expectedMale=["RS485+","RS485-","TI1","TI0","POWER","TO1","TO0","GND"];
    var expectedFemale=["RS485-","RS485+","TO0","TO1","POWER","TI0","TI1","GND"];
    var expectedIo=["AI0","AO0","DI0","DI1","POWER","DO0","DO1","GND"];
    var expectedSerial=["RS485-A","RS485-B","POWER","UNASSIGNED","GND"];
    for(i in 0...8) {
      check(ReferenceWristInterfaces.pin(male,"tool",expectedMale[i])==i+1,"Full UR male diagram fixture");
      check(ReferenceWristInterfaces.pin(female,"tool",expectedFemale[i])==i+1,"Full UR female diagram fixture");
      check(ReferenceWristInterfaces.pin(fair,"io",expectedIo[i])==i+1,"Full FAIRINO I/O diagram fixture");
    }
    for(i in 0...5) check(ReferenceWristInterfaces.pin(fair,"rs485",expectedSerial[i])==i+1,"Full FAIRINO serial diagram fixture");
    check(fair.ports.length==2 && ReferenceWristInterfaces.port(fair,"io").contactCount==8 && ReferenceWristInterfaces.port(fair,"rs485").contactCount==5,"FAIRINO has separate I/O and serial connectors");
    check(ReferenceWristInterfaces.pin(fair,"io","AI0")==1 && ReferenceWristInterfaces.pin(fair,"io","DO1")==7,"FAIRINO I/O pinout");
    check(ReferenceWristInterfaces.pin(fair,"rs485","RS485-A")==1 && ReferenceWristInterfaces.pin(fair,"rs485","POWER")==3,"FAIRINO serial pinout");
    reject(()->ReferenceWristInterfaces.pin(fair,"io","RS485-A"),"RS485 on FAIRINO I/O connector");
    reject(()->ReferenceWristInterfaces.checkUse(male,"",load()),"implicit power rating");
    for(p in [male,female]) for(r in p.ratings) {
      ReferenceWristInterfaces.checkUse(p,r.id,load());
      var over=load();over.continuousA=0.8;over.peakA=2.4;over.averageWindowA=0.96;
      reject(()->ReferenceWristInterfaces.checkUse(p,r.id,over),"two simultaneous PGC peaks");
      var duty=load();duty.peakA=2;duty.peakDurationS=1.01;duty.averageWindowA=0.56;
      reject(()->ReferenceWristInterfaces.checkUse(p,r.id,duty),"UR peak duration");
      var average=load();average.peakA=2;average.peakDutyFraction=0.11;average.averageWindowA=0.576;
      reject(()->ReferenceWristInterfaces.checkUse(p,r.id,average),"UR peak duty");
      var analog=load();analog.requestedSignals=["AI2"];
      reject(()->ReferenceWristInterfaces.checkUse(p,r.id,analog),"analog/RS485 multiplexing");
      var dual=load();dual.powerMode="dual-pin";dual.requestedSignals=["TO0"];
      reject(()->ReferenceWristInterfaces.checkUse(p,r.id,dual),"dual-pin power consumes outputs");
      var cap=load();cap.capacitanceUf=8001;
      reject(()->ReferenceWristInterfaces.checkUse(p,r.id,cap),"capacitive load");
      var nan=load();nan.peakA=Math.NaN;
      reject(()->ReferenceWristInterfaces.checkUse(p,r.id,nan),"nonfinite load");
    }
    var mid=load();mid.continuousA=0.8;mid.peakA=0.8;mid.averageWindowA=0.8;
    ReferenceWristInterfaces.checkUse(male,"ur-developer-table-captured-2026-10-18",mid);
    reject(()->ReferenceWristInterfaces.checkUse(male,"ur5e-sw5.25-manual",mid),"documented current-table conflict is retained");
    ReferenceWristInterfaces.checkUse(fair,fair.ratings[0].id,load("io","rs485"));
    reject(()->ReferenceWristInterfaces.checkUse(fair,fair.ratings[0].id,load("io","io")),"FAIRINO serial connector selection");
    var pair=load("io","rs485");pair.continuousA=0.8;pair.peakA=2.4;pair.averageWindowA=0.96;
    reject(()->ReferenceWristInterfaces.checkUse(fair,fair.ratings[0].id,pair),"FAIRINO shared end-board budget");
    var reserved=load("io","rs485");reserved.requestedSignals=["UNASSIGNED"];
    reject(()->ReferenceWristInterfaces.checkUse(fair,fair.ratings[0].id,reserved),"unused serial contact");
    var understated=load();understated.continuousA=0.5;understated.peakA=2;understated.peakDurationS=1;
    understated.peakDutyFraction=0.01;understated.averageWindowA=0.515;
    reject(()->ReferenceWristInterfaces.checkUse(male,"ur5e-sw5.25-manual",understated),"understated ten-second burst average");
    for(id in ["rs485","io"]) {
      var binding=ReferenceWristInterfaces.requireObservedGeometry(fair,id);
      var source=binding.insertFrameStepMm, offset=binding.stepMetresToParent, observed=binding.observedFrame;
      check(binding.parentLink=="wrist3_link" && binding.parentUrdfSha256==fr.manifest.urdfSha256,"Observed geometry parent provenance");
      check(Math.abs(source.x*0.001+offset.x-observed.x)<1e-12 && Math.abs(source.y*0.001+offset.y-observed.y)<1e-12 && Math.abs(source.z*0.001+offset.z-observed.z)<1e-12,"STEP millimetres to ROS metres mapping");
      check(Math.abs(observed.z-0.080)<1e-6,"Observed insert height");
      check(Math.abs((id=="rs485" ? observed.y : observed.x)+0.038)<1e-6,"Observed insert mouth position");
      var rotation=AssemblyFrames.toRotationMatrix(observed);
      check(Math.abs(rotation[2]-(id=="io" ? -1 : 0))<1e-9 && Math.abs(rotation[5]-(id=="rs485" ? -1 : 0))<1e-9,"Observed outward normal");
      var encoded=haxeon.wire.JsonWire.encode(binding);
      var restored:ReferenceWristInterfaces.WristGeometryBinding=haxeon.wire.JsonWire.decode(encoded);
      check(restored.insertFaceIndex==binding.insertFaceIndex && restored.sourceStepSha256==binding.sourceStepSha256,"Typed observation round trip");
    }
    reject(()->ReferenceWristInterfaces.requireObservedGeometry(male,"tool"),"UR STEP not available");
    check(fair.cables.length==3 && male.cables.length==0 && female.cables.length==0,"Cable evidence inventory");
    var cable=fair.cables[0];
    check(cable.exactPartNumber==null && cable.minBendRadiusMm==null && cable.seatingOffsetMm==null,"Catalogue code must not invent cable dimensions");
    check(ReferenceWristInterfaces.cableGeometryGaps(fair,cable.id).length==9,"Unresolved cable evidence remains explicit");
    reject(()->ReferenceWristInterfaces.requireCableGeometry(fair,cable.id),"unqualified cable CAD placement");
    reject(()->ReferenceWristInterfaces.requireCableGeometry(fair,"unknown"),"unknown cable candidate");
    check(ReferenceWristInterfaces.cableGeometryGaps(fair,"binder-7734290000200050500").length==4,"Commercial CAD must preserve unqualified mating and fixed lead");
    reject(()->ReferenceWristInterfaces.requireCableGeometry(fair,"binder-7734290000200050500"),"Candidate CAD is not an installed cable");
    var selection:ReferenceWristInterfaces.WristLayoutSelection=fair.layoutSelection;
    check(selection.cableId=="rockwell-889d-e5ac-right-angle" && selection.connectorExitDeg==90 && selection.preferredStudyClockDeg==180,"Preferred wrist layout uses right-angle connector");
    check(ReferenceWristInterfaces.cableGeometryGaps(fair,selection.cableId).length==4,"Right-angle geometry retains unresolved mating evidence");
    reject(()->ReferenceWristInterfaces.requireCableGeometry(fair,selection.cableId),"Layout preference does not qualify installation");
    var savedCable:ReferenceWristInterfaces.WristCableEvidence=haxeon.wire.JsonWire.decode(haxeon.wire.JsonWire.encode(cable));
    check(savedCable.selector==cable.selector && savedCable.keyClockingRad==null,"Cable evidence typed round trip preserves unknowns");
    var output=root+"/wrist-interface-review";sys.FileSystem.createDirectory(output);
    for(robot in [ur,fr]) {
      var definition:materia.assembly.AssemblyDefinition=robot.scene.assemblyDefinition;
      var state=new AssemblyState(definition);
      var moving=[for(j in robot.manifest.joints) if(j.type!="fixed") j];
      var frames=0;for(d in definition.definitions) if(d.robotFlangeConnector!=null) frames++;
      check(frames==1,"Wrist interface facts must preserve the physical flange");
      // Adding electrical descriptions must not change any imported link transform.
      for(probe in robot.manifest.probes) {
        for(i in 0...moving.length) state.setJoint(moving[i].id,probe.q[i]);
        for(expected in probe.links) {
          var actual=robot.linkPose(state,expected.id);
          var a=AssemblyFrames.toRotationMatrix(actual),b=AssemblyFrames.toRotationMatrix(expected.pose);
          check(Math.abs(actual.x-expected.pose.x)<1e-8 && Math.abs(actual.y-expected.pose.y)<1e-8 && Math.abs(actual.z-expected.pose.z)<1e-8,"Wrist metadata changed source FK");
          for(k in 0...9) check(Math.abs(a[k]-b[k])<1e-8,"Wrist metadata changed source orientation");
        }
      }
      for(p in robot.wristInterfaces) for(port in p.ports)
        reject(()->ReferenceWristInterfaces.requireMatingFrame(p,port.id),"invented manufacturer socket placement");
      reject(()->ReferenceWristInterfaces.load(robot.directory,"incorrect",robot.manifest.id),"changed catalog provenance");
      var bytes=SceneArtifact.encode(robot.scene);
      var restored:materia.project.SceneArtifact.SceneArtifactData=SceneArtifact.decode(bytes);
      var savedDiagnostics:Array<String> = restored.recipeDiagnostics,originalDiagnostics:Array<String> = robot.scene.recipeDiagnostics;
      check(savedDiagnostics.length==originalDiagnostics.length,"Saved scene lost wrist qualification diagnostics");
      sys.io.File.saveBytes(output+"/"+robot.manifest.id+".mtrg",bytes);
      sys.io.File.saveContent(output+"/"+robot.manifest.id+".wrist-interfaces.json",haxe.Json.stringify({
        model:robot.manifest.id,catalogSha256:robot.manifest.wristInterfaceCatalogSha256,profiles:robot.wristInterfaces,
        selection:"No hardware revision/power-rating edition selected; connector mating poses not qualified"},null,"  "));
    }
    sys.io.File.saveContent(output+"/report.json",haxe.Json.stringify({status:"documented electrical profiles checked; physical socket pose/key orientation unqualified",
      models:["ur5e","fairino5_v6"],profiles:3,pinoutAndPowerChecks:true,sourceFkRetained:true,geometryChanged:false,
      custom5ARatingApplied:false,qualifiedSocketMatingFrames:0},null,"  "));
    Sys.println("UR5e/FR5 wrist interfaces: pinouts, source-specific power limits, multiplexing, provenance, save/load and unchanged source FK passed");
  }
}
