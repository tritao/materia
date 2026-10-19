import machinekit.robotics.CobotArm;
import machinekit.robotics.CobotClass;
import machinekit.robotics.CobotToolLink;
import machinekit.robotics.ToolServiceSpec;
import machinekit.robotics.ToolServiceModule;
import machinekit.robotics.CobotToolServiceMount;
import machinekit.component.MachineComponent;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.AssemblyPreview;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Part;
import cadkit.modeling.Location;
import cadkit.modeling.Vector;
import cadkit.modeling.Plane;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact;

/** Electrical graph and CAD mounting checks, deliberately separate from an unbuilt robot harness. */
class ToolServicePreview {
  static function assert(ok:Bool,message:String):Void {if(!ok) throw message;}
  static function reject(fn:Void->Void,label:String):Void {
    var caught=false;try fn() catch(_:Dynamic) caught=true;assert(caught,'Expected rejection: $label');
  }
  static function posed(part:Part,f:AssemblyFrame):Part {
    var m=AssemblyFrames.toRotationMatrix(f);
    return new Part(new Location(new Plane(new Vector(f.x,f.y,f.z),new Vector(m[0],m[3],m[6]),new Vector(m[2],m[5],m[8]))).apply(part.shape));
  }
  static function noOverlap(a:Part,b:Part,label:String):Void {
    var common=a.shape.common(b.shape);
    var volume=common.subshapeCount(CadKit.ShapeKind.Solid)>0 ? common.volume() : 0;common.close();
    assert(volume<0.001,'$label overlaps by $volume mm3');
  }
  static function moduleScene(module:ToolServiceModule):haxe.io.Bytes {
    var assembly=new MachineAssembly();assembly.addComponent("junction",module);
    assembly.exposePort("powerIn","junction","powerIn");assembly.exposePort("signalIn","junction","signalIn");
    return SceneArtifact.encode(AssemblyPreview.scene(assembly,"dual-gripper-junction-reference"));
  }
  static function wristScene(scene:materia.project.SceneArtifact.SceneArtifactData):haxe.io.Bytes {
    var definition:materia.assembly.AssemblyDefinition=scene.assemblyDefinition;
    var state=new AssemblyState(definition,scene.assemblyState),model=new AssemblyModel("mm");
    var anchor=AssemblyFrames.inverse(state.worldPose("toolFlange"));
    var parts:Array<materia.project.SceneArtifact.SceneArtifactPart> = [];
    for(o in definition.occurrences) {
      if(!StringTools.startsWith(o.id,"toolService") && o.id!="link6" && o.id!="toolFlange" && o.id!="joint6" && o.id!="wristRotor6") continue;
      for(p in scene.parts) if(p.id==o.definition) {
        var socket=o.id=="toolServiceSocket";
        parts.push({id:o.id,name:o.id,red:socket ? 0.18 : p.red,green:socket ? 0.4 : p.green,blue:socket ? 0.65 : p.blue,
          physicalMassKg:0,physicalMassDeclared:true,vertexCount:p.vertexCount,indexCount:p.indexCount,
          vertices:p.vertices,normals:p.normals,indices:p.indices,edgeSegments:p.edgeSegments,edgeIds:p.edgeIds,faceRanges:p.faceRanges});
        model.add(o.id,AssemblyFrames.compose(anchor,state.worldPose(o.id)));model.ground(o.id);
      }
    }
    var d=model.definition("wrist-service-static-reference");for(c in d.definitions) c.collisionHulls=[];
    return SceneArtifact.encode({lengthUnit:"mm",metresPerUnit:0.001,parts:parts,assemblyDefinition:d,
      assemblyState:model.initialState("wrist-service-static-reference").record()});
  }
  static function outputDirectory():String {
    var current=sys.FileSystem.fullPath(Sys.getCwd());
    while(true) {
      var candidate=current+"/machinekit/examples/tool-service";
      if(sys.FileSystem.exists(candidate+"/haxeon.json")) return candidate+"/build";
      if(StringTools.endsWith(current,"/tool-service") && sys.FileSystem.exists(current+"/haxeon.json")) return current+"/build";
      var parent=haxe.io.Path.directory(current);
      if(parent==current || parent=="") throw "Cannot find tool-service example directory";
      current=parent;
    }
  }
  public static function electricalChecks():Void {
    var spec=new ToolServiceSpec(),loads=ToolServiceSpec.pgcPair(),junction=new ToolServiceModule(spec,2);
    assert(Math.abs(junction.checkLoads(24,5,loads)-2.4)<1e-9,"PGC pair current budget");
    reject(()->junction.checkLoads(48,5,loads),"48 V motor supply");
    reject(()->junction.checkLoads(24,2,loads),"insufficient shared source current");
    reject(()->new ToolServiceSpec(24,2,2).checkLoads(24,5,loads),"insufficient shared harness current");
    reject(()->new ToolServiceSpec(24,5,1).checkLoads(24,5,loads),"insufficient branch current");
    var duplicate=ToolServiceSpec.pgcPair();duplicate[1].address=1;
    reject(()->junction.checkLoads(24,5,duplicate),"duplicate Modbus addresses");
    var wrong=ToolServiceSpec.pgcPair();wrong[1].protocol="Digital I/O";
    reject(()->junction.checkLoads(24,5,wrong),"wrong protocol");
    var nan=ToolServiceSpec.pgcPair();nan[1].peakCurrent=Math.NaN;
    reject(()->junction.checkLoads(24,5,nan),"nonfinite load");
    reject(()->new ToolServiceModule(spec).checkLoads(24,5,loads),"outlet count");
    assert(new ToolServiceModule(spec).checkPassThrough(24,5,loads)==2.4,"Wrist outlet must carry the combined junction current");
    var restored:ToolServiceModule=cast ToolServiceModule.recipeType().create(junction.values());
    assert(restored.spec.protocol==spec.protocol && restored.spec.totalCurrent==5 && restored.spec.branchCurrent==2 && restored.spec.pins==spec.pins && restored.spec.connectorKey==spec.connectorKey && restored.outlets==2 && !restored.spec.pinoutVerified,"Saved electrical contract changed");
    for(rating in [{voltage:24.0,current:5.0,harness:5.0,ok:true},{voltage:48.0,current:5.0,harness:5.0,ok:false},{voltage:24.0,current:2.0,harness:5.0,ok:false},{voltage:24.0,current:5.0,harness:2.0,ok:false}]) {
      var circuit=new MachineAssembly(),wrist=new ToolServiceModule(new ToolServiceSpec(24,rating.harness,2));
      circuit.addComponent("supply",new machinekit.motion.PowerSupply(new machinekit.power.DcVoltage(rating.voltage),new machinekit.power.DcCurrent(rating.current)));
      circuit.addComponent("wrist",wrist);circuit.addComponent("junction",junction);
      circuit.connectPorts("supply","supply","power1","wrist","powerIn");
      circuit.connectPorts("power","wrist","power1","junction","powerIn");
      circuit.connectPorts("signal","wrist","signal1","junction","signalIn");
      circuit.exposePort("serialMaster","wrist","signalIn");
      for(i in 0...2) {
        var id=i==0 ? "gripper-a" : "gripper-b",channel=i+1;
        circuit.addComponent(id,new GripperServiceEndpoint(spec,loads[i]));
        circuit.connectPorts(id+"-power","junction",'power$channel',id,"power");
        circuit.connectPorts(id+"-signal","junction",'signal$channel',id,"signal");
        assert(circuit.upstreamChain(id,"power").length>=5,"Missing wired power route");
        assert(circuit.upstreamChain(id,"signal").length>=5,"Missing wired data route");
      }
      circuit.validate();
      if(rating.ok) assert(junction.checkWiredLoads(circuit,"junction",loads)==2.4,"Rated wired circuit failed");
      else reject(()->junction.checkWiredLoads(circuit,"junction",loads),"actual wired source rating");
    }
    var boundary=new MachineAssembly();boundary.addComponent("junction",junction);
    boundary.exposePort("powerIn","junction","powerIn");boundary.exposePort("signalIn","junction","signalIn");
    reject(()->junction.checkWiredLoads(boundary,"junction",loads),"unrated external power boundary");
    Sys.println("Tool service electrical contracts, wired-source rejection, graph tracing and recipe checks passed");
  }
  public static function main():Void {
    var directory=outputDirectory();sys.FileSystem.createDirectory(directory);Sys.println("Service output: "+directory);
    electricalChecks();
    var spec=new ToolServiceSpec(),loads=ToolServiceSpec.pgcPair(),junction=new ToolServiceModule(spec,2);
    var restored:ToolServiceModule=cast ToolServiceModule.recipeType().create(junction.values());
    var j=junction.geometry(),r=restored.geometry();
    assert(j.valid() && j.solidCount()==1 && Math.abs(j.volume()-r.volume())<1e-6,"Junction CAD or recipe round trip");r.close();j.exportStep(directory+"/junction.step");j.close();
    sys.io.File.saveBytes(directory+"/junction.mtrg",moduleScene(junction));
    var checks:Array<{cls:String,minHousingGapMm:Float,serviceMassKg:Float}> = [];
    for(cls in [Reach500,Reach850,Reach900,Reach1300]) {
      Sys.println('Checking wrist service: $cls');Sys.stdout().flush();
      var arm=new CobotArm(cls),socket=arm.installToolService(spec);
      reject(()->arm.installToolService(spec),"duplicate installation");
      assert(arm.hasPort("toolPower") && arm.hasPort("toolServicePowerIn"),"Missing arm electrical boundaries");
      var circuit=new MachineAssembly();circuit.include("arm",arm);
      circuit.addComponent("junction",junction,AssemblyFrames.translation(0,-200,0));
      circuit.connectPorts("tool-power","arm","toolPower","junction","powerIn");
      circuit.connectPorts("tool-data","arm","toolSignal","junction","signalIn");
      circuit.exposePort("controllerPower","arm/toolServiceSocket","powerIn");
      circuit.exposePort("controllerSerial","arm/toolServiceSocket","signalIn");
      for(i in 0...2) {
        var id=i==0 ? "gripper-a" : "gripper-b",channel=i+1;
        circuit.addComponent(id,new GripperServiceEndpoint(spec,loads[i]));
        circuit.connectPorts(id+"-power","junction",'power$channel',id,"power");
        circuit.connectPorts(id+"-data","junction",'signal$channel',id,"signal");
      }
      circuit.validate();
      for(id in ["gripper-a","gripper-b"]) for(port in ["power","signal"])
        assert(circuit.upstreamChain(id,port).length>=5,'Disconnected tool service $id/$port');
      var upstream=circuit.upstreamChain("junction","powerIn");
      assert(upstream.length>=3,"Junction power does not trace to arm boundary");
      var model=new AssemblyModel("mm");arm.addTo(model,"");var state=model.initialState("service-check");
      var all:Array<{id:String,part:Part}> = [],mass=0.0;
      for(entry in arm.components()) if(StringTools.startsWith(entry.id,"toolService")) {
        var local=entry.component.geometry();assert(local.valid() && local.solidCount()==1,'Invalid service CAD ${entry.id}');
        var world=posed(local,state.worldPose(entry.id));local.close();all.push({id:entry.id,part:world});
        mass+=entry.component.massProperties().mass;
      }
      for(i in 0...all.length) for(k in i+1...all.length) noOverlap(all[i].part,all[k].part,all[i].id+" / "+all[k].id);
      var minGap=1e30;
      for(id in ["link6","toolFlange","joint6","wristRotor6"]) {
        var component:MachineComponent=null;for(entry in arm.components()) if(entry.id==id) component=entry.component;
        assert(component!=null,'Missing adjacent robot member $id');
        var local=component.geometry(),world=posed(local,state.worldPose(id));local.close();
        for(p in all) {noOverlap(p.part,world,p.id+" / "+id);minGap=Math.min(minGap,p.part.shape.distance(world.shape));}
        world.close();
      }
      for(p in all) {p.part.exportStep(directory+"/"+Std.string(cls)+"-"+p.id+".step");p.part.close();}
      checks.push({cls:Std.string(cls),minHousingGapMm:minGap,serviceMassKg:mass});
      if(cls==Reach1300) {
        var scene:materia.project.SceneArtifact.SceneArtifactData=AssemblyPreview.scene(arm,"cobot-tool-service-prototype");
        var definition:materia.assembly.AssemblyDefinition=scene.assemblyDefinition;
        var ss=new AssemblyState(definition);var ready=arm.ready();for(i in 0...6) ss.setJoint('j${i+1}',ready[i]);scene.assemblyState=ss.record();
        // Service study has no simulated harness or commissioned robot motion.
        for(d in definition.definitions) d.collisionHulls=[];
        sys.io.File.saveBytes(directory+"/cobot-service.mtrg",SceneArtifact.encode(scene));
        sys.io.File.saveBytes(directory+"/wrist-service.mtrg",wristScene(scene));
      }
    }
    sys.io.File.saveContent(directory+"/report.json",haxe.Json.stringify({status:"provisional interface and mounting study; not commissioned",
      supplyVoltageV:24,sharedDesignCurrentA:5,branchDesignCurrentA:2,simultaneousPeakCurrentA:2.4,
      protocol:spec.protocol,loads:loads,pinoutVerified:false,connectorEnvelope:"Unselected 8-contact planning interface; no vendor pin assignments",
      source:"https://en.dh-robotics.com/product/pgc",mountChecks:checks,
      pending:["Exact DH cable option, mating connector and verified pinout","Branch protection, cable sizing, shield/reference and voltage drop",
        "RS485 topology, termination, bias and configured distinct addresses","Clamp liner, preload, fasteners and terminal shell strength",
        "Controller supply and serial master; physical harness across joints and wrist motion limits",
        "Junction installation on actual dual adapter and gripper-lead rerouting; previous return cable clashes with arm"]},null,"  "));
    Sys.println("Tool service: circuit/rejection checks, saved module and four-class local CAD mounting checks passed");
  }
}

/** Logical load endpoints only; actual DH geometry remains in the imported gripper example. */
private class GripperServiceEndpoint extends MachineComponent {
  public function new(spec:ToolServiceSpec,load:machinekit.robotics.ToolServiceSpec.ToolServiceLoad) {
    super("SERVICE-ENDPOINT-"+load.id,"Logical electrical endpoint "+load.id,"aluminium 6061");
    addPort({name:"power",kind:ElectricalPower,role:Consumer,iface:Plug(spec.key(),spec.pins),required:true});
    addPort({name:"signal",kind:Signal,role:Consumer,iface:Plug(spec.key(),spec.pins),required:true});
  }
}
