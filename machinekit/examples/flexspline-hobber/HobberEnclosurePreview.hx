import machinekit.assembly.MachineAssembly;
import machinekit.assembly.PosedParts;
import machinekit.assembly.PosedParts.PartBox;
import machinekit.assembly.AssemblyPreview;
import machinekit.enclosure.EnclosureLayout;
import machinekit.enclosure.GeneratedEnclosure;
import machinekit.enclosure.EnclosureRules;
import materia.assembly.AssemblyFrames as F;
import materia.project.SceneArtifact;

/** One enclosed machine and runtime; scenarios and view controls never replace the assembly. */
class HobberEnclosurePreview {
  static function union(a:PartBox,b:PartBox):PartBox return {minX:Math.min(a.minX,b.minX),minY:Math.min(a.minY,b.minY),minZ:Math.min(a.minZ,b.minZ),
    maxX:Math.max(a.maxX,b.maxX),maxY:Math.max(a.maxY,b.maxY),maxZ:Math.max(a.maxZ,b.maxZ)};
  static function grow(b:PartBox,p:Float):PartBox return {minX:b.minX-p,minY:b.minY-p,minZ:b.minZ-p,maxX:b.maxX+p,maxY:b.maxY+p,maxZ:b.maxZ+p};
  static function radius(b:PartBox,x:Float,y:Float,z:Float):Float {
    var dx=Math.max(Math.abs(b.minX-x),Math.abs(b.maxX-x)),dy=Math.max(Math.abs(b.minY-y),Math.abs(b.maxY-y)),dz=Math.max(Math.abs(b.minZ-z),Math.abs(b.maxZ-z));
    return Math.sqrt(dx*dx+dy*dy+dz*dz);
  }
  public static function external(id:String):Bool return id=="electricalPanel"||id=="powerSupply"||StringTools.endsWith(id,"-driver");
  public static function fitted(machine:FlexsplineHobber):{layout:EnclosureLayout,samples:Int,paddingMm:Float,excluded:Array<String>} {
    var model=new cadkit.modeling.AssemblyModel("mm");machine.addTo(model,"");var state=model.initialState(),definition=state.definition;
    var parents:Map<String,String> = [],rotors:Map<String,String> = [];
    for(j in definition.joints){parents.set(j.child,j.parent);if(j.id==FlexsplineHobber.B||j.id==FlexsplineHobber.C)rotors.set(j.child,j.id);}
    function rotor(id:String):Null<String> {
      var current=id;while(current!=null){if(rotors.exists(current))return rotors.get(current);current=parents.get(current);}return null;
    }
    var cache=new PosedParts(),members=machine.components(),swept:Null<PartBox> = null,plinth:Null<PartBox> = null,samples=0,radiusBound=0.0;
    try {
      var ranges=[for(t in machine.travel)[t.lower,t.upper]];
      var aj=[for(j in definition.joints)if(j.id==FlexsplineHobber.A)j][0];
      if(aj.limits.lower==null||aj.limits.upper==null)throw "Enclosure needs bounded swivel travel";
      var lower:Float=aj.limits.lower,upper:Float=aj.limits.upper,steps=Std.int(Math.ceil((upper-lower)/(Math.PI/180)));
      if(steps<1)throw "Enclosure needs nonzero swivel travel";
      var step=(upper-lower)/steps;
      for(x in ranges[0])for(y in ranges[1])for(z in ranges[2])for(a in 0...steps+1) {
        state.setJoint("X",x);state.setJoint("Y",y);state.setJoint("Z",z);state.setJoint(FlexsplineHobber.A,lower+a*step);state.forwardKinematics();
        var pivot=state.worldConnector(aj.parent,aj.parentConnector);
        for(m in members)if(!external(m.id)) {
          var box=cache.box(m.component,state.worldPose(m.id)),r=rotor(m.id);
          if(r!=null){var j=[for(j in definition.joints)if(j.id==r)j][0],p=state.worldConnector(j.parent,j.parentConnector),rr=radius(box,p.x,p.y,p.z);
            box={minX:p.x-rr,minY:p.y-rr,minZ:p.z-rr,maxX:p.x+rr,maxY:p.y+rr,maxZ:p.z+rr};}
          radiusBound=Math.max(radiusBound,radius(box,pivot.x,pivot.y,pivot.z));
          swept=swept==null?box:union(swept,box);
          if(m.id=="bed/casting")plinth=box;
        }
        samples++;
      }
      if(swept==null||plinth==null)throw "Missing machine/plinth envelope";
      var padding=radiusBound*step/2;
      swept=grow(swept,padding);
      // OD140 side-jaw reservation matches HobberLoadingChecks; the robot itself is not modeled.
      var r=Math.max(70,machine.spec.tipRadius()),origin=machine.workOrigin;
      var loading:PartBox={minX:origin.x-r-25,maxX:origin.x+r+25,
        minY:Math.min(origin.y-machinekit.hobbing.HobberXLayout.FRONT_APPROACH-r-80,swept.minY-225),maxY:origin.y+15,
        minZ:origin.z-60,maxZ:origin.z+80+machinekit.hobbing.HobberXLayout.FRONT_LIFT};
      // Conservative plinth cutout, with a 10 mm gasket/trim reservation around the casting footprint.
      var opening:PartBox={minX:plinth.minX-10,minY:plinth.minY-10,minZ:plinth.minZ,maxX:plinth.maxX+10,maxY:plinth.maxY+10,maxZ:plinth.maxZ};
      var floor=plinth.maxZ;
      var result={layout:new EnclosureLayout({swept:swept,loading:loading,plinth:opening,floorZ:floor,margin:75,openingWidth:400,opposedDoors:true}),
        samples:samples,paddingMm:padding,excluded:[for(m in members)if(external(m.id))m.id]};
      cache.close();return result;
    }catch(error:Dynamic){cache.close();throw error;}
  }
  public static function assembled(?source:FlexsplineHobber):{assembly:MachineAssembly,shell:GeneratedEnclosure,machine:FlexsplineHobber,report:Dynamic} {
    var machine=source==null?new FlexsplineHobber():source,fit=fitted(machine),shell=new GeneratedEnclosure(fit.layout),assembly=new MachineAssembly();
    assembly.include("machine",machine);assembly.include("enclosure",shell);
    for(name in machine.portNames()){var port=machine.port(name,"machine");assembly.exposePort(name,port.instanceId,port.portName);}
    var uses:Array<PanelUse> = [];
    for(m in shell.components())if(Std.isOfType(m.component,machinekit.enclosure.EnclosurePanel)){
      var panel:machinekit.enclosure.EnclosurePanel=cast m.component;
      var zones:Array<EnclosureZone> = (m.id=="door/leaf"||m.id=="door/leftLeaf")?[Window,Impact,Splash]:StringTools.startsWith(m.id,"tray")?[Chute]:[Splash];
      uses.push({panelId:m.id,construction:panel.construction,zones:zones,edgesSealed:false,seamsSealed:false});
    }
    var qualificationIssues=EnclosureRules.check(uses,{process:WetMachining,maximumTemperatureC:60,oilMist:true,
      extractionProvided:false,drainageProvided:false,chamberControlProvided:false,boundaryGuardProvided:false});
    return {assembly:assembly,shell:shell,machine:machine,report:{samples:fit.samples,angularPaddingMm:fit.paddingMm,excludedElectricalRack:fit.excluded,
      input:fit.layout.input,dimensionsMm:[fit.layout.width(),fit.layout.depth(),fit.layout.height()],doorStrokeMm:fit.layout.stroke,doorLeaves:2,doorDrive:"Proposed bought rod-style electric actuator, at least 210 mm usable stroke, plus passive opposite-run belt; ideal coupling only, hardware not sized",
      doorGuide:{track:"Captured steel roller channel, nominal assumed envelope",trackWidthMm:44,slotWidthMm:8,rollerDiameterMm:22,rollersPerLeaf:4,hangersPerLeaf:2,lowerGuide:"Two stationary UHMW shoes per side with overlapping channel engagement",cover:"Removable steel splash hood",wheelSpinModeled:false,mountingDetail:"Bracket bores, fasteners, axle retention and frame attachments remain fabrication work"},
      servicePanels:shell.servicePanelIds,serviceOpenings:shell.serviceAccess,seams:shell.seamMethod,qualificationIssues:qualificationIssues,
      evidence:"X/Y/Z endpoints, A one-degree grid with radius*half-step padding; B/C descendants bounded by rotation-centred spheres",
      assumptions:"75 mm clearance; 400 mm opening; opposed belt-coupled leaves; OD140 side-jaw reservation; 100 mm mist opening; 60 C process and oil-mist rule assessment assumed. Robot, seals, retention and flow not qualified."}};
  }
  /** Hide the complete external electrical rack in default views while retaining its geometry. */
  public static function defaultView(scene:materia.project.SceneArtifact.SceneArtifactData):materia.project.SceneArtifact.SceneArtifactData {
    if(scene.assemblyDefinition!=null)for(o in scene.assemblyDefinition.occurrences)
      if(StringTools.startsWith(o.id,"machine/")&&external(o.id.substr("machine/".length)))o.visible=false;
    return scene;
  }
  public static function runtimeScene(assembly:MachineAssembly,machine:FlexsplineHobber,stroke:Float,scenarios:Bool=false):materia.project.SceneArtifact.SceneArtifactData {
    var scene=defaultView(AssemblyPreview.scene(assembly,"hobber-enclosed"));
    if(scene.machineMotion==null)scene.machineMotion={belts:[]};
    var axes=["machine/X","machine/Y","machine/Z","machine/swivel/A","machine/spindle/B","machine/table/C"];
    scene.machineMotion.hobbing={module:machine.spec.module,teeth:machine.spec.teeth,width:machine.spec.faceWidth,pressureAngle:machine.spec.pressureAngle,
      starts:machine.hob.design.starts,gashes:machine.hob.design.gashes,outside:32,length:40,wall:0.6,radialStep:0.1,axialStep:0.25,
      tolerance:0.001,feed:5,cuttingSpeed:300,axes:axes,stock:"machine/blank",controllerHz:4000,cupLength:40,cupEndThickness:2};
    scene.machineMotion.enclosureGuard={doorJoint:"enclosure/door/slide",closedSensor:"enclosure/door/closed",openSensor:"enclosure/door/open",
      spindleJoints:[axes[4],axes[5]],axisJoints:axes.slice(0,4),stroke:stroke*0.001,setupSpeedLimit:0.025,
      timeoutSeconds:10,lockTravelSeconds:0.05,linearStandstillTolerance:1e-5,rotaryStandstillTolerance:1e-4,standstillConfirmationSeconds:0.25};
    var guard:materia.project.SceneArtifact.SceneArtifactEnclosureGuard=scene.machineMotion.enclosureGuard;
    var definition=scene.assemblyDefinition;if(definition==null)throw "Unified scene needs its assembly";
    guard.enclosureParts=[for(o in definition.occurrences)if(StringTools.startsWith(o.id,"enclosure/"))o.id];
    if(scenarios) {
      var config:materia.project.SceneArtifact.SceneArtifactHobbing=scene.machineMotion.hobbing;
      config.scenario="cutting";
      config.handling={sleeve:"machine/mandrelSleeve",outside:machine.mandrel.parameterNumber("outside"),wall:machine.mandrel.parameterNumber("sleeveWall"),
        length:machine.mandrel.parameterNumber("sleeveLength"),radialTravel:machine.mandrel.parameterNumber("radialFitGap"),retractX:machinekit.hobbing.HobberXLayout.RETRACT,
        frontApproachMm:machinekit.hobbing.HobberXLayout.FRONT_APPROACH,liftMm:machinekit.hobbing.HobberXLayout.FRONT_LIFT};
    }
    HobberInspectionViews.apply(scene,machine,"machine/");
    return scene;
  }
  public static function unified():materia.project.SceneArtifact.SceneArtifactData {
    var machine=new FlexsplineHobber(new gearkit.GearSpec(1,40,4,20*Math.PI/180),4,true);
    var fitted=assembled(machine);
    return runtimeScene(fitted.assembly,machine,fitted.shell.layout.stroke,true);
  }
  public static function scene():haxe.io.Bytes return SceneArtifact.encode(unified());
  public static function main():Void {
    var scene=unified();
    var root=Sys.getCwd();while(!sys.FileSystem.exists(root+"/machinekit/examples/flexspline-hobber/FlexsplineHobber.hx")){
      var parent=haxe.io.Path.directory(root);if(parent==root||parent.length==0)throw "Cannot locate hobber worktree";root=parent;}
    var output=root+"/machinekit/examples/flexspline-hobber/build/hobber";sys.FileSystem.createDirectory(haxe.io.Path.directory(output));sys.FileSystem.createDirectory(output);
    sys.io.File.saveBytes(output+"/hobber.mtrg",SceneArtifact.encode(scene));

    Sys.println("Unified hobber scene written");
  }
}
