import haxe.io.Bytes;
import machinekit.assembly.AssemblyPreview;
import materia.project.SceneArtifact;

class FlexsplineHobberPreview {
  /** Scene defaults apply to direct artifact/project opens as well as saved view wrappers. */
  public static function mechanicalView(data:materia.project.SceneArtifact.SceneArtifactData):materia.project.SceneArtifact.SceneArtifactData {
    if(data.assemblyDefinition!=null)for(occurrence in data.assemblyDefinition.occurrences)
      if(occurrence.id=="electricalPanel"||occurrence.id=="powerSupply"||StringTools.endsWith(occurrence.id,"-driver"))occurrence.visible=false;
    return data;
  }
  public static function cutting():Bytes {
    var machine=new FlexsplineHobber(new gearkit.GearSpec(1,40,4,20*Math.PI/180),4,true);
    var data=AssemblyPreview.scene(machine,"flexspline-hobber-cut");
    if(data.machineMotion==null)data.machineMotion={belts:[]};
    data.machineMotion.hobbing={module:1,teeth:40,width:4,pressureAngle:20*Math.PI/180,starts:1,gashes:4,outside:32,length:40,
      wall:0.6,radialStep:0.1,axialStep:0.25,tolerance:0.001,feed:5,cuttingSpeed:300,
      axes:["X","Y","Z","swivel/A","spindle/B","table/C"],stock:"blank",controllerHz:4000,cupLength:40,cupEndThickness:2};
    HobberInspectionViews.apply(data,machine);
    return SceneArtifact.encode(mechanicalView(data));
  }
  /** Kinematic handling demonstration; the normal cutting scene remains fast. */
  public static function loading():Bytes {
    var data=SceneArtifact.decode(cutting());
    return loadingState(data);
  }
  static function loadingState(data:materia.project.SceneArtifact.SceneArtifactData):Bytes {
    var machine=new FlexsplineHobber(new gearkit.GearSpec(1,40,4,20*Math.PI/180),4);
    var motion=data.machineMotion;if(motion==null)throw "Missing machine motion";
    var config=motion.hobbing;if(config==null)throw "Missing hobbing job";
    config.handling={sleeve:"mandrelSleeve",outside:machine.mandrel.parameterNumber("outside"),wall:machine.mandrel.parameterNumber("sleeveWall"),
      length:machine.mandrel.parameterNumber("sleeveLength"),radialTravel:machine.mandrel.parameterNumber("radialFitGap"),retractX:machinekit.hobbing.HobberXLayout.RETRACT,frontApproachMm:machinekit.hobbing.HobberXLayout.FRONT_APPROACH,liftMm:machinekit.hobbing.HobberXLayout.FRONT_LIFT};
    var definition=data.assemblyDefinition;if(definition==null)throw "Missing assembly definition";
    var state=new cadkit.modeling.AssemblyState(definition);state.setJoint("X",machinekit.hobbing.HobberXLayout.RETRACT);data.assemblyState=state.record();
    return SceneArtifact.encode(data);
  }
  /** Full reference tool, isolated for close inspection; physical properties use the blank envelope. */
  public static function hobTool():Bytes {
    var machine=new FlexsplineHobber();
    var tool=machinekit.hobbing.Hob.fromDesign(machine.hob.design,machine.spec,40,16,false,true);
    var hob=new machinekit.hobbing.Hob(tool.values().copy().setNumber("keywayWidth",5));
    var assembly=new machinekit.assembly.MachineAssembly();assembly.addComponent("hob",hob);
    return SceneArtifact.encode(AssemblyPreview.scene(assembly,"hob-inspection"));
  }
  public static function hobCad():Bytes {
    var machine=new FlexsplineHobber();
    var hob=machinekit.hobbing.Hob.fromDesign(machine.hob.design,machine.spec,40,16,true);
    var rendered=AssemblyPreview.part("hob","Reference hob CAD",hob.geometry(machinekit.component.ComponentDetail.Preview),hob.materialId,0.01);
    return SceneArtifact.encode({lengthUnit:"mm",metresPerUnit:0.001,parts:[rendered]});
  }
  /** Nominal full-depth reference setup, at the middle of the toothed face. */
  public static function inspectionState(machine:FlexsplineHobber):cadkit.modeling.AssemblyState {
    var model=new cadkit.modeling.AssemblyModel("mm");machine.addTo(model,"");
    var state=model.initialState("flexspline-hobber-detailed"),job=FlexsplineHobberCam.job(machine),pass=job.passes[job.passes.length-1];
    state.setJoint("X",pass.centreDistance);state.setJoint("Y",pass.shift);state.setJoint("Z",machine.spec.faceWidth/2);
    state.setJoint(FlexsplineHobber.A,-machine.hob.design.leadAngle);state.setJoint(FlexsplineHobber.B,0);state.setJoint(FlexsplineHobber.C,pass.tablePhase);
    state.forwardKinematics();return state;
  }
  public static function detailed():Bytes {
    var machine=new FlexsplineHobber(null,12,true);
    var data=AssemblyPreview.scene(machine,"flexspline-hobber-detailed");
    data.assemblyState=inspectionState(machine).record();return SceneArtifact.encode(mechanicalView(data));
  }
  public static function hobber():Bytes return SceneArtifact.encode(mechanicalView(AssemblyPreview.scene(new FlexsplineHobber(),"flexspline-hobber")));
}
