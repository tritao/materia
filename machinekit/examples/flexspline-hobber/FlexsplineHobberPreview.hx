import haxe.io.Bytes;
import machinekit.assembly.AssemblyPreview;
import materia.project.SceneArtifact;

class FlexsplineHobberPreview {
  public static function cutting():Bytes {
    var machine=new FlexsplineHobber(new gearkit.GearSpec(1,40,4,20*Math.PI/180),4);
    var data=AssemblyPreview.scene(machine,"flexspline-hobber-cut");
    if(data.machineMotion==null)data.machineMotion={belts:[]};
    data.machineMotion.hobbing={module:1,teeth:40,width:4,pressureAngle:20*Math.PI/180,starts:1,gashes:4,outside:32,length:40,
      wall:0.6,radialStep:0.1,axialStep:0.25,tolerance:0.001,feed:5,cuttingSpeed:300,
      axes:["X","Y","Z","swivel/A","spindle/B","table/C"],stock:"blank",controllerHz:4000};
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
  public static function detailed():Bytes {
    return SceneArtifact.encode(AssemblyPreview.scene(new FlexsplineHobber(new gearkit.GearSpec(1,40,4,20*Math.PI/180),4,true),"flexspline-hobber-detailed"));
  }
  public static function hobber():Bytes return SceneArtifact.encode(AssemblyPreview.scene(new FlexsplineHobber(),"flexspline-hobber"));
}
