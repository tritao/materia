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
  public static function hobber():Bytes return SceneArtifact.encode(AssemblyPreview.scene(new FlexsplineHobber(),"flexspline-hobber"));
}
