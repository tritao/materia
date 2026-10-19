import haxe.io.Bytes;
import machinekit.milling.*;
import machinekit.motion.*;
import machinekit.assembly.*;
import machinekit.component.*;
import cadkit.modeling.AssemblyModel;
import materia.project.SceneArtifact;

/** Main mill views. Both configurations use the same frame/Z/spindle assembly path.
 * Study-specific checks remain executable separately; old projects are view aliases.
 */
class MillProject {
 public static function smallAssembly():MachineAssembly {
  var reference=new BenchMill(false,false),f=MillFramePreview.layout(reference),guide=new HiwinHgh15ca();
  var z=MillFramePreview.zLayout(reference,f,guide),xy=new MillXYStageLayout(f,z.spindleY,BenchMill.RAIL,guide,true);
  return MillFramePreview.concept(reference,new MillFrame(f,z,xy));
 }
 public static function small():Bytes return scene(smallAssembly(),"mill-small",[
  "250 x 160 x 250 mm mechanical mill, selected Leadshine/HZMotion Y drive and complete shared Z/spindle assemblies.",
  "Spindle release is an inspection mechanism; supplier force, preload, drive capacity and control remain unqualified."]);
 public static function large():Bytes return MillCapacityStudy.preview();

 /** Shared writer retains each occurrence's hierarchy and cosmetic-thread data.
  * Imported bearing/carriage mass substitutes are for display only, not sizing.
  */
 public static function scene(assembly:MachineAssembly,id:String,diagnostics:Array<String>):Bytes {
  assembly.validate();var model=new AssemblyModel("mm");assembly.addTo(model,"");
  var parts:Array<materia.project.SceneArtifact.SceneArtifactPart> = [];
  for(row in assembly.components()) {
   var geometry=row.component.geometry(),mass:MassProperties;
   if(Std.isOfType(row.component,HgBlock)||Std.isOfType(row.component,HiwinHgh15ca)||
    (Std.isOfType(row.component,HzMotionComponent)&&StringTools.endsWith(row.id,"floatingBearingY"))) {
    var props=geometry.massProperties();
    mass=new MassProperties(props.volume*1e-9*7850,props.centerOfMass,machinekit.component.MassProperties.MassSource.Computed(Preview));
   } else mass=row.component.massProperties();
   parts.push(AssemblyPreview.part(row.id,row.component.description,geometry,row.component.materialId,null,mass));
  }
  return SceneArtifact.encode({lengthUnit:"mm",metresPerUnit:0.001,parts:parts,assemblyDefinition:model.definition(id),
   recipeDiagnostics:diagnostics.concat(["Imported bearing/carriage masses use display-only steel-volume estimates; not engineering sizing inputs."])});
 }

 public static function main():Void {
  var root=Sys.getEnv("MATERIA_ROOT"),dir=root+"/machinekit/examples/bench-mill/build/mill";
  sys.FileSystem.createDirectory(dir);
  var assembly=smallAssembly(),reference=new BenchMill(false,false),f=MillFramePreview.layout(reference);
  var z=MillFramePreview.zLayout(reference,f,new HiwinHgh15ca());
  MillSpindleIntegrity.checkPayload(assembly,z).throwIfErrors();
  MillSpindleIntegrity.checkMotorRemoval(assembly).throwIfErrors();
  var bytes=scene(assembly,"mill-small",["Complete small mechanical mill with selected Y drive."]);
  var decoded=SceneArtifact.decode(bytes);
  for(id in ["xyStage/screwX","xyStage/screwY","zAxis/screw","spindle/drawbar","spindle/cylinder","spindle/motor","spindle/belt"])
   if(decoded.parts.filter(p -> p.id==id).length!=1)throw "Unified small scene is missing "+id;
  sys.io.File.saveBytes(dir+"/small.mtrg",bytes);
  sys.io.File.saveContent(dir+"/small.assembly.json",assembly.encode());
  Sys.println("Unified small mill payload, motor removal and scene completeness passed");
  MillCapacityStudy.main();
  sys.io.File.saveBytes(dir+"/large.mtrg",sys.io.File.getBytes(root+"/machinekit/examples/bench-mill/build/capacity/mill-capacity.mtrg"));
  Sys.println("Unified mill project: small and large scenes generated");
 }
}
