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
  var z=MillFramePreview.zLayout(reference,f,guide),xy=new MillXYStageLayout(f,z.spindleY,BenchMill.RAIL,guide,MillScrewDrive.m60dBk12());
  return MillFramePreview.concept(reference,new MillFrame(f,z,xy));
 }
 public static function small():Bytes {
  var reference=new BenchMill(false,false),frame=MillFramePreview.layout(reference);
  var z=MillFramePreview.zLayout(reference,frame,new HiwinHgh15ca());
  var xy=new MillXYStageLayout(frame,z.spindleY,BenchMill.RAIL,new HiwinHgh15ca(),MillScrewDrive.m60dBk12());
  return new MillMachiningSetup(smallAssembly(),["xyStage/x","xyStage/y","zAxis/z"],"xyStage/table",xy.tableTop)
   .scene("mill-small",["250 x 160 x 250 mm mechanical mill with shared bearing-block mission."]);
 }
 public static function large():Bytes {
  var layout=new MillCapacityLayout();
  return new MillMachiningSetup(MillCapacityStudy.build(layout,false),["xyStage/x","xyStage/y","zAxis/z"],"xyStage/table",layout.tableTop)
   .scene("mill-large",["400 x 300 x 380 mm packaging study; XY drives are not fitted. Shared bearing-block mission uses kinematic preview bounds."]);
 }

 /** Shared writer retains each occurrence's hierarchy and cosmetic-thread data.
  * Imported bearing/carriage mass substitutes are for display only, not sizing.
  */
 public static function scene(assembly:MachineAssembly,id:String,diagnostics:Array<String>):Bytes {
  assembly.validate();var model=new AssemblyModel("mm");assembly.addTo(model,"");
  var parts:Array<materia.project.SceneArtifact.SceneArtifactPart> = [];
  for(row in assembly.components()) {
   var geometry=row.component.geometry(),mass:MassProperties;
   if(Std.isOfType(row.component,HgBlock)||Std.isOfType(row.component,HiwinHgh15ca)||
    (Std.isOfType(row.component,HzMotionComponent)&&cast(row.component,HzMotionComponent).spec.massKg==null)) {
    var props=geometry.massProperties();
    mass=new MassProperties(props.volume*1e-9*7850,props.centerOfMass,machinekit.component.MassProperties.MassSource.Computed(Preview));
   } else mass=row.component.massProperties();
   var label=Std.isOfType(row.component,LeadshineServo)?row.component.designation+" — "+row.component.description:row.component.description;
   parts.push(AssemblyPreview.part(row.id,label,geometry,row.component.materialId,null,mass));
  }
  var definition=model.definition(id);
  AssemblyPreview.preserveOwnership(definition,assembly.definition());
  return SceneArtifact.encode({lengthUnit:"mm",metresPerUnit:0.001,parts:parts,assemblyDefinition:definition,
   recipeDiagnostics:diagnostics.concat(["Imported bearing/carriage masses use display-only steel-volume estimates; not engineering sizing inputs."])});
 }

 public static function main():Void {
  var root=Sys.getEnv("MATERIA_ROOT"),dir=root+"/machinekit/examples/bench-mill/build/mill";
  sys.FileSystem.createDirectory(dir);
  var assembly=smallAssembly(),reference=new BenchMill(false,false),f=MillFramePreview.layout(reference);
  var z=MillFramePreview.zLayout(reference,f,new HiwinHgh15ca());
  MillSpindleIntegrity.checkPayload(assembly,z).throwIfErrors();
  MillSpindleIntegrity.checkMotorRemoval(assembly).throwIfErrors();
  var bytes=small();
  var decoded=SceneArtifact.decode(bytes);
  for(id in ["xyStage/screwX","xyStage/screwY","zAxis/screw","spindle/drawbar","spindle/cylinder","spindle/motor","spindle/belt"])
   if(decoded.parts.filter(p -> p.id==id).length!=1)throw "Unified small scene is missing "+id;
  var exported=decoded.assemblyDefinition;
  if(exported==null)throw "Unified small scene is missing its assembly";
  var owned=0;
  for(occurrence in assembly.definition().occurrences)if(occurrence.includePath!=null) {
   owned++;
   if(exported.occurrences.filter(o -> o.id==occurrence.id&&o.includePath==occurrence.includePath).length!=1)
    throw "Unified small scene lost assembly ownership for "+occurrence.id;
  }
  if(owned==0)throw "Unified small scene ownership check needs authored subassemblies";
  sys.io.File.saveBytes(dir+"/small.mtrg",bytes);
  sys.io.File.saveContent(dir+"/small.assembly.json",assembly.encode());
  Sys.println("Unified small mill payload, motor removal and scene completeness passed");
  MillCapacityStudy.main();
  sys.io.File.saveBytes(dir+"/large.mtrg",large());
  Sys.println("Unified mill project: small and large scenes generated");
 }
}
