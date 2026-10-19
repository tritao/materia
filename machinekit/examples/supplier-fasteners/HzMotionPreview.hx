import haxe.io.Bytes;
import cadkit.modeling.AssemblyModel;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.TypedProperty;
import machinekit.assembly.*;
import machinekit.component.*;
import machinekit.component.MassProperties.MassSource;
import machinekit.document.MachineKitDocuments;
import machinekit.document.MachineKitRecipes;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactData;
import materia.project.SceneArtifact.SceneArtifactPart;
import materia.assembly.AssemblyFrames as F;

/** Visible drawing catalog and worked Y stack. Unknown masses are isolated to
 * explicit visual estimates here; engineering massProperties() still rejects them.
 */
class HzMotionPreview {
 static function scene(assembly:MachineAssembly,id:String):SceneArtifactData {
  var model=new AssemblyModel("mm");assembly.addTo(model,"");
  var parts:Array<SceneArtifactPart> = [],messages:Array<String> = [
   "Drawing-based HZMotion study; supplier STEP downloads require login. Undimensioned shapes are simplified.",
   "Catalog/preview is not a manufacturing release or verified motor/drive selection."];
  for(m in assembly.components()) {
   var geometry=m.component.geometry(),mass:MassProperties;
   if(Std.isOfType(m.component,HzMotionComponent)) {
    var supplier=cast(m.component,HzMotionComponent);
    if(supplier.spec.massKg==null) {
     var physical=geometry.massProperties();
     mass=new MassProperties(physical.volume*1e-9*7850,physical.centerOfMass,Computed(Preview));
     messages.push(m.id+": unpublished mass; preview uses a steel-volume estimate only. Engineering mass API rejects this estimate.");
    }else mass=supplier.massProperties();
   }else mass=m.component.massProperties();
   parts.push(AssemblyPreview.part(m.id,m.component.description,geometry,m.component.materialId,null,mass));
  }
  return {lengthUnit:"mm",metresPerUnit:0.001,parts:parts,assemblyDefinition:model.definition(id),recipeDiagnostics:messages};
 }
 public static function preview(?savedDocument:String):Bytes {
  MachineKitRecipes.register();var document=new Document();
  try {
   var assembly=new MachineAssembly(),i=0;
   for(name in HzMotionCatalog.models()) {
    var values=new HzMotionComponent(name).values();
    var definition=MachineKitDocuments.define(document,HzMotionComponent.recipeType(),values);
    var instance=document.createInstance('catalog$i',definition);
    instance.restoreProperty("machinekit.occurrence",TypedProperty.text("machinekit.occurrence",'catalog$i'));
    i++;
   }
   var diagnostics:Array<String> = [];
   if(savedDocument!=null)MachineKitRecipes.reconcileDocument(document,savedDocument,diagnostics);
   i=0;
   for(element in document.allElements())if(element.kind=="instance") {
    var instance=cast(element,cadkit.parametric.InstanceElement);
    var component=MachineKitRecipes.component(instance);
    assembly.addComponent('catalog$i',component,F.translation((i%3)*160,0,Math.floor(i/3)*180));i++;
   }
   var result=scene(assembly,"hzmotion-drawing-catalog");
   result.recipeDocument=DocumentCodec.encode(document);
   for(message in diagnostics)result.recipeDiagnostics.push(message);
   var bytes=SceneArtifact.encode(result);MachineKitRecipes.forget(document);document.close();return bytes;
  }catch(e:Dynamic){MachineKitRecipes.forget(document);document.close();throw e;}
 }
 public static function axisPreview():Bytes {
  var result=scene(new HzMotionAxis(),"hzmotion-y-drive");
  result.recipeDiagnostics.push("Requires the HZMotion 51 mm screw-end profile; existing Vallder 54 mm end is not substituted.");
  result.recipeDiagnostics.push("Coupling is a reference geometry. Thread engagement, bearing preload, guides and actual mill placement remain unqualified.");
  return SceneArtifact.encode(result);
 }
 public static function main():Void {
  var root=Sys.getEnv("MATERIA_ROOT");if(root==null)throw "Set MATERIA_ROOT to the repository root";
  var dir=root+"/build/suppliers/hzmotion";
  if(!sys.FileSystem.exists(dir))sys.FileSystem.createDirectory(dir);
  var gallery=preview(),decoded=SceneArtifact.decode(gallery);
  if(decoded.parts.length!=HzMotionCatalog.models().length||decoded.recipeDocument==null)throw "Catalog preview lost editable parts";
  var reopened=preview(decoded.recipeDocument),second=SceneArtifact.decode(reopened);
  if(second.parts.length!=HzMotionCatalog.models().length)throw "Saved catalog reconstruction changed part count";
  var editedDocument=DocumentCodec.decode(decoded.recipeDocument,false,false),edited=false;
  for(element in editedDocument.allElements())if(element.kind=="instance"&&element.name=="catalog0") {
   var instance=cast(element,cadkit.parametric.InstanceElement);
   var definition=editedDocument.definition(instance.definitionId);
   definition.setUserEditedTypedDefault("model","M80E-BK15-C");
   definition.setUserEditedTypedDefault("bearingGrade","C4");edited=true;
  }
  if(!edited)throw "Missing editable catalog occurrence";
  var editedSaved=DocumentCodec.encode(editedDocument);MachineKitRecipes.forget(editedDocument);editedDocument.close();
  var editedScene=SceneArtifact.decode(preview(editedSaved)),changed=false;
  for(part in editedScene.parts)if(part.id=="catalog0"&&part.name.indexOf("M80E-BK15-C")>=0)changed=true;
  if(!changed)throw "Saved model selection did not rebuild visible geometry";
  var returned=DocumentCodec.decode(editedScene.recipeDocument,false,false),retained=false;
  for(element in returned.allElements())if(element.kind=="instance"&&element.name=="catalog0") {
   var instance=cast(element,cadkit.parametric.InstanceElement),part=MachineKitRecipes.component(instance);
   if(part.values().token("model")=="M80E-BK15-C"&&part.values().token("bearingGrade")=="C4")retained=true;
  }
  MachineKitRecipes.forget(returned);returned.close();
  if(!retained)throw "Saved preview lost selected model or bearing grade";
  sys.io.File.saveBytes(dir+"/catalog.mtrg",gallery);
  sys.io.File.saveBytes(dir+"/y-drive.mtrg",axisPreview());
  Sys.println("HZMotion gallery, saved preview and Y-drive artifacts passed");
 }
}
