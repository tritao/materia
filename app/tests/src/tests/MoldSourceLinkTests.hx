package tests;

import app.EditorSceneTree;
import materia.assembly.AssemblyFrames as F;
import materia.project.SceneArtifact;

/** Source selection is display-only and remains separate from occurrence ownership. */
@:access(app.EditorScene)
class MoldSourceLinkTests {
 static var checks=0;
 static function check(ok:Bool,message:String):Void {checks++;if(!ok)throw message;}
 public static function main():Int {run();return 0;}
 public static function run():Void {
  var assembly=new machinekit.assembly.MachineAssembly();
  var block=machinekit.tooling.ToolingBlock.fromBox({x:0,y:0,z:0,width:10,depth:10,height:10},"steel");
  assembly.addComponent("casting",block,F.identity());
  assembly.addComponent("former",block,F.translation(30,0,0));
  var artifact=machinekit.assembly.AssemblyPreview.scene(assembly,"mold-links");
  var definition=artifact.assemblyDefinition;if(definition==null)throw "Missing assembly";
  for(occurrence in definition.occurrences)occurrence.includePath=occurrence.id=="casting"?"Casting":"Formers/seat";
  var face=artifact.parts.filter(part->part.id=="casting")[0].faceRanges[0].faceIndex;
  artifact.project={dynamicParts:[],motions:[],sources:{
   features:[{id:"seat",label:"Rail seat",occurrence:"casting",faces:[face],whole:false},
    {id:"whole-casting",label:"Casting",occurrence:"casting",faces:[],whole:true}],
   members:[{occurrence:"former",sources:["seat"],shared:false}]},
   navigation:[{occurrence:"casting",label:"Back to Machine",variant:"machine",target:"frame/casting"}]};
  var bytes=SceneArtifact.encode(artifact),session=new app.ProjectDocumentSession(null,false);
  session.openGeneratedProject(new app.ProjectArtifactLoader().load(bytes),"/tmp/mold-source-links.mtrg");
  var scene=session.scene,tree=new EditorSceneTree(scene,session.projectAssemblyDefinition);
  var before=haxe.Json.stringify(scene.records()),history=scene.document.history.undoCount;
  try {
   check(tree.childCount("project:casting")==2,"Source row is under its reference casting");
   check(tree.childKeyAt("project:casting",0)=="source:seat","Source rows retain authored feature IDs");
   tree.selectKey("project:former");
   check(tree.sourceFeatures().length==1 && tree.sourceFeatures()[0].id=="seat","Tooling selection resolves casting feature");
   scene.renderSnapshot();
   check(scene.presentation.sourceFaces.active.length==1,"Only the declared source face is highlighted");
   tree.selectKey("source:seat");
   check(tree.selectionKey()=="source:seat","Feature selection survives the physical reference selection");
   check(tree.selectedObjects().join(",")=="project:former","Source selection highlights its forming tooling");
   var path=tree.selectionPath();
   check(path[path.length-2]=="project:casting" && path[path.length-1]=="source:seat","Feature path includes casting ownership");
   scene.renderSnapshot();
   check(scene.items().length==2,"Highlight overlays add no physical occurrence");
   check(haxe.Json.stringify(scene.records())==before && scene.document.history.undoCount==history,"Cross-highlighting leaves engineering and history unchanged");
   var navigation=session.navigation;
   check(navigation!=null && navigation[0].target=="frame/casting","Generated navigation survives session installation");
   tree.selectKey("project:casting");
   scene.selection.selectedCadFaceIndex=face;
   check(tree.selectedObjects().join(",")=="project:former","Picking a source face resolves its forming tooling");
   var faceNode:Null<nativekit.scene.NodeId>=null;
   for(entry in scene.presentation.sourceFaces.entries)if(entry.face==face)faceNode=entry.node;
   if(faceNode==null)throw "Missing source-face overlay";
   var raw=new NativeKitSceneRender.nkscene_render_pick_result();
   raw.set_node(@:privateAccess faceNode.nativeValue());raw.set_subelement(12345);
   var hit=@:privateAccess new nativekit.scene.PickResult(raw);
   check(scene.selectHit(hit)=="project:casting" && scene.selectedCadFaceIndex==face,
    "Overlay picks retain the source occurrence and original CAD face");
   var map=scene.sourceMap;scene.sourceMap=null;
   scene.selectHit(hit);
   check(scene.selectedCadFaceIndex<0,"Generated previews without source links keep whole-part selection");
   scene.sourceMap=map;
   tree.selectKey("source:whole-casting");scene.renderSnapshot();
   check(scene.presentation.sourceFaces.active.length==1,"Whole casting highlights use one overlay");
   var borrowed=false;
   for(entry in scene.presentation.sourceFaces.entries)if(entry.face<0)
    borrowed=!entry.owned && entry.geometry==scene.runtimeFor("project:casting").geometry;
   check(borrowed,"Whole-part highlights reuse the existing geometry resource");
   scene.replaceObjects([],"scene");scene.renderSnapshot();
   check(!scene.presentation.sourceFaces.entries.keys().hasNext(),"Removed source occurrences retire owned overlays without disposing borrowed geometry twice");
  } catch(error:Dynamic){session.dispose();throw error;}
  session.dispose();
  Sys.println("Mold source links: "+checks+" checks passed");
 }
}
