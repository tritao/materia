package tests;

import app.ProjectDocumentSession;
import app.ProjectArtifactLoader;
import app.EditorPerspectiveViewport;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyFrames;
import materia.presentation.AssemblyPresentation;
import materia.presentation.PresentationCodec;
import materia.presentation.PresentationEdits;
import materia.project.SceneArtifact;
import haxe.io.Bytes;
import haxe.Json;
import haxeon.wire.JsonWire;
import sys.io.File;
import sys.FileSystem;

@:access(app.EditorPerspectiveViewport)
class PresentationRuntimeTests {
  static var checks=0;
  static function check(ok:Bool,message:String):Void {checks++;if(!ok)throw message;}
  static function signature(frame:materia.presentation.PresentationEvaluator.PresentationFrame):String {
    var ids=[for(id in frame.poses.keys())id];ids.sort(Reflect.compare);
    return [for(id in ids){var pose=frame.poses.get(id);if(pose==null)throw "Missing display pose";
      id+":"+JsonWire.encode(pose)+":"+Std.string(frame.visibility.get(id));}].join("|")+":"+JsonWire.encode(frame.sections);
  }
  static function rejected(action:Void->Void):Bool {try action() catch(_:Dynamic)return true;return false;}
  public static function main():Int {
    try {run();Sys.println('Presentation runtime: $checks checks passed');return 0;}
    catch(error:Dynamic){Sys.println('Presentation runtime failed: $error');Sys.println(haxe.CallStack.toString(haxe.CallStack.exceptionStack()));return 1;}
  }
  public static function run():Void {
    var vertices=Bytes.alloc(96),normals=Bytes.alloc(96),indices=Bytes.alloc(48);
    vertices.setDouble(24,10);vertices.setDouble(56,10);vertices.setDouble(88,10);
    var corners=[0,2,1,0,1,3,0,3,2,1,2,3];for(i in 0...corners.length)indices.setInt32(i*4,corners[i]);
    var definition:AssemblyDefinition={schemaVersion:AssemblyDefinitionCodec.VERSION,id:"presentation-test",lengthUnit:"mm",
      definitions:[{id:"part",connectors:[]}],occurrences:[
        {id:"base",definition:"part",initialPose:AssemblyFrames.identity()},
        {id:"cover",definition:"part",initialPose:AssemblyFrames.translation(30,0,0)}],joints:[]};
    var record=materia.presentation.Presentations.assembled(definition);
    record.views.push({id:"inside",label:"See inside",camera:record.views[0].camera,visibility:[
      {select:{occurrence:"cover",subtree:false},mode:Transparent}],sections:[
      {occurrence:null,point:{x:5,y:5,z:5},normal:{x:1,y:1,z:1}}],explode:0,source:Generated("test")});
    record.explode=[{select:{occurrence:"cover",subtree:false},direction:{x:0,y:0,z:1},distance:50,stage:1}];
    var artifact:SceneArtifactData={metresPerUnit:0.001,lengthUnit:"mm",assemblyDefinition:definition,presentation:record,
      parts:[{id:"part",name:"Part",red:0.5,green:0.5,blue:0.5,vertexCount:4,indexCount:12,
        vertices:vertices,normals:normals,indices:indices,faceRanges:[]}]};
    var dir=app.editor.WorkerAssetPath.resolve("build/presentation-"+Std.random(100000000));FileSystem.createDirectory(dir);
    var source=FileSystem.fullPath(dir)+"/assembly.mtrg";File.saveBytes(source,SceneArtifact.encode(artifact));
    var session=new ProjectDocumentSession(null,false);
    session.openGeneratedProject(new ProjectArtifactLoader().load(File.getBytes(source)),source);
    var host=new haxeon.ui.host.UiHostContext(null,new haxeon.platform.NativeKitEvents(),()->{},()->{});
    var viewport=new EditorPerspectiveViewport("presentation-test",session.scene,host);
    var dock=new EditorPerspectiveViewport("presentation-dock-test",session.scene,host);
    function bind(view:EditorPerspectiveViewport,readOnly:Bool=false):Void
      view.setPresentation(session.presentation,session.projectAssemblyDefinition,session.presentationCenters(),session.assemblyUnitMetres(),readOnly);
    bind(viewport);bind(dock,true);
    var originalDefinition=AssemblyDefinitionCodec.encode(definition);
    var state=session.projectAssemblyState;if(state==null)throw "Missing engineering state";
    var originalState=AssemblyDefinitionCodec.encodeState(definition,state);
    var originalMates=Json.stringify(session.assemblyMates.solve(definition,state).result);
    var originalScene=Json.stringify(session.scene.records());
    var collision=new app.SceneCollision(()->new collisionkit.native.NativeCollisionWorld());
    var originalPairs=collision.query(session.scene,session,0.01);
    var originalCollisionKey=@:privateAccess app.SceneCollision.keyOf(session.scene,session);
    var originalArtifact=SceneArtifact.encode(artifact);
    var controller=viewport.presentation;if(controller==null)throw "Missing controller";
    var fresh=viewport.presentationFrame();if(fresh==null)throw "Missing frame";
    var expected=signature(fresh);
    viewport.selectPresentationView("inside");
    var view=new nativekit.scene.SceneView();viewport.sectionView(view);
    check(view.materialOverrideCount()==1&&view.clipPlaneCount()==1,"Typed visibility and general section planes reach the view");
    var independent=dock.presentation;if(independent==null)throw "Missing dock controller";
    check(independent.cursor.view=="assembled","Dock cursor is independent from main viewport");
    controller.setExplode(1);
    var moved=viewport.presentationFrame();if(moved==null)throw "Missing exploded frame";
    var cover=moved.poses.get("cover");if(cover==null)throw "Missing cover";
    check(cover.z==50,"Display transforms derive from the cursor");
    viewport.resetPresentation();var reset=viewport.presentationFrame();if(reset==null)throw "Missing reset frame";
    check(signature(reset)==expected,"Reset yields the fresh-open frame after view and explode changes");
    check(AssemblyDefinitionCodec.encode(definition)==originalDefinition&&AssemblyDefinitionCodec.encodeState(definition,state)==originalState,
      "View changes leave assembly definition and state byte-identical");
    check(Json.stringify(session.assemblyMates.solve(definition,state).result)==originalMates,"Presentation preserves the mate solve result");
    check(Json.stringify(session.scene.records())==originalScene&&SceneArtifact.encode(artifact).compare(originalArtifact)==0,
      "View changes preserve engineering scene records, collision inputs and source artifact");
    check((@:privateAccess app.SceneCollision.keyOf(session.scene,session))==originalCollisionKey&&collision.query(session.scene,session,0.01)==originalPairs,
      "Presentation preserves collision broadphase input and cached engineering query");
    check(session.document.history.undoCount==0,"Cursor changes create no document edits");
    viewport.selectPresentationView("inside");
    var authored=viewport.capturePresentationView("authored-section","Saved section");
    session.savePresentationView(authored);bind(viewport);viewport.selectPresentationView(authored.id);
    check(session.document.history.undoCount==1,"Saving a view creates one undoable document edit");
    session.document.undo();var undone=session.presentation;check(undone!=null&&undone.views.length==2,"Undo removes the authored view");
    session.document.redo();
    var saved=dir+"/saved.materia";session.save(saved);
    var savedProject=app.SceneCodec.decodeProject(File.getContent(saved));if(savedProject==null||savedProject.presentation==null)throw "Missing saved authored entries";
    var edits=PresentationEdits.decode(savedProject.presentation);
    check(edits.views.length==1&&edits.views[0].source==Authored,"Only authored entries are saved");
    var first=PresentationEdits.encode(edits),firstDocument=File.getContent(saved);
    session.open(saved);bind(viewport);
    var reopened=session.presentation;if(reopened==null)throw "Missing reopened presentation";
    check(reopened.views.length==3&&reopened.views[2].sections.length==1,"Authored section survives save and reopen");
    var reopenedController=viewport.presentation;if(reopenedController==null)throw "Missing reopened controller";
    check(reopenedController.cursor.view=="assembled"&&reopenedController.cursor.procedure==null,"Reopen resets the session cursor");
    session.save(saved);var second=app.SceneCodec.decodeProject(File.getContent(saved));
    if(second==null||second.presentation==null)throw "Missing resaved entries";
    check(second.presentation==first,"Authored entries re-encode byte-identically");
    check(File.getContent(saved)==firstDocument,"The complete saved document re-encodes byte-identically");
    edits.views[0].visibility=[{select:{occurrence:"renamed",subtree:false},mode:Hidden}];
    check(rejected(()->PresentationEdits.merge(definition,record,edits)),"Renamed occurrence fails authored validation");
    check(rejected(()->SceneArtifact.decodeProject(Json.parse('{"dynamicParts":[],"motions":[],"initialView":{"yaw":0,"pitch":0}}'))),"Old project camera fields are rejected");
    check(rejected(()->SceneArtifact.decodeProject(Json.parse('{"dynamicParts":[],"motions":[],"inspectionCameras":[]}'))),"Old camera arrays are rejected");
    collision.dispose();dock.dispose();viewport.dispose();session.dispose();
    @:privateAccess tests.ViewportCoordinatesTests.sectionInspection();
    @:privateAccess tests.SceneEditingTests.perspectiveHoverInput();
    noPresentationPicking(host);
  }
  static function noPresentationPicking(host:haxeon.ui.host.UiHostContext):Void {
    var scene=new app.EditorScene([{id:"plain",label:"Plain",type:"rectangle",x:0.0,y:0.0,z:0.0,
      width:0.2,height:0.2,depth:0.2,collisionEnabled:false,dynamicBody:false,mass:1.0,
      red:0.5,green:0.5,blue:0.5,visible:true}]);
    var viewport=new EditorPerspectiveViewport("plain-test",scene,host);
    viewport.updateViewportSize(800,600);viewport.camera.frame(0,0,0,0.2,0.2,0.2,800/600);
    check(viewport.pick(400,300)=="plain","A scene without presentation still picks through its ordinary render view");
    viewport.dispose();scene.dispose();
  }

}
