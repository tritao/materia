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
      id+":"+JsonWire.encode(pose)+":"+Std.string(frame.visibility.get(id));}].join("|")+":"+JsonWire.encode(frame.sections)+":"+frame.highlight.join(",")+":"+frame.evidence+":"+(frame.activeStep==null?"":frame.activeStep.id)+":"+JsonWire.encode(frame.camera)+":"+frame.view;
  }
  static function rejected(action:Void->Void):Bool {try action() catch(_:Dynamic)return true;return false;}
  public static function main():Int {
    try {run();for(path in Sys.args())PresentationArtifactInvariantTests.run(path);Sys.println('Presentation runtime: $checks checks passed');return 0;}
    catch(error:Dynamic){Sys.println('Presentation runtime failed: $error');Sys.println(haxe.CallStack.toString(haxe.CallStack.exceptionStack()));return 1;}
  }
  public static function run():Void {
    var assembly=new machinekit.assembly.MachineAssembly();
    var block=machinekit.tooling.ToolingBlock.fromBox({x:0,y:0,z:0,width:10,depth:10,height:10},"steel");
    assembly.addComponent("base",block,AssemblyFrames.identity());
    assembly.addComponent("cover",block,AssemblyFrames.translation(30,0,0));
    assembly.addMemberConnector("base","guide",AssemblyFrames.translation(30,0,0));
    assembly.addMate("open","prismatic","base","guide","cover","mount");
    var artifact=machinekit.assembly.AssemblyPreview.scene(assembly,"presentation-test");
    var definition=artifact.assemblyDefinition;if(definition==null)throw "Missing assembly definition";
    var record=materia.presentation.Presentations.assembled(definition);
    record.views.push({id:"inside",label:"See inside",camera:record.views[0].camera,visibility:[
      {select:{occurrence:"cover",subtree:false},mode:Transparent}],sections:[
      {occurrence:null,point:{x:5,y:5,z:5},normal:{x:1,y:1,z:1}}],explode:0,source:Generated("test")});
    record.explode=[{select:{occurrence:"cover",subtree:false},direction:{x:0,y:0,z:1},distance:50,stage:1}];
    var select:PartSelector={occurrence:"cover",subtree:false};
    var motion:PresentationMotion={path:[{x:0,y:0,z:0},{x:0,y:0,z:50}],evidence:Illustrative};
    record.procedures=[{id:"service",label:"Cover service",kind:Service,start:Assembled,source:Generated("test"),steps:[
      {id:"open",title:"Open guide",instruction:"Open the guide.",highlight:[select],action:SetJoints([{joint:"open",value:5}])},
      {id:"remove",title:"Remove cover",instruction:"Withdraw the cover.",view:"inside",highlight:[select],action:Remove(select,motion)},
      {id:"install",title:"Install cover",instruction:"Install the cover.",highlight:[select],action:Install(select,motion)}]}];
    artifact.presentation=record;
    var dir=app.editor.WorkerAssetPath.resolve("build/presentation-"+Std.random(100000000));FileSystem.createDirectory(dir);
    var source=FileSystem.fullPath(dir)+"/assembly.mtrg";File.saveBytes(source,SceneArtifact.encode(artifact));
    var session=new ProjectDocumentSession(null,false);
    session.openGeneratedProject(new ProjectArtifactLoader().load(File.getBytes(source)),source);
    var host=new haxeon.ui.host.UiHostContext(null,new haxeon.platform.NativeKitEvents(),()->{},()->{});
    var viewport=new EditorPerspectiveViewport("presentation-test",session.scene,host);
    var dock=new EditorPerspectiveViewport("presentation-dock-test",session.scene,host);
    function bind(view:EditorPerspectiveViewport,readOnly:Bool=false):Void {
      view.setPresentation(session.presentation,session.projectAssemblyDefinition,session.presentationCenters(),session.assemblyUnitMetres(),readOnly);
      view.solvePresentationJoints=session.presentationJointPoses;
    }
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
    var originalBom=Json.stringify(assembly.billOfMaterials().lines());
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
    viewport.selectPresentationView("assembled");
    viewport.setPresentationExplode(1);
    viewport.updateViewportSize(800,600);
    viewport.camera.frame(0.03,0,0.05,0.03,0.02,0.02,800/600);
    var target=viewport.camera.project(0.032,0.002,0.052,800,600);
    if(target==null)throw "Exploded pick target is behind the camera";
    check(viewport.pick(target.x,target.y)=="project:cover","Picking uses the exploded display transform");
    viewport.setSimulationState(true,[],1);
    check(!viewport.presentationControlsEnabled(),"Simulation disables presentation controls");
    viewport.setPresentationExplode(0.25);
    check(controller.cursor.explode==1,"Disabled explode edits cannot change the cursor");
    viewport.setSimulationState(false,[],2);
    viewport.selectPresentationProcedure("service");
    controller.next(false);
    var opened=viewport.presentationFrame();if(opened==null)throw "Missing procedure frame";
    var jaw=opened.poses.get("cover");if(jaw==null)throw "Missing procedure cover pose";
    check(jaw.y==5,"SetJoints evaluates an independent display kinematic state");
    var stepOne=signature(opened),highlightView=new nativekit.scene.SceneView();viewport.sectionView(highlightView);
    check(highlightView.diagnosticTintValues().length>0&&opened.evidence=="Illustrative"&&opened.view=="inside","Active movement uses its referenced view, highlights its part and reports its evidence");
    controller.next(false);controller.next(false);controller.previous();controller.previous();
    var previous=viewport.presentationFrame();if(previous==null)throw "Missing Previous frame";
    check(signature(previous)==stepOne,"Next twice then Previous twice reproduces the same procedure frame");
    controller.restart();controller.next(false);
    var restarted=viewport.presentationFrame();if(restarted==null)throw "Missing Restart frame";
    check(signature(restarted)==stepOne,"Restart and Next reproduce the same step");
    viewport.nextPresentationStep();viewport.advancePresentation(0.3);
    var halfway=viewport.presentationFrame();if(halfway==null)throw "Missing animated frame";
    var traveling=halfway.poses.get("cover");if(traveling==null)throw "Missing traveling cover";
    check(controller.animating&&Math.abs(traveling.z-25)<1e-8,"Next animates along the stored movement path");
    viewport.advancePresentation(0.3);
    check(!controller.animating&&controller.cursor.step==2,"Animation completes into the folded next step");
    viewport.resetPresentation();var reset=viewport.presentationFrame();if(reset==null)throw "Missing reset frame";
    check(signature(reset)==expected,"Reset yields the fresh-open frame after view and explode changes");
    check(AssemblyDefinitionCodec.encode(definition)==originalDefinition&&AssemblyDefinitionCodec.encodeState(definition,state)==originalState,
      "View changes leave assembly definition and state byte-identical");
    check(Json.stringify(session.assemblyMates.solve(definition,state).result)==originalMates,"Presentation preserves the mate solve result");
    check(Json.stringify(session.scene.records())==originalScene&&SceneArtifact.encode(artifact).compare(originalArtifact)==0,
      "View changes preserve engineering scene records, collision inputs and source artifact");
    check((@:privateAccess app.SceneCollision.keyOf(session.scene,session))==originalCollisionKey&&collision.query(session.scene,session,0.01)==originalPairs,
      "Presentation preserves collision broadphase input and cached engineering query");
    check(Json.stringify(assembly.billOfMaterials().lines())==originalBom,"Presentation preserves the exported engineering BOM");
    check(session.document.history.undoCount==0,"Cursor changes create no document edits");
    PresentationExportInvariantTests.run(session,assembly,viewport,dir);
    viewport.selectPresentationView("inside");
    var authored=viewport.capturePresentationView("authored-section","Saved section");
    session.savePresentationView(authored);bind(viewport);viewport.selectPresentationView(authored.id);
    check(session.document.history.undoCount==1,"Saving a view creates one undoable document edit");
    session.document.undo();var undone=session.presentation;check(undone!=null&&undone.views.length==2,"Undo removes the authored view");
    session.document.redo();
    var authoredProcedure:Procedure=JsonWire.decode(JsonWire.encode(record.procedures[0]));
    authoredProcedure.id="authored-service";authoredProcedure.source=Authored;
    session.savePresentationProcedure(authoredProcedure);
    check(session.document.history.undoCount==2,"Saving a procedure creates one undoable document edit");
    var bad:Procedure=JsonWire.decode(JsonWire.encode(authoredProcedure));
    bad.steps[1].action=Remove(select,{path:motion.path,evidence:Checked("invented",Passed,"geometry","claim")});
    check(rejected(()->session.savePresentationProcedure(bad)),"Authored procedures cannot claim checked evidence");
    var saved=dir+"/saved.materia";session.save(saved);
    var savedProject=app.SceneCodec.decodeProject(File.getContent(saved));if(savedProject==null||savedProject.presentation==null)throw "Missing saved authored entries";
    var edits=PresentationEdits.decode(savedProject.presentation);
    check(edits.views.length==1&&edits.views[0].source==Authored,"Only authored entries are saved");
    check(edits.procedures.length==1&&edits.procedures[0].source==Authored,"Authored procedure survives document encoding");
    var first=PresentationEdits.encode(edits),firstDocument=File.getContent(saved);
    session.open(saved);bind(viewport);
    var reopened=session.presentation;if(reopened==null)throw "Missing reopened presentation";
    check(reopened.procedures.length==2,"Generated and authored procedures survive save and reopen");
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
    bind(viewport);
    var offsetHistory=session.document.history.undoCount,offsetState=AssemblyDefinitionCodec.encodeState(definition,session.projectAssemblyState);
    check(viewport.beginPresentationOffsetEdit("project:cover",offset->session.savePresentationOffset(offset)),"Offset gizmo starts on an assembly occurrence");
    @:privateAccess viewport.previewPresentationOffset(75);
    check(session.document.history.undoCount==offsetHistory&&AssemblyDefinitionCodec.encodeState(definition,session.projectAssemblyState)==offsetState,"Offset preview preserves engineering state and history");
    bind(viewport);check(viewport.presentationOffsetEditing(),"An ordinary editor rebind preserves the active offset preview");
    viewport.cancelPresentationOffsetEdit();
    check(session.document.history.undoCount==offsetHistory,"Cancelling the gizmo records no edit");
    viewport.beginPresentationOffsetEdit("project:cover",offset->session.savePresentationOffset(offset));
    viewport.setSimulationState(true,[],1);
    check(!viewport.presentationOffsetEditing()&&session.document.history.undoCount==offsetHistory,"Simulation cancels an offset preview without recording it");
    check(!viewport.beginPresentationOffsetEdit("project:cover",offset->session.savePresentationOffset(offset)),"Simulation disables offset authoring");
    viewport.setSimulationState(false,[],2);
    viewport.beginPresentationOffsetEdit("project:cover",offset->session.savePresentationOffset(offset));
    @:privateAccess viewport.previewPresentationOffset(75);
    @:privateAccess viewport.finishPresentationOffsetEdit(true);
    var edited=session.presentation;if(edited==null)throw "Missing authored offset";
    check(session.document.history.undoCount==offsetHistory+1&&edited.explode.filter(offset->offset.select.occurrence=="cover").length==1,"Offset commit replaces the generated offset as one undoable edit");
    check(edited.explode.filter(offset->offset.select.occurrence=="cover")[0].distance==75&&record.explode[0].distance!=75,"Authored offset leaves generated data unchanged");
    session.document.undo();check(session.presentation==reopened,"Undo restores the original presentation");
    session.document.redo();session.save(saved);session.open(saved);
    var restored=session.presentation;if(restored==null)throw "Missing reopened offset";
    check(restored.explode.filter(offset->offset.select.occurrence=="cover")[0].distance==75,"Authored offset survives save and reopen");
    collision.dispose();dock.dispose();viewport.dispose();session.dispose();
    tests.ViewportCoordinatesTests.main();
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
