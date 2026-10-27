package tests;

import materia.assembly.AssemblyDefinitionCodec;
import haxe.Json;

/** Run the same transport and engineering invariants on any producer artifact. */
@:access(tests.PresentationRuntimeTests)
@:access(app.SceneCollision)
class PresentationArtifactInvariantTests {
  public static function run(path:String):Void {
    var session=new app.ProjectDocumentSession(null,false);
    session.openGeneratedProject(new app.ProjectArtifactLoader().load(sys.io.File.getBytes(path)),path);
    var definition=session.projectAssemblyDefinition,state=session.projectAssemblyState,record=session.presentation;
    if(definition==null||state==null||record==null)throw "Artifact requires an engineering assembly and presentation";
    var host=new haxeon.ui.host.UiHostContext(null,new haxeon.platform.event.EventPump(),()->{},()->{});
    var viewport=new app.EditorPerspectiveViewport("artifact-invariants",session.scene,host);
    viewport.setPresentation(record,definition,session.presentationCenters(),session.assemblyUnitMetres());
    viewport.solvePresentationJoints=session.presentationJointPoses;
    viewport.presentationGeometryFingerprint=session.presentationGeometryFingerprint;
    var controller=viewport.presentation;if(controller==null)throw "Missing artifact controller";
    function frame():String {
      var frame=viewport.presentationFrame();if(frame==null)throw "Missing artifact frame";
      return PresentationRuntimeTests.signature(frame);
    }
    function evidence():Null<String> {var result=viewport.presentationFrame();if(result==null)throw "Missing evidence frame";return result.evidence;}
    function engineering():String return AssemblyDefinitionCodec.encode(definition)+"|"+
      AssemblyDefinitionCodec.encodeState(definition,state)+"|"+Json.stringify(session.scene.records())+"|"+
      Json.stringify(session.assemblyMates.solve(definition,state).result)+"|"+app.SceneCollision.keyOf(session.scene,session);
    var before=engineering(),fresh=frame(),checks=0;
    function check(ok:Bool,message:String):Void {checks++;if(!ok)throw path+": "+message;}
    try {
      for(view in record.views)for(amount in [0.0,1.0]) {
        viewport.selectPresentationView(view.id);viewport.setPresentationExplode(amount);frame();
        check(engineering()==before,"View and explode leave engineering data unchanged");
      }
      for(procedure in record.procedures)for(step in 0...procedure.steps.length+1) {
        viewport.selectPresentationProcedure(procedure.id);for(_ in 0...step)controller.next(false);
        var expected=frame();
        check(engineering()==before,"Procedure leaves engineering data unchanged at step "+step);
        controller.restart();for(_ in 0...step)controller.next(false);
        check(frame()==expected,"Restart reproduces step "+step);
        var ahead=Std.int(Math.min(2,procedure.steps.length-step));
        var previousStep=controller.cursor.step;for(_ in 0...ahead)controller.next(false);
        var advanced=controller.cursor.step-previousStep;for(_ in 0...advanced)controller.previous();
        check(frame()==expected,"Previous reproduces step "+step);
      }
      for(procedure in record.procedures)for(index in 0...procedure.steps.length)for(member in materia.presentation.StepMotions.members(procedure.steps[index].action))switch member.motion.evidence {
          case Checked(_,Failed,_,_):
            viewport.selectPresentationProcedure(procedure.id);
            for(_ in 0...index)controller.next(false);
            var blocked=frame(),blockedStep=controller.cursor.step;
            check(evidence()=="Failed","Negative control displays Failed");
            controller.next();controller.advance(1);
            check(controller.cursor.step==blockedStep&&frame()==blocked,"Failed withdrawal cannot animate or advance");
            var fingerprint=viewport.presentationGeometryFingerprint;
            viewport.presentationGeometryFingerprint="changed-geometry";
            check(evidence()=="Stale","Changed geometry displays cached failure as Stale");
            viewport.presentationGeometryFingerprint=fingerprint;
          case _:
      }
      viewport.resetPresentation();check(frame()==fresh,"Reset returns to the fresh-open frame");
      check(session.document.history.undoCount==0,"Presentation transport creates no document edits");
      Sys.println('Artifact presentation: $checks checks passed: $path');
    } catch(error:Dynamic) {viewport.dispose();session.dispose();throw error;}
    viewport.dispose();session.dispose();
  }
}
