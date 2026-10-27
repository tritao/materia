package tests;

import app.ApplicationSimulation;
import app.ProjectDocumentSession;
import app.ProjectArtifactLoader;
import materia.project.SceneArtifact;
import robotkit.world.RobotWorld;

/** Real assembly, native runtime switches and velocity snapshots; no substituted guard feedback. */
class EnclosureRuntimeTests {
  static var checks=0;
  static function check(ok:Bool,why:String):Void {checks++;if(!ok)throw why;}
  public static function main():Int {
    try {run();Sys.println('Live enclosure runtime: $checks checks passed');return 0;}
    catch(e:Dynamic){Sys.println('Live enclosure runtime failed: $e');return 1;}
  }
  static function run():Void {
    EnclosureGuardStatusTests.run();
    Sys.println("Building fitted guarded hobber");Sys.stdout().flush();
    var machine=new FlexsplineHobber(new gearkit.GearSpec(1,40,4,20*Math.PI/180),4);
    var fitted=HobberEnclosurePreview.assembled(machine);
    var data=HobberEnclosurePreview.runtimeScene(fitted.assembly,machine,fitted.shell.layout.stroke);
    var artifact=SceneArtifact.decode(SceneArtifact.encode(data));
    var loaded=new ProjectArtifactLoader().load(SceneArtifact.encode(artifact));
    var physical=loaded.physical;
    if(physical==null)throw "Missing fitted physical data";
    var assemblyDefinition=artifact.assemblyDefinition;
    if(assemblyDefinition==null)throw "Missing fitted assembly";
    check(assemblyDefinition.joints.filter(j->j.role==materia.assembly.AssemblyDefinition.AssemblyJointRole.Closure).length>0,
      "Enclosure mounting constraints remain authored");
    var bridge=cadbridge.AssemblySimulationBridge.toRobotModel(assemblyDefinition,physical);
    check(bridge.closures.filter(c->c.parent==c.child).length==0,
      "Rigid mounting closures do not create physics constraints from a body to itself");
    var badMountArtifact=SceneArtifact.decode(SceneArtifact.encode(data));
    var badMountDefinition=badMountArtifact.assemblyDefinition;
    if(badMountDefinition==null)throw "Missing mounting test assembly";
    var badMount=badMountDefinition.joints.filter(j->j.role==materia.assembly.AssemblyDefinition.AssemblyJointRole.Closure)[0];
    badMount.childConnector="baseMount0";
    var badMountRejected=false;
    try {cadbridge.AssemblySimulationBridge.toRobotModel(badMountDefinition,physical);}
    catch(e:Dynamic){badMountRejected=Std.string(e).indexOf("Inconsistent rigid assembly closure")>=0;}
    check(badMountRejected,"Inconsistent mounts inside one rigid body are rejected before physics");
    var cameraProject=artifact.presentation;
    check(cameraProject!=null&&cameraProject.views.length==6,"Presentation views survive artifact roundtrip");
    var badCamera=SceneArtifact.decode(SceneArtifact.encode(data));
    var badProject=badCamera.presentation;if(badProject==null)throw "Missing presentation";
    badProject.views[1].camera=Anchored({occurrence:"missing-camera-target",subtree:false},{x:0,y:0,z:0},40,0,0);
    var cameraRejected=false;try{SceneArtifact.encode(badCamera);}catch(_:Dynamic){cameraRejected=true;}
    check(cameraRejected,"Artifact rejects missing camera anchors");
    badProject.views[1].camera=Anchored({occurrence:"machine/blank",subtree:false},{x:0,y:0,z:0},-1,0,0);
    cameraRejected=false;try{SceneArtifact.encode(badCamera);}catch(_:Dynamic){cameraRejected=true;}
    check(cameraRejected,"Artifact rejects invalid camera scale");
    check(artifact.machineMotion!=null&&artifact.machineMotion.enclosureGuard!=null,"Guard policy survives artifact roundtrip");
    var invalid=SceneArtifact.decode(SceneArtifact.encode(data));
    var invalidMotion=invalid.machineMotion;if(invalidMotion==null)throw "Missing test motion";
    var invalidGuard=invalidMotion.enclosureGuard;if(invalidGuard==null)throw "Missing test guard";
    invalidGuard.spindleJoints.push(invalidGuard.doorJoint);
    var rejected=false;try {SceneArtifact.encode(invalid);}catch(_ : Dynamic){rejected=true;}
    check(rejected,"Artifact rejects hazardous-motion feedback bound to the door itself");
    var project=new ProjectDocumentSession(null,false),simulation=new ApplicationSimulation(new RobotWorld(),ApplicationSimulation.MUJOCO);
    project.openGeneratedProject(loaded,"guard-runtime-test");
    try {
      var fullScene=new ApplicationSimulation(new RobotWorld(),ApplicationSimulation.MUJOCO);
      try {check(fullScene.rebuild(project.sensors,project.scene,project),"Full collision scene builds: "+fullScene.error);}
      catch(e:Dynamic){fullScene.dispose();throw e;}
      fullScene.dispose();
      for(object in loaded.objects)object.collisionEnabled=false; // Isolate controller motion from contact qualification.
      project.openGeneratedProject(loaded,"guard-runtime-test");
      checkPacking();
      check(simulation.rebuild(project.sensors,project.scene,project),"Runtime builds: "+simulation.error);
      Sys.println("Guarded native runtime built");Sys.stdout().flush();
      var guard=simulation.enclosureController(),cut=simulation.hobbingPlayer();
      if(guard==null||cut==null)throw "Guarded cutting player missing";
      check(guard.status().state==app.EnclosureGuardStatus.EnclosureRuntimeState.Paused&&!guard.status().feedbackSampled&&!guard.status().initialized,"Prepared controller publishes paused, unsampled status");
      var observedRobot = @:privateAccess guard.robot;
      var hobIndex = @:privateAccess guard.spindleIndices[0];
      function hobTurning():Bool return Math.abs(observedRobot.robot.snapshot().velocities.get(hobIndex))>1;
      function tick():Void {simulation.step();guard.present();}
      function until(test:Void->Bool,limit:Int,why:String):Void {
        var n=0;while(!test()&&n<limit&&!guard.status().fault&&cut.failure==null){tick();n++;}
        Sys.println(why+" after "+n+" ticks");Sys.stdout().flush();
        check(test(),why+": "+guard.status().message+(cut.failure==null?"":" / "+cut.failure));
      }
      until(()->guard.status().canStart,2000,"Measured closed and lock feedback make runtime ready");
      check(!cut.cutStarted&&!guard.status().allowCut,"Play/stepping alone cannot start cutting");
      until(()->guard.status().initialized,3000,"Controller confirms startup feedback and standstill");
      check(guard.status().state==app.EnclosureGuardStatus.EnclosureRuntimeState.Paused&&!guard.status().canStartCut,"Stepping while paused does not enable playback actions");
      simulation.start();var runningStatus=guard.status();
      check(runningStatus.state==app.EnclosureGuardStatus.EnclosureRuntimeState.Ready&&runningStatus.canOpen,"Transport start publishes ready actions immediately");
      simulation.stop();
      check(guard.status().state==app.EnclosureGuardStatus.EnclosureRuntimeState.Paused&&guard.status().canResume&&!guard.status().canOpen,"Transport pause publishes disabled actions immediately");
      check(runningStatus.state==app.EnclosureGuardStatus.EnclosureRuntimeState.Ready,"Previously published status remains immutable after pausing");
      guard.requestOpen();until(()->guard.status().open,50000,"Actual native door joint reaches open feedback");
      guard.requestCut();for(_ in 0...100)tick();
      check(!cut.cutStarted&&!guard.status().allowCut,"Door-open cut request cannot start the program");
      guard.requestClose();until(()->guard.status().closed&&guard.status().locked,50000,"Closure waits for measured switch and independent lock actuator");
      check(!cut.cutStarted,"Closure does not reuse the rejected cut request");
      guard.requestCut();until(()->cut.cutStarted&&hobTurning(),50000,"Fresh Start cut runs the actual hob spindle");
      guard.requestOpen();tick();check(!guard.status().allowCut&&guard.status().locked,"Open request removes cut permission but retains the lock");
      var sawStopping=false,n=0;
      while(!guard.status().open&&n<50000&&!guard.status().fault&&cut.failure==null) {
        tick();var s=guard.status();
        if(!s.spindleStandstill||!s.axesStandstill){check(s.locked,"Door stays locked until measured hazardous motion stops");sawStopping=true;}
        check(!s.allowCut,"Cut permission stays off throughout stopping and opening");n++;
      }
      check(guard.status().open&&sawStopping&&cut.guardInterrupted,"Controlled stop completes before actual door opens");
      check(cut.failure==null,"Controlled access stop does not fault the motion runtime");
      guard.requestClose();until(()->guard.status().closed&&guard.status().locked,50000,"Door can reclose after controlled stop");
      guard.requestCut();for(_ in 0...100)tick();
      check(!guard.status().allowCut,"Interrupted stock requires explicit simulation reset before another cut");
      check(simulation.reset(),"Explicit reset restores the guarded runtime");
      check(!guard.status().initialized&&!guard.status().feedbackSampled&&guard.status().state==app.EnclosureGuardStatus.EnclosureRuntimeState.Paused,"Reset clears initialization and stale feedback");
      until(()->guard.status().canStart,2000,"Reset waits for fresh lock feedback");
      check(!cut.cutStarted&&!guard.status().allowCut,"Reset cannot automatically restart cutting");
      guard.requestCut();until(()->cut.cutStarted,1000,"Fresh start after reset is accepted");
      guard.setBoundaryClear(false);tick();check(guard.status().fault&&!guard.status().allowCut,"Access-boundary interruption trips the live cut gate");
      check(simulation.reset(),"Boundary fault requires explicit reset");
      until(()->guard.status().canStart,2000,"Guard ready for native E-stop check");
      check(simulation.emergencyStop(),"Native emergency stop delivered");
      until(()->guard.status().fault,100,"Native E-stop feedback reaches the guard");
      check(!guard.status().allowCut&&guard.status().locked,"Native E-stop removes cutting permission and keeps the door locked");
      var fonts=haxeon.ui.FontCollection.create();
      fonts.add(app.editor.WorkerAssetPath.resolve("haxeon/packages/ui/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf"));
      var editor=new app.Main.ReferenceEditorApp(fonts,app.editor.WorkerAssetPath.resolve("build/enclosure-ui.json"),null,null,null,null,null,null,true);
      var panel=app.editor.EnclosureGuardPanel.build(simulation);check(panel!=null,"Runtime exposes operator guard controls");
      var tree=editor.ui.submit(panel,new haxeon.ui.LayoutFrame(900,600));
      check(has(tree,"Open door")&&has(tree,"Close door")&&has(tree,"Start cut")&&has(tree,"Reset guard"),"Operator panel contains door and cutting actions");
      editor.session.openGeneratedProject(loaded,"door-design-preview");
      var installedPresentation=editor.session.presentation;
      check(installedPresentation!=null&&installedPresentation.views.length==6,"Project installation retains presentation views");
      editor.switchMode(app.EditorMode.Design);editor.resetWorkspace();
      editor.openInspectionCamera();
      check(!editor.workspace.isOpen("inspection-camera"),"Inspection camera cannot open in Design mode");
      editor.requestInspectionCamera();
      check(!editor.workspace.isOpen("inspection-camera"),"Launch camera request waits for Simulate mode");
      @:privateAccess editor.modeSnapshots.remove(app.EditorMode.Simulate.id);
      editor.switchMode(app.EditorMode.Simulate);
      check(editor.workspace.isOpen("inspection-camera")&&editor.workspace.isOpen("perspective"),"Entering Simulate honours a deferred inspection request");
      check(belowPanel(editor.workspace.root,"sensors","inspection-camera")&&belowPanel(editor.workspace.root,"cnc","inspection-camera"),"Inspection defaults below the Sensors/CNC tab group in Simulate");
      var layout=editor.workspace.snapshotJson();editor.workspace.close("inspection-camera");
      check(editor.workspace.restoreJson(layout)&&editor.workspace.isOpen("inspection-camera"),"Inspection pane placement survives workspace roundtrip");
      editor.switchMode(app.EditorMode.Design);
      check(!editor.workspace.isOpen("inspection-camera"),"Returning to Design hides the inspection pane");
      editor.switchMode(app.EditorMode.Simulate);
      check(editor.workspace.isOpen("inspection-camera")&&belowPanel(editor.workspace.root,"sensors","inspection-camera"),"Returning to Simulate restores the inspection dock");
      editor.switchMode(app.EditorMode.Design);
      var cameraHost=new haxeon.ui.host.UiHostContext(fonts,new haxeon.platform.event.EventPump(),function(){},function(){});
      var closeup=new app.EditorPerspectiveViewport("inspection-test",editor.session.scene,cameraHost);
      var editorPresentation=editor.session.presentation;if(editorPresentation==null)throw "Missing editor presentation";
      var preset=editorPresentation.views[1];
      closeup.setPresentation(editor.session.presentation,editor.session.projectAssemblyDefinition,editor.session.presentationCenters(),editor.session.assemblyUnitMetres(),true);
      closeup.selectPresentationView("cutting");
      var beforeVisibility=editor.session.scene.records().map(r->r.visible).join(","),beforeHistory=editor.session.document.history.undoCount;
      var filtered=new nativekit.scene.SceneView();@:privateAccess closeup.applyPresentation(filtered);
      check(filtered.visibilityOverrideCount()>0,"Inspection camera applies its own enclosure visibility overrides");
      check(new nativekit.scene.SceneView().visibilityOverrideCount()==0,"Another camera inherits no inspection visibility overrides");
      check(beforeVisibility==editor.session.scene.records().map(r->r.visible).join(",")&&editor.session.document.history.undoCount==beforeHistory,"Inspection visibility does not edit the scene or history");
      var anchor="",offset:Array<Float> = [],yaw=0.0,pitch=0.0;
      switch preset.camera {
        case Anchored(select,delta,_,y,p): anchor=select.occurrence;offset=[delta.x*data.metresPerUnit,delta.y*data.metresPerUnit,delta.z*data.metresPerUnit];yaw=y;pitch=p;
        case _:throw "Expected anchored cutting view";
      }
      var sceneId="project:"+anchor;
      closeup.setSimulationState(true,[{id:sceneId,position:[0.1,0.2,1.0],rotation:[0,0,0,1]}],1);
      @:privateAccess closeup.updatePresentationCamera();
      var camera=@:privateAccess closeup.camera;
      var target=[camera.targetX,camera.targetY,camera.targetZ],angle=[camera.yaw,camera.pitch];
      closeup.setSimulationState(true,[{id:sceneId,position:[0.1,0.2,1.0],rotation:[0,0,1,0]}],2);
      @:privateAccess closeup.updatePresentationCamera();
      check(camera.targetX==target[0]&&camera.targetY==target[1]&&camera.targetZ==target[2]&&camera.yaw==angle[0]&&camera.pitch==angle[1],"Rotating the workpiece never spins or moves the inspection camera");
      closeup.setSimulationState(true,[{id:sceneId,position:[0.15,0.2,1.02],rotation:[0,0,1,0]}],3);
      @:privateAccess closeup.updatePresentationCamera();
      check(Math.abs(camera.targetX-target[0]-0.05)<1e-9&&Math.abs(camera.targetZ-target[2]-0.02)<1e-9,"Inspection target follows live translation");
      var distance=camera.distance;closeup.setPresentationZoom(0.5);
      @:privateAccess closeup.updatePresentationCamera();
      check(camera.distance==distance,"Optical zoom keeps the camera outside the cutter instead of moving it into the geometry");
      var wheelRoot=editor.ui.submit(closeup,new haxeon.ui.LayoutFrame(400,300));
      var imageNode=find(wheelRoot,"Scene perspective GPU view");if(imageNode==null)throw "Missing inspection image";
      var lens=0.5;closeup.onInspectionZoom=value->{lens=value;};
      function wheel(delta:Float):haxeon.ui.core.UiEvent {
        var event=new haxeon.ui.core.UiEvent(haxeon.ui.core.UiEventKind.Scroll,imageNode.id,200,150,0,delta);
        @:privateAccess imageNode.invokePhase(event,"bubble");return event;
      }
      var scroll=wheel(-120);
      check(lens<0.5&&scroll.defaultPrevented&&scroll.propagationStopped,"Inspection wheel zooms in and consumes the routed event");
      wheel(120);check(Math.abs(lens-0.5)<1e-9,"Opposite wheel movement restores optical zoom");
      wheel(1000000);check(lens==8,"Wheel zoom out respects the button limit");
      scroll=wheel(1000000);check(scroll.defaultPrevented&&scroll.propagationStopped,"Wheel is consumed even at the zoom limit");
      wheel(-1000000);check(lens==0.2,"Wheel zoom in respects the button limit");
      @:privateAccess closeup.updatePresentationCamera();
      check(camera.distance==distance&&camera.yaw==angle[0]&&camera.pitch==angle[1],"Wheel zoom preserves camera position and orientation");
      function pointer(kind:String,x:Float,y:Float,modifiers:Int=0,button:Int=2):haxeon.ui.core.UiEvent {
        var event=new haxeon.ui.core.UiEvent(kind,imageNode.id,x,y,0,0,button,0,modifiers);
        @:privateAccess imageNode.invokePhase(event,"bubble");return event;
      }
      var oldYaw=camera.yaw;
      pointer(haxeon.ui.core.UiEventKind.PointerDown,200,150);pointer(haxeon.ui.core.UiEventKind.PointerMove,240,170);pointer(haxeon.ui.core.UiEventKind.PointerUp,240,170);
      check(camera.yaw!=oldYaw,"Inspection uses regular viewport orbit navigation");
      var orbited=camera.yaw;closeup.setPresentationZoom(0.2);
      @:privateAccess closeup.updatePresentationCamera();
      check(camera.yaw==orbited,"Rebuilding the inspection pane preserves a user orbit");
      var beforePan=[camera.targetX,camera.targetY,camera.targetZ];
      pointer(haxeon.ui.core.UiEventKind.PointerDown,200,150,haxeon.ui.core.UiModifier.Shift);pointer(haxeon.ui.core.UiEventKind.PointerMove,240,170,haxeon.ui.core.UiModifier.Shift);pointer(haxeon.ui.core.UiEventKind.PointerUp,240,170,haxeon.ui.core.UiModifier.Shift);
      check(camera.targetX!=beforePan[0]||camera.targetY!=beforePan[1]||camera.targetZ!=beforePan[2],"Inspection uses regular viewport pan navigation");
      var panned=[camera.targetX,camera.targetY,camera.targetZ];
      closeup.setSimulationState(true,[{id:sceneId,position:[0.25,0.2,1.02],rotation:[0,0,0,1]}],4);
      @:privateAccess closeup.updatePresentationCamera();
      check(Math.abs(camera.targetX-panned[0]-0.1)<1e-9&&camera.yaw==orbited,"Live target motion preserves the user's pan and orbit");
      var selection=editor.session.scene.selectedId;
      pointer(haxeon.ui.core.UiEventKind.PointerDown,200,150,0,0);pointer(haxeon.ui.core.UiEventKind.PointerMove,240,170,0,0);pointer(haxeon.ui.core.UiEventKind.PointerUp,240,170,0,0);
      check(selection==editor.session.scene.selectedId&&beforeHistory==editor.session.document.history.undoCount,"Inspection navigation cannot select or edit the scene");
      closeup.resetPresentation();@:privateAccess closeup.updatePresentationCamera();
      var closeupController=closeup.presentation;if(closeupController==null)throw "Missing controller";
      check(closeupController.cursor.view=="assembled"&&closeupController.cursor.procedure==null,"Reset restores the default view and cursor");
      closeup.selectPresentationView("cutting");@:privateAccess closeup.updatePresentationCamera();
      check(camera.yaw==yaw&&camera.pitch==pitch&&Math.abs(camera.targetX-0.25-offset[0])<1e-9,"Selecting a view restores framing at the current target position");
      closeup.dispose();
      var enclosureParts=[for(rule in editorPresentation.views.filter(v->v.id=="enclosure-hidden")[0].visibility)
        if(StringTools.startsWith(rule.select.occurrence,"enclosure/"))rule.select.occurrence];
      var externalIds=[for(rule in editorPresentation.views[0].visibility)rule.select.occurrence];
      function nodeCount(ids:Array<String>):Int {
        var nodes=new Map<String,Bool>();
        for(id in ids) {
          var root=editor.session.scene.info("project:"+id).node();nodes.set(Std.string(root.stableValue()),true);
          for(node in editor.session.scene.runtimePartNodes("project:"+id))nodes.set(Std.string(node.stableValue()),true);
        }
        return [for(id in nodes.keys())id].length;
      }
      var externalCount=nodeCount(externalIds),enclosureCount=nodeCount(enclosureParts);
      var mainCamera=new app.EditorPerspectiveViewport("main-presentation-test",editor.session.scene,cameraHost);
      mainCamera.setPresentation(editor.session.presentation,editor.session.projectAssemblyDefinition,editor.session.presentationCenters(),editor.session.assemblyUnitMetres());
      mainCamera.selectPresentationView("enclosure-hidden");
      var hiddenView=new nativekit.scene.SceneView();@:privateAccess mainCamera.applyPresentation(hiddenView);
      check(hiddenView.visibilityOverrideCount()==enclosureCount+externalCount,"Enclosure hiding applies only view overrides for authored enclosure parts");
      mainCamera.selectPresentationView("enclosure-transparent");
      var ghostView=new nativekit.scene.SceneView();@:privateAccess mainCamera.applyPresentation(ghostView);
      check(ghostView.materialOverrideCount()==enclosureCount&&ghostView.visibilityOverrideCount()==externalCount,"Transparent enclosure keeps every part in the view with translucent material overrides");
      mainCamera.selectPresentationView("assembled");
      var restoredView=new nativekit.scene.SceneView();@:privateAccess mainCamera.applyPresentation(restoredView);
      check(restoredView.materialOverrideCount()==0&&restoredView.visibilityOverrideCount()==externalCount,"Visible restores authored presentation without document edits");
      check(beforeVisibility==editor.session.scene.records().map(r->r.visible).join(",")&&beforeHistory==editor.session.document.history.undoCount,"Enclosure presentation never changes document visibility or history");
      mainCamera.dispose();
      var machinePanel=new app.editor.MachineControlPanel(),frame=new haxeon.ui.LayoutFrame(900,1600);
      tree=editor.ui.submit(machinePanel.build(editor),frame);
      check(has(tree,"ENCLOSURE · DESIGN PREVIEW")&&has(tree,"Open door")&&has(tree,"Close door"),"Door controls exist before Play");
      check(!has(tree,"Start cut")&&!editor.simulation.isActive(),"Design preview does not expose cutting or initialize simulation");
      var history=editor.session.document.history.undoCount;
      function click(label:String):Void {
        var root=editor.ui.submit(machinePanel.build(editor),frame),node=find(root,label);
        if(node==null)throw 'Missing preview action $label';
        var b=node.globalBounds();editor.ui.pointerDown(b.x+b.width/2,b.y+b.height/2,0);editor.ui.pointerUp(b.x+b.width/2,b.y+b.height/2,0);
      }
      function position(id:String):Float return app.MachineControlModel.axes(editor.session).filter(a->a.id==id)[0].value;
      click("Open door");
      check(Math.abs(position("enclosure/door/slide")-fitted.shell.layout.stroke)<1e-6,"Preview opens master leaf without Play");
      check(Math.abs(position("enclosure/door/leftSlide")+fitted.shell.layout.stroke)<1e-6,"Preview moves the coupled opposite leaf");
      check(editor.session.document.history.undoCount==history+1,"Preview is one undoable pose edit");
      editor.session.document.undo();check(Math.abs(position("enclosure/door/slide"))<1e-6,"Undo restores closed doors");
      editor.session.document.redo();
      var previewSimulation=editor.simulation;
      check(previewSimulation.rebuild(editor.session.sensors,editor.session.scene,editor.session),"Simulation builds from open design preview");
      var previewGuard=previewSimulation.enclosureController();if(previewGuard==null)throw "Missing preview-transition guard";
      var paused=app.editor.EnclosureGuardPanel.build(previewSimulation);if(paused==null)throw "Missing paused controls";
      check(has(editor.ui.submit(paused,frame),"Resume simulation"),"Prepared paused simulation offers Resume beside the doors");
      var initializing=0;
      while(previewGuard.status().phase!="Open"&&initializing<10000&&!previewGuard.status().fault) {
        previewSimulation.step();previewGuard.present();initializing++;
      }
      check(previewGuard.status().phase=="Open"&&previewGuard.status().open&&!previewGuard.status().closed&&!previewGuard.status().fault,"Open design pose reaches confirmed access after measured standstill");
      check(!previewGuard.status().allowCut,"Starting with the door open never grants cut permission");
      previewGuard.requestClose();var closing=0;
      while((!previewGuard.status().closed||!previewGuard.status().locked)&&closing<15000&&!previewGuard.status().fault) {
        previewSimulation.step();previewGuard.present();closing++;
      }
      check(previewGuard.status().closed&&previewGuard.status().locked&&!previewGuard.status().fault,"Guard can close and lock from open design preview");
      previewSimulation.dispose();click("Close door");
      check(Math.abs(position("enclosure/door/slide"))<1e-6&&Math.abs(position("enclosure/door/leftSlide"))<1e-6,"Preview close restores both leaves");
      check(!editor.simulation.isActive(),"Preview never starts simulation");
      check(has(editor.ui.submit(machinePanel.build(editor),frame),"ENCLOSURE · DESIGN PREVIEW"),"Preview remains available after repeated actions");
      editor.dispose();fonts.dispose();
      simulation.dispose();project.dispose();
    } catch(e:Dynamic){simulation.dispose();project.dispose();throw e;}
  }
  static function checkPacking():Void {
    var shapes:Array<robotkit.runtime.RobotRuntimeBlueprint.RobotRuntimeLinkShape> = [];
    var points=[for(x in [-1.0,1.0])for(y in [-1.0,1.0])for(z in [-1.0,1.0])for(v in [x,y,z])v];
    for(i in 0...4)shapes.push(new robotkit.runtime.RobotRuntimeBlueprint.RobotRuntimeLinkShape(i,new robotkit.model.CollisionShape(collisionkit.CollisionGeometry.Convex(points))));
    var surface=app.SimulationContactSurfaces.robotPart();
    var pairs=[new robotkit.runtime.RobotRuntimeBlueprint.RobotRuntimeContactPair(0,2,surface)];
    var packed=robotkit.runtime.SimulationCollisionPacking.pack(shapes,pairs,3,1);
    check(packed.hulls.length==1&&packed.hulls[0]==1&&packed.shapes.length==3,"Overflow uses an unpaired convex hull slot");
    check(packed.shapeMap[0]==0&&packed.shapeMap[2]==1&&packed.shapeMap[3]==2,"Explicit contact pair indices survive packing");
    var rejected=false;try{robotkit.runtime.SimulationCollisionPacking.pack(shapes,pairs,2,1);}catch(_:Dynamic){rejected=true;}
    check(rejected,"Combined collision capacity remains bounded");
    shapes[1].shape.geometry=collisionkit.CollisionGeometry.Sphere(1);
    shapes[3].shape.geometry=collisionkit.CollisionGeometry.Sphere(1);
    rejected=false;try{robotkit.runtime.SimulationCollisionPacking.pack(shapes,pairs,3,1);}catch(_:Dynamic){rejected=true;}
    check(rejected,"Paired or primitive shapes cannot spill into hull storage");
  }
  static function belowPanel(node:haxeon.ui.docking.DockNode,upper:String,lower:String):Bool {
    return switch(node) {
      case Split(axis,_,first,second):
        (axis==haxeon.ui.docking.DockSplitAxis.Vertical&&haxeon.ui.docking.DockNodeTools.contains(first,upper)&&haxeon.ui.docking.DockNodeTools.contains(second,lower))||belowPanel(first,upper,lower)||belowPanel(second,upper,lower);
      default:false;
    };
  }
  static function find(n:haxeon.ui.core.RenderNode,label:String):Null<haxeon.ui.core.RenderNode> {
    if(n.semantics!=null&&n.semantics.label==label)return n;
    for(child in n.children){var result=find(child,label);if(result!=null)return result;}return null;
  }
  static function has(n:haxeon.ui.core.RenderNode,label:String):Bool {
    if(n.semantics!=null&&n.semantics.label==label)return true;
    for(child in n.children)if(has(child,label))return true;return false;
  }
}
