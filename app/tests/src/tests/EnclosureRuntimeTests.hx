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
    Sys.println("Building fitted guarded hobber");Sys.stdout().flush();
    var machine=new FlexsplineHobber(new gearkit.GearSpec(1,40,4,20*Math.PI/180),4);
    var fitted=HobberEnclosurePreview.assembled(machine);
    var data=HobberEnclosurePreview.runtimeScene(fitted.assembly,machine,fitted.shell.layout.stroke);
    var artifact=SceneArtifact.decode(SceneArtifact.encode(data));
    check(artifact.machineMotion!=null&&artifact.machineMotion.enclosureGuard!=null,"Guard policy survives artifact roundtrip");
    var invalid=SceneArtifact.decode(SceneArtifact.encode(data));
    var invalidMotion=invalid.machineMotion;if(invalidMotion==null)throw "Missing test motion";
    var invalidGuard=invalidMotion.enclosureGuard;if(invalidGuard==null)throw "Missing test guard";
    invalidGuard.spindleJoints.push(invalidGuard.doorJoint);
    var rejected=false;try {SceneArtifact.encode(invalid);}catch(_ : Dynamic){rejected=true;}
    check(rejected,"Artifact rejects hazardous-motion feedback bound to the door itself");
    var loaded=new ProjectArtifactLoader().load(SceneArtifact.encode(artifact));
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
      editor.session.document.redo();click("Close door");
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
  static function find(n:haxeon.ui.core.RenderNode,label:String):Null<haxeon.ui.core.RenderNode> {
    if(n.semantics!=null&&n.semantics.label==label)return n;
    for(child in n.children){var result=find(child,label);if(result!=null)return result;}return null;
  }
  static function has(n:haxeon.ui.core.RenderNode,label:String):Bool {
    if(n.semantics!=null&&n.semantics.label==label)return true;
    for(child in n.children)if(has(child,label))return true;return false;
  }
}
