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
    for(object in loaded.objects)object.collisionEnabled=false; // CAD travel/clearance is separately checked.
    project.openGeneratedProject(loaded,"guard-runtime-test");
    try {
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
      editor.dispose();fonts.dispose();
      simulation.dispose();project.dispose();
    } catch(e:Dynamic){simulation.dispose();project.dispose();throw e;}
  }
  static function has(n:haxeon.ui.core.RenderNode,label:String):Bool {
    if(n.semantics!=null&&n.semantics.label==label)return true;
    for(child in n.children)if(has(child,label))return true;return false;
  }
}
