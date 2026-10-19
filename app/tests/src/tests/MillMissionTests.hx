package tests;

import app.ApplicationSimulation;
import app.ProjectDocumentSession;
import app.ProjectSourceLoader;
import app.MissionController;
import app.editor.MissionPanel;
import app.editor.WorkerAssetPath;
import robotkit.world.RobotWorld;

/** Execute the shared setup on both actual machine configurations, not a substitute axis fixture. */
class MillMissionTests {
 static function check(ok:Bool,message:String):Void {if(!ok)throw message;}
 static function find(node:haxeon.ui.core.RenderNode,label:String):Bool {
  if(node.semantics!=null&&node.semantics.label==label)return true;
  for(child in node.children)if(find(child,label))return true;return false;
 }
 public static function main():Int {
  try {run();Sys.println("Mill missions: both configurations completed their CNC mission");return 0;}
  catch(error:Dynamic){Sys.println("Mill missions failed: "+error);return 1;}
 }
 static function run():Void {
  var fonts=haxeon.ui.FontCollection.create();fonts.add(WorkerAssetPath.resolve("haxeon/packages/ui/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf"));
  for(id in ["small","compact-vmc"]) {
   var path=WorkerAssetPath.resolve("machinekit/examples/bench-mill/materia.project.json");
   var scene=ProjectSourceLoader.load(path,null,null,id);
   Sys.println(id+" source loaded");Sys.stdout().flush();
   check(scene.cncJob!=null&&scene.cncJob.mission!=null,"machining mission survives artifact loading");
   var session=new ProjectDocumentSession(null,false),simulation=new ApplicationSimulation(new RobotWorld());
   // This test exercises CNC motion and stock removal, not physical contact qualification.
   // Keep actual machine joints/masses/geometry; the CNC kernel checks cutter/holder contacts.
   for(object in scene.objects)object.collisionEnabled=false;
   simulation.setBackend(Sys.getEnv("MILL_MISSION_BACKEND")=="mujoco"?ApplicationSimulation.MUJOCO:ApplicationSimulation.DETERMINISTIC);
   session.openGeneratedProject(scene,path);
   var controller=new MissionController(session,simulation,()->{},_ -> {});
   var ready=controller.snapshot(),mission=ready.machining;
   if(mission==null)throw "Missing machining mission";
   check(ready.hasWork&&ready.machiningPreview&&ready.mission==null&&mission.id=="bearing-block"&&ready.machiningStatus=="Ready · Play to start",id+" CNC mission ready without a robot mission");
   var uiEditor=new app.Main.ReferenceEditorApp(fonts,WorkerAssetPath.resolve("build/mill-mission-ui-"+Std.random(100000000)+".json"),null,null,null,null,null,null,true);
   var panel=uiEditor.ui.submit(new MissionPanel().build(controller,uiEditor.appearance.theme.tokens),new haxeon.ui.LayoutFrame(800,600));
   check(find(panel,"Machine bearing block")&&!find(panel,"This project has no mission."),id+" Mission panel displays the CNC mission");
   uiEditor.dispose();
   check(simulation.rebuild(session.sensors,session.scene,session),id+" simulation builds: "+simulation.error);
   Sys.println(id+" simulation ready");Sys.stdout().flush();
   var player=simulation.cncPlayer();if(player==null)throw "Missing CNC player";
   for(i in 0...20)simulation.step();
   player.hold();
   var heldSteps=0;
   while(!player.held()&&player.failure==null&&heldSteps<2000){simulation.step();heldSteps++;}
   simulation.start();
   check(player.held()&&controller.snapshot().machiningStatus=="Held",id+" mission follows actual feed hold");
   simulation.stop();player.resume();
   check(controller.snapshot().machiningStatus=="Paused",id+" mission follows transport pause");
   Sys.println(id+" hold and pause checks passed; executing the cut");Sys.stdout().flush();
   var deadline=Sys.time()+180,steps=0;
   while(player.passes==0&&player.failure==null&&Sys.time()<deadline&&steps<100000) {simulation.step();steps++;}
   check(player.failure==null,id+" CNC failure: "+player.failure);
   check(player.passes==1,id+" mission completes within bounded simulation: "+steps+" steps");
   var cut=player.stock;check(cut!=null&&cut.removed>0,id+" mission actually cuts stock");
   check(controller.snapshot().machiningStatus=="Completed",id+" mission reports actual completion");
   for(i in 0...10)simulation.step();
   check(player.passes==1,id+" completed mission does not restart automatically");
   check(simulation.reset()&&controller.snapshot().machiningStatus=="Ready · Play to start",id+" reset makes mission ready again");
   simulation.stop();simulation.dispose();session.dispose();
   Sys.println(id+" CNC mission completed in "+steps+" steps");
  }
  fonts.dispose();
 }
}
