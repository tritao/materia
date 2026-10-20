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
 static function nodeByKey(node:haxeon.ui.core.RenderNode,key:String):Null<haxeon.ui.core.RenderNode> {
  if(node.styleKey==key)return node;
  for(child in node.children){var found=nodeByKey(child,key);if(found!=null)return found;}return null;
 }
 static function checkSpeedSlider(simulation:ApplicationSimulation,session:ProjectDocumentSession,
     fonts:haxeon.ui.FontCollection,id:String):Void {
  var editor=new app.Main.ReferenceEditorApp(fonts,WorkerAssetPath.resolve("build/mill-speed-ui-"+Std.random(100000000)+".json"),null,null,null,null,null,null,true);
  var panel=new app.editor.CncPanel(),frame=new haxeon.ui.LayoutFrame(800,600);
  var player=simulation.cncPlayer();if(player==null)throw "Missing CNC player";
  function select(scale:Float):Void {
   var tree=editor.ui.submit(panel.build(simulation,editor.appearance.theme.tokens,null,session),frame);
   var slider=nodeByKey(tree,"cnc-speed");if(slider==null)throw "Missing speed slider";
   var resolved=slider.resolved;if(resolved==null)throw "Speed slider has no layout";
   var bounds=resolved.clippedViewportBounds();check(bounds.width>20&&bounds.height>0,"speed slider is visible");
   var x=bounds.x+10+(scale-0.05)/1.95*(bounds.width-20),y=bounds.y+bounds.height/2;
   editor.ui.pointerDown(x,y,0);editor.ui.pointerUp(x,y,0);
   var refreshed=editor.ui.submit(panel.build(simulation,editor.appearance.theme.tokens,null,session),frame);
   check(Math.abs(player.speedOverride-scale)<1e-9,id+" slider reaches "+scale+", observed "+player.speedOverride);
   var updated=nodeByKey(refreshed,"cnc-speed");
   check(updated!=null&&updated.semantics!=null&&Math.abs(updated.semantics.numericValue-scale)<1e-9,"slider thumb agrees with player");
  }
  try {
   var diagnostic="Motion program op 4: path.timing.stream.step failed with MotionKit error -6";
   @:privateAccess player.fail(diagnostic);
   var failed=editor.ui.submit(panel.build(simulation,editor.appearance.theme.tokens,null,session),frame);
   check(find(failed,"Could not plan the next move")&&!find(failed,diagnostic),"failure is explained with diagnostics initially hidden");
   var detailButton=nodeByKey(failed,"cnc-error-details");
   if(detailButton==null||detailButton.resolved==null)throw "Missing error details action";
   var detailBounds=detailButton.resolved.clippedViewportBounds();
   editor.ui.pointerDown(detailBounds.x+detailBounds.width/2,detailBounds.y+detailBounds.height/2,0);
   editor.ui.pointerUp(detailBounds.x+detailBounds.width/2,detailBounds.y+detailBounds.height/2,0);
   var expanded=editor.ui.submit(panel.build(simulation,editor.appearance.theme.tokens,null,session),frame);
   check(find(expanded,diagnostic),"technical details disclose the exact diagnostic");
   check(simulation.reset(),"failed program can be reset");
   var recovered=editor.ui.submit(panel.build(simulation,editor.appearance.theme.tokens,null,session),frame);
   check(!find(recovered,diagnostic)&&!find(recovered,"Could not plan the next move"),"reset clears the error presentation");
   for(scale in [0.05,0.5,1.0,2.0])select(scale);
   var timings:Array<Int> = [];
   for(scale in [0.5,1.0,2.0]) {
    check(simulation.reset(),"speed benchmark resets the real machine");select(1.0);
    for(i in 0...600)simulation.step();
    select(scale);
    var steps=600,deadline=Sys.time()+180;
    while(player.passes==0&&player.failure==null&&steps<100000&&Sys.time()<deadline){simulation.step();steps++;}
    check(player.failure==null&&player.passes==1,id+" completes at speed "+scale+": "+player.failure);
    timings.push(steps);
    Sys.println(id+" speed "+Math.round(scale*100)+"% completed in "+steps+" simulation steps");Sys.stdout().flush();
   }
   check(timings[0]>timings[1]*1.2,id+" 50% slider slows actual execution: "+timings);
   check(timings[2]<=timings[1]*1.05,id+" 200% slider does not slow execution: "+timings);
   Sys.println(id+" speed slider 50/100/200%: "+timings+" simulation steps");Sys.stdout().flush();
  }catch(error:Dynamic){editor.dispose();throw error;}
  editor.dispose();
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
   var cncPanel=new app.editor.CncPanel();
   var preview=uiEditor.ui.submit(cncPanel.build(simulation,uiEditor.appearance.theme.tokens,null,session),new haxeon.ui.LayoutFrame(800,600));
   var job=session.cncJob;if(job==null)throw "Missing CNC source";
   var originalSource=job.source;
   check(find(preview,"Ready · Play to start")&&find(preview,"    1  "+originalSource.split("\n")[0]),id+" program is readable before Play");
   check(!find(preview,"Hold"),id+" preview has no live execution controls");
   var replacement=originalSource.split("\n");replacement[0]="; Changed preview";job.source=replacement.join("\n");
   var changed=uiEditor.ui.submit(cncPanel.build(simulation,uiEditor.appearance.theme.tokens,null,session),new haxeon.ui.LayoutFrame(800,600));
   check(find(changed,"    1  ; Changed preview"),id+" same-length program replacements refresh the displayed source");
   job.source=originalSource;
   var empty=uiEditor.ui.submit(cncPanel.build(simulation,uiEditor.appearance.theme.tokens,null,uiEditor.session),new haxeon.ui.LayoutFrame(800,600));
   check(find(empty,"This project has no machining or print program.")&&!find(empty,"Ready · Play to start"),id+" empty document clears prior source");
   uiEditor.dispose();
   check(simulation.rebuild(session.sensors,session.scene,session),id+" simulation builds: "+simulation.error);
   Sys.println(id+" simulation ready");Sys.stdout().flush();
   var player=simulation.cncPlayer();if(player==null)throw "Missing CNC player";
   var conditions=job.setupJoints;
   check(conditions!=null&&conditions.length==1,id+" retains vise setup condition");
   var expected=conditions[0].position;
   conditions[0].position=expected+0.01;
   simulation.step();
   check(player.failure!=null&&player.failure.indexOf("Workholding setup changed")>=0,id+" rejects an unclamped setup");
   conditions[0].position=expected;
   check(simulation.reset(),id+" recovers after restoring workholding");
   for(i in 0...20)simulation.step();
   conditions[0].position=expected+0.01;
   simulation.step();
   check(player.failure!=null&&player.failure.indexOf("Workholding setup changed")>=0,id+" detects setup changes during execution");
   conditions[0].position=expected;
   check(simulation.reset(),id+" resets after setup change during execution");
   for(i in 0...20)simulation.step();
   player.hold();
   var heldSteps=0;
   while(!player.held()&&player.failure==null&&heldSteps<2000){simulation.step();heldSteps++;}
   simulation.start();
   check(player.held()&&controller.snapshot().machiningStatus=="Held",id+" mission follows actual feed hold: "+player.failure+" / held="+player.held()+" / "+controller.snapshot().machiningStatus);
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
   checkSpeedSlider(simulation,session,fonts,id);
   simulation.stop();simulation.dispose();session.dispose();
   Sys.println(id+" CNC mission completed in "+steps+" steps");
  }
  fonts.dispose();
 }
}
