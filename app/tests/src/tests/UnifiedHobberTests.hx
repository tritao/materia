package tests;

import app.ApplicationSimulation;
import app.ProjectArtifactLoader;
import app.ProjectDocumentSession;
import materia.project.SceneArtifact;
import robotkit.world.RobotWorld;

/** One installed machine, measured guard feedback, real stock, three restartable scenarios. */
class UnifiedHobberTests {
  static var checks=0;
  static function check(ok:Bool,message:String):Void {checks++;if(!ok)throw message;}
  public static function main():Int {
    try {run();Sys.println('Unified hobber: $checks checks passed');return 0;}
    catch(e:Dynamic){Sys.println('Unified hobber failed: $e');return 1;}
  }
  static function run():Void {
    var machine=new FlexsplineHobber(new gearkit.GearSpec(1,40,4,20*Math.PI/180),4);
    var fit=HobberEnclosurePreview.assembled(machine);
    var data=HobberEnclosurePreview.runtimeScene(fit.assembly,machine,fit.shell.layout.stroke,true);
    var initial=new cadkit.modeling.AssemblyState(data.assemblyDefinition);
    initial.setJoint("enclosure/door/slide",fit.shell.layout.stroke);data.assemblyState=initial.record();
    var loader=new ProjectArtifactLoader();
    var loaded=loader.load(SceneArtifact.encode(data));
    var project=new ProjectDocumentSession(null,false);project.openGeneratedProject(loaded,"unified-hobber");
    check(project.hobbingScenario=="cutting","Default scenario is Cutting");
    var simulation=new ApplicationSimulation(new RobotWorld());simulation.setBackend(ApplicationSimulation.MUJOCO);
    try {
      check(simulation.rebuild(project.sensors,project.scene,project),"Build unified scene: "+simulation.error);
      var player=simulation.hobbingPlayer(),guard=simulation.enclosureController();
      if(player==null||guard==null)throw "Missing shared process/guard";
      var stages:Map<String,Bool> = [];
      function complete(label:String):Void {
        var ticks=0;
        while(!player.completed&&player.failure==null&&!guard.status().fault&&ticks++<500000) {
          simulation.step();stages.set(player.scenarioStage,true);
          var s=guard.status();
          if(player.cutStarted&&!player.cutCompleted)check(s.closed&&s.locked,"Cutting always requires measured closed/locked feedback");
          if(player.scenarioStage=="loading"||player.scenarioStage=="unloading")check(s.open&&!s.locked&&s.axesStandstill&&s.spindleStandstill,"Transfer requires measured open/unlocked and standstill");
        }
        Sys.println(label+" "+haxe.Json.stringify({stage:player.scenarioStage,completed:player.completed,failure:player.failure,guard:guard.status().message,ticks:ticks,windows:player.stock.stream.passes}));
        check(player.failure==null&&!guard.status().fault,label+" must not fault");
        check(player.completed,label+" completes within 125 simulated seconds");
        check(player.stock.forces.maxTangential>0,label+" removes real stock");
      }
      complete("Cutting from initially open doors");
      check(stages.exists("securing")&&stages.exists("cutting"),"Automatic cut closes the initial open door before cutting");
      simulation.selectHobbingScenario(project,"idle");
      for(i in 0...3000)simulation.step();
      check(!player.cutStarted&&player.stock.stream.passes==0,"Idle never executes the cutting program");
      check(!guard.status().canStart,"Idle disables manual cutting");
      simulation.selectHobbingScenario(project,"loading");stages.clear();
      complete("Load, cut and unload");
      for(stage in ["parking","loading","cutting","unloading"])check(stages.exists(stage),"Full cycle includes "+stage);
      check(player.headRetracted&&player.cutCompleted&&guard.status().open,"Full cycle retracts and opens before unloading");
      check(player.sleeveMeshUploads==1,"Both scenarios share one cached sleeve mesh");
      simulation.selectHobbingScenario(project,"cutting");guard.setBoundaryClear(false);
      for(i in 0...2000)simulation.step();
      check(guard.status().fault&&!player.cutStarted,"Scenario cannot bypass a guard boundary fault");
      simulation.dispose();project.dispose();
    }catch(e:Dynamic){simulation.dispose();project.dispose();throw e;}
  }
}
