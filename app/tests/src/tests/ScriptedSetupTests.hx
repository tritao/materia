package tests;

import app.ApplicationSimulation;
import app.SceneDocumentSession;
import app.ScriptOwnership;
import app.SetupScript;
import app.ScriptedSetup;
import app.SetupScriptRegistry;
import app.examples.TwoRobotSetupScript;
import haxe.Json;
import robotkit.world.RobotWorld;
import robotkit.model.Sensor;
import sys.FileSystem;
import sys.io.File;

private class FailingReloadScript implements SetupScript {
  public function new() {}
  public function reference():String return TwoRobotSetupScript.REFERENCE;
  public function version():Int return 2;
  public function evaluate(output:ScriptedSetup):Void throw "deliberate setup failure";
}

class ScriptedSetupTests {
  static function check(value:Bool,message:String):Void if(!value)throw message;

  public static function main():Int {
    try {run();Sys.println("Scripted setup tests passed");return 0;}
    catch(error:Dynamic){Sys.println("Scripted setup tests failed: "+Std.string(error));return 1;}
  }

  public static function run():Void {
    var session=new SceneDocumentSession();
    var setup=session.openScript(TwoRobotSetupScript.REFERENCE);
    check(setup.sensors.configuredRobotIds().length==2,"script creates two stable robots");
    var lidar:Sensor=setup.sensors.selected();
    check(lidar!=null&&lidar.frame!=null&&lidar.frame.link.id=="arm",
      "script attaches LiDAR to its articulated link");
    var ownership:ScriptOwnership=session.scriptOwnership;
    check(ownership!=null&&ownership.sensorRateOrigin(setup.sensors.robotId,lidar.id)=="script",
      "inspector reports the script value origin");
    ownership.setOverridesEnabled(true);
    ownership.setSensorRate(setup.sensors.robotId,lidar.id,33.0);
    var overridden=session.refreshScriptOverrides();
    var overriddenLidar:Sensor=overridden.sensors.selected();
    check(overriddenLidar.updateRate==33.0&&
      ownership.sensorRateOrigin(overridden.sensors.robotId,overriddenLidar.id)=="override",
      "stable-ID sensor rate override materializes independently of script data");

    var directory="build/scripted-setup-"+Std.random(100000000);
    FileSystem.createDirectory(directory);var path=directory+"/scene.materia";
    session.save(path);var encoded:Dynamic=Json.parse(File.getContent(path));
    var encodedObjects:Array<Dynamic> = Reflect.field(encoded,"objects");
    check(encodedObjects.length==0&&Reflect.field(encoded,"sensors")==null,
      "script document saves a reference and overrides without an editable configuration copy");
    var reopened=new SceneDocumentSession();reopened.open(path);
    var reopenedLidar:Sensor=reopened.sensors.selected();
    check(reopened.scriptOwnership!=null&&reopenedLidar.updateRate==33.0,
      "script reference and override survive reopen");

    var stale=new ScriptOwnership(TwoRobotSetupScript.REFERENCE,{
      reference:TwoRobotSetupScript.REFERENCE,version:1,overridesEnabled:true,
      overrides:[{targetId:"materia/robot/deleted-sensor",property:"updateRate",value:7.0}]
    });
    var staleSetup=stale.materialize();
    check(stale.diagnostics.length==1&&stale.diagnostics[0].indexOf("deleted-sensor")>=0,
      "deleted stable IDs are reported without applying an override elsewhere");
    staleSetup.scene.dispose();staleSetup.sensors.dispose();stale.dispose();

    var world=new RobotWorld(),simulation=new ApplicationSimulation(world);
    simulation.setBackend(setup.backend);simulation.setTimestep(setup.timestep);
    check(simulation.rebuild(session.sensors,session.scene),"script configuration rebuilds headlessly");
    check(simulation.backend==ApplicationSimulation.MUJOCO&&simulation.timestep==0.01,
      "headless instantiation reproduces the script simulation settings");
    simulation.start();var generation=session.generation;
    SetupScriptRegistry.register(TwoRobotSetupScript.REFERENCE,function()return new FailingReloadScript());
    var failed=false;try session.reloadScript() catch(_:Dynamic)failed=true;
    check(failed&&session.generation==generation&&simulation.isRunning(),
      "failed reload preserves both published configuration and running simulation");
    SetupScriptRegistry.register(TwoRobotSetupScript.REFERENCE,function()return new TwoRobotSetupScript());

    simulation.dispose();world.close();reopened.dispose();session.dispose();
    FileSystem.deleteFile(path);FileSystem.deleteDirectory(directory);
  }
}
