package tests;

import app.ApplicationSimulation;
import app.SceneDocumentSession;
import app.ScriptOwnership;
import app.SceneCodec;
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
  public function new() {
  }
  public function reference():String return TwoRobotSetupScript.REFERENCE;
  public function version():Int return 2;
  public function evaluate(output:ScriptedSetup):Void throw "deliberate setup failure";
}

class ScriptedSetupTests {
  static function check(value:Bool, message:String):Void if (!value) throw message;

  public static function main():Int {
    try {
      run();
      Sys.println("Scripted setup tests passed");
      return 0;
    } catch (error:Dynamic) {
      Sys.println("Scripted setup tests failed: " + Std.string(error));
      return 1;
    }
  }

  public static function run():Void {
    var session = new SceneDocumentSession();
    var setup = session.openScript(TwoRobotSetupScript.REFERENCE);
    check(setup.sensors.configuredRobotIds().length == 2, "script creates two stable robots");
    var lidar:Sensor = setup.sensors.selected();
    check(
      lidar != null && lidar.frame != null && lidar.frame.link.id == "arm",
      "script attaches LiDAR to its articulated link"
    );
    var ownership:ScriptOwnership = session.scriptOwnership;
    check(
      ownership != null && ownership.sensorRateOrigin(setup.sensors.robotId, lidar.id) == "script",
      "inspector reports the script value origin"
    );
    ownership.setOverridesEnabled(true);
    ownership.setSensorRate(setup.sensors.robotId, lidar.id, 33.0);
    var overridden = session.refreshScriptOverrides();
    var overriddenLidar:Sensor = overridden.sensors.selected();
    check(
      overriddenLidar.updateRate == 33.0 && ownership.sensorRateOrigin(
        overridden.sensors.robotId,
        overriddenLidar.id
      ) == "override",
      "stable-ID sensor rate override materializes independently of script data"
    );
    var sensorTarget = overridden.sensors.robotId + "/" + overriddenLidar.id;
    ownership.setOverride(sensorTarget, "noiseStddev", "number", 0.125);
    ownership.setOverride(sensorTarget, "rayCount", "integer", 17);
    ownership.setOverride(sensorTarget, "maxRange", "number", 8.5);
    ownership.setOverride(sensorTarget, "mount.position", "vector", [0.6, 0.1, 0.2]);
    ownership.setOverride(sensorTarget, "mount.rotation", "vector", [0.0, 0.0, 0.0, 1.0]);
    ownership.setOverride("materia/robot", "position", "vector", [-2.0, 0.5, 0.0]);
    ownership.setOverride("moving-obstacle", "dimensions", "vector", [1.0, 1.5, 2.0]);
    ownership.setOverride("moving-obstacle", "collisionEnabled", "boolean", false);
    ownership.setOverride(ScriptOwnership.SIMULATION_TARGET, "timestep", "number", 0.02);
    ownership.setOverride(ScriptOwnership.SIMULATION_TARGET, "backend", "integer", ApplicationSimulation.DETERMINISTIC);
    overridden.scene.dispose();
    overridden.sensors.dispose();
    overridden = session.refreshScriptOverrides();
    overriddenLidar = overridden.sensors.selected();
    var obstacle = overridden.scene.records()[0];
    var overriddenFrame:robotkit.model.Frame = cast overriddenLidar.frame;
    check(
      overriddenLidar.noiseStddev == 0.125 && overriddenLidar.rayCount == 17
      && overriddenLidar.maxRange == 8.5 && overriddenFrame != null && overriddenFrame.position[0] == 0.6,
      "typed sensor and mount overrides materialize"
    );
    check(overridden.sensors.robotPosition("materia/robot")[0] == -2.0, "stable robot pose override materializes");
    check(
      obstacle.width == 1.0 && obstacle.height == 1.5 && !obstacle.collisionEnabled,
      "typed environment overrides materialize"
    );
    check(
      overridden.backend == ApplicationSimulation.DETERMINISTIC && overridden.timestep == 0.02,
      "typed simulation overrides materialize"
    );
    check(
      ownership.origin(sensorTarget,
      "noiseStddev") == "override" && ownership.origin(sensorTarget, "noiseSeed") == "script",
      "origin is reported per property"
    );

    var directory = "build/scripted-setup-" + Std.random(100000000);
    FileSystem.createDirectory(directory);
    var path = directory + "/scene.materia";
    session.save(path);
    var encoded:Dynamic = Json.parse(File.getContent(path));
    var encodedObjects:Array<Dynamic> = Reflect.field(encoded, "objects");
    check(
      encodedObjects.length == 0 && Reflect.field(encoded, "sensors") == null,
      "script document saves a reference and overrides without an editable configuration copy"
    );
    var reopened = new SceneDocumentSession();
    reopened.open(path);
    var reopenedLidar:Sensor = reopened.sensors.selected();
    check(
      reopened.scriptOwnership != null && reopenedLidar.updateRate == 33.0
      && reopenedLidar.noiseStddev == 0.125 && reopened.scene.records()[0].width == 1.0,
      "typed script overrides survive reopen"
    );

    var legacy = '{"format":"materia.scene","version":1,"objects":[],"sensors":null,"script":{'
      + '"reference":"${TwoRobotSetupScript.REFERENCE}","version":1,"overridesEnabled":true,'
      + '"overrides":[{"targetId":"$sensorTarget","property":"updateRate","value":44.0}]}}';
    var migrated:app.ScriptOwnershipRecord = cast SceneCodec.decodeScript(legacy);
    check(
      migrated.overrideVersion == ScriptOwnership.OVERRIDE_VERSION && migrated.overrides[0].kind == "number",
      "legacy numeric overrides migrate to the versioned typed contract"
    );

    var stale = new ScriptOwnership(TwoRobotSetupScript.REFERENCE, {reference: TwoRobotSetupScript.REFERENCE,
      version: 1, overrideVersion: ScriptOwnership.OVERRIDE_VERSION, overridesEnabled: true, overrides:[{
      targetId: "materia/robot/deleted-sensor",
      property: "updateRate",
      kind: "number",
      encodedValue: "7"
    }
    ]}
    );
    var staleSetup = stale.materialize();
    check(
      stale.diagnostics.length == 1 && stale.diagnostics[0].indexOf("deleted-sensor") >= 0,
      "deleted stable IDs are reported without applying an override elsewhere"
    );
    check(
      stale.removeStaleOverrides() && stale.staleOverrides.length == 0,
      "stale overrides can be explicitly removed with undo history"
    );
    staleSetup.scene.dispose();
    staleSetup.sensors.dispose();
    stale.dispose();

    var world = new RobotWorld(), simulation = new ApplicationSimulation(world);
    ownership.revertTarget(ScriptOwnership.SIMULATION_TARGET);
    session.refreshScriptOverrides();
    simulation.setBackend(ownership.backend());
    simulation.setTimestep(ownership.timestep());
    check(simulation.rebuild(session.sensors, session.scene), "script configuration rebuilds headlessly");
    check(
      simulation.backend == ApplicationSimulation.MUJOCO && simulation.timestep == 0.01,
      "headless instantiation reproduces the script simulation settings"
    );
    simulation.start();
    var generation = session.generation;
    SetupScriptRegistry.register(TwoRobotSetupScript.REFERENCE, function() return new FailingReloadScript());
    var failed = false;
    try session.reloadScript() catch (_:Dynamic) failed = true;
    check(
      failed && session.generation == generation && simulation.isRunning(),
      "failed reload preserves both published configuration and running simulation"
    );
    SetupScriptRegistry.register(TwoRobotSetupScript.REFERENCE, function() return new TwoRobotSetupScript());

    simulation.dispose();
    world.close();
    reopened.dispose();
    session.dispose();
    FileSystem.deleteFile(path);
    FileSystem.deleteDirectory(directory);
  }
}
