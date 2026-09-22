package app.scripted;

import app.ApplicationSimulation;
import app.ScriptOwnership;
import app.examples.TwoRobotSetupScript;
import haxe.Json;
import robotkit.world.RobotWorld;

/** Headless evaluator/validator/instantiator for the same registered setups Materia opens. */
class Main {
  static function main():Int {
    var args = Sys.args(), verify = args.indexOf("--verify") >= 0, reference = TwoRobotSetupScript.REFERENCE;
    for (arg in args) if (arg != "--verify") reference = arg;
    var ownership:Null<ScriptOwnership> = null, world:Null<RobotWorld> = null;
    var simulation:Null<ApplicationSimulation> = null;
    try {
      ownership = new ScriptOwnership(reference);
      var setup = ownership.materialize();
      if (setup.sensors.configuredRobotIds().length != 2 || setup.scene.records().length != 1 || setup.backend
        != ApplicationSimulation.MUJOCO) throw "Scripted setup does not match the authoritative contract";
      if (verify) {
        var lidar:robotkit.model.Sensor = cast setup.sensors.selected();
        var target = setup.sensors.robotId + "/" + lidar.id;
        ownership.setOverridesEnabled(true);
        ownership.setOverride(target, "noiseStddev", "number", 0.03125);
        ownership.setOverride(target, "mount.position", "vector", [0.5, 0.0, 0.25]);
        ownership.setOverride("moving-obstacle", "dimensions", "vector", [1.0, 1.5, 2.0]);
        ownership.setOverride(ScriptOwnership.SIMULATION_TARGET, "timestep", "number", 0.02);
        setup.scene.dispose();
        setup.sensors.dispose();
        setup = ownership.materialize();
        var overridden:robotkit.model.Sensor = cast setup.sensors.selected();
        var frame:robotkit.model.Frame = cast overridden.frame;
        if (overridden.noiseStddev != 0.03125 || frame.position[2] != 0.25 || setup.scene.records()[0].height != 1.5
          || setup.timestep != 0.02) throw "Typed script overrides do not reproduce headlessly";
      }
      world = new RobotWorld();
      simulation = new ApplicationSimulation(world);
      simulation.setBackend(setup.backend);
      simulation.setTimestep(setup.timestep);
      if (!simulation.rebuild(setup.sensors, setup.scene)) throw simulation.error;
      Sys.println(Json.stringify({
        reference: reference,
        version: ownership.configurationVersion,
        robots: setup.sensors.configuredRobotIds(),
        objects: setup.scene.records().length,
        backend: simulation.userBackendName(),
        timestep: simulation.timestep
      }
      ));
      setup.scene.dispose();
      setup.sensors.dispose();
      simulation.dispose();
      world.close();
      ownership.dispose();
      return 0;
    } catch (error:Dynamic) {
      if (simulation != null) simulation.dispose();
      if (world != null) world.close();
      if (ownership != null) ownership.dispose();
      Sys.stderr().writeString("Scripted setup failed: " + Std.string(error) + "\n");
      return 1;
    }
  }
}
