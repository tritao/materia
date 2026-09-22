package app.scripted;

import app.ApplicationSimulation;
import app.ScriptOwnership;
import app.examples.TwoRobotSetupScript;
import haxe.Json;
import robotkit.world.RobotWorld;

/** Headless evaluator/validator/instantiator for the same registered setups Materia opens. */
class Main {
  static function main():Int {
    var args=Sys.args(),reference=args.length==0?TwoRobotSetupScript.REFERENCE:args[0];
    var ownership:Null<ScriptOwnership>=null,world:Null<RobotWorld>=null;
    var simulation:Null<ApplicationSimulation>=null;
    try {
      ownership=new ScriptOwnership(reference);var setup=ownership.materialize();
      world=new RobotWorld();simulation=new ApplicationSimulation(world);
      simulation.setBackend(setup.backend);simulation.setTimestep(setup.timestep);
      if(!simulation.rebuild(setup.sensors,setup.scene))throw simulation.error;
      Sys.println(Json.stringify({reference:reference,version:ownership.configurationVersion,
        robots:setup.sensors.configuredRobotIds(),objects:setup.scene.records().length,
        backend:simulation.userBackendName(),timestep:simulation.timestep}));
      setup.scene.dispose();setup.sensors.dispose();simulation.dispose();world.close();ownership.dispose();
      return 0;
    } catch(error:Dynamic) {
      if(simulation!=null)simulation.dispose();if(world!=null)world.close();if(ownership!=null)ownership.dispose();
      Sys.stderr().writeString("Scripted setup failed: "+Std.string(error)+"\n");return 1;
    }
  }
}
