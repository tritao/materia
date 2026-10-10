import app.ApplicationSimulation;
import app.MateriaProjectRunner;
import app.ProjectDocumentSession;
import robotkit.world.RobotWorld;

/** Belt display caching must follow motion and reset without invalidating unrelated belts. */
class BeltDisplayTests {
  static function check(value:Bool, message:String):Void if (!value) throw message;

  public static function main():Int {
    var root = Sys.getCwd();
    while (!sys.FileSystem.exists(root + "/machinekit/examples/gantry-picker/materia.project.json")) {
      var parent = haxe.io.Path.directory(root);
      if (parent == root || parent.length == 0) throw "Could not locate Materia repository";
      root = parent;
    }
    Sys.setCwd(root);
    var manifest = sys.FileSystem.fullPath("machinekit/examples/gantry-picker/materia.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var project = new ProjectDocumentSession(null, false);
    project.openGeneratedProject(generated, manifest);
    var simulation = new ApplicationSimulation(new RobotWorld());
    simulation.setBackend(ApplicationSimulation.MUJOCO);
    try {
      check(simulation.rebuild(project.sensors, project.scene, project), "gantry simulation builds: " + simulation.error);
      simulation.capturePresentationSnapshot();
      var initial = simulation.beltDisplayUpdates();
      var motion = generated.machineMotion;
      if (motion == null) throw "gantry has no belt motion";
      check(initial == motion.belts.length, "the first presentation initializes every belt");
      var revision = project.scene.revision, visualRevision = project.scene.visualRevision;
      simulation.capturePresentationSnapshot();
      check(simulation.beltDisplayUpdates() == initial && project.scene.revision == revision,
        "a repeated paused presentation does not rebuild any belts or publish geometry");
      simulation.start();
      var retained = simulation.capturePresentationSnapshot();
      var retainedPositions = [for (pose in retained.environment) pose.position.copy()];
      var retainedRotations = [for (pose in retained.environment) pose.rotation.copy()];
      simulation.runTicks(100);
      var advanced = simulation.capturePresentationSnapshot();
      check(advanced.environment.length == retained.environment.length, "presentation keeps the same assembly parts");
      for (index in 0...retained.environment.length) {
        var old = retained.environment[index], current = advanced.environment[index];
        check(old.id == current.id && old.position != current.position && old.rotation != current.rotation,
          "each published part pose owns its arrays");
        for (axis in 0...3) check(old.position[axis] == retainedPositions[index][axis],
          "advancing simulation preserves retained snapshot positions");
        for (axis in 0...4) check(old.rotation[axis] == retainedRotations[index][axis],
          "advancing simulation preserves retained snapshot rotations");
      }
      var settled = simulation.beltDisplayUpdates();
      var partial = false;
      for (_ in 0...10) {
        simulation.runTicks(5);
        var before = simulation.beltDisplayUpdates();
        simulation.capturePresentationSnapshot();
        var count = simulation.beltDisplayUpdates() - before;
        if (count > 0 && count < initial) partial = true;
      }
      check(simulation.beltDisplayUpdates() > settled && partial,
        "homing animates the moving belt without rebuilding all the stationary belts");
      check(project.scene.revision == revision && project.scene.visualRevision > visualRevision,
        "belt animation refreshes the viewport without invalidating document-backed panels");
      simulation.stop();
      var stopped = simulation.beltDisplayUpdates();
      simulation.capturePresentationSnapshot(); simulation.capturePresentationSnapshot();
      check(simulation.beltDisplayUpdates() == stopped, "paused belts reuse their mesh");
      check(simulation.reset(), "the simulation resets");
      var reset = simulation.beltDisplayUpdates();
      simulation.capturePresentationSnapshot();
      check(simulation.beltDisplayUpdates() == reset + initial, "reset restores and republishes every belt");
      simulation.dispose(); project.dispose();
    } catch (error:Dynamic) {
      simulation.dispose(); project.dispose(); throw error;
    }
    Sys.println("Belt display tests passed");
    return 0;
  }
}
