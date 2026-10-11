package app;

/** Focused acceptance of the physical sheet in the actual hydraulic press host. */
class SheetPressIntegrationTests {
  public static function main():Void {
    var root = Sys.getCwd();
    while (!sys.FileSystem.exists(root + "/cadkit/examples/modeling/materia.project.json")) {
      var parent = haxe.io.Path.directory(root);
      if (parent == root || parent.length == 0) throw "Could not locate Materia repository";
      root = parent;
    }
    Sys.setCwd(root);
    PressBrakeSimulationChecks.run(root, false);
    PressBrakeSimulationChecks.run(root, true);
    var running = motionkit.robot.ProgramPlanner.shutdown();
    if (running != 0) throw "Physical press test left a planner running";
    Sys.println("Physical hydraulic sheet acceptance passed");
  }
}
