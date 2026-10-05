package app;

import sys.FileSystem;
import robotkit.world.RobotWorld;
import robotkit.mobile.Twist2;

/** The CAD mobile welding carrier loads and moves unchanged on both native backends. */
class MobileWelderTests {
  static function position(simulation:ApplicationSimulation, id:String):Array<Float> {
    for (entry in simulation.capturePresentationSnapshot().environment) if (entry.id == id) return entry.position.copy();
    throw 'Missing mobile welding part $id';
  }
  public static function run(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/robot-welder/materia.mobile.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    for (backend in [ApplicationSimulation.DETERMINISTIC, ApplicationSimulation.MUJOCO]) {
      var session = new ProjectDocumentSession(null, false);
      session.openGeneratedProject(generated, manifest);
      var simulation = new ApplicationSimulation(new RobotWorld());
      simulation.setBackend(backend);
      try {
        if (!simulation.rebuild(session.sensors, session.scene, session)) throw simulation.error;
        var base = simulation.mobileBase();
        if (base == null) throw "Mobile welding carrier has no CAD-derived drive";
        for (_ in 0...20) simulation.step();
        var plate = position(simulation, "project:robot/basePlate");
        var source = position(simulation, "project:robot/source");
        var pack = position(simulation, "project:robot/battery");
        var t0 = simulation.activeSession().simulationTime();
        base.command(new Twist2(0.2, 0));
        var until = t0 + 2 - 1e-9;
        while (simulation.activeSession().simulationTime() < until) simulation.step();
        var travel = simulation.activeSession().simulationTime() - t0;
        var moved = position(simulation, "project:robot/basePlate");
        var dx = moved[0] - plate[0];
        if (Math.abs(dx - 0.2 * travel) > 0.02) throw 'Mobile welder drive moved $dx m in $travel s';
        for (item in [{id: "project:robot/source", start: source}, {id: "project:robot/battery", start: pack}]) {
          var observed = position(simulation, item.id);
          if (Math.abs(observed[0] - item.start[0] - dx) > 1e-3 || Math.abs(observed[2] - item.start[2]) > 1e-3)
            throw 'Carried welding equipment ${item.id} does not follow the chassis';
        }
        base.stop();
        Sys.println('mobile welder ($backend): ${Math.round(dx * 1000)} mm in $travel s; battery and source follow chassis');
      } catch (error:Dynamic) {
        simulation.clear();
        session.dispose();
        throw error;
      }
      simulation.clear();
      session.dispose();
    }
  }
}
