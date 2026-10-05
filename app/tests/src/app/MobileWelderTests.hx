package app;

import sys.FileSystem;
import robotkit.world.RobotWorld;
import robotkit.mobile.Twist2;

/** The CAD mobile welding carrier loads and moves unchanged on both native backends. */
@:access(app.ApplicationSimulation)
@:access(app.MissionPlayer)
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

  /** External work frames and bead hosts follow their real scene bodies, independent of robot membership. */
  public static function runFrames(root:String):Void {
    var folder = root + "/machinekit/examples/robot-welder/";
    var manifest = FileSystem.fullPath(folder + "materia.mobile.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var seam = MateriaProjectRunner.loadProject(FileSystem.fullPath(folder + "materia.seam.project.json"));
    generated.mission = seam.mission;
    for (backend in [ApplicationSimulation.DETERMINISTIC, ApplicationSimulation.MUJOCO]) {
      var session = new ProjectDocumentSession(null, false);
      session.openGeneratedProject(generated, manifest);
      var simulation = new ApplicationSimulation(new RobotWorld());
      simulation.setBackend(backend);
      try {
        if (!simulation.rebuild(session.sensors, session.scene, session)) throw simulation.error;
        var mission = simulation.missionPlayer();
        var beads = simulation.weldBeads();
        if (mission == null || beads == null) throw "External mobile weld has no mission or bead host";
        var step:materia.project.SceneArtifact.SceneArtifactWeld = cast cast(generated.mission,
          materia.project.SceneArtifact.SceneArtifactMission).steps[0].weld;
        var frame = mission.assemblyParts.get("project:" + step.frame);
        var host = mission.assemblyParts.get("project:" + step.metal);
        if (frame.vertices.length < 12 || host.id != "project:work/weldMetal") throw "External weld lost CAD physical geometry";
        var before = frame.pose();
        var designed = new cadkit.modeling.AssemblyState(cast generated.assemblyDefinition, generated.assemblyState).worldPose(cast step.frame);
        for (axis in 0...3) if (Math.abs(before.position[axis] - [designed.x, designed.y, designed.z][axis] * 0.001) > 1e-6)
          throw "External CAD frame is displaced by its preview centre";
        var object = [for (entry in simulation.simulatedObjects) if (entry.id == frame.id) entry.object][0];
        var physics:robotkit.runtime.Simulation = cast simulation.simulation;
        var body = physics.objectPose(object);
        object.teleport(new nativekit.sim.SimPose(body.x + 0.012, body.y - 0.008, body.z,
          body.qx, body.qy, body.qz, body.qw));
        var after = frame.pose();
        var reference = mission.referenceFrame(step);
        var deposited = beads.referenceOf(0);
        for (axis in 0...3) {
          var expected = before.position[axis] + [0.012, -0.008, 0.0][axis];
          if (Math.abs(after.position[axis] - expected) > 1e-6 || Math.abs(deposited.position[axis] - expected) > 1e-6)
            throw "External bead frame does not follow the live body";
        }
        if (Math.abs(reference.translation.x - after.position[0]) > 1e-6) throw "Weld motion and deposition use different frames";
        var tool = session.robotTools[0];
        var arm = mission.toolArm(tool);
        var positions = mission.robot.robot.snapshot().positions;
        var q = [for (target in arm.toJointTargets([for (_ in 0...arm.dofCount()) 0.0])) positions.get(target.joint)];
        var clear = mission.weldClearance(arm, tool.contact.occurrence, [cast step.metal]);
        var prior = clear.violation(q);
        if (prior != null) throw 'Mobile carrier ready pose is not clear: ${prior.a}/${prior.b}';
        var nozzlePart = mission.assemblyParts.get("project:robot/arm/tool/nozzle");
        var nozzlePose = nozzlePart.pose();
        var rotatedCenter = AssemblyRobot.rotate(nozzlePose.rotation, nozzlePart.center);
        var nozzle = [for (axis in 0...3) nozzlePose.position[axis] + rotatedCenter[axis]];
        object.teleport(new nativekit.sim.SimPose(nozzle[0], nozzle[1], nozzle[2], body.qx, body.qy, body.qz, body.qw));
        var blocked = mission.weldClearance(arm, tool.contact.occurrence, [cast step.metal]).violation(q);
        if (blocked == null || (blocked.a != cast step.frame && blocked.b != cast step.frame))
          throw "External workpiece collision is missing from the weld planner";
        Sys.println('mobile weld frames ($backend): CAD centre restored, 12/-8 mm live displacement follows motion and bead');
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
