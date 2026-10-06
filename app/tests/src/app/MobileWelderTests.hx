package app;

import sys.FileSystem;
import robotkit.world.RobotWorld;
import robotkit.mobile.Twist2;
import robotkit.skill.Skill;
import robotkit.skill.SkillStatus;

/** The CAD mobile welding carrier loads and moves unchanged on both native backends. */
@:access(app.ApplicationSimulation)
@:access(app.MissionPlayer)
class MobileWelderTests {
  static function execute(simulation:ApplicationSimulation, mission:MissionPlayer, skill:Skill):Void {
    skill.start();
    var limit = simulation.activeSession().simulationTime() + 20;
    while (skill.status() == Running && simulation.activeSession().simulationTime() < limit) {
      simulation.step();
      var outcome = skill.update(mission.robot.robot.snapshot(), simulation.timestep);
    }
    if (skill.status() != Succeeded) throw 'Joint owner handoff failed: ${skill.status()}';
  }

  /** A cached joint runner must reacquire its start after another motion owner moves the arm. */
  public static function runJointHandoff(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/robot-welder/materia.mobile.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var tool = generated.robotTools[0];
    var first:materia.project.SceneArtifact.SceneArtifactMissionStep = {kind: "moveJoints", at: tool.contact,
      joints: [{joint: "robot/arm/j1", position: 0.2}]};
    generated.mission = {steps: [first]};
    for (backend in [ApplicationSimulation.DETERMINISTIC, ApplicationSimulation.MUJOCO]) {
      var session = new ProjectDocumentSession(null, false);
      session.openGeneratedProject(generated, manifest);
      var simulation = new ApplicationSimulation(new RobotWorld());
      simulation.setBackend(backend);
      try {
        if (!simulation.rebuild(session.sensors, session.scene, session)) throw simulation.error;
        var mission:MissionPlayer = cast simulation.missionPlayer();
        mission.finished = true; // Drive the skills explicitly so the test can insert a second owner.
        execute(simulation, mission, mission.jointMove(first));
        var arm = mission.toolArm(tool);
        var indices = [for (target in arm.toJointTargets([for (_ in 0...arm.group.count()) 0.0])) target.joint];
        var measured = mission.robot.robot.snapshot().positions;
        if (Math.abs(measured.get(indices[0]) - 0.2) > 0.005) throw "First joint owner missed its target";
        var planning = processkit.WeldingPlanRunner.planning(arm, MissionPlayer.ARM_ACCELERATION);
        var independent = new motionkit.robot.ManipulatorMotion(mission.robot.robot, planning.compiler,
          (_) -> null, () -> mission.robot.runtime.pollEvents(), indices);
        var target = [for (index in indices) measured.get(index)]; target[0] = 0.4;
        execute(simulation, mission, new motionkit.robot.MotionProgramSkill(independent,
          new motionkit.program.MotionProgram([motionkit.program.MotionOp.MoveJ(
            motionkit.program.MoveTarget.JointTarget(target), new motionkit.MotionOptions(), motionkit.program.Blend.ExactStop)])));
        if (Math.abs(mission.robot.robot.snapshot().positions.get(indices[0]) - 0.4) > 0.005)
          throw "Independent joint owner missed its target";
        execute(simulation, mission, mission.jointMove({kind: "moveJoints", at: tool.contact,
          joints: [{joint: "robot/arm/j1", position: 0.0}]}));
        if (Math.abs(mission.robot.robot.snapshot().positions.get(indices[0])) > 0.005)
          throw "Reacquired joint owner missed its target";
        Sys.println('mobile joint handoff ($backend): cached owner 0.2 rad, independent owner 0.4 rad, reacquired owner 0 rad');
      } catch (error:Dynamic) {
        simulation.clear(); session.dispose(); throw error;
      }
      simulation.clear(); session.dispose();
    }
  }

  /** Execute the generated station mission, including driving and every checked stow transition. */
  public static function runMission(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/robot-welder/materia.mobilemission.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var authored:materia.project.SceneArtifact.SceneArtifactMission = cast generated.mission;
    var expected = new Map<String, Bool>();
    var stations = 0;
    for (step in authored.steps) {
      if (step.kind == "goTo") stations++;
      if (step.kind == "weld") {
        var weld:materia.project.SceneArtifact.SceneArtifactWeld = cast step.weld;
        for (segment in weld.path) {
          if (expected.exists(segment.seam)) throw 'Mobile mission repeats ${segment.seam}';
          expected.set(segment.seam, true);
        }
      }
    }
    var seamCount = 0;
    for (_ in expected.keys()) seamCount++;
    if (stations == 0 || seamCount != 10) throw 'Mobile mission has $stations stations and $seamCount seams';
    for (backend in [ApplicationSimulation.DETERMINISTIC, ApplicationSimulation.MUJOCO]) {
      var session = new ProjectDocumentSession(null, false);
      session.openGeneratedProject(generated, manifest);
      var simulation = new ApplicationSimulation(new RobotWorld());
      simulation.setBackend(backend);
      try {
        if (!simulation.rebuild(session.sensors, session.scene, session)) throw simulation.error;
        var mission = simulation.missionPlayer(), beads = simulation.weldBeads(), welder = simulation.welder();
        if (mission == null || beads == null || welder == null) throw "Mobile welding mission is incomplete";
        var limit = simulation.activeSession().simulationTime() + 1200;
        var tick = 0, stowing = -1;
        var worst = 0.0;
        var stowClearance:Null<robotkit.manipulation.ArmClearance> = null;
        while (!mission.finished && simulation.activeSession().simulationTime() < limit) {
          simulation.step(); tick++;
          if (mission.failure != null) {
            var overlay = mission.overlay();
            var step = authored.steps[mission.completed];
            throw 'Mobile weld ($backend) failed at ${simulation.activeSession().simulationTime()} s: ${mission.failure}; goal=${haxe.Json.stringify(step.pose)}, odometry=${haxe.Json.stringify(overlay == null ? null : overlay.odometry)}';
          }
          var weldStep = mission.weldingStep();
          if (weldStep >= 0) {
            var tip = beads.toFrame(weldStep, welder.tip());
            var nearest = Math.POSITIVE_INFINITY;
            for (bead in beads.beadOf(weldStep).beads) {
              var relative = [for (axis in 0...3) tip[axis] - bead.start[axis]];
              var along = 0.0;
              for (axis in 0...3) along += relative[axis] * bead.tangent[axis];
              var point = bead.pointAt(Math.min(bead.length, Math.max(0.0, along)));
              var squared = 0.0;
              for (axis in 0...3) squared += Math.pow(tip[axis] - point[axis], 2);
              nearest = Math.min(nearest, Math.sqrt(squared));
            }
            if (welder.reading().arc) worst = Math.max(worst, nearest);
            if (tick % 5 == 0) {
              var hit = mission.clearanceViolation(nearest <= processkit.WeldPathPlanner.CONTACT_ZONE);
              if (hit != null) throw 'Mobile weld clearance: ${hit.a}/${hit.b}';
            }
          }
          var index = mission.completed;
          if (index < authored.steps.length && authored.steps[index].kind == "moveJoints") {
            if (stowing != index) {
              stowing = index;
              var tool = session.robotTools[0];
              stowClearance = mission.weldClearance(mission.toolArm(tool), tool.contact.occurrence,
                [for (step in authored.steps) if (step.kind == "weld") cast(step.weld,
                  materia.project.SceneArtifact.SceneArtifactWeld).metal]);
            }
            if (tick % 5 == 0) {
              var clear:robotkit.manipulation.ArmClearance = cast stowClearance;
              var positions = mission.robot.robot.snapshot().positions;
              var hit = clear.violation([for (joint in mission.clearanceJoints) positions.get(joint)]);
              if (hit != null) throw 'Mobile stow clearance: ${hit.a}/${hit.b}';
            }
          }
          if (index < authored.steps.length && authored.steps[index].kind != "weld" && welder.reading().arc)
            throw "Mobile mission moves between welds with the arc established";
        }
        if (!mission.finished || worst > 0.0015) throw 'Mobile weld incomplete or off seam ($worst m)';
        var lengths:Array<String> = [], legs:Array<String> = [];
        for (entry in beads.beads) {
          var bead = entry.bead, leg = bead.meanLeg(0.3, 0.7);
          if (Math.abs(leg - entry.weld.legSize) > 0.0005 || Math.abs(bead.extent() - bead.length) > 0.002 || bead.gaps() > 0)
            throw 'Mobile bead ${entry.step}/${entry.segment}: leg ${leg * 1000} mm, extent ${bead.extent() * 1000} mm';
          legs.push(Std.string(Math.round(leg * 10000) / 10));
          lengths.push(Std.string(Math.round(bead.extent() * 10000) / 10));
        }
        Sys.println('mobile welding ($backend): $stations stations, $seamCount seams, ${Math.round(simulation.activeSession().simulationTime() * 10) / 10} s; legs ${legs.join("/")} mm, lengths ${lengths.join("/")} mm, clear stow');
      } catch (error:Dynamic) {
        simulation.clear(); session.dispose(); throw error;
      }
      simulation.clear(); session.dispose();
    }
  }

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
