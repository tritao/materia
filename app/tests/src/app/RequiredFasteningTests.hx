package app;

import sys.FileSystem;
import robotkit.world.RobotWorld;
import robotkit.tool.ScrewdriverContract;
import materia.project.SceneScrewInvocation;
import materia.project.SceneScrewInvocation.ScrewInvocation;

/** Exhaust an actual fastening's named supplies and verify that required success blocks downstream work. */
class RequiredFasteningTests {
  public static function run(root:String):Void {
    var manifest = FileSystem.fullPath(root+"/machinekit/examples/robot-screwdriver/materia.assembly.project.json");
    for (backend in [ApplicationSimulation.MUJOCO,ApplicationSimulation.DETERMINISTIC]) {
      var generated = MateriaProjectRunner.loadProject(manifest), definition = generated.missions[0];
      var first = SceneScrewInvocation.job(definition.steps[0].screw,definition.steps);
      var recovery = first == null ? null : first.recovery;
      if (first == null || recovery == null) throw "Required-fastening check needs two distinct qualified supplies";
      definition.steps = [{kind:"screw",screw:Required(first)},definition.steps[1]];
      var session = new ProjectDocumentSession(null,false), simulation = new ApplicationSimulation(new RobotWorld());
      simulation.setBackend(backend);
      try {
        session.openGeneratedProject(generated,manifest);
        if (!simulation.rebuild(session.sensors,session.scene,session)) throw 'Required fastening rebuild: ${simulation.error}';
        var mission = simulation.missionPlayer(), clock = simulation.activeSession();
        if (mission == null || clock == null) throw "Required fastening lost its program";
        var original = simulation.environmentObject("project:"+first.screw);
        var spare = simulation.environmentObject("project:"+recovery.replacement.screw);
        if (original == null || spare == null || original == spare) throw "Required fastening needs both distinct physical screw bodies";
        var injected:Array<String> = [];
        while (!mission.finished && mission.failure == null && clock.simulationTime() < 400) {
          simulation.step();
          var tools = @:privateAccess simulation.tools;
          if (tools == null || tools.screwdrivers.length != 1) throw "Required fastening lost its process observer";
          var setup = @:privateAccess tools.screwdrivers[0].setups.get(first.slot);
          if (setup != null) {
            var identity = setup.screw == original ? first.screw : setup.screw == spare ? recovery.replacement.screw : null;
            if (identity != null && injected.indexOf(identity) < 0) {
              var before = setup.work;
              setup.work = () -> {var work = before(); work.damagedLeadIn = true; return work;};
              injected.push(identity);
            }
          }
        }
        if (injected.length != 2 || mission.failure == null || mission.alarmCode != MissionAlarmCode.FasteningFailure ||
            mission.finished || mission.completed != 0 || mission.stepIndex != 0 || mission.screwRuns.length != 1)
          throw 'Required fastening must stop before downstream work: backend $backend, failure ${mission.failure}, completed ${mission.completed}, step ${mission.stepIndex}, reports ${mission.screwRuns.length}, injected ${injected.length}';
        var report = mission.screwRuns[0];
        if (!report.finished || !report.failed || report.fault != null || report.installed() != null || report.attempts.length != 2)
          throw "Required fastening must preserve its exhausted NOK report and both actual attempts";
        for (attempt in report.attempts) if (attempt.result.outcome != ScrewdriverContract.CROSS_THREAD || attempt.result.angle <= 0)
          throw "Each required-fastening NOK must come from a measured cross-thread cycle";
        if (simulation.heldObjectIds().length != 0) throw "Required fastening must finish its qualified rejection before stopping";
        Sys.println('Required fastening passed backend $backend at ${clock.simulationTime()}');
        simulation.dispose(); session.dispose();
      } catch (error:Dynamic) {simulation.dispose(); session.dispose(); throw error;}
    }
  }
}
