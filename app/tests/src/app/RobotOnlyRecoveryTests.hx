package app;

import sys.FileSystem;
import robotkit.world.RobotWorld;

/** Local safety recovery must not invent a CNC reset wire in a robot-only project. */
class RobotOnlyRecoveryTests {
  public static function run(root:String):Void {
    var manifest = FileSystem.fullPath(root+"/machinekit/examples/cobot-arm/materia.project.json");
    for (backend in [ApplicationSimulation.MUJOCO,ApplicationSimulation.DETERMINISTIC]) {
      var generated = MateriaProjectRunner.loadProject(manifest);
      // Keep this control regression independent of the demonstration arm's motion route.
      // An unconnected external start remains unknown and cannot complete this wait.
      generated.missions[0].loop = false;
      generated.missions[0].steps = [{kind:"waitFor",waitFor:{name:"operator-start",value:true,timeout:30}}];
      var session = new ProjectDocumentSession(null,false), simulation = new ApplicationSimulation(new RobotWorld());
      simulation.setBackend(backend);
      try {
        session.openGeneratedProject(generated,manifest);
        if (!simulation.rebuild(session.sensors,session.scene,session)) throw 'Robot recovery rebuild: ${simulation.error}';
        var mission = simulation.missionPlayer();
        if (mission == null) throw "Robot recovery needs its controller program";
        simulation.step();
        if (!simulation.emergencyStopRobot(mission.robotId())) throw "Robot-only controller rejected targeted stop";
        for (_ in 0...10) {simulation.step(); if (mission.failure != null) break;}
        if (mission.failure == null || mission.alarmCode != MissionAlarmCode.EmergencyStop)
          throw 'Robot-only stop must fail with the emergency alarm: ${mission.failure}, alarm ${mission.alarmCode}';
        var completed = mission.completed, step = mission.stepIndex;
        if (!simulation.retryMission()) throw "Robot-only mission must accept local retry";
        for (_ in 0...20) {simulation.step(); if (mission.failure == null) break;}
        if (mission.failure != null || mission.completed != completed || mission.stepIndex != step)
          throw 'Robot-only retry must preserve progress without a machine reset: ${mission.failure}';
        for (_ in 0...10) simulation.step();
        if (mission.failure != null) throw 'Robot-only retry must remain ready: ${mission.failure}';
        Sys.println('Robot-only local recovery passed backend $backend');
        simulation.dispose(); session.dispose();
      } catch (error:Dynamic) {simulation.dispose(); session.dispose(); throw error;}
    }
  }
}
