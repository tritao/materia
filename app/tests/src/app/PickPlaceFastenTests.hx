package app;

import sys.FileSystem;
import robotkit.world.RobotWorld;
import materia.assembly.AssemblyFrames;

/** Exercises both controller programs against the actual free CAD bodies. */
class PickPlaceFastenTests {
  static function printProgress(message:String):Void {Sys.println(message); Sys.stdout().flush();}
  public static function run(root:String):Void {
    runSelected(root,Sys.getEnv("PICK_PLACE_STAGE"));
  }
  static function runSelected(root:String, selected:Null<String>):Void {
    var supplyFailure = selected == "supply";
    var fixtureFailure = selected == "fixture";
    var interruption = selected == "interruption";
    var stages = selected == "housing" || supplyFailure || interruption ? [true] : selected == "full" || fixtureFailure ? [false] : [true, false];
    var backendChoice = Sys.getEnv("PICK_PLACE_BACKEND");
    var backends = backendChoice == "0" ? [ApplicationSimulation.DETERMINISTIC] :
      backendChoice == "1" ? [ApplicationSimulation.MUJOCO] : [ApplicationSimulation.MUJOCO, ApplicationSimulation.DETERMINISTIC];
    if (selected == "recovery") {
      for (backend in backends) checkHousingRecovery(root,backend);
      return;
    }
    for (housingOnly in stages) for (backend in backends) {
      var manifest = FileSystem.fullPath(root+"/machinekit/examples/robot-screwdriver/"+
        (housingOnly ? "materia.housing-handling.project.json" : "materia.pick-place.project.json"));
      printProgress('Assembly cell: begin backend $backend, housing-only $housingOnly');
      var generated = MateriaProjectRunner.loadProject(manifest);
      var session = new ProjectDocumentSession(null,false), simulation = new ApplicationSimulation(new RobotWorld());
      simulation.setBackend(backend);
      try {
        session.openGeneratedProject(generated,manifest);
        if (!simulation.rebuild(session.sensors,session.scene,session)) throw 'Assembly cell rebuild: ${simulation.error}';
        var handling = simulation.missionPlayer("handlingController"), fastening = simulation.missionPlayer("fasteningController");
        var clock = simulation.activeSession();
        if (handling == null || fastening == null || clock == null) throw "Assembly cell lost its controller programs";
        if (supplyFailure) {
          var first = materia.project.SceneScrewInvocation.job(fastening.mission.steps[1].screw,fastening.mission.steps);
          var reserve = first == null ? null : first.recovery;
          if (first == null || reserve == null) throw "Supply failure needs its identified primary and replacement screws";
          var missing = [first.screw,reserve.replacement.screw];
          for (index in 0...missing.length) {
            var name = missing[index];
            var object = simulation.environmentObject("project:"+name);
            if (object == null) throw 'Supply failure lost screw $name';
            object.setPoseForNextTick(new nativekit.sim.SimPose(3+index*0.1,3,0.3));
          }
        }
        var initialPlate = fastening.assemblyParts.get("project:work/plate");
        printProgress('Assembly plate carrier: environment=${simulation.environmentObject("project:work/plate") != null}, free=${initialPlate.freeObject}, pose=${haxe.Json.stringify(initialPlate.pose())}');
        var observedHandling = -1, observedFastening = -1;
        var nextProgress = 10.0;
        var fixtureFaultInjected = false;
        var fasteningInterrupted = false;
        while ((!handling.finished || !fastening.finished) && handling.failure == null && fastening.failure == null && clock.simulationTime() < 600) {
          if (interruption && !fasteningInterrupted && fastening.stepIndex == 1) {
            var tools = @:privateAccess simulation.tools;
            if (tools == null || tools.screwdrivers.length != 1) throw "Fastening interruption lost its tool observer";
            var measured = tools.screwdrivers[0].reading();
            if (measured.angle > 0.2 && measured.result == 0) {
              if (!simulation.emergencyStopRobot(fastening.robotId())) throw "Fastening controller rejected targeted stop";
              for (_ in 0...20) {simulation.step(); if (fastening.failure != null) break;}
              if (fastening.failure == null || fastening.alarmCode != MissionAlarmCode.EmergencyStop ||
                  fastening.completed != 1 || handling.failure != null)
                throw "Interrupted fastening must stop only its controller before completing the screw";
              if (!simulation.retryMission()) throw "Interrupted fastening must accept local retry";
              for (_ in 0...20) {simulation.step(); if (fastening.failure == null) break;}
              if (fastening.failure != null || fastening.completed != 1)
                throw 'Interrupted fastening must retry its owning step: ${fastening.failure}';
              fasteningInterrupted = true;
              printProgress('Assembly fastening interrupted and retried at ${clock.simulationTime()}');
            }
          }
          if (fixtureFailure && !fixtureFaultInjected && handling.stepIndex == 3 &&
              simulation.heldObjectIds().indexOf("project:work/plate") < 0) {
            var plate = simulation.environmentObject("project:work/plate");
            var pose = plate == null ? null : initialPlate.pose();
            // Remove the released plate while the place skill retreats, before its fixture wait.
            if (plate != null && pose != null && Math.abs(pose.position[0]) < 0.001 && Math.abs(pose.position[1]+0.5) < 0.001 &&
                Math.abs(pose.position[2]-0.25) < 0.001) {
              plate.setPoseForNextTick(new nativekit.sim.SimPose(3,3,0.3));
              fixtureFaultInjected = true;
              printProgress('Assembly fixture plate removed at ${clock.simulationTime()}');
            }
          }
          if (observedHandling != handling.stepIndex || observedFastening != fastening.stepIndex) {
            observedHandling = handling.stepIndex; observedFastening = fastening.stepIndex;
            printProgress('Assembly cell transition H=$observedHandling F=$observedFastening at ${clock.simulationTime()}');
          }
          simulation.step();
          if (clock.simulationTime() >= nextProgress) {
            printProgress('Assembly cell running H=${handling.stepIndex} F=${fastening.stepIndex} at ${clock.simulationTime()}');
            var tools = @:privateAccess simulation.tools;
            if (tools != null) for (driver in tools.screwdrivers)
              printProgress('Assembly tool feedback: ${haxe.Json.stringify(driver.reading())}');
            nextProgress += 10;
          }
        }
        if (fixtureFailure) {
          if (!fixtureFaultInjected || handling.failure == null || handling.stepIndex != 4 ||
              fastening.failure != null || fastening.completed != 0 || fastening.screwRuns.length != 0)
            throw 'Unready fixture must stop at measured fixture presence before part-ready: ${handling.failure}';
          printProgress('Assembly fixture-not-ready passed backend $backend');
          simulation.dispose(); session.dispose();
          continue;
        }
        if (supplyFailure) {
          if (handling.failure != null || fastening.failure == null || fastening.stepIndex != 1 ||
              fastening.completed != 1 || handling.finished || fastening.finished ||
              [for (report in fastening.screwRuns) if (report.installed() != null) report].length != 0)
            throw 'Missing screw supply must stop before accepted fastening: H=${handling.failure}, F=${fastening.failure}, step=${fastening.stepIndex}';
          var fastened = @:privateAccess handling.missionInput("fastened");
          if (fastened == null || !(switch fastened {case Digital(value):!value; case _:false;}))
            throw "Missing screw supply must never assert the wired fastened output";
          printProgress('Assembly screw-supply failure passed backend $backend');
          simulation.dispose(); session.dispose();
          continue;
        }
        if (!handling.finished || !fastening.finished || handling.failure != null || fastening.failure != null) {
          var tools = @:privateAccess simulation.tools;
          if (tools != null) for (driver in tools.screwdrivers)
            printProgress('Assembly failure screwdriver: ${haxe.Json.stringify(driver.reading())}');
          for (name in ["work/plate","work/flange/housing"])
            printProgress('Assembly failure body $name: ${haxe.Json.stringify(handling.assemblyParts.get("project:"+name).pose())}');
          for (frame in handling.robot.runtime.snapshot().sensors.toArray())
            if (StringTools.contains(frame.sensorId,"gripper"))
              printProgress('Assembly failure sensor ${frame.sensorId}: ${[for (i in 0...frame.values.length) frame.values.get(i)]}');
          printProgress('Assembly failure contacts: ${haxe.Json.stringify(handling.simulation.robotContacts(handling.robot.runtime))}');
          throw 'Assembly cell incomplete on backend $backend at ${clock.simulationTime()}: handling step ${handling.stepIndex} ${handling.failure}; fastening step ${fastening.stepIndex} ${fastening.failure}';
        }
        if (fastening.screwRuns.length != 4 || [for (report in fastening.screwRuns) if (!report.finished || report.failed || report.installed() == null) report].length > 0)
          throw "Assembly cell needs four accepted, identified installed screws";
        if (interruption && !fasteningInterrupted) throw "Interruption check must stop an actually rotating screw";
        var plate = fastening.assemblyParts.get("project:work/plate");
        var housing = simulation.environmentObject("project:work/flange/housing");
        if (housing == null || !plate.carries(housing)) throw "The completed housing must be retained on its named plate";
        for (report in fastening.screwRuns) {
          var screw = simulation.environmentObject("project:"+report.installed());
          if (screw == null || !plate.carries(screw)) throw "Every installed screw must travel with the plate";
        }
        if (!housingOnly) {
          var actual = plate.pose();
          var seat = handling.assemblyParts.get("project:outfeedTray/seat-r0-c0-x-1-y-1");
          var frame = seat.pose();
          var target = AssemblyFrames.compose({x:frame.position[0], y:frame.position[1], z:frame.position[2],
            qx:frame.rotation[0], qy:frame.rotation[1], qz:frame.rotation[2], qw:frame.rotation[3]},
            handling.assemblyParts.connectorFrame(seat.id,"seat"));
          var dx = actual.position[0]-target.x, dy = actual.position[1]-target.y, dz = actual.position[2]-target.z;
          if (Math.sqrt(dx*dx+dy*dy+dz*dz) > 0.001) throw "The completed assembly must occupy the measured outfeed pocket";
        }
        printProgress('Assembly cell passed backend $backend, housing-only $housingOnly at ${clock.simulationTime()}');
        simulation.dispose(); session.dispose();
      } catch (error:Dynamic) {
        simulation.dispose(); session.dispose(); throw error;
      }
    }
    if (selected == null) {
      for (backend in backends) checkHousingRecovery(root,backend);
      runSelected(root,"supply");
      runSelected(root,"fixture");
      runSelected(root,"interruption");
    }
  }

  /** Recover the actual controller and measured pocket wait, without a machine-reset output. */
  static function checkHousingRecovery(root:String, backend:Int):Void {
    var manifest = FileSystem.fullPath(root+"/machinekit/examples/robot-screwdriver/materia.housing-handling.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var session = new ProjectDocumentSession(null,false), simulation = new ApplicationSimulation(new RobotWorld());
    simulation.setBackend(backend);
    try {
      session.openGeneratedProject(generated,manifest);
      if (!simulation.rebuild(session.sensors,session.scene,session)) throw 'Housing recovery rebuild: ${simulation.error}';
      var handling = simulation.missionPlayer("handlingController"), fastening = simulation.missionPlayer("fasteningController");
      var clock = simulation.activeSession(), housing = simulation.environmentObject("project:work/flange/housing");
      if (handling == null || fastening == null || clock == null || housing == null)
        throw "Housing recovery lost its controllers or identified housing";
      var original = handling.simulation.objectPose(housing);
      housing.setPoseForNextTick(new nativekit.sim.SimPose(3,3,0.3));
      simulation.step();
      if (!simulation.emergencyStopRobot(handling.robotId())) throw "Handling controller rejected targeted stop";
      for (_ in 0...10) {simulation.step(); if (handling.failure != null) break;}
      if (handling.failure == null || handling.alarmCode != MissionAlarmCode.EmergencyStop || fastening.failure != null)
        throw "Targeted handling stop must fail only its own program";
      var completed = handling.completed;
      if (!simulation.retryMission()) throw "Robot-only handling stop must accept local retry";
      for (_ in 0...20) {simulation.step(); if (handling.failure == null) break;}
      if (handling.failure != null || handling.completed != completed)
        throw 'Local handling retry must preserve progress: ${handling.failure}';
      var until = clock.simulationTime()+15;
      while (handling.failure == null && clock.simulationTime() < until) simulation.step();
      if (handling.failure == null || handling.stepIndex != 1 || handling.completed != 1 || fastening.completed != 0)
        throw 'Missing housing must stop at its measured pocket wait: ${handling.failure}, step ${handling.stepIndex}';
      housing.setPoseForNextTick(original);
      if (!simulation.retryMission()) throw "Restocked housing wait must accept retry";
      until = clock.simulationTime()+2;
      while ((handling.failure != null || handling.stepIndex < 2) && clock.simulationTime() < until) simulation.step();
      if (handling.failure != null || handling.stepIndex != 2 || fastening.failure != null || fastening.completed != 0)
        throw 'Restocked pocket must resume handling before part-ready: ${handling.failure}';
      printProgress('Housing missing-part and local-retry checks passed backend $backend');
      simulation.dispose(); session.dispose();
    } catch (error:Dynamic) {simulation.dispose(); session.dispose(); throw error;}
  }
}
