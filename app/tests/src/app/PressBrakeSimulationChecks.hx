package app;
import RobotKitRuntime;
import PressBrake;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetMaterial;
import cadkit.sheetmetal.BendRule;
import cadkit.sheetmetal.process.ProcessBendRule;
import machinekit.pressbrake.SimulatedSheet;
import machinekit.pressbrake.planning.PressGeometry;
import machinekit.pressbrake.planning.BendOrientation;
import machinekit.pressbrake.planning.PressSheetPose;
import cadkit.sheetmetal.ManufacturedSheetState;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import cadkit.modeling.AssemblyState;
import robotkit.world.RobotWorld;
import robotkit.core.RobotSnapshot;
import robotkit.runtime.RobotRuntimeCylinderDriveBlueprint;
class PressBrakeSimulationChecks {
  static function check(value:Bool, message:String):Void { if (!value) throw message; }
  static function pressTravelTime(start:Float, end:Float, velocity:Float,
      drive:robotkit.runtime.RobotRuntimeCylinderDriveBlueprint, closing:Bool, dt:Float):Float {
    var rate = nativekit.sim.MujocoSimWorld.velocityResponseRate();
    var decay = Math.exp(-rate * dt);
    var position = start, elapsed = 0.0;
    while ((closing ? position < end : position > end) && elapsed < 30) {
      var target = closing ? (position < drive.changePoint ? drive.approachSpeed : drive.workingSpeed) : -drive.returnSpeed;
      position += target * dt + (velocity - target) * (1 - decay) / rate;
      velocity = target + (velocity - target) * decay;
      elapsed += dt;
    }
    if (elapsed >= 30) throw "Hydraulic timing reference did not converge";
    return elapsed;
  }

  /** A scripted robot request uses real registration switches and measured ram feedback. */
  public static function run(root:String, withSheet:Bool):Void {
    var manifest = root + "/machinekit/examples/press-brake/materia.controller.project.json";
    var generated = MateriaProjectRunner.loadProject(manifest);
    var definition = generated.assemblyDefinition;
    if (definition == null) throw "Controlled press lost its assembly";
    var initial = new AssemblyState(definition, generated.assemblyState);
    var contact = initial.worldConnector("fingerLeft", "contact");
    var stockHeight = contact.z * generated.metresPerUnit;
    var gaugeFace = contact.y * generated.metresPerUnit;
    generated.objects.push({id: "registration-sheet", label: "Rigid registration fixture", type: "rectangle",
      x: 0, y: -1, z: stockHeight + 0.00075, width: 0.7, height: 0.1, depth: 0.0015,
      collisionEnabled: true, dynamicBody: false, mass: 0.7 * 0.1 * 0.0015 * 7850,
      red: 0.3, green: 0.5, blue: 0.6, visible: true});
    var session = new ProjectDocumentSession(null, false);
    var world = new RobotWorld();
    var simulation = new ApplicationSimulation(world);
    try {
      session.openGeneratedProject(generated, manifest);
      simulation.setBackend(ApplicationSimulation.MUJOCO);
      check(simulation.rebuild(session.sensors, session.scene, session), 'controlled press builds: ${simulation.error}');
      var player = simulation.pressBrakePlayer();
      if (player == null) throw "Controlled press lost its controller";
      var robot:AssemblyRobot = @:privateAccess player.robot;
      var gaugeX = -1, gaugeR = -1;
      for (i in 0...robot.model.joints.length) {
        if (robot.model.joints[i].id == "X") gaugeX = i;
        if (robot.model.joints[i].id == "R") gaugeR = i;
      }
      if (gaugeX < 0 || gaugeR < 0) throw "Controlled press lost its backgauge";
      var ticks = 0;
      while ((player.outputs == null || !player.outputs.ready) && player.failure == null && ticks++ < 2000) {
        simulation.step(); Sys.sleep(0.001);
      }
      check(player.failure == null && player.outputs != null && player.outputs.ready,
        'press positions before readiness: ${player.failure}; ticks=$ticks');
      var stock = simulation.environmentObject("registration-sheet");
      if (stock == null) throw "Controlled press lost its registration fixture";
      for (advance in 0...20) {
        var measured = robot.robot.snapshot();
        var tip = gaugeFace + measured.positions.get(gaugeX) - 0.002 + 0.002 * (advance + 1) / 20;
        simulation.stop();
        stock.teleport(new nativekit.sim.SimPose(0, tip - 0.050,
          stockHeight + measured.positions.get(gaugeR) + 0.00075));
        for (_ in 0...2) simulation.step();
      }
      for (_ in 0...100) {
        var measured = robot.robot.snapshot();
        simulation.stop();
        stock.teleport(new nativekit.sim.SimPose(0, gaugeFace + measured.positions.get(gaugeX) - 0.050,
          stockHeight + measured.positions.get(gaugeR) + 0.00075));
        simulation.step();
      }
      var touching = robot.robot.snapshot();
      check(pressSignal(touching, "fingerLeft-contact") == 1 && pressSignal(touching, "fingerRight-contact") == 1,
        'scripted fixture closes both measured registration switches: left=${pressSignal(touching, "fingerLeft-contact")} right=${pressSignal(touching, "fingerRight-contact")} X=${touching.positions.get(gaugeX)} R=${touching.positions.get(gaugeR)}');
      player.setRobotInput(new processkit.pressbrake.PressBrakeRobotRequest(true, false, false, 0, false, true));
      var phases:Array<processkit.pressbrake.PressBrakePhase> = [];
      var phaseTimes:Array<Float> = [], phasePositions:Array<Float> = [], phaseVelocities:Array<Float> = [];
      var ramIndex:Int = @:privateAccess player.ram;
      var clock:nativekit.sim.SimSession = @:privateAccess player.session;
      var press = new PressBrake(), geometry = PressGeometry.of(press);
      var installed = geometry.stations()[0], tooling = geometry.installedStation(installed.id).processTooling();
      var referenceStock = SheetDemoParts.materialA();
      var stockMaterial = new SheetMaterial(referenceStock.id, referenceStock.thickness,
        BendRule.Process(new ProcessBendRule(tooling)), referenceStock.mechanical);
      var blank = SheetDemoParts.p1(stockMaterial), bend = blank.bends()[0];
      var part = withSheet ? SimulatedSheet.atContact(clock, blank, new ManufacturedSheetState(blank.flat()),
        bend.id, tooling, installed, new BendOrientation(true, false, 0), Transform3.identity(), 7850) : null;
      var nativeSimulation:robotkit.runtime.Simulation = @:privateAccess simulation.simulation;
      var bedId = press.pressGeometryDatums().bed;
      var bed = robot.part("project:" + bedId);
      var carrier = nativeSimulation.linkBody(bed.robotIndex, bed.linkIndex);
      var activeSheet = false, physicalSamples = 0;
      var gap = installed.openTipHeight - installed.dieTopHeight;
      var cylinders = robot.blueprint.cylinderDrives;
      if (cylinders.length == 0) throw "Controlled press lost its measured hydraulic drive";
      var hydraulic = cylinders[0];
      ticks = 0;
      while (player.failure == null && ticks++ < 3000) {
        var before = robot.robot.snapshot();
        var measuredDepth = before.positions.get(ramIndex) * 1000 - gap;
        var phase = player.outputs == null ? processkit.pressbrake.PressBrakePhase.Idle : player.outputs.phase;
        var closing = phase == processkit.pressbrake.PressBrakePhase.ClosingToClamp
          || phase == processkit.pressbrake.PressBrakePhase.Withdrawing
          || phase == processkit.pressbrake.PressBrakePhase.Working;
        if (part != null && closing && measuredDepth >= -blank.thickness()) {
          if (!activeSheet) {
            part.beginStroke(bend.id, tooling, installed, new BendOrientation(true, false, 0),
              Transform3.identity(), carrier);
            activeSheet = true;
          }
          part.driveAtDepth(measuredDepth);
          physicalSamples++;
          if (physicalSamples % 100 == 0)
            Sys.println('Physical press sampled depth=$measuredDepth angle=${part.state().angle(bend.id)} phase=$phase');
        }
        simulation.step(); Sys.sleep(0.001);
        if (part != null && activeSheet) {
          var contact = PressSheetPose.manufactured(blank, part.state(), bend.id, tooling, installed,
            new BendOrientation(true, false, 0));
          check(Math.abs(part.measuredAngle(bend.id) - contact.evaluation.bend(bend.id).angle) <= Math.PI / 1800,
            "Physical sheet follows the measured hydraulic depth within 0.1 degrees");
          var expected = contact.wall(bend.wallA).plane.origin;
          check(part.wallFrame(bend.wallA).translation.sub(new Vec3(expected.x * 0.001, expected.y * 0.001,
            expected.z * 0.001)).norm() < 0.00005, "Physical wall remains at its die contact placement");
        }
        var image = player.outputs;
        if (image == null) throw "Press lost its output image";
        if (phases.length == 0 || phases[phases.length - 1] != image.phase) {
          Sys.println('Physical press phase=${image.phase} ram=${image.ramPositionMm} samples=$physicalSamples');
          phases.push(image.phase);
          phaseTimes.push(clock.simulationTime() - clock.fixedTimestep());
          phasePositions.push(image.ramPositionMm * 0.001);
          phaseVelocities.push(before.velocities.get(ramIndex));
          if (image.phase == processkit.pressbrake.PressBrakePhase.Working) {
            var step:materia.project.SceneArtifact.SceneArtifactPressBrakeStep = @:privateAccess player.authored.steps[0];
            check(Math.abs(before.positions.get(gaugeX) * 1000 - step.gaugeXmm - step.withdrawalMm) <= 0.05,
              "working waits for measured backgauge withdrawal");
            check(pressSignal(before, "fingerLeft-contact") == 0 && pressSignal(before, "fingerRight-contact") == 0,
              "physical withdrawal releases both registration switches");
          }
        }
        switch robot.runtime.channelValue(processkit.pressbrake.PressBrakeChannels.RAM_POSITION_MM) {
          case Analog(value): check(Math.abs(value - image.ramPositionMm) < 1e-7, "analog ram output publishes measured millimetres");
          case _: throw "Press ram output lost its analog type";
        }
        check(image.alarm == processkit.pressbrake.PressBrakeAlarm.None,
          'working stroke has no interlock alarm: ${image.alarm}, phase=${image.phase}, ram=${image.ramPositionMm}; ${player.failure}');
        if (image.phase == processkit.pressbrake.PressBrakePhase.Withdrawing)
          player.setRobotInput(new processkit.pressbrake.PressBrakeRobotRequest(true, false, false, 0, true, false));
        if (image.phase == processkit.pressbrake.PressBrakePhase.Bottom) break;
      }
      check(player.failure == null && player.outputs != null && player.outputs.phase == processkit.pressbrake.PressBrakePhase.Bottom,
        'working stroke reaches measured depth: ${player.failure}; phases=$phases');
      check(phases.length == 4 && phases[0] == processkit.pressbrake.PressBrakePhase.ClosingToClamp &&
        phases[1] == processkit.pressbrake.PressBrakePhase.Withdrawing &&
        phases[2] == processkit.pressbrake.PressBrakePhase.Working &&
        phases[3] == processkit.pressbrake.PressBrakePhase.Bottom, "measured closing signal sequence");
      var approachExpected = pressTravelTime(phasePositions[0], phasePositions[1], phaseVelocities[0],
        hydraulic, true, clock.fixedTimestep());
      var approachActual = phaseTimes[1] - phaseTimes[0];
      var workingExpected = pressTravelTime(phasePositions[2], phasePositions[3], phaseVelocities[2],
        hydraulic, true, clock.fixedTimestep());
      var workingActual = phaseTimes[3] - phaseTimes[2];
      check(Math.abs(approachActual / approachExpected - 1) <= 0.05,
        'approach timing agrees with hydraulic phases within 5%: measured=$approachActual reference=$approachExpected');
      check(Math.abs(workingActual / workingExpected - 1) <= 0.05,
        'working timing agrees with hydraulic speed within 5%: measured=$workingActual reference=$workingExpected');
      if (part != null) check(activeSheet && physicalSamples > 10, "Measured hydraulic working stroke must drive the physical sheet");
      if (part != null) for (_ in 0...16) {
        var settledDepth = robot.robot.snapshot().positions.get(ramIndex) * 1000 - gap;
        part.driveAtDepth(settledDepth); simulation.step();
      }
      var released:Null<Float> = null;
      var checkedReleased = false;
      player.setRobotInput(new processkit.pressbrake.PressBrakeRobotRequest(false, false, true, 0, true, false));
      var returnStarted = clock.simulationTime();
      var returnSnapshot = robot.robot.snapshot();
      var returnPosition = returnSnapshot.positions.get(ramIndex);
      var returnTime:Null<Float> = null;
      var homeAcknowledgedAt:Null<Float> = null;
      var firstHomeSwitchAt:Null<Float> = null, firstUnloadedAt:Null<Float> = null;
      var minimumReturnMm = returnPosition * 1000;
      ticks = 0;
      while (player.failure == null && ticks++ < 3000) {
        simulation.step(); Sys.sleep(0.001);
        var image = player.outputs;
        if (image == null) throw "Press lost its return output image";
        var returnElapsed = clock.simulationTime() - clock.fixedTimestep() - returnStarted;
        minimumReturnMm = Math.min(minimumReturnMm, image.ramPositionMm);
        if (firstHomeSwitchAt == null && image.atTop) firstHomeSwitchAt = returnElapsed;
        if (firstUnloadedAt == null && image.decompressed) firstUnloadedAt = returnElapsed;
        if (part != null && released != null && !checkedReleased) {
          check(Math.abs(part.measuredAngle(bend.id) - released) <= Math.PI / 1800,
            "Native springback agrees with the released process angle");
          Sys.println('Physical hydraulic sheet: samples=$physicalSamples released=$released');
          part.dispose(); checkedReleased = true;
        }
        if (part != null && released == null && image.decompressed) released = part.release();
        check(image.alarm == processkit.pressbrake.PressBrakeAlarm.None,
          'return has no alarm: ${image.alarm}; ram=${image.ramPositionMm}, top=${image.atTop}, unloaded=${image.decompressed}; ${player.failure}');
        if (returnTime == null && image.ramPositionMm <= 0)
          returnTime = clock.simulationTime() - clock.fixedTimestep() - returnStarted;
        if (homeAcknowledgedAt == null && image.atTop && image.phase == processkit.pressbrake.PressBrakePhase.Idle)
          homeAcknowledgedAt = clock.simulationTime() - clock.fixedTimestep() - returnStarted;
        if (image.ready) break;
      }
      if (part != null) check(checkedReleased, "The physical sheet releases after measured unloading");
      check(player.failure == null && player.outputs != null && player.outputs.ready && player.outputs.atTop,
        'press returns home and restores gauge readiness: ${player.failure}');
      var returnExpected = pressTravelTime(returnPosition, 0, returnSnapshot.velocities.get(ramIndex),
        hydraulic, false, clock.fixedTimestep());
      check(returnTime != null && Math.abs(returnTime / returnExpected - 1) <= 0.05,
        'decompression and return timing agrees with hydraulic speed within 5%: measured=$returnTime reference=$returnExpected');
      Sys.println('press return feedback: home switch=$firstHomeSwitchAt s, unloaded=$firstUnloadedAt s, minimum ram=$minimumReturnMm mm');
      robot.robot.stop(robotkit.core.StopMode.Emergency);
      simulation.step(); simulation.step();
      check(player.failure != null && player.outputs != null &&
        player.outputs.alarm == processkit.pressbrake.PressBrakeAlarm.RuntimeFailure && !player.outputs.ready,
        "native fault publishes a named press alarm and removes readiness");
      // Teleport establishes the object's reset pose as well as its current pose.
      // Return the scripted fixture off the gauge before resetting the machine.
      simulation.stop();
      stock.teleport(new nativekit.sim.SimPose(0, -1, stockHeight + 0.00075));
      check(simulation.reset() && player.failure == null, "reset clears the hosted press fault");
      ticks = 0;
      while ((player.outputs == null || !player.outputs.ready) && player.failure == null && ticks++ < 2000) {
        simulation.step(); Sys.sleep(0.001);
      }
      check(player.outputs != null && player.outputs.ready,
        'reset repositions the press before a fresh request: ${player.failure}; image=${player.outputs == null ? "missing" : Std.string(player.outputs.phase)}, alarm=${player.outputs == null ? "missing" : Std.string(player.outputs.alarm)}; ticks=$ticks');
      player.setRobotInput(new processkit.pressbrake.PressBrakeRobotRequest(true));
      simulation.step();
      check(player.outputs != null && player.outputs.alarm == processkit.pressbrake.PressBrakeAlarm.StrokeInterlock,
        "an unregistered request latches its interlock cause");
      robot.robot.stop(robotkit.core.StopMode.Emergency);
      simulation.step();
      check(player.failure != null && player.outputs != null &&
        player.outputs.alarm == processkit.pressbrake.PressBrakeAlarm.StrokeInterlock,
        "native safety failure preserves the first PLC diagnosis");
      switch robot.runtime.channelValue(processkit.pressbrake.PressBrakeChannels.ALARM_CODE) {
        case Analog(value): check(value == processkit.pressbrake.PressBrakeChannels.alarmCode(processkit.pressbrake.PressBrakeAlarm.RuntimeFailure),
          "native fault forces the declared runtime-alarm wire image");
        case _: throw "Press alarm code lost its analog type";
      }
      Sys.println('press controller: measured registration, clamp, withdrawal, depth, return and fault passed; phases=$phases; approach=$approachActual/$approachExpected, working=$workingActual/$workingExpected, return=$returnTime/$returnExpected; home switch=$homeAcknowledgedAt s');
      simulation.dispose(); session.dispose();
    } catch (error:Dynamic) { simulation.dispose(); session.dispose(); throw error; }
  }

  static function pressSignal(snapshot:robotkit.core.RobotSnapshot, id:String):Float {
    if (snapshot.faultCode != 0 || snapshot.safety != RobotKitRuntimeConstants.RK_SAFETY_READY)
      throw 'Press runtime failed while observing "$id": fault ${snapshot.faultCode}, safety ${snapshot.safety}';
    for (index in 0...snapshot.sensors.length) {
      var sensor = snapshot.sensors.get(index);
      if (sensor.sensorId == id) return sensor.values.get(0);
    }
    var observed = [for (index in 0...snapshot.sensors.length) snapshot.sensors.get(index).sensorId];
    throw 'Press simulation lost measured switch "$id"; observed ${observed.join(", ")}; fault ${snapshot.faultCode}, safety ${snapshot.safety}, positions ${snapshot.positions.toArray()}';
  }

}
