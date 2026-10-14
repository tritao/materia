package app;

import sys.FileSystem;
import robotkit.world.RobotWorld;
import robotkit.mobile.Twist2;
import robotkit.skill.Skill;
import robotkit.skill.SkillStatus;

/** The CAD mobile welding carrier loads and moves unchanged on both native backends. */
@:access(app.ApplicationSimulation)
@:access(app.MissionPlayer)
@:access(processkit.skill.FindContactWork)
@:access(processkit.ProbePosePlanner)
@:access(processkit.ContactProbeRunner)
@:access(processkit.perception.ContactRegistrationSequence)
@:access(processkit.simulation.SimulatedWelder)
@:access(machinekit.power.BatterySimulator)
class MobileWelderTests {
  /** Derive a physical normal contact from authored CAD without running the expensive probe selector. */
  public static function runWireClearance(root:String):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/robot-welder/materia.mobilemission.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var authored:materia.project.SceneArtifact.SceneArtifactMission = cast generated.mission;
    var session = new ProjectDocumentSession(null, false);
    session.openGeneratedProject(generated, manifest);
    var simulation = new ApplicationSimulation(new RobotWorld());
    simulation.setBackend(ApplicationSimulation.DETERMINISTIC);
    try {
      if (!simulation.rebuild(session.sensors, session.scene, session)) throw simulation.error;
      var mission:MissionPlayer = cast simulation.missionPlayer();
      while (mission.stepIndex == 0 && simulation.activeSession().simulationTime() < 60) {
        simulation.step();
        if (mission.failure != null) throw mission.failure;
      }
      if (mission.stepIndex != 1) throw "Wire-clearance replay did not reach the parking station";
      simulation.activeSession().stop();
      var physics:robotkit.runtime.Simulation = cast simulation.simulation;
      var base = physics.linkPose(mission.robotIndex, 0);
      var before = new robotkit.spatial.Transform3(robotkit.spatial.Vec3.fromArray(base.position), robotkit.spatial.Quat.fromArray(base.rotation));
      var shifted = before.compose(new robotkit.spatial.Transform3(new robotkit.spatial.Vec3(0.012, -0.009, 0),
        robotkit.spatial.Quat.fromRollPitchYaw(0, 0, 0.025)));
      physics.placeRobotBase(mission.robotIndex, shifted.translation.toArray(), shifted.rotation.toArray());
      mission.robot.robot.stop(robotkit.core.StopMode.Normal);
      var job:materia.project.SceneContactRegistration.SceneContactWork = cast authored.steps[1].contactWork;
      var factory = mission.newRegistration;
      if (factory == null) throw "Wire-clearance replay has no probe factory";
      var registration = factory(job);
      var planner = registration.probe.planner;
      var analytic = new motionkit.robot.EaikKinematics(planner.arm);
      var seed = [0.6676030989851316, -0.21797454761040297, -0.9093220264586326,
        -1.3662865416110088, -0.7644976321991027, 2.464939730522297];
      var seedPose = planner.arm.tcpPose(seed);
      var nominal = job.nominal;
      var workPose = new robotkit.spatial.Transform3(new robotkit.spatial.Vec3(nominal.x, nominal.y, nominal.z),
        new robotkit.spatial.Quat(nominal.qx, nominal.qy, nominal.qz, nominal.qw));
      var rootInverse = shifted.inverse();
      var selected:Null<Array<Float>> = null, nearest = Math.POSITIVE_INFINITY;
      var selectedMember:Null<String> = null;
      // Select by the authored face and reach alone, independently of the verdict under test.
      // The old posture is only an IK seed; the contact point follows current CAD and the moved base.
      for (face in job.faces) if (face.target) {
        var outward = workPose.rotation.rotate(robotkit.spatial.Vec3.fromArray(face.normal));
        var worldPoint = workPose.transformPoint(robotkit.spatial.Vec3.fromArray(face.centre))
          .sub(outward.scale(job.measurementError));
        var point = rootInverse.transformPoint(worldPoint);
        var normal = rootInverse.rotation.rotate(outward);
        var rotation = processkit.ProbePosePlanner.orientations(normal, seedPose.rotation, 1)[0];
        var candidate = analytic.solvePose(motionkit.kinematics.Pose3.fromArrays(point.toArray(), rotation.toArray()),
          seed, new motionkit.kinematics.IkTolerance(job.measurementError / 2, 1e-5));
        var distance = point.sub(seedPose.translation).norm();
        if (candidate != null && distance < nearest) { selected = candidate; selectedMember = face.member; nearest = distance; }
      }
      if (selected == null || selectedMember == null) throw "The authored grounded work has no reachable normal contact posture";
      var q = selected;
      var indices = planner.arm.jointIndices();
      var snapshot = mission.robot.robot.snapshot();
      var positions = [for (index in 0...snapshot.positions.length) snapshot.positions.get(index)];
      for (index in 0...indices.length) positions[indices[index]] = q[index];
      physics.setJointPositions(mission.robotIndex, positions);
      mission.robot.robot.submit(robotkit.core.RobotCommand.JointTargets([for (index in 0...indices.length)
        robotkit.core.JointTarget.position(indices[index], q[index])], null));
      // Step physics and its sensor observers directly; do not start the pending findWork skill.
      for (_ in 0...200) simulation.activeSession().step();
      var wire = planner.wire;
      if (wire == null) throw "Wire-clearance replay lacks the described CAD wire";
      var tip = planner.arm.tcpPose(q).translation;
      var welder:processkit.simulation.SimulatedWelder = cast simulation.welder();
      var tool = physics.linkPose(mission.robotIndex, welder.linkIndex);
      var actualWorld = new robotkit.spatial.Transform3(robotkit.spatial.Vec3.fromArray(tool.position),
        robotkit.spatial.Quat.fromArray(tool.rotation)).transformPoint(robotkit.spatial.Vec3.fromArray(welder.tipPosition));
      var rootBody = physics.linkPose(mission.robotIndex, 0);
      var rootPose = new robotkit.spatial.Transform3(robotkit.spatial.Vec3.fromArray(rootBody.position),
        robotkit.spatial.Quat.fromArray(rootBody.rotation));
      var actual = rootPose.inverse().transformPoint(actualWorld);
      var clearance = planner.clearance;
      if (clearance == null) throw "The wire fixture lost its shared clearance world";
      var wireGap = clearance.distanceBetween(q, processkit.ProbeWire.NAME, selectedMember);
      var diagnostic = {wireGap: wireGap, workMember: selectedMember, joints: q, tip: tip.toArray(), actualSensorTip: actual.toArray(), sensorFkError: tip.sub(actual).norm(),
        touch: welder.reading().touch, groundDistance: welder.work.distance(actualWorld.x, actualWorld.y, actualWorld.z),
        wireObject: wire.object, clearance: planner.violation(q)};
      sys.io.File.saveContent(root + "/app/build/p3-wire-posture.json", haxe.Json.stringify(diagnostic));
      if (!welder.reading().touch || tip.sub(actual).norm() > 0.0001 || wireGap >= 0 || planner.violation(q) == null)
        throw "The CAD-derived physical wire-contact posture must be rejected by the matching CAD wire envelope";
      Sys.println('mobile wire-clearance posture: ${tip.sub(actual).norm()} m sensor/FK error; wire gap $wireGap m; physical touch rejected');
    } catch (error:Dynamic) { simulation.clear(); session.dispose(); throw error; }
    simulation.clear(); session.dispose();
  }
  /** Contacts must recover a parked work frame while wheel odometry retains its pre-jump estimate. */
  public static function runRegistration(root:String, boundary:Bool = false, voltageCutoff:Bool = false, ?onlyBackend:Int):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/robot-welder/materia.mobilemission.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    var authored:materia.project.SceneArtifact.SceneArtifactMission = cast generated.mission;
    if (authored.steps[1].kind != "findWork") throw "Mobile mission lacks contact registration after parking";
    var job:materia.project.SceneContactRegistration.SceneContactWork = cast authored.steps[1].contactWork;
    generated.mission = {steps: authored.steps.slice(0, 3)};
    var backends:Array<Int> = onlyBackend == null ?
      [ApplicationSimulation.DETERMINISTIC, ApplicationSimulation.MUJOCO] : [cast onlyBackend];
    for (backend in backends) {
      var session = new ProjectDocumentSession(null, false);
      session.openGeneratedProject(generated, manifest);
      var simulation = new ApplicationSimulation(new RobotWorld()); simulation.setBackend(backend);
      try {
        if (!simulation.rebuild(session.sensors, session.scene, session)) throw simulation.error;
        var mission:MissionPlayer = cast simulation.missionPlayer();
        var torches = [for (tool in mission.project.robotTools) if (tool.kind == "torch") tool];
        if (torches.length != 1) throw 'Contact registration needs one torch, found ${torches.length}';
        var torchChannel = torches[0].channel;
        var physics:robotkit.runtime.Simulation = cast simulation.simulation;
        var injected = false;
        var cutoffInjected = false, cutoffAt = 0.0;
        var touchEpisodes = 0, touching = false;
        var probeProgress = "";
        var worstTipError = 0.0;
        var welder:processkit.simulation.SimulatedWelder = cast simulation.welder();
        // Calibrated stopping can make near-singular probe branches slower than the requested recipe.
        while (!mission.finished && simulation.activeSession().simulationTime() < 1200 &&
            (!cutoffInjected || simulation.activeSession().simulationTime() < cutoffAt + 60)) {
          if (mission.stepIndex == 1 && !injected) {
            var base = physics.linkPose(mission.robotIndex, 0);
            var old = new robotkit.spatial.Transform3(new robotkit.spatial.Vec3(base.position[0], base.position[1], base.position[2]),
              robotkit.spatial.Quat.fromArray(base.rotation));
            var error = new robotkit.spatial.Transform3(new robotkit.spatial.Vec3(boundary ? 0.020 : voltageCutoff ? 0.006 : 0.012,
              boundary ? -0.020 : voltageCutoff ? -0.007 : -0.009, 0),
              robotkit.spatial.Quat.fromRollPitchYaw(0, 0,
                boundary ? 2 * Math.PI / 180 : voltageCutoff ? 0.75 * Math.PI / 180 : 0.025));
            var shifted = old.compose(error);
            physics.placeRobotBase(mission.robotIndex, shifted.translation.toArray(), shifted.rotation.toArray());
            // placeRobotBase applies on the next owner tick. Publish the displaced plant before planning its clearance world.
            simulation.activeSession().step();
            injected = true;
            Sys.println('mobile contact registration ($backend): injected ${boundary ? "20 mm/2 degree boundary" : "parking error"}');
          }
          simulation.step();
          var failure = mission.failure;
          if (failure != null) {
            if (voltageCutoff && cutoffInjected && failure.indexOf("the arc could not be held in 3 restarts") >= 0) break;
            throw 'Mobile contact registration ($backend) failed: $failure';
          }
          var weldStep = mission.weldingStep();
          if (weldStep >= 0 && welder.reading().arc) {
            if (voltageCutoff && !cutoffInjected) {
              var power = simulation.power;
              if (power == null) throw "Voltage-cutoff scenario has no CAD pack model";
              // Inject a pack-state fault after strike. Its open-circuit voltage lies
              // halfway between the CAD empty voltage and cutoff, with energy still stored.
              power.battery.chargeFraction = 0.5 * (power.profile.cutoffVoltage - power.profile.emptyVoltage) /
                (power.profile.fullVoltage - power.profile.emptyVoltage);
              cutoffInjected = true;
              cutoffAt = simulation.activeSession().simulationTime();
            }
            var weld:materia.project.SceneArtifact.SceneArtifactWeld = cast authored.steps[weldStep].weld;
            var live = mission.assemblyParts.get("project:" + job.frame).pose();
            var work = new robotkit.spatial.Transform3(robotkit.spatial.Vec3.fromArray(live.position),
              robotkit.spatial.Quat.fromArray(live.rotation));
            var tip = mission.toolContact(torchChannel);
            var nearest = Math.POSITIVE_INFINITY;
            for (segment in weld.path) {
              var a = work.transformPoint(robotkit.spatial.Vec3.fromArray(segment.start.position));
              var b = work.transformPoint(robotkit.spatial.Vec3.fromArray(segment.stop.position));
              var edge = b.sub(a), relative = tip.sub(a);
              var square = edge.dot(edge);
              var along = square <= 1e-18 ? 0.0 : Math.max(0.0, Math.min(1.0, relative.dot(edge) / square));
              nearest = Math.min(nearest, tip.sub(a.add(edge.scale(along))).norm());
            }
            worstTipError = Math.max(worstTipError, nearest);
          }
          if (mission.stepIndex == 1) {
            var finding:processkit.skill.FindContactWork = cast mission.runner.activeSkill();
            var registration = finding == null ? null : finding.runner;
            if (registration != null) {
              var sequence = registration.sequence;
              var progress = '${sequence.normals.length + 1}/${sequence.index + 1} ${registration.probe.phase}';
              if (progress != probeProgress) {
                probeProgress = progress;
                var search = registration.probe.search;
                var speed = search == null ? "" : ' at ${search.search.speed} m/s';
                Sys.println('mobile contact registration ($backend): probe $progress at ${simulation.activeSession().simulationTime()} s$speed');
                if (boundary && registration.probe.contact != null && sequence.activeStage() != null) {
                  var stage:processkit.perception.ContactRegistrationSequence.ContactRegistrationStage = cast sequence.activeStage();
                  var body = physics.linkPose(mission.robotIndex, 0);
                  var observedRoot = new robotkit.spatial.Transform3(robotkit.spatial.Vec3.fromArray(body.position),
                    robotkit.spatial.Quat.fromArray(body.rotation));
                  var live = mission.assemblyParts.get("project:" + job.frame).pose();
                  var actualWork = new robotkit.spatial.Transform3(robotkit.spatial.Vec3.fromArray(live.position),
                    robotkit.spatial.Quat.fromArray(live.rotation));
                  var point = actualWork.inverse().transformPoint(observedRoot.transformPoint(cast registration.probe.contact));
                  Sys.println('mobile boundary contact: ${point.toArray().join(",")}; plane residual ${stage.normal.dot(point) - stage.offset} m');
                }
              }
            }
            var reading = welder.reading();
            if (reading.arc) throw "Contact registration establishes an arc";
            if (reading.touch && registration != null && Std.string(registration.probe.phase) == "Approaching") {
              var snapshot = mission.robot.robot.snapshot();
              var planner = registration.probe.planner;
              var joints = [for (index in planner.arm.jointIndices()) snapshot.positions.get(index)];
              var body = physics.linkPose(mission.robotIndex, 0);
              var rootPose = new robotkit.spatial.Transform3(robotkit.spatial.Vec3.fromArray(body.position),
                robotkit.spatial.Quat.fromArray(body.rotation));
              var expectedTip = rootPose.transformPoint(planner.arm.tcpPose(joints).translation);
              var error = expectedTip.sub(mission.toolContact(torchChannel)).norm();
              throw 'Unexpected wire touch during checked air approach: FK error $error m, clearance ${haxe.Json.stringify(planner.violation(joints))}, joints ${joints.join(",")}';
            }
            if (reading.touch && !touching) {
              touchEpisodes++;
              Sys.println('mobile contact registration ($backend): touch $touchEpisodes at ${simulation.activeSession().simulationTime()} s');
            }
            touching = reading.touch;
          }
        }
        if (voltageCutoff) {
          var power = simulation.power;
          var state = power == null ? null : power.batteryState();
          var failure = mission.failure;
          if (!cutoffInjected || touchEpisodes != 12 || power == null || state == null ||
              failure == null || failure.indexOf("the arc could not be held in 3 restarts") < 0 ||
              state.voltageVolts >= power.profile.cutoffVoltage || state.remainingEnergyWattHours == null ||
              state.remainingEnergyWattHours <= 0 || power.battery.supplyAvailable || welder.supplyReady ||
              welder.reading().arc || welder.reading().powerW > 0)
            throw 'Mobile voltage cutoff ($backend) did not shut down the established arc and report its restart failure';
          Sys.println('mobile voltage cutoff ($backend): ${state.voltageVolts} V, ' +
            '${state.remainingEnergyWattHours} Wh remaining; arc off, restart failure observed');
          simulation.clear(); session.dispose();
          continue;
        }
        var registration = mission.registeredWork.get(job.frame);
        var measuredWorld:Null<robotkit.spatial.Transform3> = registration == null ? null : registration.world;
        if (!injected || !mission.finished || measuredWorld == null || registration == null ||
            registration.measurements.length != 6 || !registration.envelope.containsWorkFrame(registration.rootWork))
          throw "Mobile contact registration did not retain its accepted pose, bounds and six typed observations";
        var base = physics.linkPose(mission.robotIndex, 0);
        var root = new robotkit.spatial.Transform3(robotkit.spatial.Vec3.fromArray(base.position), robotkit.spatial.Quat.fromArray(base.rotation));
        var live = mission.assemblyParts.get("project:" + job.frame).pose();
        var actualWorld = new robotkit.spatial.Transform3(robotkit.spatial.Vec3.fromArray(live.position),
          robotkit.spatial.Quat.fromArray(live.rotation));
        var estimated = cast(mission.wheels, robotkit.localization.WheelOdometryLocalization).state();
        if (estimated == null || Math.sqrt(Math.pow(estimated.pose.x - root.translation.x, 2) +
          Math.pow(estimated.pose.y - root.translation.y, 2)) < 0.008)
          throw "Injected parking error was handed to wheel localization";
        if (touchEpisodes != 12) throw 'Registration needs six coarse/fine contact pairs, saw $touchEpisodes touch episodes';
        var error = 0.0;
        var weld:materia.project.SceneArtifact.SceneArtifactWeld = cast authored.steps[2].weld;
        for (segment in weld.path) for (pose in [segment.start, segment.stop]) {
          var point = robotkit.spatial.Vec3.fromArray(pose.position);
          error = Math.max(error, measuredWorld.transformPoint(point).sub(actualWorld.transformPoint(point)).norm());
        }
        if (error > 0.0001) throw 'Mobile measured work frame misses the real seam by $error m';
        if (worstTipError > 0.0015) throw 'Registered weld tip misses the real seam by $worstTipError m';
        var beads:WeldBeads = cast simulation.weldBeads();
        for (entry in beads.beads) if (Math.abs(entry.bead.meanLeg(0.3, 0.7) - entry.weld.legSize) > 0.0005 || entry.bead.gaps() > 0)
          throw "Registered mobile weld has a wrong leg or gap";
        Sys.println('mobile contact registration ($backend): $touchEpisodes touch episodes, ' +
          '${error * 1000} mm seam-frame error, ${worstTipError * 1000} mm tip error; first weld passes');
      } catch (error:Dynamic) { simulation.clear(); session.dispose(); throw error; }
      simulation.clear(); session.dispose();
    }
  }
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
      var physical = session.projectPhysical;
      if (physical == null) throw "Mobile weld scene lost its physical component properties";
      var declaredPack = false;
      for (part in physical.parts) if (part.physicalMassDeclared == true && part.physicalMassKg != null &&
          Math.abs(part.physicalMassKg - 230.0) < 1e-9 && part.physicalInertiaKgM2 != null &&
          part.physicalInertiaKgM2.length == 9 && part.physicalInertiaKgM2[0] > 0 &&
          part.physicalInertiaKgM2[4] > 0 && part.physicalInertiaKgM2[8] > 0) declaredPack = true;
      if (!declaredPack) throw "The mobile scene did not carry the pack's declared mass and CAD inertia into runtime physics";
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
  public static function runMission(root:String, ?onlyBackend:Int, deratedPack:Bool = false):Void {
    var manifest = FileSystem.fullPath(root + "/machinekit/examples/robot-welder/materia.mobilemission.project.json");
    var generated = MateriaProjectRunner.loadProject(manifest);
    if (deratedPack) {
      // Keep the CAD enclosure and ratings intact while injecting a reduced usable-capacity test pack.
      var mobile:materia.project.SceneArtifact.SceneArtifactMobileBase = cast generated.mobileBase;
      var energy:materia.project.SceneArtifact.SceneArtifactMobileEnergy = cast mobile.energy;
      energy.capacityWh = 1800.0;
      energy.initialChargeFraction = 0.25;
    }
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
    var backends:Array<Int> = onlyBackend == null ?
      [ApplicationSimulation.DETERMINISTIC, ApplicationSimulation.MUJOCO] : [cast onlyBackend];
    for (backend in backends) {
      var session = new ProjectDocumentSession(null, false);
      session.openGeneratedProject(generated, manifest);
      var simulation = new ApplicationSimulation(new RobotWorld());
      simulation.setBackend(backend);
      try {
        if (!simulation.rebuild(session.sensors, session.scene, session)) throw simulation.error;
        var mission = simulation.missionPlayer(), beads = simulation.weldBeads(), welder = simulation.welder();
        if (mission == null || beads == null || welder == null) throw "Mobile welding mission is incomplete";
        var power:SimulatedMobilePower = cast simulation.power;
        if (power == null) throw "Mobile welding mission has no CAD pack integration";
        var initialBattery:robotkit.power.BatteryState = cast power.batteryState();
        if (initialBattery == null) throw "Mobile pack did not report its initial state";
        var initialEnergy = initialBattery.remainingEnergyWattHours;
        if (initialEnergy == null) throw "Mobile pack did not report stored energy";
        var busEnergy = 0.0, resistiveEnergy = 0.0, weldEnergy = 0.0;
        var armEnergy = 0.0, driveEnergy = 0.0;
        // Preserve the original mission allowance and the validated per-registration test allowance.
        // Authored stations each execute contact registration. Charging excursions get
        // their own bounded travel/charge allowance below and retain the clamped work frame.
        var registrations = [for (step in authored.steps) if (step.kind == "findWork") step].length;
        var limit = simulation.activeSession().simulationTime() + 1200 * (1 + registrations);
        var tick = 0, stowing = -1, reportedStep = -1, reportedDockTrips = 0;
        var stepStartedAt = simulation.activeSession().simulationTime(), stepDockTrips = 0;
        var reportedProbe = "";
        var registrationDebug = "";
        var lastFinding:Null<processkit.skill.FindContactWork> = null;
        var reportedEnergyPhase = "", reportedChargeMinute = -1;
        var reportedEnergyProgressAt = stepStartedAt;
        var worst = 0.0;
        var arm = mission.toolArm(session.robotTools[0]);
        var armJoints = arm.jointIndices();
        var registrationStart:Null<Array<Float>> = null;
        var stowClearance:Null<robotkit.collision.CollisionClearance> = null;
        while (!mission.finished && simulation.activeSession().simulationTime() < limit) {
          simulation.step(); tick++;
          var battery:robotkit.power.BatteryState = cast power.batteryState();
          if (battery == null || !Math.isFinite(battery.voltageVolts) || !Math.isFinite(battery.currentAmps) ||
              !Math.isFinite(power.packLoadWatts) || power.packLoadWatts < 0)
            throw "Executed mobile power integration has invalid measurements";
          var hours = simulation.timestep / 3600.0;
          busEnergy += power.packLoadWatts * hours;
          resistiveEnergy += battery.currentAmps * battery.currentAmps * power.profile.internalResistanceOhms * hours;
          weldEnergy += power.weldAcWatts * hours;
          armEnergy += power.armMechanicalWatts * hours;
          driveEnergy += power.driveMechanicalWatts * hours;
          if (mission.energyDockTrips > reportedDockTrips) {
            var newTrips = mission.energyDockTrips - reportedDockTrips;
            reportedDockTrips = mission.energyDockTrips;
            limit += 1200 * newTrips;
            Sys.println('mobile welding ($backend): energy dock trip $reportedDockTrips; mission allowance extended to $limit s');
          }
          var energyPhase = mission.energyTransferPhase();
          var chargeMinute = Std.int(mission.energyChargeSeconds / 60);
          if (energyPhase == "to-dock" && reportedEnergyPhase != "to-dock") {
            var positions = mission.robot.robot.snapshot().positions;
            for (joint in armJoints) if (Math.abs(positions.get(joint)) > 0.01)
              throw 'Energy dock trip ${mission.energyDockTrips} left the torch outside its ready posture';
            Sys.println('mobile welding ($backend): torch stowed at the checked ready posture before dock travel');
          }
          if (energyPhase != "idle" && (energyPhase != reportedEnergyPhase ||
              (energyPhase == "charging" && chargeMinute != reportedChargeMinute))) {
            reportedEnergyPhase = energyPhase;
            reportedChargeMinute = chargeMinute;
            Sys.println('mobile welding ($backend): energy phase $energyPhase at ${simulation.activeSession().simulationTime()} s; ' +
              '${mission.energyChargedWattHours} Wh charged in ${mission.energyChargeSeconds} s');
          }
          var energyProgressAt = simulation.activeSession().simulationTime();
          if (energyPhase != "idle" && energyProgressAt - reportedEnergyProgressAt >= 60) {
            reportedEnergyProgressAt = energyProgressAt;
            var state = mission.localization.state();
            var pose = state == null ? "unknown" :
              '(${Math.round(state.pose.x * 1000) / 1000},${Math.round(state.pose.y * 1000) / 1000},${Math.round(state.pose.yaw * 1000) / 1000})';
            Sys.println('mobile welding ($backend): energy progress $energyPhase at ${energyProgressAt} s; ' +
              'nav=${mission.navigating()} replans=${mission.replans()} map=$pose, ' +
              '${mission.energyChargedWattHours} Wh charged in ${mission.energyChargeSeconds} s');
          }
          if (mission.stepIndex != reportedStep) {
            var previousStep = reportedStep;
            reportedStep = mission.stepIndex;
            stepStartedAt = simulation.activeSession().simulationTime();
            stepDockTrips = mission.energyDockTrips;
            var kind = reportedStep < authored.steps.length ? authored.steps[reportedStep].kind : "done";
            Sys.println('mobile welding ($backend): step $reportedStep $kind at ${simulation.activeSession().simulationTime()} s');
            if (kind == "findWork") {
              registrationStart = [for (joint in armJoints) mission.robot.robot.snapshot().positions.get(joint)];
              if (previousStep >= 0 && authored.steps[previousStep].kind == "goTo") {
                var job:materia.project.SceneContactRegistration.SceneContactWork =
                  cast authored.steps[reportedStep].contactWork;
                var target = MissionPlayer.floorPose(authored.steps[previousStep]);
                var localized = mission.localization.state();
                if (localized == null) throw 'Station ${reportedStep} has no measured pose after goTo';
                var positionError = Math.sqrt(Math.pow(localized.pose.x - target.x, 2) +
                  Math.pow(localized.pose.y - target.y, 2));
                var headingError = Math.abs(Math.atan2(Math.sin(localized.pose.yaw - target.yaw),
                  Math.cos(localized.pose.yaw - target.yaw)));
                var positionLimit = Math.min(job.translation[0], job.translation[1]) * 0.5 + 1e-6;
                var headingLimit = job.rotation[2] * 0.5 + 1e-6;
                if (positionError > positionLimit || headingError > headingLimit)
                  throw 'Station ${reportedStep} parked ${positionError} m / ${headingError} rad from its contact-work envelope ' +
                    '(${positionLimit} m / ${headingLimit} rad)';
                Sys.println('mobile welding ($backend): station ${reportedStep} parked within ${positionError} m / ${headingError} rad');
              }
            } else if (kind == "weld" && previousStep >= 0 && authored.steps[previousStep].kind == "findWork") {
              var start:Array<Float> = cast registrationStart;
              var positions = mission.robot.robot.snapshot().positions;
              for (index in 0...armJoints.length) if (Math.abs(positions.get(armJoints[index]) - start[index]) > 0.005)
                throw 'Contact registration changed the planned starting posture at joint ${armJoints[index]}';
              registrationStart = null;
              Sys.println('mobile welding ($backend): contact registration restored its starting posture');
            }
          }
          if (mission.stepIndex < authored.steps.length && authored.steps[mission.stepIndex].kind == "findWork") {
            var finding:processkit.skill.FindContactWork = cast mission.runner.activeSkill();
            if (finding != null) lastFinding = finding;
            if (finding != null && finding.runner != null) {
              var active = finding.runner;
              var progress = '${mission.stepIndex}: ${active.sequence.normals.length + 1}/${active.sequence.index + 1} ${active.probe.phase}';
              if (progress != reportedProbe) {
                reportedProbe = progress;
                Sys.println('mobile welding ($backend): probe $progress at ${simulation.activeSession().simulationTime()} s');
              }
            }
          }
          if (mission.failure != null) {
            var state = mission.localization.state();
            var overlay = mission.overlay();
            var failedStep = mission.stepIndex < authored.steps.length ? authored.steps[mission.stepIndex] : null;
            var goalText = failedStep == null ? "none" : failedStep.kind == "findWork"
              ? "findWork contact registration" : haxe.Json.stringify(failedStep);
            if (failedStep != null && failedStep.kind == "findWork" && lastFinding != null && lastFinding.runner != null) {
              var sequence = lastFinding.runner.sequence;
              var delta = sequence.envelope.nominalWork.inverse().compose(sequence.envelope.estimate());
              var shift = delta.translation, rotation = delta.rotation;
              var joints = [for (joint in armJoints) mission.robot.robot.snapshot().positions.get(joint)];
              var tcp = arm.tcpPose(joints).translation;
              registrationDebug = ' registration=${sequence.contacts.length} contacts/${sequence.normals.length} planes, ' +
                'envelope=${sequence.envelope.cells} cells, nominal shift=(${shift.x},${shift.y},${shift.z}), ' +
                'rotation=(${rotation.x},${rotation.y},${rotation.z},${rotation.w}), tcp=(${tcp.x},${tcp.y},${tcp.z})';
              var stage = sequence.activeStage();
              if (stage != null) {
                var last = stage.probes[stage.probes.length - 1].approachJoints;
                if (last != null) {
                  var error = [for (index in 0...armJoints.length)
                    mission.robot.robot.snapshot().positions.get(armJoints[index]) - last[index]];
                  registrationDebug += ' last-probe joint error=(${error.join(",")})';
                }
              }
            }
            var pose = state == null ? "unknown" :
              '(${state.pose.x}, ${state.pose.y}, ${state.pose.yaw})';
            var odometryPose = overlay == null ? null : overlay.odometry;
            var odometry = odometryPose == null ? "unknown" :
              '(${odometryPose.x}, ${odometryPose.y}, ${odometryPose.yaw})';
            var route = overlay == null ? "[]" :
              '[${[for (point in overlay.route) '(${point.x},${point.y})'].join(", ")}]';
            throw 'Mobile weld ($backend) failed at ${simulation.activeSession().simulationTime()} s: ${mission.failure}; ' +
              'pose=$pose odom=$odometry goal=$goalText$registrationDebug nav=${mission.navigating()} ' +
              'replans=${mission.replans()} route=$route';
          }
          if (deratedPack) {
            var elapsedInStep = simulation.activeSession().simulationTime() - stepStartedAt;
            var stepAllowance = 1200 * (1 + mission.energyDockTrips - stepDockTrips);
            if (elapsedInStep > stepAllowance) {
              var reading = welder.reading();
              throw 'Mobile mission step ${mission.stepIndex} made no progress for $elapsedInStep/$stepAllowance s: ' +
                'energy=${mission.energyTransferPhase()}, nav=${mission.navigating()}, weldStep=${mission.weldingStep()}, ' +
                'arc=${reading.arc}, weldPower=${reading.powerW} W, trips=${mission.energyDockTrips}, ' +
                'charged=${mission.energyChargedWattHours} Wh, charging=${mission.energyChargeSeconds} s';
            }
          }
          var weldStep = mission.weldingStep();
          if (weldStep >= 0 && welder.reading().arc) {
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
            worst = Math.max(worst, nearest);
            if (tick % 5 == 0) {
              var hit = mission.clearanceViolation(nearest <= processkit.WeldPathPlanner.CONTACT_ZONE);
              if (hit != null) throw 'Mobile weld clearance: ${hit.a}/${hit.b}';
            }
          }
          var index = mission.completed;
          if (index < authored.steps.length && authored.steps[index].kind == "stow") {
            if (stowing != index) {
              stowing = index;
              var tool = session.robotTools[0];
              stowClearance = mission.weldClearance(mission.toolArm(tool), tool.contact.occurrence,
                [for (step in authored.steps) if (step.kind == "weld") cast(step.weld,
                  materia.project.SceneArtifact.SceneArtifactWeld).metal]);
            }
            if (tick % 5 == 0) {
              var clear:robotkit.collision.CollisionClearance = cast stowClearance;
              var positions = mission.robot.robot.snapshot().positions;
              var q = [for (joint in mission.clearanceJoints) positions.get(joint)];
              var hit = clear.violation(q);
              if (hit != null) throw 'Mobile stow clearance: ${hit.a}/${hit.b}, ${hit.distance} m < ${hit.required} m; joints=${q.join(",")}; time=${simulation.activeSession().simulationTime()} s';
            }
          }
          if (index < authored.steps.length && authored.steps[index].kind != "weld" && welder.reading().arc)
            throw "Mobile mission moves between welds with the arc established";
        }
        if (!mission.finished || worst > 0.0015) {
          var now = simulation.activeSession().simulationTime();
          throw 'Mobile weld incomplete or off seam ($worst m): finished=${mission.finished}, ' +
            'step=${mission.stepIndex}/${authored.steps.length}, completed=${mission.completed}, time=$now/$limit s, ' +
            'phase=${mission.energyTransferPhase()}, nav=${mission.navigating()}, weldStep=${mission.weldingStep()}, ' +
            'dockTrips=${mission.energyDockTrips}, charged=${mission.energyChargedWattHours} Wh, ' +
            'charging=${mission.energyChargeSeconds} s';
        }
        if (deratedPack) {
          if (mission.energyDockTrips == 0 || mission.energyChargedWattHours <= 0 || mission.energyChargeSeconds <= 0)
            throw 'Derated mobile pack did not dock, charge and resume: ${mission.energyDockTrips} trips, ' +
              '${mission.energyChargedWattHours} Wh, ${mission.energyChargeSeconds} s';
        } else if (mission.energyDockTrips != 0 || mission.energyChargedWattHours != 0) {
          throw 'Full mobile pack made an unnecessary charging trip: ${mission.energyDockTrips} trips, ' +
            '${mission.energyChargedWattHours} Wh';
        }
        var finalBattery:robotkit.power.BatteryState = cast power.batteryState();
        if (finalBattery == null) throw "Mobile pack did not report its final state";
        var finalEnergy = finalBattery.remainingEnergyWattHours;
        if (finalEnergy == null || !(weldEnergy > 0) || !(busEnergy > weldEnergy) || finalEnergy <= 0)
          throw "Complete mobile weld did not integrate its powered work into stored pack energy";
        if (!deratedPack && Math.abs((initialEnergy - finalEnergy) - busEnergy - resistiveEnergy) > 0.002)
          throw 'Full pack energy balance differs from executed bus and resistance losses: ' +
            '${initialEnergy - finalEnergy} Wh stored loss, $busEnergy Wh bus, $resistiveEnergy Wh resistance';
        Sys.println('mobile power ($backend): $busEnergy Wh bus, $resistiveEnergy Wh resistance, ' +
          '$weldEnergy Wh welding AC, $armEnergy Wh arm and $driveEnergy Wh drive mechanical; $finalEnergy Wh remaining');
        var lengths:Array<String> = [], legs:Array<String> = [];
        for (entry in beads.beads) {
          var bead = entry.bead, leg = bead.meanLeg(0.3, 0.7);
          if (Math.abs(leg - entry.weld.legSize) > 0.0005 || Math.abs(bead.extent() - bead.length) > 0.002 || bead.gaps() > 0)
            throw 'Mobile bead ${entry.step}/${entry.segment}: leg ${leg * 1000} mm, extent ${bead.extent() * 1000} mm';
          legs.push(Std.string(Math.round(leg * 10000) / 10));
          lengths.push(Std.string(Math.round(bead.extent() * 10000) / 10));
        }
        Sys.println('mobile welding ($backend): $stations stations, $seamCount seams, ${Math.round(simulation.activeSession().simulationTime() * 10) / 10} s; legs ${legs.join("/")} mm, lengths ${lengths.join("/")} mm, clear stow; energy ${mission.energyDockTrips} dock trips, ${Math.round(mission.energyChargedWattHours * 10) / 10} Wh in ${Math.round(mission.energyChargeSeconds * 10) / 10} s');
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
        if (frame.pieces.length == 0 || host.id != "project:work/weldMetal") throw "External weld lost CAD physical geometry";
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
