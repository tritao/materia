package tests;

import haxe.Int64;
import robotkit.model.RobotModel;
import robotkit.model.Link;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.JointLimits;
import robotkit.model.Frame;
import robotkit.model.Sensor;
import robotkit.model.RobotDriveConfiguration;
import robotkit.model.RobotMobileConfiguration;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.manipulation.ChainTip;
import robotkit.manipulation.KinematicChain;
import robotkit.manipulation.Manipulator;
import robotkit.manipulation.WorkPatchPlanner;
import robotkit.tool.SimulatedSprayer;
import robotkit.process.CartesianTrajectory;
import robotkit.process.ToolpathExecutor;
import robotkit.process.ToolpathExecutionResult;
import robotkit.process.ToolpathExecutionFailure;
import robotkit.work.WorkSurface;
import robotkit.work.Polygon2;
import robotkit.work.Point2;
import robotkit.work.CoverageMap;
import robotkit.work.Provenance;
import robotkit.work.SourceKind;
import robotkit.perception.SimulatedSurfaceScanner;
import robotkit.perception.SurfaceRegistration;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.Simulation;
import robotkit.runtime.HolonomicDrivePlant;
import robotkit.mobile.MobileBase;
import robotkit.mobile.Pose2;
import robotkit.mobile.Twist2;
import robotkit.mobile.Footprint;
import robotkit.localization.SimulationTruthLocalization;
import robotkit.navigation.Navigation;
import robotkit.navigation.Navigator;
import robotkit.navigation.NavigationGoal;
import robotkit.navigation.OccupancyGrid2;
import robotkit.navigation.OccupancyCell;
import robotkit.navigation.Costmap2;
import robotkit.navigation.AStarPlanner;
import robotkit.perception.PerceptionSnapshot;
import robotkit.skill.SkillRunner;
import robotkit.skill.GoTo;
import robotkit.skill.SkillStatus;
import robotkit.world.SimulatedRobot;
import robotkit.world.RecordingRobot;
import robotkit.world.ReplayRobot;
import robotkit.world.McapRobotRecording;
import robotkit.world.McapRecordingReader;
import robotkit.world.RobotDescription;
import robotkit.world.RobotCapabilities;
import robotkit.world.RobotCommand;
import robotkit.world.RobotSnapshot;

/**
 * M9 acceptance test: a simulated wall-finishing robot (omni base + UR-class
 * 6R arm + sprayer + scanner) end to end: design WorkSurface -> scan ->
 * registration -> patch plan -> navigate -> execute -> coverage ->
 * verification, cross-checked against the simulator's own FK, then recorded
 * to MCAP and replayed.
 *
 * The "BIM wall" is built directly as a WorkSurface here rather than through
 * `robotkit/cadbridge` (see ARCHITECTURE.md "Simulated wall-finishing robot
 * (M9)"): robotkit/tests stays part of the CAD-agnostic core, matching
 * `robotkit/haxeon.json`'s own nativekit-only dependency. A separate
 * BIM-wall-to-patch-plan end-to-end check lives in
 * `robotkit/cadbridge/tests`.
 *
 * `runScenario` also drives the same scenario against the MuJoCo backend
 * (`runMuJoCo`/`testSimulatedWallFinishingScenarioMuJoCo`, backend = 1),
 * with coverage/tracking-error acceptance loosened per the plan (coverage
 * >= 97%, TCP tracking error reported rather than held to the default
 * backend's exact-FK precision). `runMuJoCo` is not called from
 * `RobotWorldTests.main()`/`run()` — the standard `robotkit/tests` haxeon
 * project's native build has no MuJoCo support compiled in, so it is
 * exercised only by the separate `robotkit/tests/mujoco` haxeon project;
 * see ARCHITECTURE.md for why that needed its own CMake wrapper.
 */
class WallFinishingScenarioTests {
  static var assertions = 0;
  public static var lastDefaultBackendReport:String = "";
  public static var lastMuJoCoReport:String = "";

  public static function run():Int {
    assertions = 0;
    testSimulatedWallFinishingScenario();
    Sys.println('RobotKit wall-finishing scenario tests passed ($assertions assertions)');
    return assertions;
  }

  /**
   * Entry for `robotkit/tests/mujoco` (a separate haxeon project whose
   * native.cmake source is `robotkit/robotd/native-mujoco`, a thin CMake
   * wrapper that forces `NKSIM_BUILD_MUJOCO=ON` for just that project — see
   * ARCHITECTURE.md "Simulated wall-finishing robot (M9)"). Not called from
   * `RobotWorldTests.main()`/`run()`: the standard `robotkit/tests`
   * project's own native build has MuJoCo backend support compiled out, so
   * `new Simulation(dt, substeps, 1)` there would throw `RK_ERROR_UNSUPPORTED`.
   */
  public static function runMuJoCo():Int {
    assertions = 0;
    testSimulatedWallFinishingScenarioMuJoCo();
    Sys.println('RobotKit wall-finishing MuJoCo scenario tests passed ($assertions assertions)');
    return assertions;
  }

  static function testSimulatedWallFinishingScenario():Void
    runScenario(0, 0.02, 1, true, 0.99);

  static function testSimulatedWallFinishingScenarioMuJoCo():Void
    runScenario(1, 0.01, 2, false, 0.97);

  static function runScenario(backend:Int, physicsTimestep:Float, physicsSubsteps:Int,
      strictFkCrossCheck:Bool, minCoverage:Float):Void {
    var fixture = buildWallFinishingRobotModel();
    var model = fixture.model;
    var manipulator = new Manipulator(model, fixture.chain, fixture.flangeTTcp);
    var linkNames = [for (link in model.links) link.name];
    var jointNames = [for (joint in model.joints) joint.name];
    var blueprint = RobotRuntimeCompiler.compile(model);

    var simulation = new Simulation(physicsTimestep, physicsSubsteps, backend);
    var recordingPath = '/tmp/robotkit-${Sys.getPid()}-wall-finishing-backend$backend.mcap';
    var writer = new McapRobotRecording(recordingPath, 4 * 1024 * 1024);
    var sourceRobot = new SimulatedRobot("wall-finishing-robot", simulation.addRobot(blueprint),
      model.name, linkNames, jointNames);
    var robot = new RecordingRobot(sourceRobot, writer);

    var base = MobileBase.fromBlueprint(robot, blueprint);
    var plant = new HolonomicDrivePlant(simulation, 0, base);
    var localization = new SimulationTruthLocalization(simulation, 0, "map", "base");
    var navigation = new Navigation(base, localization, 0.2, 0.3, 1.0);
    var grid = new OccupancyGrid2(0.1, new Pose2(-2.0, -2.0), 40, 40, "map", OccupancyCell.Free);
    var baseFootprint = base.footprint;
    if (baseFootprint == null) throw "Wall-finishing model did not provide a base footprint";
    var costmap = new Costmap2(grid, baseFootprint.radius);
    var navigator = new Navigator(navigation, new AStarPlanner(costmap), costmap);

    var tick = 0;
    var timestep = physicsTimestep;
    var seed = [0.2, -1.0, 1.3, -0.3, 0.5, 0.0];
    // Hold the arm at its seed configuration from the very first tick.
    // Nothing else commands the arm during navigation between patches, and
    // the default backend's purely kinematic joint placement (M8.5 F4) never
    // drifts an uncommanded joint away from its rest value -- but MuJoCo has
    // real dynamics, so an uncommanded joint free-falls under gravity while
    // the base drives to each patch, eventually exceeding these joints'
    // velocity envelope. A real robot would not let its arm flail while
    // driving between patches either.
    robot.submit(RobotCommand.JointTargets(manipulator.toJointTargets(seed), null));
    var latestSnapshot:RobotSnapshot = plant.step(Int64.ofInt(tick++));
    localization.update(latestSnapshot);

    // -- design WorkSurface (stands in for a BIM wall face; see class doc) --
    // A small exclusion relative to the wall keeps RasterToolpathGenerator's
    // conservative pullback margin around it (documented in ARCHITECTURE.md,
    // robotkit.work) a small fraction of the total allowed area, so the
    // >= 99% coverage target is reachable without an enormous raster.
    var boundary = new Polygon2([
      new Point2(-0.5, -0.15), new Point2(0.5, -0.15),
      new Point2(0.5, 0.15), new Point2(-0.5, 0.15)
    ]);
    var exclusion = new Polygon2([
      new Point2(0.08, -0.01), new Point2(0.1, -0.01),
      new Point2(0.1, 0.01), new Point2(0.08, 0.01)
    ]);
    // Surface-local Y (vertical) maps to map Z. WorkPatchPlanner's own
    // candidate search has no notion of a separate wheeled-base height: the
    // 3D candidate pose it verifies reachable (before `.toPose2()` drops Z
    // for navigation) sits at map Z = translation.z + patchCenterY exactly,
    // and HolonomicDrivePlant keeps the chassis at its authored map Z = 0. With
    // the boundary centered at local Y = 0 (patchCenterY = 0) and this
    // translation's Z = 0, those two heights coincide exactly, so the
    // candidate WorkPatchPlanner verified reachable is the same one the real
    // (ground-level) chassis actually executes from -- no separate "floor"
    // is modelled in this synthetic map frame, so there is no inconsistency
    // in the wall's local Y range spanning both sides of Z = 0.
    var mapTSurfaceRotation = Quat.fromRotationMatrix([0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0]);
    var mapTSurface = new Transform3(new Vec3(-0.47, 0.0, 0.0), mapTSurfaceRotation);
    var design = new WorkSurface("finish-wall", "map", mapTSurface, boundary, [exclusion],
      0.002, "gypsum", new Provenance("bim:wall-1", SourceKind.Design));

    // -- scan --
    var trueOffset = new Transform3(new Vec3(0.0, 0.0, 0.004),
      Quat.fromAxisAngle(new Vec3(0.0, 1.0, 0.0), 0.005));
    var cloud = SimulatedSurfaceScanner.scan(design, trueOffset, 0.002, 0.02, 0.0005, 42,
      latestSnapshot.sourceTimestampNs);
    check(cloud.size() > 100, 'Simulated scan samples the design surface (got ${cloud.size()} points)');

    // -- registration --
    var registration = SurfaceRegistration.register(design, cloud, 42, 500, 0.02);
    check(registration.accepted, 'Registration accepts the injected offset (${registration.rejectionReason})');
    check(Math.abs(registration.translationCorrection - 0.004) < 0.001,
      'Registration recovers the injected 4mm offset (got ${registration.translationCorrection})');
    var registered:WorkSurface = registration.registered;
    // The registered surface's own frame_T_surface (design's transform
    // composed with the recovered correction) is the actual map_T_surface
    // WorkPatchPlanner/ReachabilityChecker/execution must agree on -- not
    // the uncorrected design transform, which differs by the (small)
    // injected registration offset.
    var registeredMapTSurface = registered.frame_T_surface;

    // -- patch plan --
    var plan = WorkPatchPlanner.plan(registered, registeredMapTSurface, manipulator,
      0.35, 0.06, 0.2, 0.03, 0.05, 0.0, 0.4, 0.55, seed, 4, 5, 0.05);
    check(plan.patches.length >= 1, "Wall splits into at least one patch");
    check(plan.fullyPlanned, "Every patch is fully reachable from its chosen base pose");

    var coverage = new CoverageMap(registered, 0.01);
    var maxJointError = 0.0;
    var maxPositionTrackingError = 0.0;
    var trackingErrors:Array<Float> = [];
    var sprayer = new SimulatedSprayer();

    for (patch in plan.patches) {
      // -- navigate --
      // Tight tolerances keep the achieved base pose close to the candidate
      // WorkPatchPlanner verified reachable, since that check used the exact
      // candidate pose with no navigation slack.
      var goal = new NavigationGoal(patch.basePose, "map", 0.003, 0.005);
      var runner = new SkillRunner();
      var goTo = new GoTo(navigator, goal, function(snapshot) {
        latestSnapshot = snapshot;
        localization.update(snapshot);
        return new PerceptionSnapshot();
      });
      var status = runner.start(goTo);
      var navSteps = 0;
      // A fixed tick budget must scale with the timestep so every backend
      // gets the same simulated-time budget to converge (the MuJoCo run
      // uses a smaller physicsTimestep, per the plan's `new Simulation(0.01,
      // 2, 1)`, so it needs proportionally more ticks for the same 40s).
      var maxNavSteps = Std.int(40.0 / timestep);
      while (status == SkillStatus.Running && navSteps < maxNavSteps) {
        var snapshot = plant.step(Int64.ofInt(tick++));
        latestSnapshot = snapshot;
        localization.update(snapshot);
        status = runner.update(snapshot, timestep);
        navSteps++;
      }
      check(status == SkillStatus.Succeeded, 'Base navigates to patch base pose (status $status)');
      // Hold the chassis perfectly still for the arm-only phase: the plant
      // rolls the base by whatever wheel targets the robot applies, so
      // navigation's last small correction must be replaced by an explicit
      // zero twist. Step once so the robot applies it: the runtime keeps only
      // the latest command batch per tick, so the arm's joint targets below
      // would otherwise replace the zero wheel targets before they took effect
      // and leave the wheels turning through the whole raster.
      base.command(new Twist2(0.0, 0.0));
      latestSnapshot = plant.step(Int64.ofInt(tick++));
      localization.update(latestSnapshot);

      // -- execute --
      var localizationState = localization.state();
      if (localizationState == null) throw "Wall-finishing scenario lost localization after navigation";
      var achieved = localizationState.pose;
      var actualBaseTransform = Transform3.fromPose2(achieved, 0.0);
      var baseTWork = actualBaseTransform.inverse().compose(registeredMapTSurface);

      // Solve and move to the toolpath's own first (lead-in) point before
      // timing the raster itself, so ToolpathExecutor's joint-continuity
      // check runs between successive raster samples, not between an
      // arbitrary cold seed and the first point (the seed only needs to
      // converge, not land close to the first waypoint's own solution).
      var firstTarget = baseTWork.compose(patch.toolpath.points[0].work_T_tcp);
      var initialIk = manipulator.solveIkForTcp(firstTarget, seed, 2e-3, 5e-3, 300, 0.03);
      check(initialIk.converged, "Manipulator reaches the patch's first toolpath point from the seed configuration");
      robot.submit(RobotCommand.JointTargets(manipulator.toJointTargets(initialIk.q), null));
      // The default backend applies a position target exactly and instantly
      // (M8.5 F4), so two ticks is plenty; MuJoCo's computed-torque
      // controller (M8.5 F2) is a real critically-damped second-order
      // response (time constant ~1/omega_n ~ 16ms) that needs several time
      // constants to close a potentially multi-radian jump from the held
      // seed configuration to the patch's first reach pose, unlike the small
      // per-sample deltas within the raster itself.
      var settleTicks = backend == 0 ? 2 : 60;
      for (_ in 0...settleTicks) plant.step(Int64.ofInt(tick++));

      // A coarse sample interval keeps the per-sample step count low (each
      // sample costs one IK solve plus simulation ticks, which matters for
      // the MuJoCo run): consecutive samples land well under the sprayer
      // footprint diameter (2 * 0.03m) apart at this feed rate, so coverage
      // stays continuous along a row without a finer trajectory sampling.
      var trajectory = CartesianTrajectory.build(patch.toolpath, 0.3, 0.25);
      var execution:ToolpathExecutionResult = ToolpathExecutor.execute(manipulator, trajectory,
        baseTWork, initialIk.q, 0.6, 2e-3, 5e-3, 300, 0.03);
      check(execution.success, 'Toolpath executes without an unreachable point or discontinuity (${describeFailure(execution)})');

      // As above: MuJoCo's real second-order joint response needs more
      // per-sample settling ticks than the default backend's instant
      // application to keep up with the raster's per-sample joint deltas,
      // especially at row-transition jumps.
      var perSampleTicks = backend == 0 ? 2 : 6;
      var sprayerOn = false;
      for (step in execution.steps) {
        robot.submit(RobotCommand.JointTargets(step.targets, null));
        var snapshot:RobotSnapshot = latestSnapshot;
        for (_ in 0...perSampleTicks) snapshot = plant.step(Int64.ofInt(tick++));
        latestSnapshot = snapshot;

        if (step.processOn != sprayerOn) {
          if (step.processOn) {
            sprayer.setFlow(0.3, snapshot.sourceTimestampNs);
          } else {
            sprayer.setFlow(0.0, snapshot.sourceTimestampNs);
          }
          sprayerOn = step.processOn;
        }

        // Cross-check: the simulator's own flange link pose (never the
        // commanded target) composed with the tool offset must match
        // base pose . FK(reported joints) . flange_T_tcp.
        var jointError = 0.0;
        for (i in 0...step.q.length)
          jointError = Math.max(jointError, Math.abs(snapshot.positions.get(3 + i) - step.q[i]));
        maxJointError = Math.max(maxJointError, jointError);
        var reportedQ = [for (i in 0...6) snapshot.positions.get(3 + i)];
        var wrist3FromSim = simulation.linkPose(0, WRIST3_LINK_INDEX);
        var wrist3FromSimTransform = Transform3.fromArrays(wrist3FromSim.position, wrist3FromSim.rotation);
        var tcpFromSim = wrist3FromSimTransform.compose(fixture.linkTFlange).compose(fixture.flangeTTcp);
        var basePoseNow = simulation.robotPose(0);
        var baseTransformNow = Transform3.fromArrays(basePoseNow.position, basePoseNow.rotation);
        var tcpFromFk = baseTransformNow.compose(manipulator.tcpPose(reportedQ));
        var positionError = tcpFromSim.translation.sub(tcpFromFk.translation).norm();
        trackingErrors.push(positionError);
        maxPositionTrackingError = Math.max(maxPositionTrackingError, positionError);

        if (step.processOn) {
          var tcpWorld = baseTransformNow.compose(manipulator.tcpPose(reportedQ));
          var surfacePoint = registeredMapTSurface.inverse().compose(tcpWorld);
          coverage.markFootprint(new Point2(surfacePoint.translation.x, surfacePoint.translation.y), 0.03);
        }
      }
    }

    if (strictFkCrossCheck) {
      // Default backend applies joint targets exactly (no dynamics), so the
      // simulator's own link pose must match RobotKit's FK to numerical
      // precision -- this is the cross-check the plan requires between
      // SimKit and robotkit.manipulation's forward kinematics.
      check(maxJointError < 1e-3, 'Commanded joint targets are reached within tolerance (max error $maxJointError)');
      check(maxPositionTrackingError < 1e-6,
        'Simulator link pose matches base pose . FK(reported joints) . flange_T_tcp (max error $maxPositionTrackingError)');
    } else {
      // MuJoCo has real per-joint computed-torque dynamics and finite
      // settling time (M8.5 F2), so commanded and observed joint positions
      // never match exactly; a loose sanity bound still catches an actuator
      // regression (e.g. a stalled or diverging joint) without asserting
      // simulator-vs-FK agreement to default-backend precision.
      check(maxJointError < 0.05, 'Commanded joint targets are reached within a loose MuJoCo tolerance (max error $maxJointError)');
      check(maxPositionTrackingError < 0.02,
        'Simulator link pose stays within a loose sanity bound of base pose . FK(reported joints) . flange_T_tcp (max error $maxPositionTrackingError)');
    }

    var coverageFraction = coverage.coverageFraction();
    var exclusionCoverageFraction = coverage.exclusionCoverageFraction();
    check(coverageFraction >= minCoverage, 'Observed coverage reaches the design target (got $coverageFraction, need >= $minCoverage)');
    check(exclusionCoverageFraction < 0.01, 'No process-on footprint touches the excluded opening (got $exclusionCoverageFraction)');
    var rms = 0.0;
    for (value in trackingErrors) rms += value * value;
    rms = trackingErrors.length == 0 ? 0.0 : Math.sqrt(rms / trackingErrors.length);
    var report = 'coverage=$coverageFraction exclusionCoverage=$exclusionCoverageFraction ' +
      'trackingMax=$maxPositionTrackingError trackingRms=$rms jointErrorMax=$maxJointError ' +
      'registrationTranslation=${registration.translationCorrection} ' +
      'registrationRotation=${registration.rotationCorrectionRadians}';
    if (backend == 0) {
      lastDefaultBackendReport = report;
      Sys.println('M9 default-backend scenario: $report');
    } else {
      lastMuJoCoReport = report;
      Sys.println('M9 MuJoCo scenario: $report');
    }

    check(robot.recordingError == null, "recording adapter preserves an error-free wall-finishing run");
    writer.close();
    var recording = McapRecordingReader.load(recordingPath);
    check(recording.commands.length > 3 && recording.snapshots.length > 3,
      "MCAP stores the wall-finishing command batches and observation stream");

    // -- replay: re-run the same command stream against a ReplayRobot --
    var replayDescription = new RobotDescription("wall-finishing-robot", "recorded wall-finishing robot",
      linkNames, jointNames);
    var replayCapabilities = new RobotCapabilities("wall-finishing-robot", jointNames.length, true, true, true, false);
    var replay = new ReplayRobot("wall-finishing-robot", recording, replayDescription, replayCapabilities);
    var replayed = 0;
    for (command in recording.commands) {
      replay.submit(command);
      if (replay.advance()) replayed++;
    }
    check(replay.generatedCommands.commands.length == recording.commands.length,
      "replay reproduces every recorded joint target / mobile base command");
    var commandsMatch = true;
    for (index in 0...recording.commands.length) {
      switch [recording.commands[index], replay.generatedCommands.commands[index]] {
        case [JointTargets(sourceTargets, _), JointTargets(replayTargets, _)]:
          if (sourceTargets.length != replayTargets.length) commandsMatch = false;
          else for (targetIndex in 0...sourceTargets.length)
            if (sourceTargets[targetIndex].joint != replayTargets[targetIndex].joint ||
                Math.abs(sourceTargets[targetIndex].target - replayTargets[targetIndex].target) > 1e-9)
              commandsMatch = false;
        case _: commandsMatch = false;
      }
    }
    check(commandsMatch, "replayed command batches exactly match the recorded wall-finishing run");
    replay.close();

    robot.close();
    simulation.dispose();
    if (sys.FileSystem.exists(recordingPath)) sys.FileSystem.deleteFile(recordingPath);
    if (sys.FileSystem.exists(recordingPath + ".incomplete.status"))
      sys.FileSystem.deleteFile(recordingPath + ".incomplete.status");
  }

  static function describeFailure(result:ToolpathExecutionResult):String {
    if (result.success) return "none";
    return switch result.failure {
      case Unreachable(index, ik): 'unreachable at sample $index (positionError=${ik.positionError})';
      case Discontinuity(index, joint, delta): 'discontinuity at sample $index joint $joint (delta=$delta)';
      case null: "unknown failure";
    };
  }

  // Link order fixed by buildWallFinishingRobotModel's addLink calls:
  // base(0), wheel0(1), wheel1(2), wheel2(3), shoulder_link(4),
  // upper_arm_link(5), forearm_link(6), wrist_1_link(7), wrist_2_link(8),
  // wrist_3_link(9).
  static inline var WRIST3_LINK_INDEX:Int = 9;

  /**
   * Omni base (three wheels at 120-degree intervals) + UR5-style 6R arm
   * (same published DH-equivalent offsets as KinematicsTests/PlacementTests'
   * fixture, attached directly to the base link) + sprayer flange offset +
   * a base-mounted lidar-kind "scanner" sensor.
   */
  static function buildWallFinishingRobotModel():{model:RobotModel, chain:KinematicChain, flangeTTcp:Transform3, linkTFlange:Transform3} {
    var model = new RobotModel("wall-finishing-robot");
    var base = model.addLink(new Link("base", "link/base"));

    var wheelRadius = 0.05;
    var baseRadius = 0.3;
    var wheelAngles = [Math.PI * 0.5, Math.PI * 0.5 + Math.PI * 2.0 / 3.0, Math.PI * 0.5 + Math.PI * 4.0 / 3.0];
    var wheelIds:Array<String> = [];
    for (i in 0...3) {
      var wheel = model.addLink(new Link('wheel$i', 'link/wheel$i'));
      var joint = new Joint('wheel$i-joint', JointType.Continuous, base, wheel, 'joint/wheel$i');
      joint.limits = new JointLimits(-1000.0, 1000.0, 20.0, 1000.0);
      joint.parentFramePosition = [baseRadius * Math.cos(wheelAngles[i]), baseRadius * Math.sin(wheelAngles[i]), 0.0];
      joint.axis = [0.0, 1.0, 0.0];
      model.addJoint(joint);
      wheelIds.push(joint.id);
    }
    model.mobileBase = new RobotMobileConfiguration(
      RobotDriveConfiguration.Holonomic(wheelIds, wheelRadius, baseRadius),
      0.4, 0.6, 0.5, 1.0, 0.5, 0.5);

    var d1 = 0.089159, shoulderOffset = 0.13585, elbowOffset = -0.1197,
      a2 = 0.425, a3 = 0.39225, d4 = 0.10915, d5 = 0.09465, d6 = 0.0823;
    var linkNames = ["shoulder_link", "upper_arm_link", "forearm_link",
      "wrist_1_link", "wrist_2_link", "wrist_3_link"];
    var links = [base].concat([for (name in linkNames) model.addLink(new Link(name, 'link/$name'))]);
    var offsets = [
      new Vec3(0.0, 0.0, d1),
      new Vec3(0.0, shoulderOffset, 0.0),
      new Vec3(0.0, elbowOffset, a2),
      new Vec3(0.0, 0.0, a3),
      new Vec3(0.0, d4, 0.0),
      new Vec3(0.0, 0.0, d5)
    ];
    var axes = [
      [0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [0.0, 1.0, 0.0],
      [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]
    ];
    var jointNames = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"];
    for (i in 0...6) {
      var joint = model.addJoint(new Joint(jointNames[i], JointType.Revolute, links[i], links[i + 1],
        'joint/${jointNames[i]}'));
      joint.parentFramePosition = offsets[i].toArray();
      joint.axis = axes[i];
      joint.limits.lower = -2.0 * Math.PI;
      joint.limits.upper = 2.0 * Math.PI;
      joint.limits.velocity = 0.0;
    }
    var flangeOffset = new Vec3(0.0, d6, 0.0);
    var flange = model.addFrame(new Frame("flange", links[6], "frame/flange"));
    flange.position = flangeOffset.toArray();
    var chain = new KinematicChain(model, base.id, ChainTip.Frame(flange.id));

    var scannerFrame = model.addFrame(new Frame("scanner mount", base, "frame/scanner"));
    scannerFrame.position = [0.3, 0.0, 0.1];
    var scanner = model.addSensor(new Sensor("wall scanner", "lidar", 5.0, "sensor/scanner"));
    scanner.frame = scannerFrame;
    scanner.rayCount = 16;
    scanner.maxRange = 5.0;

    var flangeTTcp = new Transform3(new Vec3(0.0, 0.0, 0.08), robotkit.spatial.Quat.identity());
    var linkTFlange = new Transform3(flangeOffset, robotkit.spatial.Quat.identity());
    return { model: model, chain: chain, flangeTTcp: flangeTTcp, linkTFlange: linkTFlange };
  }

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'assertion failed: $message';
  }
}
