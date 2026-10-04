package tests;

import haxe.Int64;
import motionkit.robot.SurfacePlanRunner;
import robotkit.model.RobotModel;
import robotkit.model.Link;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.JointLimits;
import robotkit.model.Frame;
import robotkit.profile.RobotMobileConfiguration;
import robotkit.profile.RobotDriveConfiguration;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.manipulation.Manipulator;
import robotkit.manipulation.ToolBoxObstacle;
import robotkit.manipulation.ToolPlanningContext;
import robotkit.manipulation.ToolClearanceShape;
import robotkit.runtime.Simulation;
import robotkit.runtime.SimulationHarness;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.HolonomicDrivePlant;
import robotkit.mobile.MobileBase;
import robotkit.mobile.Pose2;
import robotkit.localization.HolonomicOdometryLocalization;
import robotkit.navigation.Navigation;
import robotkit.navigation.Navigator;
import robotkit.navigation.NavigationGoal;
import robotkit.navigation.OccupancyGrid2;
import robotkit.navigation.OccupancyCell;
import robotkit.navigation.Costmap2;
import robotkit.navigation.AStarPlanner;
import robotkit.perception.PerceptionSnapshot;
import robotkit.perception.PointCloud;
import robotkit.perception.SimulatedSurfaceScanner;
import robotkit.skill.Skill;
import robotkit.skill.SkillRunner;
import robotkit.skill.SkillStatus;
import robotkit.skill.ScanSurface;
import robotkit.skill.RegisterSurface;
import robotkit.skill.FinishSurface;
import robotkit.skill.Paint;
import robotkit.skill.Sand;
import robotkit.tool.SimulatedSprayer;
import robotkit.tool.SimulatedSander;
import robotkit.tool.Tool;
import robotkit.tool.ToolCollisionShape;
import robotkit.work.WorkSurface;
import robotkit.work.Polygon2;
import robotkit.work.Point2;
import robotkit.world.SimulatedRobot;
import robotkit.world.RecordingRobot;
import robotkit.world.ReplayRobot;
import robotkit.world.McapRobotRecording;
import robotkit.world.McapRecordingReader;
import robotkit.world.RobotDescription;
import robotkit.world.RobotCapabilities;
import robotkit.world.RobotSnapshot;
import robotkit.world.ProcessChannelDeclaration;
import robotkit.world.ProcessEventValue;

/**
 * M10 acceptance tests: `ScanSurface`, `RegisterSurface`, `Paint`, and
 * `Sand` (thin `FinishSurface` variants) run through `SkillRunner` against
 * the M9-style simulated robot (omni base + UR-class arm), then against a
 * `ReplayRobot` of a recording, mirroring the forklift skills' pattern.
 * `HolonomicOdometryLocalization` (new here) is what makes replay possible
 * at all for a holonomic base: `SimulationTruthLocalization` needs a live
 * `Simulation`, which a `ReplayRobot` does not have.
 */
class ConstructionSkillTests {
  static var assertions = 0;

  public static function run():Int {
    assertions = 0;
    testConstructionSkillsOnSimulationAndReplay();
    Sys.println('RobotKit construction skill tests passed ($assertions assertions)');
    return assertions;
  }

  static function testConstructionSkillsOnSimulationAndReplay():Void {
    var fixture = buildFixture();
    var model = fixture.model;
    var manipulator = fixture.arm;
    var linkNames = [for (link in model.links) link.name];
    var jointNames = [for (joint in model.joints) joint.name];
    var blueprint = RobotRuntimeCompiler.compile(model, fixture.profile);
    blueprint.channels.push(new ProcessChannelDeclaration("surface.process",
      ProcessEventValue.Digital(false)));

    var simulationHarness = new SimulationHarness(0.02);


    var simulation = simulationHarness.simulation;
    var recordingPath = '/tmp/robotkit-${Sys.getPid()}-construction-skills.mcap';
    var writer = new McapRobotRecording(recordingPath, 4 * 1024 * 1024);
    var runtime = simulation.addRobot(blueprint);
    var sourceRobot = new SimulatedRobot("construction-robot", runtime,
      model.name, linkNames, jointNames);
    var robot = new RecordingRobot(sourceRobot, writer);

    var base = MobileBase.fromBlueprint(robot, blueprint);
    var plant = new HolonomicDrivePlant(simulationHarness, 0, base);
    var localization = new HolonomicOdometryLocalization([0, 1, 2], fixture.wheelRadius, fixture.baseRadius);
    var navigation = new Navigation(base, localization, 0.2, 0.3, 1.0);
    var grid = new OccupancyGrid2(0.1, new Pose2(-2.0, -2.0), 40, 40, "odom", OccupancyCell.Free);
    var baseFootprint = base.footprint;
    if (baseFootprint == null) throw "construction skill fixture did not provide a base footprint";
    var costmap = new Costmap2(grid, baseFootprint.radius);
    var navigator = new Navigator(navigation, new AStarPlanner(costmap), costmap);

    var tick = 0;
    var timestep = 0.02;
    var latestSnapshot:RobotSnapshot = plant.step(Int64.ofInt(tick++));
    localization.update(latestSnapshot);
    var observe:RobotSnapshot -> PerceptionSnapshot = function(snapshot) {
      latestSnapshot = snapshot;
      localization.update(snapshot);
      return new PerceptionSnapshot();
    };
    var runner = new SkillRunner();
    function runToCompletion(skill:Skill):SkillStatus {
      var status = runner.start(skill);
      var steps = 0;
      while (status == SkillStatus.Running && steps < 20000) {
        var snapshot = plant.step(Int64.ofInt(tick++));
        observe(snapshot);
        status = runner.update(snapshot, timestep);
        steps++;
      }
      return status;
    }

    // -- design surface (small wall; see PlacementTests for this exact map_T_surface convention) --
    var boundary = new Polygon2([
      new Point2(-0.25, 0.0), new Point2(0.25, 0.0), new Point2(0.25, 0.3), new Point2(-0.25, 0.3)
    ]);
    var rotation = Quat.fromRotationMatrix([0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0]);
    var map_T_surface = new Transform3(new Vec3(-0.47, 0.0, -0.15), rotation);
    var design = new WorkSurface("skill-wall", "odom", map_T_surface, boundary);

    // -- ScanSurface --
    var trueOffset = new Transform3(new Vec3(0.0, 0.0, 0.003),
      Quat.fromAxisAngle(new Vec3(0.0, 1.0, 0.0), 0.004));
    var scan = new ScanSurface(function() {
      return SimulatedSurfaceScanner.scan(design, trueOffset, 0.0015, 0.02, 0.0003, 5, Int64.ofInt(0));
    });
    check(runToCompletion(scan) == SkillStatus.Succeeded, "ScanSurface completes through SkillRunner");
    check(scan.cloud != null && scan.cloud.size() > 20, "ScanSurface captures a non-trivial point cloud");

    // -- RegisterSurface --
    var register = new RegisterSurface(design, scan.cloud, 5);
    check(runToCompletion(register) == SkillStatus.Succeeded, "RegisterSurface completes through SkillRunner");
    var registrationOutcome = register.registration;
    check(registrationOutcome != null && registrationOutcome.accepted,
      "RegisterSurface accepts the injected offset");
    if (registrationOutcome == null || registrationOutcome.registered == null)
      throw "RegisterSurface did not produce a registered surface";
    var registered:WorkSurface = registrationOutcome.registered;

    var spec:FinishSpec = {
      toolWidth: 0.08, overlap: 0.2, standoff: 0.02, feedRate: 0.2, leadInOut: 0.0,
      maxPatchWidth: 1.0, standoffDistanceMin: 0.46, standoffDistanceMax: 0.5,
      maxAcceleration: 0.3, coverageCellSize: 0.01, footprintRadius: 0.04, maxJointStep: 4.0 * Math.PI
    };
    var seed = [0.2, -1.0, 1.3, -0.3, 0.5, 0.0];

    // -- Paint --
    var sprayer = new SimulatedSprayer();
    function eventSource():{events:Array<robotkit.world.FiredProcessEvent>, overflow:Bool} {
      var batch = runtime.pollEvents();
      for (event in batch.events) writer.recordProcessEvent(robot.id(), event);
      return batch;
    }
    var paintMotion = SurfacePlanRunner.create(robot, manipulator, eventSource,
      spec.feedRate, spec.maxAcceleration, spec.maxJointStep);
    var paint = new Paint(navigator, manipulator, robot, registered.frame_T_surface, registered, spec,
      observe, sprayer, 0.3, 2.0, seed, paintMotion);
    check(runToCompletion(paint) == SkillStatus.Succeeded, 'Paint completes through SkillRunner (${paint.status()})');
    var paintCoverage = paint.coverage();
    check(paintCoverage != null && paintCoverage.coverageFraction() >= 0.9,
      'Paint covers at least 90% of the wall (got ${paintCoverage == null ? -1.0 : paintCoverage.coverageFraction()})');
    var sprayedOn = false;
    for (event in sprayer.history) if (event.flow > 0.0) sprayedOn = true;
    check(sprayedOn, "Paint commands the simulated sprayer while the raster is process-on");

    // -- Sand --
    var sander = new SimulatedSander();
    var sandMotion = SurfacePlanRunner.create(robot, manipulator, eventSource,
      spec.feedRate, spec.maxAcceleration, spec.maxJointStep);
    var sand = new Sand(navigator, manipulator, robot, registered.frame_T_surface, registered, spec,
      observe, sander, 8000.0, 15.0, seed, sandMotion);
    check(runToCompletion(sand) == SkillStatus.Succeeded, 'Sand completes through SkillRunner (${sand.status()})');
    var sandedOn = false;
    for (event in sander.history) if (event.contactForce > 0.0) sandedOn = true;
    check(sandedOn, "Sand commands a contact-force setpoint on the simulated sander");

    // A skill must pass its mounted tool and cell obstacles to the planner,
    // then stop before navigation or process output when the tool is blocked.
    var mounted = new Tool("paint-head", "paint head", Transform3.identity(),
      ToolCollisionShape.Box(new Vec3(0.05, 0.05, 0.05)));
    var cell = new ToolPlanningContext(mounted, [new ToolBoxObstacle(
      Transform3.identity(), new Vec3(10.0, 10.0, 10.0))]);
    var preparedBeforeSkill = ToolClearanceShape.preparationCount;
    var blockedPaint = new Paint(navigator, manipulator, robot,
      registered.frame_T_surface, registered, spec, observe, new SimulatedSprayer(),
      0.3, 2.0, seed, paintMotion, null, cell);
    var blockedStatus = runToCompletion(blockedPaint);
    check(ToolClearanceShape.preparationCount == preparedBeforeSkill,
      "FinishSurface reuses the planning context's prepared tool shape");
    check(switch blockedStatus { case Failed(_): true; case _: false; },
      "Paint rejects a path blocked by a 3D obstacle at planning time");
    check(blockedPaint.coverage() == null,
      "Blocked Paint does not start coverage or process execution");

    var wrongTcp = new Tool("wrong-tcp", "wrong TCP",
      new Transform3(new Vec3(0.01, 0.0, 0.0), Quat.identity()),
      ToolCollisionShape.Box(new Vec3(0.05, 0.05, 0.05)));
    var rejected = false;
    try {
      new Paint(navigator, manipulator, robot, registered.frame_T_surface, registered,
        spec, observe, new SimulatedSprayer(), 0.3, 2.0, seed, paintMotion, null,
        new ToolPlanningContext(wrongTcp, cell.obstacles));
    } catch (_:Dynamic) rejected = true;
    check(rejected, "Paint rejects a tool whose TCP differs from the manipulator TCP");

    writer.close();
    robot.close();
    simulationHarness.dispose();
    var recording = McapRecordingReader.load(recordingPath);
    check(recording.commands.length > 10 && recording.snapshots.length > 10,
      "construction skill run is recorded to MCAP");

    // Replay the plan command stream and inspect its recorded process output.
    var replayDescription = new RobotDescription("construction-robot",
      "recorded construction robot", linkNames, jointNames);
    var replayCapabilities = new RobotCapabilities("construction-robot",
      jointNames.length,
      [robotkit.world.JointTargetMode.Position, robotkit.world.JointTargetMode.Velocity, robotkit.world.JointTargetMode.Effort],
      new robotkit.world.ExecutionCapabilities(true, 5, 64, 4096, true, true, true, trajectorykit.validation.ValidationGuarantee.Unchecked),
      new robotkit.world.TimingCapabilities(false, false, trajectorykit.validation.ValidationGuarantee.Unchecked));
    var replay = new ReplayRobot("construction-robot", recording,
      replayDescription, replayCapabilities);
    for (command in recording.commands) {
      replay.submit(command);
      replay.advance();
    }
    check(replay.generatedCommands.commands.length == recording.commands.length,
      "ReplayRobot preserves the plan and navigation command stream");
    check(recording.processEvents.length > 0,
      "MCAP preserves process output records from Paint and Sand");
    replay.close();

    if (sys.FileSystem.exists(recordingPath)) sys.FileSystem.deleteFile(recordingPath);
    if (sys.FileSystem.exists(recordingPath + ".incomplete.status"))
      sys.FileSystem.deleteFile(recordingPath + ".incomplete.status");
  }

  static function buildFixture():{model:RobotModel, profile:robotkit.profile.RobotProfile, arm:Manipulator, wheelRadius:Float, baseRadius:Float} {
    var profile = new robotkit.profile.RobotProfile();
    var model = new RobotModel("construction-skill-robot");
    var base = model.addLink(new Link("base", "link/base"));

    var wheelRadius = 0.05;
    var baseRadius = 0.3;
    var wheelAngles = [Math.PI * 0.5, Math.PI * 0.5 + Math.PI * 2.0 / 3.0, Math.PI * 0.5 + Math.PI * 4.0 / 3.0];
    var wheelIds:Array<String> = [];
    for (i in 0...3) {
      var wheel = model.addLink(new Link('wheel$i', 'link/wheel$i'));
      var joint = new Joint('wheel$i-joint', JointType.Continuous, base, wheel, 'joint/wheel$i');
      // A continuous joint still compiles to a bounded native revolute joint
      // (RobotRuntimeCompiler has no separate "unbounded" native type), so
      // lower/upper must be set wide -- the default JointLimits() is [0, 0],
      // which rejects any nonzero wheel position with RK_ERROR_LIMIT.
      joint.limits = new JointLimits(-1000.0, 1000.0, 20.0, 1000.0, 1.0);
      joint.parentFramePosition = [baseRadius * Math.cos(wheelAngles[i]), baseRadius * Math.sin(wheelAngles[i]), 0.0];
      joint.axis = [0.0, 1.0, 0.0];
      model.addJoint(joint);
      wheelIds.push(joint.id);
    }
    profile.mobileBase = new RobotMobileConfiguration(
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
      joint.limits.velocity = 3.0;
      joint.limits.effort = 150.0;
      joint.limits.maxAcceleration = 0.3;
    }
    var flange = model.addFrame(new Frame("flange", links[6], "frame/flange"));
    flange.position = [0.0, d6, 0.0];
    var arm = new Manipulator(model, base.id, flange.id, null, null, null, profile);
    return { model: model, profile: profile, arm: arm, wheelRadius: wheelRadius, baseRadius: baseRadius };
  }

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'assertion failed: $message';
  }
}
