package tests;

import NativeKitRuntime;
import haxe.Int64;
import materia.automation.facility.Charger;
import materia.automation.facility.Facility;
import materia.automation.facility.FacilityRouter;
import materia.automation.facility.Lane;
import materia.automation.facility.Rack;
import materia.automation.facility.Station;
import materia.automation.facility.Zone;
import materia.automation.fleet.Dispatcher;
import materia.automation.fleet.Fleet;
import materia.automation.fleet.FleetAssignment;
import materia.automation.fleet.TrafficManager;
import materia.automation.mission.Mission;
import materia.automation.mission.MissionExecutionStatus;
import materia.automation.mission.MissionExecutor;
import materia.automation.mission.MissionStatus;
import materia.automation.mission.TaskSkillFactory;
import materia.automation.task.Charge;
import materia.automation.task.Pick;
import materia.automation.task.Place;
import materia.automation.task.Task;
import materia.automation.task.TaskKind;
import materia.automation.task.Transport;
import materia.automation.facility.Station;
import robotkit.model.Joint;
import robotkit.model.JointLimits;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotDriveConfiguration;
import robotkit.model.RobotMobileConfiguration;
import robotkit.model.RobotModel;
import robotkit.material.Payload;
import robotkit.localization.Localization;
import robotkit.localization.LocalizationQuality;
import robotkit.localization.LocalizationState;
import robotkit.localization.PoseCovariance2;
import robotkit.localization.WheelOdometryLocalization;
import robotkit.mobile.Footprint;
import robotkit.mobile.MobileBase;
import robotkit.mobile.Pose2;
import robotkit.navigation.Navigation;
import robotkit.navigation.NavigationGoal;
import robotkit.navigation.Path;
import robotkit.skill.GoTo;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.Simulation;
import robotkit.world.Robot;
import robotkit.world.RobotCapabilities;
import robotkit.world.RobotCommand;
import robotkit.world.RobotDescription;
import robotkit.world.RobotFault;
import robotkit.world.RobotId;
import robotkit.world.McapRecordingReader;
import robotkit.world.McapRobotRecording;
import robotkit.world.RecordingRobot;
import robotkit.world.RobotSnapshot;
import robotkit.world.RobotRecording;
import robotkit.world.ReplayRobot;
import robotkit.world.SimulatedRobot;
import robotkit.world.RobotStatus;
import robotkit.world.RobotWorld;
import robotkit.world.SensorFrame;
import robotkit.world.StopMode;
import robotkit.skill.Skill;
import robotkit.skill.SkillResult;
import robotkit.skill.SkillStatus;

class AutomationTests {
  static var assertions = 0;

  static function main():Void {
    var nativeRuntime = NativeKitRuntime.start();
    var world = new RobotWorld();
    var robotModel = configuredMobileRobotModel();
    var robot = new AutomationFakeRobot("forklift-1", robotModel);
    try {
      var payload = new Payload(500.0, 1.2, 0.8, 0.15, 0.6);
      var facility = new Facility("warehouse", "Warehouse A");
      facility.addZone(new Zone("main", "Main floor", "map",
        Footprint.rectangle(40.0, 20.0)));
      var inbound = new Station("inbound", "Inbound", "main", "map", new Pose2(0.0, 0.0));
      var outbound = new Station("outbound", "Outbound", "main", "map", new Pose2(4.0, 0.0));
      var rack = new Rack("rack-4", "Rack 4", "main", "map", new Pose2(2.0, 1.0), ["A1", "A2"]);
      var charger = new Charger("charger-2", "Charger 2", "main", "map",
        new Pose2(-1.0, 0.0), "type-2", 11.0);
      facility.addStation(inbound);
      facility.addStation(outbound);
      facility.addRack(rack);
      facility.addCharger(charger);
      facility.addLane(new Lane("inbound-outbound", inbound.id, outbound.id,
        new Path([inbound.pose, outbound.pose], "map"), 2.0, 0.2, false));
      facility.addLane(new Lane("inbound-rack", inbound.id, rack.id,
        new Path([inbound.pose, new Pose2(2.0, 0.0), rack.pose], "map"), 1.5, 1.5));
      facility.addLane(new Lane("rack-outbound", rack.id, outbound.id,
        new Path([rack.pose, new Pose2(3.0, 1.0), outbound.pose], "map"), 1.5, 0.3));
      var lane:Lane = cast facility.lane("inbound-outbound");
      var storedRack:Rack = cast facility.rack("rack-4");
      equal(facility.stations().length, 4, "facility indexes stations and specialized locations");
      check(storedRack.hasSlot("A2"), "rack exposes a known slot");
      equal(lane.centerline.frameId, "map",
        "lane preserves its facility frame");
      throws(function() facility.addLane(new Lane("bad-frame", inbound.id, outbound.id,
        new Path([inbound.pose, outbound.pose], "odom"), 1.0, 0.5)),
        "facility rejects a lane whose frame differs from its stations");
      throws(function() new Rack("bad-rack", "Bad", "main", "map", new Pose2(), ["A", "A"]),
        "rack rejects duplicate slot IDs");
      var router = new FacilityRouter(facility);
      var route = router.route(inbound.id, outbound.id);
      var routeLegs = route.legs();
      check(routeLegs.length == 2 && routeLegs[0].lane.id == "inbound-rack" &&
        route.maximumSpeedMetersPerSecond == 0.3 && route.path.frameId == "map" &&
        route.path.start().x == inbound.pose.x && route.path.goal().x == outbound.pose.x,
        "facility router chooses the fastest lane sequence and composes its framed path");
      var routeSpeedLimits = route.speedLimits();
      check(routeSpeedLimits.length == 2 &&
        routeSpeedLimits[0].maximumSpeedMetersPerSecond == 1.5 &&
        routeSpeedLimits[1].maximumSpeedMetersPerSecond == 0.3 &&
        Math.abs(routeSpeedLimits[0].endDistanceMeters -
          routeSpeedLimits[1].startDistanceMeters) < 1e-9,
        "facility route preserves each lane speed interval along the composed path");
      var reverseRoute = router.route(outbound.id, inbound.id);
      var reverseLegs = reverseRoute.legs();
      check(reverseLegs.length == 2 && reverseLegs[0].reversed && reverseLegs[1].reversed,
        "facility router traverses bidirectional lanes while excluding one-way lanes");

      var tasks:Array<Task> = [
        new Pick("pick-1", rack.id, "A1", payload),
        new Transport("move-1", inbound.id, outbound.id, payload),
        new Place("place-1", rack.id, "A2", payload),
        new Charge("charge-1", charger.id, 0.9)
      ];
      var mission = new Mission("mission-17", "Restock order 17", tasks);
      var firstTask:Task = cast mission.currentTask();
      equal(firstTask.id, "pick-1", "mission starts at its first task");
      equal(mission.tasks().length, 4, "mission owns a copy of its task sequence");
      var source:Array<Task> = tasks;
      source.pop();
      equal(mission.tasks().length, 4, "mutating input cannot change the mission");

      world.attach(robot);
      var fleet = new Fleet("warehouse-east", world);
      fleet.addRobot(robot.id());
      var dispatcher = new Dispatcher(fleet);
      var assignment:FleetAssignment = cast dispatcher.dispatch(mission);
      check(assignment != null && assignment.robotId == robot.id(),
        "dispatcher assigns a pending mission to the first ready robot");
      check(switch mission.status { case Running: true; case _: false; },
        "dispatch starts the mission lifecycle");
      equal(fleet.availableRobotIds().length, 0, "assigned robots are unavailable to other missions");
      var secondMission = new Mission("mission-18", "Second order", [new Charge("charge-2", charger.id, 0.8)]);
      equal(dispatcher.dispatch(secondMission), null, "dispatcher leaves work queued when no robot is free");

      var secondRobot = new AutomationFakeRobot("forklift-2", robotModel);
      world.attach(secondRobot);
      fleet.addRobot(secondRobot.id());
      var secondAssignment:FleetAssignment = cast dispatcher.dispatch(secondMission);
      equal(secondAssignment.robotId, secondRobot.id(), "dispatcher assigns the next mission to the next ready robot");
      var traffic = new TrafficManager(facility, fleet);
      check(traffic.reserve("inbound-outbound", robot.id()), "fleet member reserves a free lane");
      check(traffic.reserve("inbound-outbound", robot.id()), "lane reservation is idempotent for its owner");
      check(!traffic.reserve("inbound-outbound", secondRobot.id()),
        "traffic manager prevents a second robot from entering an occupied lane");
      equal(traffic.owner("inbound-outbound"), robot.id(), "traffic manager reports the lane owner");
      check(traffic.release("inbound-outbound", robot.id()), "owner releases its lane");
      check(traffic.reserve("inbound-outbound", secondRobot.id()), "waiting robot can reserve the released lane");
      check(traffic.release("inbound-outbound", secondRobot.id()), "second lane owner releases its reservation");
      check(traffic.reserve("inbound-outbound", robot.id()) &&
        !traffic.reserveIntersection(rack.id, robot.id()) &&
        traffic.release("inbound-outbound", robot.id()) &&
        traffic.reserveIntersection(rack.id, robot.id()) &&
        traffic.releaseIntersection(rack.id, robot.id()),
        "individual resource locks reject acquisition that violates global deadlock order");

      for (index in 0...4) mission.completeCurrentTask();
      check(switch mission.status { case Succeeded: true; case _: false; },
        "mission succeeds after all tasks complete");
      dispatcher.releaseCompleted(mission);
      secondMission.completeCurrentTask();
      check(switch secondMission.status { case Succeeded: true; case _: false; },
        "single-task mission succeeds after its task completes");
      dispatcher.releaseCompleted(secondMission);
      equal(fleet.availableRobotIds().length, 2, "terminal releases return robots to the fleet");
      equal(fleet.assignmentForMission(mission.id), null, "terminal release removes the assignment");

      var thirdRobot = new AutomationFakeRobot("forklift-3", robotModel);
      world.attach(thirdRobot);
      fleet.addRobot(thirdRobot.id());
      check(traffic.reserveRoute(route, robot.id(), 1),
        "traffic manager atomically reserves every lane and intermediate junction for a route");
      equal(traffic.owner("inbound-rack"), robot.id(), "route reservation owns its first lane");
      equal(traffic.intersectionOwner(rack.id), robot.id(),
        "route reservation owns the shared rack junction");
      check(!traffic.reserveRoute(reverseRoute, secondRobot.id(), 5) &&
        !traffic.reserveRoute(reverseRoute, thirdRobot.id(), 10),
        "conflicting route reservations wait without holding partial resources");
      var routeQueue = traffic.waitingForLane("inbound-rack");
      check(routeQueue.length == 2 && routeQueue[0] == thirdRobot.id() &&
        routeQueue[1] == secondRobot.id(),
        "traffic route waiters are ordered by priority and then arrival");
      check(!traffic.releaseRoute(reverseRoute, secondRobot.id()) &&
        traffic.waitingForLane("inbound-rack").length == 1 &&
        traffic.waitingForLane("inbound-rack")[0] == thirdRobot.id(),
        "cancelling a pending route request removes its waiter without releasing another robot's route");
      check(traffic.releaseRoute(route, robot.id()) &&
        traffic.reserveRoute(reverseRoute, thirdRobot.id(), 10),
        "highest-priority route acquires the complete resource bundle when released");
      check(!traffic.reserveRoute(reverseRoute, secondRobot.id(), 5),
        "lower-priority route remains queued behind an active reservation");
      check(traffic.releaseRoute(reverseRoute, thirdRobot.id()) &&
        traffic.reserveRoute(reverseRoute, secondRobot.id(), 5) &&
        traffic.releaseRoute(reverseRoute, secondRobot.id()),
        "queued route proceeds after the higher-priority robot releases its bundle");
      traffic.blockLane("inbound-rack", "pallet spill");
      check(!traffic.reserveRoute(route, robot.id(), 10) &&
        traffic.laneBlockReason("inbound-rack") == "pallet spill" &&
        traffic.reservedLanes(robot.id()).length == 0,
        "blocked lanes reject route acquisition without partial reservations");
      check(traffic.unblockLane("inbound-rack") && traffic.reserveRoute(route, robot.id()) &&
        traffic.releaseRoute(route, robot.id()),
        "clearing a lane block allows a complete route reservation");

      var executableMission = new Mission("mission-19", "Execute work order", [
        new Transport("move-2", inbound.id, outbound.id, payload),
        new Charge("charge-3", charger.id, 0.95)
      ]);
      var executableAssignment:FleetAssignment = cast dispatcher.dispatch(executableMission);
      var skillFactory = new ScriptedTaskSkillFactory();
      var executor = new MissionExecutor(fleet, executableAssignment, facility, skillFactory);
      executor.start();
      check(switch executor.status { case MissionExecutionStatus.Running: true; case _: false; },
        "mission executor starts the first task skill");
      equal(skillFactory.createdTaskIds.length, 1, "executor creates one skill for the active task");
      check(switch executor.update(0.02) { case MissionExecutionStatus.Running: true; case _: false; },
        "mission executor advances to the next task after skill success");
      equal(executableMission.completedTaskCount(), 1, "skill completion advances mission progress");
      equal(skillFactory.createdTaskIds.length, 2, "executor composes a skill for each mission task");
      check(switch executor.update(0.02) { case MissionExecutionStatus.Succeeded: true; case _: false; },
        "mission executor completes after all task skills succeed");
      check(switch executableMission.status { case MissionStatus.Succeeded: true; case _: false; },
        "skill completion propagates to the mission lifecycle");
      equal(fleet.assignmentForMission(executableMission.id), null,
        "mission executor releases its fleet assignment at completion");

      var rejectedMission = new Mission("mission-20", "Rejected task", [
        new Charge("charge-4", charger.id, 0.8)
      ]);
      var rejectedAssignment:FleetAssignment = cast dispatcher.dispatch(rejectedMission);
      var rejectingFactory = new ScriptedTaskSkillFactory(true);
      var rejectedExecutor = new MissionExecutor(fleet, rejectedAssignment, facility, rejectingFactory);
      rejectedExecutor.start();
      check(switch rejectedExecutor.status {
        case MissionExecutionStatus.Failed(_): true;
        case _: false;
      }, "skill construction failure terminates the mission");
      check(switch rejectedMission.status { case MissionStatus.Failed(_): true; case _: false; },
        "executor failure propagates to mission status");
      equal(fleet.assignmentForMission(rejectedMission.id), null,
        "failed mission releases its fleet assignment");

      var cancellableMission = new Mission("mission-22", "Cancellable order", [
        new Charge("charge-5", charger.id, 0.85)
      ]);
      var cancellableAssignment:FleetAssignment = cast dispatcher.dispatch(cancellableMission);
      var pendingSkillFactory = new ScriptedTaskSkillFactory(false, false);
      var cancellableExecutor = new MissionExecutor(fleet, cancellableAssignment,
        facility, pendingSkillFactory);
      cancellableExecutor.start();
      cancellableExecutor.cancel();
      check(switch cancellableExecutor.status {
        case MissionExecutionStatus.Cancelled: true;
        case _: false;
      }, "mission executor cancels its active skill and mission");
      check(switch cancellableMission.status { case MissionStatus.Cancelled: true; case _: false; },
        "executor cancellation propagates to mission status");
      check(switch pendingSkillFactory.lastSkill.status() {
        case SkillStatus.Cancelled: true;
        case _: false;
      }, "executor cancellation reaches the active RobotKit skill");
      equal(fleet.assignmentForMission(cancellableMission.id), null,
        "cancelled mission releases its fleet assignment");

      var transportMission = new Mission("mission-21", "Drive between stations", [
        new Transport("move-3", inbound.id, outbound.id, payload)
      ]);
      var transportAssignment:FleetAssignment = cast dispatcher.dispatch(transportMission);
      var transportExecutor = new MissionExecutor(fleet, transportAssignment, facility,
        new FacilityTransportSkillFactory(robotModel));
      transportExecutor.start();
      check(switch transportExecutor.update(0.02) {
        case MissionExecutionStatus.Succeeded: true;
        case _: false;
      }, "facility transport task executes through a RobotKit GoTo skill");
      check(switch transportMission.status { case MissionStatus.Succeeded: true; case _: false; },
        "RobotKit skill completion finishes the facility transport mission");

      testSimulatedMissionReplay(robotModel);

      world.close();
      world = null;
      nativeRuntime.dispose();
      Sys.println('Materia automation tests passed ($assertions assertions)');
    } catch (error:Dynamic) {
      if (world != null) world.close();
      nativeRuntime.dispose();
      throw error;
    }
  }

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'assertion failed: $message';
  }

  static function equal(actual:Dynamic, expected:Dynamic, message:String):Void
    check(actual == expected, '$message (expected ${Std.string(expected)}, got ${Std.string(actual)})');

  static function throws(action:Void -> Void, message:String):Void {
    var didThrow = false;
    try action() catch (_:Dynamic) didThrow = true;
    check(didThrow, message);
  }

  static function configuredMobileRobotModel():RobotModel {
    var model = new RobotModel("authored-automation-forklift");
    var base = model.addLink(new Link("base", "link/base"));
    var lift = model.addLink(new Link("mast", "link/mast"));
    var leftWheel = model.addLink(new Link("left wheel", "link/left-wheel"));
    var rightWheel = model.addLink(new Link("right wheel", "link/right-wheel"));
    var liftJoint = model.addJoint(new Joint("mast lift", JointType.Prismatic,
      base, lift, "joint/lift"));
    liftJoint.limits = new JointLimits(0.0, 1.0, 1.0, 100.0);
    var leftJoint = model.addJoint(new Joint("left wheel joint", JointType.Continuous,
      base, leftWheel, "joint/left-wheel"));
    leftJoint.limits = new JointLimits(-100.0, 100.0, 100.0, 100.0);
    var rightJoint = model.addJoint(new Joint("right wheel joint", JointType.Continuous,
      base, rightWheel, "joint/right-wheel"));
    rightJoint.limits = new JointLimits(-100.0, 100.0, 100.0, 100.0);
    model.mobileBase = new RobotMobileConfiguration(
      RobotDriveConfiguration.Differential("joint/left-wheel", "joint/right-wheel",
        0.1, 0.5), 1.0, 1.0);
    return model;
  }

  static function testSimulatedMissionReplay(model:RobotModel):Void {
    var simulation = new Simulation(0.01);
    var world = new RobotWorld();
    var replayWorld:Null<RobotWorld> = null;
    var recordingPath = '/tmp/materia-automation-${Sys.getPid()}-mission.mcap';
    var writer:Null<McapRobotRecording> = null;
    var closed = false;
    try {
      writer = new McapRobotRecording(recordingPath, 4 * 1024 * 1024);
      var linkNames = [for (link in model.links) link.name];
      var jointNames = [for (joint in model.joints) joint.name];
      var blueprint = RobotRuntimeCompiler.compile(model);
      var simulationRobot = new SimulatedRobot("mission-forklift", simulation.addRobot(blueprint),
        "mission forklift", linkNames, jointNames);
      var liveRobot = new RecordingRobot(simulationRobot, cast writer);
      world.attach(liveRobot);

      var facility = new Facility("test-warehouse", "Test warehouse");
      facility.addZone(new Zone("floor", "Floor", "map", Footprint.rectangle(10.0, 5.0)));
      var origin = new Station("origin", "Origin", "floor", "map", new Pose2());
      var destination = new Station("destination", "Destination", "floor", "map",
        new Pose2(0.18, 0.0, 0.0));
      facility.addStation(origin);
      facility.addStation(destination);
      facility.addLane(new Lane("main-lane", origin.id, destination.id,
        new Path([origin.pose, destination.pose], "map"), 1.5, 0.5));

      var fleet = new Fleet("test-fleet", world);
      fleet.addRobot(liveRobot.id());
      var dispatcher = new Dispatcher(fleet);
      var task = new Transport("transport-1", origin.id, destination.id,
        new Payload(100.0, 0.8, 0.6, 0.4, 0.4));
      var mission = new Mission("sim-mission", "Simulated station transfer", [task]);
      var assignment:FleetAssignment = cast dispatcher.dispatch(mission);
      var liveFactory = new MobileTransportSkillFactory(model);
      var trafficRobot = new AutomationFakeRobot("traffic-owner", model);
      world.attach(trafficRobot);
      fleet.addRobot(trafficRobot.id());
      var traffic = new TrafficManager(facility, fleet);
      var route = new FacilityRouter(facility).route(origin.id, destination.id);
      check(traffic.reserveRoute(route, trafficRobot.id()),
        "another fleet robot can hold a transport route before mission execution");
      var executor = new MissionExecutor(fleet, assignment, facility, liveFactory, traffic);
      executor.start();
      var waitingNavigation:Navigation = cast liveFactory.lastNavigation;
      check(switch executor.status {
        case MissionExecutionStatus.Running: true;
        case _: false;
      } && switch waitingNavigation.status {
        case robotkit.navigation.NavigationStatus.Idle: true;
        case _: false;
      } && traffic.owner("main-lane") == trafficRobot.id() &&
        traffic.waitingForLane("main-lane").indexOf(liveRobot.id()) >= 0,
        "mission waits for an occupied route before starting its transport skill");
      traffic.releaseRoute(route, trafficRobot.id());
      var acquiredStatus = executor.update(0.01);
      check(switch acquiredStatus {
        case MissionExecutionStatus.Running: true;
        case _: false;
      } && traffic.owner("main-lane") == liveRobot.id() &&
        switch waitingNavigation.status {
          case robotkit.navigation.NavigationStatus.Following: true;
          case _: false;
        },
        "unblocking a lane lets the mission reserve its route before navigation starts");
      var executionStatus = executor.status;
      var ticks = 0;
      while (switch executionStatus { case MissionExecutionStatus.Running: true; case _: false; } &&
          ticks < 300) {
        executionStatus = executor.update(0.01);
        if (switch executionStatus { case MissionExecutionStatus.Running: true; case _: false; })
          simulation.step(Int64.ofInt((ticks + 1) * 10000000));
        ticks++;
      }
      check(switch executionStatus { case MissionExecutionStatus.Succeeded: true; case _: false; } &&
          ticks < 300, "fleet mission drives the simulated robot to its facility destination");
      check(switch mission.status { case MissionStatus.Succeeded: true; case _: false; },
        "simulated skill completion closes the assigned mission");
      equal(traffic.owner("main-lane"), null,
        "mission completion releases its route reservation");

      check(traffic.reserveRoute(route, trafficRobot.id()),
        "another robot can reserve the route for a pending-mission cancellation check");
      var cancelledMission = new Mission("cancelled-transport",
        "Cancel a queued transport", [task]);
      var cancelledAssignment:FleetAssignment = cast dispatcher.dispatch(cancelledMission);
      var cancelledFactory = new MobileTransportSkillFactory(model);
      var cancelledExecutor = new MissionExecutor(fleet, cancelledAssignment,
        facility, cancelledFactory, traffic);
      cancelledExecutor.start();
      check(traffic.waitingForLane("main-lane").indexOf(liveRobot.id()) >= 0,
        "queued transport records its robot in the route waiter list");
      cancelledExecutor.cancel();
      check(traffic.waitingForLane("main-lane").length == 0 &&
        traffic.owner("main-lane") == trafficRobot.id() &&
        switch cancelledMission.status { case MissionStatus.Cancelled: true; case _: false; },
        "cancelling a queued mission removes its waiter and preserves the route owner");
      traffic.releaseRoute(route, trafficRobot.id());
      var liveNavigation:Navigation = cast liveFactory.lastNavigation;
      var liveEstimateValue:LocalizationState = cast liveNavigation.localization.state();
      check(Math.abs(liveEstimateValue.pose.x - destination.pose.x) <= 0.02,
        "mission executor leaves the live odometry at the station goal");
      check(liveRobot.recordingError == null, "recording adapter preserves an error-free live run");

      var liveWriter:McapRobotRecording = cast writer;
      check(Int64.compare(liveWriter.status().dropped, Int64.ofInt(0)) == 0,
        "MCAP writer has no dropped mission events before shutdown");
      liveWriter.close();
      writer = null;
      var recording = McapRecordingReader.load(recordingPath);
      check(recording.commands.length > 0,
        "RecordingRobot captures the skill's joint target batches in MCAP");
      check(recording.snapshots.length > 1,
        "RecordingRobot captures the mission's observation stream in MCAP");
      var replay = new ReplayRobot(liveRobot.id(), recording,
        new RobotDescription(liveRobot.id(), "recorded mission forklift", linkNames, jointNames),
        new RobotCapabilities(liveRobot.id(), jointNames.length, true, true, true, false));
      replayWorld = new RobotWorld();
      replayWorld.attach(replay);
      var replayFleet = new Fleet("replay-fleet", replayWorld);
      replayFleet.addRobot(replay.id());
      var replayDispatcher = new Dispatcher(replayFleet);
      var replayMission = new Mission("replay-mission", "Replay station transfer", [task]);
      var replayAssignment:FleetAssignment = cast replayDispatcher.dispatch(replayMission);
      var replayFactory = new MobileTransportSkillFactory(model);
      var replayTraffic = new TrafficManager(facility, replayFleet);
      var replayExecutor = new MissionExecutor(replayFleet, replayAssignment, facility,
        replayFactory, replayTraffic);
      replayExecutor.start();
      var replayStatus = replayExecutor.status;
      var replayTicks = 0;
      while (switch replayStatus { case MissionExecutionStatus.Running: true; case _: false; } &&
          replayTicks < 300) {
        replayStatus = replayExecutor.update(0.01);
        replayTicks++;
        if (switch replayStatus { case MissionExecutionStatus.Running: true; case _: false; }) {
          if (!replay.advance()) break;
        }
      }
      check(switch replayStatus { case MissionExecutionStatus.Succeeded: true; case _: false; } &&
          replayTicks < 300, "the recorded facility mission succeeds through ReplayRobot");
      equal(replayTraffic.owner("main-lane"), null,
        "replayed mission releases its route reservation");
      check(replay.generatedCommands.commands.length > 0,
        "ReplayRobot captures commands generated by the same task skill factory");
      check(replay.generatedCommands.commands.length == recording.commands.length,
        "replayed mission emits the same number of target batches as the MCAP run");
      var commandsMatch = replay.generatedCommands.commands.length == recording.commands.length;
      for (commandIndex in 0...recording.commands.length) {
        switch recording.commands[commandIndex] {
          case JointTargets(sourceTargets, _):
            switch replay.generatedCommands.commands[commandIndex] {
              case JointTargets(replayTargets, _):
                if (sourceTargets.length != replayTargets.length) commandsMatch = false;
                else for (targetIndex in 0...sourceTargets.length) {
                  var sourceTarget = sourceTargets[targetIndex];
                  var replayTarget = replayTargets[targetIndex];
                  if (sourceTarget.joint != replayTarget.joint ||
                      Std.string(sourceTarget.mode) != Std.string(replayTarget.mode) ||
                      Math.abs(sourceTarget.target - replayTarget.target) > 1e-9)
                    commandsMatch = false;
                }
              case _: commandsMatch = false;
            }
          case _: commandsMatch = false;
        }
      }
      check(commandsMatch, "MCAP skill commands replay with the same joint targets and values");
      var replayNavigation:Navigation = cast replayFactory.lastNavigation;
      var replayEstimateValue:LocalizationState = cast replayNavigation.localization.state();
      check(Math.abs(replayEstimateValue.pose.x - destination.pose.x) <= 0.02,
        "replayed mission reaches the same facility station pose");

      replayWorld.close();
      replayWorld = null;
      world.close();
      simulation.dispose();
      cleanupRecording(recordingPath);
      closed = true;
    } catch (error:Dynamic) {
      if (replayWorld != null) replayWorld.close();
      world.close();
      if (!closed) simulation.dispose();
      if (writer != null) try cast(writer, McapRobotRecording).close() catch (_:Dynamic) {}
      cleanupRecording(recordingPath);
      throw error;
    }
  }

  static function cleanupRecording(path:String):Void {
    for (suffix in ["", ".incomplete", ".incomplete.status"]) {
      var candidate = path + suffix;
      if (sys.FileSystem.exists(candidate)) sys.FileSystem.deleteFile(candidate);
    }
  }
}

private class AutomationFakeRobot implements Robot {
  final logicalId:RobotId;
  final descriptionValue:RobotDescription;
  final capabilitiesValue:RobotCapabilities;
  final snapshotValue:RobotSnapshot;
  var changeListener:Null<RobotId->Void> = null;

  public function new(id:RobotId, model:RobotModel) {
    logicalId = id;
    var links = [for (link in model.links) link.name];
    var joints = [for (joint in model.joints) joint.name];
    descriptionValue = new RobotDescription(logicalId, model.name, links, joints);
    capabilitiesValue = new RobotCapabilities(logicalId, joints.length,
      true, true, true, false);
    snapshotValue = new RobotSnapshot(logicalId, Int64.ofInt(0), Int64.ofInt(0),
      [for (_ in joints) 0.0], [for (_ in joints) 0.0], [for (_ in joints) 0.0], 0, 0);
  }
  public function id():RobotId return logicalId;
  public function status():RobotStatus return Ready;
  public function description():RobotDescription return descriptionValue;
  public function capabilities():RobotCapabilities return capabilitiesValue;
  public function snapshot():RobotSnapshot return snapshotValue;
  public function sensors():Array<SensorFrame> return [];
  public function fault():Null<RobotFault> return null;
  public function submit(command:RobotCommand):Void {}
  public function stop(mode:StopMode):Void {}
  public function setChangeListener(listener:Null<RobotId->Void>):Void changeListener = listener;
  public function close():Void changeListener = null;
}

private class ScriptedTaskSkillFactory implements TaskSkillFactory {
  public final createdTaskIds:Array<String> = [];
  public var lastSkill(default, null):Null<ScriptedTaskSkill> = null;
  final rejectCreation:Bool;
  final finishOnUpdate:Bool;

  public function new(?rejectCreation:Bool = false, ?finishOnUpdate:Bool = true) {
    this.rejectCreation = rejectCreation;
    this.finishOnUpdate = finishOnUpdate;
  }

  public function create(task:Task, robot:Robot, facility:Facility):Skill {
    if (rejectCreation) throw "no configured skill for task";
    createdTaskIds.push(task.id);
    lastSkill = new ScriptedTaskSkill(finishOnUpdate);
    return lastSkill;
  }
}

private class ScriptedTaskSkill implements Skill {
  final finishOnUpdate:Bool;
  var currentStatus:SkillStatus = Idle;

  public function new(finishOnUpdate:Bool) this.finishOnUpdate = finishOnUpdate;
  public function start():Void currentStatus = Running;
  public function update(snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus {
    if (finishOnUpdate && switch currentStatus { case Running: true; case _: false; })
      currentStatus = Succeeded;
    return currentStatus;
  }
  public function cancel():Void {
    if (switch currentStatus { case Running: true; case _: false; }) currentStatus = Cancelled;
  }
  public function status():SkillStatus return currentStatus;
  public function result():Null<SkillResult> return null;
}

private class FacilityTransportSkillFactory implements TaskSkillFactory {
  final model:RobotModel;

  public function new(model:RobotModel) this.model = model;

  public function create(task:Task, robot:Robot, facility:Facility):Skill {
    return switch task.kind {
      case TaskKind.Transport:
        var transport:Transport = cast task;
        var pickup:Station = cast facility.station(transport.pickupStationId);
        var destination:Station = cast facility.station(transport.destinationStationId);
        if (pickup == null || destination == null)
          throw "transport task references an unknown facility station";
        var base = MobileBase.fromRobot(robot, model);
        var localization = new FixedPoseLocalization(destination.pose);
        var navigation = new Navigation(base, localization, 0.2, 0.2, 0.8);
        var route = new FacilityRouter(facility).route(pickup.id, destination.id);
        new GoTo(navigation, route.path,
          new NavigationGoal(destination.pose, destination.frameId), route.speedLimits());
      case _: throw 'unsupported task kind ${Std.string(task.kind)}';
    }
  }
}

private class FixedPoseLocalization implements Localization {
  final pose:Pose2;
  var currentState:Null<LocalizationState> = null;

  public function new(pose:Pose2) this.pose = pose;

  public function update(snapshot:RobotSnapshot):LocalizationState {
    currentState = new LocalizationState(snapshot.sourceSequence, pose, "map", "base",
      PoseCovariance2.zero(), LocalizationQuality.Good, snapshot.sourceTimestampNs,
      snapshot.receivedTimestampNs, snapshot.sourceClockId, snapshot.receivedClockId);
    return currentState;
  }

  public function state():Null<LocalizationState> return currentState;
  public function reset(?pose:Pose2):Void currentState = null;
}

private class MobileTransportSkillFactory implements TaskSkillFactory {
  final model:RobotModel;
  public var lastNavigation(default, null):Null<Navigation> = null;

  public function new(model:RobotModel) this.model = model;

  public function create(task:Task, robot:Robot, facility:Facility):Skill {
    return switch task.kind {
      case TaskKind.Transport:
        var transport:Transport = cast task;
        var pickup:Station = cast facility.station(transport.pickupStationId);
        var destination:Station = cast facility.station(transport.destinationStationId);
        if (pickup == null || destination == null)
          throw "transport task references an unknown facility station";
        var base = MobileBase.fromRobot(robot, model);
        var localization = new WheelOdometryLocalization(base, destination.frameId);
        var navigation = new Navigation(base, localization, 0.2, 0.2, 0.8);
        lastNavigation = navigation;
        var route = new FacilityRouter(facility).route(pickup.id, destination.id);
        new GoTo(navigation, route.path,
          new NavigationGoal(destination.pose, destination.frameId, 0.02, 0.1),
          route.speedLimits());
      case _: throw 'unsupported task kind ${Std.string(task.kind)}';
    }
  }
}
