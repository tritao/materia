package tests;

import NativeKitRuntime;
import haxe.Int64;
import materia.automation.facility.Charger;
import materia.automation.facility.Facility;
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
import robotkit.material.Payload;
import robotkit.localization.Localization;
import robotkit.localization.LocalizationQuality;
import robotkit.localization.LocalizationState;
import robotkit.localization.PoseCovariance2;
import robotkit.localization.WheelOdometryLocalization;
import robotkit.mobile.DifferentialDrive;
import robotkit.mobile.Footprint;
import robotkit.mobile.MobileBase;
import robotkit.mobile.MotionLimits;
import robotkit.mobile.Pose2;
import robotkit.navigation.Navigation;
import robotkit.navigation.NavigationGoal;
import robotkit.navigation.Path;
import robotkit.skill.GoTo;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.RobotRuntimeJointBlueprint;
import robotkit.runtime.Simulation;
import robotkit.world.Robot;
import robotkit.world.RobotCapabilities;
import robotkit.world.RobotCommand;
import robotkit.world.RobotDescription;
import robotkit.world.RobotFault;
import robotkit.world.RobotId;
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
    var robot = new AutomationFakeRobot("forklift-1");
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
        new Path([inbound.pose, outbound.pose], "map"), 2.0, 1.0, false));
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

      var secondRobot = new AutomationFakeRobot("forklift-2");
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
        new FacilityTransportSkillFactory());
      transportExecutor.start();
      check(switch transportExecutor.update(0.02) {
        case MissionExecutionStatus.Succeeded: true;
        case _: false;
      }, "facility transport task executes through a RobotKit GoTo skill");
      check(switch transportMission.status { case MissionStatus.Succeeded: true; case _: false; },
        "RobotKit skill completion finishes the facility transport mission");

      testSimulatedMissionReplay();

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

  static function testSimulatedMissionReplay():Void {
    var simulation = new Simulation(0.01);
    var world = new RobotWorld();
    var replayWorld:Null<RobotWorld> = null;
    var closed = false;
    try {
      var linkNames = ["base", "left-wheel", "right-wheel"];
      var jointNames = ["left-wheel", "right-wheel"];
      var blueprint = new RobotRuntimeBlueprint(1, 2, 3);
      for (index in 0...2) blueprint.addJoint(new RobotRuntimeJointBlueprint(index,
        RobotKitRuntimeConstants.RK_RUNTIME_JOINT_REVOLUTE, 0, index + 1,
        -1000.0, 1000.0, 1000.0, 1000.0));
      var liveRobot = new SimulatedRobot("mission-forklift", simulation.addRobot(blueprint),
        "mission forklift", linkNames, jointNames);
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
      var recording = new RobotRecording();
      var liveFactory = new MobileTransportSkillFactory();
      var executor = new MissionExecutor(fleet, assignment, facility, liveFactory);
      executor.start();
      var executionStatus = executor.status;
      var ticks = 0;
      while (switch executionStatus { case MissionExecutionStatus.Running: true; case _: false; } &&
          ticks < 300) {
        var observation = liveRobot.snapshot();
        recording.recordSnapshot(observation);
        executionStatus = executor.update(0.01);
        if (switch executionStatus { case MissionExecutionStatus.Running: true; case _: false; })
          simulation.step(Int64.ofInt((ticks + 1) * 10000000));
        ticks++;
      }
      check(switch executionStatus { case MissionExecutionStatus.Succeeded: true; case _: false; } &&
          ticks < 300, "fleet mission drives the simulated robot to its facility destination");
      check(switch mission.status { case MissionStatus.Succeeded: true; case _: false; },
        "simulated skill completion closes the assigned mission");
      var liveNavigation:Navigation = cast liveFactory.lastNavigation;
      var liveEstimateValue:LocalizationState = cast liveNavigation.localization.state();
      check(Math.abs(liveEstimateValue.pose.x - destination.pose.x) <= 0.02,
        "mission executor leaves the live odometry at the station goal");

      var replay = new ReplayRobot(liveRobot.id(), recording,
        new RobotDescription(liveRobot.id(), "recorded mission forklift", linkNames, jointNames),
        new RobotCapabilities(liveRobot.id(), 2, true, true, true, false));
      replayWorld = new RobotWorld();
      replayWorld.attach(replay);
      var replayFleet = new Fleet("replay-fleet", replayWorld);
      replayFleet.addRobot(replay.id());
      var replayDispatcher = new Dispatcher(replayFleet);
      var replayMission = new Mission("replay-mission", "Replay station transfer", [task]);
      var replayAssignment:FleetAssignment = cast replayDispatcher.dispatch(replayMission);
      var replayFactory = new MobileTransportSkillFactory();
      var replayExecutor = new MissionExecutor(replayFleet, replayAssignment, facility, replayFactory);
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
      check(replay.generatedCommands.commands.length > 0,
        "ReplayRobot captures commands generated by the same task skill factory");
      var replayNavigation:Navigation = cast replayFactory.lastNavigation;
      var replayEstimateValue:LocalizationState = cast replayNavigation.localization.state();
      check(Math.abs(replayEstimateValue.pose.x - destination.pose.x) <= 0.02,
        "replayed mission reaches the same facility station pose");

      replayWorld.close();
      replayWorld = null;
      world.close();
      simulation.dispose();
      closed = true;
    } catch (error:Dynamic) {
      if (replayWorld != null) replayWorld.close();
      world.close();
      if (!closed) simulation.dispose();
      throw error;
    }
  }
}

private class AutomationFakeRobot implements Robot {
  final logicalId:RobotId;
  var changeListener:Null<RobotId->Void> = null;

  public function new(id:RobotId) logicalId = id;
  public function id():RobotId return logicalId;
  public function status():RobotStatus return Ready;
  public function description():RobotDescription return new RobotDescription(logicalId, logicalId, [], []);
  public function capabilities():RobotCapabilities return new RobotCapabilities(logicalId, 0, false, false, false, false);
  public function snapshot():RobotSnapshot return new RobotSnapshot(logicalId, Int64.ofInt(0),
    Int64.ofInt(0), [], [], [], 0, 0);
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
  public function new() {}

  public function create(task:Task, robot:Robot, facility:Facility):Skill {
    return switch task.kind {
      case TaskKind.Transport:
        var transport:Transport = cast task;
        var pickup:Station = cast facility.station(transport.pickupStationId);
        var destination:Station = cast facility.station(transport.destinationStationId);
        if (pickup == null || destination == null)
          throw "transport task references an unknown facility station";
        var base = new MobileBase(robot, new DifferentialDrive(0, 1, 0.1, 0.5),
          new MotionLimits(1.0, 1.0));
        var localization = new FixedPoseLocalization(destination.pose);
        var navigation = new Navigation(base, localization, 0.2, 0.2, 0.8);
        var path = new Path([pickup.pose, destination.pose], destination.frameId);
        new GoTo(navigation, path, new NavigationGoal(destination.pose, destination.frameId));
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
  public var lastNavigation(default, null):Null<Navigation> = null;

  public function new() {}

  public function create(task:Task, robot:Robot, facility:Facility):Skill {
    return switch task.kind {
      case TaskKind.Transport:
        var transport:Transport = cast task;
        var pickup:Station = cast facility.station(transport.pickupStationId);
        var destination:Station = cast facility.station(transport.destinationStationId);
        if (pickup == null || destination == null)
          throw "transport task references an unknown facility station";
        var base = new MobileBase(robot, new DifferentialDrive(0, 1, 0.1, 0.5),
          new MotionLimits(0.5, 1.0));
        var localization = new WheelOdometryLocalization(base, destination.frameId);
        var navigation = new Navigation(base, localization, 0.2, 0.2, 0.8);
        lastNavigation = navigation;
        var path = new Path([pickup.pose, destination.pose], destination.frameId);
        new GoTo(navigation, path, new NavigationGoal(destination.pose, destination.frameId,
          0.02, 0.1));
      case _: throw 'unsupported task kind ${Std.string(task.kind)}';
    }
  }
}
