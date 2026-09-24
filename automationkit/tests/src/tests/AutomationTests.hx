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
import materia.automation.mission.MissionStatus;
import materia.automation.task.Charge;
import materia.automation.task.Pick;
import materia.automation.task.Place;
import materia.automation.task.Task;
import materia.automation.task.Transport;
import robotkit.material.Payload;
import robotkit.mobile.Footprint;
import robotkit.mobile.Pose2;
import robotkit.navigation.Path;
import robotkit.world.Robot;
import robotkit.world.RobotCapabilities;
import robotkit.world.RobotCommand;
import robotkit.world.RobotDescription;
import robotkit.world.RobotFault;
import robotkit.world.RobotId;
import robotkit.world.RobotSnapshot;
import robotkit.world.RobotStatus;
import robotkit.world.RobotWorld;
import robotkit.world.SensorFrame;
import robotkit.world.StopMode;

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
