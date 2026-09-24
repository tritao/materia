package materia.automation.fleet;

import materia.automation.mission.Mission;
import robotkit.world.RobotStatus;
import robotkit.world.RobotWorld;

/** Fleet roster and mission reservations composed over a live RobotWorld. */
class Fleet {
  public final id:String;
  public final world:RobotWorld;
  final memberIds = new Map<String,Bool>();
  final assignmentsByMission = new Map<String,FleetAssignment>();
  final missionByRobot = new Map<String,String>();

  public function new(id:String, world:RobotWorld) {
    if (id == null || id.length == 0 || world == null) throw "Fleet requires an ID and RobotWorld";
    this.id = id;
    this.world = world;
  }

  public function addRobot(robotId:String):Void {
    if (robotId == null || memberIds.exists(robotId)) throw "Fleet robot ID is missing or duplicated";
    if (world.robot(robotId) == null) throw 'Robot "$robotId" is not attached to RobotWorld';
    memberIds.set(robotId, true);
  }

  public function removeRobot(robotId:String):Void {
    if (!memberIds.exists(robotId)) return;
    if (missionByRobot.exists(robotId)) throw "Cannot remove a robot with an active mission";
    memberIds.remove(robotId);
  }

  public function containsRobot(robotId:String):Bool return memberIds.exists(robotId);

  public function robotIds():Array<String> {
    var result = [for (id in memberIds.keys()) id];
    result.sort(Reflect.compare);
    return result;
  }

  public function availableRobotIds():Array<String> {
    var result:Array<String> = [];
    for (robotId in robotIds()) {
      if (missionByRobot.exists(robotId)) continue;
      var robot = world.robot(robotId);
      if (robot != null && robot.status() == RobotStatus.Ready) result.push(robotId);
    }
    return result;
  }

  public function assignmentForRobot(robotId:String):Null<FleetAssignment> {
    var missionId = missionByRobot.get(robotId);
    return missionId == null ? null : assignmentsByMission.get(missionId);
  }

  public function assignmentForMission(missionId:String):Null<FleetAssignment>
    return assignmentsByMission.get(missionId);

  public function assign(robotId:String, mission:Mission):FleetAssignment {
    if (mission == null || mission.status != Pending) throw "Only a pending mission can be assigned";
    if (!memberIds.exists(robotId)) throw 'Robot "$robotId" is not in fleet "$id"';
    if (assignmentsByMission.exists(mission.id)) throw "Mission already has a fleet assignment";
    if (missionByRobot.exists(robotId)) throw 'Robot "$robotId" already has an active mission';
    var robot = world.robot(robotId);
    if (robot == null || robot.status() != RobotStatus.Ready)
      throw 'Robot "$robotId" is not ready';
    var assignment = new FleetAssignment(robotId, mission);
    assignmentsByMission.set(mission.id, assignment);
    missionByRobot.set(robotId, mission.id);
    mission.start();
    return assignment;
  }

  public function release(missionId:String):Void {
    var assignment = assignmentsByMission.get(missionId);
    if (assignment == null) return;
    if (!assignment.mission.isTerminal()) throw "Mission must be terminal before its robot is released";
    assignmentsByMission.remove(missionId);
    missionByRobot.remove(assignment.robotId);
  }
}
