package materia.automation.fleet;

import materia.automation.mission.Mission;

/** One mission reservation on one RobotWorld member. */
class FleetAssignment {
  public final robotId:String;
  public final mission:Mission;

  public function new(robotId:String, mission:Mission) {
    this.robotId = robotId;
    this.mission = mission;
  }
}
