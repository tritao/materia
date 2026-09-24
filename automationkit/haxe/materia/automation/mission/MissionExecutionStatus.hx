package materia.automation.mission;

/** Runtime result of advancing a mission through RobotKit skills. */
enum MissionExecutionStatus {
  Idle;
  Running;
  Succeeded;
  Failed(message:String);
  Cancelled;
}
