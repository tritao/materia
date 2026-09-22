package robotkit.world;

/** One typed entry in a deterministic RobotKit recording. */
enum RobotRecordingEvent {
  Command(value:RobotCommand);
  Snapshot(value:RobotSnapshot);
  Fault(value:RobotFault);
  World(value:WorldSnapshot);
  WorldEvent(value:RobotWorldEvent);
}
