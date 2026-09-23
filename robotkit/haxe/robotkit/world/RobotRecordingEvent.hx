package robotkit.world;

/** One typed entry in a deterministic RobotKit recording. */
enum RobotRecordingEvent {
  Command(value:RobotCommand);
  RobotSnapshot(value:RobotSnapshot);
  Sensor(robotId:RobotId, value:SensorFrame);
  Fault(value:RobotFault);
  World(value:WorldSnapshot);
  WorldEvent(value:RobotWorldEvent);
}
