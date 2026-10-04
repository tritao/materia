package robotkit.recording;

import robotkit.core.RobotCommand;
import robotkit.core.RobotFault;
import robotkit.core.RobotId;
import robotkit.core.RobotSnapshot;
import robotkit.core.SensorFrame;
import robotkit.execution.FiredProcessEvent;
import robotkit.world.RobotWorldEvent;
import robotkit.world.WorldSnapshot;

/** One typed entry in a deterministic RobotKit recording. */
enum RobotRecordingEvent {
  Command(value:RobotCommand);
  RobotSnapshot(value:RobotSnapshot);
  Sensor(robotId:RobotId, value:SensorFrame);
  Fault(value:RobotFault);
  World(value:WorldSnapshot);
  WorldEvent(value:RobotWorldEvent);
  ProcessEvent(value:FiredProcessEvent);
  Channel(robotId:RobotId, channelName:String, payload:Dynamic);
}
