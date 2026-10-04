package robotkit.recording;

import robotkit.core.Robot;
import robotkit.core.RobotCommand;
import robotkit.core.RobotEvent;
import robotkit.core.RobotFault;
import robotkit.core.RobotId;
import robotkit.core.RobotSnapshot;

/** Minimal recording interface accepted by the Robot recording adapter. */
interface RobotRecordingSink {
  function recordCommand(command:RobotCommand, ?robotId:RobotId):Void;
  function recordSnapshot(snapshot:RobotSnapshot):Void;
  function recordFault(fault:RobotFault):Void;
  function recordRobotEvent(robotId:RobotId, event:RobotEvent):Void;
}
