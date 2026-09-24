package robotkit.world;

/** Minimal recording interface accepted by the Robot recording adapter. */
interface RobotRecordingSink {
  function recordCommand(command:RobotCommand, ?robotId:RobotId):Void;
  function recordSnapshot(snapshot:RobotSnapshot):Void;
  function recordFault(fault:RobotFault):Void;
}
