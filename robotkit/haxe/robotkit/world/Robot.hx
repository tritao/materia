package robotkit.world;

/**
 * Common live-robot boundary for physical and simulated robots.
 *
 * RobotWorld depends only on this contract, so behavior code can be reused
 * with a RemoteRobot, SimulatedRobot, replay adapter, or hardware endpoint.
 */
interface Robot {
  function id():RobotId;

  function status():RobotStatus;
  function description():RobotDescription;
  function capabilities():RobotCapabilities;
  function snapshot():RobotSnapshot;
  function sensors():Array<SensorFrame>;
  function fault():Null<RobotFault>;
  function submit(command:RobotCommand):Void;
  function stop(mode:StopMode):Void;
  /** Explicit application acknowledgement that permits clearing a safety latch. */
  function resetSafety():Void;
  function setChangeListener(listener:Null < RobotId -> Void >):Void;
  function close():Void;
}
