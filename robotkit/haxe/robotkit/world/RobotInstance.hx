package robotkit.world;

/** Common world-facing boundary for physical and simulated robots. */
interface RobotInstance {
  function id():RobotId;

  function status():RobotStatus;
  function description():RobotDescription;
  function capabilities():RobotCapabilities;
  function snapshot():RobotSnapshot;
  function fault():Null<RobotFault>;
  function submit(command:RobotCommand):Void;
  function stop(mode:StopMode):Void;
  function setChangeListener(listener:Null < Void -> Void >):Void;
  function close():Void;
}
