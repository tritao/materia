package robotkit.behavior;

import haxe.Int64;
import robotkit.world.RobotCommand;
import robotkit.world.RobotSnapshot;
import robotkit.world.SensorFrame;

/** Read-only application input plus bounded command output for one robot. */
class WorldBehaviorContext {
  public static inline final DEFAULT_COMMAND_LIFETIME_NS:Int = 50_000_000;
  public final snapshot:RobotSnapshot;
  public final sensors:Array<SensorFrame>;
  final commands:Array<RobotCommand>;

  @:allow(robotkit.behavior.WorldBehaviorRunner)
  function new(snapshot:RobotSnapshot, commands:Array<RobotCommand>) {
    this.snapshot = snapshot;
    this.sensors = snapshot.sensors.toArray();
    this.commands = commands;
  }

  public function jointPosition(joint:Int, target:Float, ?expiryNs:Int64):Void {
    var expiry = expiryNs == null
      ? Int64.add(snapshot.sourceTimestampNs, Int64.ofInt(DEFAULT_COMMAND_LIFETIME_NS))
      : expiryNs;
    commands.push(RobotCommand.JointPosition(joint, target, expiry));
  }
}
