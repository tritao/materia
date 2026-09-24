package robotkit.behavior;

import haxe.Int64;
import robotkit.world.RobotCommand;
import robotkit.world.RobotSnapshot;
import robotkit.world.SensorFrame;

/** Read-only application input plus bounded command output for one robot. */
class WorldBehaviorContext {
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
    // A world behavior cannot assume that source and endpoint clocks share an
    // epoch. A concrete adapter assigns its local default deadline; callers
    // may still provide an explicit endpoint-clock deadline.
    commands.push(RobotCommand.JointTargets([
      robotkit.world.JointTarget.position(joint, target)
    ], expiryNs));
  }
}
