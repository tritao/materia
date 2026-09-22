package robotkit.world;

import haxe.Int64;

/** Transport-independent command accepted by a logical robot. */
enum RobotCommand {
  /** expiryNs must be null/zero until runtime deadline enforcement is available. */
  JointPosition(joint:Int, target:Float, expiryNs:Null<Int64>);
}
