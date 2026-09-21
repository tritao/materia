package robotkit.world;

import haxe.Int64;

/** Transport-independent command accepted by a logical robot. */
enum RobotCommand {
  JointPosition(joint:Int, target:Float, expiryNs:Null<Int64>);
}
