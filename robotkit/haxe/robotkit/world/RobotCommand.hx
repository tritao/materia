package robotkit.world;

import haxe.Int64;

/** Transport-independent commands accepted by a logical robot. */
enum RobotCommand {
  /**
   * One atomic set of position, velocity, and/or effort targets.
   * expiryNs must remain null/zero until runtime deadline enforcement is available.
  */
  JointTargets(targets:Array<JointTarget>, expiryNs:Null<Int64>);
  /** Append a bounded timestamped position chunk to a runtime-owned queue. */
  TrajectoryChunk(chunk:robotkit.world.TrajectoryChunk);
}
