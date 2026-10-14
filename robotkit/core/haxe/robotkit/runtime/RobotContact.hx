package robotkit.runtime;

import robotkit.spatial.Vec3;

/** One contact involving a robot link in one captured simulation tick.
 * otherRobot and otherLink are meaningful for RobotLink contacts. */
typedef RobotContact = {
  linkIndex:Int,
  toolPieceIndex:Int,
  /** Zero-based collision compound part on the contacted body, or -1 for unpartitioned geometry. */
  otherPieceIndex:Int,
  otherObject:Int,
  distance:Float,
  position:Vec3,
  normal:Vec3,
  /** Wrench on this robot link, in world axes, at position; false when unsupported. */
  wrenchValid:Bool,
  force:Vec3,
  torque:Vec3,
  active:Bool,
  stepIndex:haxe.Int64,
  otherKind:RobotContactOtherKind,
  otherRobot:Int,
  otherLink:Int
};
