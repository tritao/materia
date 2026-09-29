package robotkit.runtime;

import robotkit.spatial.Vec3;

/** One contact involving a robot link in the latest simulation tick. */
typedef RobotContact = {
  linkIndex:Int,
  toolPieceIndex:Int,
  otherObject:Int,
  distance:Float,
  position:Vec3,
  normal:Vec3,
  active:Bool
};
