package materia.automation.facility;

import robotkit.mobile.Pose2;

/**
 * The flat, rectangular top of a rack or table: what a person reaches over and must stand clear of.
 * Its position is relative to the pose of whatever owns it. A rack's pose is the rack itself, so its
 * top is usually centred on it; a station's pose is where a person stands, so its table sits ahead of
 * that pose. Metres and radians.
 */
class Surface {
  /** Half the top's extent along its own x axis, and along its y axis. */
  public final halfWidth:Float;
  public final halfDepth:Float;
  /** Height of the top above the floor. */
  public final top:Float;
  /** The top's centre and turn in its owner's frame. */
  public final offsetX:Float;
  public final offsetY:Float;
  public final yaw:Float;

  public function new(halfWidth:Float, halfDepth:Float, top:Float, offsetX:Float = 0.0, offsetY:Float = 0.0, yaw:Float = 0.0) {
    if (!(halfWidth > 0.0) || !(halfDepth > 0.0) || !Math.isFinite(halfWidth) || !Math.isFinite(halfDepth))
      throw "A surface needs a positive width and depth";
    if (!Math.isFinite(top) || !Math.isFinite(offsetX) || !Math.isFinite(offsetY) || !Math.isFinite(yaw))
      throw "A surface needs a finite height, position, and turn";
    this.halfWidth = halfWidth;
    this.halfDepth = halfDepth;
    this.top = top;
    this.offsetX = offsetX;
    this.offsetY = offsetY;
    this.yaw = yaw;
  }

  /** The top's centre in the owner's frame, given the owner's pose there, with its turn including the owner's. */
  public function centerIn(owner:Pose2):{x:Float, y:Float, yaw:Float} {
    var c = Math.cos(owner.yaw), s = Math.sin(owner.yaw);
    return {x: owner.x + c * offsetX - s * offsetY, y: owner.y + s * offsetX + c * offsetY,
      yaw: Pose2.wrapAngle(owner.yaw + yaw)};
  }
}
