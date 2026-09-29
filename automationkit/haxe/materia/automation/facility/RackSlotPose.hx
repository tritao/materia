package materia.automation.facility;

/** Rack slot pose in a named facility frame, metres and yaw radians. */
class RackSlotPose {
  public final x:Float;
  public final y:Float;
  public final z:Float;
  public final yaw:Float;

  public function new(x:Float, y:Float, z:Float, yaw:Float = 0.0) {
    if (!Math.isFinite(x) || !Math.isFinite(y) || !Math.isFinite(z) || !Math.isFinite(yaw))
      throw "Rack slot pose must be finite";
    this.x = x;
    this.y = y;
    this.z = z;
    this.yaw = yaw;
  }
}
