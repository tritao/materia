package motionkit.kinematics;

/** Immutable linear and angular tool velocity in one reference frame. */
class Twist6 {
  public final linearX:Float;
  public final linearY:Float;
  public final linearZ:Float;
  public final angularX:Float;
  public final angularY:Float;
  public final angularZ:Float;

  public function new(linearX:Float, linearY:Float, linearZ:Float,
      angularX:Float, angularY:Float, angularZ:Float) {
    for (value in [linearX, linearY, linearZ, angularX, angularY, angularZ])
      if (!Math.isFinite(value)) throw "Twist6 components must be finite";
    this.linearX = linearX;
    this.linearY = linearY;
    this.linearZ = linearZ;
    this.angularX = angularX;
    this.angularY = angularY;
    this.angularZ = angularZ;
  }

  public function toArray():Array<Float>
    return [linearX, linearY, linearZ, angularX, angularY, angularZ];
}
