package gearkit;

/** Immutable hob-local point in mm; spindle axis is +Z. */
class Point3 {
  public final x:Float;
  public final y:Float;
  public final z:Float;
  public function new(x:Float, y:Float, z:Float) {
    this.x = x;
    this.y = y;
    this.z = z;
  }
}
