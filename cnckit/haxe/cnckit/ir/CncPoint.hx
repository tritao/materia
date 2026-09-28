package cnckit.ir;

/** Cartesian point in machine metres, independent of MotionKit. */
class CncPoint {
  public final x:Float;
  public final y:Float;
  public final z:Float;

  public function new(x:Float, y:Float, z:Float) {
    this.x = x; this.y = y; this.z = z;
  }

  public function distanceTo(other:CncPoint):Float {
    var dx = x - other.x, dy = y - other.y, dz = z - other.z;
    return Math.sqrt(dx * dx + dy * dy + dz * dz);
  }
}
