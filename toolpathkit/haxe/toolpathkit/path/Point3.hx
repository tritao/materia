package toolpathkit.path;

/** Cartesian point in machine metres, independent of MotionKit. */
class Point3 {
  public final x:Float;
  public final y:Float;
  public final z:Float;

  public function new(x:Float, y:Float, z:Float) {
    this.x = x; this.y = y; this.z = z;
  }

  public function distanceTo(other:Point3):Float {
    var dx = x - other.x, dy = y - other.y, dz = z - other.z;
    return Math.sqrt(dx * dx + dy * dy + dz * dz);
  }
}
