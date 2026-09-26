package motionkit.path;

/** Small transport-neutral Cartesian point used by geometric path primitives. */
class PathPoint {
  public final x:Float;
  public final y:Float;
  public final z:Float;

  public function new(x:Float = 0.0, y:Float = 0.0, z:Float = 0.0) {
    if (!Math.isFinite(x) || !Math.isFinite(y) || !Math.isFinite(z))
      throw "Path point coordinates must be finite";
    this.x = x;
    this.y = y;
    this.z = z;
  }

  public function distanceTo(other:PathPoint):Float {
    if (other == null) throw "Path point distance needs another point";
    var dx = other.x - x, dy = other.y - y, dz = other.z - z;
    return Math.sqrt(dx * dx + dy * dy + dz * dz);
  }

  public function lerp(other:PathPoint, alpha:Float):PathPoint {
    if (other == null) throw "Path point interpolation needs another point";
    return new PathPoint(x + (other.x - x) * alpha,
      y + (other.y - y) * alpha, z + (other.z - z) * alpha);
  }
}
