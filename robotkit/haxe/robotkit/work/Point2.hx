package robotkit.work;

/** Immutable x/y coordinate in a WorkSurface's own plane, meters. */
class Point2 {
  public final x:Float;
  public final y:Float;

  public function new(x:Float, y:Float) {
    if (!Math.isFinite(x) || !Math.isFinite(y)) throw "WorkSurface points must be finite";
    this.x = x;
    this.y = y;
  }
}
