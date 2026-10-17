package gearkit;

/** Immutable transverse-plane point/vector, in millimetres. */
class Point2 {
  public final x:Float;
  public final y:Float;
  public function new(x:Float, y:Float) {
    this.x = x;
    this.y = y;
  }
  public function radius():Float return Math.sqrt(x * x + y * y);
  public function rotated(angle:Float):Point2 {
    var c = Math.cos(angle), s = Math.sin(angle);
    return new Point2(c * x - s * y, s * x + c * y);
  }
  public function scaled(scale:Float):Point2 return new Point2(x * scale, y * scale);
  public function distance(other:Point2):Float return new Point2(x - other.x, y - other.y).radius();
}
