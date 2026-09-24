package robotkit.mobile;

/** Immutable counter-clockwise polygon describing a robot's planar footprint. */
class Footprint {
  final points:Array<FootprintPoint>;
  public final radius:Float;

  public function new(points:Array<FootprintPoint>) {
    if (points == null || points.length < 3)
      throw "A footprint requires at least three points";
    this.points = [];
    var twiceArea = 0.0;
    var maxRadius = 0.0;
    for (index in 0...points.length) {
      var point = points[index];
      var next = points[(index + 1) % points.length];
      if (point == null || next == null)
        throw "Footprint points cannot be null";
      this.points.push(new FootprintPoint(point.x, point.y));
      twiceArea += point.x * next.y - next.x * point.y;
      maxRadius = Math.max(maxRadius, Math.pow(point.x * point.x + point.y * point.y, 0.5));
    }
    if (!Math.isFinite(twiceArea) || twiceArea <= 1e-9)
      throw "Footprint polygon must have positive counter-clockwise area";
    radius = maxRadius;
  }

  public function vertices():Array<FootprintPoint> {
    return [for (point in points) new FootprintPoint(point.x, point.y)];
  }

  public static function rectangle(length:Float, width:Float):Footprint {
    if (!Math.isFinite(length) || !Math.isFinite(width) || length <= 0.0 || width <= 0.0)
      throw "Footprint rectangle dimensions must be finite and positive";
    var halfLength = length * 0.5;
    var halfWidth = width * 0.5;
    return new Footprint([
      new FootprintPoint(-halfLength, -halfWidth),
      new FootprintPoint(halfLength, -halfWidth),
      new FootprintPoint(halfLength, halfWidth),
      new FootprintPoint(-halfLength, halfWidth)
    ]);
  }
}
