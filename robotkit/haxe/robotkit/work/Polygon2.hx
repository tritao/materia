package robotkit.work;

/** Axis-aligned bounding box of a Polygon2, in its own plane. */
typedef PolygonBounds = { minX:Float, minY:Float, maxX:Float, maxY:Float };

/** Immutable simple polygon in a WorkSurface's own XY plane; vertices counter-clockwise. */
class Polygon2 {
  final points:Array<Point2>;

  public function new(points:Array<Point2>) {
    if (points == null || points.length < 3) throw "A polygon requires at least three points";
    this.points = [];
    var twiceArea = 0.0;
    for (index in 0...points.length) {
      var point = points[index];
      var next = points[(index + 1) % points.length];
      if (point == null || next == null) throw "Polygon points cannot be null";
      this.points.push(new Point2(point.x, point.y));
      twiceArea += point.x * next.y - next.x * point.y;
    }
    if (!Math.isFinite(twiceArea) || twiceArea <= 1e-9)
      throw "Polygon must have positive counter-clockwise area";
  }

  public function vertices():Array<Point2> return [for (point in points) new Point2(point.x, point.y)];

  public function area():Float {
    var twiceArea = 0.0;
    for (index in 0...points.length) {
      var point = points[index];
      var next = points[(index + 1) % points.length];
      twiceArea += point.x * next.y - next.x * point.y;
    }
    return twiceArea * 0.5;
  }

  public function bounds():PolygonBounds {
    var minX = points[0].x, maxX = points[0].x, minY = points[0].y, maxY = points[0].y;
    for (point in points) {
      if (point.x < minX) minX = point.x;
      if (point.x > maxX) maxX = point.x;
      if (point.y < minY) minY = point.y;
      if (point.y > maxY) maxY = point.y;
    }
    return { minX: minX, minY: minY, maxX: maxX, maxY: maxY };
  }

  /** X-intervals (ascending, non-overlapping) where the horizontal line at `y` is inside this polygon. */
  public function scanlineIntervals(y:Float):Array<Array<Float>> {
    var xs:Array<Float> = [];
    for (index in 0...points.length) {
      var a = points[index];
      var b = points[(index + 1) % points.length];
      if ((a.y <= y && b.y > y) || (b.y <= y && a.y > y)) {
        var t = (y - a.y) / (b.y - a.y);
        xs.push(a.x + t * (b.x - a.x));
      }
    }
    xs.sort(function(left, right) return left < right ? -1 : (left > right ? 1 : 0));
    var intervals:Array<Array<Float>> = [];
    var i = 0;
    while (i + 1 < xs.length) {
      intervals.push([xs[i], xs[i + 1]]);
      i += 2;
    }
    return intervals;
  }

  /** Point-in-polygon test (even-odd rule); boundary points are treated as outside. */
  public function contains(point:Point2):Bool {
    var inside = false;
    for (index in 0...points.length) {
      var a = points[index];
      var b = points[(index + 1) % points.length];
      if ((a.y > point.y) != (b.y > point.y)) {
        var t = (point.y - a.y) / (b.y - a.y);
        var xCross = a.x + t * (b.x - a.x);
        if (point.x < xCross) inside = !inside;
      }
    }
    return inside;
  }
}
