package motionkit.path;

/** Immutable ordered geometric path made from reusable primitives. */
class GeometricPath {
  public final primitives:Array<PathPrimitive>;
  public final totalLength:Float;
  final cumulativeLengths:Array<Float>;

  public function new(primitives:Array<PathPrimitive>) {
    if (primitives == null || primitives.length == 0)
      throw "Geometric path needs at least one primitive";
    this.primitives = primitives.copy();
    cumulativeLengths = [];
    var total = 0.0;
    for (primitive in this.primitives) {
      if (primitive == null) throw "Geometric path cannot contain null primitives";
      var length = primitive.length();
      if (!Math.isFinite(length) || length < 0.0)
        throw "Geometric path primitive length must be finite and non-negative";
      total += length;
      cumulativeLengths.push(total);
    }
    totalLength = total;
  }

  public static function lines(points:Array<PathPoint>):GeometricPath {
    if (points == null || points.length < 2)
      throw "A line path needs at least two points";
    var segments:Array<PathPrimitive> = [];
    for (i in 0...(points.length - 1))
      segments.push(new LineSegment(points[i], points[i + 1]));
    return new GeometricPath(segments);
  }

  public function pointAt(distance:Float):PathPoint {
    if (!Math.isFinite(distance) || distance < 0.0 || distance > totalLength)
      throw "Geometric-path distance is outside its length";
    var previous = 0.0;
    for (i in 0...primitives.length) {
      var end = cumulativeLengths[i];
      if (distance <= end || i == primitives.length - 1)
        return primitives[i].pointAt(distance - previous);
      previous = end;
    }
    return primitives[primitives.length - 1].pointAt(primitives[primitives.length - 1].length());
  }
}
