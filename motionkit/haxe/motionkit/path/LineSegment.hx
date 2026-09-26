package motionkit.path;

/** Straight Cartesian path primitive. */
class LineSegment implements PathPrimitive {
  public final start:PathPoint;
  public final end:PathPoint;
  final segmentLength:Float;

  public function new(start:PathPoint, end:PathPoint) {
    if (start == null || end == null) throw "Line segment needs two points";
    this.start = start;
    this.end = end;
    segmentLength = start.distanceTo(end);
  }

  public function length():Float return segmentLength;

  public function pointAt(distance:Float):PathPoint {
    if (!Math.isFinite(distance) || distance < 0.0 || distance > segmentLength)
      throw "Line-segment distance is outside its length";
    var alpha = segmentLength <= 0.0 ? 0.0 : distance / segmentLength;
    return start.lerp(end, alpha);
  }
}
