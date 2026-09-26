package motionkit.path;

/** Planar circular arc in the XY plane with a constant Z coordinate. */
class ArcSegment implements PathPrimitive {
  public final center:PathPoint;
  public final radius:Float;
  public final startAngle:Float;
  public final sweepAngle:Float;
  public final start:PathPoint;
  public final end:PathPoint;
  final arcLength:Float;

  public function new(center:PathPoint, radius:Float, startAngle:Float,
      sweepAngle:Float, ?z:Float) {
    if (center == null) throw "Arc segment needs a center";
    var planeZ = z == null ? center.z : z;
    if (!Math.isFinite(radius) || radius < 0.0 || !Math.isFinite(startAngle) ||
        !Math.isFinite(sweepAngle) || !Math.isFinite(planeZ))
      throw "Arc segment geometry must be finite and non-negative where required";
    this.center = new PathPoint(center.x, center.y, planeZ);
    this.radius = radius;
    this.startAngle = startAngle;
    this.sweepAngle = sweepAngle;
    this.start = pointAtAngle(startAngle);
    this.end = pointAtAngle(startAngle + sweepAngle);
    arcLength = Math.abs(sweepAngle) * radius;
  }

  public function length():Float return arcLength;

  public function pointAt(distance:Float):PathPoint {
    if (!Math.isFinite(distance) || distance < 0.0 || distance > arcLength)
      throw "Arc-segment distance is outside its length";
    var alpha = arcLength <= 0.0 ? 0.0 : distance / arcLength;
    return pointAtAngle(startAngle + sweepAngle * alpha);
  }

  public function tangentAt(distance:Float):Array<Float> {
    if (!Math.isFinite(distance) || distance < 0.0 || distance > arcLength)
      throw "Arc-segment distance is outside its length";
    if (arcLength <= 0.0) return [0.0, 0.0, 0.0];
    var angle = startAngle + sweepAngle * (distance / arcLength);
    var direction = sweepAngle < 0.0 ? -1.0 : 1.0;
    return [-Math.sin(angle) * direction, Math.cos(angle) * direction, 0.0];
  }

  public function curvatureAt(distance:Float):Float {
    if (!Math.isFinite(distance) || distance < 0.0 || distance > arcLength)
      throw "Arc-segment distance is outside its length";
    if (radius <= 0.0) return 0.0;
    return sweepAngle < 0.0 ? -1.0 / radius : 1.0 / radius;
  }

  function pointAtAngle(angle:Float):PathPoint
    return new PathPoint(center.x + radius * Math.cos(angle),
      center.y + radius * Math.sin(angle), center.z);
}
