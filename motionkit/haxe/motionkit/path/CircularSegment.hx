package motionkit.path;

/** Circle in a principal plane, optionally rising along its normal axis. */
class CircularSegment implements PathPrimitive {
  public function kind():PathPrimitiveKind return PathPrimitiveKind.Circular;
  public final center:PathPoint;
  public final plane:CircularPlane;
  public final radius:Float;
  public final startAngle:Float;
  public final sweepAngle:Float;
  public final axialRise:Float;
  public final start:PathPoint;
  public final end:PathPoint;
  final segmentLength:Float;

  public function new(center:PathPoint, radius:Float, startAngle:Float,
      sweepAngle:Float, plane:CircularPlane, ?axialRise:Float = 0.0) {
    if (center == null || plane == null || !Math.isFinite(radius) ||
        radius <= 0.0 || !Math.isFinite(startAngle) ||
        !Math.isFinite(sweepAngle) || Math.abs(sweepAngle) <= 1e-12 ||
        !Math.isFinite(axialRise))
      throw "Circular segment needs finite nonzero circle geometry";
    this.center = center;
    this.radius = radius;
    this.startAngle = startAngle;
    this.sweepAngle = sweepAngle;
    this.plane = plane;
    this.axialRise = axialRise;
    segmentLength = Math.sqrt(Math.pow(radius * sweepAngle, 2) +
      axialRise * axialRise);
    start = atFraction(0.0);
    end = atFraction(1.0);
  }

  public function length():Float return segmentLength;

  public function pointAt(distance:Float):PathPoint {
    validateDistance(distance);
    return atFraction(distance / segmentLength);
  }

  public function tangentAt(distance:Float):Array<Float> {
    validateDistance(distance);
    var angle = startAngle + sweepAngle * distance / segmentLength;
    return axes(-radius * sweepAngle * Math.sin(angle) / segmentLength,
      radius * sweepAngle * Math.cos(angle) / segmentLength,
      axialRise / segmentLength);
  }

  /** Cartesian second derivative with respect to travelled distance. */
  public function secondDerivativeAt(distance:Float):Array<Float> {
    validateDistance(distance);
    var angle = startAngle + sweepAngle * distance / segmentLength;
    var scale = -radius * sweepAngle * sweepAngle /
      (segmentLength * segmentLength);
    return axes(scale * Math.cos(angle), scale * Math.sin(angle), 0.0);
  }

  public function curvatureAt(distance:Float):Float {
    validateDistance(distance);
    return radius * sweepAngle * sweepAngle /
      (segmentLength * segmentLength);
  }

  /** Shortest 3D distance, including the axial rise of a helix. */
  public function distanceTo(point:PathPoint):Float {
    if (point == null) throw "Circular distance needs a point";
    var best = Math.POSITIVE_INFINITY;
    var count = Std.int(Math.max(16,
      Math.ceil(Math.abs(sweepAngle) * 8.0 / Math.PI)));
    for (index in 0...(count + 1)) {
      var t = index / count;
      for (_ in 0...6) {
        var p = atFraction(t);
        var angle = startAngle + sweepAngle * t;
        var first = axes(-radius * sweepAngle * Math.sin(angle),
          radius * sweepAngle * Math.cos(angle), axialRise);
        var second = axes(-radius * sweepAngle * sweepAngle * Math.cos(angle),
          -radius * sweepAngle * sweepAngle * Math.sin(angle), 0.0);
        var delta = [p.x - point.x, p.y - point.y, p.z - point.z];
        var gradient = dot(delta, first);
        var hessian = dot(first, first) + dot(delta, second);
        if (Math.abs(hessian) < 1e-14) break;
        var next = Math.max(0.0, Math.min(1.0, t - gradient / hessian));
        if (Math.abs(next - t) < 1e-12) { t = next; break; }
        t = next;
      }
      best = Math.min(best, point.distanceTo(atFraction(t)));
    }
    return best;
  }

  function atFraction(fraction:Float):PathPoint {
    var angle = startAngle + sweepAngle * fraction;
    var delta = axes(radius * Math.cos(angle), radius * Math.sin(angle),
      axialRise * fraction);
    return new PathPoint(center.x + delta[0], center.y + delta[1],
      center.z + delta[2]);
  }

  function axes(u:Float, v:Float, axial:Float):Array<Float> return switch plane {
    case XY: [u, v, axial];
    case XZ: [u, axial, v];
    case YZ: [axial, u, v];
  };

  static function dot(a:Array<Float>, b:Array<Float>):Float
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

  function validateDistance(distance:Float):Void {
    if (!Math.isFinite(distance) || distance < 0.0 || distance > segmentLength)
      throw "Circular-segment distance is outside its length";
  }
}
