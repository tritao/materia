package motionkit.path;

/** Planar circular fillets bounded by distance to the authored polyline. */
class CornerBlender {
  public static function blend(path:GeometricPath, tolerance:Float,
      maxTurnAngleRadians:Float):BlendedGeometry {
    if (path == null || !Math.isFinite(tolerance) || tolerance < 0.0 ||
        !Math.isFinite(maxTurnAngleRadians) || maxTurnAngleRadians <= 0.0 ||
        maxTurnAngleRadians >= Math.PI)
      throw "Invalid corner blend request";
    var count = path.primitives.length;
    var arcs:Array<Null<ArcSegment>> = [for (_ in 0...count) null];
    var mixed:Array<Null<PathPrimitive>> = [for (_ in 0...count) null];
    var startCuts = [for (_ in 0...count) 0.0];
    var endCuts = [for (_ in 0...count) 0.0];
    var diagnostics:Array<String> = [];
    if (tolerance > 0.0) for (i in 0...(count - 1)) {
      var before = path.primitives[i];
      var after = path.primitives[i + 1];
      if (before.kind() == PathPrimitiveKind.Circular ||
          after.kind() == PathPrimitiveKind.Circular) {
        diagnostics.push('corner ${i + 1}: exact stop (circular primitive)');
        continue;
      }
      if (before.kind() != PathPrimitiveKind.Line || after.kind() != PathPrimitiveKind.Line) {
        if ((before.kind() == PathPrimitiveKind.Line || before.kind() == PathPrimitiveKind.Arc) &&
            (after.kind() == PathPrimitiveKind.Line || after.kind() == PathPrimitiveKind.Arc)) {
          var corner = before.pointAt(before.length());
          if (corner.distanceTo(after.pointAt(0.0)) > 1e-8)
            throw 'Corner ${i + 1} is disconnected';
          var incoming = before.tangentAt(before.length());
          var outgoing = after.tangentAt(0.0);
          var dot = Math.max(-1.0, Math.min(1.0,
            incoming[0] * outgoing[0] + incoming[1] * outgoing[1]));
          var turn = Math.acos(dot);
          if (Math.abs(incoming[2]) > 1e-9 || Math.abs(outgoing[2]) > 1e-9 ||
              Math.abs(corner.z - after.pointAt(0.0).z) > 1e-9) {
            diagnostics.push('corner ${i + 1}: exact stop (nonplanar primitive)');
            continue;
          }
          if (turn < 1e-6) continue;
          if (turn >= maxTurnAngleRadians) {
            diagnostics.push('corner ${i + 1}: exact stop (turn angle $turn exceeds blend limit)');
            continue;
          }
          var cut = Math.min(tolerance * 1.5,
            0.45 * Math.min(before.length(), after.length()));
          var accepted:Null<QuinticBlend> = null;
          for (_ in 0...12) {
            var candidate = new QuinticBlend(before, before.length() - cut,
              after, cut, 2.0 * cut);
            var worst = 0.0;
            for (sample in 0...257) {
              var point = candidate.pointAt(candidate.length() * sample / 256.0);
              worst = Math.max(worst, Math.min(distanceToPrimitive(point, before),
                distanceToPrimitive(point, after)));
            }
            // Distance to a set is 1-Lipschitz. This half-sample travel
            // allowance bounds deviation between the checked points.
            if (worst + candidate.length() / 512.0 <= tolerance) {
              accepted = candidate;
              break;
            }
            cut *= 0.5;
          }
          if (accepted == null || cut <= 1e-9) {
            diagnostics.push('corner ${i + 1}: exact stop (blend tolerance unavailable)');
            continue;
          }
          mixed[i] = accepted;
          endCuts[i] = cut;
          startCuts[i + 1] = cut;
          continue;
        }
        var incoming = before.tangentAt(before.length());
        var outgoing = after.tangentAt(0.0);
        var alignment = 0.0;
        for (coordinate in 0...3)
          alignment += incoming[coordinate] * outgoing[coordinate];
        if (alignment < 0.99999)
          diagnostics.push('corner ${i + 1}: exact stop (nonlinear primitive)');
        continue;
      }
      var first:LineSegment = cast before;
      var second:LineSegment = cast after;
      if (first.end.distanceTo(second.start) > 1e-8)
        throw 'Corner ${i + 1} is disconnected';
      var incoming = first.tangentAt(first.length());
      var outgoing = second.tangentAt(0.0);
      if (Math.abs(incoming[2]) > 1e-9 || Math.abs(outgoing[2]) > 1e-9 ||
          Math.abs(first.end.z - second.end.z) > 1e-9) {
        diagnostics.push('corner ${i + 1}: exact stop (nonplanar line)');
        continue;
      }
      var dot = Math.max(-1.0, Math.min(1.0,
        incoming[0] * outgoing[0] + incoming[1] * outgoing[1]));
      var cross = incoming[0] * outgoing[1] - incoming[1] * outgoing[0];
      var turn = Math.atan2(cross, dot);
      var magnitude = Math.abs(turn);
      if (magnitude < 1e-6) continue;
      if (magnitude >= maxTurnAngleRadians || magnitude >= Math.PI - 1e-6) {
        diagnostics.push('corner ${i + 1}: exact stop (turn angle $magnitude exceeds blend limit)');
        continue;
      }
      var halfTangent = Math.tan(magnitude * 0.5);
      var radius = tolerance / (1.0 / Math.cos(magnitude * 0.5) - 1.0);
      var cut = Math.min(radius * halfTangent,
        0.45 * Math.min(first.length(), second.length()));
      if (cut <= 1e-9) {
        diagnostics.push('corner ${i + 1}: exact stop (insufficient line length)');
        continue;
      }
      radius = cut / halfTangent;
      endCuts[i] = cut;
      startCuts[i + 1] = cut;
      var start = new PathPoint(first.end.x - incoming[0] * cut,
        first.end.y - incoming[1] * cut, first.end.z);
      var sign = turn < 0.0 ? -1.0 : 1.0;
      var center = new PathPoint(start.x - sign * incoming[1] * radius,
        start.y + sign * incoming[0] * radius, start.z);
      var startAngle = Math.atan2(start.y - center.y, start.x - center.x);
      arcs[i] = new ArcSegment(center, radius, startAngle, turn);
    }
    var primitives:Array<PathPrimitive> = [];
    var sourcePrimitiveIndices:Array<Int> = [];
    for (i in 0...count) {
      var primitive = path.primitives[i];
      if (primitive.kind() == PathPrimitiveKind.Line) {
        var line:LineSegment = cast primitive;
        primitives.push(new LineSegment(line.pointAt(startCuts[i]),
          line.pointAt(line.length() - endCuts[i])));
      } else if (primitive.kind() == PathPrimitiveKind.Arc) {
        var arc:ArcSegment = cast primitive;
        var direction = arc.sweepAngle < 0.0 ? -1.0 : 1.0;
        primitives.push(new ArcSegment(arc.center, arc.radius,
          arc.startAngle + direction * startCuts[i] / arc.radius,
          arc.sweepAngle - direction * (startCuts[i] + endCuts[i]) / arc.radius));
      } else primitives.push(primitive);
      sourcePrimitiveIndices.push(i);
      if (arcs[i] != null) {
        primitives.push(arcs[i]); sourcePrimitiveIndices.push(i + 1);
      }
      if (mixed[i] != null) {
        primitives.push(mixed[i]); sourcePrimitiveIndices.push(i + 1);
      }
    }
    return new BlendedGeometry(new GeometricPath(primitives), diagnostics,
      sourcePrimitiveIndices);
  }

  static function distanceToPrimitive(point:PathPoint, primitive:PathPrimitive):Float {
    if (primitive.kind() == PathPrimitiveKind.Line) {
      var line:LineSegment = cast primitive;
      var tangent = line.tangentAt(0.0);
      var projection = (point.x - line.start.x) * tangent[0] +
        (point.y - line.start.y) * tangent[1] +
        (point.z - line.start.z) * tangent[2];
      return point.distanceTo(line.pointAt(Math.max(0.0, Math.min(line.length(), projection))));
    }
    var arc:ArcSegment = cast primitive;
    var angle = Math.atan2(point.y - arc.center.y, point.x - arc.center.x);
    var best = Math.POSITIVE_INFINITY;
    for (shift in -2...3) {
      var candidate = angle + 2.0 * Math.PI * shift;
      var fraction = (candidate - arc.startAngle) / arc.sweepAngle;
      best = Math.min(best, point.distanceTo(arc.pointAt(arc.length() *
        Math.max(0.0, Math.min(1.0, fraction)))));
    }
    return best;
  }
}

class BlendedGeometry {
  public final path:GeometricPath;
  public final diagnostics:Array<String>;
  /** Index of the authored primitive owning each output primitive. */
  public final sourcePrimitiveIndices:Array<Int>;
  public function new(path:GeometricPath, diagnostics:Array<String>,
      sourcePrimitiveIndices:Array<Int>) {
    this.path = path;
    this.diagnostics = diagnostics.copy();
    this.sourcePrimitiveIndices = sourcePrimitiveIndices.copy();
  }
}
