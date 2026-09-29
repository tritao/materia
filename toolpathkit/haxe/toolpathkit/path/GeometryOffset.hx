package toolpathkit.path;

/** Translates paths and offsets planar closed contours without CAM dependencies. */
class GeometryOffset {
  public static function translate(geometry:PathGeometry, offset:Array<Float>):PathGeometry {
    if (geometry == null || offset == null || offset.length != 3)
      throw "geometry translation needs a path and three offsets";
    for (value in offset) if (!Math.isFinite(value))
      throw "geometry translation needs finite offsets";
    return switch geometry {
      case Line(start, end): Line(point(start, offset), point(end, offset));
      case Arc(center, radius, startAngle, sweep):
        Arc(point(center, offset), radius, startAngle, sweep);
      case Circular(center, radius, startAngle, sweep, plane, rise):
        Circular(point(center, offset), radius, startAngle, sweep, plane, rise);
    };
  }

  static function point(value:Point3, offset:Array<Float>):Point3
    return new Point3(value.x + offset[0], value.y + offset[1],
      value.z + offset[2]);
  /** Round corners that open toward the cutter and trim the other joins. */
  public static function profile(vertices:Array<Point3>, radius:Float,
      depth:Float, outside:Bool):Array<PathGeometry> {
    if (vertices == null || vertices.length < 3 ||
        !Math.isFinite(radius) || radius <= 0.0 || !Math.isFinite(depth))
      throw "profile offset needs a contour, positive radius and finite depth";
    var count = vertices.length;
    var signedArea = 0.0;
    for (i in 0...count) {
      var a = vertices[i], b = vertices[(i + 1) % count];
      if (a == null || b == null || !Math.isFinite(a.x) ||
          !Math.isFinite(a.y) || !Math.isFinite(a.z) ||
          !Math.isFinite(b.x) || !Math.isFinite(b.y) ||
          !Math.isFinite(b.z) || Math.abs(a.z - b.z) > 1e-8 ||
          a.distanceTo(b) < 1e-10)
        throw "profile offset needs finite planar distinct vertices";
      signedArea += a.x * b.y - b.x * a.y;
    }
    signedArea *= 0.5;
    if (Math.abs(signedArea) < 1e-14)
      throw "profile offset needs a nonzero contour area";
    var orientation = signedArea > 0.0 ? 1.0 : -1.0;
    var shiftedStarts:Array<Point3> = [], shiftedEnds:Array<Point3> = [];
    var convex:Array<Bool> = [];
    for (i in 0...count) {
      var a = vertices[i], b = vertices[(i + 1) % count];
      var dx = b.x - a.x, dy = b.y - a.y;
      var length = Math.sqrt(dx * dx + dy * dy);
      var direction = outside ? 1.0 : -1.0;
      var nx = direction * orientation * dy * radius / length;
      var ny = -direction * orientation * dx * radius / length;
      shiftedStarts.push(new Point3(a.x + nx, a.y + ny, depth));
      shiftedEnds.push(new Point3(b.x + nx, b.y + ny, depth));
    }
    for (i in 0...count) {
      var previous = (i + count - 1) % count;
      var a = vertices[previous], b = vertices[i];
      var c = vertices[(i + 1) % count];
      var abx = b.x - a.x, aby = b.y - a.y;
      var bcx = c.x - b.x, bcy = c.y - b.y;
      var turn = orientation * (abx * bcy - aby * bcx) /
        (Math.sqrt(abx * abx + aby * aby) *
          Math.sqrt(bcx * bcx + bcy * bcy));
      var rounded = outside ? turn > 1e-9 : turn < -1e-9;
      convex.push(rounded);
      if (rounded) continue;
      if (Math.abs(turn) <= 1e-9) {
        var meeting = shiftedEnds[previous];
        shiftedStarts[i] = meeting;
        continue;
      }
      var p = shiftedStarts[previous], q = shiftedStarts[i];
      var rx = shiftedEnds[previous].x - p.x;
      var ry = shiftedEnds[previous].y - p.y;
      var sx = shiftedEnds[i].x - q.x;
      var sy = shiftedEnds[i].y - q.y;
      var denominator = rx * sy - ry * sx;
      if (Math.abs(denominator) < 1e-14)
        throw "CAM profile offset has a degenerate corner";
      var t = ((q.x - p.x) * sy - (q.y - p.y) * sx) / denominator;
      var u = ((q.x - p.x) * ry - (q.y - p.y) * rx) / denominator;
      if (t < -1e-9 || t > 1.0 + 1e-9 ||
          u < -1e-9 || u > 1.0 + 1e-9)
        throw "CAM profile offset exceeds a narrow feature";
      var meeting = new Point3(p.x + t * rx, p.y + t * ry, depth);
      shiftedEnds[previous] = meeting;
      shiftedStarts[i] = meeting;
    }
    var result:Array<PathGeometry> = [];
    for (i in 0...count) {
      var a = vertices[i], b = vertices[(i + 1) % count];
      var start = shiftedStarts[i], end = shiftedEnds[i];
      var forward = (end.x - start.x) * (b.x - a.x) +
        (end.y - start.y) * (b.y - a.y);
      if (forward <= 1e-12)
        throw "CAM profile offset collapses a narrow feature";
      result.push(PathGeometry.Line(start, end));
      var nextIndex = (i + 1) % count;
      if (!convex[nextIndex]) continue;
      var vertex = vertices[(i + 1) % count];
      var next = shiftedStarts[nextIndex];
      var startAngle = Math.atan2(end.y - vertex.y,
        end.x - vertex.x);
      var endAngle = Math.atan2(next.y - vertex.y, next.x - vertex.x);
      var sweep = endAngle - startAngle;
      var arcDirection = outside ? orientation : -orientation;
      if (arcDirection > 0.0) while (sweep <= 0.0) sweep += 2.0 * Math.PI;
      else while (sweep >= 0.0) sweep -= 2.0 * Math.PI;
      result.push(PathGeometry.Arc(new Point3(vertex.x, vertex.y, depth),
        radius, startAngle, sweep));
    }
    validateProfile(vertices, result, radius, !outside);
    return result;
  }

  public static function validateProfile(vertices:Array<Point3>,
      path:Array<PathGeometry>, radius:Float, expectedInside:Bool):Void {
    var points = vertices, count = points.length;
    for (geometry in path) {
      var length = GeometryTools.length(geometry);
      switch geometry {
        case Line(start, end):
          for (i in 0...count)
            if (segmentDistance(start, end, points[i],
                points[(i + 1) % count]) < radius - 1e-8)
              throw "CAM profile offset gouges a nonadjacent edge";
        case Arc(_, _, _, sweep):
          var samples = Std.int(Math.max(32,
            Math.ceil(Math.abs(sweep) * 128.0)));
          for (sample in 0...(samples + 1)) {
            var point = GeometryTools.pointAt(geometry,
              length * sample / samples);
            for (i in 0...count)
              if (pointSegmentDistance(point, points[i],
                  points[(i + 1) % count]) < radius - 1e-8)
                throw "CAM profile offset gouges a nonadjacent edge";
          }
        case _: throw "CAM profile offset needs planar lines and arcs";
      }
      var midpoint = GeometryTools.pointAt(geometry, length * 0.5);
      if (insidePolygon(midpoint, points) != expectedInside)
        throw "CAM profile offset crosses its source contour";
    }
  }

  static function insidePolygon(point:Point3, polygon:Array<Point3>):Bool {
    var inside = false;
    for (i in 0...polygon.length) {
      var a = polygon[i], b = polygon[(i + 1) % polygon.length];
      if ((a.y > point.y) != (b.y > point.y) &&
          point.x < a.x + (point.y - a.y) * (b.x - a.x) / (b.y - a.y))
        inside = !inside;
    }
    return inside;
  }

  static function segmentDistance(a:Point3, b:Point3,
      c:Point3, d:Point3):Float {
    var ax = b.x - a.x, ay = b.y - a.y;
    var cx = d.x - c.x, cy = d.y - c.y;
    var denominator = ax * cy - ay * cx;
    if (Math.abs(denominator) > 1e-14) {
      var t = ((c.x - a.x) * cy - (c.y - a.y) * cx) / denominator;
      var u = ((c.x - a.x) * ay - (c.y - a.y) * ax) / denominator;
      if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) return 0.0;
    }
    return Math.min(Math.min(pointSegmentDistance(a, c, d),
      pointSegmentDistance(b, c, d)),
      Math.min(pointSegmentDistance(c, a, b),
        pointSegmentDistance(d, a, b)));
  }

  static function pointSegmentDistance(point:Point3,
      a:Point3, b:Point3):Float {
    var dx = b.x - a.x, dy = b.y - a.y;
    var t = Math.max(0.0, Math.min(1.0,
      ((point.x - a.x) * dx + (point.y - a.y) * dy) / (dx * dx + dy * dy)));
    var x = point.x - a.x - t * dx, y = point.y - a.y - t * dy;
    return Math.sqrt(x * x + y * y);
  }


}
