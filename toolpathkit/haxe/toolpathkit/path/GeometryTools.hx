package toolpathkit.path;

/** Preview queries that do not require MotionKit. */
class GeometryTools {
  public static function length(geometry:PathGeometry):Float return switch geometry {
    case Line(start, end): start.distanceTo(end);
    case Arc(_, radius, _, sweep): Math.abs(sweep) * radius;
    case Circular(_, radius, _, sweep, _, rise):
      Math.sqrt(radius * radius * sweep * sweep + rise * rise);
  };

  public static function pointAt(geometry:PathGeometry, distance:Float):Point3 {
    var total = length(geometry);
    if (!Math.isFinite(distance) || distance < 0.0 || distance > total)
      throw "CNC geometry distance is outside its length";
    var alpha = total == 0.0 ? 0.0 : distance / total;
    return switch geometry {
      case Line(start, end):
        new Point3(start.x + (end.x - start.x) * alpha,
          start.y + (end.y - start.y) * alpha,
          start.z + (end.z - start.z) * alpha);
      case Arc(center, radius, startAngle, sweep):
        var angle = startAngle + sweep * alpha;
        new Point3(center.x + radius * Math.cos(angle),
          center.y + radius * Math.sin(angle), center.z);
      case Circular(center, radius, startAngle, sweep, plane, rise):
        var angle = startAngle + sweep * alpha;
        var u = radius * Math.cos(angle), v = radius * Math.sin(angle);
        switch plane {
          case XY: new Point3(center.x + u, center.y + v,
            center.z + rise * alpha);
          case XZ: new Point3(center.x + u, center.y + rise * alpha,
            center.z + v);
          case YZ: new Point3(center.x + rise * alpha,
            center.y + u, center.z + v);
        }
    };
  }

  /** Unit tangent in the direction of travel at a distance along a path. */
  public static function tangentAt(geometry:PathGeometry, distance:Float):Point3 {
    var total = length(geometry);
    if (!(total > 0.0) || !Math.isFinite(distance) ||
        distance < 0.0 || distance > total)
      throw "CNC geometry tangent needs a nonzero path and in-range distance";
    var alpha = distance / total;
    return switch geometry {
      case Line(start, end):
        new Point3((end.x - start.x) / total,
          (end.y - start.y) / total, (end.z - start.z) / total);
      case Arc(_, _, startAngle, sweep):
        var angle = startAngle + sweep * alpha;
        var direction = sweep < 0.0 ? -1.0 : 1.0;
        new Point3(-Math.sin(angle) * direction,
          Math.cos(angle) * direction, 0.0);
      case Circular(_, radius, startAngle, sweep, plane, rise):
        var angle = startAngle + sweep * alpha;
        var du = -radius * Math.sin(angle) * sweep / total;
        var dv = radius * Math.cos(angle) * sweep / total;
        var axial = rise / total;
        switch plane {
          case XY: new Point3(du, dv, axial);
          case XZ: new Point3(du, axial, dv);
          case YZ: new Point3(axial, du, dv);
        }
    };
  }
}
