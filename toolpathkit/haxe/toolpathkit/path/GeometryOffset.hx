package toolpathkit.path;

/** Translates authored path geometry between work and machine origins. */
class GeometryOffset {
  public static function translate(geometry:PathGeometry, offset:Array<Float>):PathGeometry {
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
}
