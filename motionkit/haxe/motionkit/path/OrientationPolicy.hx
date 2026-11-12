package motionkit.path;

/** Permitted tool orientation along a Cartesian path. Axis is in the path frame. */
enum OrientationPolicy implements Equatable {
  Fixed;
  Interpolated;
  Cone(axis:Direction3, halfAngle:Float);
  FreeAboutTool;
  /** Position only; every orientation is acceptable. */
  Free;
}
