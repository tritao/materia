package motionkit.path;

/** Permitted tool orientation along a Cartesian path. Axis is in the path frame. */
enum OrientationPolicy {
  Fixed;
  Interpolated;
  Cone(axis:Array<Float>, halfAngle:Float);
  FreeAboutTool;
}
