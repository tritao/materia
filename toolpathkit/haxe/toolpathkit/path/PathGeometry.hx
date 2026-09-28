package toolpathkit.path;

/** Authored toolpath primitive. All coordinates and lengths are metres. */
enum PathGeometry {
  Line(start:Point3, end:Point3);
  Arc(center:Point3, radius:Float, startAngle:Float, sweepAngle:Float);
  Circular(center:Point3, radius:Float, startAngle:Float,
    sweepAngle:Float, plane:ArcPlane, axialRise:Float);
}
