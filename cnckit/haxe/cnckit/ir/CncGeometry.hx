package cnckit.ir;

/** Authored toolpath primitive. All coordinates and lengths are metres. */
enum CncGeometry {
  Line(start:CncPoint, end:CncPoint);
  Arc(center:CncPoint, radius:Float, startAngle:Float, sweepAngle:Float);
}
