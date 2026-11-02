package gearkit;

/** The circular-spline tooth space derived as the envelope of a swept flexspline tooth
    (TOOTH_DESIGN_PLAN.md TD-D3), sampled: for each radius, the half-angle of the space about its
    axis (+Y), valid for `radii[0] .. radii[last]`. The space repeats `teeth` times around the ring.
    `deviationBound` is the largest change of the half-angle (as a length at its radius) when the
    sweep is refined, i.e. the numerical accuracy of this sampling, not a design tolerance. */
class SweptSpace {
  public final teeth:Int;
  public final radii:Array<Float>;
  public final halfAngles:Array<Float>;
  public final deviationBound:Float;
  /** Largest left/right difference of the swept extent, as a length. */
  public final asymmetry:Float;

  public function new(teeth:Int, radii:Array<Float>, halfAngles:Array<Float>, deviationBound:Float, asymmetry:Float) {
    if (radii.length != halfAngles.length || radii.length < 3) throw "swept space needs matching samples";
    this.teeth = teeth;
    this.radii = radii;
    this.halfAngles = halfAngles;
    this.deviationBound = deviationBound;
    this.asymmetry = asymmetry;
  }

  public function pitchAngle():Float return 2 * Math.PI / teeth;
  public function minRadius():Float return radii[0];
  public function maxRadius():Float return radii[radii.length - 1];

  function segmentAt(r:Float):Int {
    var lo = 0, hi = radii.length - 1;
    while (hi - lo > 1) {
      var mid = (lo + hi) >> 1;
      if (radii[mid] <= r) lo = mid;
      else hi = mid;
    }
    return lo;
  }

  /** Half-angle of the space at radius r (linear between samples); outside the sampled range is rejected. */
  public function halfAngleAt(r:Float):Float {
    if (!(r >= radii[0] && r <= radii[radii.length - 1])) throw "radius is outside the swept space samples";
    var i = segmentAt(r), f = (r - radii[i]) / (radii[i + 1] - radii[i]);
    return halfAngles[i] + f * (halfAngles[i + 1] - halfAngles[i]);
  }

  /** Circular-spline tooth thickness (arc length at radius r) between this space and its neighbour. */
  public function toothThickness(r:Float):Float return r * (pitchAngle() - 2 * halfAngleAt(r));

  /** Distance (mm) from a point of the flexspline, in the circular-spline frame, to the circular
      spline: positive in the open space (clearance), negative inside circular-spline material
      (penetration). Valid for radii inside the sampled range; above it the caller owns the root. */
  public function clearance(p:Point2):Float {
    var r = p.radius();
    if (r < radii[0] || r > radii[radii.length - 1]) throw "point is outside the swept space radii";
    var a = Math.atan2(p.x, p.y), pitch = pitchAngle();
    a = a - pitch * Math.round(a / pitch);
    var x = Math.abs(a), s = halfAngleAt(r);
    var i = segmentAt(r), best = Math.POSITIVE_INFINITY;
    // Distance to the flank polyline near the radius, in the frame where the space axis is +Y.
    var px = r * Math.sin(x), py = r * Math.cos(x);
    var k0 = Std.int(Math.max(0, i - 4)), k1 = Std.int(Math.min(radii.length - 2, i + 4));
    for (k in k0...k1 + 1) {
      var ax = radii[k] * Math.sin(halfAngles[k]), ay = radii[k] * Math.cos(halfAngles[k]);
      var bx = radii[k + 1] * Math.sin(halfAngles[k + 1]), by = radii[k + 1] * Math.cos(halfAngles[k + 1]);
      var dx = bx - ax, dy = by - ay, u = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy);
      u = Math.max(0, Math.min(1, u));
      var ex = ax + u * dx - px, ey = ay + u * dy - py;
      best = Math.min(best, Math.sqrt(ex * ex + ey * ey));
    }
    return x <= s ? best : -best;
  }
}
