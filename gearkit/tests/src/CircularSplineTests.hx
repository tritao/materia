import gearkit.CircularSpline;
import gearkit.Point2;
import gearkit.StrainWaveInput;
import gearkit.StrainWavePair;
import gearkit.ToothRegion;

/** TD11: the circular spline as a concrete internal tooth (`CircularSpline`, `SampledInternalProfile`). */
class CircularSplineTests {
  static var assertions = 0;
  static final ALPHA = 20 * Math.PI / 180;
  static final M = 0.3;

  static function check(ok:Bool, message:String):Void {
    assertions++;
    if (!ok) throw message;
  }
  static function near(a:Float, b:Float, tolerance:Float, message:String):Void check(
    Math.isFinite(a) && Math.abs(a - b) <= tolerance, '$message: $a versus $b (tol $tolerance)');

  public static function run():Void {
    profile();
    Sys.println('CircularSplineTests: $assertions assertions passed');
  }

  /** The circular spline as a tooth profile: closed outline, radii, regions, and a clearance that agrees with the profile it is built from. */
  static function profile():Void {
    var pair = new StrainWavePair(new StrainWaveInput("D1", "involute", M, 200, 202, 1.0, "cosine", ALPHA));
    var cs = CircularSpline.derived("D1 cosine kappa 1", pair);
    var p = cs.profile, spec = p.gearSpec();
    check(spec.internal && spec.teeth == 202, "derived circular spline is the 202-tooth internal gear");
    check(cs.deviationBound > 0 && cs.deviationBound == pair.space().deviationBound, "derived circular spline carries the sweep's deviation bound");
    var o = p.outline(64);
    check(o.length > 8 && o[0].x == o[o.length - 1].x && o[0].y == o[o.length - 1].y, "outline is closed");
    var lo = Math.POSITIVE_INFINITY, hi = Math.NEGATIVE_INFINITY;
    for (q in o) {
      lo = Math.min(lo, q.radius());
      hi = Math.max(hi, q.radius());
    }
    near(lo, cs.tipRadius, 1e-9, "outline lowest radius is the tip circle");
    near(hi, cs.rootRadius, 1e-9, "outline highest radius is the root circle");
    check(p.tipJoinRadius() == cs.tipRadius && p.rootJoinRadius() < cs.rootRadius, "tip join at the tip circle, root join below the root circle");
    check(p.regionAt(0.5 * (p.tipJoinRadius() + p.rootJoinRadius())) == ToothRegion.Flank, "mid-flank radius is on the flank");
    check(p.regionAt(cs.rootRadius) == ToothRegion.Root && p.regionAt(cs.tipRadius) == ToothRegion.Tip, "root and tip radii classify as root and tip");
    // The tooth is one pitch wide on average: thickness at the pitch radius equals the swept space's.
    var rp = spec.pitchRadius();
    near(2 * rp * p.halfAngleAt(rp), pair.space().toothThickness(rp), 1e-9, "profile thickness at the pitch radius");
    // Clearance: a point 1 um off the flank along its normal reads 1 um clear (outside the tooth) and -1 um inside it.
    for (spline in [cs, CircularSpline.involute("nominal", M, 202, ALPHA, 0.1)]) {
      var sp = spline.profile, half = Math.PI / spline.teeth;
      for (t in [0.15, 0.4, 0.7]) {
        var q = sp.flank(t), d = sp.flankDerivative(t), len = d.radius();
        var nx = d.y / len, ny = -d.x / len; // the normal pointing away from the tooth axis: towards +X, +Y side of the flank
        for (side in [1.0, -1.0]) {
          // Tooth centred on +Y at angle half (the ring has a space on +Y); rotate the tooth frame by +pitch/2.
          var probe = new Point2(q.x + side * 1e-3 * nx, q.y + side * 1e-3 * ny).rotated(-half);
          var c = spline.clearance(probe);
          near(Math.abs(c), 1e-3, 4e-5, '${spline.label} clearance of a probe 1 um from the flank (t $t)');
        }
      }
      check(spline.clearance(new Point2(0, rp)) > 0.05 * M, '${spline.label}: the space axis at the pitch radius is open');
      check(spline.clearance(new Point2(rp * Math.sin(half), rp * Math.cos(half))) < -0.05 * M, '${spline.label}: the tooth axis at the pitch radius is material');
    }
  }

}
