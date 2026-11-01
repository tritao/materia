import gearkit.CircularSplineEnvelope;
import gearkit.GearSpec;
import gearkit.Involute;
import gearkit.Point2;
import gearkit.StrainWavePair;
import gearkit.SweptSpace;
import gearkit.ToothPose;

/** TD4: the circular-spline tooth space as the envelope of the swept flexspline tooth. */
class EnvelopeTests {
  static var assertions = 0;
  static var rigidError = 0.0;

  static function check(ok:Bool, message:String):Void {
    assertions++;
    if (!ok) throw message;
  }
  static function near(a:Float, b:Float, tolerance:Float, message:String):Void check(
    Math.isFinite(a) && Math.abs(a - b) <= tolerance, '$message: $a versus $b (tol $tolerance)');

  public static function run():Void {
    rigid();
    pair(ToothDesigns.d1());
    pair(ToothDesigns.d3());
    Sys.println('EnvelopeTests: $assertions assertions passed; rigid-limit envelope error ${rigidError} mm');
  }

  /** Rigid limit: a pinion rolling inside a ring without deformation sweeps the standard involute
      internal gear (space half-angle pi/(2 Zc) - (inv a_r - inv a_p)). */
  static function rigid():Void {
    var alpha = 20 * Math.PI / 180, m = 0.3, zf = 200, zc = 202;
    var spec = new GearSpec(m, zf, 8, new Involute(alpha));
    var rpF = m * zf / 2, rpC = m * zc / 2, e = rpC - rpF;
    var o = spec.profile().outline(64);
    var boundary = o.slice(0, o.length - 1);
    // Pinion centre at e along the line of centres (angle psi); rolls without slipping inside the ring.
    // The window covers about one circular pitch each way, the pinion tooth's time in mesh with space 0.
    var motion = (psi:Float) -> {
      var a = -(zc - zf) / zf * psi; // absolute pinion turn; its centre circles at e
      return new ToothPose(a, -e * Math.sin(psi), e * Math.cos(psi));
    };
    var rLo = rpC - 0.5 * m, rHi = rpC + 0.7 * m;
    var space = CircularSplineEnvelope.derive(boundary, boundary, motion, -0.03, 0.03, 3000, zc, rLo, rHi, 60);
    var rbC = rpC * Math.cos(alpha);
    function inv(a:Float):Float return Math.tan(a) - a;
    var worst = 0.0;
    for (i in 0...41) {
      var r = rLo + (rHi - rLo) * i / 40;
      var analytic = Math.PI / (2 * zc) - (inv(Math.acos(rbC / r)) - inv(alpha));
      worst = Math.max(worst, Math.abs(space.halfAngleAt(r) - analytic) * r);
    }
    rigidError = worst;
    near(worst, 0, 2e-4, "rigid-limit envelope matches the standard internal involute");
    check(space.asymmetry < 2e-4, 'rigid envelope symmetric: ${space.asymmetry}');
  }

  static function pair(input:gearkit.StrainWaveInput):Void {
    var p = new StrainWavePair(input, input.name == "D3" ? 0.08 * input.module : null);
    var space = p.space();
    // Symmetric about the space axis.
    check(space.asymmetry < 1e-6, '${input.name} derived space symmetric: ${space.asymmetry}');
    check(space.deviationBound < 2e-3, '${input.name} sweep converged: ${space.deviationBound}');
    // Re-sweep: no penetration beyond the bound, and at least one contact, at times off the sweep grid.
    var theta = Math.PI / 2 * input.flexTeeth / input.circularTeeth;
    var points = p.boundary(64), worst = Math.POSITIVE_INFINITY, contacts = 0;
    for (s in 0...601) {
      var pose = p.kinematics.pose(0, -theta + 2 * theta * (s + 0.37) / 602);
      var minimum = Math.POSITIVE_INFINITY;
      for (q in points) {
        var w = pose.apply(q), r = w.radius();
        if (r < space.minRadius() || r > space.maxRadius()) continue;
        minimum = Math.min(minimum, space.clearance(w));
      }
      worst = Math.min(worst, minimum);
      if (minimum < 2 * space.deviationBound + 1e-6) contacts++;
    }
    Sys.println('  ${input.name}: sweep bound ${space.deviationBound * 1000} um, worst re-sweep penetration ${-worst * 1000} um, ${contacts} of 601 off-grid times in contact');
    check(worst > -space.deviationBound, '${input.name} penetration $worst exceeds fit bound ${space.deviationBound}');
    check(contacts >= 1, '${input.name} has no contact');
  }
}
