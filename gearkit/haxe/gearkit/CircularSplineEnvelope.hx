package gearkit;

/** Derives the circular-spline tooth space as the swept union of one rigid flexspline tooth
    (TOOTH_DESIGN_PLAN.md TD-D3). `motion(t)` places the tooth in the circular-spline frame at
    parameter t; the space axis is +Y. For each radius the space half-angle is the largest angular
    extent, over all placements, of the tooth boundary crossing that radius (exact chord-circle
    crossings, so no radial binning error). Placements further than half a circular pitch from the
    space axis belong to a neighbouring space and are ignored; a space wider than that saturates
    there, which reads as zero tooth thickness. */
class CircularSplineEnvelope {
  /** `boundary` is the open tooth outline in the tooth frame (tooth centred on +Y). */
  public static function sweep(
    boundary:Array<Point2>,
    motion:Float -> ToothPose,
    from:Float,
    to:Float,
    steps:Int,
    circularTeeth:Int,
    rLo:Float,
    rHi:Float,
    bins:Int
  ):SweptSpace {
    var radii = [for (k in 0...bins) rLo + (rHi - rLo) * k / (bins - 1)];
    var pos = [for (_ in 0...bins) Math.NEGATIVE_INFINITY], neg = [for (_ in 0...bins) Math.NEGATIVE_INFINITY];
    var half = Math.PI / circularTeeth, n = boundary.length;
    var xs = [for (_ in 0...n) 0.0], ys = [for (_ in 0...n) 0.0];
    for (s in 0...steps + 1) {
      var pose = motion(from + (to - from) * s / steps);
      var c = Math.cos(pose.angle), sn = Math.sin(pose.angle);
      for (i in 0...n) {
        xs[i] = c * boundary[i].x - sn * boundary[i].y + pose.tx;
        ys[i] = sn * boundary[i].x + c * boundary[i].y + pose.ty;
      }
      for (i in 0...n - 1) {
        var ax = xs[i], ay = ys[i], dx = xs[i + 1] - ax, dy = ys[i + 1] - ay;
        var ra = Math.sqrt(ax * ax + ay * ay), rb = Math.sqrt(xs[i + 1] * xs[i + 1] + ys[i + 1] * ys[i + 1]);
        var lo = Math.min(ra, rb), hi = Math.max(ra, rb);
        if (hi < rLo || lo > rHi) continue;
        var kLo = Std.int(Math.max(0, Math.ceil((lo - rLo) / (rHi - rLo) * (bins - 1))));
        var kHi = Std.int(Math.min(bins - 1, Math.floor((hi - rLo) / (rHi - rLo) * (bins - 1))));
        var qa = dx * dx + dy * dy;
        if (qa == 0) continue;
        var qb = ax * dx + ay * dy, qc0 = ax * ax + ay * ay;
        for (k in kLo...kHi + 1) {
          var r = radii[k];
          // |a + t d| = r with t in [0, 1]: t = (-qb +/- sqrt(qb^2 - qa (|a|^2 - r^2))) / qa
          var disc = qb * qb - qa * (qc0 - r * r);
          if (disc < 0) continue;
          var root = Math.sqrt(disc);
          for (sign in [-1.0, 1.0]) {
            var t = (-qb + sign * root) / qa;
            if (t < 0 || t > 1) continue;
            var x = ax + t * dx, y = ay + t * dy, phi = Math.atan2(x, y);
            if (Math.abs(phi) >= half) continue;
            if (phi > pos[k]) pos[k] = phi;
            if (-phi > neg[k]) neg[k] = -phi;
          }
        }
      }
    }
    var halfAngles:Array<Float> = [], asym = 0.0;
    for (k in 0...bins) {
      if (pos[k] == Math.NEGATIVE_INFINITY || neg[k] == Math.NEGATIVE_INFINITY) throw 'swept tooth never reaches radius ${radii[k]}';
      halfAngles.push(Math.max(pos[k], neg[k]));
      asym = Math.max(asym, Math.abs(pos[k] - neg[k]) * radii[k]);
    }
    return new SweptSpace(circularTeeth, radii, halfAngles, 0, asym);
  }

  /** Sweep, then again at twice the time, boundary and radial resolution; the second result is
      returned with `deviationBound` set to four times its largest difference from the first. The
      factor is a conservative allowance, because the swept union converges more slowly than
      quadratically near the tip rounding (measured: re-sweep penetration up to 3.6 times the raw difference). */
  public static function derive(
    boundary:Array<Point2>,
    refined:Array<Point2>,
    motion:Float -> ToothPose,
    from:Float,
    to:Float,
    steps:Int,
    circularTeeth:Int,
    rLo:Float,
    rHi:Float,
    bins:Int
  ):SweptSpace {
    var coarse = sweep(boundary, motion, from, to, steps, circularTeeth, rLo, rHi, bins);
    var fine = sweep(refined, motion, from, to, 2 * steps, circularTeeth, rLo, rHi, 2 * bins - 1);
    var bound = 0.0;
    for (k in 0...coarse.radii.length) {
      var r = coarse.radii[k];
      bound = Math.max(bound, Math.abs(coarse.halfAngleAt(r) - fine.halfAngleAt(r)) * r);
    }
    return new SweptSpace(circularTeeth, fine.radii, fine.halfAngles, 4 * bound, fine.asymmetry);
  }
}
