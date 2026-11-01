import gearkit.CircularSpline;
import gearkit.ConjugateRack;
import gearkit.DoubleCircularArc;
import gearkit.FixedMesh;
import gearkit.FixedMeshReport;
import gearkit.GearSpec;
import gearkit.HobCheck;
import gearkit.Involute;
import gearkit.MeshErrors;
import gearkit.MeshingAnalysis;
import gearkit.Point2;
import gearkit.StrainWaveInput;
import gearkit.StrainWavePair;
import gearkit.ToothProfile;

/** The D1 (involute 20 deg) against D3 (involute 30 deg, kappa 0.8 to 1.2) and D2 (double-circular-arc)
    comparison, with the cam comparison and the hob implications of D2. A study, not a test: run on its own
    (`gearkit/TOOTH_DESIGN_PLAN.md`, "Running the tooth study"). Prints a table; every number is model output (unloaded,
    kinematic, zero-backlash envelope circular spline), not a measurement. */
class ToothStudy {
  static final DEG = Math.PI / 180;

  static function d2Scaled(kappa:Float):StrainWaveInput return new StrainWaveInput(
    "D2s", "double-circular-arc", 0.3, 200, 202, kappa, "cosine");

  static function d3(kappa:Float):StrainWaveInput return new StrainWaveInput("D3", "involute", 0.3, 200, 202, kappa, "cosine", 30 * DEG);

  /** Largest cutter tip radius (mm) for which the profile still has a cutter land, by bisection on whether the profile builds. */
  static function maxCutterTipRadius(make:Float -> ToothProfile, module:Float):Float {
    var lo = 0.0, hi = 0.99 * module;
    for (_ in 0...40) {
      var mid = (lo + hi) / 2, ok = true;
      try make(mid) catch (e:Dynamic) ok = false;
      if (ok) lo = mid;
      else hi = mid;
    }
    return lo;
  }

  static function f(x:Float, digits:Int = 3):String {
    var k = Math.pow(10, digits);
    return Std.string(Math.round(x * k) / k);
  }

  /** Smallest radius of curvature along the root fillet, mm (three-point curvature on the profile's own curve). */
  static function minRootCurvatureRadius(p:ToothProfile):Float {
    var best = Math.POSITIVE_INFINITY, h = 1e-3;
    for (i in 1...200) {
      var t = i / 200, a = p.rootFillet(t - h), b = p.rootFillet(t), c = p.rootFillet(t + h);
      var d1x = (c.x - a.x) / (2 * h), d1y = (c.y - a.y) / (2 * h);
      var d2x = (c.x - 2 * b.x + a.x) / (h * h), d2y = (c.y - 2 * b.y + a.y) / (h * h);
      var curvature = Math.abs(d1x * d2y - d1y * d2x) / Math.pow(d1x * d1x + d1y * d1y, 1.5);
      if (curvature > 0) best = Math.min(best, 1 / curvature);
    }
    return best;
  }

  static function tipLand(p:ToothProfile):Float {
    var t = p.tipFillet(1);
    return 2 * p.gearSpec().tipRadius() * Math.atan2(t.x, t.y);
  }


  /** Chebyshev (minimax) deviation of the rack flank x(y) from a straight line of the given slope, best offset. */
  static function lineDeviation(pts:Array<Point2>, slope:Float):Float {
    var lo = Math.POSITIVE_INFINITY, hi = Math.NEGATIVE_INFINITY;
    for (p in pts) {
      var v = p.x - slope * p.y;
      lo = Math.min(lo, v);
      hi = Math.max(hi, v);
    }
    return (hi - lo) / 2;
  }

  /** Offset x at y = 0 of the best-offset straight line of the given slope. */
  static function lineOffset(pts:Array<Point2>, slope:Float):Float {
    var lo = Math.POSITIVE_INFINITY, hi = Math.NEGATIVE_INFINITY;
    for (p in pts) {
      var v = p.x - slope * p.y;
      lo = Math.min(lo, v);
      hi = Math.max(hi, v);
    }
    return (hi + lo) / 2;
  }

  /** Slope dx/dy of the best straight line (minimax), by ternary search (the deviation is convex in the slope). */
  static function bestSlope(pts:Array<Point2>):Float {
    var a = -2.0, b = 2.0;
    for (_ in 0...80) {
      var m1 = a + (b - a) / 3, m2 = b - (b - a) / 3;
      if (lineDeviation(pts, m1) < lineDeviation(pts, m2)) b = m2;
      else a = m1;
    }
    return (a + b) / 2;
  }

  /** Curvature radius at interior samples of a polyline (three-point circle), mm; skips near-collinear triples. */
  static function curvatureRadii(pts:Array<Point2>):Array<Float> {
    var out:Array<Float> = [];
    for (i in 1...pts.length - 1) {
      var a = pts[i - 1], b = pts[i], c = pts[i + 1];
      var ab = a.distance(b), bc = b.distance(c), ca = c.distance(a);
      var area2 = Math.abs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
      if (area2 < 1e-15 || ab < 1e-9 || bc < 1e-9) continue;
      out.push(ab * bc * ca / (2 * area2));
    }
    return out;
  }

  static function reversals(pts:Array<Point2>):Int {
    var n = 0, last = 0.0;
    for (i in 1...pts.length) {
      var d = pts[i].x - pts[i - 1].x;
      if (Math.abs(d) < 1e-12) continue;
      if (last != 0 && d * last < 0) n++;
      last = d;
    }
    return n;
  }

  static function rackReport(label:String, spec:GearSpec):Void {
    var m = spec.module, rack = ConjugateRack.of(spec.profile(), 256), fl = rack.flankSamples(), corner = rack.cornerSamples();
    var yLo = fl[0].y, yHi = fl[fl.length - 1].y;
    var slope20 = -Math.tan(20 * DEG), best = bestSlope(fl);
    // Flank angle (from the tooth axis) at the ends and at the pitch datum.
    function angleAt(y:Float):Float {
      var h = 1e-4 * m, a = rack.flankAt(Math.max(yLo, y - h)), b = rack.flankAt(Math.min(yHi, y + h));
      return Math.atan((b.x - a.x) / (b.y - a.y)) * 180 / Math.PI;
    }
    var radii = curvatureRadii(corner);
    radii.sort((a, b) -> a < b ? -1 : a > b ? 1 : 0);
    var cornerDepth = corner[0].y;
    Sys.println('  rack $label: flank heights ${f(yLo / m, 3)}..${f(yHi / m, 3)} m, x at pitch datum ${f(rack.flankAt(0).x, 4)} mm (pi m/4 = ${f(Math.PI * m / 4, 4)}), flank angle ${f(angleAt(yLo + 0.02 * (yHi - yLo)), 1)} / ${f(angleAt(0), 1)} / ${f(angleAt(yHi - 0.02 * (yHi - yLo)), 1)} deg (low / pitch / high), '
      + 'straight-20deg-line deviation ${f(lineDeviation(fl, slope20) * 1000, 1)} um at datum half-thickness ${f(lineOffset(fl, slope20), 4)} mm, best straight line ${f(Math.atan(best) * 180 / Math.PI, 1)} deg deviation ${f(lineDeviation(fl, best) * 1000, 1)} um, '
      + 'flank x reversals ${reversals(fl)}, root fold ${rack.hasRootFold}, rack tip depth ${f(-cornerDepth / m, 3)} m, '
      + 'rack corner curvature radius min/median/max ${radii.length == 0 ? "n/a" : f(radii[0], 4) + "/" + f(radii[radii.length >> 1], 4) + "/" + f(radii[radii.length - 1], 4)} mm');
  }

  static function hobStudy():Void {
    var m = 0.3, deg = DEG;
    var d1 = new GearSpec(m, 200, 8, new Involute(20 * deg));
    var d3 = new GearSpec(m, 200, 8, new Involute(30 * deg), false, 0, 0.1 * m);
    var d2 = new GearSpec(m, 200, 8, DoubleCircularArc.liu(m));
    rackReport("D1 20deg corner 0.38m", d1);
    rackReport("D3 30deg corner 0.10m", d3);
    rackReport("D2s DCA root fillet 0.38m", d2);
    var maxFillet = maxCutterTipRadius(r -> new GearSpec(m, 200, 8, DoubleCircularArc.liu(m), false, 0, r).profile(), m);
    Sys.println('  D2s largest circular root fillet that fits: ${f(maxFillet)} mm (${f(maxFillet / m)} m); D1 trochoid 0.472 m, D3 0.110 m (same test)');
    for (rf in [0.1, 0.2, 0.38, 0.6, 0.7]) {
      var spec = new GearSpec(m, 200, 8, DoubleCircularArc.liu(m), false, 0, rf * m);
      var hob = HobCheck.problem(spec);
      var r = curvatureRadii(ConjugateRack.of(spec.profile(), 256).cornerSamples());
      r.sort((a, b) -> a < b ? -1 : a > b ? 1 : 0);
      Sys.println('    D2s root fillet ${rf} m: hobbable ${hob == null ? "yes" : "NO " + hob}, rack corner curvature radius min ${f(r[0], 4)} mm, median ${f(r[r.length >> 1], 4)} mm');
    }
    // The reference 20 deg hob (straight flanks, tooth depth 1.25 m + tip land) against the D2s flank, aligned at the pitch datum.
    var r1 = ConjugateRack.of(d1.profile(), 256), r2 = ConjugateRack.of(d2.profile(), 256);
    var worst = 0.0, worstY = 0.0, n = 0;
    var f2 = r2.flankSamples();
    for (p in f2) {
      if (p.y < r1.flankSamples()[0].y || p.y > r1.flankSamples()[r1.flankSamples().length - 1].y) continue;
      var d = Math.abs(p.x - r1.flankAt(p.y).x);
      if (d > worst) {
        worst = d;
        worstY = p.y;
      }
      n++;
    }
    Sys.println('  D1 rack flank against the D2s rack flank over the common heights ($n samples): largest x difference ${f(worst * 1000, 1)} um at height ${f(worstY / m, 3)} m (as cut, no offset)');
  }

  /** Cam comparison: each cam with its own derived circular spline (so transmission error is zero by construction;
      the cam's effect shows as how closely the other teeth follow the envelope). */
  static function camStudy():Void {
    var deg = DEG;
    for (base in ["D1", "D2s"]) for (cam in ["cosine", "ellipse", "four-roller", "resal"]) {
      var input = base == "D1" ? new StrainWaveInput("D1", "involute", 0.3, 200, 202, 1.0, cam, 20 * deg) : new StrainWaveInput("D2s", "double-circular-arc", 0.3, 200, 202, 1.0, cam);
      var pair = new StrainWavePair(input);
      var r = MeshingAnalysis.run(pair, 0.005, 24), r20 = MeshingAnalysis.run(pair, 0.02, 24);
      var worstW = 0.0, worstDw = 0.0, ref = new gearkit.CosineCam(input.w0());
      for (i in 0...181) {
        var phi = Math.PI * i / 180;
        worstW = Math.max(worstW, Math.abs(pair.cam.w(phi) - ref.w(phi)));
        worstDw = Math.max(worstDw, Math.abs(pair.cam.dw(phi) - ref.dw(phi)));
      }
      Sys.println('  cam $base ${cam}: interference ${r.interference.length == 0 ? "no" : "YES"}, rootClr ${f(r.rootClearance * 1000, 1)} um, minorTipClr ${f(r.minorTipClearance * 1000, 1)} um, '
        + 'flank(5um) ${f(r.flankEngagedFraction * 100, 1)}%, flank(20um) ${f(r20.flankEngagedFraction * 100, 1)}%, tip edge(5um) ${f(r.tipContactFraction * 100, 1)}%, tip edge(20um) ${f(r20.tipContactFraction * 100, 1)}%, min clr flank ${f(r.minFlankClearance * 1000, 1)} / tip ${f(r.minTipClearance * 1000, 1)} um, zone(5um) +-${f(r.contactZone * 180 / Math.PI, 0)} deg, '
        + 'max |w - cosine w| ${f(worstW * 1000, 1)} um, max |dw - cosine dw| ${f(worstDw * 1000, 1)} um/rad');
    }
  }

  /** Fixed-circular-spline study (TD11): the circular spline is derived once, from the cosine cam at kappa 1, and every other cam,
      kappa and cam amplitude error is meshed against it. One circular pitch of wave-generator angle is a full period. */
  static function fixedStudy():Void {
    var deg = DEG, samples = 24;
    Sys.println("  fixed circular spline study: TE = drive-flank contact rotation about its mean; backlash = drive minus reverse rotation (negative = overlap, i.e. interference); "
      + 'arcsec at the flexspline, um at the circular-spline pitch circle (R = 30.3 mm); $samples wave-generator angles over one circular pitch; flank / tip = teeth within 1 um at the drive-flank position, mean (max)');
    for (base in ["D1", "D2s"]) {
      function make(kappa:Float, cam:String):StrainWaveInput return base == "D1"
        ? new StrainWaveInput("D1", "involute", 0.3, 200, 202, kappa, cam, 20 * deg)
        : new StrainWaveInput("D2s", "double-circular-arc", 0.3, 200, 202, kappa, cam);
      var t0 = Sys.time();
      var spline = CircularSpline.derived('$base cosine kappa 1', new StrainWavePair(make(1.0, "cosine")));
      Sys.println('  circular spline $base cosine kappa 1: tip radius ${f(spline.tipRadius, 4)} mm, root circle ${f(spline.rootRadius, 4)} mm, sweep deviation bound ${f(spline.deviationBound * 1000, 2)} um, built in ${f(Sys.time() - t0, 1)} s');
      for (cam in ["cosine", "ellipse", "resal"]) for (kappa in [0.9, 1.0, 1.1]) for (amp in [-0.002, 0.0, 0.002]) {
        var t1 = Sys.time();
        var r = FixedMesh.run(spline, new StrainWavePair(make(kappa, cam)), new MeshErrors(amp), samples);
        var rp = 0.3 * 202 / 2, um = rp * 1000;
        var issues = [];
        if (r.rootBottoming > 1e-6) issues.push('root ${f(r.rootBottoming * 1000, 1)} um');
        if (r.radialOverlap > 1e-6) issues.push('tip overlap ${f(r.radialOverlap * 1000, 1)} um');
        if (r.backlashMin < 0) issues.push('negative free range');
        if (r.unresolved > 0) issues.push('${r.unresolved} unresolved');
        Sys.println('  $base cs, flex $cam kappa $kappa amp ${f(amp * 1000, 0)} um: TE p-p ${f(r.tePeakToPeakArcsec(), 2)}", mean drive ${f(r.meanDrive * FixedMeshReport.ARCSEC, 1)}", '
          + 'backlash mean ${f(r.backlashMeanArcsec(), 1)}" (${f(r.backlashMean * um, 1)} um; min ${f(r.backlashMin * um, 1)}, max ${f(r.backlashMax * um, 1)}), '
          + 'flank ${f(r.meanFlankContacts, 1)} (${r.maxFlankContacts}), tip edge ${f(r.meanTipContacts, 1)} (${r.maxTipContacts}), first contact flank ${f(r.firstContactFlank * 100, 0)}% / tip ${f(r.firstContactTip * 100, 0)}%, '
          + 'interference: ${issues.length == 0 ? "none" : issues.join(", ")} [${f(Sys.time() - t1, 1)} s]');
      }
    }
  }

  public static function main():Void {
    if (Sys.args().indexOf("fixed") >= 0) {
      fixedStudy();
      return;
    }
    var m = 0.3, deg = DEG;
    var max20 = maxCutterTipRadius(r -> {
      return new GearSpec(m, 200, 8, new Involute(20 * deg), false, 0, r).profile();
    }, m);
    var max30 = maxCutterTipRadius(r -> {
      return new GearSpec(m, 200, 8, new Involute(30 * deg), false, 0, r).profile();
    }, m);
    Sys.println('  largest hob corner radius that leaves a cutter land (m 0.3, Z 200, full depth): 20 deg ${f(max20)} mm (${f(max20 / m)} m), 30 deg ${f(max30)} mm (${f(max30 / m)} m)');
    var rho30 = 0.9 * max30;
    var cases:Array<{label:String, input:StrainWaveInput, rho:Null<Float>}> = [];
    for (k in [0.8, 1.0, 1.1, 1.2]) cases.push({label: "D1 20deg rho 0.38m", input: new StrainWaveInput("D1", "involute", m, 200, 202, k, "cosine", 20 * deg), rho: null});
    for (k in [0.8, 0.9, 1.0, 1.1, 1.2]) cases.push({label: 'D3 30deg rho ${f(rho30 / m, 2)}m', input: d3(k), rho: rho30});
    // Same hob corner radius on both angles, so the angle is the only difference.
    for (k in [1.0, 1.1]) cases.push({label: 'D1 20deg rho ${f(rho30 / m, 2)}m', input: new StrainWaveInput("D1", "involute", m, 200, 202, k, "cosine", 20 * deg), rho: rho30});
    // The default 0.38 m hob corner radius (the reference hob's) cannot cut 30 deg at full depth.
    try {
      new StrainWavePair(d3(1.0));
      Sys.println("  D3 30deg with the default 0.38 m corner radius: builds");
    } catch (e:Dynamic) {
      Sys.println('  D3 30deg with the default 0.38 m corner radius: ${Std.string(e)}');
    }
    // D2, double-circular-arc (Liu 2025 optimum). "D2s" is the shape scaled to m 0.3 / 200 teeth; "D2" is the unscaled paper case.
    for (k in [0.8, 1.0, 1.1, 1.2]) cases.push({label: "D2s DCA rho 0.38m", input: d2Scaled(k), rho: null});
    for (k in [0.8, 1.0, 1.2]) cases.push({label: "D2 DCA(paper m 0.5) rho 0.38m", input: new StrainWaveInput("D2", "double-circular-arc", 0.5, 100, 102, k, "cosine"), rho: null});
    hobStudy();
    camStudy();
    fixedStudy();
    for (c in cases) {
      var pair = new StrainWavePair(c.input, c.rho);
      var r = MeshingAnalysis.run(pair, 0.005, 24);
      var r20 = MeshingAnalysis.run(pair, 0.02, 24);
      var hob = HobCheck.problem(pair.flex);
      Sys.println('  ${c.label} kappa ${c.input.kappa}: '
        + 'interference ${r.interference.length == 0 ? "no" : "YES"}, rootClr ${f(r.rootClearance * 1000, 1)} um, minorTipClr ${f(r.minorTipClearance * 1000, 1)} um, '
        + 'CS tip trim ${f((r.derivedTipRadius - pair.circularTipRadius()) * 1000, 1)} um, flank(5um) ${f(r.flankEngagedFraction * 100, 1)}%, flank(20um) ${f(r20.flankEngagedFraction * 100, 1)}%, tip edge(5um) ${f(r.tipContactFraction * 100, 1)}%, tip edge(20um) ${f(r20.tipContactFraction * 100, 1)}%, min clr flank ${f(r.minFlankClearance * 1000, 1)} / tip ${f(r.minTipClearance * 1000, 1)} um, zone(5um) +-${f(r.contactZone * 180 / Math.PI, 0)} deg, '
        + 'rootFilletRmin ${f(minRootCurvatureRadius(pair.profile), 4)} mm, tipLand ${f(tipLand(pair.profile), 4)} mm, hobbable ${hob == null ? "yes" : "NO " + hob}');
    }
  }
}
