import gearkit.GearSpec;
import gearkit.HobCheck;
import gearkit.Involute;
import gearkit.MeshingAnalysis;
import gearkit.Point2;
import gearkit.StrainWaveInput;
import gearkit.StrainWavePair;
import gearkit.ToothProfile;

/** The D1 (involute 20 deg) against D3 (involute 30 deg, kappa 0.8 to 1.2) comparison that decides
    which pressure angle to order a hob for. Prints a table; every number is model output (unloaded,
    kinematic, zero-backlash envelope circular spline), not a measurement. */
class ToothStudy {
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

  public static function run():Void {
    var m = 0.3, deg = ToothDesigns.DEG;
    var max20 = HobCheck.maxCutterTipRadius(r -> {
      return new GearSpec(m, 200, 8, new Involute(20 * deg), false, 0, r).profile();
    }, m);
    var max30 = HobCheck.maxCutterTipRadius(r -> {
      return new GearSpec(m, 200, 8, new Involute(30 * deg), false, 0, r).profile();
    }, m);
    Sys.println('  largest hob corner radius that leaves a cutter land (m 0.3, Z 200, full depth): 20 deg ${f(max20)} mm (${f(max20 / m)} m), 30 deg ${f(max30)} mm (${f(max30 / m)} m)');
    var rho30 = 0.9 * max30;
    var cases:Array<{label:String, input:StrainWaveInput, rho:Null<Float>}> = [];
    for (k in [0.8, 1.0, 1.1, 1.2]) cases.push({label: "D1 20deg rho 0.38m", input: new StrainWaveInput("D1", "involute", m, 200, 202, k, "cosine", 20 * deg), rho: null});
    for (k in [0.8, 0.9, 1.0, 1.1, 1.2]) cases.push({label: 'D3 30deg rho ${f(rho30 / m, 2)}m', input: ToothDesigns.d3(k), rho: rho30});
    // Same hob corner radius on both angles, so the angle is the only difference.
    for (k in [1.0, 1.1]) cases.push({label: 'D1 20deg rho ${f(rho30 / m, 2)}m', input: new StrainWaveInput("D1", "involute", m, 200, 202, k, "cosine", 20 * deg), rho: rho30});
    // The default 0.38 m hob corner radius (the reference hob's) cannot cut 30 deg at full depth.
    try {
      new StrainWavePair(ToothDesigns.d3(1.0));
      Sys.println("  D3 30deg with the default 0.38 m corner radius: builds");
    } catch (e:Dynamic) {
      Sys.println('  D3 30deg with the default 0.38 m corner radius: ${Std.string(e)}');
    }
    for (c in cases) {
      var pair = new StrainWavePair(c.input, c.rho);
      var r = MeshingAnalysis.run(pair, 0.005, 24);
      var r20 = MeshingAnalysis.run(pair, 0.02, 24);
      var hob = HobCheck.problem(pair.flex);
      Sys.println('  ${c.label} kappa ${c.input.kappa}: '
        + 'interference ${r.interference.length == 0 ? "no" : "YES"}, rootClr ${f(r.rootClearance * 1000, 1)} um, minorTipClr ${f(r.minorTipClearance * 1000, 1)} um, '
        + 'CS tip trim ${f((r.derivedTipRadius - pair.circularTipRadius()) * 1000, 1)} um, engaged(5um) ${f(r.engagedFraction * 100, 1)}%, engaged(20um) ${f(r20.engagedFraction * 100, 1)}%, zone(5um) +-${f(r.contactZone * 180 / Math.PI, 0)} deg, '
        + 'rootFilletRmin ${f(minRootCurvatureRadius(pair.profile), 4)} mm, tipLand ${f(tipLand(pair.profile), 4)} mm, hobbable ${hob == null ? "yes" : "NO " + hob}');
    }
  }
}
