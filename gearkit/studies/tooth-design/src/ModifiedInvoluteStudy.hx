import gearkit.CircularSpline;
import gearkit.ConjugateRack;
import gearkit.FixedMesh;
import gearkit.FixedMeshReport;
import gearkit.GearSpec;
import gearkit.HobCheck;
import gearkit.Involute;
import gearkit.MeshErrors;
import gearkit.Point2;
import gearkit.StrainWaveInput;
import gearkit.StrainWavePair;
import gearkit.TipRelief;
import gearkit.ToothProfile;

/** Can a modified 20 deg involute flexspline (shortened tooth, profile shift, tip relief or a larger tip rounding) mesh
    a fixed circular spline, either a derived one or a nominal involute one (TOOTH_DESIGN_PLAN.md, "TD12")? Variants are
    given on the command line as `ha=0.7,x=0.5,tip=d`, where `tip` is `d` (default 0.05 m rounding), `r150` (rounding of
    0.150 m) or `c20x120` (linear relief of 20 um at the tip over 120 um of radius, default rounding; `c20x120p2` a quadratic one). Every number is
    model output: unloaded, rigid teeth, one rotational degree of freedom, cosine cam, 12 wave-generator angles.
    Modes: `mod-search spec...` (coarse rows per variant), `mod-detail spec...` (full sweep), `mod-hob spec...` (rack and tooth room). */
class ModifiedInvoluteStudy {
  static final DEG = Math.PI / 180;
  static final M = 0.3;
  /** Wave-generator angles over one circular pitch; the `SAMPLES` environment variable overrides it (a resolution check). */
  static function samples():Int {
    var e = Sys.getEnv("SAMPLES");
    return e == null ? 12 : Std.parseInt(e);
  }

  static function f(x:Float, digits:Int = 3):String {
    var k = Math.pow(10, digits);
    return Std.string(Math.round(x * k) / k);
  }

  static function parseTip(tip:String):{rounding:Null<Float>, relief:Null<TipRelief>} {
    var rounding:Null<Float> = null, relief:Null<TipRelief> = null;
    if (tip.charAt(0) == "r") rounding = Std.parseFloat(tip.substr(1)) / 1000 * M;
    else if (tip.charAt(0) == "c") {
      var q = tip.substr(1).split("x"), e = q[1].split("p");
      relief = new TipRelief(Std.parseFloat(q[0]) / 1000, Std.parseFloat(e[0]) / 1000, e.length > 1 ? Std.parseFloat(e[1]) : 1.0);
    } else if (tip != "d") throw 'unknown tip $tip';
    return {rounding: rounding, relief: relief};
  }

  /** `cha` and `ctip` give the nominal circular spline its own addendum factor and tip treatment (default: the flexspline's). */
  public static function parse(spec:String):{ha:Float, x:Float, tip:String, rounding:Null<Float>, relief:Null<TipRelief>, cha:Float, ctip:String} {
    var ha = 1.0, x = 0.0, tip = "d", cha:Null<Float> = null, ctip:Null<String> = null;
    for (kv in spec.split(",")) {
      var p = kv.split("=");
      switch p[0] {
        case "ha": ha = Std.parseFloat(p[1]);
        case "x": x = Std.parseFloat(p[1]);
        case "tip": tip = p[1];
        case "cha": cha = Std.parseFloat(p[1]);
        case "ctip": ctip = p[1];
        default: throw 'unknown variant key ${p[0]}';
      }
    }
    var t = parseTip(tip);
    return {ha: ha, x: x, tip: tip, rounding: t.rounding, relief: t.relief, cha: cha == null ? ha : cha, ctip: ctip == null ? tip : ctip};
  }

  static function flexInput(spec:String, kappa:Float):StrainWaveInput {
    var v = parse(spec);
    return new StrainWaveInput("D1mod", "involute", M, 200, 202, kappa, "cosine", 20 * DEG, v.ha, v.x, v.relief);
  }

  static function flexPair(spec:String, kappa:Float):StrainWavePair return new StrainWavePair(flexInput(spec, kappa), null, parse(spec).rounding);

  /** Nominal involute internal spline for a variant: same addendum factor and tip treatment, shift x + delta (delta thins the
      circular-spline tooth, i.e. opens the backlash: 2 delta m tan(20 deg) on the pitch circle). */
  static function nominalSpline(spec:String, delta:Float):CircularSpline {
    var v = parse(spec), t = parseTip(v.ctip);
    return CircularSpline.involute('nominal ha ${v.cha} x ${f(v.x + delta, 3)} ${v.ctip}', M, 202, 20 * DEG, v.x + delta, null, t.rounding, v.cha, t.relief);
  }

  static function row(tag:String, spec:String, spline:CircularSpline, flexK:Float, amp:Float):Void {
    var t1 = Sys.time();
    try {
      var r = FixedMesh.run(spline, flexPair(spec, flexK), new MeshErrors(amp), samples());
      var um = M * 202 / 2 * 1000, issues = [];
      if (r.rootBottoming > 1e-6) issues.push('root ${f(r.rootBottoming * 1000, 1)}');
      if (r.radialOverlap > 1e-6) issues.push('tipoverlap ${f(r.radialOverlap * 1000, 1)}');
      if (r.backlashMin < 0) issues.push('overlap');
      if (r.unresolved > 0) issues.push('unresolved${r.unresolved}');
      Sys.println('ROW\t$spec\t$tag\tflexK=${flexK}\tamp=${f(amp * 1000, 0)}\tTE=${f(r.tePeakToPeakArcsec(), 2)}\tbl=${f(r.backlashMean * um, 1)}\tblmin=${f(r.backlashMin * um, 1)}\tblmax=${f(r.backlashMax * um, 1)}\tflank=${f(r.meanFlankContacts, 1)}\ttip=${f(r.meanTipContacts, 1)}\tissues=${issues.length == 0 ? "none" : issues.join("+")}\ts=${f(Sys.time() - t1, 1)}');
    } catch (e:Dynamic) Sys.println('ROW\t$spec\t$tag\tflexK=${flexK}\tamp=${f(amp * 1000, 0)}\tFAILED\t${Std.string(e)}');
  }

  /** Minimum root-fillet curvature radius over the profile (mm). */
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

  static function maxCutterTipRadius(make:Float -> ToothProfile):Float {
    var lo = 0.0, hi = 0.99 * M;
    for (_ in 0...40) {
      var mid = (lo + hi) / 2, ok = true;
      try make(mid) catch (e:Dynamic) ok = false;
      if (ok) lo = mid;
      else hi = mid;
    }
    return lo;
  }

  /** Geometry of a variant's flexspline tooth: tip land, thickness at the pitch circle, root fillet room and the hob rack. */
  static function geometry(spec:String):Void {
    var v = parse(spec);
    try {
      var pair = flexPair(spec, 1.0), p = pair.profile, g = pair.flex;
      var tipEnd = p.tipFillet(1), land = 2 * g.tipRadius() * Math.atan2(tipEnd.x, tipEnd.y);
      var tj = p.flankAtRadius(p.tipJoinRadius()), joinThick = 2 * tj.x;
      var rootEnd = p.rootFillet(1), rootThick = 2 * g.rootRadius() * Math.atan2(rootEnd.x, rootEnd.y);
      var space = 2 * Math.PI * g.rootRadius() / g.teeth - rootThick;
      var maxCorner = maxCutterTipRadius(r -> new GearSpec(M, 200, 8, g.form, false, g.profileShift, r, g.tipFilletRadius).profile());
      var rack = ConjugateRack.of(p, 256), fl = rack.flankSamples();
      var slope = -Math.tan(20 * DEG), lo = Math.POSITIVE_INFINITY, hi = Math.NEGATIVE_INFINITY;
      for (q in fl) {
        var d = q.x - slope * q.y;
        lo = Math.min(lo, d);
        hi = Math.max(hi, d);
      }
      // Height up to which the rack flank is the straight 20 deg line (within 0.5 um), and the largest departure above it.
      var base = fl[0].x - slope * fl[0].y, straightTo = fl[0].y, worst = 0.0;
      for (q in fl) {
        var dev = Math.abs(q.x - slope * q.y - base);
        if (dev <= 0.0005 && q.y >= straightTo) straightTo = q.y;
        worst = Math.max(worst, dev);
      }
      var hob = HobCheck.problem(g);
      Sys.println('GEOM\t$spec\ttipR=${f(g.tipRadius(), 4)}\trootR=${f(g.rootRadius(), 4)}\ttipLand=${f(land * 1000, 1)}um (${f(land / M, 3)}m)\tthickAtTipJoin=${f(joinThick * 1000, 1)}um\tpitchThick=${f(g.pitchToothThickness() / M, 3)}m\t'
        + 'rootToothThick=${f(rootThick / M, 3)}m\trootSpace=${f(space / M, 3)}m\trootFilletRmin(0.38m corner)=${f(minRootCurvatureRadius(p) * 1000, 1)}um\tmaxHobCorner=${f(maxCorner / M, 3)}m\t'
        + 'rackFlankHeights=${f(fl[0].y / M, 3)}..${f(fl[fl.length - 1].y / M, 3)}m\track20degDeviation=${f((hi - lo) / 2 * 1000, 2)}um\track20degStraightUpTo=${f(straightTo / M, 3)}m (flank top ${f(fl[fl.length - 1].y / M, 3)}m)\trackMaxDepartureFromLine=${f(worst * 1000, 1)}um\trackDatumHalfThick=${f((hi + lo) / 2, 4)}mm\trackRootFold=${rack.hasRootFold}\thobbable=${hob == null ? "yes" : "NO " + hob}');
    } catch (e:Dynamic) Sys.println('GEOM\t$spec\tINVALID\t${Std.string(e)}');
  }

  /** Geometry of the nominal internal spline for a variant: tip land, tooth thickness, root fillet. */
  static function splineGeometry(spec:String, delta:Float):Void {
    try {
      var cs = nominalSpline(spec, delta), p = cs.profile, g = p.gearSpec();
      var tipEnd = p.tipFillet(1), land = 2 * g.tipRadius() * Math.atan2(tipEnd.x, tipEnd.y);
      var tj = p.flankAtRadius(p.tipJoinRadius());
      Sys.println('CSGEOM\t$spec\tdelta=$delta\ttipR=${f(g.tipRadius(), 4)}\trootR=${f(g.rootRadius(), 4)}\ttipLand=${f(land * 1000, 1)}um (${f(land / M, 3)}m)\tthickAtTipJoin=${f(2 * tj.x * 1000, 1)}um\tpitchThick=${f(g.pitchToothThickness() / M, 3)}m\t'
        + 'rootFilletRmin(0.38m corner)=${f(minRootCurvatureRadius(p) * 1000, 1)}um\tmaxToothHeight=${f((g.rootRadius() - g.tipRadius()) / M, 3)}m');
    } catch (e:Dynamic) Sys.println('CSGEOM\t$spec\tdelta=$delta\tINVALID\t${Std.string(e)}');
  }

  static function search(specs:Array<String>):Void {
    for (spec in specs) {
      geometry(spec);
      var ok = true;
      try flexPair(spec, 1.0) catch (e:Dynamic) ok = false;
      if (!ok) continue;
      for (kd in [1.0, 1.1]) {
        var cs:Null<CircularSpline> = null;
        try cs = CircularSpline.derived('derived kd $kd', flexPair(spec, kd)) catch (e:Dynamic) Sys.println('ROW\t$spec\tderived kd=$kd\tNOTDERIVABLE\t${Std.string(e)}');
        if (cs == null) continue;
        Sys.println('CS\t$spec\tderived kd=$kd\ttip=${f(cs.tipRadius, 4)}\troot=${f(cs.rootRadius, 4)}');
        for (dk in [0.1, 0.05]) row('derived kd=$kd', spec, cs, kd - dk, 0.0);
      }
      for (delta in [0.05, 0.15]) {
        var cs:Null<CircularSpline> = null;
        try cs = nominalSpline(spec, delta) catch (e:Dynamic) Sys.println('ROW\t$spec\tnominal delta=$delta\tINVALIDSPLINE\t${Std.string(e)}');
        if (cs == null) continue;
        for (k in [0.9, 1.0, 1.1]) row('nominal delta=$delta', spec, cs, k, 0.0);
      }
    }
  }

  /** Nominal-spline grid: each spec with delta 0.05 and flexspline kappa 0.9 / 1.0 / 1.1 at 0 um. */
  static function nominalGrid(specs:Array<String>):Void {
    for (spec in specs) {
      var cs:Null<CircularSpline> = null;
      try cs = nominalSpline(spec, 0.05) catch (e:Dynamic) Sys.println('ROW\t$spec\tnominal delta=0.05\tINVALIDSPLINE\t${Std.string(e)}');
      if (cs == null) continue;
      for (k in [0.9, 1.0, 1.1]) row('nominal delta=0.05', spec, cs, k, 0.0);
    }
  }

  static function detail(specs:Array<String>):Void {
    for (spec in specs) {
      geometry(spec);
      splineGeometry(spec, 0.05);
      for (kd in [0.9, 1.0, 1.1, 1.2]) {
        var cs:Null<CircularSpline> = null;
        try cs = CircularSpline.derived('derived kd $kd', flexPair(spec, kd)) catch (e:Dynamic) Sys.println('ROW\t$spec\tderived kd=$kd\tNOTDERIVABLE\t${Std.string(e)}');
        if (cs == null) continue;
        for (dk in [0.1, 0.05, 0.0]) for (amp in [-0.002, 0.0, 0.002]) row('derived kd=$kd', spec, cs, kd - dk, amp);
      }
      for (delta in [0.0, 0.05, 0.1, 0.15, 0.25]) {
        var cs:Null<CircularSpline> = null;
        try cs = nominalSpline(spec, delta) catch (e:Dynamic) Sys.println('ROW\t$spec\tnominal delta=$delta\tINVALIDSPLINE\t${Std.string(e)}');
        if (cs == null) continue;
        for (k in [0.8, 0.9, 1.0, 1.1, 1.2]) for (amp in [-0.002, 0.0, 0.002]) row('nominal delta=$delta', spec, cs, k, amp);
      }
    }
  }

  public static function run(mode:String, specs:Array<String>):Void {
    switch mode {
      case "mod-search": search(specs);
      case "mod-detail": detail(specs);
      case "mod-hob": for (s in specs) {
          geometry(s);
          splineGeometry(s, 0.05);
        }
      case "mod-nominal": nominalGrid(specs);
      default: throw 'unknown mode $mode';
    }
  }
}
