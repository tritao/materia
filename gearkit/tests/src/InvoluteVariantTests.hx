import gearkit.ConjugateRack;
import gearkit.GearSpec;
import gearkit.HobCheck;
import gearkit.Involute;
import gearkit.InvoluteProfile;
import gearkit.Point2;
import gearkit.StrainWaveInput;
import gearkit.StrainWavePair;
import gearkit.TipRelief;

/** Involute variants: shortened addendum, profile shift beyond one module, tip relief. */
class InvoluteVariantTests {
  static var assertions = 0;

  static function check(ok:Bool, message:String):Void {
    assertions++;
    if (!ok) throw message;
  }
  static function near(a:Float, b:Float, tolerance:Float, message:String):Void check(
    Math.isFinite(a) && Math.abs(a - b) <= tolerance, '$message: $a versus $b');
  static function rejects(f:Void -> Void, fragment:String):Void {
    var error = "";
    try f() catch (e:Dynamic) error = Std.string(e);
    check(error.indexOf(fragment) >= 0, 'expected "$fragment", got "$error"');
  }

  public static function run():Void {
    var a = 20 * Math.PI / 180, m = 0.3;
    var base = new GearSpec(m, 200, 8, new Involute(a));
    // Defaults are the full-depth form.
    near(new Involute(a, 1.0).addendum(m), m, 0, "default addendum");
    // A shortened tooth lowers the tip circle only; the root, the rack and the flank are those of the full tooth.
    var short = new GearSpec(m, 200, 8, new Involute(a, 0.7));
    near(short.tipRadius(), 30 + 0.7 * m, 1e-12, "shortened tip radius");
    near(short.rootRadius(), base.rootRadius(), 0, "shortened root radius");
    var rb = base.profile(), rs = short.profile();
    near(rs.flankAtRadius(rb.rootJoinRadius() + 0.1).x, rb.flankAtRadius(rb.rootJoinRadius() + 0.1).x, 1e-12, "shortened flank is the full flank");
    check(HobCheck.problem(short) == null, "shortened tooth is hobbable");
    // Profile shift beyond one module (the old bound) builds, thickens the tooth and moves the circles out.
    var shifted = new GearSpec(m, 200, 8, new Involute(a, 0.7), false, 1.5);
    near(shifted.tipRadius(), 30 + 2.2 * m, 1e-12, "shifted tip radius");
    near(shifted.pitchToothThickness(), Math.PI * m / 2 + 2 * 1.5 * m * Math.tan(a), 1e-12, "shifted thickness");
    rejects(() -> new GearSpec(m, 200, 8, new Involute(a), false, 2.0), "profile shift");
    rejects(() -> new Involute(a, 0), "addendum factor");
    rejects(() -> new TipRelief(0, 0.1), "amount");
    rejects(() -> new TipRelief(0.01, 0), "length");
    // Tip relief: no change below its start, `amount` at the tip circle, smooth tip rounding, derivative consistent.
    for (internal in [false, true]) {
      var teeth = internal ? 202 : 200, sign = internal ? -1.0 : 1.0;
      var relief = new TipRelief(0.02, 0.12, 1.0);
      var plain = new GearSpec(m, teeth, 8, new Involute(a), internal), cut = new GearSpec(m, teeth, 8, new Involute(a, 1.0, relief), internal);
      var pp = plain.profile(), pc = cut.profile();
      var tip = plain.tipRadius(), start = tip - sign * 0.12;
      near(pc.flankAtRadius(start - sign * 0.01).x, pp.flankAtRadius(start - sign * 0.01).x, 1e-12, "no relief below the start");
      // Removal along the normal at radius r is (half-angle difference) * base radius.
      var rbase = Involute.baseRadius(plain), r = tip - sign * 0.04;
      near((pp.halfAngleAt(r) - pc.halfAngleAt(r)) * rbase, relief.removal(0.12 - 0.04), 1e-12, "relief removed along the normal");
      near((pp.halfAngleAt(tip) - pc.halfAngleAt(tip)) * rbase, 0.02, 1e-12, "relief amount at the tip circle");
      // The flank derivative matches finite differences of the flank, and the tip rounding is tangent to the flank.
      for (t in [0.2, 0.6, 0.95]) {
        var h = 1e-6, p0 = pc.flank(t), d = pc.flankDerivative(t);
        var rr = p0.radius(), q1 = pc.flankAtRadius(rr + h), q0 = pc.flankAtRadius(rr - h);
        near((q1.x - q0.x) / (2 * h), d.x, 1e-6, "relief flank derivative x");
        near((q1.y - q0.y) / (2 * h), d.y, 1e-6, "relief flank derivative y");
      }
      var end = pc.flank(1), tf = pc.tipFillet(0), td = pc.tipDerivative(0), fd = pc.flankDerivative(1);
      near(end.x, tf.x, 1e-12, "tip rounding meets the relieved flank x");
      near(end.y, tf.y, 1e-12, "tip rounding meets the relieved flank y");
      var cross = (td.x * fd.y - td.y * fd.x) / (td.radius() * fd.radius());
      near(cross, 0, 1e-9, "tip rounding is tangent to the relieved flank");
    }
    // The relieved flank is hobbable, and its rack is the straight 20 degree flank below the relief.
    var relieved = new GearSpec(m, 200, 8, new Involute(a, 0.8, new TipRelief(0.02, 0.12)));
    check(HobCheck.problem(relieved) == null, "relieved tooth is hobbable");
    var rack = ConjugateRack.of(relieved.profile(), 256), flank = rack.flankSamples();
    var slope = -Math.tan(a), first = flank[0];
    near(flank[Std.int(flank.length / 2)].x - first.x, slope * (flank[Std.int(flank.length / 2)].y - first.y), 1e-9, "rack flank straight 20 degrees below the relief");
    // Strain-wave inputs carry the variant; the default input builds the default tooth.
    var input = new StrainWaveInput("v", "involute", m, 200, 202, 1, "cosine", a, 0.7, 0.5, new TipRelief(0.01, 0.1));
    var pair = new StrainWavePair(input);
    near(pair.flex.tipRadius(), 30 + 1.2 * m, 1e-12, "pair flexspline tip radius");
    near(pair.circularTipRadius(), 30.3 - m, 1e-12, "nominal circular-spline tip stays full depth for an involute");
    rejects(() -> new StrainWaveInput("x", "double-circular-arc", 0.5, 100, 102, 1, "cosine", null, 0.7), "own tooth height");
    Sys.println('InvoluteVariantTests: $assertions assertions passed');
  }
}
