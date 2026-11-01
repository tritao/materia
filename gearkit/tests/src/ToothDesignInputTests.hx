import gearkit.StrainWavePair;
import gearkit.StrainWaveInput;

class ToothDesignInputTests {
  static var assertions = 0;

  static function check(ok:Bool, message:String):Void {
    assertions++;
    if (!ok) throw message;
  }
  static function near(a:Float, b:Float, tolerance:Float, message:String):Void check(
    Math.isFinite(a) && Math.abs(a - b) <= tolerance, '$message: $a versus $b');
  static function angle(i:StrainWaveInput):Float {
    var a = i.pressureAngle;
    if (a == null) throw "involute design without a pressure angle";
    return a;
  }
  static function rejects(f:Void -> Void, fragment:String):Void {
    var error = "";
    try f() catch (e:Dynamic) error = Std.string(e);
    check(error.indexOf(fragment) >= 0, 'expected "$fragment", got "$error"');
  }

  public static function run():Void {
    var d1 = ToothDesigns.d1(), d2 = ToothDesigns.d2(), d3 = ToothDesigns.d3(), d4 = ToothDesigns.d4();
    // D1 is the gear the hobber cuts (FlexsplineHobber default: m 0.3, 200 teeth, 20 deg).
    check(d1.form == "involute" && d1.flexTeeth == 200 && d1.circularTeeth == 202, "D1 tooth counts");
    near(d1.module, 0.3, 0, "D1 module");
    near(angle(d1), 20 * ToothDesigns.DEG, 1e-15, "D1 pressure angle");
    near(d1.w0(), 0.3, 1e-15, "D1 w0 = kappa m (Zc-Zf)/2");
    near(d1.flexPitchRadius(), 30, 1e-12, "D1 flexspline pitch radius");
    near(d1.circularPitchRadius(), 30.3, 1e-12, "D1 circular spline pitch radius");
    // D2 is Liu 2025's defaults: w0 0.5 at m 0.5, 100/102.
    check(d2.form == "double-circular-arc" && d2.pressureAngle == null, "D2 form");
    near(d2.w0(), 0.5, 1e-15, "D2 w0 from Liu defaults");
    check(d2.flexTeeth == 100 && d2.circularTeeth == 102, "D2 tooth counts");
    // D3 is D1 at 30 degrees with kappa swept 0.8..1.2.
    near(angle(d3), 30 * ToothDesigns.DEG, 1e-15, "D3 pressure angle");
    var sweep = ToothDesigns.d3Sweep();
    check(sweep.length == 5, "D3 sweep size");
    near(sweep[0].kappa, 0.8, 0, "D3 sweep start");
    near(sweep[4].kappa, 1.2, 0, "D3 sweep end");
    for (s in sweep) near(s.w0(), s.kappa * 0.3, 1e-15, "D3 w0");
    // D4 is valid as an input but outside the literature range, so TD5/TD6 must refuse it.
    check(d4.kappa > 1.2, "D4 kappa is above the swept range");
    for (d in [d1, d2, d3, d4]) check(d.cam == "cosine", "all test designs use the cosine cam");

    rejects(() -> new StrainWaveInput("x", "involute", 0.3, 200, 201, 1, "cosine", 0.35), "two more teeth");
    rejects(() -> new StrainWaveInput("x", "involute", 0.3, 200, 202, 0, "cosine", 0.35), "kappa");
    rejects(() -> new StrainWaveInput("x", "involute", 0.3, 200, 202, Math.NaN, "cosine", 0.35), "kappa");
    rejects(() -> new StrainWaveInput("x", "involute", 0, 200, 202, 1, "cosine", 0.35), "module");
    rejects(() -> new StrainWaveInput("x", "involute", 0.3, 10, 12, 1, "cosine", 0.35), "twenty teeth");
    rejects(() -> new StrainWavePair(new StrainWaveInput("x", "involute", 0.3, 200, 202, 1, "triangle", 0.35)), "unknown cam");
    rejects(() -> new StrainWaveInput("x", "spiral", 0.3, 200, 202, 1, "cosine", 0.35), "tooth form");
    rejects(() -> new StrainWaveInput("x", "involute", 0.3, 200, 202, 1, "cosine"), "pressure angle");
    rejects(() -> new StrainWaveInput("x", "involute", 0.3, 200, 202, 1, "cosine", Math.PI / 2), "pressure angle");
    rejects(() -> new StrainWaveInput("x", "double-circular-arc", 0.5, 100, 102, 1, "cosine", 0.35), "own flank");
    Sys.println('ToothDesignInputTests: $assertions assertions passed');
  }
}
