import gearkit.GearSpec;
import gearkit.HobCheck;
import gearkit.Involute;
import gearkit.MeshingAnalysis;
import gearkit.MeshingReport;
import gearkit.StrainWaveInput;
import gearkit.StrainWavePair;

/** TD5: unloaded kinematic meshing of the derived pairs. */
class MeshingTests {
  static var assertions = 0;
  public static final THRESHOLD = 0.005; // mm, assumed contact threshold (the plan's ~5 um backlash class)

  static function check(ok:Bool, message:String):Void {
    assertions++;
    if (!ok) throw message;
  }

  static function f(x:Float, digits:Int = 4):String {
    var k = Math.pow(10, digits);
    return Std.string(Math.round(x * k) / k);
  }

  public static function row(r:MeshingReport, hob:Null<String>):String {
    return '${r.name}: rootClr ${f(r.rootClearance * 1000, 1)} um, minorTipClr ${f(r.minorTipClearance * 1000, 1)} um, '
      + 'derivedTip ${f(r.derivedTipRadius, 4)}, '
      + 'penetration ${f(r.penetration * 1000, 2)} um (bound ${f(r.fitBound * 1000, 2)}), '
      + 'flank engaged ${f(r.flankEngagedFraction * 100, 1)}% (max ${r.maxFlankEngaged}, zone +-${f(r.contactZone * 180 / Math.PI, 1)} deg, min clr ${f(r.minFlankClearance * 1000, 1)} um), '
      + 'tip edge contact ${f(r.tipContactFraction * 100, 1)}% (max ${r.maxTipContact}, min clr ${f(r.minTipClearance * 1000, 1)} um), '
      + 'interference ${r.interference.length == 0 ? "no" : "YES"}, hobbable ${hob == null ? "yes" : "NO"}';
  }

  public static function run():Void {
    var d1 = new StrainWavePair(ToothDesigns.d1());
    var r1 = MeshingAnalysis.run(d1, THRESHOLD);
    Sys.println('  ' + row(r1, HobCheck.problem(d1.flex)));
    check(r1.interference.length == 0, 'D1 has no interference: ${r1.interference}');
    check(r1.penetration <= r1.fitBound, "D1 penetration within the fit bound");
    check(r1.flankEngagedFraction > 0 && r1.flankEngagedFraction < 1, "D1 flank engaged fraction in (0, 1)");
    check(r1.rootClearance > 0 && r1.minorTipClearance >= -r1.fitBound, "D1 radial clearances");
    check(HobCheck.problem(d1.flex) == null, "D1 is hobbable");

    // D2 (Liu 2025 optimum): the unscaled paper case and the same shape scaled to our gear, both free of interference.
    for (input in [ToothDesigns.d2(), ToothDesigns.d2Scaled()]) {
      var pair = new StrainWavePair(input);
      var r = MeshingAnalysis.run(pair, THRESHOLD);
      var hob = HobCheck.problem(pair.flex);
      Sys.println('  ' + row(r, hob));
      check(r.interference.length == 0, '${input.name} has no interference: ${r.interference}');
      check(r.penetration <= r.fitBound, '${input.name} penetration within the fit bound');
      check(r.flankEngagedFraction > 0 && r.flankEngagedFraction < 1, '${input.name} flank engaged fraction in (0, 1)');
      check(r.rootClearance > 0 && r.minorTipClearance > 0, '${input.name} radial clearances');
      check(hob == null, '${input.name} is hobbable: $hob');
    }
    // D2 bottoms in the root above kappa ~1.12 (root clearance hf - ha = 0.122 m); named as kappa too large.
    var d2hi = MeshingAnalysis.run(new StrainWavePair(ToothDesigns.d2Scaled(1.2)), THRESHOLD, 16);
    check(d2hi.interference.length > 0 && d2hi.interference[0].indexOf("kappa too large") >= 0, "D2 at kappa 1.2 is refused, naming kappa");

    // D4: a deliberately bad design is refused, naming kappa, with tooth index and wave-generator angle.
    var d4 = new StrainWavePair(ToothDesigns.d4());
    var r4 = MeshingAnalysis.run(d4, THRESHOLD);
    Sys.println('  ' + row(r4, HobCheck.problem(d4.flex)));
    Sys.println('    D4 reasons: ${r4.interference.join(" | ")}');
    check(r4.interference.length > 0, "D4 is rejected");
    check(r4.interference[0].indexOf("kappa too large") >= 0, "D4 names kappa");
    check(r4.penetrationTooth >= 0 && r4.penetration > 0, "D4 reports where it interferes");
    check(r4.interference[0].indexOf("tooth " + r4.penetrationTooth) >= 0, "D4 reports the tooth index");

    // Flank engagement and tip edge contact versus kappa for D1 (informational; see TOOTH_DESIGN_PLAN.md).
    for (k in [1.0, 1.1, 1.2]) {
      var input = new StrainWaveInput("D1k", "involute", 0.3, 200, 202, k, "cosine", 20 * ToothDesigns.DEG);
      var r = MeshingAnalysis.run(new StrainWavePair(input), THRESHOLD, 16);
      Sys.println('    D1 kappa $k: flank engaged ${f(r.flankEngagedFraction * 100, 1)}%, tip edge ${f(r.tipContactFraction * 100, 1)}%, rootClr ${f(r.rootClearance * 1000, 1)} um, derivedTip ${f(r.derivedTipRadius, 4)}');
      check(r.interference.length == 0 && r.flankEngagedFraction > 0 && r.flankEngagedFraction < 1, 'D1 at kappa $k meshes');
    }
    Sys.println('MeshingTests: $assertions assertions passed');
  }
}
