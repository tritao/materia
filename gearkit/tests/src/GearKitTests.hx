import gearkit.GearSpec;

class GearKitTests {
  static var assertions = 0;

  static function check(ok:Bool, message:String):Void {
    assertions++;
    if (!ok) throw message;
  }

  static function rejects(make:Void -> GearSpec):Void {
    var rejected = false;
    try make() catch (_:Dynamic) rejected = true;
    check(rejected, "invalid nominal gear dimensions must be rejected");
  }

  public static function main():Void {
    var alpha = 20 * Math.PI / 180;
    var demo = new GearSpec(1, 40, 10, alpha);
    var reference = new GearSpec(0.3, 200, 8, alpha);
    var comparison = new GearSpec(0.5, 100, 15, alpha);
    check(Math.abs(demo.pitchDiameter() - 40) < 1e-12, "demo pitch diameter");
    check(Math.abs(reference.pitchDiameter() - 60) < 1e-12, "reference pitch diameter");
    check(Math.abs(comparison.pitchDiameter() - 50) < 1e-12, "comparison pitch diameter");
    for (bad in [0.0, -1.0, Math.NaN, Math.POSITIVE_INFINITY]) {
      rejects(() -> new GearSpec(bad, 200, 8, alpha));
      rejects(() -> new GearSpec(0.3, 200, bad, alpha));
      rejects(() -> new GearSpec(0.3, 200, 8, bad));
    }
    rejects(() -> new GearSpec(0.3, 2, 8, alpha));
    rejects(() -> new GearSpec(0.3, 200, 8, Math.PI / 2));
    rejects(() -> new GearSpec(1e308, 200, 8, alpha));
    GeometryTests.run();
    Sys.println('GearKitTests: $assertions assertions passed');
  }
}
