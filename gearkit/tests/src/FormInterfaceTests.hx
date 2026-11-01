import gearkit.ConjugateRack;
import gearkit.GearEvaluationRange;
import gearkit.GearInspection;
import gearkit.GearMeasurement;
import gearkit.GearSpec;
import gearkit.Iso1328;
import gearkit.HobDesign;
import gearkit.Involute;
import gearkit.Iso1328;
import gearkit.Point2;
import gearkit.ToothForm;
import gearkit.ToothProfile;
import gearkit.ToothRegion;

/** Hides an involute behind the bare ToothForm/ToothProfile interfaces, so any code that
    sneaks in an involute assumption (a downcast, a base circle) fails on it. */
class OpaqueForm implements ToothForm {
  final inner:Involute;
  public function new(inner:Involute) this.inner = inner;
  public function name():String return "opaque";
  public function pressureAngle():Float return inner.pressureAngle();
  public function addendum(module:Float):Float return inner.addendum(module);
  public function dedendum(module:Float):Float return inner.dedendum(module);
  public function pitchToothThickness(spec:GearSpec):Float return inner.pitchToothThickness(spec);
  public function validate(spec:GearSpec):Void inner.validate(spec);
  public function profile(spec:GearSpec):ToothProfile {
    // The wrapped involute computes on a twin spec that carries the real involute form.
    var twin = new GearSpec(spec.module, spec.teeth, spec.faceWidth, inner, spec.internal, spec.profileShift,
      spec.cutterTipRadius, spec.tipFilletRadius, spec.evaluation);
    return new OpaqueProfile(spec, inner.profile(twin));
  }
}

class OpaqueProfile implements ToothProfile {
  final spec:GearSpec;
  final p:ToothProfile;
  public function new(spec:GearSpec, p:ToothProfile) {
    this.spec = spec;
    this.p = p;
  }
  public function gearSpec():GearSpec return spec;
  public function rootJoinRadius():Float return p.rootJoinRadius();
  public function tipJoinRadius():Float return p.tipJoinRadius();
  public function flank(t:Float):Point2 return p.flank(t);
  public function flankDerivative(t:Float):Point2 return p.flankDerivative(t);
  public function flankAtRadius(radius:Float):Point2 return p.flankAtRadius(radius);
  public function rootFillet(t:Float):Point2 return p.rootFillet(t);
  public function rootDerivative(t:Float):Point2 return p.rootDerivative(t);
  public function tipFillet(t:Float):Point2 return p.tipFillet(t);
  public function tipDerivative(t:Float):Point2 return p.tipDerivative(t);
  public function regionAt(radius:Float):ToothRegion return p.regionAt(radius);
  public function evaluationRange():GearEvaluationRange return p.evaluationRange();
  public function outline(samples:Int):Array<Point2> return p.outline(samples);
  public function halfAngleAt(radius:Float):Float return p.halfAngleAt(radius);
  public function profileCoordinate(radius:Float):Float return p.profileCoordinate(radius);
  public function radiusAtCoordinate(coordinate:Float):Float return p.radiusAtCoordinate(coordinate);
  public function normalLever(radius:Float):Float return p.normalLever(radius);
}

/** TD1: the rack, hob and metrology see the tooth only through the form interface, so the same
    involute reached through an opaque form gives identical numbers. */
class FormInterfaceTests {
  static var assertions = 0;

  static function check(ok:Bool, message:String):Void {
    assertions++;
    if (!ok) throw message;
  }
  static function same(a:Float, b:Float, message:String):Void check(a == b, '$message: $a versus $b');
  static function samePoints(a:Array<Point2>, b:Array<Point2>, message:String):Void {
    check(a.length == b.length, '$message: length');
    for (i in 0...a.length) check(a[i].x == b[i].x && a[i].y == b[i].y, '$message: point $i');
  }

  public static function run():Void {
    var alpha = 20 * Math.PI / 180;
    for (cfg in [[0.3, 200, 8.0], [0.5, 100, 10.0]]) {
      var m:Float = cfg[0], z = Std.int(cfg[1]), b:Float = cfg[2];
      var inv = new Involute(alpha);
      var plain = new GearSpec(m, z, b, inv), opaque = new GearSpec(m, z, b, new OpaqueForm(inv));
      check(Std.isOfType(opaque.profile(), OpaqueProfile), "opaque spec builds its profile through the form");
      same(opaque.tipRadius(), plain.tipRadius(), "tip radius");
      same(opaque.rootRadius(), plain.rootRadius(), "root radius");
      same(opaque.pitchToothThickness(), plain.pitchToothThickness(), "pitch tooth thickness");
      same(opaque.pressureAngle, plain.pressureAngle, "pressure angle");
      var pp = plain.profile(), op = opaque.profile();
      samePoints(op.outline(64), pp.outline(64), "outline");
      var rackP = ConjugateRack.of(pp, 128), rackO = ConjugateRack.of(op, 128);
      samePoints(rackO.flankSamples(), rackP.flankSamples(), "rack flank");
      samePoints(rackO.cornerSamples(), rackP.cornerSamples(), "rack corner");
      samePoints(rackO.rootSamples(), rackP.rootSamples(), "rack root");
      var hobP = new HobDesign(rackP, 1, 32, 12), hobO = new HobDesign(rackO, 1, 32, 12);
      same(hobO.pitchDiameter, hobP.pitchDiameter, "hob pitch diameter");
      same(hobO.lead, hobP.lead, "hob lead");
      same(hobO.rootHeight, hobP.rootHeight, "hob root height");
      var fp = hobP.rakeFaceRegion(3), fo = hobO.rakeFaceRegion(3);
      check(fp.length == fo.length, "hob face length");
      for (i in 0...fp.length) check(fp[i].x == fo[i].x && fp[i].y == fo[i].y && fp[i].z == fo[i].z, "hob face point");
      var control = pp.rootJoinRadius() + 0.02 * (pp.tipJoinRadius() - pp.rootJoinRadius());
      var ip = new GearInspection(plain, control), io = new GearInspection(opaque, control);
      same(io.rollStart, ip.rollStart, "roll start");
      same(io.rollEnd, ip.rollEnd, "roll end");
      same(io.rollTip, ip.rollTip, "roll tip");
      same(io.profileCutoff, ip.profileCutoff, "profile cutoff");
      // Rings are generated from the involute spec (its analytic fixture), measured through each.
      var rings = new SyntheticGearRings(plain, control, 2e-4, 3.0, 1.0);
      var mp = GearMeasurement.fromRings(rings, plain, ip), mo = GearMeasurement.fromRings(rings, opaque, io);
      for (key in mp.values.keys()) {
        if (key == "Fr") continue; // Annex E.5 ball reconstruction is involute-only by definition
        check(mo.values.exists(key), 'opaque measurement lacks $key');
        same(mo.value(key), mp.value(key), 'measured $key');
      }
      check(!mo.values.exists("Fr") && mp.values.exists("Fr"), "involute-only runout is skipped for other forms");
      check(mo.warnings.length == mp.warnings.length + 1, "skipped runout is disclosed");
      for (g in Iso1328.parameters) if (g != "Fr") check(
        mo.grades.get(g) == mp.grades.get(g), 'grade $g');
      check(Iso1328.tolerance(opaque, "Falpha", 5) == Iso1328.tolerance(plain, "Falpha", 5), "ISO tolerance");
    }
    var rejected = "";
    try new GearSpec(0.3, 200, 8, null) catch (e:Dynamic) rejected = Std.string(e);
    check(rejected.indexOf("tooth form") >= 0, "a gear needs a tooth form");
    rejected = "";
    try new GearSpec(0.3, 200, 8, new OpaqueForm(new Involute(alpha))).baseRadius() catch (e:Dynamic) rejected = Std.string(e);
    check(rejected.indexOf("no base circle") >= 0, "base circle only exists for involute forms");
    Sys.println('FormInterfaceTests: $assertions assertions passed');
  }
}
