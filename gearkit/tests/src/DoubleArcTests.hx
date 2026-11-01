import gearkit.ConjugateRack;
import gearkit.DoubleCircularArc;
import gearkit.GearSpec;
import gearkit.HobCheck;
import gearkit.HobDesign;
import gearkit.Point2;
import gearkit.ToothProfile;

/** TD2: the double-circular-arc flexspline tooth, checked against its own definitions (not the paper's
    printed equations; see TOOTH_DESIGN_PLAN.md TD2 notes). */
class DoubleArcTests {
  static var assertions = 0;

  static function check(ok:Bool, message:String):Void {
    assertions++;
    if (!ok) throw message;
  }
  static function near(a:Float, b:Float, tolerance:Float, message:String):Void check(
    Math.isFinite(a) && Math.abs(a - b) <= tolerance, '$message: $a versus $b (tol $tolerance)');
  static function rejects(f:Void -> Void, fragment:String):Void {
    var error = "";
    try f() catch (e:Dynamic) error = Std.string(e);
    check(error.indexOf(fragment) >= 0, 'expected "$fragment", got "$error"');
  }
  static function parallel(a:Point2, b:Point2, tolerance:Float, message:String):Void {
    var cross = (a.x * b.y - a.y * b.x) / (a.radius() * b.radius()), dot = a.x * b.x + a.y * b.y;
    check(Math.abs(cross) < tolerance && dot != 0, '$message: cross $cross, dot $dot');
  }
  static function same(a:Point2, b:Point2, tolerance:Float, message:String):Void near(a.distance(b), 0, tolerance, message);

  public static function liuSet(m:Float, ?edit:Array<Float> -> Void):DoubleCircularArc {
    var p = DoubleCircularArc.LIU.copy();
    for (i in 0...p.length) p[i] *= m / 0.5;
    if (edit != null) edit(p);
    return new DoubleCircularArc(m, p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
  }

  static function hyp(a:Float, b:Float):Float return Math.sqrt(a * a + b * b);

  static function proper(a:Array<Point2>):Bool {
    // Simple closed polygon: no two non-adjacent edges cross.
    var n = a.length - 1;
    function cross(o:Point2, p:Point2, q:Point2):Float return (p.x - o.x) * (q.y - o.y) - (p.y - o.y) * (q.x - o.x);
    for (i in 0...n) for (j in i + 2...n) {
      if (i == 0 && j == n - 1) continue;
      var p1 = a[i], p2 = a[i + 1], p3 = a[j], p4 = a[j + 1];
      if (cross(p1, p2, p3) * cross(p1, p2, p4) < 0 && cross(p3, p4, p1) * cross(p3, p4, p2) < 0) return false;
    }
    return true;
  }

  static function construction(f:DoubleCircularArc, label:String):Void {
    // B and C lie on their circles, and the common tangent touches both: tangency at both joins.
    near(hyp(f.bx - f.o1x, f.by - f.o1y), f.r1, 1e-12, '$label B on arc 1');
    near(hyp(f.cx - f.o2x, f.cy - f.o2y), f.r2, 1e-12, '$label C on arc 2');
    var tx = Math.sin(f.delta), ty = -Math.cos(f.delta);
    near((f.cx - f.bx) * tx + (f.cy - f.by) * ty, hyp(f.cx - f.bx, f.cy - f.by), 1e-12, '$label B to C runs along the tangent');
    near((f.bx - f.o1x) * tx + (f.by - f.o1y) * ty, 0, 1e-12, '$label tangent perpendicular to O1B');
    near((f.cx - f.o2x) * tx + (f.cy - f.o2y) * ty, 0, 1e-12, '$label tangent perpendicular to O2C');
    // Slope continuity (dx/dy) either side of B and C, computed from the circles themselves.
    var sB1 = -(f.by - f.o1y) / Math.sqrt(f.r1 * f.r1 - (f.by - f.o1y) * (f.by - f.o1y));
    var sC2 = (f.cy - f.o2y) / Math.sqrt(f.r2 * f.r2 - (f.cy - f.o2y) * (f.cy - f.o2y));
    near(sB1, -Math.tan(f.delta), 1e-9, '$label slope at B');
    near(sC2, -Math.tan(f.delta), 1e-9, '$label slope at C');
    // C0: the three pieces meet.
    near(f.xAt(f.by - 1e-13), f.xAt(f.by + 1e-13), 1e-9, '$label C0 at B');
    near(f.xAt(f.cy - 1e-13), f.xAt(f.cy + 1e-13), 1e-9, '$label C0 at C');
    near(f.dxdyAt(f.by - 1e-9), f.dxdyAt(f.by + 1e-9), 1e-8, '$label C1 at B');
    near(f.dxdyAt(f.cy - 1e-9), f.dxdyAt(f.cy + 1e-9), 1e-8, '$label C1 at C');
    // Ends: A on the addendum line, D on the dedendum line.
    near(f.xAt(f.ha), f.ax, 1e-12, '$label A x');
    near(hyp(f.ax - f.o1x, f.ha - f.o1y), f.r1, 1e-12, '$label A on arc 1');
    near(f.xAt(-f.hf), f.dx, 1e-12, '$label D x');
    near(hyp(f.dx - f.o2x, -f.hf - f.o2y), f.r2, 1e-12, '$label D on arc 2');
    // Length parametrisation round-trips; flank length is the sum of its pieces.
    for (i in 0...21) {
      var y = -f.hf + (f.ha + f.hf) * i / 20;
      near(f.heightAtLength(f.lengthAt(y)), y, 1e-12, '$label length round trip');
    }
    near(f.lengthAt(f.ha), f.r2 * (f.beta - f.delta) + f.h1 / Math.cos(f.delta) + f.r1 * (f.alpha - f.delta), 1e-12, '$label flank length');
    // The pitch line crosses the tangent: the pressure angle is delta.
    check(f.cy < 0 && f.by > 0, '$label pitch line crosses BC');
    near(f.pressureAngle(), f.delta, 1e-15, '$label pressure angle is delta');
  }

  static function profileChecks(spec:GearSpec, label:String):Void {
    var p:ToothProfile = spec.profile();
    // Joins coincide and are tangent.
    same(p.rootFillet(0), p.flank(0), 1e-12, '$label root join point');
    same(p.tipFillet(0), p.flank(1), 1e-12, '$label tip join point');
    parallel(p.rootDerivative(0), p.flankDerivative(0), 1e-9, '$label root join tangent');
    parallel(p.tipDerivative(0), p.flankDerivative(1), 1e-9, '$label tip join tangent');
    // Ends are tangent to the root and tip circles (derivative perpendicular to the radius).
    var r = p.rootFillet(1), tp = p.tipFillet(1), dr = p.rootDerivative(1), dt = p.tipDerivative(1);
    near(r.radius(), spec.rootRadius(), 1e-12, '$label root circle');
    near(tp.radius(), spec.tipRadius(), 1e-12, '$label tip circle');
    near((r.x * dr.x + r.y * dr.y) / (r.radius() * dr.radius()), 0, 1e-9, '$label root blend tangent to root circle');
    near((tp.x * dt.x + tp.y * dt.y) / (tp.radius() * dt.radius()), 0, 1e-9, '$label tip blend tangent to tip circle');
    // Flank derivative is the derivative w.r.t. radius: finite difference check.
    for (t in [0.1, 0.35, 0.5, 0.8]) {
      var rad = p.rootJoinRadius() + t * (p.tipJoinRadius() - p.rootJoinRadius()), h = 1e-6;
      var a = p.flankAtRadius(rad - h), b = p.flankAtRadius(rad + h), d = p.flankDerivative(t);
      near((b.x - a.x) / (2 * h), d.x, 1e-6, '$label flank dx/dr');
      near((b.y - a.y) / (2 * h), d.y, 1e-6, '$label flank dy/dr');
    }
    // Profile coordinate round trip and half angle.
    for (t in [0.0, 0.3, 0.7, 1.0]) {
      var rad = p.rootJoinRadius() + t * (p.tipJoinRadius() - p.rootJoinRadius());
      near(p.radiusAtCoordinate(p.profileCoordinate(rad)), rad, 1e-10, '$label coordinate round trip');
      var q = p.flankAtRadius(rad);
      near(Math.atan2(q.x, q.y), p.halfAngleAt(rad), 1e-12, '$label half angle');
      check(p.normalLever(rad) > 0 && p.normalLever(rad) <= rad, '$label normal lever');
    }
    // Normal lever against the flank-normal displacement of a small angular shift.
    var rad = (p.rootJoinRadius() + p.tipJoinRadius()) / 2, h = 1e-7;
    var q0 = p.flankAtRadius(rad), d0 = p.flankDerivative(0.5), len = d0.radius();
    var c = Math.cos(h), s = Math.sin(h);
    var shifted = new Point2(c * q0.x + s * q0.y, -s * q0.x + c * q0.y);
    var normalShift = Math.abs(((shifted.x - q0.x) * d0.y - (shifted.y - q0.y) * d0.x) / len);
    near(normalShift / h, p.normalLever(rad), 1e-5, '$label normal lever matches displacement');
    // Outline: simple polygon, one pitch, tooth positive thickness at the tip.
    var o = p.outline(24);
    check(proper(o), '$label outline has no self-intersection');
    check(Math.atan2(tp.x, tp.y) > 0, '$label tip land has positive thickness');
    check(Math.atan2(r.x, r.y) < Math.PI / spec.teeth, '$label root floor positive');
  }

  public static function run():Void {
    for (m in [0.5, 0.3]) {
      var f = liuSet(m), label = 'Liu m $m';
      construction(f, label);
      var zf = m == 0.5 ? 100 : 200;
      var spec = new GearSpec(m, zf, 8, f);
      profileChecks(spec, label);
      near(spec.pitchToothThickness(), 2 * f.xAt(0), 1e-15, '$label pitch thickness');
      near(spec.tipRadius(), spec.pitchRadius() + f.ha, 1e-12, '$label tip radius');
      near(spec.rootRadius(), spec.pitchRadius() - f.hf, 1e-12, '$label root radius');
      var hob = HobCheck.problem(spec);
      check(hob == null, '$label can be hobbed: $hob');
      var rack = ConjugateRack.of(spec.profile());
      check(!rack.hasRootFold, '$label rack root does not fold');
      new HobDesign(rack, 1, 32, 12).rakeFaceRegion(0);
      Sys.println('  $label: delta ${f.delta * 180 / Math.PI} deg, alpha ${f.alpha * 180 / Math.PI}, beta ${f.beta * 180 / Math.PI}, h1 ${f.h1}, A half-thickness ${f.ax}, D x ${f.dx}, pitch thickness ${spec.pitchToothThickness()} of ${Math.PI * m}, hob: ${hob == null ? "ok" : hob}');
    }
    // Scaling by module is exact: the shape in units of module is the paper's.
    var a = liuSet(0.5), b = liuSet(0.3);
    near(b.delta, a.delta, 1e-14, "scaled tangent angle");
    near(b.h1 / 0.3, a.h1 / 0.5, 1e-14, "scaled tangent length");
    near(DoubleCircularArc.liu(0.3).r1, b.r1, 1e-15, "liu(module) scales every length");

    // Invalid sets are rejected, naming the parameter.
    rejects(() -> liuSet(0.5, p -> p[6] = 0.9), "parameter ha");     // ha + e1 > r1
    rejects(() -> liuSet(0.5, p -> p[7] = 1.2), "parameter hf");     // e2 + hf > r2
    rejects(() -> liuSet(0.5, p -> p[0] = -1), "parameter r1");
    rejects(() -> liuSet(0.5, p -> p[3] = 0), "parameter r2");
    rejects(() -> liuSet(0.5, p -> p[3] = 1.6), "arcs overlap");    // r1 + r2 >= d
    rejects(() -> liuSet(0.5, p -> p[1] = 0.52), "c1");             // pointed tooth at A
    rejects(() -> liuSet(0.5, p -> p[4] = -0.1), "c2");             // D reaches mid-space
    rejects(() -> new GearSpec(0.4, 100, 8, liuSet(0.5)), "module");
    rejects(() -> new GearSpec(0.5, 100, 8, liuSet(0.5), true), "internal");
    rejects(() -> new GearSpec(0.5, 100, 8, liuSet(0.5), false, 0.1), "profile shift");
    rejects(() -> new GearSpec(0.5, 100, 8, liuSet(0.5), false, 0, 0.95 * 0.5).profile(), "cutterTipRadius");
    Sys.println('DoubleArcTests: $assertions assertions passed');
  }
}
