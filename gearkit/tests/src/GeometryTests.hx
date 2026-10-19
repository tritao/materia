import gearkit.ConjugateRack;
import gearkit.GearEvaluationRange;
import gearkit.GearSpec;
import gearkit.HobDesign;
import gearkit.Point2;
import gearkit.ToothProfile;

class GeometryTests {
  static var assertions = 0;
  static var rackError = 0.0;
  static var envelopeError = 0.0;

  static function check(ok:Bool, message:String):Void {
    assertions++;
    if (!ok) throw message;
  }
  static function near(a:Float, b:Float, tolerance:Float, message:String):Void check(
    Math.isFinite(a) && Math.abs(a - b) <= tolerance,
    '$message: $a versus $b (tol $tolerance)'
  );
  static function rejects(f:Void -> Void, fragment:String):Void {
    var error = "";
    try f() catch (e:Dynamic) error = Std.string(e);
    check(error.indexOf(fragment) >= 0, 'expected "$fragment", got "$error"');
  }
  static function parallel(a:Point2, b:Point2, message:String):Void {
    var cross = (a.x * b.y - a.y * b.x) / (a.radius() * b.radius());
    near(cross, 0, 1e-7, message);
  }
  static function tangentCircle(p:Point2):Point2 return new Point2(p.y, -p.x);

  static function profileChecks(spec:GearSpec):Void {
    var profile = new ToothProfile(spec), rack = ConjugateRack.of(profile);
    var root = profile.rootFillet(1), rootJoin = profile.rootFillet(0);
    near(root.radius(), spec.rootRadius(), 1e-11, "root circle");
    near(rootJoin.distance(profile.flank(0)), 0, 1e-11, "root/involute continuity");
    parallel(profile.rootDerivative(0), profile.flankDerivative(0), "root/involute tangency");
    parallel(profile.rootDerivative(1), tangentCircle(root), "root circle tangency");
    var tip = profile.tipFillet(1), tipJoin = profile.tipFillet(0), h = 1e-6;
    near(tip.radius(), spec.tipRadius(), 1e-11, "tip circle");
    near(tipJoin.distance(profile.flank(1)), 0, 1e-11, "tip/involute continuity");

    parallel(profile.tipDerivative(0), profile.flankDerivative(1), "tip/involute tangency");
    parallel(profile.tipDerivative(1), tangentCircle(tip), "tip circle tangency");
    for (i in 1...100) {
      var t = i / 100, p = profile.rootFillet(t), d = profile.rootDerivative(t);
      var before = profile.rootFillet(t - h), after = profile.rootFillet(t + h);
      near(d.x, (after.x - before.x) / (2 * h), 2e-8, "root derivative X");
      near(d.y, (after.y - before.y) / (2 * h), 2e-8, "root derivative Y");
      check(
        spec.sign() * (p.radius() - root.radius()) > 0,
        'fillet leaves root: internal=${spec.internal}, shift=${spec.profileShift}, module=${spec.module}, t=$t, r=${p.radius()}, root=${root.radius()}'
      );
    }
    for (p in rack.flankSamples()) {
      var expected = Math.PI * spec.module / 4 - p.y * Math.tan(spec.pressureAngle);
      rackError = Math.max(rackError, Math.abs(p.x - expected));
      near(p.x, expected, 1e-9, "numerical envelope is a straight basic rack");
    }
    // Recover the cutter-tip circle independently of the original profile's corner centre.
    var rho = spec.cutterTipRadius, alpha = spec.pressureAngle;
    var cx = Math.PI
      * spec.module / 4 + 1.25 * spec.module * Math.tan(alpha) + rho * (1 / Math.cos(alpha) - Math.tan(alpha));
    var cy = -1.25 * spec.module + rho;
    for (p in rack.cornerSamples()) near(
      new Point2(p.x - cx, p.y - cy).radius(),
      rho,
      1e-9,
      "recovered rounded cutter tip"
    );
    var points = rack.flankSamples(), ymin = points[0].y, ymax = points[points.length - 1].y;
    // Heights between samples test interpolation and the independently derived closed-form oracle.
    for (i in 0...101) {
      var y = ymin +(ymax - ymin) * (i + 0.37) / 101;
      var p = rack.envelopeAt(spec, y), r = p.radius();
      var alphaR = Math.acos(spec.baseRadius() / r);
      var expectedAngle = spec.pitchToothThickness()
        / spec.pitchDiameter() + spec.sign() * (Math.tan(alpha) - alpha - Math.tan(alphaR) + alphaR);
      var oracle = new Point2(r * Math.sin(expectedAngle), r * Math.cos(expectedAngle));
      envelopeError = Math.max(envelopeError, p.distance(oracle));
      near(p.distance(oracle), 0, 1e-6, "rack re-envelope against independent involute");
    }
    var rootSamples = rack.rootSamples();
    near(
      rootSamples[rootSamples.length - 1].y,
      spec.module,
      1e-10,
      'recovered rack root height: internal=${spec.internal}, shift=${spec.profileShift}, m=${spec.module}'
    );
    for (p in rootSamples) check(p.x > 0, "rack root blend leaves a gear tip land");
    var outline = profile.outline();
    check(outline.length > 100, "complete tooth outline includes lands and fillets");
    near(outline[0].distance(outline[outline.length - 1]), 0, 0, "closed outline");
    for (p in outline) {
      check(Math.isFinite(p.x) && Math.isFinite(p.y), "finite outline point");
      check(Math.abs(Math.atan2(p.x, p.y)) <= Math.PI / spec.teeth + 1e-12, "outline stays within one pitch");
    }
    var range = spec.evaluationRange();
    near(range.radiusMin, Math.min(profile.rootJoinRadius, profile.tipJoinRadius), 1e-12, "default radial range");
    near(range.axialMax, spec.faceWidth, 0, "default axial range");
    rejects(() -> {profile.flankAtRadius(spec.baseRadius() * 0.99);
    }, "outside");
    rejects(() -> {profile.rootFillet(Math.NaN);
    }, "parameter");
    rejects(() -> {rack.flankAt(ymax + 1);
    }, "outside");
  }

  static function hobChecks():Void {
    var alpha = 20 * Math.PI / 180;
    var profile = new ToothProfile(new GearSpec(0.3, 200, 8, alpha));
    var rack = ConjugateRack.of(profile);
    for (starts in [1, 2, 3]) for (rake in [0.0, 0.12, -0.12]) {
      var hob = new HobDesign(rack, starts, 32, 12, rake);
      var dp = 32 - 2 * 1.25 * 0.3;
      near(hob.pitchDiameter, dp, 1e-10, "hob pitch diameter from cutter addendum");
      near(Math.sin(hob.leadAngle), 0.3 * starts / dp, 1e-13, "normal module lead angle");
      near(hob.axialPitch, Math.PI * 0.3 / Math.cos(hob.leadAngle), 1e-13, "axial pitch");
      near(Math.tan(hob.leadAngle), hob.lead / (Math.PI * dp), 1e-13, "lead helix cross-check");
      for (gash in 0...hob.gashes) for (thread in 0...starts) {
        var region = hob.rakeFaceRegion(gash, thread), nextTurn = hob.rakeFaceRegion(gash, thread, 1);
        var phase = 2 * Math.PI * gash / hob.gashes, cs = Math.cos(phase), sn = Math.sin(phase);
        var maxRadius = 0.0;
        for (i in 0...region.length) {
          var p = region[i], x = cs * p.x + sn * p.y, y = -sn * p.x + cs * p.y;
          var radius = Math.sqrt(p.x * p.x + p.y * p.y);
          maxRadius = Math.max(maxRadius, radius);
          near(Math.sin(rake) * x + Math.cos(rake) * y, 16 * Math.sin(rake), 1e-11, "edge lies in rake plane");
          check(radius <= 16 + 1e-11, "hob edge stays inside OD");
          near(nextTurn[i].z - p.z, hob.lead, 1e-12, "thread turn spacing");
        }
        near(maxRadius, 16, 1e-11, "cutting edge reaches outside circle");
        var behind=0.1,back=hob.relievedFaceRegion(gash,thread,0,behind,0.05);
        for(i in 0...region.length){
          var before=region[i],after=back[i],radius=Math.sqrt(after.x*after.x+after.y*after.y);
          var rotated=phase-behind,x=Math.cos(rotated)*after.x+Math.sin(rotated)*after.y,y=-Math.sin(rotated)*after.x+Math.cos(rotated)*after.y;
          near(Math.sin(rake)*x+Math.cos(rake)*y,16*Math.sin(rake),1e-11,"relieved section stays planar at rake offset");
          check(radius<=Math.sqrt(before.x*before.x+before.y*before.y)+1e-12&&radius>=dp/2-hob.rootHeight-1e-12,"relief stays behind edge and above root");
          var a0=Math.atan2(-sn*before.x+cs*before.y,cs*before.x+sn*before.y);
          var a1=Math.atan2(-sn*after.x+cs*after.y,cs*after.x+sn*after.y);
          near(after.z-before.z,hob.lead*(a1-a0)/(2*Math.PI),1e-12,"backing follows the generating helix");
        }
        near(region[0].x, region[region.length - 1].x, 0, "closed rake region X");
        near(region[0].z, region[region.length - 1].z, 0, "closed rake region Z");
      }
      var first = hob.rakeFaceRegion(0), second = hob.rakeFaceRegion(1);
      near(second[0].z - first[0].z, hob.lead / hob.gashes, 1e-12, "gash helical phasing");
      if (starts > 1) {
        var thread1 = hob.rakeFaceRegion(0, 1);
        near(thread1[0].z - first[0].z, hob.axialPitch, 1e-12, "start spacing");
      }
    }
    rejects(() -> {new HobDesign(rack, 0, 32, 12);
    }, "positive starts");
    rejects(() -> {new HobDesign(rack, 1, 0.8, 12);
    }, "pitch diameter");
    rejects(() -> {new HobDesign(rack, 1, 32, 12, Math.NaN);
    }, "rake");
    rejects(() -> {new HobDesign(rack, 1, 32, 12, 0, 10);
    }, "outside");
    rejects(() -> {new HobDesign(rack, 1, 32, 12).rakeFaceRegion(12);
    }, "index");
  }

  public static function run():Void {
    var alpha = 20 * Math.PI / 180;
    for (internal in [false, true]) for (shift in [-0.2, 0.0, 0.3]) {
      profileChecks(new GearSpec(0.3, internal ? 240 : 200, 8, alpha, internal, shift));
      profileChecks(new GearSpec(1, internal ? 240 : 40, 10, alpha, internal, shift));
    }
    profileChecks(new GearSpec(0.5, 100, 15, alpha));
    profileChecks(new GearSpec(0.3, 200, 8, 25 * Math.PI / 180, false, 0, 0.075));
    profileChecks(new GearSpec(0.3, 202, 8, alpha, true));
    rejects(() -> {new ToothProfile(new GearSpec(1, 40, 10, alpha, true));
    }, "internal rack-limit root");
    // The same recovered rack must generate a different tooth count and profile shift.
    var source = new GearSpec(0.3, 200, 8, alpha);
    var common = ConjugateRack.of(new ToothProfile(source));
    for (internal in [false, true]) {
      var target = new GearSpec(0.3, 240, 8, alpha, internal, 0.1);
      var tooth = new ToothProfile(target), points = common.flankSamples();
      for (i in 1...50) {
        var y = points[0].y +(points[points.length - 1].y - points[0].y) * i / 50;
        var p = common.envelopeAt(target, y);
        near(p.distance(tooth.flankAtRadius(p.radius())), 0, 1e-6, "common rack generates another gear");
      }
    }
    var sharp = new ToothProfile(new GearSpec(0.3, 200, 8, alpha, false, 0, null, 0));
    var sharpRack = ConjugateRack.of(sharp);
    var sharpRoot = sharpRack.rootSamples();
    near(sharpRoot[sharpRoot.length - 1].y, 0.3, 1e-10, "sharp tip normal cone gives full rack height");
    near(new HobDesign(sharpRack, 1, 32, 12).rootHeight, 0.3, 1e-10, "sharp tip hob root height");
    var folded = ConjugateRack.of(new ToothProfile(new GearSpec(0.3, 202, 8, alpha, true)));
    check(folded.hasRootFold, "internal tip rounding exposes a folded conjugate cutter root");
    rejects(() -> {new HobDesign(folded, 1, 32, 12);
    }, "cutter root folds");
    hobChecks();
    rejects(() -> {new ToothProfile(new GearSpec(1, 8, 8, alpha));
    }, "undercut");
    rejects(() -> {new GearSpec(1, 40, 10, alpha, false, Math.NaN);
    }, "shift");
    rejects(() -> {new GearSpec(1, 40, 10, alpha, false, 0, -1);
    }, "cutter tip");
    rejects(() -> {new GearSpec(1, 40, 10, alpha, false, 0, null, -1);
    }, "tip fillet");
    var range = new GearEvaluationRange(29.9, 30.1, 0.4, 7.6);
    var spec = new GearSpec(0.3, 200, 8, alpha, false, 0, null, null, range);
    near(spec.evaluationRange().axialMin, 0.4, 0, "explicit evaluation range");
    rejects(() -> {new GearSpec(0.3,
      200, 8, alpha, false, 0, null, null, new GearEvaluationRange(29.9, 30.1, 0, 9)).evaluationRange();
    }, "evaluation window");
    Sys
      .println('GeometryTests: $assertions assertions passed; rack error $rackError mm; re-envelope error $envelopeError mm');
  }
}
