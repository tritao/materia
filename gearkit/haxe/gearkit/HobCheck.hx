package gearkit;

/** Whether a flexspline tooth can be cut by a hob (TOOTH_DESIGN_PLAN.md TD-D5): the tooth builds
    without undercut, a generating rack is recovered whose root does not fold, and a hob can be
    designed from it. Returns null when hobbable, else the reason (the first check to fail). */
class HobCheck {
  public static function problem(spec:GearSpec, hobOutsideDiameter:Float = 32, gashes:Int = 12):Null<String> {
    var rack:ConjugateRack;
    try {
      rack = ConjugateRack.of(spec.profile());
    } catch (e:Dynamic) {
      return Std.string(e);
    }
    if (rack.hasRootFold) return "conjugate rack root folds; the hob tooth cannot be single-valued";
    try {
      var hob = new HobDesign(rack, 1, hobOutsideDiameter, gashes);
      hob.rakeFaceRegion(0);
    } catch (e:Dynamic) {
      return Std.string(e);
    }
    return null;
  }

  /** Largest cutter tip radius (mm) for which the profile still has a cutter land, by bisection on whether the profile builds. */
  public static function maxCutterTipRadius(make:Float -> ToothProfile, module:Float):Float {
    var lo = 0.0, hi = 0.99 * module;
    for (_ in 0...40) {
      var mid = (lo + hi) / 2, ok = true;
      try make(mid) catch (e:Dynamic) ok = false;
      if (ok) lo = mid;
      else hi = mid;
    }
    return lo;
  }
}
