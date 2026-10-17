package gearkit;

/** Numerically generated conjugate rack in its unshifted normal plane.
    X is the right gear-flank coordinate; Y is height above the gear pitch datum.
    The cutter tooth is centred at X=pitch/2, so its half-width is pitch/2-X.
    Samples retain the recovered rounded cutter tip as well as the active flank. */
class ConjugateRack {
  public final module:Float;
  public final pressureAngle:Float;
  /** A folded conjugate root cannot be used as a single-valued hob cutter region. */
  public final hasRootFold:Bool;
  final flankPoints:Array<Point2>;
  final cornerPoints:Array<Point2>;
  final rootPoints:Array<Point2>;

  function new(module:Float, pressureAngle:Float, flank:Array<Point2>, corner:Array<Point2>, root:Array<Point2>) {
    this.module = module;
    this.pressureAngle = pressureAngle;
    flankPoints = flank;
    cornerPoints = corner;
    rootPoints = root;
    var folded = false;
    for (i in 1...root.length) if (root[i].y < root[i - 1].y - 1e-12 * module) folded = true;
    hasRootFold = folded;
  }

  public static function of(profile:ToothProfile, samples:Int = 128):ConjugateRack {
    if (samples < 8) throw "rack envelope needs at least eight samples";
    var flank:Array<Point2> = [], corner:Array<Point2> = [], root:Array<Point2> = [];
    for (i in 0...samples + 1) {
      var t = i / samples;
      flank.push(contact(profile.spec, profile.flank(t), profile.flankDerivative(t)));
      corner.push(contact(profile.spec, profile.rootFillet(t), profile.rootDerivative(t)));
      if (profile.spec.tipFilletRadius > 0) {
        root.push(contact(profile.spec, profile.tipFillet(t), profile.tipDerivative(t)));
      } else {
        // A sharp tip has a normal cone: its stationary point sweeps the cutter-root blend.
        var p = profile.flank(1), d = profile.flankDerivative(1);
        var end = new Point2(-profile.spec.sign() * p.y, profile.spec.sign() * p.x);
        var sweep = Math.atan2(d.x * end.y - d.y * end.x, d.x * end.x + d.y * end.y);
        root.push(contact(profile.spec, p, d.rotated(t * sweep)));
      }
    }
    flank.sort((a, b) -> a.y < b.y ? -1 : a.y > b.y ? 1 : 0);
    corner.sort((a, b) -> a.y < b.y ? -1 : a.y > b.y ? 1 : 0);
    return new ConjugateRack(profile.spec.module, profile.spec.pressureAngle, flank, corner, root);
  }

  /** Solve the envelope equation n·v=0 by bisection; no involute/rack formula shortcut.
      The two normal-contact branches are distinguished by the gear's signed radius. */
  static function contact(spec:GearSpec, p:Point2, derivative:Point2):Point2 {
    var d = derivative;
    // At circle tangencies p·d is zero; use angular orientation to select the normal
    // consistently instead of flipping on the sign of floating-point radial roundoff.
    if (spec.sign() * (p.x * d.y - p.y * d.x) < 0) d = d.scaled(-1);
    var length = d.radius(), target = (p.x * d.x + p.y * d.y) / spec.pitchRadius();
    if (Math.abs(target) < 1e-14 * length) target = 0;
    if (length <= 0 || target > length + 1e-12
      || !Math.isFinite(target)) throw "profile normal cannot contact the rack pitch line";
    var delta = Math.atan2(d.y, d.x);
    var low = spec.internal ? delta - Math.PI / 2 : delta - Math.PI;
    var high = spec.internal ? delta : delta - Math.PI / 2;
    for (_ in 0...64) {
      var phi = (low + high) / 2;
      var error = -Math.sin(phi) * d.x + Math.cos(phi) * d.y - target;
      if ((error > 0) == spec.internal) low = phi;
      else high = phi;
    }
    var phi = (low + high) / 2;
    var q = spec.sign() * spec.pitchRadius();
    var rack = p.scaled(spec.sign()).rotated(-phi);
    return new Point2(spec.sign() * (rack.x - q * phi), rack.y - q - spec.sign() * spec.profileShift * spec.module);
  }

  /** Copies: callers cannot alter the recovered envelope. */
  public function flankSamples():Array<Point2> return flankPoints.copy();
  public function cornerSamples():Array<Point2> return cornerPoints.copy();
  public function rootSamples():Array<Point2> return rootPoints.copy();

  /** Piecewise-linear flank value inside its measured range; extrapolation is rejected. */
  public function flankAt(height:Float):Point2 {
    var segment = segmentAt(height);
    var a = flankPoints[segment], b = flankPoints[segment + 1];
    return new Point2(a.x +(b.x - a.x) * (height - a.y) / (b.y - a.y), height);
  }

  /** Re-envelope the recovered rack at a normal-plane height onto a new gear.
      This uses the sample slope, not the target gear's involute formula. */
  public function envelopeAt(spec:GearSpec, height:Float):Point2 {
    if (Math.abs(spec.module - module) > 1e-12 || Math.abs(spec.pressureAngle - pressureAngle)
      > 1e-12) throw "rack and target gear must have the same module and pressure angle";
    var segment = segmentAt(height), a = flankPoints[segment], b = flankPoints[segment + 1];
    var slope = (b.x - a.x) / (b.y - a.y);
    if (Math.abs(slope) < 1e-14) throw "rack flank has no finite generating contact";
    var u = spec.sign() * flankAt(height).x;
    var v = height + spec.sign() * spec.profileShift * module;
    var q = spec.sign() * spec.pitchRadius(), x = -spec.sign() * v / slope;
    return new Point2(x, q + v).rotated((x - u) / q).scaled(spec.sign());
  }

  function segmentAt(height:Float):Int {
    if (!Math.isFinite(height) || height < flankPoints[0].y
      || height > flankPoints[flankPoints.length - 1].y) throw "rack height is outside the recovered flank";
    var lo = 0, hi = flankPoints.length - 1;
    while (hi - lo > 1) {
      var mid = (lo + hi) >> 1;
      if (flankPoints[mid].y <= height) lo = mid;
      else hi = mid;
    }
    return lo;
  }

  /** Closed cutter-tooth region in the normal rake plane. Includes the recovered
      cutter-root blend conjugate to the target gear's tip rounding. */
  public function cutterRegion(rootHeight:Float):Array<Point2> {
    if (hasRootFold) throw "conjugate cutter root folds; use a different tip rounding or cutter model";
    if (!Math.isFinite(rootHeight)) throw "cutter root height must be finite";
    var upper = rootPoints.length == 0 ? flankPoints[flankPoints.length - 1] : rootPoints[rootPoints.length - 1];
    var height = Math.abs(rootHeight - upper.y) < 1e-10 * module ? upper.y : rootHeight;
    var pitchHalf = Math.PI * module / 2;
    var top:Point2;
    var blend:Array<Point2> = [];
    if (height > flankPoints[flankPoints.length - 1].y) {
      if (height > upper.y) throw "cutter root height is outside the recovered rack";
      var i = 0;
      while (i + 1 < rootPoints.length && rootPoints[i + 1].y < height) i++;
      var a = rootPoints[i], b = rootPoints[i + 1];
      top = new Point2(a.x +(b.x - a.x) * (height - a.y) / (b.y - a.y), height);
      var j = rootPoints.length - 1;
      while (j >= 0) {
        if (rootPoints[j].y < height) blend.push(rootPoints[j]);
        j--;
      }
    } else top = flankAt(height);
    var side:Array<Point2> = [top];
    for (p in blend) side.push(p);
    var k = flankPoints.length - 1;
    while (k >= 0) {
      if (flankPoints[k].y < rootHeight) side.push(flankPoints[k]);
      k--;
    }
    k = cornerPoints.length - 1;
    while (k >= 0) side.push(cornerPoints[k--]);
    var result:Array<Point2> = [];
    for (p in side) {
      var width = pitchHalf - p.x;
      if (width <= 0) throw "recovered rack leaves no cutter tooth width";
      result.push(new Point2(-width, p.y));
    }
    k = side.length - 1;
    while (k >= 0) {
      var p = side[k--];
      result.push(new Point2(pitchHalf - p.x, p.y));
    }
    result.push(result[0]);
    return result;
  }
}
