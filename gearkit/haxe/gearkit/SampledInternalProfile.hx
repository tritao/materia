package gearkit;

/** One tooth of an internal gear whose flank is a sampled space (`SampledInternalForm`), centred on +Y, right flank at
    positive X, as a `ToothProfile`. The tooth half-angle at radius r is `pi/teeth - space half-angle`.
    - Flank: radii `tipRadius .. space.maxRadius()`; `flank(0)` is the root join (largest radius), `flank(1)` the tip join.
    - Tip: the tip circle is at the flank's inner end, so the tip blend has zero length (a sharp corner, the land is the
      tip arc). The tooth is not rounded: a real circular spline would be.
    - Root: from the largest sampled radius to `rootRadius` the half-angle is held constant (a radial wall), then the
      root circle. **Assumed**, not generated: the generated root trochoid is a later step.
    `deviationBound()` is the sampling bound of the swept space (a length, mm) and applies to the flank only; the radial
    wall beyond `sampledReach()` carries no bound. */
class SampledInternalProfile implements ToothProfile {
  static final TABLE = 400;
  final spec:GearSpec;
  final form:SampledInternalForm;
  final space:SweptSpace;
  final half:Float;
  final tipR:Float;
  final reachR:Float;
  final rootR:Float;
  final arc:Array<Float> = [];

  public function new(spec:GearSpec, form:SampledInternalForm) {
    this.spec = spec;
    this.form = form;
    space = form.space;
    half = Math.PI / space.teeth;
    tipR = form.tipRadius;
    reachR = space.maxRadius();
    rootR = form.rootRadius;
    if (half - space.halfAngleAt(tipR) <= 0) throw "circular-spline tooth has no thickness at its tip";
    if (half - space.halfAngleAt(reachR) <= 0) throw "circular-spline tooth has no thickness at the sampled reach";
    arc.push(0.0);
    var prev = flankAtRadius(tipR);
    for (k in 1...TABLE + 1) {
      var q = flankAtRadius(tipR + (reachR - tipR) * k / TABLE);
      arc.push(arc[k - 1] + q.distance(prev));
      prev = q;
    }
  }

  public function deviationBound():Float return space.deviationBound;
  /** Largest radius the sampled flank reaches; beyond it the tooth wall is assumed radial. */
  public function sampledReach():Float return reachR;

  function toothHalf(r:Float):Float return half - space.halfAngleAt(Math.min(reachR, r));

  public function gearSpec():GearSpec return spec;
  public function rootJoinRadius():Float return reachR;
  public function tipJoinRadius():Float return tipR;

  public function flankAtRadius(radius:Float):Point2 {
    if (!(radius >= tipR - 1e-12 && radius <= reachR + 1e-12)) throw "radius is outside the sampled flank";
    var r = Math.max(tipR, Math.min(reachR, radius)), h = toothHalf(r);
    return new Point2(r * Math.sin(h), r * Math.cos(h));
  }

  public function flank(t:Float):Point2 {
    ToothOutline.checkParameter(t);
    return flankAtRadius(reachR + t * (tipR - reachR));
  }

  public function flankDerivative(t:Float):Point2 {
    var p = flank(t), r = p.radius(), d = 1e-5 * (reachR - tipR);
    var r0 = Math.max(tipR, r - d), r1 = Math.min(reachR, r + d);
    var a = flankAtRadius(r0), b = flankAtRadius(r1);
    return new Point2((b.x - a.x) / (r1 - r0), (b.y - a.y) / (r1 - r0));
  }

  public function rootFillet(t:Float):Point2 {
    ToothOutline.checkParameter(t);
    var r = reachR + t * (rootR - reachR), h = toothHalf(reachR);
    return new Point2(r * Math.sin(h), r * Math.cos(h));
  }

  public function rootDerivative(t:Float):Point2 {
    ToothOutline.checkParameter(t);
    var h = toothHalf(reachR);
    return new Point2(Math.sin(h), Math.cos(h)).scaled(rootR - reachR);
  }

  public function tipFillet(t:Float):Point2 {
    ToothOutline.checkParameter(t);
    return flankAtRadius(tipR);
  }

  public function tipDerivative(t:Float):Point2 {
    ToothOutline.checkParameter(t);
    return new Point2(0, 0);
  }

  public function regionAt(radius:Float):ToothRegion return ToothOutline.regionAt(this, radius);
  public function evaluationRange():GearEvaluationRange return ToothOutline.evaluationRange(spec, tipR, reachR);
  public function outline(samples:Int):Array<Point2> return ToothOutline.of(this, samples);
  public function halfAngleAt(radius:Float):Float return toothHalf(radius);

  /** Arc length along the flank from the tip join, increasing with radius. */
  public function profileCoordinate(radius:Float):Float {
    if (!(radius >= tipR - 1e-12 && radius <= reachR + 1e-12)) throw "radius is outside the sampled flank";
    var x = (Math.max(tipR, Math.min(reachR, radius)) - tipR) / (reachR - tipR) * TABLE;
    var k = Std.int(Math.min(TABLE - 1, Math.floor(x)));
    return arc[k] + (x - k) * (arc[k + 1] - arc[k]);
  }

  public function radiusAtCoordinate(coordinate:Float):Float {
    if (!(coordinate >= -1e-12 && coordinate <= arc[TABLE] + 1e-12)) throw "coordinate is outside the sampled flank";
    var lo = 0, hi = TABLE;
    while (hi - lo > 1) {
      var mid = (lo + hi) >> 1;
      if (arc[mid] <= coordinate) lo = mid;
      else hi = mid;
    }
    var f = (coordinate - arc[lo]) / (arc[lo + 1] - arc[lo]);
    return tipR + (reachR - tipR) * (lo + f) / TABLE;
  }

  /** Distance from the centre to the flank normal at `radius`. */
  public function normalLever(radius:Float):Float {
    var r = Math.max(tipR, Math.min(reachR, radius)), p = flankAtRadius(r), t = flankDerivative(1 - (r - tipR) / (reachR - tipR));
    return Math.abs(p.x * t.x + p.y * t.y) / Math.sqrt(t.x * t.x + t.y * t.y);
  }
}
