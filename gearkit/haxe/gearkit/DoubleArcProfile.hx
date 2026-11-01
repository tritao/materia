package gearkit;

/** One double-circular-arc tooth centred on +Y, right flank at positive X (external gear).
    The flat construction of `DoubleCircularArc` is wrapped onto the pitch circle: flat point (x, y) with y
    from the pitch line goes to radius `Rp + y`, polar angle `x / Rp`, so arc length on the pitch circle is
    exact and the angular pitch is 2 pi / Z. Wrapping is a diffeomorphism, so tangencies survive.
    Blends (assumed, the paper gives none): the tip blend is a circular arc of radius `tipFilletRadius`,
    internally tangent to the convex arc and the addendum line; the root blend is a circular arc of radius
    `cutterTipRadius` (the "root fillet radius" for this form), internally tangent to the concave arc and the
    dedendum line. It is not a generated trochoid; the rack that would cut it is recovered by `ConjugateRack`. */
class DoubleArcProfile implements ToothProfile {
  final spec:GearSpec;
  final f:DoubleCircularArc;
  final rp:Float;
  // Root blend: centre, start angle (join on arc 2), sweep to the dedendum line; join and floor points.
  final rcx:Float;
  final rcy:Float;
  final rho:Float;
  final rStart:Float;
  final rSweep:Float;
  final rootJoinY:Float;
  // Tip blend.
  final tcx:Float;
  final tcy:Float;
  final tau:Float;
  final tStart:Float;
  final tSweep:Float;
  final tipJoinY:Float;
  final tipXJoin:Float;

  public function new(spec:GearSpec, form:DoubleCircularArc) {
    this.spec = spec;
    f = form;
    rp = spec.pitchRadius();
    form.validate(spec);
    // Root blend, circle tangent to the dedendum line (centre y = -hf + rho) and inside circle 2.
    rho = spec.cutterTipRadius;
    rcy = -f.hf + rho;
    var q = (f.r2 - rho) * (f.r2 - rho) - (rcy - f.o2y) * (rcy - f.o2y);
    if (q < 0) throw 'double-circular-arc cutterTipRadius ($rho): the root fillet does not fit inside the concave arc (r2 = ${f.r2}, e2 = ${f.e2}, hf = ${f.hf})';
    rcx = f.o2x - Math.sqrt(q);
    var ux = (rcx - f.o2x) / (f.r2 - rho), uy = (rcy - f.o2y) / (f.r2 - rho);
    var jx = f.o2x + f.r2 * ux, jy = f.o2y + f.r2 * uy;
    // Join must lie between D and C on arc 2.
    var theta = Math.atan2(f.o2y - jy, f.o2x - jx);
    if (theta < f.delta - 1e-12) throw 'double-circular-arc cutterTipRadius ($rho): the root fillet is tangent to the concave arc above C; use a smaller radius';
    if (rcx >= f.referenceModule * Math.PI / 2) throw 'double-circular-arc cutterTipRadius ($rho): the root fillet reaches the middle of the tooth space; use a smaller radius';
    rootJoinY = jy;
    rStart = Math.atan2(uy, ux);
    var sw = -Math.PI / 2 - rStart;
    while (sw < 0) sw += 2 * Math.PI;
    rSweep = sw;
    // Tip blend, circle tangent to the addendum line (centre y = ha - tau) and inside circle 1.
    tau = spec.tipFilletRadius;
    if (tau > 0) {
      tcy = f.ha - tau;
      var p = (f.r1 - tau) * (f.r1 - tau) - (tcy - f.o1y) * (tcy - f.o1y);
      if (p < 0) throw 'double-circular-arc tipFilletRadius ($tau): the tip fillet does not fit inside the convex arc (r1 = ${f.r1}, e1 = ${f.e1}, ha = ${f.ha})';
      tcx = f.o1x + Math.sqrt(p);
      var vx = (tcx - f.o1x) / (f.r1 - tau), vy = (tcy - f.o1y) / (f.r1 - tau);
      tipJoinY = f.o1y + f.r1 * vy;
      tipXJoin = f.o1x + f.r1 * vx;
      tStart = Math.atan2(vy, vx);
      tSweep = Math.PI / 2 - tStart;
      if (tStart < f.delta - 1e-12) throw 'double-circular-arc tipFilletRadius ($tau): the tip fillet is tangent to the convex arc below B; use a smaller radius';
      if (tcx <= 0) throw 'double-circular-arc tipFilletRadius ($tau): tip rounding leaves a pointed or overlapping tooth; use a smaller radius';
    } else {
      tcx = f.ax;
      tcy = f.ha;
      tStart = 0;
      tSweep = 0;
      tipJoinY = f.ha;
      tipXJoin = f.ax;
    }
    evaluationRange();
  }

  function wrap(x:Float, y:Float):Point2 {
    var r = rp + y, a = x / rp;
    return new Point2(r * Math.sin(a), r * Math.cos(a));
  }

  /** Derivative of the wrapped point for flat derivative (dx, dy) at flat point (x, y). */
  function wrapD(x:Float, y:Float, dx:Float, dy:Float):Point2 {
    var r = rp + y, a = x / rp;
    var k = r / rp * dx;
    return new Point2(k * Math.cos(a) + dy * Math.sin(a), -k * Math.sin(a) + dy * Math.cos(a));
  }

  public function gearSpec():GearSpec return spec;
  public function rootJoinRadius():Float return rp + rootJoinY;
  public function tipJoinRadius():Float return rp + tipJoinY;

  function yAtRadius(radius:Float):Float {
    var lo = rootJoinRadius(), hi = tipJoinRadius();
    if (!Math.isFinite(radius) || radius < lo - 1e-12 || radius > hi + 1e-12) throw "radius is outside the active double-arc flank";
    return Math.min(tipJoinY, Math.max(rootJoinY, radius - rp));
  }

  public function flankAtRadius(radius:Float):Point2 {
    var y = yAtRadius(radius);
    return wrap(f.xAt(y), y);
  }

  public function flank(t:Float):Point2 {
    unit(t);
    return flankAtRadius(rootJoinRadius() + t * (tipJoinRadius() - rootJoinRadius()));
  }

  /** Derivative w.r.t. radius, towards increasing radius. */
  public function flankDerivative(t:Float):Point2 {
    unit(t);
    var y = rootJoinY + t * (tipJoinY - rootJoinY);
    return wrapD(f.xAt(y), y, f.dxdyAt(y), 1);
  }

  /** t=0: join on the concave arc; t=1: dedendum circle, both tangent. */
  public function rootFillet(t:Float):Point2 {
    unit(t);
    var a = rStart + t * rSweep;
    return wrap(rcx + rho * Math.cos(a), rcy + rho * Math.sin(a));
  }

  public function rootDerivative(t:Float):Point2 {
    unit(t);
    var a = rStart + t * rSweep;
    return wrapD(rcx + rho * Math.cos(a), rcy + rho * Math.sin(a), -rho * rSweep * Math.sin(a), rho * rSweep * Math.cos(a));
  }

  /** t=0: join on the convex arc; t=1: addendum circle, both tangent. */
  public function tipFillet(t:Float):Point2 {
    unit(t);
    if (tau <= 0) return wrap(f.ax, f.ha);
    var a = tStart + t * tSweep;
    return wrap(tcx + tau * Math.cos(a), tcy + tau * Math.sin(a));
  }

  public function tipDerivative(t:Float):Point2 {
    unit(t);
    if (tau <= 0) return new Point2(0, 0);
    var a = tStart + t * tSweep;
    return wrapD(tcx + tau * Math.cos(a), tcy + tau * Math.sin(a), -tau * tSweep * Math.sin(a), tau * tSweep * Math.cos(a));
  }

  public function evaluationRange():GearEvaluationRange {
    var lo = rootJoinRadius(), hi = tipJoinRadius();
    var result = spec.evaluation;
    if (result == null) return new GearEvaluationRange(lo, hi, 0, spec.faceWidth);
    if (result.radiusMin < lo || result.radiusMax > hi || result.axialMax
      > spec.faceWidth) throw "evaluation window must lie on the unrounded flank and within the face";
    return result;
  }

  public function outline(samples:Int):Array<Point2> {
    if (samples < 4) throw "profile needs at least four samples per segment";
    var side:Array<Point2> = [];
    for (i in 0...samples + 1) side.push(rootFillet(1 - i / samples));
    for (i in 1...samples + 1) side.push(flank(i / samples));
    for (i in 1...samples + 1) side.push(tipFillet(i / samples));
    return ToothOutline.close(spec, side, samples);
  }

  public function halfAngleAt(radius:Float):Float {
    var y = yAtRadius(radius);
    return f.xAt(y) / rp;
  }

  /** Length along the flank (flat frame, mm) from the root join. */
  public function profileCoordinate(radius:Float):Float {
    var y = yAtRadius(radius);
    return f.lengthAt(y) - f.lengthAt(rootJoinY);
  }

  public function radiusAtCoordinate(u:Float):Float return rp + f.heightAtLength(u + f.lengthAt(rootJoinY));

  /** Radius times the cosine between the flank normal and the tangential direction. */
  public function normalLever(radius:Float):Float {
    var y = yAtRadius(radius), x = f.xAt(y), a = x / rp, r = rp + y;
    var d = wrapD(x, y, f.dxdyAt(y), 1), len = d.radius();
    // unit normal (d.y, -d.x)/len dotted with the tangential direction (cos a, -sin a)
    return r * Math.abs((d.y * Math.cos(a) + d.x * Math.sin(a)) / len);
  }

  static function unit(t:Float):Void {
    if (!Math.isFinite(t) || t < 0 || t > 1) throw "curve parameter must be in [0, 1]";
  }
}
