package gearkit;

/** One tooth centred on +Y, right flank at positive X. External/internal involutes
    and the signed rolling envelope of a rounded rack cutter at the root.
    Internal roots are a formal rack-limit construction, not a pinion-shaper model.
    Undercut, pointed and overlapping teeth are rejected rather than silently trimmed. */
class InvoluteProfile implements ToothProfile {
  final spec:GearSpec;
  final rootJoinR:Float;
  final tipJoinR:Float;
  final half:Float;
  final depth:Float;
  final cornerU:Float;
  final cornerV:Float;
  final tipCentre:Point2;
  final tipArcStart:Float;
  final tipArcSweep:Float;
  final relief:Null<TipRelief>;

  public function new(spec:GearSpec, ?relief:TipRelief) {
    this.spec = spec;
    this.relief = relief;
    var a = spec.form.pressureAngle(), rho = spec.cutterTipRadius;
    half = spec.pitchToothThickness() / 2;
    depth = spec.module * (Involute.DEDENDUM - spec.sign() * spec.profileShift);
    cornerV = -depth + rho;
    cornerU = half + depth * Math.tan(a) + rho * (1 / Math.cos(a) - Math.tan(a));
    if (cornerU >= Math.PI * spec.module / 2) throw "rack corner leaves no cutter tip land";
    var vJoin = cornerV - rho * Math.sin(a);
    if (!spec.internal && vJoin <= -spec.pitchRadius() * Math.pow(Math.sin(a),
      2)) throw "rack-generated tooth is undercut; increase tooth count or profile shift";
    if (spec.internal && rho * spec.pitchRadius() <= cornerV * vJoin / Math.pow(Math.sin(a),
      3)) throw "internal rack-limit root is undercut; use more teeth, less shift or a pinion-shaped cutter";
    rootJoinR = rootFillet(0).radius();
    var rb = Involute.baseRadius(spec), rt = spec.tipRadius(), radius = spec.tipFilletRadius;
    var rc = rt - spec.sign() * radius;
    if (rc <= rb) throw "tip fillet centre lies below the base circle";
    var p:Point2, nx:Float, ny:Float;
    if (relief == null) {
      var roll = spec.sign() * radius + Math.sqrt(rc * rc - rb * rb);
      if (roll <= 0) throw "tip fillet cannot meet this involute";
      tipJoinR = Math.sqrt(rb * rb + roll * roll);
      if (spec.sign() * (tipJoinR - rootJoinR) <= 0) throw "root and tip fillets overlap";
      p = involuteAtRadius(tipJoinR);
      var theta = Math.atan2(p.x, p.y), alpha = Math.acos(rb / tipJoinR);
      nx = spec.sign() * Math.sin(alpha) * Math.sin(theta) + Math.cos(alpha) * Math.cos(theta);
      ny = spec.sign() * Math.sin(alpha) * Math.cos(theta) - Math.cos(alpha) * Math.sin(theta);
    } else {
      // Relieved flank: the tip rounding is the circle of the tip radius tangent to the tip circle and to the relieved flank;
      // find the join radius nearest the tip where the offset-flank point lies on the circle of the fillet centres.
      var lo = Math.min(rootJoinR, rt), hi = Math.max(rootJoinR, rt), steps = 400, found = false, rJoin = 0.0;
      var prev = Math.NaN, prevR = 0.0;
      for (i in 0...steps + 1) {
        var r = rt + (rootJoinR - rt) * i / steps;
        if (r < lo || r > hi || r <= rb) continue;
        var n = filletNormal(r), q = involuteAtRadius(r);
        var g = Math.sqrt(Math.pow(q.x - radius * n.x, 2) + Math.pow(q.y - radius * n.y, 2)) - rc;
        if (!Math.isNaN(prev) && prev * g <= 0) {
          var a = prevR, b = r, ga = prev;
          for (_ in 0...80) {
            var mid = (a + b) / 2, nm = filletNormal(mid), qm = involuteAtRadius(mid);
            var gm = Math.sqrt(Math.pow(qm.x - radius * nm.x, 2) + Math.pow(qm.y - radius * nm.y, 2)) - rc;
            if (ga * gm <= 0) b = mid;
            else {
              a = mid;
              ga = gm;
            }
          }
          rJoin = (a + b) / 2;
          found = true;
          break;
        }
        prev = g;
        prevR = r;
      }
      if (!found) throw "tip fillet cannot meet the relieved flank";
      tipJoinR = rJoin;
      if (spec.sign() * (tipJoinR - rootJoinR) <= 0) throw "root and tip fillets overlap";
      p = involuteAtRadius(tipJoinR);
      var n = filletNormal(tipJoinR);
      nx = n.x;
      ny = n.y;
    }
    tipCentre = new Point2(p.x - radius * nx, p.y - radius * ny);
    tipArcStart = Math.atan2(ny, nx);
    var end = Math.atan2(spec.sign() * tipCentre.y, spec.sign() * tipCentre.x);
    tipArcSweep = end - tipArcStart;
    while (tipArcSweep > Math.PI) tipArcSweep -= 2 * Math.PI;
    while (tipArcSweep < -Math.PI) tipArcSweep += 2 * Math.PI;
    if (Math.atan2(tipCentre.x, tipCentre.y) <= 0) throw "tip rounding leaves a pointed or overlapping tooth";
    if (Math.atan2(rootFillet(1).x,
      rootFillet(1).y) >= Math.PI / spec.teeth) throw "root fillets overlap the adjacent tooth";
    evaluationRange();
  }

  public static function involute(angle:Float):Float return Math.tan(angle) - angle;

  public function gearSpec():GearSpec return spec;
  public function rootJoinRadius():Float return rootJoinR;
  public function tipJoinRadius():Float return tipJoinR;

  /** Half the tooth's angular thickness at `radius` on the active involute (radians). */
  public function halfAngleAt(radius:Float):Float {
    var a = Math.acos(Involute.baseRadius(spec) / radius);
    var h = spec.pitchToothThickness()
      / (2 * spec.pitchRadius()) + spec.sign() * (Math.tan(spec.form.pressureAngle()) - spec.form.pressureAngle() - Math.tan(a) + a);
    return relief == null ? h : h - reliefRemoval(radius) / Involute.baseRadius(spec);
  }

  /** Roll length from the base circle; the involute's natural profile abscissa. */
  public function profileCoordinate(radius:Float):Float {
    var rb = Involute.baseRadius(spec);
    return Math.sqrt(Math.max(0, radius * radius - rb * rb));
  }

  public function radiusAtCoordinate(u:Float):Float {
    var rb = Involute.baseRadius(spec);
    return Math.sqrt(rb * rb + u * u);
  }

  /** Angular error times this gives the flank-normal error: the base radius. */
  public function normalLever(radius:Float):Float return Involute.baseRadius(spec);

  /** Exact active involute. Radius outside the unrounded flank is rejected. */
  public function flankAtRadius(radius:Float):Point2 {
    var lo = Math.min(rootJoinR, tipJoinR), hi = Math.max(rootJoinR, tipJoinR);
    if (!Math.isFinite(radius)
      || radius < lo - 1e-12 || radius > hi + 1e-12) throw "radius is outside the active involute flank";
    return involuteAtRadius(radius);
  }

  function involuteAtRadius(radius:Float):Point2 {
    var alpha = Math.acos(Math.min(1, Involute.baseRadius(spec) / radius));
    var theta = half / spec.pitchRadius() + spec.sign() * (involute(spec.form.pressureAngle()) - involute(alpha));
    if (relief != null) theta -= reliefRemoval(radius) / Involute.baseRadius(spec);
    return new Point2(radius * Math.sin(theta), radius * Math.cos(theta));
  }

  /** Material removed along the flank normal at `radius` (0 without relief). */
  function reliefRemoval(radius:Float):Float {
    if (relief == null) return 0.0;
    return relief.removal(relief.length - spec.sign() * (spec.tipRadius() - radius));
  }

  function reliefSlope(radius:Float):Float {
    if (relief == null) return 0.0;
    return spec.sign() * relief.slope(relief.length - spec.sign() * (spec.tipRadius() - radius));
  }

  /** Outward unit normal of the (relieved) flank at a radius: the pure-involute normal turned by the relief slope. */
  function filletNormal(radius:Float):Point2 {
    var d = flankDerivativeAtRadius(radius), len = d.radius();
    var n = new Point2(d.y / len, -d.x / len);
    var rb = Involute.baseRadius(spec), p = involuteAtRadius(radius);
    var theta = Math.atan2(p.x, p.y), alpha = Math.acos(rb / radius);
    var nx0 = spec.sign() * Math.sin(alpha) * Math.sin(theta) + Math.cos(alpha) * Math.cos(theta);
    var ny0 = spec.sign() * Math.sin(alpha) * Math.cos(theta) - Math.cos(alpha) * Math.sin(theta);
    return n.x * nx0 + n.y * ny0 >= 0 ? n : n.scaled(-1);
  }

  public function flank(t:Float):Point2 {
    ToothOutline.checkParameter(t);
    return flankAtRadius(rootJoinR + t * (tipJoinR - rootJoinR));
  }

  /** Derivative w.r.t. radius, oriented towards increasing radius for either gear kind. */
  public function flankDerivative(t:Float):Point2 {
    var p = flank(t);
    return flankDerivativeAtRadius(p.radius(), Math.atan2(p.x, p.y));
  }

  function flankDerivativeAtRadius(radius:Float, ?angle:Float):Point2 {
    var theta = angle == null ? Math.atan2(involuteAtRadius(radius).x, involuteAtRadius(radius).y) : angle;
    var rb = Involute.baseRadius(spec);
    var tangent = Math.sqrt(radius * radius / (rb * rb) - 1);
    // r * dtheta/dr: the involute's -sign * tangent, less the relief slope over the base radius.
    var rTheta = -spec.sign() * tangent - (relief == null ? 0.0 : reliefSlope(radius) * radius / rb);
    return new Point2(Math.sin(theta) + rTheta * Math.cos(theta), Math.cos(theta) - rTheta * Math.sin(theta));
  }

  /** t=0: involute join; t=1: root circle, both tangent. This is not a circular blend. */
  public function rootFillet(t:Float):Point2 {
    ToothOutline.checkParameter(t);
    var beta = spec.form.pressureAngle() + t * (Math.PI / 2 - spec.form.pressureAngle());
    var v = cornerV - spec.cutterTipRadius * Math.sin(beta);
    var u = spec.sign() * (cornerU - spec.cutterTipRadius * Math.cos(beta));
    var x = spec.sign() * v * Math.cos(beta) / Math.sin(beta);
    var q = spec.sign() * spec.pitchRadius();
    return new Point2(x, q + v).rotated((x - u) / q).scaled(spec.sign());
  }

  public function rootDerivative(t:Float):Point2 {
    ToothOutline.checkParameter(t);
    var beta = spec.form.pressureAngle() + t * (Math.PI / 2 - spec.form.pressureAngle());
    var sn = Math.sin(beta), cs = Math.cos(beta), rho = spec.cutterTipRadius;
    var v = cornerV - rho * sn, dv = -rho * cs;
    var u = spec.sign() * (cornerU - rho * cs), du = spec.sign() * rho * sn;
    var x = spec.sign() * v * cs / sn;
    var dx = spec.sign() * (dv * cs / sn - v / (sn * sn));
    var q = spec.sign() * spec.pitchRadius(), phi = (x - u) / q, dphi = (dx - du) / q;
    return new Point2(dx - dphi * (q + v),
      dv + dphi * x).rotated(phi).scaled(spec.sign() * (Math.PI / 2 - spec.form.pressureAngle()));
  }

  /** Circular tip rounding from flank join to the tip-circle land. */
  public function tipFillet(t:Float):Point2 {
    ToothOutline.checkParameter(t);
    var angle = tipArcStart + t * tipArcSweep, rho = spec.tipFilletRadius;
    return new Point2(tipCentre.x + rho * Math.cos(angle), tipCentre.y + rho * Math.sin(angle));
  }

  public function tipDerivative(t:Float):Point2 {
    ToothOutline.checkParameter(t);
    var angle = tipArcStart + t * tipArcSweep;
    return new Point2(-Math.sin(angle), Math.cos(angle)).scaled(spec.tipFilletRadius * tipArcSweep);
  }

  public function regionAt(radius:Float):ToothRegion return ToothOutline.regionAt(this, radius);

  public function evaluationRange():GearEvaluationRange return ToothOutline.evaluationRange(
    spec, Math.min(rootJoinR, tipJoinR), Math.max(rootJoinR, tipJoinR));

  /** Closed polygon of one pitch, including root and tip lands. Mirror for the left flank. */
  public function outline(samples:Int):Array<Point2> return ToothOutline.of(this, samples);
}
