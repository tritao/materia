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

  public function new(spec:GearSpec) {
    this.spec = spec;
    var a = spec.pressureAngle, rho = spec.cutterTipRadius;
    half = spec.pitchToothThickness() / 2;
    depth = spec.module * (1.25 - spec.sign() * spec.profileShift);
    cornerV = -depth + rho;
    cornerU = half + depth * Math.tan(a) + rho * (1 / Math.cos(a) - Math.tan(a));
    if (cornerU >= Math.PI * spec.module / 2) throw "rack corner leaves no cutter tip land";
    var vJoin = cornerV - rho * Math.sin(a);
    if (!spec.internal && vJoin <= -spec.pitchRadius() * Math.pow(Math.sin(a),
      2)) throw "rack-generated tooth is undercut; increase tooth count or profile shift";
    if (spec.internal && rho * spec.pitchRadius() <= cornerV * vJoin / Math.pow(Math.sin(a),
      3)) throw "internal rack-limit root is undercut; use more teeth, less shift or a pinion-shaped cutter";
    rootJoinR = rootFillet(0).radius();
    var rb = spec.baseRadius(), rt = spec.tipRadius(), radius = spec.tipFilletRadius;
    var rc = rt - spec.sign() * radius;
    if (rc <= rb) throw "tip fillet centre lies below the base circle";
    var roll = spec.sign() * radius + Math.sqrt(rc * rc - rb * rb);
    if (roll <= 0) throw "tip fillet cannot meet this involute";
    tipJoinR = Math.sqrt(rb * rb + roll * roll);
    if (spec.sign() * (tipJoinR - rootJoinR) <= 0) throw "root and tip fillets overlap";
    var p = involuteAtRadius(tipJoinR);
    var theta = Math.atan2(p.x, p.y), alpha = Math.acos(rb / tipJoinR);
    var nx = spec.sign() * Math.sin(alpha) * Math.sin(theta) + Math.cos(alpha) * Math.cos(theta);
    var ny = spec.sign() * Math.sin(alpha) * Math.cos(theta) - Math.cos(alpha) * Math.sin(theta);
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
    var a = Math.acos(spec.baseRadius() / radius);
    return spec.pitchToothThickness()
      / (2 * spec.pitchRadius()) + spec.sign() * (Math.tan(spec.pressureAngle) - spec.pressureAngle - Math.tan(a) + a);
  }

  /** Roll length from the base circle; the involute's natural profile abscissa. */
  public function profileCoordinate(radius:Float):Float {
    var rb = spec.baseRadius();
    return Math.sqrt(Math.max(0, radius * radius - rb * rb));
  }

  public function radiusAtCoordinate(u:Float):Float {
    var rb = spec.baseRadius();
    return Math.sqrt(rb * rb + u * u);
  }

  /** Angular error times this gives the flank-normal error: the base radius. */
  public function normalLever(radius:Float):Float return spec.baseRadius();

  /** Exact active involute. Radius outside the unrounded flank is rejected. */
  public function flankAtRadius(radius:Float):Point2 {
    var lo = Math.min(rootJoinR, tipJoinR), hi = Math.max(rootJoinR, tipJoinR);
    if (!Math.isFinite(radius)
      || radius < lo - 1e-12 || radius > hi + 1e-12) throw "radius is outside the active involute flank";
    return involuteAtRadius(radius);
  }

  function involuteAtRadius(radius:Float):Point2 {
    var alpha = Math.acos(Math.min(1, spec.baseRadius() / radius));
    var theta = half / spec.pitchRadius() + spec.sign() * (involute(spec.pressureAngle) - involute(alpha));
    return new Point2(radius * Math.sin(theta), radius * Math.cos(theta));
  }

  public function flank(t:Float):Point2 {
    unit(t);
    return flankAtRadius(rootJoinR + t * (tipJoinR - rootJoinR));
  }

  /** Derivative w.r.t. radius, oriented towards increasing radius for either gear kind. */
  public function flankDerivative(t:Float):Point2 {
    var p = flank(t), radius = p.radius(), theta = Math.atan2(p.x, p.y);
    var tangent = Math.sqrt(radius * radius / (spec.baseRadius() * spec.baseRadius()) - 1);
    return new Point2(
      Math.sin(theta) - spec.sign() * tangent * Math.cos(theta),
      Math.cos(theta) + spec.sign() * tangent * Math.sin(theta)
    );
  }

  /** t=0: involute join; t=1: root circle, both tangent. This is not a circular blend. */
  public function rootFillet(t:Float):Point2 {
    unit(t);
    var beta = spec.pressureAngle + t * (Math.PI / 2 - spec.pressureAngle);
    var v = cornerV - spec.cutterTipRadius * Math.sin(beta);
    var u = spec.sign() * (cornerU - spec.cutterTipRadius * Math.cos(beta));
    var x = spec.sign() * v * Math.cos(beta) / Math.sin(beta);
    var q = spec.sign() * spec.pitchRadius();
    return new Point2(x, q + v).rotated((x - u) / q).scaled(spec.sign());
  }

  public function rootDerivative(t:Float):Point2 {
    unit(t);
    var beta = spec.pressureAngle + t * (Math.PI / 2 - spec.pressureAngle);
    var sn = Math.sin(beta), cs = Math.cos(beta), rho = spec.cutterTipRadius;
    var v = cornerV - rho * sn, dv = -rho * cs;
    var u = spec.sign() * (cornerU - rho * cs), du = spec.sign() * rho * sn;
    var x = spec.sign() * v * cs / sn;
    var dx = spec.sign() * (dv * cs / sn - v / (sn * sn));
    var q = spec.sign() * spec.pitchRadius(), phi = (x - u) / q, dphi = (dx - du) / q;
    return new Point2(dx - dphi * (q + v),
      dv + dphi * x).rotated(phi).scaled(spec.sign() * (Math.PI / 2 - spec.pressureAngle));
  }

  /** Circular tip rounding from flank join to the tip-circle land. */
  public function tipFillet(t:Float):Point2 {
    unit(t);
    var angle = tipArcStart + t * tipArcSweep, rho = spec.tipFilletRadius;
    return new Point2(tipCentre.x + rho * Math.cos(angle), tipCentre.y + rho * Math.sin(angle));
  }

  public function tipDerivative(t:Float):Point2 {
    unit(t);
    var angle = tipArcStart + t * tipArcSweep;
    return new Point2(-Math.sin(angle), Math.cos(angle)).scaled(spec.tipFilletRadius * tipArcSweep);
  }

  public function evaluationRange():GearEvaluationRange {
    var lo = Math.min(rootJoinR, tipJoinR), hi = Math.max(rootJoinR, tipJoinR);
    var result = spec.evaluation;
    if (result == null) return new GearEvaluationRange(lo, hi, 0, spec.faceWidth);
    if (result.radiusMin < lo || result.radiusMax > hi || result.axialMax
      > spec.faceWidth) throw "evaluation window must lie on the unrounded flank and within the face";
    return result;
  }

  /** Closed polygon of one pitch, including root and tip lands. Mirror for the left flank. */
  public function outline(samples:Int):Array<Point2> {
    if (samples < 4) throw "profile needs at least four samples per segment";
    var side:Array<Point2> = [];
    for (i in 0...samples + 1) side.push(rootFillet(1 - i / samples));
    for (i in 1...samples + 1) side.push(flank(i / samples));
    for (i in 1...samples + 1) side.push(tipFillet(i / samples));
    return ToothOutline.close(spec, side, samples);
  }

  static function unit(t:Float):Void {
    if (!Math.isFinite(t) || t < 0 || t > 1) throw "curve parameter must be in [0, 1]";
  }
}
