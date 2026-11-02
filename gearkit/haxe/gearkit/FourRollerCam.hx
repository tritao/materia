package gearkit;

/** Four-arc oval standing in for a four-roller wave generator: contact is on a circular arc
    of radius `lobeRadius` centred on the major axis (|phi| < joinAngle) and on a larger arc
    centred on the minor axis, tangent at the join. Semi-axes are `neutralRadius +/- w0` before
    the mean radius is removed. The lobe radius (default half the minor semi-axis) is an
    assumption, not a published roller geometry. */
class FourRollerCam extends ShapedCam {
  public final lobeRadius:Float;
  public final sideRadius:Float;
  /** Centres on the major / minor axis (signed distance from the origin). */
  public final lobeCentre:Float;
  public final sideCentre:Float;
  public final joinAngle:Float;

  /** [lobe radius, side radius, lobe centre, side centre, join angle] for semi-axes a = n + w0, b = n - w0. */
  static function geometry(neutralRadius:Float, w0:Float, lobe:Null<Float>):Array<Float> {
    var a = neutralRadius + w0, b = neutralRadius - w0;
    var r1 = lobe == null ? 0.5 * b : lobe;
    if (!Math.isFinite(neutralRadius) || neutralRadius <= w0) throw "neutral radius must exceed the cam amplitude";
    if (!Math.isFinite(r1) || r1 <= 0 || r1 >= b) throw "lobe radius must be positive and below the minor semi-axis";
    // Internally tangent circles: |C1 - C2| = R2 - R1 with C1 = (a - R1, 0), C2 = (0, b - R2).
    var r2 = ((a - r1) * (a - r1) + b * b - r1 * r1) / (2 * (b - r1));
    var c1 = a - r1, c2 = b - r2, d = Math.sqrt(c1 * c1 + c2 * c2);
    var jx = c1 + r1 * c1 / d, jy = r1 * (-c2) / d;
    return [r1, r2, c1, c2, Math.atan2(jy, jx)];
  }

  public function new(neutralRadius:Float, w0:Float, ?lobeRadius:Float) {
    super("four-roller", w0, [geometry(neutralRadius, w0, lobeRadius)[4]]);
    var g = geometry(neutralRadius, w0, lobeRadius);
    this.lobeRadius = g[0];
    this.sideRadius = g[1];
    this.lobeCentre = g[2];
    this.sideCentre = g[3];
    this.joinAngle = g[4];
    normalise();
  }

  /** Ray from the origin at angle phi meets the circle of centre (cx, cy), radius r. */
  static function hit(cx:Float, cy:Float, r:Float, phi:Float):Float {
    var p = cx * Math.cos(phi) + cy * Math.sin(phi);
    return p + Math.sqrt(p * p - cx * cx - cy * cy + r * r);
  }

  static function dhit(cx:Float, cy:Float, r:Float, phi:Float):Float {
    var p = cx * Math.cos(phi) + cy * Math.sin(phi), dp = -cx * Math.sin(phi) + cy * Math.cos(phi);
    return dp + p * dp / Math.sqrt(p * p - cx * cx - cy * cy + r * r);
  }

  override function rho(phi:Float):Float return phi <= joinAngle
    ? hit(lobeCentre, 0, lobeRadius, phi) : hit(0, sideCentre, sideRadius, phi);

  override function drho(phi:Float):Float return phi <= joinAngle
    ? dhit(lobeCentre, 0, lobeRadius, phi) : dhit(0, sideCentre, sideRadius, phi);

  /** Physical (mean-unremoved) profile point at phi, for checking arc contact. */
  public function profilePoint(phi:Float):Point2 {
    var r = rho(fold(phi).x);
    return new Point2(r * Math.cos(phi), r * Math.sin(phi));
  }
}
