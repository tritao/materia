package gearkit;

/** Double-circular-arc common-tangent flexspline tooth (Liu et al. 2025, Machines 13, 535, section 2.1),
    TOOTH_DESIGN_PLAN.md TD2. Convex arc AB (r1, centre O1), common tangent BC, concave arc CD (r2, centre O2).

    The construction is made in a flat frame rebuilt from the definitions, not from the printed equations
    (see the plan's TD2 notes for every deviation). Frame: x from the tooth's symmetry axis, y measured from
    the pitch line, so the paper's `ds` (neutral layer to dedendum) cancels and never enters the tooth shape.
    O1 = (-c1, -e1), O2 = (pi m / 2 + c2, e2), addendum line y = ha, dedendum line y = -hf. The right flank
    runs from D on the dedendum line up through C, B to A on the addendum line.

    Parameters are lengths in mm for module `referenceModule`; `liu` scales the paper's set to a module.
    Construction throws naming the offending parameter. The form is external-gear only, no profile shift. */
class DoubleCircularArc implements ToothForm {
  /** Liu 2025 Table 2 optimum (m 0.5), with c2 and e2 un-swapped (Table 2 lists them the wrong way round). */
  public static final LIU_MODULE = 0.5;
  public static final LIU = [0.68517449, 0.33183981, 0.15535155, 0.78482551, 0.32994565, 0.13439061, 0.29316387, 0.35425825];

  public final referenceModule:Float;
  public final r1:Float;
  public final c1:Float;
  public final e1:Float;
  public final r2:Float;
  public final c2:Float;
  public final e2:Float;
  public final ha:Float;
  public final hf:Float;
  /** Centres O1, O2. */
  public final o1x:Float;
  public final o1y:Float;
  public final o2x:Float;
  public final o2y:Float;
  /** Common tangential inclination angle: BC against the symmetry axis. */
  public final delta:Float;
  /** Angles (from the horizontal at the arc centres) of A on arc 1 and D on arc 2. */
  public final alpha:Float;
  public final beta:Float;
  public final bx:Float;
  public final by:Float;
  public final cx:Float;
  public final cy:Float;
  public final ax:Float;
  public final dx:Float;
  /** Longitudinal (y) length of the tangent BC. */
  public final h1:Float;

  public function new(referenceModule:Float, r1:Float, c1:Float, e1:Float, r2:Float, c2:Float, e2:Float, ha:Float, hf:Float) {
    for (v in [referenceModule, r1, c1, e1, r2, c2, e2, ha, hf]) if (!Math.isFinite(v)) throw "double-circular-arc parameters must be finite";
    if (referenceModule <= 0) throw "double-circular-arc module must be positive";
    if (r1 <= 0) throw "double-circular-arc parameter r1 must be positive";
    if (r2 <= 0) throw "double-circular-arc parameter r2 must be positive";
    if (ha <= 0) throw "double-circular-arc parameter ha must be positive";
    if (hf <= 0) throw "double-circular-arc parameter hf must be positive";
    this.referenceModule = referenceModule;
    this.r1 = r1;
    this.c1 = c1;
    this.e1 = e1;
    this.r2 = r2;
    this.c2 = c2;
    this.e2 = e2;
    this.ha = ha;
    this.hf = hf;
    o1x = -c1;
    o1y = -e1;
    o2x = referenceModule * Math.PI / 2 + c2;
    o2y = e2;
    var sa = (ha + e1) / r1, sd = (e2 + hf) / r2;
    if (sa >= 1) throw 'double-circular-arc parameter ha: addendum line is above the convex arc (ha + e1 = ${ha + e1} must be below r1 = $r1)';
    if (sd >= 1) throw 'double-circular-arc parameter hf: dedendum line is below the concave arc (e2 + hf = ${e2 + hf} must be below r2 = $r2)';
    alpha = Math.asin(sa);
    beta = Math.asin(sd);
    var gx = o2x - o1x, gy = o2y - o1y, d = Math.sqrt(gx * gx + gy * gy);
    if (d <= r1 + r2) throw 'double-circular-arc parameters r1, r2, c1, c2, e1, e2: the arcs overlap (centre distance $d <= r1 + r2 = ${r1 + r2}), no common internal tangent exists';
    // Internal tangent. With n the outward normal at B, (O2 - O1).n = r1 + r2, so n is the direction of
    // O1->O2 turned by +-acos((r1+r2)/d). n = (cos delta, sin delta); the tangent line runs (sin delta, -cos delta).
    // Only the +eps root has B above C (checked below); the -eps root is the crossing tangent.
    delta = Math.atan2(gy, gx) + Math.acos((r1 + r2) / d);
    var nx = Math.cos(delta), ny = Math.sin(delta);
    bx = o1x + r1 * nx;
    by = o1y + r1 * ny;
    cx = o2x - r2 * nx;
    cy = o2y - r2 * ny;
    h1 = by - cy;
    if (h1 <= 0) throw 'double-circular-arc parameters r1, r2, e1, e2: tangent BC does not run downward from B to C (h1 = $h1)';
    if (delta <= 0 || delta >= Math.PI / 2) throw 'double-circular-arc parameters: tangent inclination delta = $delta rad is outside (0, pi/2)';
    if (ha <= by) throw 'double-circular-arc parameter ha: addendum $ha does not reach above B (y = $by); the convex arc has no length';
    if (-hf >= cy) throw 'double-circular-arc parameter hf: dedendum $hf does not reach below C (y = $cy); the concave arc has no length';
    ax = o1x + r1 * Math.cos(alpha);
    dx = o2x - r2 * Math.cos(beta);
    if (ax <= 0) throw 'double-circular-arc parameters c1, r1, e1, ha: the tooth is pointed or inverted at A (half thickness $ax)';
    if (dx >= referenceModule * Math.PI / 2) throw 'double-circular-arc parameters c2, r2, e2, hf: point D (x = $dx) reaches the middle of the tooth space ${referenceModule * Math.PI / 2}; teeth overlap';
  }

  /** The Liu 2025 optimum scaled by `module / 0.5`; every length scales, the shape does not change. */
  public static function liu(module:Float):DoubleCircularArc {
    var k = module / LIU_MODULE, p = LIU;
    return new DoubleCircularArc(module, p[0] * k, p[1] * k, p[2] * k, p[3] * k, p[4] * k, p[5] * k, p[6] * k, p[7] * k);
  }

  public function name():String return "double-circular-arc";
  public function addendum(module:Float):Float return ha;
  public function dedendum(module:Float):Float return hf;
  public function pressureAngle():Float return normalAngleAt(0);
  public function pitchToothThickness(spec:GearSpec):Float return 2 * xAt(0);

  public function validate(spec:GearSpec):Void {
    if (Math.abs(spec.module - referenceModule) > 1e-12 * referenceModule) throw 'double-circular-arc parameters are for module $referenceModule, not ${spec.module}; scale them (DoubleCircularArc.liu)';
    if (spec.internal) throw "double-circular-arc is a flexspline (external) form; internal gears are not supported";
    if (spec.profileShift != 0) throw "double-circular-arc has no profile shift; profile shift must be 0";
  }

  public function profile(spec:GearSpec):ToothProfile return new DoubleArcProfile(spec, this);

  /** Height of C and B (y), the segment boundaries: arc 2 below `cy`, tangent between, arc 1 above `by`. */
  function checkY(y:Float):Void {
    if (!Math.isFinite(y) || y < -hf - 1e-12 || y > ha + 1e-12) throw "flank height is outside the dedendum..addendum range";
  }

  /** Right-flank x at height y (from the pitch line), dedendum to addendum. */
  public function xAt(y:Float):Float {
    checkY(y);
    if (y <= cy) return o2x - Math.sqrt(Math.max(0, r2 * r2 - (y - o2y) * (y - o2y)));
    if (y <= by) return bx + (by - y) * Math.tan(delta);
    return o1x + Math.sqrt(Math.max(0, r1 * r1 - (y - o1y) * (y - o1y)));
  }

  /** dx/dy of the flank (negative: the tooth widens downward). */
  public function dxdyAt(y:Float):Float {
    checkY(y);
    if (y <= cy) return (y - o2y) / Math.sqrt(r2 * r2 - (y - o2y) * (y - o2y));
    if (y <= by) return -Math.tan(delta);
    return -(y - o1y) / Math.sqrt(r1 * r1 - (y - o1y) * (y - o1y));
  }

  /** Angle of the flank normal from the horizontal (radians): the local pressure angle. */
  public function normalAngleAt(y:Float):Float {
    checkY(y);
    if (y <= cy) return Math.asin((o2y - y) / r2);
    if (y <= by) return delta;
    return Math.asin((y - o1y) / r1);
  }

  /** Length along the flank from D (y = -hf) up to height y. */
  public function lengthAt(y:Float):Float {
    checkY(y);
    var sC = r2 * (beta - delta);
    if (y <= cy) return r2 * (beta - Math.asin((o2y - y) / r2));
    var sB = sC + h1 / Math.cos(delta);
    if (y <= by) return sC + (y - cy) / Math.cos(delta);
    return sB + r1 * (Math.asin((y - o1y) / r1) - delta);
  }

  /** Inverse of `lengthAt`. */
  public function heightAtLength(s:Float):Float {
    var sC = r2 * (beta - delta), sB = sC + h1 / Math.cos(delta), sA = sB + r1 * (alpha - delta);
    if (!Math.isFinite(s) || s < -1e-12 || s > sA + 1e-12) throw "flank length is outside the profile";
    if (s <= sC) return o2y - r2 * Math.sin(beta - s / r2);
    if (s <= sB) return cy + (s - sC) * Math.cos(delta);
    return o1y + r1 * Math.sin(delta + (s - sB) / r1);
  }
}
