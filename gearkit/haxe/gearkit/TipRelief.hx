package gearkit;

/** Tip relief of an involute flank: material removed along the flank normal, zero at `length` (radial mm) below the
    tip circle and `amount` (mm) at the tip circle, growing as the `exponent` power of the distance from the start
    (1 linear, 2 near-circular, tangent at the start). Applies to external and internal teeth alike (the tooth is
    thinned either way). A general form parameter, not specific to any study. */
class TipRelief {
  public final amount:Float;
  public final length:Float;
  public final exponent:Float;

  public function new(amount:Float, length:Float, exponent:Float = 1.0) {
    if (!Math.isFinite(amount) || amount <= 0) throw "tip relief amount must be finite and positive";
    if (!Math.isFinite(length) || length <= 0) throw "tip relief length must be finite and positive";
    if (!Math.isFinite(exponent) || exponent < 1) throw "tip relief exponent must be finite and at least one";
    this.amount = amount;
    this.length = length;
    this.exponent = exponent;
  }

  /** Material removed (mm along the flank normal) at distance `s` (radial mm, from the relief start towards the tip). */
  public function removal(s:Float):Float {
    if (s <= 0) return 0.0;
    return amount * Math.pow(Math.min(s, length) / length, exponent);
  }

  /** Derivative of `removal` with respect to `s`. */
  public function slope(s:Float):Float {
    if (s <= 0 || s >= length) return 0.0;
    return amount * exponent * Math.pow(s / length, exponent - 1) / length;
  }
}
