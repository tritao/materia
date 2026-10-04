package robotkit.mobile;

/** Small planar helpers missing from the embedded Haxeon Math surface. */
class PlanarMath {
  public static function atan(value:Float):Float {
    if (!Math.isFinite(value)) throw "atan input must be finite";
    var sign = value < 0.0 ? -1.0 : 1.0;
    var x = Math.abs(value);
    var outerOffset = 0.0;
    var outerSign = 1.0;
    var localOffset = 0.0;
    if (x > 1.0) {
      x = 1.0 / x;
      outerOffset = Math.PI * 0.5;
      outerSign = -1.0;
    }
    if (x > 0.5) {
      x = (x - 1.0) / (x + 1.0);
      localOffset = Math.PI * 0.25;
    }
    // Range reduction keeps the alternating series within |x| <= 0.5.
    var squared = x * x;
    var power = x;
    var result = x;
    for (index in 1...13) {
      power *= -squared;
      result += power / (index * 2.0 + 1.0);
    }
    return sign * (outerOffset + outerSign * (localOffset + result));
  }

  public static function atan2(y:Float, x:Float):Float {
    if (!Math.isFinite(x) || !Math.isFinite(y))
      throw "atan2 inputs must be finite";
    if (x > 0.0) return atan(y / x);
    if (x < 0.0) return y >= 0.0 ? atan(y / x) + Math.PI : atan(y / x) - Math.PI;
    if (y > 0.0) return Math.PI * 0.5;
    if (y < 0.0) return -Math.PI * 0.5;
    return 0.0;
  }
}
