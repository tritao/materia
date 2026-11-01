package machinekit.component;

import runtime.FloatBits;

/** Canonical binary64 recipe input encoding, separate from display dimensions. */
class ComponentNumberKey {
  public static function encode(value:Float):String {
    if (!Math.isFinite(value)) throw "Recipe number must be finite";
    // Positive and negative zero describe the same engineering value.
    var bits = FloatBits.toInt64(value == 0 ? 0.0 : value);
    return haxe.Int64.toStr(bits);
  }
}
