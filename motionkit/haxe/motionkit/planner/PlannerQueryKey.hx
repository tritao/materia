package motionkit.planner;

import haxe.io.Bytes;

/** Exact IEEE-754 keys: nearby configurations must never share collision answers. */
class PlannerQueryKey {
  public static function of(values:Array<Float>, start:Int = 0, ?count:Int):String {
    var length = count == null ? values.length - start : count;
    var bytes = Bytes.alloc(length * 8);
    for (i in 0...length) {
      var value = values[start + i];
      if (!Math.isFinite(value)) throw "Planner query coordinates must be finite";
      bytes.setDouble(i * 8, value);
    }
    return haxe.crypto.Base64.encode(bytes);
  }
}
