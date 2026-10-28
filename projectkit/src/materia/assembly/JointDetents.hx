package materia.assembly;

import haxe.ds.ReadOnlyArray;
import materia.assembly.AssemblyDefinition.AssemblyJointLimits;

/**
 * The fixed positions a joint rests at (`AssemblyJointLimits.detents`): a rotary switch's clicks, a selector's throws. A joint
 * with detents is a switch, not a free joint, so editors place it only at a detent unless the user asks for free motion.
 * Values are in the joint's own units (radians or the assembly's length unit).
 */
class JointDetents {
  /** Detents are finite, strictly ascending and within the travel limits that are set; at least one. */
  public static function valid(detents:Null<ReadOnlyArray<Float>>, lower:Null<Float>, upper:Null<Float>):Bool {
    if (detents == null) return true;
    var list:Array<Float> = cast detents;
    if (list.length == 0) return false;
    for (i in 0...list.length) {
      var value = list[i];
      if (!Math.isFinite(value) || (i > 0 && value <= list[i - 1])) return false;
      if ((lower != null && value < lower - 1e-9) || (upper != null && value > upper + 1e-9)) return false;
    }
    return true;
  }

  /** Whether the limits give the joint detents. */
  public static function has(limits:Null<AssemblyJointLimits>):Bool {
    if (limits == null || limits.detents == null) return false;
    var list:Array<Float> = cast limits.detents;
    return list.length > 0;
  }

  /** The detent closest to `value`. */
  public static function nearest(detents:ReadOnlyArray<Float>, value:Float):Float {
    var list:Array<Float> = cast detents;
    var best = list[0];
    for (candidate in list) if (Math.abs(candidate - value) < Math.abs(best - value)) best = candidate;
    return best;
  }

  /** `value` moved to the nearest detent when the joint has any, otherwise unchanged. */
  public static function snap(limits:Null<AssemblyJointLimits>, value:Float):Float {
    if (limits == null) return value;
    var detents = limits.detents;
    if (detents == null) return value;
    var list:Array<Float> = cast detents;
    return list.length == 0 ? value : nearest(detents, value);
  }

  /** The index of the detent at `value`, or -1 when it is between two. */
  public static function indexAt(detents:ReadOnlyArray<Float>, value:Float, tolerance:Float = 1e-6):Int {
    var list:Array<Float> = cast detents;
    for (i in 0...list.length) if (Math.abs(list[i] - value) <= tolerance) return i;
    return -1;
  }
}
