package robotkit.manipulation;

import robotkit.model.JointId;
import robotkit.model.JointLimits;

/**
 * An ordered, named selection of joints with their limits, independent of
 * any one chain. A joint whose limits have `lower >= upper` (the
 * `JointLimits` default, and the convention for unbounded/continuous
 * joints) is treated as unlimited and is never clamped.
 */
class JointGroup {
  public final jointIds:Array<JointId>;
  final limits:Array<JointLimits>;

  public function new(jointIds:Array<JointId>, limits:Array<JointLimits>) {
    if (jointIds == null || limits == null || jointIds.length == 0 || jointIds.length != limits.length)
      throw "Joint group requires matching, non-empty joint ID and limit lists";
    var seen = new Map<String, Bool>();
    for (id in jointIds) {
      if (id == null || id.length == 0) throw "Joint group entries require a non-empty joint ID";
      if (seen.exists(id)) throw 'Joint group contains joint "$id" more than once';
      seen.set(id, true);
    }
    this.jointIds = jointIds.copy();
    this.limits = limits.copy();
  }

  public static function fromChain(chain:KinematicChain):JointGroup {
    var joints = chain.dofJointsCopy();
    return new JointGroup([for (joint in joints) joint.id], [for (joint in joints) joint.limits]);
  }

  public function count():Int return jointIds.length;

  public function limitsOf(index:Int):JointLimits {
    if (index < 0 || index >= limits.length) throw 'Joint group has no joint at index $index';
    return limits[index];
  }

  /** Clamps each value to its joint's limits; leaves unlimited joints (lower >= upper) untouched. */
  public function clamp(q:Array<Float>):Array<Float> {
    if (q == null || q.length != limits.length)
      throw 'Joint group requires ${limits.length} values, got ${q == null ? 0 : q.length}';
    var result:Array<Float> = [];
    for (index in 0...q.length) {
      var limit = limits[index];
      var value = q[index];
      if (limit.lower < limit.upper) {
        if (value < limit.lower) value = limit.lower;
        if (value > limit.upper) value = limit.upper;
      }
      result.push(value);
    }
    return result;
  }
}
