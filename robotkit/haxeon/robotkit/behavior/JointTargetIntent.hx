package robotkit.behavior;

import haxe.Int64;

/** One expiring joint command emitted by a Haxeon behavior. */
class JointTargetIntent {
  public final joint:Int;
  public final mode:Int;
  public final target:Float;
  public final expiryNs:Int64;

  public function new(joint:Int, mode:Int, target:Float, expiryNs:Int64) {
    this.joint = joint;
    this.mode = mode;
    this.target = target;
    this.expiryNs = expiryNs;
  }
}
