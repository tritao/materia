package robotkit.behavior;

import haxe.Int64;
import robotkit.runtime.RobotSnapshot;

/** Read-only snapshot plus bounded intent output exposed to a behavior. */
class RobotContext {
  public static inline final DEFAULT_INTENT_LIFETIME_NS:Int = 50_000_000;

  public final snapshot:RobotSnapshot;
  final intents:IntentBuffer;

  @:allow(RobotBehaviorRunner)
  function new(snapshot:RobotSnapshot, intents:IntentBuffer) {
    this.snapshot = snapshot;
    this.intents = intents;
  }

  public function jointTarget(joint:Int, mode:Int, target:Float,
      ?expiryNs:Int64):Void {
    var expiry = expiryNs == null
      ? Int64.add(snapshot.timestampNs, Int64.ofInt(DEFAULT_INTENT_LIFETIME_NS))
      : expiryNs;
    intents.publish(new JointTargetIntent(joint, mode, target, expiry));
  }
}
