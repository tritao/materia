package robotkit.behavior;

import haxe.Int64;

/** Bounded latest-intent buffer; stale behavior output is discarded. */
class IntentBuffer {
  var latest:Null<JointTargetIntent> = null;

  public function new() {}

  public function publish(intent:JointTargetIntent):Void {
    if (intent == null)
      throw "RobotKit intent buffer cannot publish null";
    latest = intent;
  }

  public function current(nowNs:Int64):Null<JointTargetIntent> {
    var intent = latest;
    if (intent == null)
      return null;
    if (Int64.compare(intent.expiryNs, Int64.ofInt(0)) != 0
        && Int64.compare(intent.expiryNs, nowNs) < 0) {
      latest = null;
      return null;
    }
    return intent;
  }

  public function clear():Void
    latest = null;
}
