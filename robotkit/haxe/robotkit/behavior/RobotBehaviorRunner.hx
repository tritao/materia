package robotkit.behavior;

import haxe.Int64;
import robotkit.runtime.RobotSnapshot;

/** Runs one behavior against each new snapshot and returns its live intent. */
class RobotBehaviorRunner {
  public final behavior:RobotBehavior;
  public final intents:IntentBuffer;
  var hasSnapshot:Bool = false;
  var lastSequence:Int64 = Int64.ofInt(0);

  public function new(behavior:RobotBehavior) {
    if (behavior == null)
      throw "RobotKit behavior runner requires a behavior";
    this.behavior = behavior;
    intents = new IntentBuffer();
  }

  public function update(snapshot:RobotSnapshot, nowNs:Int64):Null<JointTargetIntent> {
    if (snapshot == null)
      return null;
    if (!hasSnapshot || Int64.compare(snapshot.sequence, lastSequence) != 0) {
      hasSnapshot = true;
      lastSequence = snapshot.sequence;
      behavior.update(new RobotContext(snapshot, intents));
    }
    return intents.current(nowNs);
  }

  public function reset():Void {
    hasSnapshot = false;
    lastSequence = Int64.ofInt(0);
    intents.clear();
  }
}
