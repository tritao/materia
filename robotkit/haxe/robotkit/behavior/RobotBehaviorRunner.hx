package robotkit.behavior;

import haxe.Int64;
import robotkit.runtime.RobotSnapshot;
import robotkit.world.RobotEvent;

/** Runs one behavior against each new snapshot and returns its live intent. */
class RobotBehaviorRunner {
  public final behavior:RobotBehavior;
  public final intents:IntentBuffer;
  var hasSnapshot:Bool = false;
  var lastSequence:Int64 = Int64.ofInt(0);
  final pendingEvents:Array<RobotEvent> = [];

  public function new(behavior:RobotBehavior) {
    if (behavior == null)
      throw "RobotKit behavior runner requires a behavior";
    this.behavior = behavior;
    intents = new IntentBuffer();
  }

  public function update(snapshot:RobotSnapshot, nowNs:Int64):Null<JointTargetIntent> {
    if (snapshot == null)
      return null;
    if (!hasSnapshot || Int64.compare(snapshot.sequence, lastSequence) != 0 || pendingEvents.length > 0) {
      hasSnapshot = true;
      lastSequence = snapshot.sequence;
      var delivered = pendingEvents.copy();
      pendingEvents.resize(0);
      behavior.update(new RobotContext(snapshot, intents, delivered));
    }
    return intents.current(nowNs);
  }

  public function offerEvent(event:RobotEvent):Void pendingEvents.push(event);

  public function reset():Void {
    hasSnapshot = false;
    lastSequence = Int64.ofInt(0);
    intents.clear();
    pendingEvents.resize(0);
  }
}
