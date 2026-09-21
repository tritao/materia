package robotkit.behavior;

import haxe.Int64;
import robotkit.world.Robot;

/** Runs identical application behavior against simulated or remote adapters. */
class WorldBehaviorRunner {
  public final behavior:WorldBehavior;
  var lastSequence:Int64 = Int64.ofInt(-1);

  public function new(behavior:WorldBehavior) {
    if (behavior == null) throw "WorldBehaviorRunner requires a behavior";
    this.behavior = behavior;
  }

  public function update(robot:Robot):Int {
    var snapshot = robot.snapshot();
    if (Int64.compare(snapshot.sourceSequence, lastSequence) == 0) return 0;
    lastSequence = snapshot.sourceSequence;
    var commands:Array<robotkit.world.RobotCommand> = [];
    behavior.update(new WorldBehaviorContext(snapshot, commands));
    for (command in commands) robot.submit(command);
    return commands.length;
  }

  public function reset():Void lastSequence = Int64.ofInt(-1);
}
