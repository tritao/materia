package robotkit.behavior;

import haxe.Int64;
import robotkit.world.Robot;

/** Runs identical application behavior against simulated or remote adapters. */
class WorldBehaviorRunner {
  public final behavior:WorldBehavior;
  var hasObservation:Bool = false;
  var lastObservationKey:String = "";

  public function new(behavior:WorldBehavior) {
    if (behavior == null) throw "WorldBehaviorRunner requires a behavior";
    this.behavior = behavior;
  }

  public function update(robot:Robot):Int {
    var snapshot = robot.snapshot();
    var key = observationKey(snapshot);
    if (hasObservation && key == lastObservationKey) return 0;
    hasObservation = true;
    lastObservationKey = key;
    var commands:Array<robotkit.world.RobotCommand> = [];
    behavior.update(new WorldBehaviorContext(snapshot, commands));
    for (command in commands) robot.submit(command);
    return commands.length;
  }

  public function reset():Void { hasObservation = false; lastObservationKey = ""; }

  static function observationKey(snapshot:robotkit.world.RobotSnapshot):String {
    var key = '${snapshot.sourceClockId}:${Int64.toStr(snapshot.sourceSequence)}:'
      + '${Int64.toStr(snapshot.sourceTimestampNs)}:${snapshot.receivedClockId}:'
      + Int64.toStr(snapshot.receivedTimestampNs);
    for (sensor in snapshot.sensors.toArray())
      key += '|${sensor.sensorId}:${sensor.sourceClockId}:${Int64.toStr(sensor.sequence)}:'
        + '${Int64.toStr(sensor.sourceTimestampNs)}:${Int64.toStr(sensor.receivedTimestampNs)}';
    return key;
  }
}
