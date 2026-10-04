package robotkit.behavior;

import haxe.Int64;
import robotkit.core.Robot;

/** Runs identical application behavior against simulated or remote adapters. */
class WorldBehaviorRunner {
  public final behavior:WorldBehavior;
  var hasObservation:Bool = false;
  var lastObservationKey:String = "";
  var lastEventOrdinal:Int64 = Int64.ofInt(0);

  public function new(behavior:WorldBehavior) {
    if (behavior == null) throw "WorldBehaviorRunner requires a behavior";
    this.behavior = behavior;
  }

  public function update(robot:Robot):Int {
    var snapshot = robot.snapshot();
    var key = observationKey(snapshot);
    var events = robot.events(lastEventOrdinal, 256);
    if (hasObservation && key == lastObservationKey && events.length == 0) return 0;
    if (events.length > 0) lastEventOrdinal = robotkit.core.RobotEventRing.ordinalOf(events[events.length - 1]);
    hasObservation = true;
    lastObservationKey = key;
    var commands:Array<robotkit.core.RobotCommand> = [];
    behavior.update(new WorldBehaviorContext(snapshot, events, commands, robot.streams().latestFrames()));
    for (command in commands) robot.submit(command);
    return commands.length;
  }

  public function reset():Void { hasObservation = false; lastObservationKey = ""; lastEventOrdinal = Int64.ofInt(0); }

  static function observationKey(snapshot:robotkit.core.RobotSnapshot):String {
    var key = '${snapshot.sourceClockId}:${Int64.toStr(snapshot.sourceSequence)}:'
      + '${Int64.toStr(snapshot.sourceTimestampNs)}:${snapshot.receivedClockId}:'
      + Int64.toStr(snapshot.receivedTimestampNs);
    for (sensor in snapshot.sensors.toArray())
      key += '|${sensor.sensorId}:${sensor.sourceClockId}:${Int64.toStr(sensor.sequence)}:'
        + '${Int64.toStr(sensor.sourceTimestampNs)}:${Int64.toStr(sensor.receivedTimestampNs)}';
    for (stream in snapshot.streamSequences)
      key += '|${stream.streamId}:${stream.sourceClockId}:${Int64.toStr(stream.sequence)}';
    return key;
  }
}
