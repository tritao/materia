package robotkit.world;

import haxe.Int64;

/** Immutable-by-ownership view of all observable world state. */
class WorldSnapshot {
  public final sequence:Int;
  public final topologyRevision:Int;
  /** Zero for a mixed-clock world; inspect each robot's source timestamp. */
  public final sourceTimestampNs:Int64;
  public final receivedTimestampNs:Int64;
  final robotMap:Map<RobotId, RobotSnapshot>;

  /** Compatibility alias for callers that used the old single timestamp. */
  public var timestampNs(get, never):Int64;

  public function new(sequence:Int, topologyRevision:Int, sourceTimestampNs:Int64,
      source:Map<RobotId, RobotSnapshot>, ?receivedTimestampNs:Int64) {
    this.sequence = sequence;
    this.topologyRevision = topologyRevision;
    this.sourceTimestampNs = sourceTimestampNs;
    this.receivedTimestampNs = receivedTimestampNs == null
      ? Int64.ofInt(0)
      : receivedTimestampNs;
    robotMap = new Map<RobotId, RobotSnapshot>();
    for (id in source.keys()) {
      var value = source.get(id);
      if (value != null) robotMap.set(
        id,
        new RobotSnapshot(
          value.id,
          value.sourceSequence,
          value.sourceTimestampNs,
          value.positions.toArray(),
          value.velocities.toArray(),
          value.efforts.toArray(),
          value.mode,
          value.faultCode,
          value.receivedTimestampNs,
          value.sensors.toArray(),
          value.sourceClockId,
          value.receivedClockId,
          value.safety,
          value.trajectoryQueueDepth,
          value.trajectoryActive,
          value.trajectoryTimeNs,
          value.trajectoryDurationNs
        )
      );
    }
  }

  /** Looks up one immutable robot observation by logical ID. */
  public function robot(id:RobotId):Null<RobotSnapshot> return robotMap.get(id);

  public function robotIds():Array < RobotId > {
    var result:Array<RobotId> = [];
    for (id in robotMap.keys()) result.push(id);
    result.sort(function(left, right) return Reflect.compare(left, right));
    return result;
  }

  /** Returns a copy of the immutable observations in deterministic ID order. */
  public function robots():Array<RobotSnapshot> {
    var result:Array<RobotSnapshot> = [];
    for (id in robotIds()) result.push(robotMap.get(id));
    return result;
  }

  inline function get_timestampNs():Int64 return sourceTimestampNs;
}
