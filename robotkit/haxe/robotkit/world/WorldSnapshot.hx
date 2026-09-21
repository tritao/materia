package robotkit.world;

import haxe.Int64;

/** Immutable-by-ownership view of all observable world state. */
class WorldSnapshot {
  public final sequence:Int;
  public final topologyRevision:Int;
  public final timestampNs:Int64;
  final robotMap:Map<RobotId, RobotSnapshot>;

  public function new(sequence:Int, topologyRevision:Int, timestampNs:Int64, source:Map<RobotId, RobotSnapshot>) {
    this.sequence = sequence;
    this.topologyRevision = topologyRevision;
    this.timestampNs = timestampNs;
    robotMap = new Map<RobotId, RobotSnapshot>();
    for (id in source.keys()) {
      var value = source.get(id);
      if (value != null) robotMap.set(
        id,
        new RobotSnapshot(
          value.id,
          value.sourceSequence,
          value.timestampNs,
          value.positions,
          value.velocities,
          value.efforts,
          value.mode,
          value.faultCode
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
}
