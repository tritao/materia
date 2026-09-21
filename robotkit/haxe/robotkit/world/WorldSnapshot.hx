package robotkit.world;

import haxe.Int64;

/** Immutable-by-ownership view of all observable world state. */
class WorldSnapshot {
  public final sequence:Int;
  public final topologyRevision:Int;
  public final timestampNs:Int64;
  /** Consumers must treat this copied map as read-only. */
  public final robots:Map<RobotId, RobotSnapshot>;

  public function new(sequence:Int, topologyRevision:Int, timestampNs:Int64, source:Map<RobotId, RobotSnapshot>) {
    this.sequence = sequence;
    this.topologyRevision = topologyRevision;
    this.timestampNs = timestampNs;
    robots = new Map<RobotId, RobotSnapshot>();
    for (id in source.keys()) {
      var value = source.get(id);
      if (value != null) robots.set(
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

  public function robot(id:RobotId):Null < RobotSnapshot > return robots.get(id);

  public function robotIds():Array < RobotId > {
    var result:Array<RobotId> = [];
    for (id in robots.keys()) result.push(id);
    result.sort(function(left, right) return Reflect.compare(left, right));
    return result;
  }
}
