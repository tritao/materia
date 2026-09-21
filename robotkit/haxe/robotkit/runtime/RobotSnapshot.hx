package robotkit.runtime;

import haxe.Int64;

/**
 * Immutable-by-ownership copy of one robot runtime observation.
 *
 * The arrays belong to this snapshot and are never reused by the runtime.
 * Consumers must treat them as read-only and retain the snapshot instead of
 * reaching back into mutable runtime state.
 */
class RobotSnapshot {
  public final robotId:Int64;
  public final sequence:Int64;
  public final timestampNs:Int64;
  public final mode:Int;
  public final safety:Int;
  public final endpoint:Int;
  public final faultCode:Int;
  public final q:Array<Float>;
  public final dq:Array<Float>;
  public final effort:Array<Float>;

  public function new(robotId:Int64, sequence:Int64, timestampNs:Int64,
      mode:Int, safety:Int, endpoint:Int, faultCode:Int,
      q:Array<Float>, dq:Array<Float>, effort:Array<Float>) {
    this.robotId = robotId;
    this.sequence = sequence;
    this.timestampNs = timestampNs;
    this.mode = mode;
    this.safety = safety;
    this.endpoint = endpoint;
    this.faultCode = faultCode;
    this.q = q == null ? [] : q.copy();
    this.dq = dq == null ? [] : dq.copy();
    this.effort = effort == null ? [] : effort.copy();
  }

  public static function fromRuntime(robotId:Int64, value:RuntimeSnapshot):RobotSnapshot
    return new RobotSnapshot(robotId, Int64.ofInt(value.sequence), value.timestampNs,
      value.mode, value.safety, value.endpoint, value.faultCode,
      value.positions, value.velocities, value.efforts);
}
