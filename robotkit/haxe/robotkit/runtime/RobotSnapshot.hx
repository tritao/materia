package robotkit.runtime;

import RobotKitRuntime;
import haxe.Int64;
import robotkit.world.ImmutableFloatArray;

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
  public final sourceTimestampNs:Int64;
  public final receivedTimestampNs:Int64;
  public final mode:Int;
  public final safety:Int;
  public final endpoint:Int;
  public final faultCode:Int;
  public final q:ImmutableFloatArray;
  public final dq:ImmutableFloatArray;
  public final effort:ImmutableFloatArray;

  /** Compatibility alias; source time is the runtime's primary observation clock. */
  public var timestampNs(get, never):Int64;

  public function new(robotId:Int64, sequence:Int64, sourceTimestampNs:Int64,
      mode:Int, safety:Int, endpoint:Int, faultCode:Int,
      q:Array<Float>, dq:Array<Float>, effort:Array<Float>,
      ?receivedTimestampNs:Int64) {
    this.robotId = robotId;
    this.sequence = sequence;
    this.sourceTimestampNs = sourceTimestampNs;
    this.receivedTimestampNs = receivedTimestampNs == null
      ? sourceTimestampNs
      : receivedTimestampNs;
    this.mode = mode;
    this.safety = safety;
    this.endpoint = endpoint;
    this.faultCode = faultCode;
    this.q = new ImmutableFloatArray(q);
    this.dq = new ImmutableFloatArray(dq);
    this.effort = new ImmutableFloatArray(effort);
  }

  /** Converts the native ABI value while leaving the semantic robot ID unset. */
  @:allow(RobotRuntime)
  static function fromNative(value:rk_robot_snapshot):RobotSnapshot {
    var positions:Array<Float> = [];
    var velocities:Array<Float> = [];
    var efforts:Array<Float> = [];
    var count = value.get_joint_count();
    for (index in 0...count) {
      positions.push(value.get_position(index));
      velocities.push(value.get_velocity(index));
      efforts.push(value.get_effort(index));
    }
    return new RobotSnapshot(Int64.ofInt(0), value.get_sequence(), value.get_source_timestamp_ns(),
      value.get_mode(), value.get_safety(), value.get_endpoint(), value.get_fault_code(),
      positions, velocities, efforts, value.get_received_timestamp_ns());
  }

  /** Returns an immutable copy associated with a caller-provided robot ID. */
  public function withRobotId(value:Int64):RobotSnapshot
    return new RobotSnapshot(value, sequence, sourceTimestampNs, mode, safety, endpoint, faultCode,
      q.toArray(), dq.toArray(), effort.toArray(), receivedTimestampNs);

  inline function get_timestampNs():Int64 return sourceTimestampNs;
}
