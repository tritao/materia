package robotkit.runtime;

import RobotKitRuntime;
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
    return new RobotSnapshot(Int64.ofInt(0), value.get_sequence(), value.get_timestamp_ns(),
      value.get_mode(), value.get_safety(), value.get_endpoint(), value.get_fault_code(),
      positions, velocities, efforts);
  }

  /** Returns an immutable copy associated with a caller-provided robot ID. */
  public function withRobotId(value:Int64):RobotSnapshot
    return new RobotSnapshot(value, sequence, timestampNs, mode, safety, endpoint, faultCode,
      q, dq, effort);
}
